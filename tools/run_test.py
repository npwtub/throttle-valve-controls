"""Set the Arduino's Unix time, monitor output, and start only on user command."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import queue
import sys
import threading
import time

import serial
from serial.tools import list_ports


def synchronize(port, record_line, timeout=30):
    """Wait through board reset/setup; send a fresh timestamp once ready."""
    deadline = time.monotonic() + timeout
    sent = False
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        record_line(line)
        if line.startswith("SD_ERROR") or "SD logging failed" in line:
            raise RuntimeError("SD setup failed. Fix the card/wiring before starting.")
        if line == "INVALID_TIME":
            raise RuntimeError("Arduino rejected the timestamp.")
        if line == "READY_FOR_TIME" and not sent:
            port.write(f"{time.time_ns() // 1_000_000}\n".encode("ascii"))
            port.flush()
            sent = True
            deadline = time.monotonic() + 5
        elif line == "TIME_SET" and sent:
            return
    raise TimeoutError("No time handshake. Check port/firmware, or press board reset.")


def read_commands(commands):
    for line in sys.stdin:
        commands.put(line.strip().lower())
    commands.put("quit")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Arduino port, e.g. COM5")
    parser.add_argument("--list", action="store_true", help="List ports and exit")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--output", type=Path, help="New Serial transcript file")
    args = parser.parse_args()
    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(f"{port.device}: {port.description}")
        if not ports:
            print("No serial devices found.")
        return 0
    if not args.port:
        parser.error("--port is required unless using --list")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or Path("outputs") / f"serial_{stamp}.txt"
    output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Saving Serial output to {output}")
    print("Opening the port may reset the board. Close other serial monitors first.")
    try:
        with output.open("x", encoding="utf-8", buffering=1) as transcript:
            def record_line(line):
                print(line, flush=True)
                transcript.write(line + "\n")

            with serial.Serial(args.port, args.baud, timeout=0.2,
                               write_timeout=2) as port:
                synchronize(port, record_line)
                print("Time set. Type start + Enter to actuate, or quit + Enter to exit.")
                print("Quitting disconnects monitoring; it does NOT stop the controller.")
                commands = queue.Queue()
                threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
                started = False
                sd_failed = False
                while True:
                    line = port.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        record_line(line)
                        if line.startswith("SD_ERROR") or "sd_error=1" in line:
                            sd_failed = True
                    try:
                        command = commands.get_nowait()
                    except queue.Empty:
                        continue
                    if command == "quit":
                        return 0
                    if command == "start":
                        if started or sd_failed:
                            print("Start not sent: already started or SD error reported.")
                        else:
                            port.write(b" ")
                            port.flush()
                            started = True
                            record_line("HOST: start command sent")
                    else:
                        print("Commands: start, quit")
    except KeyboardInterrupt:
        print("Monitor disconnected; controller was not stopped.")
        return 0
    except (OSError, serial.SerialException, RuntimeError, TimeoutError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
