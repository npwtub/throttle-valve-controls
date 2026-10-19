"""UTC synchronization and explicit operator commands for the dwell-test prototype.

Run --help without hardware. Opening/closing a serial connection may reset the
Arduino. Use only before a test, with the team's hardware-test procedure.
"""

import argparse
import queue
import threading
import time


class Session:
    def __init__(self, port, transcript=None):
        self.port = port
        self.transcript = transcript
        self.messages = queue.Queue(maxsize=128)
        self.stopping = threading.Event()
        self.failure = None
        self.boot_count = 0
        self.report_condition = threading.Condition()
        self.report_sequence = 0
        self.latest_report = {}
        self.thread = threading.Thread(target=self._read, daemon=True)

    def _read(self):
        pending = bytearray()
        report = None
        try:
            while not self.stopping.is_set():
                chunk = self.port.read(64)
                if not chunk:
                    continue
                pending.extend(chunk)
                while b"\n" in pending:
                    raw, _, remaining = pending.partition(b"\n")
                    pending = bytearray(remaining)
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    print(f"\nArduino: {line}", flush=True)
                    if self.transcript is not None:
                        self.transcript.write(f"{time.time_ns() // 1_000_000} {line}\n")
                    if line == "# BOOT_WAITING_FOR_TIME":
                        self.boot_count += 1
                        if self.boot_count > 1:
                            raise RuntimeError("Arduino reset unexpectedly; do not restart the test blindly.")
                    if line == "# STATS_BEGIN":
                        report = {}
                        continue
                    if line == "# STATS_END":
                        if report is not None:
                            with self.report_condition:
                                self.latest_report = report
                                self.report_sequence += 1
                                self.report_condition.notify_all()
                            report = None
                        continue
                    if line.startswith("# ") and "," in line:
                        if report is not None:
                            key, value = line[2:].split(",", 1)
                            report[key] = value
                        continue
                    self.messages.put_nowait(line)
                if len(pending) > 4096:
                    raise RuntimeError("Received an oversized serial line; check the baud rate/firmware.")
        except Exception as exc:
            if not self.stopping.is_set():
                self.failure = str(exc) or type(exc).__name__
                print(f"\nSerial error: {self.failure}", flush=True)

    def check(self):
        if self.failure:
            raise RuntimeError(self.failure)

    def send(self, data):
        self.check()
        if self.port.write(data) != len(data):
            raise RuntimeError("Incomplete command transmission; command outcome is unknown.")

    def wait_for_report(self, after=0, status=None, timeout=30):
        deadline = time.monotonic() + timeout
        with self.report_condition:
            while True:
                self.check()
                if self.report_sequence > after and (
                    status is None or self.latest_report.get("log_status") == status
                ):
                    return dict(self.latest_report)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Diagnostics not received; logging outcome must be checked separately.")
                self.report_condition.wait(min(remaining, 0.1))

    def wait_for(self, expected, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check()
            try:
                line = self.messages.get(timeout=0.1)
            except queue.Empty:
                continue
            if line == expected:
                return
            if line in {
                "# TIME_SYNC_INVALID", "# TIME_ALREADY_SET",
                "# START_BLOCKED_NO_TIME", "# START_BLOCKED_SD_PIN",
                "# SD_INIT_FAILED_RESET_REQUIRED", "# SD_WRITE_FAILED",
            }:
                raise RuntimeError(line)
        raise RuntimeError(
            f"No {expected!r} response within {timeout} seconds. "
            "Do not assume a command failed or repeat a start command."
        )


def synchronize(session, epoch_ms=None):
    # Wait for fresh firmware startup, rather than sending time during boot.
    session.wait_for("# BOOT_WAITING_FOR_TIME", timeout=15)
    if epoch_ms is None:
        epoch_ms = time.time_ns() // 1_000_000
    digits = str(epoch_ms)
    if len(digits) != 13 or not digits.isascii() or not digits.isdigit():
        raise ValueError("Firmware requires a 13-digit UTC epoch-millisecond value.")
    session.send(f"TIME,{digits}\n".encode("ascii"))
    session.wait_for("# TIME_SYNC_OK")


def start_test(session):
    session.send(b" ")
    session.wait_for("# TEST_STARTED", timeout=120)


def stop_logging(session):
    session.send(b"!")
    # LOG_STOP_REQUESTED is printed by the reader. Only closure is completion.
    session.wait_for("# LOG_CLOSED", timeout=30)
    # Wait for the complete closed-state snapshot, including finalization timing.
    session.wait_for_report(status="3")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Arduino serial port, for example COM5")
    parser.add_argument("--list-ports", action="store_true", help="List ports without opening them")
    parser.add_argument("--transcript", help="New text file for received status/diagnostics (default: unique local filename)")
    args = parser.parse_args()

    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.exit(1, "Install pySerial first: python -m pip install pyserial\n")

    if args.list_ports:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found. This is expected if no board is connected.")
        for port in ports:
            print(f"{port.device}: {port.description}")
        return 0

    if not args.port:
        parser.error("provide --port COMx or use --list-ports")

    print("Opening this port may reset the Arduino. Use before a test, not during one.")
    print("Ensure the computer clock is correct and synchronized.")
    if input("Type CONNECT to open the port: ").strip() != "CONNECT":
        print("Cancelled; no connection opened.")
        return 0

    session = None
    test_may_be_running = False
    try:
        transcript_path = args.transcript or f"valve_console_{time.time_ns()}.txt"
        with open(transcript_path, "x", encoding="utf-8", buffering=1) as transcript, \
                serial.Serial(args.port, 57600, timeout=0.1, write_timeout=2) as port:
            print(f"Received status and diagnostics saved to {transcript_path}")
            session = Session(port, transcript)
            session.thread.start()
            try:
                synchronize(session)
                print("UTC accepted. No valve-start command has been sent.")
                print("START: issue valve-test start command (moves valve).")
                print("STOP: stop logging only; valve test continues.")
                print("STATUS: show logging loss, filename, and timing measurements.")
                print("QUIT: exit before starting or after confirmed log closure.")
                print("EXIT: disconnect even if log closure is unconfirmed.")
                phase = "ready"
                while True:
                    command = input("Command: ").strip().upper()
                    session.check()
                    if command == "START" and phase == "ready":
                        phase = "unknown"
                        test_may_be_running = True
                        try:
                            start_test(session)
                            phase = "running"
                        except RuntimeError as exc:
                            print(f"START not confirmed: {exc}")
                            print("No automatic retry. Inspect the setup with the test operator.")
                    elif command == "STOP" and phase in {"running", "unknown"}:
                        try:
                            stop_logging(session)
                            phase = "closed"
                            print("Log closed. Valve control has NOT been stopped.")
                        except RuntimeError as exc:
                            phase = "unknown"
                            print(f"Log closure NOT confirmed: {exc}")
                    elif command == "STATUS":
                        previous_report = session.report_sequence
                        session.send(b"?")
                        try:
                            session.wait_for_report(after=previous_report)
                        except RuntimeError as exc:
                            print(f"STATUS not confirmed: {exc}")
                    elif command == "QUIT":
                        if phase not in {"ready", "closed"}:
                            print("Closure is unconfirmed. Use STOP or explicitly choose EXIT.")
                        else:
                            break
                    elif command == "EXIT":
                        print("Disconnecting. Any unconfirmed log may be incomplete.")
                        break
                    else:
                        print("Command unavailable. Use START, STOP, STATUS, QUIT, or EXIT as described above.")
            finally:
                session.stopping.set()
                session.thread.join(timeout=1)
    except (OSError, serial.SerialException, RuntimeError, ValueError, EOFError, KeyboardInterrupt) as exc:
        print(f"Console stopped: {exc or type(exc).__name__}")
        print("Log closure is not guaranteed. No valve-close command was sent.")
        return 1
    finally:
        if test_may_be_running:
            print("Valve behavior remains controlled by its firmware; this console did not command it closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
