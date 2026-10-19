"""Host checks: no Arduino, SD card, or serial connection required."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('console', ROOT / 'valve_log_console.py')
console = importlib.util.module_from_spec(spec)
spec.loader.exec_module(console)

class Commands(unittest.TestCase):
    def test_time_message_does_not_start_valve(self):
        session = Mock()
        console.synchronize(session, 1792152000123)
        session.send.assert_called_once_with(b'TIME,1792152000123\n')
        self.assertEqual(session.wait_for.call_args_list[0].args[0], '# BOOT_WAITING_FOR_TIME')
        self.assertEqual(session.wait_for.call_args_list[1].args[0], '# TIME_SYNC_OK')

    def test_invalid_timestamp_sends_nothing(self):
        session = Mock()
        with self.assertRaises(ValueError): console.synchronize(session, 123)
        session.send.assert_not_called()

    def test_start_is_explicit_single_space(self):
        session = Mock()
        console.start_test(session)
        session.send.assert_called_once_with(b' ')
        session.wait_for.assert_called_once_with('# TEST_STARTED', timeout=120)

    def test_stop_is_logging_only_and_waits_for_closure(self):
        session = Mock()
        console.stop_logging(session)
        session.send.assert_called_once_with(b'!')
        session.wait_for.assert_called_once_with('# LOG_CLOSED', timeout=30)
        session.wait_for_report.assert_called_once_with(status='3')

    def test_short_write_reports_unknown_outcome(self):
        port = Mock()
        port.write.return_value = 0
        with self.assertRaisesRegex(RuntimeError, 'unknown'):
            console.Session(port).send(b'!')

    def test_storage_error_is_not_success(self):
        session = console.Session(Mock())
        session.messages.put('# SD_WRITE_FAILED')
        with self.assertRaisesRegex(RuntimeError, 'SD_WRITE_FAILED'):
            session.wait_for('# LOG_CLOSED', timeout=1)

    def read_chunks(self, chunks):
        port = Mock()
        transcript = io.StringIO()
        session = console.Session(port, transcript)
        chunks = iter(chunks)
        def read(_):
            try: return next(chunks)
            except StopIteration:
                session.stopping.set()
                return b''
        port.read.side_effect = read
        with contextlib.redirect_stdout(io.StringIO()): session._read()
        return session, transcript.getvalue()

    def test_fragmented_responses_and_transcript(self):
        session, text = self.read_chunks([b'# BOOT_WAIT', b'ING_FOR_TIME\n# TIME_', b'SYNC_OK\n'])
        self.assertIsNone(session.failure)
        self.assertEqual(session.messages.get_nowait(), '# BOOT_WAITING_FOR_TIME')
        self.assertEqual(session.messages.get_nowait(), '# TIME_SYNC_OK')
        self.assertIn('# TIME_SYNC_OK', text)

    def test_repeated_status_does_not_overflow_response_queue(self):
        chunks = [b'# STATS_BEGIN\n# missed_log_slots,2\n# STATS_END\n'] * 200
        session, _ = self.read_chunks(chunks + [b'# LOG_CLOSED\n'])
        self.assertIsNone(session.failure)
        self.assertEqual(session.messages.qsize(), 1)
        self.assertEqual(session.messages.get_nowait(), '# LOG_CLOSED')

    def test_unexpected_reboot_is_detected(self):
        session, _ = self.read_chunks([b'# BOOT_WAITING_FOR_TIME\n'] * 2)
        self.assertIn('reset unexpectedly', session.failure)

    def test_closed_report_includes_final_timing(self):
        session, _ = self.read_chunks([
            b'# STATS_BEGIN\n# log_status,3\n# close_us,',
            b'4200\n# FILE,RUN0002.CSV\n# STATS_END\n'
        ])
        report = session.wait_for_report(status='3', timeout=0)
        self.assertEqual(report['close_us'], '4200')
        self.assertEqual(report['FILE'], 'RUN0002.CSV')

    def test_running_report_is_not_a_closed_report(self):
        session, _ = self.read_chunks([b'# STATS_BEGIN\n# log_status,1\n# STATS_END\n'])
        with self.assertRaisesRegex(RuntimeError, 'Diagnostics'):
            session.wait_for_report(status='3', timeout=0)

    def test_incomplete_report_does_not_count_as_received(self):
        session, _ = self.read_chunks([b'# STATS_BEGIN\n# log_status,3\n'])
        self.assertEqual(session.report_sequence, 0)

    def run_console(self, commands):
        serial = types.ModuleType('serial')
        serial.SerialException = OSError
        port = Mock()
        context = Mock()
        context.__enter__ = Mock(return_value=port)
        context.__exit__ = Mock(return_value=False)
        serial.Serial = Mock(return_value=context)
        serial_tools = types.ModuleType('serial.tools')
        serial_tools.list_ports = Mock()
        session = Mock()
        with tempfile.TemporaryDirectory() as folder, \
             patch.dict(sys.modules, {'serial': serial, 'serial.tools': serial_tools}), \
             patch.object(console, 'Session', return_value=session), \
             patch.object(sys, 'argv', ['console', '--port', 'FAKE', '--transcript', str(Path(folder)/'status.txt')]), \
             patch('builtins.input', side_effect=commands), \
             contextlib.redirect_stdout(io.StringIO()):
            result = console.main()
        return result, session, serial.Serial

    def test_connect_and_status_never_start_valve(self):
        result, session, _ = self.run_console(['CONNECT', 'STATUS', 'QUIT'])
        self.assertEqual(result, 0)
        sent = [call.args[0] for call in session.send.call_args_list]
        self.assertTrue(sent[0].startswith(b'TIME,'))
        self.assertEqual(sent[1:], [b'?'])

    def test_duplicate_start_is_not_transmitted(self):
        result, session, _ = self.run_console(['CONNECT', 'START', 'START', 'STOP', 'STATUS', 'QUIT'])
        self.assertEqual(result, 0)
        sent = [call.args[0] for call in session.send.call_args_list]
        self.assertEqual(sent[1:], [b' ', b'!', b'?'])

    def test_cancel_never_opens_port(self):
        result, session, serial_open = self.run_console(['cancel'])
        self.assertEqual(result, 0)
        serial_open.assert_not_called()
        session.send.assert_not_called()

if __name__ == '__main__': unittest.main()
