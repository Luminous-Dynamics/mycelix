from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("summarize_pgbench.py")
SPEC = importlib.util.spec_from_file_location("summarize_pgbench", MODULE_PATH)
assert SPEC and SPEC.loader
summarizer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summarizer)


STDOUT = """number of transactions actually processed: 3/3
number of failed transactions: 0 (0.000%)
latency average = 2.000 ms
tps = 42.5 (without initial connection time)
"""

TIMED_STDOUT = """number of transactions actually processed: 3
number of failed transactions: 0 (0.000%)
latency average = 2.000 ms
tps = 42.5 (without initial connection time)
"""


def make_dir(lines: str, stdout: str = STDOUT):
    owner = tempfile.TemporaryDirectory(prefix="pgbench-parser-test-")
    path = Path(owner.name)
    (path / "pgbench.stdout").write_text(stdout, encoding="utf-8")
    (path / "pgbench.123").write_text(lines, encoding="utf-8")
    return owner, path


class PgbenchReceiptTests(unittest.TestCase):
    def test_microseconds_are_converted_to_milliseconds_and_count_is_written(self):
        owner, path = make_dir("1 1 1000 1 1760000000 100\n2 1 2000 1 1760000000 200\n3 1 3000 1 1760000000 300\n")
        try:
            result = summarizer.summarize(path)
            self.assertEqual(result["logged_successful_transactions"], 3)
            self.assertEqual(result["latency_ms"], {"p50": 2.0, "p95": 3.0, "p99": 3.0, "max": 3.0})
            self.assertEqual((path / "successful-count.txt").read_text(), "3\n")
        finally:
            owner.cleanup()

    def test_tps_regex_parses_real_pgbench_line(self):
        parsed = summarizer.parse_pgbench_stdout(STDOUT)
        self.assertEqual(parsed["pgbench_reported_tps"], 42.5)
        self.assertEqual(parsed["pgbench_processed_transactions"], 3)

    def test_timed_mode_single_count_output_is_accepted(self):
        parsed = summarizer.parse_pgbench_stdout(TIMED_STDOUT)
        self.assertEqual(parsed["pgbench_processed_transactions"], 3)
        self.assertIsNone(parsed["pgbench_scheduled_transactions"])

    def test_failed_transaction_is_fail_closed(self):
        stdout = STDOUT.replace("3/3", "1/2").replace("0 (0.000%)", "1 (50.000%)")
        owner, path = make_dir("1 1 1000 1 1760000000 100\n2 1 failed 1 1760000000 200\n", stdout=stdout)
        try:
            with self.assertRaises(ValueError):
                summarizer.summarize(path)
        finally:
            owner.cleanup()

    def test_log_and_stdout_count_mismatch_is_rejected(self):
        owner, path = make_dir("1 1 1000 1 1760000000 100\n2 1 2000 1 1760000000 200\n")
        try:
            with self.assertRaisesRegex(ValueError, "accounting mismatch"):
                summarizer.summarize(path)
        finally:
            owner.cleanup()

    def test_malformed_log_row_is_rejected(self):
        owner, path = make_dir("1 1 nope\n2 1 2000 1 1760000000 200\n3 1 3000 1 1760000000 300\n")
        try:
            with self.assertRaisesRegex(ValueError, "unparseable"):
                summarizer.summarize(path)
        finally:
            owner.cleanup()

    def test_missing_log_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix="pgbench-no-log-") as td:
            Path(td, "pgbench.stdout").write_text(STDOUT, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no pgbench per-transaction"):
                summarizer.summarize(Path(td))


if __name__ == "__main__":
    unittest.main()
