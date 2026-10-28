#!/usr/bin/env python3
"""Parse pgbench per-transaction logs and stdout into a fail-closed receipt.

PostgreSQL pgbench logs transaction duration in microseconds. This helper
converts that duration to milliseconds, checks log/stdout accounting, and writes
the successful-transaction count consumed by final database reconciliation.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys


FAILURE_TIMES = {"failed", "serialization", "deadlock", "skipped"}


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    # Nearest-rank percentile, p in [0,1].
    return round(ordered[max(0, math.ceil(p * len(ordered)) - 1)], 4)


def parse_pgbench_stdout(stdout: str) -> dict:
    processed = re.search(
        r"number of transactions actually processed:\s*([0-9]+)\s*/\s*([0-9]+)",
        stdout,
    )
    failed = re.search(r"number of failed transactions:\s*([0-9]+)", stdout)
    tps = re.search(r"\btps = ([0-9.]+)", stdout)
    average = re.search(r"latency average = ([0-9.]+) ms", stdout)
    if not processed:
        raise ValueError("pgbench stdout lacks transaction actual/target count")
    if not failed:
        raise ValueError("pgbench stdout lacks failed-transaction count")
    data = {
        "pgbench_processed_transactions": int(processed.group(1)),
        "pgbench_scheduled_transactions": int(processed.group(2)),
        "pgbench_failed_transactions": int(failed.group(1)),
        "pgbench_reported_tps": float(tps.group(1)) if tps else None,
        "pgbench_reported_average_latency_ms": float(average.group(1)) if average else None,
    }
    if data["pgbench_failed_transactions"] != 0:
        raise ValueError(f"pgbench reports {data['pgbench_failed_transactions']} failed transactions")
    return data


def summarize(out_dir: Path) -> dict:
    stdout_path = out_dir / "pgbench.stdout"
    if not stdout_path.is_file():
        raise ValueError(f"missing pgbench stdout: {stdout_path}")
    stdout = stdout_path.read_text(encoding="utf-8")
    stdout_data = parse_pgbench_stdout(stdout)

    files = sorted(p for p in out_dir.glob("pgbench.*") if p.name != "pgbench.stdout" and p.is_file())
    if not files:
        raise ValueError("no pgbench per-transaction log files found")

    samples_ms: list[float] = []
    bad_rows: list[dict] = []
    for path in files:
        for line_no, row in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            if not row.strip():
                continue
            parts = row.split()
            if len(parts) < 6:
                bad_rows.append({"file": path.name, "line": line_no, "row": row, "reason": "too_few_fields"})
                continue
            raw_time = parts[2]
            if raw_time in FAILURE_TIMES:
                bad_rows.append({"file": path.name, "line": line_no, "row": row, "reason": raw_time})
                continue
            try:
                # PostgreSQL 18 pgbench per-transaction log field time is microseconds.
                samples_ms.append(float(raw_time) / 1000.0)
            except ValueError:
                bad_rows.append({"file": path.name, "line": line_no, "row": row, "reason": "invalid_latency"})

    if bad_rows:
        raise ValueError("pgbench logs contain failed or unparseable transactions: " + json.dumps(bad_rows[:10]))
    if not samples_ms:
        raise ValueError("no successful pgbench transaction latency samples found")
    if len(samples_ms) != stdout_data["pgbench_processed_transactions"]:
        raise ValueError(
            "transaction accounting mismatch: "
            f"log_successes={len(samples_ms)} stdout_processed={stdout_data['pgbench_processed_transactions']}"
        )

    result = {
        "pgbench_transaction_log_files": len(files),
        "logged_successful_transactions": len(samples_ms),
        "logged_failed_or_unparseable_transactions": len(bad_rows),
        "latency_ms": {
            "p50": percentile(samples_ms, 0.50),
            "p95": percentile(samples_ms, 0.95),
            "p99": percentile(samples_ms, 0.99),
            "max": round(max(samples_ms), 4),
        },
        **stdout_data,
        "claim_ceiling": "PostgreSQL shared-coordinator function benchmark only; not Holochain DHT or distributed/offline settlement throughput.",
    }
    (out_dir / "latency-summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (out_dir / "successful-count.txt").write_text(str(len(samples_ms)) + "\n", encoding="utf-8")
    return result


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} OUT_DIR", file=sys.stderr)
        return 2
    try:
        result = summarize(Path(sys.argv[1]))
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"pgbench receipt error: {exc}", file=sys.stderr)
        return 3
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
