#!/usr/bin/env python3
"""Extended qualifier for strict + independent temporal-order inputs."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
INNER = SCRIPT.parent / "qualify_temporal_provenance.py"
SPEC = REPO / "mycelix-governance" / "specs" / "ConstitutionalEvidenceClosure.tla"
INDEPENDENT_CFG = REPO / "mycelix-governance" / "specs" / "ConstitutionalEvidenceClosure.independent.cfg"
INDEPENDENT_REACH_CFG = REPO / "mycelix-governance" / "specs" / "ConstitutionalEvidenceClosure.reach-independent-incomparable.cfg"
EXTRA_INPUTS = [
    REPO / "mycelix-governance" / "crates" / "constitutional-temporal-provenance" / "tests" / "strict_observation.rs",
    REPO / "mycelix-governance" / "crates" / "constitutional-temporal-provenance" / "tests" / "independent_order.rs",
    REPO / "mycelix-governance" / "docs" / "CONSTITUTIONAL_TEMPORAL_PROVENANCE_ABSTRACTION_NOTES.md",
    INDEPENDENT_CFG,
    INDEPENDENT_REACH_CFG,
]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def snapshot() -> dict[str, str]:
    out: dict[str, str] = {}
    for path in [INNER, SCRIPT, SPEC, *EXTRA_INPUTS]:
        if not path.is_file():
            raise RuntimeError(f"required extended qualification input missing: {path}")
        out[str(path.resolve().relative_to(REPO.resolve()))] = sha256_file(path)
    return out


def parse_stats(output: str) -> dict[str, int | None]:
    def get(pattern: str) -> int | None:
        m = re.search(pattern, output)
        return int(m.group(1).replace(",", "")) if m else None
    return {
        "states_generated": get(r"([0-9,]+) states generated"),
        "distinct_states": get(r"([0-9,]+) distinct states found"),
        "depth": get(r"depth of the complete state graph search is ([0-9,]+)"),
    }


def violations(output: str) -> list[str]:
    return sorted(set(re.findall(r"Invariant ([A-Za-z_][A-Za-z0-9_]*) is violated", output)))


def run_tlc(label: str, java: str, jar: Path, cfg_path: Path, outdir: Path, expected: str | None) -> dict:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-order-{label}-") as td:
        work = Path(td)
        spec = work / SPEC.name
        cfg = work / "ConstitutionalEvidenceClosure.cfg"
        spec.write_text(SPEC.read_text(encoding="utf-8"), encoding="utf-8")
        cfg.write_text(cfg_path.read_text(encoding="utf-8"), encoding="utf-8")
        cmd = [
            java, "-cp", str(jar.resolve()), "tlc2.TLC",
            "-workers", "1", "-deadlock", "-config", cfg.name, spec.name,
        ]
        proc = subprocess.run(
            cmd, cwd=work, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace", check=False,
        )

    log_path = outdir / "tlc" / f"{label}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(proc.stdout, encoding="utf-8")
    stats = parse_stats(proc.stdout)
    found = violations(proc.stdout)
    explored = (stats["states_generated"] or 0) > 0 and (stats["distinct_states"] or 0) > 0
    if expected is None:
        passed = proc.returncode == 0 and explored and not found and "No error has been found" in proc.stdout
        expectation = "SAFETY_PASS"
    else:
        passed = explored and found == [expected]
        expectation = f"EXACT_VIOLATION:{expected}"
    return {
        "label": label,
        "command": cmd,
        "returncode": proc.returncode,
        "expectation": expectation,
        "violations": found,
        "stats": stats,
        "log_sha256": sha256_file(log_path),
        "passed": passed,
    }


def main() -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--java", default="java")
    args, _ = parser.parse_known_args()
    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    pre = snapshot()
    inner = subprocess.run(
        [sys.executable, str(INNER), *sys.argv[1:]],
        cwd=str(REPO),
        check=False,
    )

    independent_safety = run_tlc(
        "safety-independent",
        args.java,
        args.tla_jar,
        INDEPENDENT_CFG,
        outdir,
        None,
    )
    independent_reach = run_tlc(
        "reach-independent-incomparable",
        args.java,
        args.tla_jar,
        INDEPENDENT_REACH_CFG,
        outdir,
        "NeverIndependentIncomparableAccepted",
    )

    post = snapshot()
    immutable = pre == post
    receipt = {
        "schema": "mycelix.temporal-provenance-extended-inputs.v2",
        "preflight_sha256": pre,
        "postflight_sha256": post,
        "postflight_immutable": immutable,
        "inner_returncode": inner.returncode,
        "strict_observation_test_included": any(key.endswith("tests/strict_observation.rs") for key in pre),
        "independent_order_test_included": any(key.endswith("tests/independent_order.rs") for key in pre),
        "abstraction_note_included": any(key.endswith("CONSTITUTIONAL_TEMPORAL_PROVENANCE_ABSTRACTION_NOTES.md") for key in pre),
        "independent_safety": independent_safety,
        "independent_reachability": independent_reach,
        "passed": inner.returncode == 0 and independent_safety["passed"] and independent_reach["passed"] and immutable,
    }
    (outdir / "extra-input-receipt.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
