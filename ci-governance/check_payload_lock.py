#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LOCK = ROOT / "CI_GOV_001E_A.lock.json"
IGNORED_PARTS = {"__pycache__"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_manifest(files: dict[str, str], domain: str) -> bytes:
    entries = [{"path": path, "sha256": files[path]} for path in sorted(files)]
    encoded = json.dumps(entries, sort_keys=True, separators=(",", ":")).encode() + b"\n"
    return domain.encode() + b"\n" + encoded


def main() -> int:
    lock = json.loads(LOCK.read_text())
    assert lock["schema"] == "mycelix.ci-gov.001e-a.lock.v0.1"
    assert lock["parent_commit"] == "a85369699099d4c7524e502e531735eed4ab36f4"
    expected = lock["files"]

    observed_paths = []
    for path in ROOT.rglob("*"):
        if not path.is_file() or path == LOCK:
            continue
        rel = path.relative_to(ROOT)
        if any(part in IGNORED_PARTS for part in rel.parts):
            continue
        observed_paths.append("ci-governance/" + rel.as_posix())

    assert sorted(observed_paths) == sorted(expected), (
        f"payload file-set mismatch: observed={sorted(observed_paths)!r} expected={sorted(expected)!r}"
    )

    observed = {}
    for rel in sorted(expected):
        local = ROOT / rel.removeprefix("ci-governance/")
        observed[rel] = sha256(local)
    assert observed == expected, f"payload file digest mismatch: {observed!r}"

    digest = hashlib.sha256(canonical_manifest(observed, lock["payload_domain"])).hexdigest()
    assert digest == lock["payload_manifest_sha256"], digest
    print(f"ci_gov_001e_a_payload_manifest_sha256={digest}")
    print(f"ci_gov_001e_a_payload_file_count={len(observed)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
