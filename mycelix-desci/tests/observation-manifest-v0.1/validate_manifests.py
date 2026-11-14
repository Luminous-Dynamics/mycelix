#!/usr/bin/env python3
"""Development-only schema and byte-integrity validator for manifest fixtures.

This is not a production ingestion or signature-verification service.
Requires: python3 -m pip install jsonschema==4.26.0
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

TEST_DIR = Path(__file__).resolve().parent
DESCI_ROOT = TEST_DIR.parents[1]
SCHEMA_PATH = DESCI_ROOT / "schemas" / "observation-manifest-v0.1.schema.json"
FIXTURES = TEST_DIR / "fixtures"
PAYLOADS = FIXTURES / "payloads"

PAYLOADS_BY_ID = {
    "urn:mycelix:fixture:astro:0001": "astronomy.synthetic.txt",
    "urn:mycelix:fixture:ocean:0001": "ocean.synthetic.txt",
    "urn:mycelix:fixture:eo:0001": "earth-observation.synthetic.txt",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def validate_fixture(validator: Draft202012Validator, path: Path) -> dict[str, Any]:
    manifest = load_json(path)
    errors = sorted(validator.iter_errors(manifest), key=lambda e: list(e.absolute_path))
    require(not errors, f"{path.name}: schema errors: " + "; ".join(e.message for e in errors))
    require(manifest["observation_id"] in PAYLOADS_BY_ID, f"{path.name}: no frozen payload mapping")

    payload_path = PAYLOADS / PAYLOADS_BY_ID[manifest["observation_id"]]
    payload = payload_path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    artifact = manifest["artifact"]
    require(artifact["digest"]["algorithm"] == "sha256", f"{path.name}: fixture digest algorithm must be sha256")
    require(artifact["digest"]["value"] == digest, f"{path.name}: payload digest mismatch")
    require(artifact["byte_length"] == len(payload), f"{path.name}: payload byte length mismatch")
    require(artifact["locator"] == f"urn:sha256:{digest}", f"{path.name}: locator/digest mismatch")
    return manifest


def expect_rejected(validator: Draft202012Validator, label: str, mutant: dict[str, Any]) -> None:
    errors = list(validator.iter_errors(mutant))
    require(bool(errors), f"negative control unexpectedly accepted: {label}")
    print(f"PASS\tnegative\t{label}")


def main() -> int:
    schema = load_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema)

    manifests = []
    for path in sorted(FIXTURES.glob("*.synthetic.json")):
        manifests.append(validate_fixture(validator, path))
        print(f"PASS\tpositive\t{path.name}")
    require(len(manifests) == 3, f"expected exactly 3 positive fixtures; found {len(manifests)}")
    ids = [m["observation_id"] for m in manifests]
    require(len(ids) == len(set(ids)), "duplicate observation_id in positive fixtures")

    base = copy.deepcopy(manifests[0])
    mutant = copy.deepcopy(base)
    mutant["profile"] = "wrong-profile"
    expect_rejected(validator, "wrong-profile", mutant)

    mutant = copy.deepcopy(base)
    mutant["unreviewed_field"] = True
    expect_rejected(validator, "unknown-top-level-key", mutant)

    mutant = copy.deepcopy(base)
    mutant["artifact"]["digest"]["value"] = "not-a-sha256-digest"
    expect_rejected(validator, "malformed-digest", mutant)

    mutant = copy.deepcopy(base)
    mutant["rights"]["permitted_purposes"] = []
    expect_rejected(validator, "missing-purpose", mutant)

    mutant = copy.deepcopy(base)
    del mutant["received_at"]
    expect_rejected(validator, "missing-fabric-receipt-time", mutant)

    print("SUMMARY\tpositive=3\tnegative=5\tschema=draft2020-12\tpayload-digests=verified")
    print("CLAIM-CEILING\tsynthetic schema/byte-contract fixtures only; no source authentication or physical qualification")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"FAIL\t{exc}", file=sys.stderr)
        raise SystemExit(1)
