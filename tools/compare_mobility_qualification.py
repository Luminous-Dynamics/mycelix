#!/usr/bin/env python3
"""Fail-closed structural comparison of normalized mobility evaluator outputs."""
import json
import sys
from pathlib import Path

EXPECTED_KEYS = {"schema", "schema_version", "status", "vectors"}
VECTOR_KEYS = {"id", "scenario", "expected_outcome", "forbidden_inference"}

def load(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict) or set(value) != EXPECTED_KEYS:
        raise ValueError(f"{path}: unexpected normalized top-level shape")
    if value["schema"] != "mobility-qualification-normalized-v1":
        raise ValueError(f"{path}: unsupported normalized schema")
    if value["status"] != "semantic-qualification-only":
        raise ValueError(f"{path}: unsafe qualification status")
    vectors = value["vectors"]
    if not isinstance(vectors, list) or len(vectors) != 20:
        raise ValueError(f"{path}: expected exactly 20 vectors")
    ids = []
    for item in vectors:
        if not isinstance(item, dict) or set(item) != VECTOR_KEYS:
            raise ValueError(f"{path}: malformed vector")
        ids.append(item["id"])
    if len(set(ids)) != 20 or ids != sorted(ids):
        raise ValueError(f"{path}: duplicate or noncanonical vector IDs")
    return value

def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: compare_mobility_qualification.py RUST.json PYTHON.json")
    rust, reference = load(sys.argv[1]), load(sys.argv[2])
    if rust != reference:
        print(json.dumps({"result":"mismatch","rust_vectors":len(rust["vectors"]),
                          "reference_vectors":len(reference["vectors"])}, sort_keys=True))
        return 1
    print(json.dumps({"result":"matched","vectors":20,
                      "scope":"semantic-structural-only"}, sort_keys=True))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
