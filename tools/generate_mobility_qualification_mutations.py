#!/usr/bin/env python3
"""Generate deterministic negative corpus mutations for both mobility evaluators."""
import copy
import json
import sys
from pathlib import Path

EXPECTED_SCENARIO = "same-design-different-artifacts"
EXPECTED_ID = "MC-CONFIG-001"

def write(out, name, corpus):
    path = out / f"{name}.json"
    path.write_text(json.dumps(corpus, indent=2, sort_keys=True) + "\n")

def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: generate_mobility_qualification_mutations.py CORPUS.json OUTPUT_DIR")
    source = json.loads(Path(sys.argv[1]).read_text())
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)

    mutations = {}

    m = copy.deepcopy(source); del m["schema_version"]; mutations["missing-top-level-schema-version"] = m
    m = copy.deepcopy(source); m["unexpected"] = True; mutations["unknown-top-level-field"] = m
    m = copy.deepcopy(source); m["schema_version"] = "mobility-qualification-v0"; mutations["wrong-schema-version"] = m
    m = copy.deepcopy(source); m["status"] = "engineering-certification"; mutations["unsafe-status"] = m
    m = copy.deepcopy(source); del m["vectors"]; mutations["missing-vectors"] = m
    m = copy.deepcopy(source); m["vectors"] = m["vectors"][:-1]; mutations["missing-vector"] = m
    m = copy.deepcopy(source); m["vectors"] = m["vectors"] + [copy.deepcopy(m["vectors"][0])]; mutations["duplicate-vector"] = m
    m = copy.deepcopy(source); m["vectors"][0]["id"] = "MC-CONFIG-999"; mutations["unknown-vector-id"] = m
    m = copy.deepcopy(source); m["vectors"][0]["id"] = 1; mutations["wrong-id-type"] = m
    m = copy.deepcopy(source); m["vectors"][0]["scenario"] = ["not", "a", "scenario"]; mutations["wrong-scenario-type"] = m
    m = copy.deepcopy(source); m["vectors"][0]["expected_outcome"] = None; mutations["wrong-outcome-type"] = m
    m = copy.deepcopy(source); m["vectors"][0]["forbidden_inference"] = {"boundary": "changed"}; mutations["wrong-boundary-type"] = m
    m = copy.deepcopy(source); m["vectors"][0]["unknown"] = "rejected"; mutations["unknown-vector-field"] = m
    m = copy.deepcopy(source); m["vectors"][0]["expected_outcome"] = "unsafe-universal-safety-score"; mutations["altered-expected-outcome"] = m
    m = copy.deepcopy(source); m["vectors"][0]["forbidden_inference"] = "corrupted-boundary"; mutations["altered-forbidden-inference"] = m
    m = copy.deepcopy(source); m["vectors"][0]["scenario"] = "unknown-scenario"; mutations["unknown-scenario"] = m
    m = copy.deepcopy(source); m["vectors"][0]["scenario"] = ""; mutations["empty-scenario"] = m
    m = copy.deepcopy(source); m["vectors"][0]["expected_outcome"] = ""; mutations["empty-expected-outcome"] = m
    m = copy.deepcopy(source); m["vectors"][0]["forbidden_inference"] = ""; mutations["empty-forbidden-inference"] = m
    m = copy.deepcopy(source); m["vectors"] = list(reversed(m["vectors"])); mutations["reordered-vectors"] = m
    m = copy.deepcopy(source); m["vectors"][0], m["vectors"][1] = m["vectors"][1], m["vectors"][0]; m["vectors"][0]["id"] = EXPECTED_ID; mutations["duplicate-id-after-permutation"] = m

    for name, corpus in mutations.items():
        write(out, name, corpus)

    print(f"generated {len(mutations)} deterministic mutations in {out}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
