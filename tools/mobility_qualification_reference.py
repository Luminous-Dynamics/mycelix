#!/usr/bin/env python3
"""Independent reference evaluator for Mobility Configuration Contract V1."""
import json
import sys
from pathlib import Path

EXPECTED = {
"same-design-different-artifacts":"distinct-physical-artifacts",
"same-cad-different-process":"manufacturing-lineage-preserved",
"component-substitution":"substitution-lineage-and-revalidation",
"failed-test-followed-by-later-test":"historical-evidence-preserved",
"simulation-equals-measurement":"prediction-observation-remain-distinct",
"negative-inspection-finding":"negative-evidence-preserved",
"undeclared-dependency":"unknown-or-review-required",
"private-payload-public-provenance":"controlled-payload-with-public-provenance",
"foreign-engineering-identifier":"explicit-binding-or-rejection",
"holochain-hash-as-engineering-id":"reject-semantic-substitution",
"observation-versus-diagnosis":"observation-separate-from-interpretation",
"repair-lineage":"repair-history-and-resulting-state-preserved",
"external-authority-disposition":"external-authority-remains-attributable",
"assurance-criticality-metadata":"metadata-remains-descriptive",
"multimodal-interface":"interface-dependencies-explicit",
"configuration-supersession":"historical-predecessor-preserved",
"open-revalidation-obligation":"obligation-remains-open",
"ground-and-marine-instances":"shared-core-profile-specific-divergence",
"software-firmware-change":"dependency-aware-change-lineage",
"retired-physical-artifact":"historical-artifact-not-active",
}
FORBIDDEN = {
"same-design-different-artifacts":"artifact-identity-collapse",
"same-cad-different-process":"cad-equality-proves-physical-equivalence",
"component-substitution":"compatibility-claim-proves-equivalence",
"failed-test-followed-by-later-test":"later-pass-erases-failure",
"simulation-equals-measurement":"numerical-equality-converts-prediction-to-observation",
"negative-inspection-finding":"only-success-is-authoritative",
"undeclared-dependency":"missing-dependency-means-unaffected",
"private-payload-public-provenance":"public-commitment-proves-private-access",
"foreign-engineering-identifier":"foreign-id-silently-becomes-native",
"holochain-hash-as-engineering-id":"protocol-hash-is-engineering-identity",
"observation-versus-diagnosis":"observation-proves-root-cause",
"repair-lineage":"repair-erases-history",
"external-authority-disposition":"graph-consensus-generates-authority",
"assurance-criticality-metadata":"criticality-label-proves-safety",
"multimodal-interface":"shared-connector-proves-interoperability",
"configuration-supersession":"supersession-deletes-history",
"open-revalidation-obligation":"obligation-means-test-passed",
"ground-and-marine-instances":"domain-neutrality-forces-identical-engineering",
"software-firmware-change":"unchanged-cad-means-unchanged-configuration",
"retired-physical-artifact":"historical-existence-means-current-validity",
}

def evaluate(corpus):
    if corpus.get("schema_version") != "mobility-configuration-contract-qualification-v1":
        return False, "schema_version"
    if corpus.get("status") != "semantic-qualification-only":
        return False, "status"
    vectors = corpus.get("vectors")
    if not isinstance(vectors, list) or len(vectors) != len(EXPECTED):
        return False, "vector_count"
    seen = set()
    for vector in vectors:
        scenario = vector.get("scenario")
        if scenario in seen:
            return False, f"duplicate:{scenario}"
        seen.add(scenario)
        if scenario not in EXPECTED:
            return False, f"unknown_scenario:{scenario}"
        if vector.get("expected_outcome") != EXPECTED[scenario]:
            return False, f"outcome:{scenario}"
        if vector.get("forbidden_inference") != FORBIDDEN[scenario]:
            return False, f"boundary:{scenario}"
    if seen != set(EXPECTED):
        return False, "scenario_set"
    return True, "valid"

def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1.json")
    corpus = json.loads(path.read_text())
    ok, reason = evaluate(corpus)
    corrupted = json.loads(json.dumps(corpus))
    corrupted["vectors"][0]["expected_outcome"] = "unsafe-universal-safety-score"
    corrupted_ok, _ = evaluate(corrupted)
    result = {"implementation":"python-reference-v1","result":"valid" if ok else "invalid",
              "reason":reason,"corruption_probe":"rejected" if not corrupted_ok else "ACCEPTED_UNEXPECTEDLY"}
    print(json.dumps(result, sort_keys=True))
    return 0 if ok and not corrupted_ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
