#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path

ROOT = Path("fixtures/epi/concealed_information/v0.1")
FILES = {
    "manifest": ROOT / "manifest.json",
    "visible": ROOT / "visible_cases.json",
    "expected": ROOT / "expected.json",
    "hidden": ROOT / "hidden_evaluator.json",
}


def load(name):
    return json.loads(FILES[name].read_text(encoding="utf-8"))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(cond, msg):
    if not cond:
        raise SystemExit(f"FAIL: {msg}")


manifest = load("manifest")
visible_doc = load("visible")
expected_doc = load("expected")
hidden_doc = load("hidden")

require(manifest["profile_id"] == "CONCEAL-001T-v0.1", "profile id")
require(manifest["authority"] == "ConcealedInformationSemanticsTestOnly", "authority ceiling")
require(manifest["synthetic_only"] is True, "synthetic_only")
require(manifest["case_count"] == 15, "manifest case count")
require(manifest["metamorphic_relation_count"] == 4, "manifest relation count")
require(manifest["solver_visible_files"] == ["visible_cases.json"], "solver-visible file set")
require("hidden_evaluator.json" in manifest["forbidden_solver_inputs"], "hidden evaluator solver firewall")
require(hidden_doc.get("evaluator_only") is True, "hidden evaluator marked evaluator_only")
require(hidden_doc.get("forbidden_as_solver_input") is True, "hidden evaluator forbidden as solver input")

visible = visible_doc["cases"]
expected = expected_doc["expected"]
hidden = hidden_doc["hidden_truth"]
relations = expected_doc["metamorphic_relations"]

require(len(visible) == 15, "visible count")
require(len(expected) == 15, "expected count")
require(len(hidden) == 15, "hidden count")
require(len(relations) == 4, "relation count")

ids = [c["case_id"] for c in visible]
require(len(ids) == len(set(ids)), "visible case ids unique")
expected_ids = {c["case_id"] for c in expected}
hidden_ids = {c["case_id"] for c in hidden}
require(set(ids) == expected_ids == hidden_ids, "cross-file case sets identical")

V = {c["case_id"]: c for c in visible}
E = {c["case_id"]: c for c in expected}
H = {c["case_id"]: c for c in hidden}

for case in expected:
    require(case.get("maliciousness") == "NotEstablished", f"maliciousness ceiling {case['case_id']}")

forbidden = set(expected_doc["forbidden_promotions"])
require(forbidden == {
    "MaliciousPayloadEstablished",
    "AttackerIdentified",
    "IntentEstablished",
    "ExfiltrationEstablished",
    "CompromiseEstablished",
    "ExecutionAuthorized",
    "RemediationAuthorized",
}, "forbidden promotion registry")
for case in expected:
    require(case["disposition"] not in forbidden, f"forbidden disposition {case['case_id']}")

# False-positive controls.
require(E["CI-001"]["disposition"] == "NoConcealedContentEstablished", "high entropy control")
require(E["CI-002"]["disposition"] == "NoConcealedContentEstablished", "benign metadata control")
require(E["CI-004"]["disposition"] == "EmbeddedObjectObserved", "benign embedded object control")
require(V["CI-004"]["observations"][0]["class"] == "ReferencedThumbnail", "embedded object is referenced")

# Structural anomalies remain bounded.
require(E["CI-003"]["disposition"] == "ConcealedContentCandidate", "trailing data candidate")
require("benign_alternative" in V["CI-003"], "trailing data benign alternative")
require(E["CI-005"]["disposition"] == "CarrierStructureAnomalyObserved", "media mismatch restraint")
require(E["CI-006"]["disposition"] == "ParserDivergenceObserved", "parser divergence restraint")
require(len(V["CI-006"]["parser_observations"]) == 2, "two parser observations")

# Hidden truth noninterference pair: solver-visible semantics differ only in case/friendly name.
def strip(d, keys):
    return {k: v for k, v in d.items() if k not in keys}

require(
    strip(V["CI-007A"], {"case_id", "friendly_name"}) == strip(V["CI-007B"], {"case_id", "friendly_name"}),
    "CI-007 visible noninterference pair",
)
require(E["CI-007A"]["disposition"] == E["CI-007B"]["disposition"] == "CarrierStructureAnomalyObserved", "CI-007 same bounded output")
require(H["CI-007A"]["fixture_truth"] != H["CI-007B"]["fixture_truth"], "CI-007 hidden truth actually differs")

# Normalization never rewrites original.
norm = V["CI-008"]["normalization"]
require(V["CI-008"]["carrier_ref"] != norm["derived_artifact_ref"], "normalization distinct identity")
require(E["CI-008"]["disposition"] == "NormalizationDerivativeAvailable", "normalization disposition")

# Invisible text remains observation, not intent.
require(E["CI-009"]["disposition"] == "InvisibleTextStructureObserved", "invisible text restraint")
require("benign_alternative" in V["CI-009"], "invisible text benign alternative")

# Equal content can retain distinct acquisition lineages.
require(len(set(V["CI-010"]["acquisition_refs"])) == 2, "distinct acquisitions retained")
require(E["CI-010"].get("lineage_rule") == "DistinctAcquisitionsPreserved", "lineage expectation")

# Unsupported analysis is not a clean negative.
require(V["CI-011"]["analysis_outcome"] == "UnsupportedFeature", "unsupported input")
require(E["CI-011"]["disposition"] == "AnalysisUnsupported", "unsupported remains unsupported")

# Behavioral positive sensitivity.
a = V["CI-012A"]
b = V["CI-012B"]
require(strip(a, {"case_id", "behavior_observations"}) == strip(b, {"case_id", "behavior_observations"}), "CI-012 non-behavior fields match")
require(set(a["behavior_observations"]) - set(b["behavior_observations"]) == {"ScriptProcessObserved", "OutboundConnectionObserved"}, "CI-012 load-bearing behavior delta")
require(E["CI-012A"]["disposition"] == "SuspiciousExecutionCorrelationCandidate", "CI-012A correlation candidate")
require(E["CI-012B"]["disposition"] == "CarrierStructureAnomalyObserved", "CI-012B weaker without behavior")

# Direct synthetic examination is the only stronger synthetic concealment disposition.
require(V["CI-013"]["observations"] == V["CI-007A"]["observations"], "CI-013 base structure matches ambiguous case")
proof = V["CI-013"]["direct_examination_evidence"]
require(isinstance(proof, dict) and proof["establishes"] == "ConcealmentWithinSyntheticFixtureOnly", "direct synthetic examination receipt")
require(E["CI-013"]["disposition"] == "ConcealmentEstablishedUnderSyntheticProfile", "bounded synthetic establishment")
require(E["CI-013"]["maliciousness"] == "NotEstablished", "synthetic concealment does not establish maliciousness")

# Relation registry binds the intended metamorphic theorems.
kinds = {r["kind"] for r in relations}
require(kinds == {
    "HiddenEvaluatorNoninterference",
    "BehaviorCorrelationSensitivity",
    "DirectExaminationSensitivity",
    "NormalizationDoesNotRewriteOriginal",
}, "metamorphic relation registry")

# Safety scan: the visible fixtures must not contain executable payload or encoder recipe fields.
visible_text = FILES["visible"].read_text(encoding="utf-8").lower()
for forbidden_field in ["payload_bytes", "shellcode", "encoder_recipe", "exfiltration_endpoint", "secret_key_material"]:
    require(forbidden_field not in visible_text, f"forbidden operational field {forbidden_field}")

print("CONCEAL-001T v0.1 independent static oracle: PASS")
for name, path in FILES.items():
    print(f"sha256 {name} {sha256(path)} {path}")
print(f"cases {len(visible)}")
print(f"metamorphic_relations {len(relations)}")
