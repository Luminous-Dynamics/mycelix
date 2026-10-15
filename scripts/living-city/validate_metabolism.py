#!/usr/bin/env python3
"""Fail-closed CI checks for the Living-City metabolism/synergy contract. No network calls."""
from __future__ import annotations
import copy, json, sys
from datetime import date
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "schemas/living-city-metabolism-v0.1.schema.json"
PROTOCOL_PATH = ROOT / "fixtures/living-city/metabolism/johannesburg-civic-corridor-metabolism-protocol.v0.1.json"
SYNTHETIC_PATH = ROOT / "fixtures/living-city/metabolism/johannesburg-civic-corridor-synthetic.v0.1.json"
MANIFEST_PATH = ROOT / "fixtures/living-city/manifest.v0.1.json"
CLAIMS_DIR = ROOT / "fixtures/living-city/evidence/claims"

UNIT_BY_METRIC = {
    "final_energy": {"Wh", "kWh", "MWh", "MJ", "GJ"},
    "useful_energy": {"Wh", "kWh", "MWh", "MJ", "GJ"},
    "solar_generation": {"Wh", "kWh", "MWh", "MJ", "GJ"},
    "exergy_input": {"Wh_exergy", "kWh_exergy", "MJ_exergy", "GJ_exergy"},
    "exergy_output": {"Wh_exergy", "kWh_exergy", "MJ_exergy", "GJ_exergy"},
    "potable_water_withdrawal": {"L", "m3"}, "reclaimed_water_use": {"L", "m3"},
    "stormwater_runoff": {"L", "m3"}, "stormwater_retained": {"L", "m3"},
    "material_input": {"g", "kg", "t"}, "recovered_material": {"g", "kg", "t"},
    "landfill_waste": {"g", "kg", "t"},
    "operational_ghg": {"gCO2e", "kgCO2e", "tCO2e"},
    "embodied_ghg": {"gCO2e", "kgCO2e", "tCO2e"},
    "thermal_exposure_degree_hours": {"person_degree_hour"},
    "accessible_person_hours": {"person_hour"}, "essential_service_uptime": {"fraction"},
    "tree_canopy_area": {"m2"},
}

def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)

def load_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"{path.relative_to(ROOT)} is not readable valid JSON: {exc}")

def schema_errors(validator, item):
    return sorted(validator.iter_errors(item), key=lambda e: (list(map(str, e.path)), e.message))

def describe(errors):
    return "; ".join(f"{list(e.path)} [{e.validator}]: {e.message}" for e in errors) or "no validation errors"

def catalogs():
    manifest = load_json(MANIFEST_PATH)
    concepts = {}
    for entry in manifest.get("fixtures", []):
        if entry.get("expect_valid") is True:
            concept = load_json(ROOT / entry["path"])
            key = (concept["concept_id"], concept["version"])
            if key in concepts:
                fail(f"duplicate concept fixture version: {key!r}")
            concepts[key] = concept
    evidence = set()
    for path in sorted(CLAIMS_DIR.glob("*.json")):
        claim = load_json(path)
        if claim["evidence_id"] in evidence:
            fail(f"duplicate evidence claim ID: {claim['evidence_id']!r}")
        evidence.add(claim["evidence_id"])
    return concepts, evidence

def flow_for(scenario, metric):
    matches = [flow for flow in scenario["flow_records"] if flow["metric_id"] == metric]
    return matches[0] if len(matches) == 1 else None

def interpretation(direction, lower, upper):
    if direction == "higher_is_better":
        if lower > 0: return "positive_synergy"
        if upper < 0: return "negative_interaction"
    else:
        if upper < 0: return "positive_synergy"
        if lower > 0: return "negative_interaction"
    return "inconclusive"

def select_interaction_scenarios(review, interaction):
    scenarios = review["scenarios"]
    a, b = interaction["intervention_a"], interaction["intervention_b"]
    sets = [
        [s for s in scenarios if s["role"] == "baseline" and not s["intervention_refs"]],
        [s for s in scenarios if s["role"] == "single_intervention" and set(s["intervention_refs"]) == {a}],
        [s for s in scenarios if s["role"] == "single_intervention" and set(s["intervention_refs"]) == {b}],
        [s for s in scenarios if s["role"] == "combined_bundle" and set(s["intervention_refs"]) == {a, b}],
    ]
    return [items[0] for items in sets] if all(len(items) == 1 for items in sets) else None

def semantic_errors(review, concepts, evidence_ids):
    errors = []
    boundary = review["system_boundary"]
    if date.fromisoformat(boundary["start_date"]) > date.fromisoformat(boundary["end_date"]):
        errors.append("system_boundary: start_date is after end_date")
    cref = review["concept_ref"]
    key = (cref["concept_id"], cref["concept_version"])
    concept = concepts.get(key)
    if concept is None:
        errors.append(f"unresolved concept/version reference {key!r}")
    elif cref["hypothesis_id"] not in {h["hypothesis_id"] for h in concept["design_hypotheses"]}:
        errors.append(f"unresolved hypothesis reference {cref['hypothesis_id']!r} for {key!r}")

    scenario_ids = [s["scenario_id"] for s in review["scenarios"]]
    if len(scenario_ids) != len(set(scenario_ids)): errors.append("scenario_id values must be unique")
    by_scenario = {s["scenario_id"]: s for s in review["scenarios"]}
    run_ids = [r["model_run_id"] for r in review["model_runs"]]
    if len(run_ids) != len(set(run_ids)): errors.append("model_run_id values must be unique")
    by_run = {r["model_run_id"]: r for r in review["model_runs"]}

    status = review["record_status"]
    if status == "protocol_only" and (review["scenarios"] or review["interactions"] or review["model_runs"]):
        errors.append("protocol_only record cannot contain scenarios, interactions, or model runs")
    if status == "synthetic_test_only":
        if review["model_runs"]: errors.append("synthetic_test_only record cannot declare real model runs")
        for s in review["scenarios"]:
            for f in s["flow_records"]:
                if f["evidence_kind"] != "synthetic_only": errors.append(f"{s['scenario_id']}/{f['flow_id']}: synthetic record contains non-synthetic flow")
                if f["evidence_refs"]: errors.append(f"{s['scenario_id']}/{f['flow_id']}: synthetic flow must not cite real evidence")
        if any(i["assessment_status"] != "synthetic_only" for i in review["interactions"]):
            errors.append("synthetic_test_only record contains a non-synthetic interaction")
    if status == "assessment":
        if any(f["evidence_kind"] == "synthetic_only" for s in review["scenarios"] for f in s["flow_records"]):
            errors.append("assessment record cannot contain synthetic_only flow values")
        if any(i["assessment_status"] == "synthetic_only" for i in review["interactions"]):
            errors.append("assessment record cannot contain synthetic_only interaction results")

    for run in review["model_runs"]:
        if any(ref not in by_scenario for ref in run["scenario_refs"]):
            errors.append(f"{run['model_run_id']}: unresolved scenario_refs")
        for ref in run["validation_evidence_refs"]:
            if ref not in evidence_ids: errors.append(f"{run['model_run_id']}: unresolved validation evidence {ref!r}")
        if run["calibration_status"] == "validated" and not run["validation_evidence_refs"]:
            errors.append(f"{run['model_run_id']}: validated status requires validation evidence")

    for s in review["scenarios"]:
        ids = [f["flow_id"] for f in s["flow_records"]]
        if len(ids) != len(set(ids)): errors.append(f"{s['scenario_id']}: flow_id values must be unique")
        if s["role"] == "baseline" and s["intervention_refs"]: errors.append(f"{s['scenario_id']}: baseline cannot include interventions")
        if s["role"] == "single_intervention" and len(s["intervention_refs"]) != 1:
            errors.append(f"{s['scenario_id']}: single_intervention needs exactly one intervention")
        if s["role"] == "combined_bundle" and len(s["intervention_refs"]) < 2:
            errors.append(f"{s['scenario_id']}: combined_bundle needs at least two interventions")
        for f in s["flow_records"]:
            allowed = UNIT_BY_METRIC.get(f["metric_id"])
            if allowed is not None and f["unit"] not in allowed:
                errors.append(f"{s['scenario_id']}/{f['flow_id']}: unit {f['unit']!r} incompatible with {f['metric_id']!r}")
            for ref in f["evidence_refs"]:
                if ref not in evidence_ids: errors.append(f"{s['scenario_id']}/{f['flow_id']}: unresolved evidence reference {ref!r}")
            if f["evidence_kind"] in {"measured", "source_estimate", "model_output"} and not f["evidence_refs"]:
                errors.append(f"{s['scenario_id']}/{f['flow_id']}: {f['evidence_kind']} requires evidence references")
            if f["evidence_kind"] == "model_output":
                rid = f.get("model_run_ref")
                if rid not in by_run:
                    errors.append(f"{s['scenario_id']}/{f['flow_id']}: unresolved model_run_ref {rid!r}")
                elif s["scenario_id"] not in by_run[rid]["scenario_refs"]:
                    errors.append(f"{s['scenario_id']}/{f['flow_id']}: model run omits this scenario")
            if "uncertainty" in f:
                u = f["uncertainty"]
                if u["lower_bound"] > u["upper_bound"] or not u["lower_bound"] <= f["value"] <= u["upper_bound"]:
                    errors.append(f"{s['scenario_id']}/{f['flow_id']}: value and uncertainty bounds are inconsistent")
        for check in s["conservation_checks"]:
            expected_unit = {"energy": "Wh", "water": "mL", "mass": "g"}[check["balance_kind"]]
            check_context = f"{s['scenario_id']}/{check['check_id']}"
            if check["unit"] != expected_unit:
                errors.append(f"{check_context}: {check['balance_kind']} balance requires exact unit {expected_unit!r}")
            for ref in check["evidence_refs"]:
                if ref not in evidence_ids:
                    errors.append(f"{check_context}: unresolved conservation evidence reference {ref!r}")
            if review["record_status"] == "assessment" and not check["evidence_refs"]:
                errors.append(f"{check_context}: real assessment balance requires evidence references")
            if review["record_status"] == "synthetic_test_only" and check["evidence_refs"]:
                errors.append(f"{check_context}: synthetic balance must not cite real-world evidence")
            try:
                inflow = int(check["inflow"])
                generated = int(check["generated_or_added"])
                outflow = int(check["outflow"])
                loss = int(check["declared_loss"])
                storage = int(check["observed_storage_delta"])
                reported = int(check["reported_residual"])
                tolerance = int(check["tolerance"])
            except (ValueError, TypeError):
                errors.append(f"{check_context}: resource-balance quantities must be decimal integer strings")
                continue
            residual = inflow + generated - outflow - loss - storage
            if residual != reported:
                errors.append(f"{check_context}: reported residual does not equal inflow + generated_or_added - outflow - declared_loss - observed_storage_delta")
            if check["status"] == "balanced" and abs(residual) > tolerance:
                errors.append(f"{check_context}: declared balanced but integer residual exceeds tolerance")

    for item in review["interactions"]:
        for ref in item["evidence_refs"]:
            if ref not in evidence_ids: errors.append(f"{item['interaction_id']}: unresolved evidence reference {ref!r}")
        if item["intervention_a"] == item["intervention_b"]:
            errors.append(f"{item['interaction_id']}: intervention references must differ")
        stat = item["assessment_status"]
        if stat == "quantified" and not item["evidence_refs"]:
            errors.append(f"{item['interaction_id']}: quantified result requires supporting evidence")
        if stat in {"not_assessed", "qualitative"} and "reported_interaction_delta" in item:
            errors.append(f"{item['interaction_id']}: unquantified interaction cannot carry numeric delta")
        if status == "assessment" and stat == "synthetic_only":
            errors.append(f"{item['interaction_id']}: synthetic interaction not allowed in assessment")
        if stat not in {"quantified", "synthetic_only"}: continue
        selected = select_interaction_scenarios(review, item)
        if selected is None:
            errors.append(f"{item['interaction_id']}: difference-in-differences requires baseline, A only, B only, and combined scenarios")
            continue
        flows = [flow_for(s, item["outcome_metric"]) for s in selected]
        if any(f is None for f in flows):
            errors.append(f"{item['interaction_id']}: all four scenarios require exactly one {item['outcome_metric']!r} flow")
            continue
        if len({f["unit"] for f in flows}) != 1 or flows[0]["unit"] != item["unit"]:
            errors.append(f"{item['interaction_id']}: scenario-flow and reported interaction units do not match")
            continue
        delta = flows[3]["value"] - flows[1]["value"] - flows[2]["value"] + flows[0]["value"]
        if not math.isclose(delta, item["reported_interaction_delta"], rel_tol=0.0, abs_tol=item["tolerance"]):
            errors.append(f"{item['interaction_id']}: delta must equal combined - A - B + baseline")
        flow_intervals = [flow.get("uncertainty") for flow in flows]
        if any(interval is None for interval in flow_intervals):
            errors.append(f"{item['interaction_id']}: every scenario flow in a quantified interaction requires an uncertainty interval")
            continue
        # Order is baseline, A-only, B-only, combined. Interval arithmetic forms a
        # conservative enclosure; it is not a statistical confidence interval and
        # can be wider when scenario errors are correlated.
        baseline_u, a_u, b_u, combined_u = flow_intervals
        propagated_lower = combined_u["lower_bound"] - a_u["upper_bound"] - b_u["upper_bound"] + baseline_u["lower_bound"]
        propagated_upper = combined_u["upper_bound"] - a_u["lower_bound"] - b_u["lower_bound"] + baseline_u["upper_bound"]
        u = item["uncertainty"]
        lo, hi = u["lower_bound"], u["upper_bound"]
        if lo > hi or not lo <= item["reported_interaction_delta"] <= hi:
            errors.append(f"{item['interaction_id']}: delta inconsistent with uncertainty interval")
        if lo > propagated_lower or hi < propagated_upper:
            errors.append(f"{item['interaction_id']}: reported interaction interval understates propagated scenario uncertainty")
        if item["interpreted_result"] != interpretation(item["direction"], lo, hi):
            errors.append(f"{item['interaction_id']}: interpretation conflicts with uncertainty interval and metric direction")
        if stat == "synthetic_only":
            if item["evidence_refs"]: errors.append(f"{item['interaction_id']}: synthetic interaction must not cite real evidence")
            if any(f["evidence_kind"] != "synthetic_only" for f in flows):
                errors.append(f"{item['interaction_id']}: synthetic interaction must use synthetic flows only")
    return errors

def main():
    schema = load_json(SCHEMA_PATH)
    try: Draft202012Validator.check_schema(schema)
    except Exception as exc: fail(f"schema fails Draft 2020-12 meta-schema: {exc}")
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    concepts, evidence_ids = catalogs()
    protocol, synthetic = load_json(PROTOCOL_PATH), load_json(SYNTHETIC_PATH)
    for path, item in ((PROTOCOL_PATH, protocol), (SYNTHETIC_PATH, synthetic)):
        errors = schema_errors(validator, item)
        if errors: fail(f"{path.relative_to(ROOT)} schema-invalid: {describe(errors)}")
        semantic = semantic_errors(item, concepts, evidence_ids)
        if semantic: fail(f"{path.relative_to(ROOT)} semantic-invalid: " + "; ".join(semantic))
    print("PASS: protocol-only ledger contains no fabricated scenarios.")
    print("PASS: synthetic record passes the four-scenario interaction and balance baseline.")

    bad_unit = copy.deepcopy(synthetic)
    bad_unit["scenarios"][0]["flow_records"][0]["unit"] = "kWh"
    if not schema_errors(validator, bad_unit): fail("mutation: water metric with energy unit accepted")
    print("PASS mutation: metric/unit incompatibility rejected")

    wrong_balance_unit = copy.deepcopy(synthetic)
    wrong_balance_unit["scenarios"][0]["conservation_checks"][0]["unit"] = "m3"
    if not schema_errors(validator, wrong_balance_unit):
        fail("mutation: conservation ledger accepted a non-canonical water unit")
    print("PASS mutation: conservation ledger requires canonical integer water units")

    floating_balance_quantity = copy.deepcopy(synthetic)
    floating_balance_quantity["scenarios"][0]["conservation_checks"][0]["inflow"] = 100000000
    if not schema_errors(validator, floating_balance_quantity):
        fail("mutation: conservation ledger accepted a floating-point/JSON-number quantity")
    print("PASS mutation: conservation ledger rejects JSON-number quantities to preserve exactness")

    missing_balance_evidence = copy.deepcopy(synthetic)
    missing_balance_evidence["record_status"] = "assessment"
    evidence_id = next(iter(sorted(evidence_ids)))
    for scenario in missing_balance_evidence["scenarios"]:
        for flow in scenario["flow_records"]:
            flow["evidence_kind"] = "measured"
            flow["evidence_refs"] = [evidence_id]
    if not any("real assessment balance requires evidence references" in e for e in semantic_errors(missing_balance_evidence, concepts, evidence_ids)):
        fail("mutation: a real assessment balance without supporting evidence was accepted")
    print("PASS mutation: real assessed balances must cite supporting Mycelix evidence")

    bad_residual = copy.deepcopy(synthetic)
    bad_residual["scenarios"][0]["conservation_checks"][0]["reported_residual"] = "4"
    if not any("reported residual does not equal" in e for e in semantic_errors(bad_residual, concepts, evidence_ids)):
        fail("mutation: false conservation residual accepted")
    print("PASS mutation: exact integer residual must match the resource-balance equation")

    bad_status = copy.deepcopy(synthetic)
    bad_status["scenarios"][0]["conservation_checks"][0]["inflow"] = "120000000"
    if not any("declared balanced but integer residual exceeds" in e for e in semantic_errors(bad_status, concepts, evidence_ids)):
        fail("mutation: unbalanced check labelled balanced")
    print("PASS mutation: nonzero exact residual cannot be declared balanced")

    bad_delta = copy.deepcopy(synthetic)
    bad_delta["interactions"][0]["reported_interaction_delta"] = -12
    if not any("delta must equal combined" in e for e in semantic_errors(bad_delta, concepts, evidence_ids)):
        fail("mutation: wrong interaction delta accepted")
    print("PASS mutation: synergy interaction must match difference-in-differences")

    missing_case = copy.deepcopy(synthetic)
    missing_case["scenarios"] = [s for s in missing_case["scenarios"] if s["scenario_id"] != "canopy-only"]
    if not any("requires baseline, A only, B only" in e for e in semantic_errors(missing_case, concepts, evidence_ids)):
        fail("mutation: missing counterfactual accepted")
    print("PASS mutation: interaction requires baseline, A-only, B-only, and combined scenarios")

    fake_assessment = copy.deepcopy(synthetic)
    fake_assessment["record_status"] = "assessment"
    if not any("cannot contain synthetic_only flow values" in e for e in semantic_errors(fake_assessment, concepts, evidence_ids)):
        fail("mutation: synthetic values masquerade as actual assessment")
    print("PASS mutation: synthetic numbers cannot masquerade as real assessment")

    unresolved = copy.deepcopy(synthetic)
    unresolved["record_status"] = "assessment"
    flow = unresolved["scenarios"][0]["flow_records"][0]
    flow["evidence_kind"], flow["evidence_refs"] = "measured", ["does-not-exist"]
    if not any("unresolved evidence reference" in e for e in semantic_errors(unresolved, concepts, evidence_ids)):
        fail("mutation: unresolved measurement evidence accepted")
    print("PASS mutation: measurement evidence must resolve in Mycelix")

    model_missing = copy.deepcopy(synthetic)
    flow = model_missing["scenarios"][0]["flow_records"][0]
    flow["evidence_kind"], flow["evidence_refs"], flow["model_run_ref"] = "model_output", ["does-not-exist"], "missing-run"
    if not any("unresolved model_run_ref" in e for e in semantic_errors(model_missing, concepts, evidence_ids)):
        fail("mutation: model output without run provenance accepted")
    print("PASS mutation: model output requires a versioned model run")

    wrong_result = copy.deepcopy(synthetic)
    wrong_result["interactions"][0]["interpreted_result"] = "negative_interaction"
    if not any("interpretation conflicts" in e for e in semantic_errors(wrong_result, concepts, evidence_ids)):
        fail("mutation: interaction interpretation conflicts with direction/uncertainty")
    print("PASS mutation: interpretation respects uncertainty and metric direction")

    understated_interval = copy.deepcopy(synthetic)
    understated_interval["interactions"][0]["uncertainty"]["lower_bound"] = -17
    if not any("understates propagated scenario uncertainty" in e for e in semantic_errors(understated_interval, concepts, evidence_ids)):
        fail("mutation: interaction interval narrower than scenario-propagated uncertainty accepted")
    print("PASS mutation: interaction interval cannot understate the four scenario intervals")

    missing_flow_interval = copy.deepcopy(synthetic)
    missing_flow_interval["scenarios"][2]["flow_records"][0].pop("uncertainty")
    if not any("every scenario flow in a quantified interaction requires an uncertainty interval" in e for e in semantic_errors(missing_flow_interval, concepts, evidence_ids)):
        fail("mutation: quantified synergy without uncertainty on one counterfactual accepted")
    print("PASS mutation: all four counterfactual flows require uncertainty bounds")

    reverse_period = copy.deepcopy(protocol)
    reverse_period["system_boundary"]["start_date"] = "2027-01-01"
    if not any("start_date is after end_date" in e for e in semantic_errors(reverse_period, concepts, evidence_ids)):
        fail("mutation: reversed study period accepted")
    print("PASS mutation: reversed study period rejected")

    wrong_hypothesis = copy.deepcopy(protocol)
    wrong_hypothesis["concept_ref"]["hypothesis_id"] = "hypothesis-does-not-exist"
    if not any("unresolved hypothesis reference" in e for e in semantic_errors(wrong_hypothesis, concepts, evidence_ids)):
        fail("mutation: unresolved hypothesis accepted")
    print("PASS mutation: concept hypothesis reference must resolve")

    wrong_exergy = copy.deepcopy(synthetic)
    wrong_exergy["scenarios"][0]["flow_records"][0]["metric_id"] = "exergy_input"
    if not schema_errors(validator, wrong_exergy): fail("mutation: water volume unit accepted for exergy")
    print("PASS mutation: exergy units cannot be confused with water/energy units")

    missing_role = copy.deepcopy(protocol)
    missing_role["system_roles"].pop("symtropy")
    if not schema_errors(validator, missing_role): fail("mutation: omitted system responsibility accepted")
    print("PASS mutation: all four system roles are required")

    bad_run_ref = copy.deepcopy(synthetic)
    bad_run_ref["model_runs"] = [{
        "model_run_id":"unlinked-run", "engine":"symtropy", "engine_version":"0.1", "engine_revision":"a"*40,
        "input_manifest_sha256":"0"*64, "seed":1, "time_step_seconds":60,
        "scenario_refs":["scenario-not-present"], "calibration_status":"uncalibrated",
        "validation_evidence_refs":[]
    }]
    if not any("unresolved scenario_refs" in e for e in semantic_errors(bad_run_ref, concepts, evidence_ids)):
        fail("mutation: model run with unresolved scenario identity accepted")
    print("PASS mutation: model run must link to known scenario IDs")

    unpinned_revision = copy.deepcopy(synthetic)
    unpinned_revision["record_status"] = "assessment"
    unpinned_revision["model_runs"] = [{
        "model_run_id":"revision-control-run", "engine":"symtropy", "engine_version":"0.1",
        "engine_revision":"main", "input_manifest_sha256":"0"*64, "seed":1, "time_step_seconds":60,
        "scenario_refs":["baseline"], "calibration_status":"uncalibrated", "validation_evidence_refs":[]
    }]
    revision_errors = schema_errors(validator, unpinned_revision)
    if not any("engine_revision" in list(error.path) for error in revision_errors):
        fail("mutation: model run without an exact source revision was accepted")
    print("PASS mutation: model runs require an exact 40-character source revision SHA")

    print("PASS: 22 urban-metabolism/synergy controls (20 mutation controls and two positive baselines); no composite symtropy score or automatic authorization.")

if __name__ == "__main__":
    main()
