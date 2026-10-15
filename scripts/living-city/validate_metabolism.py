#!/usr/bin/env python3
"""Fail-closed CI checks for the Living-City metabolism/synergy contract. No network calls."""
from __future__ import annotations
import copy, json, sys
from decimal import Decimal
from fractions import Fraction
from datetime import date
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "schemas/living-city-metabolism-v0.1.schema.json"
PROTOCOL_PATH = ROOT / "fixtures/living-city/metabolism/johannesburg-civic-corridor-metabolism-protocol.v0.1.json"
SYNTHETIC_PATH = ROOT / "fixtures/living-city/metabolism/johannesburg-civic-corridor-synthetic.v0.1.json"
MANIFEST_PATH = ROOT / "fixtures/living-city/manifest.v0.1.json"
CLAIMS_DIR = ROOT / "fixtures/living-city/evidence/claims"

FLOW_UNIT_DIMENSION_FACTOR = {
    # Fractions preserve exact unit ratios: e.g. 9 MJ == 2,500 Wh.
    # Never round conversion factors before comparing to integer ledger terms.
    "Wh": ("energy", Fraction(1)),
    "kWh": ("energy", Fraction(1000)),
    "MWh": ("energy", Fraction(1000000)),
    "MJ": ("energy", Fraction(1000000, 3600)),
    "GJ": ("energy", Fraction(1000000000, 3600)),
    "L": ("water", Fraction(1000)),
    "m3": ("water", Fraction(1000000)),
    "g": ("mass", Fraction(1)),
    "kg": ("mass", Fraction(1000)),
    "t": ("mass", Fraction(1000000)),
}
BINDING_ROLE_FIELDS = {
    "inflow_refs": ("inflow", "add"),
    "generated_or_added_refs": ("generated_or_added", "add"),
    "outflow_refs": ("outflow", "add"),
    "loss_refs": ("declared_loss", "add"),
    "storage_increase_refs": ("observed_storage_delta", "add"),
    "storage_decrease_refs": ("observed_storage_delta", "subtract"),
}

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
    "edible_food_delivered": {"g", "kg", "t"},
    "operational_ghg": {"gCO2e", "kgCO2e", "tCO2e"},
    "embodied_ghg": {"gCO2e", "kgCO2e", "tCO2e"},
    "thermal_exposure_degree_hours": {"person_degree_hour"},
    "accessible_person_hours": {"person_hour"}, "essential_service_uptime": {"fraction"},
    "tree_canopy_area": {"m2"},
}

def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)

def exact_fraction(value):
    """Parse JSON decimal quantities into exact rational values, never binary floats."""
    if isinstance(value, Fraction):
        return value
    if isinstance(value, Decimal):
        return Fraction(value)
    return Fraction(str(value))

def reject_nonstandard_constant(token: str):
    """Reject NaN/Infinity tokens, which are not valid JSON numbers."""
    raise ValueError(f"non-standard JSON numeric constant: {token}")

def reject_duplicate_keys(pairs):
    """Reject duplicate object names rather than silently applying last-key-wins."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result

def load_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(
                stream,
                parse_float=Decimal,
                parse_constant=reject_nonstandard_constant,
                object_pairs_hook=reject_duplicate_keys,
            )
    except (OSError, UnicodeError, ValueError) as exc:
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

FOOD_UNIT_TO_GRAMS = {"g": Fraction(1), "kg": Fraction(1000), "t": Fraction(1000000)}

def nutrient_assessment_errors(review, evidence_ids):
    """Cross-check versioned Symtropy nutrient receipt envelopes; never infer absence as zero."""
    errors = []
    receipts = review.get("nutrient_assessments", [])
    ids = [r["receipt_id"] for r in receipts]
    if len(ids) != len(set(ids)):
        errors.append("nutrient assessment receipt_id values must be unique")
    by_receipt = {r["receipt_id"]: r for r in receipts}
    link_counts = {rid: 0 for rid in by_receipt}
    scenarios = {s["scenario_id"]: s for s in review["scenarios"]}

    for receipt in receipts:
        rid = receipt["receipt_id"]
        scenario_id = receipt["scenario_id"]
        if scenario_id not in scenarios:
            errors.append(f"{rid}: unresolved nutrient-assessment scenario_id {scenario_id!r}")
        for ref in receipt["evidence_refs"]:
            if ref not in evidence_ids:
                errors.append(f"{rid}: unresolved nutrient-assessment evidence reference {ref!r}")
        if review["record_status"] == "assessment":
            if receipt["receipt_origin"] != "symtropy_output":
                errors.append(f"{rid}: real assessment cannot use a synthetic-only nutrient receipt")
            if receipt["status"] in {"meets_declared_minimum", "shortfall_against_declared_minimum"} and not receipt["evidence_refs"]:
                errors.append(f"{rid}: assessed nutrient contribution needs resolving evidence references")
        if review["record_status"] == "synthetic_test_only":
            if receipt["receipt_origin"] != "synthetic_test_only":
                errors.append(f"{rid}: synthetic test record cannot contain a real Symtropy-output receipt")
            if receipt["evidence_refs"]:
                errors.append(f"{rid}: synthetic nutrient receipt must not cite real evidence")

        status = receipt["status"]
        if status in {"meets_declared_minimum", "shortfall_against_declared_minimum"}:
            if not receipt["composition_source_refs"]:
                errors.append(f"{rid}: assessed nutrient receipt needs at least one composition source reference")
            if not receipt["delivery_source_refs"]:
                errors.append(f"{rid}: assessed nutrient receipt needs at least one delivery source reference")
            try:
                scale = int(receipt["value_scale"])
                total_grams = int(receipt["total_edible_delivered_grams"])
                supply = int(receipt["supply_numerator"])
                target = int(receipt["target_numerator"])
                denominator = int(receipt["denominator"])
                shortfall = int(receipt["shortfall_numerator"])
            except (TypeError, ValueError):
                errors.append(f"{rid}: assessable nutrient receipt quantities must be exact decimal integer strings")
                continue
            if scale <= 0 or denominator != 100 * scale:
                errors.append(f"{rid}: nutrient receipt denominator must equal 100 * value_scale")
            if target <= 0:
                errors.append(f"{rid}: declared nutrient target numerator must be positive")
            expected_shortfall = max(target - supply, 0)
            if shortfall != expected_shortfall:
                errors.append(f"{rid}: nutrient receipt shortfall does not match exact supply and target numerators")
            expected_status = "meets_declared_minimum" if supply >= target else "shortfall_against_declared_minimum"
            if status != expected_status:
                errors.append(f"{rid}: nutrient receipt status conflicts with exact supply/target comparison")
        for scenario in review["scenarios"]:
            for flow in scenario["flow_records"]:
                refs = flow.get("nutrient_assessment_refs", [])
                if rid in refs:
                    if scenario_id != scenario["scenario_id"]:
                        errors.append(f"{rid}: receipt scenario does not match the flow scenario")
                    if flow.get("beneficiary_scope_ref") != receipt["target_scope_ref"]:
                        errors.append(f"{rid}: receipt target scope does not match the delivered-food flow scope")
                    if flow.get("delivery_period_ref") != receipt["period_ref"]:
                        errors.append(f"{rid}: receipt period does not match the delivered-food flow period")

    for scenario in review["scenarios"]:
        for flow in scenario["flow_records"]:
            refs = flow.get("nutrient_assessment_refs", [])
            if refs and flow["metric_id"] != "edible_food_delivered":
                errors.append(f"{scenario['scenario_id']}/{flow['flow_id']}: nutrient assessments may only attach to edible_food_delivered flows")
            for ref in refs:
                receipt = by_receipt.get(ref)
                if receipt is None:
                    errors.append(f"{scenario['scenario_id']}/{flow['flow_id']}: unresolved nutrient assessment reference {ref!r}")
                    continue
                link_counts[ref] += 1
                if receipt["status"] in {"meets_declared_minimum", "shortfall_against_declared_minimum"}:
                    total = int(receipt["total_edible_delivered_grams"])
                    factor = FOOD_UNIT_TO_GRAMS.get(flow["unit"])
                    if factor is None:
                        errors.append(f"{scenario['scenario_id']}/{flow['flow_id']}: unsupported food mass unit for receipt comparison")
                    else:
                        converted = exact_fraction(flow["value"]) * factor
                        if converted.denominator != 1 or converted.numerator != total:
                            errors.append(f"{receipt['receipt_id']}: receipt edible mass does not match the linked food-delivery flow")
    for rid, count in link_counts.items():
        if count != 1:
            errors.append(f"{rid}: nutrient assessment receipt must be linked to exactly one food-delivery flow (found {count})")
    return errors

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
        flow_by_id = {flow["flow_id"]: flow for flow in s["flow_records"]}
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
                if (exact_fraction(u["lower_bound"]) > exact_fraction(u["upper_bound"])
                    or not exact_fraction(u["lower_bound"]) <= exact_fraction(f["value"]) <= exact_fraction(u["upper_bound"])):
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

            bindings = check["flow_bindings"]
            seen_refs = set()
            aggregate_values = {"inflow": Fraction(0), "generated_or_added": Fraction(0),
                                "outflow": Fraction(0), "declared_loss": Fraction(0),
                                "storage_increase": Fraction(0), "storage_decrease": Fraction(0)}
            for role, (target_field, operation) in BINDING_ROLE_FIELDS.items():
                for ref in bindings[role]:
                    if ref in seen_refs:
                        errors.append(f"{check_context}: flow reference {ref!r} is double-counted across conservation roles")
                        continue
                    seen_refs.add(ref)
                    flow = flow_by_id.get(ref)
                    if flow is None:
                        errors.append(f"{check_context}: unknown flow reference in {role}: {ref!r}")
                        continue
                    unit_entry = FLOW_UNIT_DIMENSION_FACTOR.get(flow["unit"])
                    if unit_entry is None:
                        errors.append(f"{check_context}: flow {ref!r} has no supported balance-unit conversion for {flow['unit']!r}")
                        continue
                    dimension, factor = unit_entry
                    if dimension != check["balance_kind"]:
                        errors.append(f"{check_context}: flow {ref!r} dimension {dimension!r} cannot bind to {check['balance_kind']!r} balance")
                        continue
                    key_name = "storage_increase" if role == "storage_increase_refs" else "storage_decrease" if role == "storage_decrease_refs" else target_field
                    aggregate_values[key_name] += Fraction(str(flow["value"])) * factor

            # Tolerance applies to the observed balance residual, not to lossy
            # unit conversion. Every linked role total must be exactly expressible
            # in its canonical integer base unit before tolerance is considered.
            for role_name, aggregate in aggregate_values.items():
                if aggregate.denominator != 1:
                    errors.append(
                        f"{check_context}: flow bindings for {role_name} are not exactly representable "
                        f"as an integer {check['unit']} quantity; conversion precision loss cannot be "
                        "hidden by the balance tolerance"
                    )

            comparisons = [
                ("inflow", Fraction(inflow)),
                ("generated_or_added", Fraction(generated)),
                ("outflow", Fraction(outflow)),
                ("declared_loss", Fraction(loss)),
                ("observed_storage_delta", Fraction(storage)),
            ]
            for field_name, declared in comparisons:
                if field_name == "observed_storage_delta":
                    aggregate = aggregate_values["storage_increase"] - aggregate_values["storage_decrease"]
                else:
                    aggregate = aggregate_values[field_name]
                if abs(aggregate - declared) > Fraction(tolerance):
                    errors.append(f"{check_context}: flow bindings for {field_name} do not match the declared base-unit quantity")

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
        if item["outcome_metric"] == "edible_food_delivered":
            food_comparability = {
                (flow.get("beneficiary_scope_ref"), flow.get("delivery_period_ref"), flow.get("delivery_basis"))
                for flow in flows
            }
            if len(food_comparability) != 1 or any(None in fields for fields in food_comparability):
                errors.append(
                    f"{item['interaction_id']}: edible-food counterfactuals require matching beneficiary scope, "
                    "delivery period, and delivered-edible-portion basis"
                )
                continue
        delta = (exact_fraction(flows[3]["value"]) - exact_fraction(flows[1]["value"])
                 - exact_fraction(flows[2]["value"]) + exact_fraction(flows[0]["value"]))
        if abs(delta - exact_fraction(item["reported_interaction_delta"])) > exact_fraction(item["tolerance"]):
            errors.append(f"{item['interaction_id']}: delta must equal combined - A - B + baseline")
        flow_intervals = [flow.get("uncertainty") for flow in flows]
        if any(interval is None for interval in flow_intervals):
            errors.append(f"{item['interaction_id']}: every scenario flow in a quantified interaction requires an uncertainty interval")
            continue
        # Order is baseline, A-only, B-only, combined. Interval arithmetic forms a
        # conservative enclosure; it is not a statistical confidence interval and
        # can be wider when scenario errors are correlated.
        baseline_u, a_u, b_u, combined_u = flow_intervals
        propagated_lower = (exact_fraction(combined_u["lower_bound"]) - exact_fraction(a_u["upper_bound"])
                            - exact_fraction(b_u["upper_bound"]) + exact_fraction(baseline_u["lower_bound"]))
        propagated_upper = (exact_fraction(combined_u["upper_bound"]) - exact_fraction(a_u["lower_bound"])
                            - exact_fraction(b_u["lower_bound"]) + exact_fraction(baseline_u["upper_bound"]))
        u = item["uncertainty"]
        lo, hi = exact_fraction(u["lower_bound"]), exact_fraction(u["upper_bound"])
        reported_delta = exact_fraction(item["reported_interaction_delta"])
        if lo > hi or not lo <= reported_delta <= hi:
            errors.append(f"{item['interaction_id']}: delta inconsistent with uncertainty interval")
        if lo > propagated_lower or hi < propagated_upper:
            errors.append(f"{item['interaction_id']}: reported interaction interval understates propagated scenario uncertainty")
        if item["interpreted_result"] != interpretation(item["direction"], lo, hi):
            errors.append(f"{item['interaction_id']}: interpretation conflicts with uncertainty interval and metric direction")
        if stat == "synthetic_only":
            if item["evidence_refs"]: errors.append(f"{item['interaction_id']}: synthetic interaction must not cite real evidence")
            if any(f["evidence_kind"] != "synthetic_only" for f in flows):
                errors.append(f"{item['interaction_id']}: synthetic interaction must use synthetic flows only")
    impacts = review.get("distributional_impacts", [])
    coverage = review.get("distributional_coverage")
    covered_group_refs = set()
    if coverage is not None:
        covered_group_refs = set(coverage["included_group_refs"])
        for ref in coverage["evidence_refs"]:
            if ref not in evidence_ids:
                errors.append(f"distributional coverage: unresolved evidence reference {ref!r}")
    impact_ids = [item["impact_id"] for item in impacts]
    if len(impact_ids) != len(set(impact_ids)):
        errors.append("distributional impact_id values must be unique")
    comparison_keys = [
        (item["scenario_id"], item["baseline_scenario_ref"], item["affected_group_ref"], item["metric_id"])
        for item in impacts
    ]
    if len(comparison_keys) != len(set(comparison_keys)):
        errors.append("distributional scenario/baseline/group/metric comparisons must be unique")
    for impact in impacts:
        iid = impact["impact_id"]
        scenario_id = impact["scenario_id"]
        baseline_id = impact["baseline_scenario_ref"]
        scenario = by_scenario.get(scenario_id)
        baseline = by_scenario.get(baseline_id)
        if scenario is None:
            errors.append(f"{iid}: unresolved distributional scenario_id {scenario_id!r}")
        if baseline is None:
            errors.append(f"{iid}: unresolved distributional baseline_scenario_ref {baseline_id!r}")
        elif baseline["role"] != "baseline":
            errors.append(f"{iid}: distributional baseline_scenario_ref must resolve to a baseline-role scenario")
        if scenario_id == baseline_id:
            errors.append(f"{iid}: intervention and baseline scenario references must differ")
        if impact["affected_group_ref"].strip() == "":
            errors.append(f"{iid}: affected_group_ref cannot be empty")
        if status == "assessment" and coverage is not None and impact["affected_group_ref"] not in covered_group_refs:
            errors.append(f"{iid}: distributional impact group is missing from coverage declaration")
        for ref in impact["evidence_refs"]:
            if ref not in evidence_ids:
                errors.append(f"{iid}: unresolved distributional evidence reference {ref!r}")
        metric_units = UNIT_BY_METRIC.get(impact["metric_id"])
        if metric_units is not None and impact["unit"] not in metric_units:
            errors.append(f"{iid}: unit {impact['unit']!r} incompatible with distributional metric {impact['metric_id']!r}")
        if status == "assessment" and impact["assessment_status"] in {"quantified", "qualitative"} and not impact["evidence_refs"]:
            errors.append(f"{iid}: real distributional impact requires evidence references")
        if impact["assessment_status"] == "quantified":
            baseline_value = exact_fraction(impact["baseline_value"])
            scenario_value = exact_fraction(impact["scenario_value"])
            delta = exact_fraction(impact["reported_delta"])
            expected_delta = scenario_value - baseline_value
            if delta != expected_delta:
                errors.append(f"{iid}: reported_delta must equal scenario_value - baseline_value exactly")
            uncertainty = impact["uncertainty"]
            lower, upper = exact_fraction(uncertainty["lower_bound"]), exact_fraction(uncertainty["upper_bound"])
            if lower > upper or not lower <= delta <= upper:
                errors.append(f"{iid}: uncertainty interval must be ordered and enclose reported_delta")
        if impact["constraint_status"] != "not_evaluated" and not impact["constraint_ref"]:
            errors.append(f"{iid}: evaluated non-compensatory floor needs a constraint reference")
        if impact["constraint_status"] in {"within_declared_floor", "breach", "indeterminate"} and not impact["evidence_refs"]:
            errors.append(f"{iid}: evaluated non-compensatory floor needs evidence references")
    errors.extend(nutrient_assessment_errors(review, evidence_ids))
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

    unknown_flow_binding = copy.deepcopy(synthetic)
    unknown_flow_binding["scenarios"][0]["conservation_checks"][0]["flow_bindings"]["outflow_refs"] = ["flow-does-not-exist"]
    if not any("unknown flow reference in outflow_refs" in e for e in semantic_errors(unknown_flow_binding, concepts, evidence_ids)):
        fail("mutation: conservation ledger with an unresolved flow binding was accepted")
    print("PASS mutation: conservation flow bindings must resolve within the scenario")

    mismatched_flow_binding = copy.deepcopy(synthetic)
    mismatched_flow_binding["scenarios"][0]["conservation_checks"][0]["outflow"] = "99000000"
    if not any("flow bindings for outflow do not match" in e for e in semantic_errors(mismatched_flow_binding, concepts, evidence_ids)):
        fail("mutation: declared outflow that disagrees with linked flow records was accepted")
    print("PASS mutation: physical flow records and conservation totals must agree")

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

    interval_crosses_zero = copy.deepcopy(synthetic)
    interval_crosses_zero["interactions"][0]["uncertainty"]["lower_bound"] = -20
    interval_crosses_zero["interactions"][0]["uncertainty"]["upper_bound"] = 2
    interval_crosses_zero["interactions"][0]["interpreted_result"] = "inconclusive"
    if semantic_errors(interval_crosses_zero, concepts, evidence_ids):
        fail("positive control: a propagated interaction interval crossing zero should be accepted as inconclusive")
    print("PASS control: uncertainty spanning both signs remains inconclusive despite a favorable point estimate")

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
    if not any(error.validator == "required" and "engine_revision" in error.message for error in revision_errors):
        fail("mutation: model run without an exact source revision was accepted")
    print("PASS mutation: model runs require an exact 40-character source revision SHA")

    exact_energy = copy.deepcopy(synthetic)
    baseline_scenario = exact_energy["scenarios"][0]
    baseline_scenario["flow_records"].extend([
        {
            "flow_id": "baseline-energy-input-9-mj", "metric_id": "final_energy",
            "value": 9, "unit": "MJ", "evidence_kind": "synthetic_only",
            "life_cycle_stage": "operational", "evidence_refs": [],
            "rationale": "Synthetic exact-unit conversion control only; not a measured energy flow."
        },
        {
            "flow_id": "baseline-energy-output-2500-wh", "metric_id": "final_energy",
            "value": 2500, "unit": "Wh", "evidence_kind": "synthetic_only",
            "life_cycle_stage": "operational", "evidence_refs": [],
            "rationale": "Synthetic canonical-unit conversion control only; not a measured energy flow."
        }
    ])
    baseline_scenario["conservation_checks"].append({
        "check_id": "baseline-energy-exact-conversion", "balance_kind": "energy",
        "inflow": "2500", "generated_or_added": "0", "outflow": "2500",
        "declared_loss": "0", "observed_storage_delta": "0", "reported_residual": "0",
        "tolerance": "0", "unit": "Wh", "status": "balanced",
        "rationale": "Synthetic exact arithmetic control: 9 MJ equals 2500 Wh exactly.",
        "evidence_refs": [],
        "flow_bindings": {
            "inflow_refs": ["baseline-energy-input-9-mj"],
            "generated_or_added_refs": [], "outflow_refs": ["baseline-energy-output-2500-wh"],
            "loss_refs": [], "storage_increase_refs": [], "storage_decrease_refs": []
        }
    })
    exact_errors = schema_errors(validator, exact_energy)
    if exact_errors:
        fail("positive control: exact MJ-to-Wh conversion fixture fails schema validation: " + describe(exact_errors))
    exact_semantic = semantic_errors(exact_energy, concepts, evidence_ids)
    if exact_semantic:
        fail("positive control: exact MJ-to-Wh conversion failed conservation checks: " + "; ".join(exact_semantic))
    print("PASS control: rational MJ/GJ conversion preserves exact integer-Wh balances")

    wrong_energy_conversion = copy.deepcopy(exact_energy)
    wrong_energy_input = next(f for f in wrong_energy_conversion["scenarios"][0]["flow_records"] if f["flow_id"] == "baseline-energy-input-9-mj")
    wrong_energy_input["value"] = 9.0036  # Exactly 2,501 Wh; declared inflow remains 2,500 Wh.
    if not any("flow bindings for inflow do not match" in e for e in semantic_errors(wrong_energy_conversion, concepts, evidence_ids)):
        fail("mutation: non-matching exact energy conversion was accepted")
    print("PASS mutation: exact energy conversion rejects a one-Wh balance mismatch")

    fractional_energy = copy.deepcopy(exact_energy)
    fractional_scenario = fractional_energy["scenarios"][0]
    fractional_input = next(f for f in fractional_scenario["flow_records"] if f["flow_id"] == "baseline-energy-input-9-mj")
    fractional_output = next(f for f in fractional_scenario["flow_records"] if f["flow_id"] == "baseline-energy-output-2500-wh")
    fractional_input["value"] = 1
    fractional_output["value"], fractional_output["unit"] = 1, "MJ"
    fractional_balance = fractional_scenario["conservation_checks"][-1]
    fractional_balance["inflow"], fractional_balance["outflow"], fractional_balance["tolerance"] = "278", "278", "1"
    if not any("not exactly representable as an integer Wh quantity" in e for e in semantic_errors(fractional_energy, concepts, evidence_ids)):
        fail("mutation: fractional Wh conversion hidden by a nonzero balance tolerance was accepted")
    print("PASS mutation: non-integral canonical energy conversion cannot hide inside balance tolerance")

    decimal_case = copy.deepcopy(synthetic)
    decimal_values = [Decimal("0.3"), Decimal("0.1"), Decimal("0.2"), Decimal("0.4")]
    for scenario, value in zip(decimal_case["scenarios"], decimal_values):
        scenario["flow_records"].append({
            "flow_id": scenario["scenario_id"] + "-decimal-access",
            "metric_id": "accessible_person_hours", "value": value, "unit": "person_hour",
            "evidence_kind": "synthetic_only", "life_cycle_stage": "operational",
            "evidence_refs": [],
            "rationale": "Synthetic decimal arithmetic control only; not a real accessibility measurement.",
            "uncertainty": {"lower_bound": value, "upper_bound": value,
                            "method": "Exact decimal point interval for arithmetic regression only."}
        })
    decimal_interaction = decimal_case["interactions"][0]
    decimal_interaction["outcome_metric"] = "accessible_person_hours"
    decimal_interaction["direction"] = "higher_is_better"
    decimal_interaction["reported_interaction_delta"] = Decimal("0.4")
    decimal_interaction["tolerance"] = Decimal("0")
    decimal_interaction["uncertainty"]["lower_bound"] = Decimal("0.4")
    decimal_interaction["uncertainty"]["upper_bound"] = Decimal("0.4")
    decimal_interaction["interpreted_result"] = "positive_synergy"
    decimal_schema_errors = schema_errors(validator, decimal_case)
    if decimal_schema_errors:
        fail("positive control: exact decimal interaction fails schema validation: " + describe(decimal_schema_errors))
    if semantic_errors(decimal_case, concepts, evidence_ids):
        fail("positive control: exact decimal difference-in-differences was not accepted")
    print("PASS control: decimal JSON inputs preserve exact difference-in-differences at zero tolerance")

    decimal_drift = copy.deepcopy(decimal_case)
    decimal_drift["interactions"][0]["reported_interaction_delta"] = Decimal("0.4000000000000001")
    if not any("delta must equal combined" in e for e in semantic_errors(decimal_drift, concepts, evidence_ids)):
        fail("mutation: sub-ULP decimal interaction mismatch was accepted at zero tolerance")
    print("PASS mutation: exact decimal interaction rejects tiny declared-delta drift at zero tolerance")

    nonstandard_json_number_rejected = False
    try:
        json.loads('{"value": NaN}', parse_float=Decimal, parse_constant=reject_nonstandard_constant)
    except ValueError:
        nonstandard_json_number_rejected = True
    if not nonstandard_json_number_rejected:
        fail("mutation: non-standard JSON NaN numeric constant was accepted")
    print("PASS mutation: non-standard JSON NaN/Infinity constants are rejected at the parser boundary")

    duplicate_key_rejected = False
    try:
        json.loads('{"record_status":"assessment","record_status":"synthetic_test_only"}',
                   parse_float=Decimal, parse_constant=reject_nonstandard_constant,
                   object_pairs_hook=reject_duplicate_keys)
    except ValueError:
        duplicate_key_rejected = True
    if not duplicate_key_rejected:
        fail("mutation: duplicate JSON object key was accepted with ambiguous last-key-wins behavior")
    print("PASS mutation: duplicate JSON object keys are rejected at the parser boundary")

    food_delivery_case = copy.deepcopy(synthetic)
    food_values = [100, 140, 125, 180]  # Synthetic kg, not a crop/yield forecast.
    for scenario, value in zip(food_delivery_case["scenarios"], food_values):
        scenario["flow_records"].append({
            "flow_id": scenario["scenario_id"] + "-edible-food",
            "metric_id": "edible_food_delivered",
            "value": value,
            "unit": "kg",
            "evidence_kind": "synthetic_only",
            "life_cycle_stage": "operational",
            "evidence_refs": [],
            "rationale": "Synthetic edible food delivered solely for scenario-contract tests.",
            "beneficiary_scope_ref": "corridor-beneficiary-cohort",
            "delivery_period_ref": "synthetic-corridor-year",
            "delivery_basis": "edible_portion_received_by_beneficiary",
            "uncertainty": {
                "lower_bound": value - 5,
                "upper_bound": value + 5,
                "method": "Synthetic ±5 kg interval for arithmetic testing only; not a measured uncertainty."
            }
        })
    food_item = food_delivery_case["interactions"][0]
    food_item["outcome_metric"] = "edible_food_delivered"
    food_item["unit"] = "kg"
    food_item["direction"] = "higher_is_better"
    food_item["reported_interaction_delta"] = 15
    food_item["tolerance"] = 0
    food_item["uncertainty"]["lower_bound"] = -5
    food_item["uncertainty"]["upper_bound"] = 35
    food_item["interpreted_result"] = "inconclusive"
    food_schema_errors = schema_errors(validator, food_delivery_case)
    if food_schema_errors:
        fail("positive control: edible food delivery scenario fails schema: " + describe(food_schema_errors))
    food_semantic_errors = semantic_errors(food_delivery_case, concepts, evidence_ids)
    if food_semantic_errors:
        fail("positive control: edible food delivery counterfactual is not valid: " + "; ".join(food_semantic_errors))
    print("PASS control: food-delivery interaction is evaluated across all four counterfactuals; uncertainty spanning zero stays inconclusive")

    invalid_food_unit = copy.deepcopy(food_delivery_case)
    invalid_food_flow = next(f for f in invalid_food_unit["scenarios"][0]["flow_records"] if f["metric_id"] == "edible_food_delivered")
    invalid_food_flow["unit"] = "Wh"
    if not schema_errors(validator, invalid_food_unit):
        fail("mutation: edible food delivered with an energy unit was accepted")
    print("PASS mutation: edible food delivery requires a mass unit")

    missing_food_scope = copy.deepcopy(food_delivery_case)
    missing_food_flow = next(f for f in missing_food_scope["scenarios"][0]["flow_records"] if f["metric_id"] == "edible_food_delivered")
    missing_food_flow.pop("beneficiary_scope_ref")
    if not schema_errors(validator, missing_food_scope):
        fail("mutation: edible food delivery without a beneficiary scope was accepted")
    print("PASS mutation: edible food delivery must name a beneficiary scope")

    mismatched_food_period = copy.deepcopy(food_delivery_case)
    mismatched_food_flow = next(f for f in mismatched_food_period["scenarios"][1]["flow_records"] if f["metric_id"] == "edible_food_delivered")
    mismatched_food_flow["delivery_period_ref"] = "synthetic-different-period"
    if not any("require matching beneficiary scope, delivery period" in e for e in semantic_errors(mismatched_food_period, concepts, evidence_ids)):
        fail("mutation: food-delivery interaction across incomparable periods was accepted")
    print("PASS mutation: food-delivery interaction rejects mismatched beneficiary/period/basis")

    non_operational_food = copy.deepcopy(food_delivery_case)
    non_operational_flow = next(f for f in non_operational_food["scenarios"][0]["flow_records"] if f["metric_id"] == "edible_food_delivered")
    non_operational_flow["life_cycle_stage"] = "embodied_construction"
    if not schema_errors(validator, non_operational_food):
        fail("mutation: delivered edible food labelled as embodied construction was accepted")
    print("PASS mutation: delivered-food service outcomes must use the operational life-cycle stage")

    receipt_case = copy.deepcopy(food_delivery_case)
    receipt_flow = next(
        f for f in receipt_case["scenarios"][-1]["flow_records"]
        if f["metric_id"] == "edible_food_delivered"
    )
    receipt_flow["nutrient_assessment_refs"] = ["synthetic-vitamin-c-assessment"]
    receipt_case["nutrient_assessments"] = [{
        "receipt_id": "synthetic-vitamin-c-assessment",
        "receipt_schema_version": "living-city-nutrient-assessment-envelope/0.1",
        "receipt_origin": "synthetic_test_only",
        "scenario_id": "bioswale-and-canopy",
        "target_id": "synthetic-vitamin-c-target",
        "nutrient_id": "vitamin-c",
        "nutrient_definition_ref": "synthetic-vitamin-c-definition",
        "unit": "milligrams",
        "target_scope_ref": "corridor-beneficiary-cohort",
        "period_ref": "synthetic-corridor-year",
        "value_scale": "1000",
        "total_edible_delivered_grams": "180000",
        "supply_numerator": "100000000",
        "target_numerator": "50000000",
        "denominator": "100000",
        "shortfall_numerator": "0",
        "composition_source_refs": ["synthetic-composition-source"],
        "delivery_source_refs": ["synthetic-delivery-source"],
        "target_source_ref": "synthetic-target-source",
        "status": "meets_declared_minimum",
        "evidence_refs": [],
        "limitations": ["Synthetic arithmetic receipt only; no actual food composition or dietary adequacy claim."]
    }]
    receipt_schema_errors = schema_errors(validator, receipt_case)
    if receipt_schema_errors:
        fail("positive control: valid synthetic nutrient receipt fails schema: " + describe(receipt_schema_errors))
    receipt_semantic = semantic_errors(receipt_case, concepts, evidence_ids)
    if receipt_semantic:
        fail("positive control: valid synthetic nutrient receipt fails semantic checks: " + "; ".join(receipt_semantic))
    print("PASS control: versioned nutrient receipt resolves to a same-scope, same-period food flow and exact edible mass")

    missing_composition_source = copy.deepcopy(receipt_case)
    missing_composition_source["nutrient_assessments"][0]["composition_source_refs"] = []
    if not schema_errors(validator, missing_composition_source) or not any(
        "needs at least one composition source reference" in e
        for e in semantic_errors(missing_composition_source, concepts, evidence_ids)
    ):
        fail("mutation: assessed nutrient receipt without a composition source reference was accepted")
    print("PASS mutation: assessed nutrient receipt requires at least one composition source reference")

    missing_delivery_source = copy.deepcopy(receipt_case)
    missing_delivery_source["nutrient_assessments"][0]["delivery_source_refs"] = []
    if not schema_errors(validator, missing_delivery_source) or not any(
        "needs at least one delivery source reference" in e
        for e in semantic_errors(missing_delivery_source, concepts, evidence_ids)
    ):
        fail("mutation: assessed nutrient receipt without a delivery source reference was accepted")
    print("PASS mutation: assessed nutrient receipt requires at least one delivery source reference")

    not_assessable_case = copy.deepcopy(receipt_case)
    not_assessable_receipt = not_assessable_case["nutrient_assessments"][0]
    not_assessable_receipt["receipt_id"] = "synthetic-missing-composition-assessment"
    not_assessable_receipt["status"] = "not_assessable"
    for key in ("total_edible_delivered_grams", "supply_numerator", "target_numerator", "denominator", "shortfall_numerator"):
        not_assessable_receipt[key] = None
    not_assessable_flow = next(
        f for f in not_assessable_case["scenarios"][-1]["flow_records"]
        if f["metric_id"] == "edible_food_delivered"
    )
    not_assessable_flow["nutrient_assessment_refs"] = ["synthetic-missing-composition-assessment"]
    if schema_errors(validator, not_assessable_case) or semantic_errors(not_assessable_case, concepts, evidence_ids):
        fail("positive control: missing nutrient composition must remain NotAssessable, not zero or a fabricated shortfall")
    print("PASS control: NotAssessable nutrition receipt preserves missingness instead of turning missing composition into zero")

    unknown_receipt_link = copy.deepcopy(receipt_case)
    unknown_receipt_link["scenarios"][-1]["flow_records"][-1]["nutrient_assessment_refs"] = ["missing-nutrient-receipt"]
    if not any("unresolved nutrient assessment reference" in e for e in semantic_errors(unknown_receipt_link, concepts, evidence_ids)):
        fail("mutation: unresolved nutrient receipt reference was accepted")
    print("PASS mutation: food flow cannot reference a missing nutrient receipt")

    wrong_receipt_scope = copy.deepcopy(receipt_case)
    wrong_receipt_scope["nutrient_assessments"][0]["target_scope_ref"] = "different-beneficiary-cohort"
    if not any("receipt target scope does not match" in e for e in semantic_errors(wrong_receipt_scope, concepts, evidence_ids)):
        fail("mutation: receipt from another beneficiary scope was accepted")
    print("PASS mutation: nutrient receipt must match the food-flow beneficiary scope")

    wrong_receipt_mass = copy.deepcopy(receipt_case)
    wrong_receipt_mass["nutrient_assessments"][0]["total_edible_delivered_grams"] = "180001"
    if not any("receipt edible mass does not match" in e for e in semantic_errors(wrong_receipt_mass, concepts, evidence_ids)):
        fail("mutation: nutrient receipt with a one-gram mass mismatch was accepted")
    print("PASS mutation: nutrient receipt total must match the linked food flow exactly")

    wrong_receipt_shortfall = copy.deepcopy(receipt_case)
    wrong_receipt_shortfall["nutrient_assessments"][0]["shortfall_numerator"] = "1"
    if not any("shortfall does not match exact supply and target" in e for e in semantic_errors(wrong_receipt_shortfall, concepts, evidence_ids)):
        fail("mutation: arithmetic-inconsistent nutrient receipt was accepted")
    print("PASS mutation: nutrient receipt shortfall must recompute from exact numerators")

    orphan_receipt = copy.deepcopy(receipt_case)
    orphan_receipt["scenarios"][-1]["flow_records"][-1]["nutrient_assessment_refs"] = []
    if not any("receipt must be linked to exactly one food-delivery flow" in e for e in semantic_errors(orphan_receipt, concepts, evidence_ids)):
        fail("mutation: unreferenced nutrient receipt was accepted")
    print("PASS mutation: each nutrient receipt must link to exactly one food-delivery flow")

    unpinned_receipt = copy.deepcopy(receipt_case)
    unpinned_receipt["nutrient_assessments"][0]["receipt_origin"] = "symtropy_output"
    unpinned_receipt_errors = schema_errors(validator, unpinned_receipt)
    if not any(
        error.validator == "required" and
        any(field in error.message for field in ("producer_revision", "payload_ref", "payload_sha256"))
        for error in unpinned_receipt_errors
    ):
        fail("mutation: real Symtropy receipt without producer revision/payload commitment was accepted")
    print("PASS mutation: a real Symtropy receipt requires a pinned producer revision and payload hash")


    distributional_case = copy.deepcopy(food_delivery_case)
    distributional_case["distributional_impacts"] = [{
        "impact_id": "synthetic-renter-access-change",
        "scenario_id": "bioswale-and-canopy",
        "baseline_scenario_ref": "baseline",
        "affected_group_ref": "synthetic-renters-cohort",
        "metric_id": "accessible_person_hours",
        "assessment_status": "quantified",
        "baseline_value": 100,
        "scenario_value": 112,
        "reported_delta": 12,
        "unit": "person_hour",
        "direction": "higher_is_better",
        "uncertainty": {"lower_bound": 8, "upper_bound": 16,
                        "method": "Synthetic interval for the cohort-level delta only; not a real access finding."},
        "evidence_refs": [],
        "limitations": ["Synthetic cohort-level control only; not evidence about actual residents or a real intervention."],
        "constraint_status": "not_evaluated",
        "constraint_ref": None
    }]
    distributional_case["distributional_coverage"] = {
        "coverage_schema_version": "living-city-distributional-coverage/0.1",
        "coverage_status": "not_assessed",
        "population_frame": "Synthetic cohort only; this fixture has no real population frame.",
        "group_identification_method": "Fixed synthetic test value; no real-world group identification was performed.",
        "included_group_refs": ["synthetic-renters-cohort"],
        "known_omissions": ["All real-world cohorts and actual population sampling are intentionally omitted."],
        "omission_review_method": "Synthetic fixture declaration only; no assessment of actual population omissions.",
        "intersectional_review_status": "not_performed",
        "intersectional_dimensions_considered": [],
        "evidence_refs": [],
        "limitations": ["Synthetic coverage metadata only; not proof of population completeness."]
    }
    if schema_errors(validator, distributional_case) or semantic_errors(distributional_case, concepts, evidence_ids):
        fail("positive control: valid cohort-disaggregated impact receipt was rejected")
    print("PASS control: cohort impact retains baseline, exact delta, uncertainty and independent floor status")

    wrong_distribution_delta = copy.deepcopy(distributional_case)
    wrong_distribution_delta["distributional_impacts"][0]["reported_delta"] = Decimal("12.0000000000000001")
    if not any("reported_delta must equal" in e for e in semantic_errors(wrong_distribution_delta, concepts, evidence_ids)):
        fail("mutation: sub-ULP distributional delta mismatch accepted")
    print("PASS mutation: distributional change must equal exact scenario minus baseline")

    distribution_interval_excludes_delta = copy.deepcopy(distributional_case)
    distribution_interval_excludes_delta["distributional_impacts"][0]["uncertainty"]["lower_bound"] = 13
    if not any("uncertainty interval must be ordered and enclose reported_delta" in e for e in semantic_errors(distribution_interval_excludes_delta, concepts, evidence_ids)):
        fail("mutation: cohort uncertainty interval not enclosing delta accepted")
    print("PASS mutation: cohort uncertainty must enclose the exact change")

    unknown_distribution_baseline = copy.deepcopy(distributional_case)
    unknown_distribution_baseline["distributional_impacts"][0]["baseline_scenario_ref"] = "missing-baseline"
    if not any("unresolved distributional baseline_scenario_ref" in e for e in semantic_errors(unknown_distribution_baseline, concepts, evidence_ids)):
        fail("mutation: distributional result with unresolved baseline accepted")
    print("PASS mutation: distributional comparison requires a resolvable baseline")

    duplicate_distribution_id = copy.deepcopy(distributional_case)
    duplicated_impact = copy.deepcopy(duplicate_distribution_id["distributional_impacts"][0])
    duplicated_impact["limitations"] = ["Distinct synthetic limitation so duplicate content does not trigger JSON Schema uniqueItems."]
    duplicate_distribution_id["distributional_impacts"].append(duplicated_impact)
    if not any("distributional impact_id values must be unique" in e for e in semantic_errors(duplicate_distribution_id, concepts, evidence_ids)):
        fail("mutation: duplicate distributional impact ID accepted")
    print("PASS mutation: distributional impact receipt IDs must be unique")

    floor_breach_without_reference = copy.deepcopy(distributional_case)
    floor_breach_without_reference["distributional_impacts"][0]["constraint_status"] = "breach"
    if not any(list(error.path)[-1:] == ["constraint_ref"] for error in schema_errors(validator, floor_breach_without_reference)):
        fail("mutation: declared floor breach without a sourceable constraint reference accepted")
    print("PASS mutation: an evaluated hard-floor state requires a named constraint reference")

    real_distribution_without_evidence = copy.deepcopy(distributional_case)
    real_distribution_without_evidence["record_status"] = "assessment"
    if not any("real distributional impact requires evidence references" in e for e in semantic_errors(real_distribution_without_evidence, concepts, evidence_ids)):
        fail("mutation: quantified real-world distributional impact without evidence accepted")
    print("PASS mutation: assessed cohort-level outcomes need real evidence")

    assessment_without_coverage = copy.deepcopy(distributional_case)
    assessment_without_coverage["record_status"] = "assessment"
    assessment_without_coverage.pop("distributional_coverage")
    if not any(error.validator == "required" and "distributional_coverage" in error.message
               for error in schema_errors(validator, assessment_without_coverage)):
        fail("mutation: real assessment without a distributional coverage declaration was accepted")
    print("PASS mutation: real assessment requires an explicit distributional coverage declaration")

    group_missing_from_coverage = copy.deepcopy(distributional_case)
    group_missing_from_coverage["record_status"] = "assessment"
    group_missing_from_coverage["distributional_coverage"]["included_group_refs"] = ["different-reported-group"]
    if not any("distributional impact group is missing from coverage declaration" in e
               for e in semantic_errors(group_missing_from_coverage, concepts, evidence_ids)):
        fail("mutation: distributional outcome for an undeclared group was accepted")
    print("PASS mutation: each reported cohort must be listed in the coverage declaration")

    unresolved_coverage_evidence = copy.deepcopy(distributional_case)
    unresolved_coverage_evidence["distributional_coverage"]["evidence_refs"] = ["missing-coverage-evidence"]
    if not any("distributional coverage: unresolved evidence reference" in e
               for e in semantic_errors(unresolved_coverage_evidence, concepts, evidence_ids)):
        fail("mutation: distributional coverage with unresolved evidence was accepted")
    print("PASS mutation: distributional coverage evidence references must resolve")

    print("PASS: 57 urban-metabolism/synergy controls (48 mutation/parser rejection controls and nine positive controls); distributional outcomes remain disaggregated; coverage limits are explicit; no composite symtropy score or automatic authorization.")

if __name__ == "__main__":
    main()
