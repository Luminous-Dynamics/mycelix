use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::{env, fs, process};

const PLAN_SCHEMA: &str = include_str!("../../../docs/research/schema/CIV_IMPACT_CAUSAL_TRIAL_PLAN_V0_1.schema.json");
const ASSIGNMENT_SCHEMA: &str = include_str!("../../../docs/research/schema/CIV_IMPACT_CAUSAL_ASSIGNMENT_MANIFEST_V0_1.schema.json");
const OUTCOME_SCHEMA: &str = include_str!("../../../docs/research/schema/CIV_IMPACT_CAUSAL_OUTCOME_LEDGER_V0_1.schema.json");

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn number(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64)
}

fn integer(value: &Value, key: &str) -> Option<usize> {
    value.get(key).and_then(Value::as_u64).map(|n| n as usize)
}

fn array(value: &Value, key: &str) -> &[Value] {
    value.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}

fn nested_string<'a>(value: &'a Value, parent: &str, key: &str) -> Option<&'a str> {
    value.get(parent).and_then(|v| string(v, key))
}

fn schema_errors(value: &Value, schema_text: &str, label: &str) -> Vec<String> {
    let schema: Value = match serde_json::from_str(schema_text) {
        Ok(value) => value,
        Err(error) => return vec![format!("{label}: bundled schema JSON is invalid: {error}")],
    };
    let validator = match jsonschema::validator_for(&schema) {
        Ok(validator) => validator,
        Err(error) => return vec![format!("{label}: bundled schema did not compile: {error}")],
    };
    validator
        .iter_errors(value)
        .take(12)
        .map(|error| format!("{label}: {error}"))
        .collect()
}

fn plan_gate_errors(plan: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let design = string(plan, "design").unwrap_or("");
    let claim = string(plan, "claim_tier").unwrap_or("");
    let protocol_status = string(plan, "protocol_status").unwrap_or("");
    let assignment_method = nested_string(plan, "assignment", "method").unwrap_or("");
    let randomization_unit = nested_string(plan, "assignment", "randomization_unit").unwrap_or("");
    let estimand_type = nested_string(plan, "estimand", "type").unwrap_or("");
    let unit_of_assignment = nested_string(plan, "estimand", "unit_of_assignment").unwrap_or("");
    let primary_method = nested_string(plan, "analysis", "primary_method").unwrap_or("");
    let uncertainty_method = nested_string(plan, "analysis", "uncertainty_method").unwrap_or("");
    let interference = nested_string(plan, "interference", "assumption").unwrap_or("");
    let allocator_independent = plan
        .get("assignment")
        .and_then(|v| v.get("independent_allocator"))
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let fixed_horizon = plan
        .get("stopping")
        .and_then(|v| v.get("fixed_horizon"))
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let interim_looks = plan
        .get("stopping")
        .and_then(|v| integer(v, "interim_looks"))
        .unwrap_or(0);
    let sequential_method = nested_string(plan, "stopping", "sequential_method").unwrap_or("");
    let participant_burden_recorded = plan
        .get("cost_accounting")
        .and_then(|v| v.get("participant_burden_recorded"))
        .and_then(Value::as_bool)
        .unwrap_or(false);

    match design {
        "randomized_individual" => {
            if assignment_method != "independent_randomized" || randomization_unit != "individual" || unit_of_assignment != "individual" {
                errors.push("randomized_individual design must declare independent_randomized assignment at the individual unit".into());
            }
        }
        "randomized_cluster" => {
            if assignment_method != "cluster_randomized" || randomization_unit != "cluster" || unit_of_assignment != "cluster" {
                errors.push("randomized_cluster design must declare cluster_randomized assignment at the cluster unit".into());
            }
        }
        "observational" | "quasi_experimental" => {
            if claim == "confirmatory_causal" {
                errors.push("observational/quasi_experimental designs are descriptive-only in CIV-IMPACT v0.1".into());
            }
            if assignment_method == "independent_randomized" || assignment_method == "cluster_randomized" {
                errors.push("declared non-randomized design conflicts with a randomized assignment method".into());
            }
        }
        _ => {}
    }

    if claim == "confirmatory_causal" {
        if protocol_status != "frozen" {
            errors.push("confirmatory_causal claim requires protocol_status=frozen".into());
        }
        if !matches!(design, "randomized_individual" | "randomized_cluster") {
            errors.push("confirmatory_causal claim requires a randomized design in v0.1".into());
        }
        if estimand_type != "intention_to_treat" {
            errors.push("confirmatory_causal claim requires intention_to_treat as the primary estimand".into());
        }
        if primary_method != "intention_to_treat_difference" {
            errors.push("confirmatory_causal claim requires the preregistered intention_to_treat_difference analysis".into());
        }
        if uncertainty_method == "none" || uncertainty_method.is_empty() {
            errors.push("confirmatory_causal claim requires a preregistered uncertainty method".into());
        }
        if interference == "unknown" || interference.is_empty() {
            errors.push("confirmatory_causal claim requires a stated interference assumption and mitigation plan".into());
        }
        if !allocator_independent || string(plan.get("assignment").unwrap_or(&Value::Null), "attestation_ref").unwrap_or("").trim().is_empty() {
            errors.push("confirmatory_causal claim requires an independent allocator declaration and an attestation reference".into());
        }
        if !fixed_horizon && interim_looks > 0 && matches!(sequential_method, "" | "none" | "not_applicable") {
            errors.push("interim looks without a frozen sequential method invalidate confirmatory eligibility".into());
        }
        if !participant_burden_recorded {
            errors.push("confirmatory_causal claim requires participant burden to be recorded in the cost boundary".into());
        }
        if design == "randomized_cluster"
            && nested_string(plan, "interference", "cluster_field").unwrap_or("").trim().is_empty()
        {
            errors.push("cluster-randomized confirmatory design must declare the cluster field used for interference analysis".into());
        }
        if design == "randomized_cluster"
            && !plan.get("analysis").and_then(|v| v.get("cluster_adjustment")).and_then(Value::as_bool).unwrap_or(false)
        {
            errors.push("cluster-randomized confirmatory design requires cluster-adjusted analysis".into());
        }
    }
    let minimum = integer(plan.get("sample_plan").unwrap_or(&Value::Null), "minimum_units_per_arm").unwrap_or(0);
    let target = integer(plan.get("sample_plan").unwrap_or(&Value::Null), "target_units").unwrap_or(0);
    if target < minimum.saturating_mul(2) {
        errors.push("target_units must be at least twice minimum_units_per_arm".into());
    }
    errors
}

fn verify_trial(
    plan: &Value,
    assignment: &Value,
    outcomes: &Value,
    plan_bytes: &[u8],
    assignment_bytes: &[u8],
) -> Vec<String> {
    let mut errors = Vec::new();
    errors.extend(schema_errors(plan, PLAN_SCHEMA, "trial plan"));
    errors.extend(schema_errors(assignment, ASSIGNMENT_SCHEMA, "assignment manifest"));
    errors.extend(schema_errors(outcomes, OUTCOME_SCHEMA, "outcome ledger"));
    errors.extend(plan_gate_errors(plan));

    let trial_id = string(plan, "trial_id").unwrap_or("");
    if string(assignment, "trial_id").unwrap_or("") != trial_id {
        errors.push("assignment manifest trial_id does not match the frozen plan".into());
    }
    if string(outcomes, "trial_id").unwrap_or("") != trial_id {
        errors.push("outcome ledger trial_id does not match the frozen plan".into());
    }

    let plan_digest = sha256(plan_bytes);
    let assignment_digest = sha256(assignment_bytes);
    if string(plan, "assignment_manifest_sha256").unwrap_or("") != assignment_digest {
        errors.push("assignment manifest exact-byte SHA-256 does not match the plan commitment".into());
    }
    if string(outcomes, "trial_plan_sha256").unwrap_or("") != plan_digest {
        errors.push("outcome ledger trial_plan_sha256 does not match the exact plan bytes".into());
    }
    if string(outcomes, "assignment_manifest_sha256").unwrap_or("") != assignment_digest {
        errors.push("outcome ledger assignment_manifest_sha256 does not match the exact assignment bytes".into());
    }

    let design = string(plan, "design").unwrap_or("");
    let assignment_entries = array(assignment, "entries");
    let outcome_entries = array(outcomes, "entries");
    let mut assigned: BTreeMap<String, (String, Option<String>)> = BTreeMap::new();
    let mut arm_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut clusters_by_arm: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut cluster_assignment: BTreeMap<String, String> = BTreeMap::new();
    let mut assignment_receipt_ids = BTreeSet::new();

    for entry in assignment_entries {
        let Some(unit_id) = string(entry, "unit_id") else {
            errors.push("assignment entry lacks unit_id".into());
            continue;
        };
        if let Some(receipt_id) = string(entry, "assignment_receipt_id") {
            if !assignment_receipt_ids.insert(receipt_id.to_owned()) {
                errors.push(format!("duplicate assignment_receipt_id: {receipt_id}"));
            }
        }
        let arm = string(entry, "assigned_arm").unwrap_or("").to_owned();
        let cluster = entry.get("cluster_id").and_then(Value::as_str).map(str::to_owned);
        if assigned.insert(unit_id.to_owned(), (arm.clone(), cluster.clone())).is_some() {
            errors.push(format!("duplicate assigned unit_id: {unit_id}"));
            continue;
        }
        *arm_counts.entry(arm.clone()).or_default() += 1;
        if let Some(cluster_id) = cluster {
            clusters_by_arm.entry(arm.clone()).or_default().insert(cluster_id.clone());
            if design == "randomized_cluster" {
                if let Some(previous_arm) = cluster_assignment.insert(cluster_id.clone(), arm.clone()) {
                    if previous_arm != arm {
                        errors.push(format!("cluster {cluster_id} appears in both assignment arms"));
                    }
                }
            }
        } else if design == "randomized_cluster" {
            errors.push(format!("cluster-randomized assignment for unit {unit_id} lacks cluster_id"));
        }
    }

    let minimum_units = integer(plan.get("sample_plan").unwrap_or(&Value::Null), "minimum_units_per_arm").unwrap_or(0);
    for arm in ["intervention", "comparator"] {
        let count = *arm_counts.get(arm).unwrap_or(&0);
        if count < minimum_units {
            errors.push(format!("arm {arm} has {count} assigned units; preregistered minimum is {minimum_units}"));
        }
    }
    if design == "randomized_cluster" {
        let minimum_clusters = integer(plan.get("sample_plan").unwrap_or(&Value::Null), "minimum_clusters_per_arm").unwrap_or(0);
        for arm in ["intervention", "comparator"] {
            let count = clusters_by_arm.get(arm).map(BTreeSet::len).unwrap_or(0);
            if count < minimum_clusters {
                errors.push(format!("arm {arm} has {count} clusters; preregistered minimum is {minimum_clusters}"));
            }
        }
    }

    let required_harm_ids: BTreeSet<String> = plan
        .get("estimand")
        .and_then(|v| v.get("harm_outcomes"))
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|item| string(item, "id"))
        .map(str::to_owned)
        .collect();
    let mut seen_outcomes = BTreeSet::new();
    let mut missing_by_arm: BTreeMap<String, usize> = BTreeMap::new();
    let mut observed_by_arm: BTreeMap<String, usize> = BTreeMap::new();
    for row in outcome_entries {
        let Some(unit_id) = string(row, "unit_id") else {
            errors.push("outcome entry lacks unit_id".into());
            continue;
        };
        if !seen_outcomes.insert(unit_id.to_owned()) {
            errors.push(format!("duplicate outcome unit_id: {unit_id}"));
            continue;
        }
        let Some((arm, _)) = assigned.get(unit_id) else {
            errors.push(format!("outcome exists for unassigned unit_id: {unit_id}"));
            continue;
        };
        let harm_record = row.get("harm_outcomes").and_then(Value::as_object);
        for harm_id in &required_harm_ids {
            if !harm_record.map(|record| record.contains_key(harm_id)).unwrap_or(false) {
                errors.push(format!("outcome unit {unit_id} omits predeclared harm outcome {harm_id}"));
            }
        }
        let status = string(row, "outcome_status").unwrap_or("");
        let value_is_null = row.get("primary_outcome_value").map(Value::is_null).unwrap_or(true);
        if status == "observed" {
            if value_is_null {
                errors.push(format!("observed unit {unit_id} has null primary_outcome_value"));
            }
            *observed_by_arm.entry(arm.clone()).or_default() += 1;
        } else {
            if !value_is_null {
                errors.push(format!("non-observed unit {unit_id} must have null primary_outcome_value"));
            }
            *missing_by_arm.entry(arm.clone()).or_default() += 1;
        }
    }

    for unit_id in assigned.keys() {
        if !seen_outcomes.contains(unit_id) {
            errors.push(format!("assigned unit {unit_id} has no outcome/disposition row"));
        }
    }
    if seen_outcomes.len() != assigned.len() {
        // More specific unknown/duplicate errors are emitted above; this guards future schema changes.
        errors.push("outcome census size differs from the assignment census".into());
    }

    let missing_ceiling = number(
        plan.get("missing_data").unwrap_or(&Value::Null),
        "maximum_missing_fraction_per_arm",
    ).unwrap_or(1.0);
    for arm in ["intervention", "comparator"] {
        let assigned_count = *arm_counts.get(arm).unwrap_or(&0);
        let missing_count = *missing_by_arm.get(arm).unwrap_or(&0);
        let observed_count = *observed_by_arm.get(arm).unwrap_or(&0);
        if assigned_count > 0 {
            let fraction = missing_count as f64 / assigned_count as f64;
            if fraction > missing_ceiling {
                errors.push(format!(
                    "arm {arm} missing fraction {fraction:.4} exceeds preregistered maximum {missing_ceiling:.4}"
                ));
            }
        }
        if observed_count + missing_count != assigned_count {
            errors.push(format!("arm {arm} outcome dispositions do not reconcile to assigned units"));
        }
    }
    errors
}

fn read_json(path: &str) -> Result<(Vec<u8>, Value), String> {
    let bytes = fs::read(path).map_err(|error| format!("cannot read {path}: {error}"))?;
    let value = serde_json::from_slice(&bytes).map_err(|error| format!("{path}: invalid JSON: {error}"))?;
    Ok((bytes, value))
}

fn usage() -> &'static str {
    "Usage:\n  civ-impact-causal-field validate-plan <trial-plan.json>\n  civ-impact-causal-field verify-trial <trial-plan.json> <assignment-manifest.json> <outcome-ledger.json>\n\nArtifact consistency is not proof of genuine randomization or a causal effect."
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("--help" | "-h") | None => {
            println!("{}", usage());
            Ok(())
        }
        Some("validate-plan") if args.len() == 2 => {
            let (bytes, plan) = read_json(&args[1])?;
            let mut errors = schema_errors(&plan, PLAN_SCHEMA, "trial plan");
            errors.extend(plan_gate_errors(&plan));
            if !errors.is_empty() {
                return Err(errors.join("\n"));
            }
            println!("{}", json!({
                "artifact_status": "plan_schema_and_design_gates_consistent",
                "trial_id": string(&plan, "trial_id").unwrap_or(""),
                "claim_tier": string(&plan, "claim_tier").unwrap_or(""),
                "trial_plan_sha256": sha256(&bytes),
                "claim_ceiling": if string(&plan, "claim_tier") == Some("confirmatory_causal") {
                    "eligible_for_independent_confirmatory_review_only"
                } else {
                    "descriptive_only"
                },
                "not_proven": ["allocation authenticity", "outcome validity", "causal effect", "statistical significance"]
            }));
            Ok(())
        }
        Some("verify-trial") if args.len() == 4 => {
            let (plan_bytes, plan) = read_json(&args[1])?;
            let (assignment_bytes, assignment) = read_json(&args[2])?;
            let (outcome_bytes, outcomes) = read_json(&args[3])?;
            let errors = verify_trial(&plan, &assignment, &outcomes, &plan_bytes, &assignment_bytes);
            if !errors.is_empty() {
                return Err(errors.join("\n"));
            }
            let counts = |arm: &str| {
                let assigned = array(&assignment, "entries").iter()
                    .filter(|row| string(row, "assigned_arm") == Some(arm)).count();
                let observed = array(&outcomes, "entries").iter()
                    .filter(|row| {
                        string(row, "outcome_status") == Some("observed")
                            && string_for_unit(&assignment, string(row, "unit_id").unwrap_or("")) == Some(arm)
                    }).count();
                json!({"assigned": assigned, "observed": observed, "missing_or_not_observed": assigned.saturating_sub(observed)})
            };
            let claim_tier = string(&plan, "claim_tier").unwrap_or("descriptive");
            println!("{}", json!({
                "artifact_status": "internally_consistent",
                "trial_id": string(&plan, "trial_id").unwrap_or(""),
                "claim_tier_requested": claim_tier,
                "review_gate_status": if claim_tier == "confirmatory_causal" {
                    "eligible_for_independent_confirmatory_review_only"
                } else {
                    "descriptive_only"
                },
                "trial_plan_sha256": sha256(&plan_bytes),
                "assignment_manifest_sha256": sha256(&assignment_bytes),
                "outcome_ledger_sha256": sha256(&outcome_bytes),
                "outcome_census": {
                    "intervention": counts("intervention"),
                    "comparator": counts("comparator"),
                    "total_assigned_units": array(&assignment, "entries").len(),
                    "total_outcome_rows": array(&outcomes, "entries").len()
                },
                "claim_ceiling": "No treatment effect is estimated. Internal artifact consistency is not proof of randomization, valid measurement, absence of confounding/interference, or causal impact.",
                "not_proven": ["allocation authenticity", "signature provenance or external freeze anchoring", "outcome validity", "causal effect", "statistical significance", "transportability"]
            }));
            Ok(())
        }
        _ => Err(usage().to_owned()),
    }
}

fn string_for_unit<'a>(assignment: &'a Value, unit_id: &str) -> Option<&'a str> {
    array(assignment, "entries").iter().find(|row| string(row, "unit_id") == Some(unit_id))
        .and_then(|row| string(row, "assigned_arm"))
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Fixture {
        plan: Value,
        assignment: Value,
        outcomes: Value,
        plan_bytes: Vec<u8>,
        assignment_bytes: Vec<u8>,
    }

    impl Fixture {
        fn new() -> Self {
            let assignment = json!({
                "schema_version": "civ-impact-causal-assignment-manifest-v0.1",
                "trial_id": "trial-demo-001",
                "entries": [
                    {"unit_id":"u1","assigned_arm":"intervention","cluster_id":null,"assignment_receipt_id":"receipt-u1","assignment_time":"2026-10-01T10:00:00Z"},
                    {"unit_id":"u2","assigned_arm":"intervention","cluster_id":null,"assignment_receipt_id":"receipt-u2","assignment_time":"2026-10-01T10:01:00Z"},
                    {"unit_id":"u3","assigned_arm":"comparator","cluster_id":null,"assignment_receipt_id":"receipt-u3","assignment_time":"2026-10-01T10:02:00Z"},
                    {"unit_id":"u4","assigned_arm":"comparator","cluster_id":null,"assignment_receipt_id":"receipt-u4","assignment_time":"2026-10-01T10:03:00Z"}
                ]
            });
            let assignment_bytes = serde_json::to_vec(&assignment).unwrap();
            let assignment_sha = sha256(&assignment_bytes);
            let plan = json!({
                "schema_version":"civ-impact-causal-trial-plan-v0.1",
                "trial_id":"trial-demo-001",
                "protocol_status":"frozen",
                "design":"randomized_individual",
                "claim_tier":"confirmatory_causal",
                "assignment_manifest_sha256":assignment_sha,
                "intervention":{"id":"system-v1","version":"1.0.0","description":"Registered intervention"},
                "comparator":{"id":"baseline-v1","version":"1.0.0","description":"Registered comparator"},
                "estimand":{
                    "type":"intention_to_treat","target_population":"Synthetic fixture units",
                    "unit_of_assignment":"individual","unit_of_analysis":"individual",
                    "primary_outcome":{"id":"served_units","scale":"continuous","direction":"higher_is_better","operational_definition":"Synthetic observed value for contract test"},
                    "secondary_outcomes":[],
                    "harm_outcomes":[{"id":"harm_flag","scale":"binary","direction":"harm_signal","operational_definition":"Synthetic harm flag"}],
                    "observation_window":"Fixed synthetic window"
                },
                "assignment":{
                    "method":"independent_randomized","randomization_unit":"individual",
                    "allocation_ratio":{"intervention":1,"comparator":1},"stratification_fields":[],
                    "independent_allocator":true,"attestation_ref":"synthetic-review-reference"
                },
                "sample_plan":{
                    "minimum_units_per_arm":2,"minimum_clusters_per_arm":2,"target_units":4,
                    "sample_size_justification":"Only a contract smoke fixture; not a power calculation."
                },
                "analysis":{
                    "primary_method":"intention_to_treat_difference","uncertainty_method":"randomization_inference",
                    "cluster_adjustment":false,"multiplicity_policy":"One registered primary outcome",
                    "effect_size_threshold":null
                },
                "interference":{"assumption":"no_interference_assumed","cluster_field":null,"risk_and_mitigation":"Synthetic fixture; no external units."},
                "missing_data":{"policy":"Retain all assignments with explicit disposition","missing_outcomes_remain_in_denominator":true,"maximum_missing_fraction_per_arm":0.25,"withdrawal_handling":"Record as withdrawn, not as a zero outcome."},
                "stopping":{"fixed_horizon":true,"interim_looks":0,"sequential_method":"not_applicable","safety_stop_rule":"Stop on any configured harm threshold."},
                "ethics_privacy":{"consent_or_authorized_basis":"Synthetic fixtures only","review_ref":"not-a-human-study","data_minimization":"No direct identifiers","withdrawal_and_appeal":"No real participants."},
                "cost_accounting":{"currency_or_unit":"synthetic microcredits","included_cost_categories":["compute","operations","integration","human_review","failure_recovery"],"participant_burden_recorded":true}
            });
            let plan_bytes = serde_json::to_vec(&plan).unwrap();
            let outcomes = json!({
                "schema_version":"civ-impact-causal-outcome-ledger-v0.1",
                "trial_id":"trial-demo-001",
                "trial_plan_sha256":sha256(&plan_bytes),
                "assignment_manifest_sha256":assignment_sha,
                "entries":[
                    {"unit_id":"u1","outcome_status":"observed","primary_outcome_value":12.0,"harm_outcomes":{"harm_flag":false},"noncompliance":false,"deviation_codes":[]},
                    {"unit_id":"u2","outcome_status":"observed","primary_outcome_value":10.0,"harm_outcomes":{"harm_flag":false},"noncompliance":false,"deviation_codes":[]},
                    {"unit_id":"u3","outcome_status":"observed","primary_outcome_value":8.0,"harm_outcomes":{"harm_flag":false},"noncompliance":false,"deviation_codes":[]},
                    {"unit_id":"u4","outcome_status":"observed","primary_outcome_value":9.0,"harm_outcomes":{"harm_flag":false},"noncompliance":false,"deviation_codes":[]}
                ]
            });
            Self { plan, assignment, outcomes, plan_bytes, assignment_bytes }
        }

        fn refresh_bindings(&mut self) {
            self.assignment_bytes = serde_json::to_vec(&self.assignment).unwrap();
            self.plan["assignment_manifest_sha256"] = Value::String(sha256(&self.assignment_bytes));
            self.plan_bytes = serde_json::to_vec(&self.plan).unwrap();
            self.outcomes["trial_plan_sha256"] = Value::String(sha256(&self.plan_bytes));
            self.outcomes["assignment_manifest_sha256"] = Value::String(sha256(&self.assignment_bytes));
        }

        fn errors(&self) -> Vec<String> {
            verify_trial(&self.plan, &self.assignment, &self.outcomes, &self.plan_bytes, &self.assignment_bytes)
        }
    }

    #[test]
    fn all_bundled_schemas_compile() {
        for (schema, label) in [
            (PLAN_SCHEMA, "plan"),
            (ASSIGNMENT_SCHEMA, "assignment"),
            (OUTCOME_SCHEMA, "outcome"),
        ] {
            let value: Value = serde_json::from_str(schema).unwrap();
            assert!(jsonschema::validator_for(&value).is_ok(), "{label} schema failed to compile");
        }
    }

    #[test]
    fn valid_fixture_is_artifact_consistent_not_a_causal_result() {
        let fixture = Fixture::new();
        assert!(fixture.errors().is_empty(), "{:?}", fixture.errors());
    }

    #[test]
    fn assignment_bytes_must_match_the_plan_commitment() {
        let mut fixture = Fixture::new();
        fixture.assignment_bytes.push(b' ');
        let errors = verify_trial(&fixture.plan, &fixture.assignment, &fixture.outcomes, &fixture.plan_bytes, &fixture.assignment_bytes);
        assert!(errors.iter().any(|error| error.contains("exact-byte SHA-256")));
    }

    #[test]
    fn assignment_receipts_cannot_be_reused_across_units() {
        let mut fixture = Fixture::new();
        fixture.assignment["entries"][1]["assignment_receipt_id"] = json!("receipt-u1");
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("duplicate assignment_receipt_id")));
    }

    #[test]
    fn every_registered_harm_outcome_must_be_present_even_when_unknown() {
        let mut fixture = Fixture::new();
        fixture.outcomes["entries"][0]["harm_outcomes"].as_object_mut().unwrap().remove("harm_flag");
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("omits predeclared harm outcome")));
    }

    #[test]
    fn duplicate_assigned_units_are_rejected() {
        let mut fixture = Fixture::new();
        fixture.assignment["entries"][3]["unit_id"] = json!("u1");
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("duplicate assigned unit_id")));
    }

    #[test]
    fn missing_outcome_row_is_rejected_not_dropped() {
        let mut fixture = Fixture::new();
        fixture.outcomes["entries"].as_array_mut().unwrap().pop();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("has no outcome/disposition row")));
    }

    #[test]
    fn outcome_for_unassigned_unit_is_rejected() {
        let mut fixture = Fixture::new();
        fixture.outcomes["entries"][3]["unit_id"] = json!("ghost");
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("outcome exists for unassigned unit_id")));
    }

    #[test]
    fn observed_outcome_requires_a_value() {
        let mut fixture = Fixture::new();
        fixture.outcomes["entries"][0]["primary_outcome_value"] = Value::Null;
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("observed unit u1 has null")));
    }

    #[test]
    fn missing_outcome_must_not_be_encoded_as_zero() {
        let mut fixture = Fixture::new();
        fixture.outcomes["entries"][0]["outcome_status"] = json!("missing");
        fixture.outcomes["entries"][0]["primary_outcome_value"] = json!(0);
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("non-observed unit u1 must have null")));
    }

    #[test]
    fn observational_design_cannot_claim_confirmatory_causality() {
        let mut fixture = Fixture::new();
        fixture.plan["design"] = json!("observational");
        fixture.plan["assignment"]["method"] = json!("nonrandomized");
        fixture.plan["assignment"]["randomization_unit"] = json!("none");
        fixture.plan["claim_tier"] = json!("confirmatory_causal");
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("descriptive-only")));
    }

    #[test]
    fn unknown_interference_blocks_confirmatory_review() {
        let mut fixture = Fixture::new();
        fixture.plan["interference"]["assumption"] = json!("unknown");
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("interference assumption")));
    }

    #[test]
    fn missingness_ceiling_is_checked_per_arm() {
        let mut fixture = Fixture::new();
        fixture.plan["missing_data"]["maximum_missing_fraction_per_arm"] = json!(0.0);
        fixture.outcomes["entries"][0]["outcome_status"] = json!("withdrawn");
        fixture.outcomes["entries"][0]["primary_outcome_value"] = Value::Null;
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("missing fraction")));
    }

    #[test]
    fn outcome_ledger_must_bind_exact_plan_bytes() {
        let mut fixture = Fixture::new();
        fixture.outcomes["trial_plan_sha256"] = json!("a".repeat(64));
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("trial_plan_sha256")));
    }

    #[test]
    fn cluster_randomization_rejects_cluster_leakage_across_arms() {
        let mut fixture = Fixture::new();
        fixture.plan["design"] = json!("randomized_cluster");
        fixture.plan["assignment"]["method"] = json!("cluster_randomized");
        fixture.plan["assignment"]["randomization_unit"] = json!("cluster");
        fixture.plan["estimand"]["unit_of_assignment"] = json!("cluster");
        fixture.plan["analysis"]["cluster_adjustment"] = json!(true);
        fixture.plan["interference"]["cluster_field"] = json!("cluster_id");
        fixture.plan["sample_plan"]["minimum_clusters_per_arm"] = json!(2);
        fixture.assignment["entries"][0]["cluster_id"] = json!("c1");
        fixture.assignment["entries"][1]["cluster_id"] = json!("c2");
        fixture.assignment["entries"][2]["cluster_id"] = json!("c1");
        fixture.assignment["entries"][3]["cluster_id"] = json!("c3");
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("appears in both assignment arms")));
    }

    #[test]
    fn frozen_plan_byte_change_requires_new_outcome_binding() {
        let mut fixture = Fixture::new();
        fixture.plan["analysis"]["multiplicity_policy"] = json!("changed after freeze");
        fixture.plan_bytes = serde_json::to_vec(&fixture.plan).unwrap();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("trial_plan_sha256")));
    }

    #[test]
    fn insufficient_per_arm_sample_is_rejected() {
        let mut fixture = Fixture::new();
        fixture.assignment["entries"].as_array_mut().unwrap().pop();
        fixture.refresh_bindings();
        let errors = fixture.errors();
        assert!(errors.iter().any(|error| error.contains("preregistered minimum")));
    }
}
