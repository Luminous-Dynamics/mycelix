//! Semantic verifier for CIV-IMPACT adaptive challenge preregistrations and exposure ledgers.
//!
//! This tool verifies contracts and campaign sequencing. It is not a sealed-bank
//! secret manager, a benchmark selector, or proof of real-world performance.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

const CLAIM_CEILING: &str =
    "confirmatory_claim_limited_to_frozen_candidate_comparator_evaluator_and_declared_challenge_domain";
const DESCRIPTIVE_CLAIM_CEILING: &str =
    "descriptive_selected_set_only; no_population_inference_or_confirmatory_claim";

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Ledger {
    schema_version: String,
    campaign_id: String,
    preregistration_receipt_sha256: String,
    initial_sealed_bank_commitment_sha256: String,
    event_count: usize,
    cumulative_evaluation_cost_microcredits: u64,
    events: Vec<Event>,
    campaign_disposition: String,
    confirmatory_claim_valid: bool,
    claim_ceiling: String,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Event {
    sequence: usize,
    event_type: String,
    recorded_at: String,
    actor_role: String,
    previous_event_sha256: Option<String>,
    event_sha256: String,
    preregistration_sha256_at_event: String,
    candidate_artifact_sha256_at_event: String,
    comparator_artifact_sha256_at_event: String,
    evaluator_profile_sha256_at_event: String,
    scorer_profile_sha256_at_event: String,
    eligible_case_decisions: Vec<SelectorDecision>,
    selected_case_commitments: Vec<String>,
    result_frozen: bool,
    result_disclosed_to_candidate: bool,
    material_mutation_since_freeze: bool,
    protocol_invalidated: bool,
    reason: String,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SelectorDecision {
    challenge_commitment_sha256: String,
    regime: String,
    eligible: bool,
    selected: bool,
    selection_probability_permille: Option<u16>,
    selector_score_microcredits: Option<i64>,
    decision_reason_code: String,
}

#[derive(Debug, serde::Serialize)]
struct VerificationReport {
    status: String,
    campaign_id: String,
    event_count: usize,
    selected_challenge_count: usize,
    executed_challenge_count: usize,
    frozen_result_count: usize,
    disclosed_result_count: usize,
    rotated_challenge_count: usize,
    cumulative_evaluation_cost_microcredits: u64,
    post_freeze_mutation_detected: bool,
    confirmatory_claim_valid: bool,
    claim_ceiling: String,
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn schema_path(name: &str) -> PathBuf {
    repo_root().join("docs/research/schema").join(name)
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

fn canonical_json(value: &Value) -> Result<Vec<u8>, Box<dyn Error>> {
    // serde_json::Map preserves lexicographic object-key ordering unless the
    // preserve_order feature is enabled. Arrays retain their declared order.
    // This project's v0.1 hash convention is compact serde_json serialization
    // of the event core after parsing, excluding the event_sha256 property.
    Ok(serde_json::to_vec(value)?)
}

fn event_digest(event: &Value) -> Result<String, Box<dyn Error>> {
    let mut core = event.clone();
    let object = core
        .as_object_mut()
        .ok_or("exposure ledger event must be a JSON object")?;
    object.remove("event_sha256");
    Ok(sha256_hex(&canonical_json(&core)?))
}

fn validator_for_file(path: &Path) -> Result<jsonschema::Validator, Box<dyn Error>> {
    let schema_bytes = fs::read(path)?;
    let schema: Value = serde_json::from_slice(&schema_bytes)?;
    Ok(jsonschema::validator_for(&schema)?)
}

fn validate_instance(schema_path: &Path, instance: &Value) -> Result<(), Box<dyn Error>> {
    let validator = validator_for_file(schema_path)?;
    let errors = validator
        .iter_errors(instance)
        .map(|e| e.to_string())
        .collect::<Vec<_>>();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "schema validation failed for {}: {}",
            schema_path.display(),
            errors.join("; ")
        )
        .into())
    }
}

fn require_sha256(value: &str, name: &str) -> Result<(), Box<dyn Error>> {
    if value.len() == 64 && value.bytes().all(|b| b.is_ascii_hexdigit()) {
        Ok(())
    } else {
        Err(format!("{name} must be a 64-character hexadecimal SHA-256").into())
    }
}

fn sorted_unique(values: &[String], name: &str) -> Result<BTreeSet<String>, Box<dyn Error>> {
    let result = values.iter().cloned().collect::<BTreeSet<_>>();
    if result.len() != values.len() {
        return Err(format!("{name} contains duplicate challenge commitments").into());
    }
    Ok(result)
}

fn semantic_verify_ledger(value: &Value) -> Result<VerificationReport, Box<dyn Error>> {
    let ledger: Ledger = serde_json::from_value(value.clone())?;
    if ledger.schema_version != "0.1" {
        return Err("unsupported exposure-ledger schema version".into());
    }
    if ledger.claim_ceiling != CLAIM_CEILING
        && ledger.claim_ceiling != DESCRIPTIVE_CLAIM_CEILING
    {
        return Err("claim ceiling does not match a supported protocol profile".into());
    }
    require_sha256(
        &ledger.preregistration_receipt_sha256,
        "preregistration_receipt_sha256",
    )?;
    require_sha256(
        &ledger.initial_sealed_bank_commitment_sha256,
        "initial_sealed_bank_commitment_sha256",
    )?;
    if ledger.event_count != ledger.events.len() {
        return Err(format!(
            "event_count={} does not equal events.len()={}",
            ledger.event_count,
            ledger.events.len()
        )
        .into());
    }
    if ledger.events.is_empty() {
        return Err("exposure ledger has no campaign_frozen event".into());
    }

    let first = &ledger.events[0];
    if first.event_type != "campaign_frozen" || first.sequence != 0 {
        return Err("first ledger event must be sequence-zero campaign_frozen".into());
    }
    if first.previous_event_sha256.is_some() {
        return Err("first event must have null previous_event_sha256".into());
    }
    if first.preregistration_sha256_at_event != ledger.preregistration_receipt_sha256 {
        return Err("campaign_frozen event does not bind the ledger preregistration receipt".into());
    }

    let mut previous_hash: Option<String> = None;
    let mut sealed_identity: Option<(String, String, String, String, String)> = None;
    let mut considered_challenges = BTreeSet::<String>::new();
    let mut frozen = BTreeSet::<String>::new();
    let mut selected = BTreeSet::<String>::new();
    let mut executed = BTreeSet::<String>::new();
    let mut results_frozen = BTreeSet::<String>::new();
    let mut disclosed = BTreeSet::<String>::new();
    let mut rotated = BTreeSet::<String>::new();
    let mut mutation_seen = false;
    let mut invalidation_seen = false;
    let mut terminal_event_type: Option<String> = None;

    for (expected_sequence, event) in ledger.events.iter().enumerate() {
        if event.sequence != expected_sequence {
            return Err(format!(
                "event sequence gap: expected {expected_sequence}, received {}",
                event.sequence
            )
            .into());
        }
        if terminal_event_type.is_some() {
            return Err("events appear after campaign_closed/campaign_abandoned".into());
        }
        if event.previous_event_sha256 != previous_hash {
            return Err(format!(
                "event {} has a broken previous_event_sha256 link",
                event.sequence
            )
            .into());
        }
        require_sha256(&event.event_sha256, "event_sha256")?;
        let event_value = serde_json::to_value(event)?;
        let calculated_digest = event_digest(&event_value)?;
        if calculated_digest != event.event_sha256 {
            return Err(format!(
                "event {} digest mismatch: expected {}, calculated {}",
                event.sequence, event.event_sha256, calculated_digest
            )
            .into());
        }
        if event.recorded_at.is_empty() || event.actor_role.is_empty() || event.reason.is_empty() {
            return Err(format!("event {} has empty audit metadata", event.sequence).into());
        }

        let identity = (
            event.preregistration_sha256_at_event.clone(),
            event.candidate_artifact_sha256_at_event.clone(),
            event.comparator_artifact_sha256_at_event.clone(),
            event.evaluator_profile_sha256_at_event.clone(),
            event.scorer_profile_sha256_at_event.clone(),
        );
        for (value, field) in [
            (&identity.0, "preregistration"),
            (&identity.1, "candidate artifact"),
            (&identity.2, "comparator artifact"),
            (&identity.3, "evaluator profile"),
            (&identity.4, "scorer profile"),
        ] {
            require_sha256(value, field)?;
        }

        if sealed_identity.is_none() {
            sealed_identity = Some(identity.clone());
        } else if sealed_identity
            .as_ref()
            .is_some_and(|frozen_identity| frozen_identity != &identity)
        {
            if !event.material_mutation_since_freeze || !event.protocol_invalidated {
                return Err(format!(
                    "event {} changes a frozen identity without explicit mutation/invalidation",
                    event.sequence
                )
                .into());
            }
            // The campaign is already invalidated, but the ledger still records
            // which identity was actually present after the detected mutation.
            sealed_identity = Some(identity.clone());
        }

        if event.protocol_invalidated {
            invalidation_seen = true;
        }
        if event.material_mutation_since_freeze {
            mutation_seen = true;
            if !event.protocol_invalidated {
                return Err(format!(
                    "event {} records a material post-freeze mutation without invalidation",
                    event.sequence
                )
                .into());
            }
        }
        if event.event_type == "mutation_detected"
            && (!event.material_mutation_since_freeze || !event.protocol_invalidated)
        {
            return Err(format!(
                "mutation_detected event {} must invalidate the protocol",
                event.sequence
            )
            .into());
        }

        let event_cases = sorted_unique(
            &event.selected_case_commitments,
            "selected_case_commitments",
        )?;
        for case in &event.selected_case_commitments {
            require_sha256(case, "challenge commitment")?;
        }

        if event.event_type == "selection_batch" {
            let mut selected_by_selector = BTreeSet::new();
            let mut seen_in_batch = BTreeSet::new();
            for decision in &event.eligible_case_decisions {
                if !seen_in_batch.insert(decision.challenge_commitment_sha256.clone()) {
                    return Err(format!(
                        "event {} lists the same eligible challenge more than once",
                        event.sequence
                    ).into());
                }
                require_sha256(
                    &decision.challenge_commitment_sha256,
                    "selector challenge commitment",
                )?;
                if !considered_challenges.insert(decision.challenge_commitment_sha256.clone()) {
                    return Err(format!(
                        "challenge {} appears in more than one selection batch or is duplicated",
                        decision.challenge_commitment_sha256
                    ).into());
                }
                if decision.selected {
                    if !decision.eligible {
                        return Err(format!(
                            "event {} selected an ineligible challenge",
                            event.sequence
                        )
                        .into());
                    }
                    let probability = decision.selection_probability_permille
                        .ok_or("selected challenge lacks selection probability")?;
                    if probability == 0 || probability > 1000 {
                        return Err("selected challenge probability must be 1..=1000 permille".into());
                    }
                    if !matches!(
                        decision.decision_reason_code.as_str(),
                        "selected_by_frozen_selector"
                            | "selected_for_minimum_regime_coverage"
                            | "selected_for_random_audit"
                    ) {
                        return Err("selected challenge has an invalid selection reason".into());
                    }
                    selected_by_selector.insert(decision.challenge_commitment_sha256.clone());
                } else if decision.eligible {
                    if decision.decision_reason_code != "not_selected_by_frozen_selector" {
                        return Err(format!(
                            "eligible unselected challenge at event {} has inconsistent reason code",
                            event.sequence
                        ).into());
                    }
                } else {
                    if decision.selection_probability_permille.is_some()
                        || !matches!(
                            decision.decision_reason_code.as_str(),
                            "ineligible_by_preregistered_rule" | "rejected_duplicate_or_replay"
                        )
                    {
                        return Err(format!(
                            "ineligible challenge at event {} has inconsistent probability or reason",
                            event.sequence
                        ).into());
                    }
                }
                // Consume fields so accidental omission from the audit contract
                // is visible during compiler review.
                let _ = (&decision.regime, decision.selector_score_microcredits);
            }
            if selected_by_selector != event_cases {
                return Err(format!(
                    "event {} selected_case_commitments do not match selector decisions",
                    event.sequence
                )
                .into());
            }
            for case in &event_cases {
                if !selected.insert(case.clone()) {
                    return Err(format!("challenge {} was selected more than once", case).into());
                }
            }
        } else if !event.eligible_case_decisions.is_empty() {
            return Err(format!(
                "only selection_batch events may include eligible_case_decisions (event {})",
                event.sequence
            )
            .into());
        }

        match event.event_type.as_str() {
            "campaign_frozen" => {
                if event.sequence != 0 {
                    return Err("campaign_frozen can appear only as first event".into());
                }
            }
            "selection_batch" => {
                if event_cases.is_empty() {
                    return Err("selection_batch must account for at least one selected or evaluated decision".into());
                }
            }
            "challenge_executed" => {
                if event_cases.is_empty() {
                    return Err("challenge_executed must identify the executed challenge".into());
                }
                for case in &event_cases {
                    if !selected.contains(case) || !executed.insert(case.clone()) {
                        return Err(format!("challenge {case} executed before selection or more than once").into());
                    }
                }
            }
            "result_frozen" => {
                if event_cases.is_empty() {
                    return Err("result_frozen must identify its challenge".into());
                }
                if !event.result_frozen || event.result_disclosed_to_candidate {
                    return Err("result_frozen event must freeze without disclosure".into());
                }
                for case in &event_cases {
                    if !executed.contains(case) || !results_frozen.insert(case.clone()) {
                        return Err(format!("result {case} frozen before execution or more than once").into());
                    }
                }
            }
            "result_disclosed" => {
                if event_cases.is_empty() {
                    return Err("result_disclosed must identify its challenge".into());
                }
                if !event.result_frozen || !event.result_disclosed_to_candidate {
                    return Err("result_disclosed event must mark a previously frozen result as disclosed".into());
                }
                for case in &event_cases {
                    if !results_frozen.contains(case) || !disclosed.insert(case.clone()) {
                        return Err(format!("result {case} disclosed before freeze or more than once").into());
                    }
                }
            }
            "challenge_rotated" => {
                if event_cases.is_empty() {
                    return Err("challenge_rotated must identify the challenge".into());
                }
                for case in &event_cases {
                    if !disclosed.contains(case) || !rotated.insert(case.clone()) {
                        return Err(format!("challenge {case} rotated before disclosure or more than once").into());
                    }
                }
            }
            "campaign_closed" => {
                terminal_event_type = Some("campaign_closed".to_string());
                if expected_sequence + 1 != ledger.events.len() {
                    return Err("campaign_closed must be the final ledger event".into());
                }
            }
            "campaign_abandoned" => {
                terminal_event_type = Some("campaign_abandoned".to_string());
                if expected_sequence + 1 != ledger.events.len() {
                    return Err("campaign_abandoned must be the final ledger event".into());
                }
            }
            "mutation_detected" => {}
            other => return Err(format!("unknown event_type {other:?}").into()),
        }

        if event.result_disclosed_to_candidate && !event.result_frozen {
            return Err(format!(
                "event {} discloses a result without result_frozen=true",
                event.sequence
            )
            .into());
        }
        previous_hash = Some(event.event_sha256.clone());
    }

    if matches!(
        ledger.campaign_disposition.as_str(),
        "confirmatory_complete" | "descriptive_complete"
    ) {
        if mutation_seen {
            return Err("completed campaign has a post-freeze mutation".into());
        }
        if terminal_event_type.as_deref() != Some("campaign_closed") {
            return Err("completed campaign must end with campaign_closed".into());
        }
        if invalidation_seen {
            return Err("completed campaign contains a protocol invalidation event".into());
        }
        if selected != executed || selected != results_frozen || selected != disclosed || selected != rotated {
            return Err("completed campaign has unexecuted, unfrozen, undisclosed, or unrotated selections".into());
        }
        if ledger.campaign_disposition == "confirmatory_complete" && !ledger.confirmatory_claim_valid {
            return Err("confirmatory_complete campaign has confirmatory_claim_valid=false".into());
        }
        if ledger.campaign_disposition == "descriptive_complete" && ledger.confirmatory_claim_valid {
            return Err("descriptive_complete campaign cannot carry a confirmatory claim".into());
        }
    } else if ledger.confirmatory_claim_valid {
        return Err("only a fully complete, non-invalidated campaign may carry a confirmatory claim".into());
    }

    if mutation_seen && ledger.campaign_disposition != "invalidated" {
        return Err("campaign with post-freeze mutation must be invalidated".into());
    }
    if ledger.campaign_disposition == "invalidated" && !invalidation_seen {
        return Err("invalidated campaign lacks an explicit protocol_invalidated event".into());
    }
    if ledger.campaign_disposition == "abandoned"
        && terminal_event_type.as_deref() != Some("campaign_abandoned")
    {
        return Err("abandoned campaign must end with campaign_abandoned".into());
    }
    if ledger.campaign_disposition == "sealed_active" && terminal_event_type.is_some() {
        return Err("sealed_active campaign cannot have a terminal event".into());
    }
    if ledger.campaign_disposition == "invalidated" && ledger.confirmatory_claim_valid {
        return Err("invalidated campaign cannot retain a confirmatory claim".into());
    }

    Ok(VerificationReport {
        status: "exposure_ledger_semantically_valid".to_string(),
        campaign_id: ledger.campaign_id,
        event_count: ledger.event_count,
        selected_challenge_count: selected.len(),
        executed_challenge_count: executed.len(),
        frozen_result_count: results_frozen.len(),
        disclosed_result_count: disclosed.len(),
        rotated_challenge_count: rotated.len(),
        cumulative_evaluation_cost_microcredits: ledger.cumulative_evaluation_cost_microcredits,
        post_freeze_mutation_detected: mutation_seen,
        confirmatory_claim_valid: ledger.confirmatory_claim_valid,
        claim_ceiling: ledger.claim_ceiling,
    })
}

fn validate_preregistration(path: &Path) -> Result<Value, Box<dyn Error>> {
    let bytes = fs::read(path)?;
    let value: Value = serde_json::from_slice(&bytes)?;
    validate_instance(
        &schema_path("CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json"),
        &value,
    )?;
    Ok(json!({
        "status": "preregistration_schema_valid",
        "campaign_id": value.get("campaign_id").and_then(Value::as_str).unwrap_or("unknown"),
        "claim_ceiling": value.get("claim_ceiling").and_then(Value::as_str).unwrap_or(CLAIM_CEILING),
        "performance_claim_established": false
    }))
}

fn semantic_verify_campaign(
    preregistration: &Value,
    ledger_value: &Value,
    preregistration_sha256: &str,
) -> Result<VerificationReport, Box<dyn Error>> {
    let ledger: Ledger = serde_json::from_value(ledger_value.clone())?;
    if ledger.preregistration_receipt_sha256 != preregistration_sha256 {
        return Err("ledger does not bind the exact preregistration artifact bytes".into());
    }
    if ledger.campaign_id
        != preregistration.get("campaign_id").and_then(Value::as_str).unwrap_or_default()
    {
        return Err("campaign_id differs between preregistration and exposure ledger".into());
    }
    if ledger.initial_sealed_bank_commitment_sha256
        != preregistration.get("sealed_bank_commitment_sha256").and_then(Value::as_str).unwrap_or_default()
    {
        return Err("exposure ledger sealed-bank commitment differs from frozen preregistration".into());
    }

    let expected_candidate = preregistration["candidate_artifact_sha256"].as_str().ok_or("missing candidate digest")?;
    let expected_comparator = preregistration["comparator_artifact_sha256"].as_str().ok_or("missing comparator digest")?;
    let expected_evaluator = preregistration["evaluator_profile_sha256"].as_str().ok_or("missing evaluator digest")?;
    let expected_scorer = preregistration["scorer_profile_sha256"].as_str().ok_or("missing scorer digest")?;
    let mut mutation_seen = false;
    for event in &ledger.events {
        if event.preregistration_sha256_at_event != preregistration_sha256 {
            return Err(format!("event {} does not bind the exact frozen preregistration bytes", event.sequence).into());
        }
        let identity_matches = event.candidate_artifact_sha256_at_event == expected_candidate
            && event.comparator_artifact_sha256_at_event == expected_comparator
            && event.evaluator_profile_sha256_at_event == expected_evaluator
            && event.scorer_profile_sha256_at_event == expected_scorer;
        if !identity_matches {
            if event.sequence == 0 {
                return Err("campaign_frozen event differs from its preregistered artifact identities".into());
            }
            if event.material_mutation_since_freeze && event.protocol_invalidated {
                mutation_seen = true;
            } else if !mutation_seen
                || ledger.campaign_disposition != "invalidated"
            {
                return Err(format!(
                    "event {} diverges from frozen candidate/comparator/evaluator/scorer identity without explicit invalidation",
                    event.sequence
                ).into());
            }
        }
        if event.material_mutation_since_freeze && event.protocol_invalidated {
            mutation_seen = true;
        }
    }

    let adaptive = preregistration.get("adaptive_selection").ok_or("missing adaptive_selection")?;
    let max_selected = adaptive.get("max_selected_cases").and_then(Value::as_u64).ok_or("missing max_selected_cases")?;
    let max_cost = adaptive.get("max_evaluation_cost_microcredits").and_then(Value::as_u64).ok_or("missing max_evaluation_cost_microcredits")?;
    let audit_fraction = adaptive.get("random_audit_fraction_permille").and_then(Value::as_u64).ok_or("missing random_audit_fraction_permille")?;

    let mut selected_by_regime = std::collections::BTreeMap::<String, usize>::new();
    let mut selected_reasons = std::collections::BTreeMap::<String, String>::new();
    for event in &ledger.events {
        if event.event_type != "selection_batch" {
            continue;
        }
        for decision in &event.eligible_case_decisions {
            if decision.selected {
                *selected_by_regime.entry(decision.regime.clone()).or_default() += 1;
                selected_reasons.insert(
                    decision.challenge_commitment_sha256.clone(),
                    decision.decision_reason_code.clone(),
                );
            }
        }
    }
    let selected_count = selected_by_regime.values().sum::<usize>();
    if selected_count as u64 > max_selected && ledger.campaign_disposition != "invalidated" {
        return Err("campaign selected more challenge cases than preregistered maximum".into());
    }
    if ledger.cumulative_evaluation_cost_microcredits > max_cost
        && ledger.campaign_disposition != "invalidated"
    {
        return Err("campaign exceeded preregistered evaluation-cost budget without invalidation".into());
    }

    let preregistered_claim_ceiling =
        preregistration.get("claim_ceiling").and_then(Value::as_str).unwrap_or_default();
    if ledger.claim_ceiling != preregistered_claim_ceiling {
        return Err("ledger claim ceiling differs from the preregistered claim ceiling".into());
    }

    if ledger.campaign_disposition == "confirmatory_complete" {
        if adaptive.get("selection_bias_handling").and_then(Value::as_str)
            != Some("design_based_estimator_preregistered")
        {
            return Err("confirmatory completion requires a preregistered estimator for the selection design".into());
        }
        if preregistration["statistical_plan"]["uncertainty_method"].as_str()
            == Some("descriptive_only")
        {
            return Err("descriptive-only uncertainty analysis cannot support a confirmatory claim".into());
        }
        for event in &ledger.events {
            if event.event_type == "selection_batch" {
                for decision in &event.eligible_case_decisions {
                    if decision.eligible {
                        let probability = decision.selection_probability_permille.ok_or(
                            "confirmatory design-based analysis requires inclusion probabilities for every eligible case"
                        )?;
                        if probability == 0 || probability > 1000 {
                            return Err("confirmatory inclusion probabilities must be 1..=1000 permille".into());
                        }
                    }
                }
            }
        }
    }

    if matches!(ledger.campaign_disposition.as_str(), "confirmatory_complete" | "descriptive_complete") {
        let quotas = adaptive.get("minimum_cases_per_regime")
            .and_then(Value::as_array).ok_or("missing minimum_cases_per_regime")?;
        for quota in quotas {
            let regime = quota.get("regime").and_then(Value::as_str).ok_or("quota lacks regime")?;
            let minimum = quota.get("minimum_count").and_then(Value::as_u64).ok_or("quota lacks minimum_count")?;
            let observed = selected_by_regime.get(regime).copied().unwrap_or(0) as u64;
            if observed < minimum {
                return Err(format!("regime {regime} coverage {observed} is below preregistered minimum {minimum}").into());
            }
        }
        let audit_count = selected_reasons.values()
            .filter(|reason| reason.as_str() == "selected_for_random_audit")
            .count() as u64;
        let required_audit_count = (selected_count as u64 * audit_fraction + 999) / 1000;
        if audit_count < required_audit_count {
            return Err(format!(
                "random audit count {audit_count} is below preregistered minimum {required_audit_count}"
            ).into());
        }
        if mutation_seen {
            return Err("post-freeze identity mutation invalidates completed campaign".into());
        }
    }

    semantic_verify_ledger(ledger_value)
}

fn verify_campaign(
    preregistration_path: &Path,
    ledger_path: &Path,
) -> Result<VerificationReport, Box<dyn Error>> {
    let preregistration_bytes = fs::read(preregistration_path)?;
    let preregistration: Value = serde_json::from_slice(&preregistration_bytes)?;
    validate_instance(
        &schema_path("CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json"),
        &preregistration,
    )?;
    let ledger_bytes = fs::read(ledger_path)?;
    let ledger: Value = serde_json::from_slice(&ledger_bytes)?;
    validate_instance(
        &schema_path("CIV_IMPACT_ADAPTIVE_EXPOSURE_LEDGER_V0_1.schema.json"),
        &ledger,
    )?;
    semantic_verify_campaign(
        &preregistration,
        &ledger,
        &sha256_hex(&preregistration_bytes),
    )
}

fn print_json<T: serde::Serialize>(value: &T) -> Result<(), Box<dyn Error>> {
    println!("{}", serde_json::to_string_pretty(value)?);
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "help".to_string());
    match command.as_str() {
        "validate-prereg" => {
            let path = args.next().ok_or("usage: validate-prereg <receipt.json>")?;
            let result = validate_preregistration(Path::new(&path))?;
            print_json(&result)?;
        }
        "verify-ledger" => {
            let preregistration = args.next()
                .ok_or("usage: verify-ledger <preregistration.json> <exposure-ledger.json>")?;
            let ledger = args.next()
                .ok_or("usage: verify-ledger <preregistration.json> <exposure-ledger.json>")?;
            let result = verify_campaign(Path::new(&preregistration), Path::new(&ledger))?;
            print_json(&result)?;
        }
        "help" | "--help" | "-h" => {
            println!("civ-impact-adaptive-challenge commands:");
            println!("  validate-prereg <receipt.json>  Validate a frozen campaign preregistration.");
            println!("  verify-ledger <preregistration.json> <exposure-ledger.json>");
            println!("                                  Validate schemas, frozen identities, budget, quotas, and event semantics.");
        }
        other => return Err(format!("unknown command {other:?}; use --help").into()),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest() -> String {
        "a".repeat(64)
    }

    fn event(
        sequence: usize,
        event_type: &str,
        previous: Option<String>,
        case_commitments: Vec<String>,
    ) -> Value {
        json!({
            "sequence": sequence,
            "event_type": event_type,
            "recorded_at": "2026-10-16T14:00:00Z",
            "actor_role": "independent_evaluator",
            "previous_event_sha256": previous,
            "event_sha256": digest(),
            "preregistration_sha256_at_event": digest(),
            "candidate_artifact_sha256_at_event": "b".repeat(64),
            "comparator_artifact_sha256_at_event": "c".repeat(64),
            "evaluator_profile_sha256_at_event": "d".repeat(64),
            "scorer_profile_sha256_at_event": "e".repeat(64),
            "eligible_case_decisions": [],
            "selected_case_commitments": case_commitments,
            "result_frozen": event_type == "result_frozen" || event_type == "result_disclosed" || event_type == "challenge_rotated",
            "result_disclosed_to_candidate": event_type == "result_disclosed" || event_type == "challenge_rotated",
            "material_mutation_since_freeze": event_type == "mutation_detected",
            "protocol_invalidated": event_type == "mutation_detected",
            "reason": format!("test event {sequence}")
        })
    }

    fn finalized_ledger(mut events: Vec<Value>, disposition: &str, claim_valid: bool) -> Value {
        let mut previous: Option<String> = None;
        for (sequence, ev) in events.iter_mut().enumerate() {
            ev["sequence"] = json!(sequence);
            ev["previous_event_sha256"] = previous.clone().map_or(Value::Null, Value::String);
            ev["event_sha256"] = Value::String(digest());
            let hash = event_digest(ev).expect("event digest");
            ev["event_sha256"] = Value::String(hash.clone());
            previous = Some(hash);
        }
        json!({
            "schema_version": "0.1",
            "campaign_id": "civ-impact-campaign-001",
            "preregistration_receipt_sha256": digest(),
            "initial_sealed_bank_commitment_sha256": "f".repeat(64),
            "event_count": events.len(),
            "cumulative_evaluation_cost_microcredits": 100,
            "events": events,
            "campaign_disposition": disposition,
            "confirmatory_claim_valid": claim_valid,
            "claim_ceiling": CLAIM_CEILING
        })
    }

    fn valid_ledger_value() -> Value {
        let challenge = "1".repeat(64);
        let mut events = vec![
            event(0, "campaign_frozen", None, vec![]),
            event(1, "selection_batch", None, vec![challenge.clone()]),
            event(2, "challenge_executed", None, vec![challenge.clone()]),
            event(3, "result_frozen", None, vec![challenge.clone()]),
            event(4, "result_disclosed", None, vec![challenge.clone()]),
            event(5, "challenge_rotated", None, vec![challenge.clone()]),
            event(6, "campaign_closed", None, vec![]),
        ];
        events[1]["eligible_case_decisions"] = json!([{
            "challenge_commitment_sha256": challenge,
            "regime": "stale_or_missing",
            "eligible": true,
            "selected": true,
            "selection_probability_permille": 1000,
            "selector_score_microcredits": 22,
            "decision_reason_code": "selected_for_random_audit"
        }]);
        events[1]["selected_case_commitments"] = json!([challenge]);
        finalized_ledger(events, "confirmatory_complete", true)
    }

    fn valid_preregistration_value() -> Value {
        let minimum_cases = vec![
            json!({"regime":"stale_or_missing","minimum_count":1})
        ];
        json!({
          "schema_version":"0.1",
          "benchmark_id":"CIV-IMPACT-DECISION-VALUE-SYNTH-001",
          "campaign_id":"civ-impact-campaign-001",
          "parent_protocol_sha256":digest(),
          "candidate_artifact_sha256":"b".repeat(64),
          "comparator_artifact_sha256":"c".repeat(64),
          "evaluator_profile_sha256":"d".repeat(64),
          "scorer_profile_sha256":"e".repeat(64),
          "environment_profile_sha256":"f".repeat(64),
          "scenario_generator_sha256":"1".repeat(64),
          "public_regression_corpus_sha256":"2".repeat(64),
          "sealed_bank_commitment_sha256":"3".repeat(64),
          "public_regression_count":12,
          "sealed_challenge_count":240,
          "preregistered_hypotheses":[{
            "hypothesis_id":"H1",
            "statement":"The candidate lowers paired end-to-end loss on the declared challenge domain.",
            "expected_direction":"candidate_lower_is_better",
            "minimum_effect_microcredits":5
          }],
          "primary_outcome":"paired_end_to_end_loss_reduction_microcredits",
          "comparators":["observation_rule"],
          "adaptive_selection":{
            "mode":"fixed_random",
            "selector_profile_sha256":"4".repeat(64),
            "selector_objective":"uniform_random",
            "max_selected_cases":60,
            "max_evaluation_cost_microcredits":5000,
            "random_audit_fraction_permille":250,
            "minimum_cases_per_regime":minimum_cases,
            "stopping_rule":"fixed_case_count",
            "selection_bias_handling":"design_based_estimator_preregistered"
          },
          "statistical_plan":{
            "comparison_design":"paired_candidate_comparator_on_same_challenge",
            "analysis_unit":"challenge_episode",
            "cluster_unit":"scenario_family",
            "interim_peeking_allowed":false,
            "multiplicity_method":"none_single_primary_endpoint",
            "uncertainty_method":"paired_cluster_bootstrap",
            "missing_outcome_policy":"retain_as_failure_or_declared_partial"
          },
          "exposure_freeze":{
            "frozen_at":"2026-10-16T14:00:00Z",
            "no_tune_after_reveal":true,
            "challenge_rotation":"disclosed_case_becomes_public_regression_and_is_replaced_under_new_version",
            "public_cases_confirmatory_credit":false
          },
          "claim_ceiling":CLAIM_CEILING
        })
    }

    fn bound_valid_campaign() -> (Value, Value, String) {
        let preregistration = valid_preregistration_value();
        let preregistration_bytes =
            serde_json::to_vec_pretty(&preregistration).expect("serialize preregistration");
        let preregistration_sha256 = sha256_hex(&preregistration_bytes);
        let mut ledger = valid_ledger_value();
        ledger["preregistration_receipt_sha256"] = json!(preregistration_sha256);
        ledger["initial_sealed_bank_commitment_sha256"] =
            preregistration["sealed_bank_commitment_sha256"].clone();
        let events = ledger["events"].as_array_mut().expect("events");
        for event in events.iter_mut() {
            event["preregistration_sha256_at_event"] = json!(preregistration_sha256);
            event["candidate_artifact_sha256_at_event"] =
                preregistration["candidate_artifact_sha256"].clone();
            event["comparator_artifact_sha256_at_event"] =
                preregistration["comparator_artifact_sha256"].clone();
            event["evaluator_profile_sha256_at_event"] =
                preregistration["evaluator_profile_sha256"].clone();
            event["scorer_profile_sha256_at_event"] =
                preregistration["scorer_profile_sha256"].clone();
        }
        rehash_ledger(&mut ledger);
        (preregistration, ledger, preregistration_sha256)
    }

    fn install_mutation_event(ledger: &mut Value) {
        let event_count = {
            let events = ledger["events"].as_array_mut().expect("events array");
            let mut mutation = event(6, "mutation_detected", None, vec![]);
            mutation["candidate_artifact_sha256_at_event"] = json!("9".repeat(64));
            events.insert(6, mutation);
            // The original campaign_closed event shifts to sequence 7; preserve
            // it and keep the mutated candidate identity stable in later records.
            events[7]["candidate_artifact_sha256_at_event"] = json!("9".repeat(64));

            let mut previous: Option<String> = None;
            for (sequence, event) in events.iter_mut().enumerate() {
                event["sequence"] = json!(sequence);
                event["previous_event_sha256"] =
                    previous.clone().map_or(Value::Null, Value::String);
                event["event_sha256"] = json!(digest());
                let hash = event_digest(event).expect("event digest");
                event["event_sha256"] = json!(hash.clone());
                previous = Some(hash);
            }
            events.len()
        };
        ledger["campaign_disposition"] = json!("invalidated");
        ledger["confirmatory_claim_valid"] = json!(false);
        ledger["event_count"] = json!(event_count);
    }

    #[test]
    fn preregistration_schema_accepts_a_frozen_valid_profile() {
        let schema = schema_path("CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json");
        validate_instance(&schema, &valid_preregistration_value()).expect("valid preregistration");
    }

    #[test]
    fn preregistration_rejects_peeking_without_sequential_method() {
        let mut value = valid_preregistration_value();
        value["statistical_plan"]["interim_peeking_allowed"] = json!(true);
        let schema = schema_path("CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json");
        assert!(validate_instance(&schema, &value).is_err());
    }

    #[test]
    fn preregistration_rejects_credit_for_public_regression_items() {
        let mut value = valid_preregistration_value();
        value["exposure_freeze"]["public_cases_confirmatory_credit"] = json!(true);
        let schema = schema_path("CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json");
        assert!(validate_instance(&schema, &value).is_err());
    }

    #[test]
    fn campaign_verification_binds_exact_preregistration_and_bank_commitment() {
        let (preregistration, ledger, preregistration_sha256) = bound_valid_campaign();
        let report = semantic_verify_campaign(
            &preregistration,
            &ledger,
            &preregistration_sha256,
        ).expect("valid frozen campaign");
        assert!(report.confirmatory_claim_valid);

        let mut altered_preregistration = preregistration.clone();
        altered_preregistration["primary_outcome"] = json!("mean_decision_regret_microcredits");
        let altered_sha = sha256_hex(
            &serde_json::to_vec_pretty(&altered_preregistration).expect("serialize altered prereg"),
        );
        assert!(semantic_verify_campaign(&altered_preregistration, &ledger, &altered_sha).is_err());
    }

    #[test]
    fn campaign_verification_rejects_candidate_identity_not_in_preregistration() {
        let (preregistration, mut ledger, preregistration_sha256) = bound_valid_campaign();
        for event in ledger["events"].as_array_mut().expect("events") {
            event["candidate_artifact_sha256_at_event"] = json!("9".repeat(64));
        }
        rehash_ledger(&mut ledger);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn campaign_verification_rejects_cost_budget_overrun_for_confirmatory_run() {
        let (preregistration, mut ledger, preregistration_sha256) = bound_valid_campaign();
        ledger["cumulative_evaluation_cost_microcredits"] = json!(6000);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn campaign_verification_rejects_regime_quota_shortfall() {
        let (mut preregistration, mut ledger, _) = bound_valid_campaign();
        preregistration["adaptive_selection"]["minimum_cases_per_regime"][0]["minimum_count"] = json!(2);
        let preregistration_sha256 = sha256_hex(
            &serde_json::to_vec_pretty(&preregistration).expect("serialize prereg"),
        );
        ledger["preregistration_receipt_sha256"] = json!(preregistration_sha256);
        for event in ledger["events"].as_array_mut().expect("events") {
            event["preregistration_sha256_at_event"] = json!(preregistration_sha256);
        }
        rehash_ledger(&mut ledger);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn descriptive_complete_campaign_is_reportable_without_confirmatory_credit() {
        let (mut preregistration, mut ledger, _) = bound_valid_campaign();
        preregistration["adaptive_selection"]["selection_bias_handling"] =
            json!("report_descriptive_selected_set_only");
        preregistration["claim_ceiling"] = json!(DESCRIPTIVE_CLAIM_CEILING);
        let preregistration_sha256 = sha256_hex(
            &serde_json::to_vec_pretty(&preregistration).expect("serialize prereg"),
        );
        ledger["preregistration_receipt_sha256"] = json!(preregistration_sha256);
        ledger["claim_ceiling"] = json!(DESCRIPTIVE_CLAIM_CEILING);
        ledger["campaign_disposition"] = json!("descriptive_complete");
        ledger["confirmatory_claim_valid"] = json!(false);
        for event in ledger["events"].as_array_mut().expect("events") {
            event["preregistration_sha256_at_event"] = json!(preregistration_sha256);
        }
        rehash_ledger(&mut ledger);
        validate_instance(
            &schema_path("CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json"),
            &preregistration,
        ).expect("descriptive preregistration schema");
        validate_instance(
            &schema_path("CIV_IMPACT_ADAPTIVE_EXPOSURE_LEDGER_V0_1.schema.json"),
            &ledger,
        ).expect("descriptive ledger schema");
        let report = semantic_verify_campaign(
            &preregistration,
            &ledger,
            &preregistration_sha256,
        ).expect("valid descriptive-only campaign");
        assert!(!report.confirmatory_claim_valid);
        assert_eq!(report.claim_ceiling, DESCRIPTIVE_CLAIM_CEILING);
    }

    #[test]
    fn confirmatory_campaign_rejects_descriptive_only_selection_analysis() {
        let (mut preregistration, mut ledger, _) = bound_valid_campaign();
        preregistration["adaptive_selection"]["selection_bias_handling"] =
            json!("report_descriptive_selected_set_only");
        let preregistration_sha256 = sha256_hex(
            &serde_json::to_vec_pretty(&preregistration).expect("serialize prereg"),
        );
        ledger["preregistration_receipt_sha256"] = json!(preregistration_sha256);
        for event in ledger["events"].as_array_mut().expect("events") {
            event["preregistration_sha256_at_event"] = json!(preregistration_sha256);
        }
        rehash_ledger(&mut ledger);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn confirmatory_campaign_rejects_descriptive_only_uncertainty_method() {
        let (mut preregistration, mut ledger, _) = bound_valid_campaign();
        preregistration["statistical_plan"]["uncertainty_method"] = json!("descriptive_only");
        let preregistration_sha256 = sha256_hex(
            &serde_json::to_vec_pretty(&preregistration).expect("serialize prereg"),
        );
        ledger["preregistration_receipt_sha256"] = json!(preregistration_sha256);
        for event in ledger["events"].as_array_mut().expect("events") {
            event["preregistration_sha256_at_event"] = json!(preregistration_sha256);
        }
        rehash_ledger(&mut ledger);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn confirmatory_campaign_rejects_missing_probability_for_eligible_unselected_case() {
        let (preregistration, mut ledger, preregistration_sha256) = bound_valid_campaign();
        ledger["events"][1]["eligible_case_decisions"].as_array_mut()
            .expect("selector decisions")
            .push(json!({
                "challenge_commitment_sha256":"2".repeat(64),
                "regime":"nominal",
                "eligible":true,
                "selected":false,
                "selection_probability_permille":null,
                "selector_score_microcredits":0,
                "decision_reason_code":"not_selected_by_frozen_selector"
            }));
        rehash_ledger(&mut ledger);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn campaign_verification_enforces_random_audit_fraction() {
        let (preregistration, mut ledger, preregistration_sha256) = bound_valid_campaign();
        ledger["events"][1]["eligible_case_decisions"][0]["decision_reason_code"] =
            json!("selected_by_frozen_selector");
        rehash_ledger(&mut ledger);
        assert!(semantic_verify_campaign(&preregistration, &ledger, &preregistration_sha256).is_err());
    }

    #[test]
    fn valid_ledger_chain_verifies_all_selected_cases() {
        let value = valid_ledger_value();
        let schema = schema_path("CIV_IMPACT_ADAPTIVE_EXPOSURE_LEDGER_V0_1.schema.json");
        validate_instance(&schema, &value).expect("ledger schema");
        let report = semantic_verify_ledger(&value).expect("semantic verification");
        assert_eq!(report.selected_challenge_count, 1);
        assert_eq!(report.executed_challenge_count, 1);
        assert_eq!(report.frozen_result_count, 1);
        assert_eq!(report.disclosed_result_count, 1);
        assert_eq!(report.rotated_challenge_count, 1);
        assert!(report.confirmatory_claim_valid);
    }

    fn rehash_ledger(value: &mut Value) {
        let event_count = {
            let events = value["events"].as_array_mut().expect("events array");
            let mut previous: Option<String> = None;
            for (sequence, event) in events.iter_mut().enumerate() {
                event["sequence"] = json!(sequence);
                event["previous_event_sha256"] =
                    previous.clone().map_or(Value::Null, Value::String);
                event["event_sha256"] = json!(digest());
                let hash = event_digest(event).expect("digest event");
                event["event_sha256"] = json!(hash.clone());
                previous = Some(hash);
            }
            events.len()
        };
        value["event_count"] = json!(event_count);
    }

    #[test]
    fn rejects_reintroduced_unselected_challenge_in_later_selection_batch() {
        let mut value = valid_ledger_value();
        let second_challenge = "2".repeat(64);
        let mut second_batch = event(7, "selection_batch", None, vec![second_challenge.clone()]);
        second_batch["eligible_case_decisions"] = json!([
            {
                "challenge_commitment_sha256": "1".repeat(64),
                "regime": "stale_or_missing",
                "eligible": true,
                "selected": false,
                "selection_probability_permille": 500,
                "selector_score_microcredits": 10,
                "decision_reason_code": "not_selected_by_frozen_selector"
            },
            {
                "challenge_commitment_sha256": second_challenge,
                "regime": "nominal",
                "eligible": true,
                "selected": true,
                "selection_probability_permille": 500,
                "selector_score_microcredits": 20,
                "decision_reason_code": "selected_by_frozen_selector"
            }
        ]);
        second_batch["selected_case_commitments"] = json!(["2".repeat(64)]);
        value["events"].as_array_mut().expect("events").insert(6, second_batch);
        value["campaign_disposition"] = json!("invalidated");
        value["confirmatory_claim_valid"] = json!(false);
        rehash_ledger(&mut value);
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_selector_candidate_rows_even_when_hashes_are_recomputed() {
        let mut value = valid_ledger_value();
        let decision = value["events"][1]["eligible_case_decisions"][0].clone();
        value["events"][1]["eligible_case_decisions"]
            .as_array_mut()
            .expect("decision array")
            .push(decision);
        rehash_ledger(&mut value);
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_terminal_event_inconsistent_with_campaign_disposition() {
        let mut value = valid_ledger_value();
        value["campaign_disposition"] = json!("sealed_active");
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_material_mutation_without_explicit_invalidation() {
        let mut value = valid_ledger_value();
        value["events"][3]["material_mutation_since_freeze"] = json!(true);
        rehash_ledger(&mut value);
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_event_count_mismatch() {
        let mut value = valid_ledger_value();
        let count = value["event_count"].as_u64().expect("count") + 1;
        value["event_count"] = json!(count);
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_event_digest_tampering() {
        let mut value = valid_ledger_value();
        value["events"][2]["reason"] = json!("tampered after event digest");
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_broken_previous_event_hash() {
        let mut value = valid_ledger_value();
        value["events"][3]["previous_event_sha256"] = json!("9".repeat(64));
        let events = value["events"].as_array_mut().expect("events");
        let mut computed_previous: Option<String> = None;
        for (i, ev) in events.iter_mut().enumerate() {
            if i == 3 {
                ev["previous_event_sha256"] = json!("9".repeat(64));
            } else {
                ev["previous_event_sha256"] =
                    computed_previous.clone().map_or(Value::Null, Value::String);
            }
            ev["event_sha256"] = json!(digest());
            let hash = event_digest(ev).expect("digest");
            ev["event_sha256"] = json!(hash.clone());
            computed_previous = Some(hash);
        }
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn rejects_result_disclosure_before_freeze() {
        let mut value = valid_ledger_value();
        let challenge = value["events"][3]["selected_case_commitments"][0].clone();
        value["events"][3] = event(3, "result_disclosed", None, vec![challenge.as_str().expect("case").to_string()]);
        value["events"][4] = event(4, "result_frozen", None, vec![challenge.as_str().expect("case").to_string()]);
        let events = value["events"].as_array_mut().expect("events");
        let mut previous: Option<String> = None;
        for (i, ev) in events.iter_mut().enumerate() {
            ev["sequence"] = json!(i);
            ev["previous_event_sha256"] = previous.clone().map_or(Value::Null, Value::String);
            ev["event_sha256"] = json!(digest());
            let hash = event_digest(ev).expect("hash");
            ev["event_sha256"] = json!(hash.clone());
            previous = Some(hash);
        }
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn post_freeze_mutation_invalidates_confirmatory_claim() {
        let mut value = valid_ledger_value();
        install_mutation_event(&mut value);
        let report = semantic_verify_ledger(&value).expect("invalidation is represented");
        assert!(report.post_freeze_mutation_detected);
        assert!(!report.confirmatory_claim_valid);
    }

    #[test]
    fn a_material_identity_change_without_invalidation_is_rejected() {
        let mut value = valid_ledger_value();
        value["events"][3]["candidate_artifact_sha256_at_event"] = json!("9".repeat(64));
        let events = value["events"].as_array_mut().expect("events");
        let mut previous: Option<String> = None;
        for (i, ev) in events.iter_mut().enumerate() {
            ev["sequence"] = json!(i);
            ev["previous_event_sha256"] = previous.clone().map_or(Value::Null, Value::String);
            ev["event_sha256"] = json!(digest());
            let hash = event_digest(ev).expect("hash");
            ev["event_sha256"] = json!(hash.clone());
            previous = Some(hash);
        }
        assert!(semantic_verify_ledger(&value).is_err());
    }

    #[test]
    fn a_challenge_cannot_be_executed_before_selection() {
        let mut value = valid_ledger_value();
        let events = value["events"].as_array_mut().expect("events");
        events.swap(1, 2);
        let mut previous: Option<String> = None;
        for (i, ev) in events.iter_mut().enumerate() {
            ev["sequence"] = json!(i);
            ev["previous_event_sha256"] = previous.clone().map_or(Value::Null, Value::String);
            ev["event_sha256"] = json!(digest());
            let hash = event_digest(ev).expect("hash");
            ev["event_sha256"] = json!(hash.clone());
            previous = Some(hash);
        }
        assert!(semantic_verify_ledger(&value).is_err());
    }
}
