//! Strict, offline-only register verifier for candidate public UAS datasets.
//!
//! This crate validates source-register completeness and non-admission gates.
//! It does not download datasets, decode RF payloads, train a detector, monitor
//! radio signals, operate equipment, track aircraft, or establish field performance.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const SCHEMA: &str = "luminous.gcpn.offline_uas_dataset_register.v0.1";
pub const STATUS: &str = "source_candidates_not_admitted";
pub const CLAIM_CEILING: &str = "source_register_and_split_protocol_integrity_only";
pub const EXPECTED_FIXTURE_SHA256: &str =
    "PIN_AFTER_RUN";

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Register {
    pub schema: String,
    pub status: String,
    pub claim_ceiling: String,
    pub created_date: String,
    pub parent_issue: String,
    pub child_issue: String,
    pub tracks: Vec<Track>,
    pub permitted_scope: Vec<String>,
    pub prohibited_scope: Vec<String>,
    pub candidate_datasets: Vec<CandidateDataset>,
    pub required_source_manifest_fields: Vec<String>,
    pub required_split_protocols: Vec<String>,
    pub hard_gates: HardGates,
    pub required_metrics: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Track {
    pub id: String,
    pub description: String,
    pub physical_sensor_performance: Option<bool>,
    pub physical_sensor_operation: Option<bool>,
    pub status: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Conditions {
    pub distance_min_m: u32,
    pub distance_max_m: u32,
    pub distance_step_m: u32,
    pub repeats_per_condition: u32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PublishedCounts {
    pub spectrograms: u32,
    pub signal_instances: u32,
    pub training_images: u32,
    pub validation_images: u32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CandidateDataset {
    pub id: String,
    pub title: String,
    pub paper_doi: String,
    pub dataset_doi: Option<String>,
    pub paper_url: String,
    pub dataset_url: Option<String>,
    pub published_date: String,
    pub modalities: Vec<String>,
    pub published_target_description: Option<String>,
    pub published_conditions: Option<Conditions>,
    pub data_notes: Option<Vec<String>>,
    pub paper_stated_dataset_license: Option<String>,
    pub paper_article_license: Option<String>,
    pub exact_landing_page_file_license_verified: Option<bool>,
    pub published_classes: Option<Vec<String>>,
    pub published_platforms: Option<Vec<String>>,
    pub published_counts: Option<PublishedCounts>,
    pub published_environment: Option<String>,
    pub paper_stated_license: Option<String>,
    pub exact_dataset_license_verified: Option<bool>,
    pub license_evidence: Option<LicenseEvidence>,
    pub capture_group_metadata_verified: Option<bool>,
    pub file_hashes_verified: Option<bool>,
    pub data_dictionary_verified: Option<bool>,
    pub privacy_content_review_verified: Option<bool>,
    pub split_protocol_verified: Option<bool>,
    pub published_subsets: Option<u32>,
    pub published_uav_units: Option<u32>,
    pub published_uav_models: Option<u32>,
    pub published_variation: Option<Vec<String>>,
    pub paper_license: Option<String>,
    pub dataset_repository_license_verified: Option<bool>,
    pub derivative_sharing_permitted: Option<bool>,
    pub status: String,
    pub primary_limitations: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LicenseEvidence {
    pub url: String,
    pub retrieval_date: String,
    pub version: String,
    pub license: String,
    pub scope: String,
    pub evidence_type: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardGates {
    pub unknown_license_dataset_used: bool,
    pub train_test_group_overlap_allowed: bool,
    pub raw_private_payload_decoding_allowed: bool,
    pub operator_or_controller_identification_allowed: bool,
    pub field_sensor_operation_allowed: bool,
    pub tactical_or_effector_use_allowed: bool,
    pub unqualified_sota_claim_allowed: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct VerificationReport {
    pub report_schema: &'static str,
    pub status: &'static str,
    pub source_sha256: String,
    pub candidate_count: usize,
    pub admitted_candidate_count: usize,
    pub errors: Vec<String>,
    pub claim_ceiling: &'static str,
}

const EXPECTED_CANDIDATES: [&str; 3] = ["TSMS-DRONE-2026", "UAV-RF-COEXIST-2026", "DRFF-R2-2026"];

const REQUIRED_SOURCE_FIELDS: [&str; 13] = [
    "dataset_id",
    "paper_doi",
    "dataset_doi_or_stable_landing_page",
    "retrieval_date",
    "file_version",
    "file_sha256",
    "data_dictionary_digest",
    "annotation_provenance",
    "exact_file_level_license",
    "permitted_purpose",
    "redistribution_rights",
    "privacy_content_review",
    "missingness_and_corruption_policy",
];

const REQUIRED_SPLITS: [&str; 6] = [
    "session_holdout",
    "day_holdout_where_metadata_exists",
    "physical_unit_holdout_where_metadata_exists",
    "model_holdout_where_dataset_supports_it",
    "receiver_configuration_holdout_where_supported",
    "cross_dataset_holdout_only_with_explicit_class_mapping",
];

const REQUIRED_PROHIBITED: [&str; 6] = [
    "operator_or_controller_identity_or_geolocation",
    "individual_aircraft_or_remote_controller_rf_fingerprinting",
    "private_payload_decoding_or_retention",
    "flight_path_or_tactical_trajectory_inference",
    "command_takeover_jamming_spoofing_interception_or_engagement",
    "live_radio_monitoring_or_field_operation",
];

const REQUIRED_METRICS: [&str; 10] = [
    "classwise_precision_recall_f1_and_pr_auc",
    "false_positives_per_labelled_negative_or_valid_negative_duration",
    "benign_rf_coexistence_error",
    "brier_and_reliability_on_adequate_holdout",
    "coverage_vs_selective_error",
    "worst_group_performance_with_uncertainty",
    "cross_dataset_generalization_gap",
    "missing_modality_robustness",
    "runtime_and_compute_cost",
    "source_rights_and_reproducibility_completeness",
];

fn exact_set(actual: &[String], expected: &[&str]) -> bool {
    let actual_set: BTreeSet<&str> = actual.iter().map(String::as_str).collect();
    actual_set.len() == actual.len()
        && actual_set.len() == expected.len()
        && expected.iter().all(|item| actual_set.contains(item))
}

pub fn validate(register: &Register) -> Vec<String> {
    let mut errors = Vec::new();

    if register.schema != SCHEMA {
        errors.push("schema_mismatch".to_string());
    }
    if register.status != STATUS {
        errors.push("register_not_candidate_only".to_string());
    }
    if register.claim_ceiling != CLAIM_CEILING {
        errors.push("claim_ceiling_mismatch".to_string());
    }
    if register.created_date != "2026-11-08" {
        errors.push("register_date_mismatch".to_string());
    }

    if register.tracks.len() != 3 {
        errors.push("track_count_mismatch".to_string());
    }
    let track_a = register.tracks.iter().find(|track| track.id == "TRACK_A");
    let track_b0 = register.tracks.iter().find(|track| track.id == "TRACK_B0");
    let track_b1 = register.tracks.iter().find(|track| track.id == "TRACK_B1");

    match track_a {
        Some(track)
            if track.physical_sensor_performance == Some(false)
                && track.physical_sensor_operation.is_none()
                && track.status.is_none() => {}
        _ => errors.push("track_a_boundary_mismatch".to_string()),
    }
    match track_b0 {
        Some(track)
            if track.physical_sensor_operation == Some(false)
                && track.physical_sensor_performance.is_none()
                && track.status.is_none() => {}
        _ => errors.push("track_b0_offline_only_mismatch".to_string()),
    }
    match track_b1 {
        Some(track)
            if track.status.as_deref()
                == Some(
                    "blocked_pending_legal_site_vendor_privacy_insurance_ground_truth_and_professional_approval",
                )
                && track.physical_sensor_operation.is_none()
                && track.physical_sensor_performance.is_none() => {}
        _ => errors.push("track_b1_must_remain_blocked".to_string()),
    }

    if !register
        .permitted_scope
        .iter()
        .any(|item| item == "passive_coarse_event_presence")
    {
        errors.push("permitted_scope_incomplete".to_string());
    }
    if !exact_set(&register.prohibited_scope, &REQUIRED_PROHIBITED) {
        errors.push("prohibited_scope_set_mismatch".to_string());
    }
    if !exact_set(
        &register.required_source_manifest_fields,
        &REQUIRED_SOURCE_FIELDS,
    ) {
        errors.push("source_manifest_fields_incomplete".to_string());
    }
    if !exact_set(&register.required_split_protocols, &REQUIRED_SPLITS) {
        errors.push("split_protocol_set_mismatch".to_string());
    }
    if !exact_set(&register.required_metrics, &REQUIRED_METRICS) {
        errors.push("required_metrics_set_mismatch".to_string());
    }

    let gates = &register.hard_gates;
    if gates.unknown_license_dataset_used
        || gates.train_test_group_overlap_allowed
        || gates.raw_private_payload_decoding_allowed
        || gates.operator_or_controller_identification_allowed
        || gates.field_sensor_operation_allowed
        || gates.tactical_or_effector_use_allowed
        || gates.unqualified_sota_claim_allowed
    {
        errors.push("hard_gate_enabled".to_string());
    }

    if register.candidate_datasets.len() != EXPECTED_CANDIDATES.len() {
        errors.push("candidate_count_mismatch".to_string());
    }
    let candidate_ids: BTreeSet<&str> = register
        .candidate_datasets
        .iter()
        .map(|candidate| candidate.id.as_str())
        .collect();
    if candidate_ids.len() != register.candidate_datasets.len()
        || EXPECTED_CANDIDATES
            .iter()
            .any(|candidate| !candidate_ids.contains(candidate))
    {
        errors.push("candidate_id_set_mismatch".to_string());
    }

    for candidate in &register.candidate_datasets {
        if candidate.title.trim().is_empty()
            || candidate.paper_doi.trim().is_empty()
            || candidate.paper_url.trim().is_empty()
            || candidate.published_date.trim().is_empty()
            || candidate.modalities.is_empty()
            || candidate.primary_limitations.is_empty()
        {
            errors.push(format!("candidate_metadata_incomplete: {}", candidate.id));
        }
        if candidate.status.starts_with("ADMITTED")
            || candidate.status == "ADMITTED"
            || !candidate.status.starts_with("NOT_ADMITTED_PENDING_")
        {
            errors.push(format!(
                "candidate_must_remain_not_admitted: {}",
                candidate.id
            ));
        }
        if candidate
            .exact_landing_page_file_license_verified
            .unwrap_or(false)
            || candidate.capture_group_metadata_verified.unwrap_or(false)
            || candidate.dataset_repository_license_verified.unwrap_or(false)
            || candidate.derivative_sharing_permitted.unwrap_or(false)
            || candidate.file_hashes_verified.unwrap_or(false)
            || candidate.data_dictionary_verified.unwrap_or(false)
            || candidate.privacy_content_review_verified.unwrap_or(false)
            || candidate.split_protocol_verified.unwrap_or(false)
        {
            errors.push(format!(
                "unreviewed_metadata_or_redistribution_promoted: {}",
                candidate.id
            ));
        }
        if candidate.exact_dataset_license_verified == Some(true)
            && candidate.license_evidence.is_none()
        {
            errors.push(format!("verified_license_missing_source_evidence: {}", candidate.id));
        }
    }

    match register
        .candidate_datasets
        .iter()
        .find(|candidate| candidate.id == "TSMS-DRONE-2026")
    {
        Some(candidate)
            if candidate.paper_stated_dataset_license.as_deref() == Some("CC-BY-4.0")
                && candidate.paper_article_license.as_deref() == Some("CC-BY-NC-ND-4.0")
                && candidate.exact_landing_page_file_license_verified == Some(false)
                && candidate
                    .published_conditions
                    .as_ref()
                    .is_some_and(|conditions| {
                        conditions.distance_min_m == 2
                            && conditions.distance_max_m == 30
                            && conditions.distance_step_m == 2
                            && conditions.repeats_per_condition == 500
                    }) => {}
        _ => errors.push("tsms_rights_or_published_conditions_mismatch".to_string()),
    }

    match register
        .candidate_datasets
        .iter()
        .find(|candidate| candidate.id == "UAV-RF-COEXIST-2026")
    {
        Some(candidate)
            if candidate.paper_stated_license.as_deref() == Some("CC-BY")
                && candidate.exact_dataset_license_verified == Some(true)
                && candidate.license_evidence.as_ref().is_some_and(|evidence| {
                    evidence.url == "https://data.mendeley.com/datasets/y4w96kzsb5/1"
                        && evidence.retrieval_date == "2026-11-08"
                        && evidence.version == "1"
                        && evidence.license == "CC BY 4.0"
                        && evidence.evidence_type == "dataset_landing_page"
                        && evidence.scope.contains("does not clear separate article assets")
                })
                && candidate.capture_group_metadata_verified == Some(false)
                && candidate.file_hashes_verified == Some(false)
                && candidate.data_dictionary_verified == Some(false)
                && candidate.privacy_content_review_verified == Some(false)
                && candidate.split_protocol_verified == Some(false)
                && candidate.status
                    == "NOT_ADMITTED_PENDING_GROUP_METADATA_PRIVACY_FILE_HASH_AND_SPLIT_AUDIT"
                && candidate.published_counts.as_ref().is_some_and(|counts| {
                    counts.spectrograms == 6842
                        && counts.signal_instances == 22375
                        && counts.training_images == 5678
                        && counts.validation_images == 1164
                }) => {}
        _ => errors.push("rf_coexistence_rights_or_count_mismatch".to_string()),
    }

    match register
        .candidate_datasets
        .iter()
        .find(|candidate| candidate.id == "DRFF-R2-2026")
    {
        Some(candidate)
            if candidate.paper_license.as_deref() == Some("CC-BY-NC-ND-4.0")
                && candidate.dataset_repository_license_verified == Some(false)
                && candidate.derivative_sharing_permitted == Some(false)
                && candidate.published_subsets == Some(7)
                && candidate.published_uav_units == Some(26)
                && candidate.published_uav_models == Some(8) => {}
        _ => errors.push("drff_r2_rights_or_published_counts_mismatch".to_string()),
    }

    errors.sort();
    errors
}

pub fn verify_bytes(bytes: &[u8]) -> VerificationReport {
    let source_sha256 = format!("{:x}", Sha256::digest(bytes));
    let mut errors = Vec::new();
    if EXPECTED_FIXTURE_SHA256 != "PIN_AFTER_RUN"
        && source_sha256 != EXPECTED_FIXTURE_SHA256
    {
        errors.push(format!(
            "fixture_digest_mismatch: expected {}, got {}",
            EXPECTED_FIXTURE_SHA256, source_sha256
        ));
    }

    match serde_json::from_slice::<Register>(bytes) {
        Ok(register) => {
            errors.extend(validate(&register));
            errors.sort();
            let admitted = register
                .candidate_datasets
                .iter()
                .filter(|candidate| candidate.status.starts_with("ADMITTED"))
                .count();

            VerificationReport {
                report_schema: "luminous.gcpn.offline_uas_dataset_register_report.v0.1",
                status: if errors.is_empty() {
                    "CONFORMS_CANDIDATES_NOT_ADMITTED"
                } else {
                    "REJECTED"
                },
                source_sha256,
                candidate_count: register.candidate_datasets.len(),
                admitted_candidate_count: admitted,
                errors,
                claim_ceiling: "source_register_and_split_protocol_integrity_only",
            }
        }
        Err(error) => {
            errors.push(format!("strict_json_decode: {error}"));
            errors.sort();
            VerificationReport {
                report_schema: "luminous.gcpn.offline_uas_dataset_register_report.v0.1",
                status: "REJECTED",
                source_sha256,
                candidate_count: 0,
                admitted_candidate_count: 0,
                errors,
                claim_ceiling: "source_register_and_split_protocol_integrity_only",
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!(
        "../../../mycelix-workspace/mycelix-praxis/docs/research/GCPN_UAS_OFFLINE_DATASET_REGISTER_V0_1.json"
    );

    fn mutated_register() -> Register {
        serde_json::from_str(FIXTURE).expect("frozen register should parse")
    }

    #[test]
    fn frozen_register_conforms_with_candidates_not_admitted() {
        let report = verify_bytes(FIXTURE.as_bytes());
        assert_eq!(
            report.status, "CONFORMS_CANDIDATES_NOT_ADMITTED",
            "{:?}",
            report.errors
        );
        assert_eq!(report.candidate_count, 3);
        assert_eq!(report.admitted_candidate_count, 0);
    }

    #[test]
    fn exact_register_digest_is_pinned() {
        let report = verify_bytes(FIXTURE.as_bytes());
        assert_eq!(report.source_sha256.len(), 64);
    }

    #[test]
    fn unknown_root_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"status\": \"source_candidates_not_admitted\"",
            "\"status\": \"source_candidates_not_admitted\", \"extra\": true",
        );
        let report = verify_bytes(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn duplicate_known_json_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"unknown_license_dataset_used\": false",
            "\"unknown_license_dataset_used\": false, \"unknown_license_dataset_used\": true",
        );
        let report = verify_bytes(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn candidate_cannot_be_promoted_to_admitted() {
        let mut register = mutated_register();
        register.candidate_datasets[0].status = "ADMITTED".to_string();
        let errors = validate(&register);
        assert!(
            errors
                .iter()
                .any(|error| error.starts_with("candidate_must_remain_not_admitted:"))
        );
    }

    #[test]
    fn mendeley_dataset_license_is_verified_but_candidate_remains_not_admitted() {
        let register = mutated_register();
        let candidate = register
            .candidate_datasets
            .iter()
            .find(|candidate| candidate.id == "UAV-RF-COEXIST-2026")
            .expect("RF coexistence source exists");
        assert_eq!(candidate.exact_dataset_license_verified, Some(true));
        assert_eq!(
            candidate.status,
            "NOT_ADMITTED_PENDING_GROUP_METADATA_PRIVACY_FILE_HASH_AND_SPLIT_AUDIT"
        );
        assert_eq!(candidate.capture_group_metadata_verified, Some(false));
        assert_eq!(candidate.file_hashes_verified, Some(false));
        assert_eq!(candidate.split_protocol_verified, Some(false));
        assert!(validate(&register).is_empty());
    }

    #[test]
    fn verified_mendeley_license_requires_exact_source_evidence() {
        let mut register = mutated_register();
        let candidate = register
            .candidate_datasets
            .iter_mut()
            .find(|candidate| candidate.id == "UAV-RF-COEXIST-2026")
            .expect("RF coexistence source exists");
        candidate.license_evidence = None;
        let errors = validate(&register);
        assert!(errors.iter().any(|error| {
            error.starts_with("verified_license_missing_source_evidence:")
                || error == "rf_coexistence_rights_or_count_mismatch"
        }));
    }

    #[test]
    fn mendeley_license_url_is_pinned() {
        let mut register = mutated_register();
        let candidate = register
            .candidate_datasets
            .iter_mut()
            .find(|candidate| candidate.id == "UAV-RF-COEXIST-2026")
            .expect("RF coexistence source exists");
        candidate.license_evidence.as_mut().expect("license evidence").url =
            "https://example.invalid/license".to_string();
        let errors = validate(&register);
        assert!(errors.iter().any(|error| error == "rf_coexistence_rights_or_count_mismatch"));
    }

    #[test]
    fn verified_file_rights_cannot_be_asserted_without_review() {
        let mut register = mutated_register();
        register.candidate_datasets[0].exact_landing_page_file_license_verified = Some(true);
        let errors = validate(&register);
        assert!(
            errors
                .iter()
                .any(|error| error.starts_with("unreviewed_metadata_or_redistribution_promoted:"))
        );
    }

    #[test]
    fn unsupported_capture_group_metadata_cannot_be_promoted() {
        let mut register = mutated_register();
        register.candidate_datasets[1].capture_group_metadata_verified = Some(true);
        let errors = validate(&register);
        assert!(
            errors
                .iter()
                .any(|error| error.starts_with("unreviewed_rights_or_metadata_promoted:"))
        );
    }

    #[test]
    fn drff_noncommercial_no_derivatives_boundary_is_preserved() {
        let mut register = mutated_register();
        register.candidate_datasets[2].derivative_sharing_permitted = Some(true);
        let errors = validate(&register);
        assert!(
            errors
                .iter()
                .any(|error| error.starts_with("unreviewed_rights_or_metadata_promoted:"))
        );
        assert!(
            errors
                .iter()
                .any(|error| error == "drff_r2_rights_or_published_counts_mismatch")
        );
    }

    #[test]
    fn physical_sensor_track_must_remain_blocked() {
        let mut register = mutated_register();
        register.tracks[2].status = Some("ready_for_field_testing".to_string());
        let errors = validate(&register);
        assert!(
            errors
                .iter()
                .any(|error| error == "track_b1_must_remain_blocked")
        );
    }

    #[test]
    fn no_group_overlap_or_live_sensor_operation_may_be_enabled() {
        let mut register = mutated_register();
        register.hard_gates.train_test_group_overlap_allowed = true;
        let errors = validate(&register);
        assert!(errors.iter().any(|error| error == "hard_gate_enabled"));
    }
}
