//! Strict verifier for a two-track, civilian-only awareness comparison manifest.
//!
//! This validates comparison eligibility semantics and safety/privacy boundaries.
//! It does not ingest sensor feeds, track aircraft, estimate hostile intent,
//! command devices, test physical sensor performance, or establish CWA conformity.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const SCHEMA: &str = "luminous.gcpn.awareness_industry_comparison.v0.1";
pub const STATUS: &str = "synthetic_comparison_protocol_only";
pub const CLAIM_CEILING: &str = "comparison_manifest_integrity_and_synthetic_track_semantics_only";
pub const EXPECTED_FIXTURE_SHA256: &str =
    "4a66a0d631580eeba82b567821b0be692e2857ff1317b677945362a7d08a1173";

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub schema: String,
    pub status: String,
    pub claim_ceiling: String,
    pub created_date: String,
    pub parent_issue: String,
    pub child_issue: String,
    pub standard_reference: StandardReference,
    pub tracks: Vec<Track>,
    pub baselines: Vec<Baseline>,
    pub matching_controls: MatchingControls,
    pub required_track_a_metrics: Vec<String>,
    pub required_track_b_metrics: Vec<String>,
    pub proposed_target: ProposedTarget,
    pub hard_gates: HardGates,
    pub cases: Vec<Case>,
    pub professional_review: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StandardReference {
    pub id: String,
    pub title: String,
    pub public_scope: String,
    pub defeat_disable_in_scope: bool,
    pub authorized_text_required_for_conformity_claim: bool,
    pub luminous_conformity_claim: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Track {
    pub id: String,
    pub kind: String,
    pub status: String,
    pub same_frozen_event_envelopes_required: bool,
    pub same_ground_truth_set_required: bool,
    pub identical_physical_sensor_inputs_required: bool,
    pub scores_physical_sensor_detection: bool,
    pub field_permission_required: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Baseline {
    pub id: String,
    pub vendor: String,
    pub product: String,
    pub architecture: String,
    pub source_url: String,
    pub source_access_date: String,
    pub version_status: String,
    pub public_claims: Vec<String>,
    pub exclusions: Vec<String>,
    pub independent_performance_verified: bool,
    pub vendor_hostile_label_authoritative: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MatchingControls {
    pub required_fields: Vec<String>,
    pub unmatched_disposition: String,
    pub operator_location_allowed: bool,
    pub exact_flight_path_allowed: bool,
    pub raw_audio_video_allowed: bool,
    pub private_communications_allowed: bool,
    pub active_mitigation_allowed: bool,
    pub third_party_control_allowed: bool,
    pub vendor_hostile_label_authoritative: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProposedTarget {
    pub relative_duplicate_unsubstantiated_alert_and_review_workload_reduction_percent: u8,
    pub semantic_state_conformance_non_degradation_required: bool,
    pub privacy_safety_and_authority_violations_may_increase: bool,
    pub status: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardGates {
    pub active_mitigation_count: u64,
    pub third_party_control_count: u64,
    pub autonomous_neutralization_count: u64,
    pub operator_geolocation_export_count: u64,
    pub vendor_hostile_label_to_ground_truth_count: u64,
    pub raw_audio_video_export_count: u64,
    pub privacy_or_retention_violation_count: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    pub id: String,
    pub name: String,
    pub expected_disposition: String,
    pub assertions: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub status: &'static str,
    pub source_sha256: String,
    pub case_count: usize,
    pub baseline_count: usize,
    pub errors: Vec<String>,
}

const EXPECTED_TRACK_A: &str = "common_event_envelope_replay";
const EXPECTED_TRACK_B: &str = "physical_sensor_performance";

const EXPECTED_BASELINES: [&str; 4] = ["D0", "D1", "D2", "D3"];

const EXPECTED_CASES: [(&str, &str); 10] = [
    ("AIC-001", "ELIGIBLE_FOR_TRACK_A"),
    ("AIC-002", "INCOMPARABLE"),
    ("AIC-003", "BLOCKED"),
    ("AIC-004", "CLAIM_LIMITED"),
    ("AIC-005", "BLOCKED"),
    ("AIC-006", "BLOCKED"),
    ("AIC-007", "BLOCKED"),
    ("AIC-008", "BLOCKED"),
    ("AIC-009", "UNKNOWN"),
    ("AIC-010", "BLOCKED"),
];

const REQUIRED_A_METRICS: [&str; 13] = [
    "schema_and_provenance_completeness",
    "identity_authentication_authorization_separation",
    "calibration_and_source_health_validity",
    "event_time_receipt_time_and_staleness",
    "missingness_and_disagreement_preservation",
    "common_mode_dependency_completeness",
    "correct_abstention_and_selective_risk",
    "duplicate_and_unsubstantiated_alert_burden",
    "human_review_latency_p50_p95",
    "human_review_minutes_per_case",
    "privacy_minimization_and_disclosure_violations",
    "offline_reconciliation_and_correction_lineage",
    "deterministic_replay_reproducibility",
];

const REQUIRED_B_METRICS: [&str; 11] = [
    "event_precision_and_recall_with_ground_truth_denominator",
    "false_alerts_per_monitor_hour",
    "false_alerts_per_site_day",
    "unknown_ambiguous_and_missed_events",
    "correct_abstention",
    "per_modality_environmental_stratum_performance",
    "sensor_uptime_calibration_and_stale_input_rate",
    "detection_to_human_review_latency_p50_p95",
    "operator_workload_and_adjudication_time",
    "privacy_and_retention_violations",
    "lifecycle_cost_per_monitored_site_hour",
];

const REQUIRED_MATCH_FIELDS: [&str; 11] = [
    "site_and_jurisdiction_profile",
    "exact_product_and_configuration",
    "event_set_and_ground_truth_revision",
    "eligible_denominator",
    "modality_coverage_and_missingness",
    "observation_window",
    "environmental_context_strata",
    "operator_training_and_review_budget",
    "event_matching_and_deduplication_policy",
    "privacy_and_data_rights_policy",
    "currency_and_price_base_year",
];

const REQUIRED_REVIEW: [&str; 8] = [
    "SACAA_and_aviation_legal",
    "ICASA_spectrum_and_type_approval",
    "CWA_18150_authorized_standards_reviewer",
    "site_operator_and_vendor_OEM",
    "privacy_POPIA_and_community",
    "independent_ground_truth_adjudicator",
    "site_safety_and_insurance",
    "quantity_surveyor_or_economist",
];

pub fn verify(bytes: &[u8]) -> Report {
    let source_sha256 = format!("{:x}", Sha256::digest(bytes));
    let mut errors = Vec::new();
    if source_sha256 != EXPECTED_FIXTURE_SHA256 {
        errors.push(format!(
            "fixture_digest_mismatch: expected {}, got {}",
            EXPECTED_FIXTURE_SHA256, source_sha256
        ));
    }

    let mut case_count = 0usize;
    let mut baseline_count = 0usize;

    match serde_json::from_slice::<Manifest>(bytes) {
        Err(error) => errors.push(format!("strict_json_decode: {error}")),
        Ok(manifest) => {
            case_count = manifest.cases.len();
            baseline_count = manifest.baselines.len();

            if manifest.schema != SCHEMA {
                errors.push("schema_mismatch".into());
            }
            if manifest.status != STATUS {
                errors.push("status_mismatch".into());
            }
            if manifest.claim_ceiling != CLAIM_CEILING {
                errors.push("claim_ceiling_mismatch".into());
            }

            if manifest.standard_reference.id != "CWA_18150_2024"
                || manifest.standard_reference.defeat_disable_in_scope
                || !manifest
                    .standard_reference
                    .authorized_text_required_for_conformity_claim
                || manifest.standard_reference.luminous_conformity_claim
            {
                errors.push("standard_reference_or_conformity_claim_violation".into());
            }

            if manifest.tracks.len() != 2 {
                errors.push("track_count_mismatch".into());
            }
            let track_a = manifest.tracks.iter().find(|track| track.id == "TRACK_A");
            let track_b = manifest.tracks.iter().find(|track| track.id == "TRACK_B");

            match track_a {
                None => errors.push("track_a_missing".into()),
                Some(track) => {
                    if track.kind != EXPECTED_TRACK_A
                        || track.status != "synthetic_protocol_ready"
                        || !track.same_frozen_event_envelopes_required
                        || !track.same_ground_truth_set_required
                        || track.identical_physical_sensor_inputs_required
                        || track.scores_physical_sensor_detection
                        || track.field_permission_required
                    {
                        errors.push("track_a_semantics_mismatch".into());
                    }
                }
            }

            match track_b {
                None => errors.push("track_b_missing".into()),
                Some(track) => {
                    if track.kind != EXPECTED_TRACK_B
                        || track.status
                            != "blocked_pending_site_legal_professional_and_ground_truth_approval"
                        || track.same_frozen_event_envelopes_required
                        || !track.same_ground_truth_set_required
                        || track.identical_physical_sensor_inputs_required
                        || !track.scores_physical_sensor_detection
                        || !track.field_permission_required
                    {
                        errors.push("track_b_gate_or_semantics_mismatch".into());
                    }
                }
            }

            let baseline_ids: BTreeSet<&str> = manifest
                .baselines
                .iter()
                .map(|baseline| baseline.id.as_str())
                .collect();
            if baseline_ids.len() != EXPECTED_BASELINES.len()
                || EXPECTED_BASELINES
                    .iter()
                    .any(|id| !baseline_ids.contains(id))
            {
                errors.push("baseline_set_mismatch".into());
            }

            for baseline in &manifest.baselines {
                if baseline.vendor.trim().is_empty()
                    || baseline.product.trim().is_empty()
                    || baseline.source_url.trim().is_empty()
                    || baseline.source_access_date != "2026-11-08"
                    || baseline.version_status.trim().is_empty()
                    || baseline.public_claims.is_empty()
                {
                    errors.push(format!("baseline_metadata_incomplete: {}", baseline.id));
                }
                if baseline.independent_performance_verified {
                    errors.push(format!(
                        "vendor_performance_misrepresented_as_independent: {}",
                        baseline.id
                    ));
                }
                if baseline.vendor_hostile_label_authoritative {
                    errors.push(format!(
                        "vendor_hostile_label_authority_enabled: {}",
                        baseline.id
                    ));
                }
                if baseline.exclusions.is_empty() {
                    errors.push(format!("baseline_exclusions_missing: {}", baseline.id));
                }
            }

            let required_fields: BTreeSet<&str> = manifest
                .matching_controls
                .required_fields
                .iter()
                .map(String::as_str)
                .collect();
            if required_fields.len() != REQUIRED_MATCH_FIELDS.len()
                || REQUIRED_MATCH_FIELDS
                    .iter()
                    .any(|field| !required_fields.contains(field))
                || manifest.matching_controls.unmatched_disposition != "INCOMPARABLE"
            {
                errors.push("matching_contract_incomplete".into());
            }

            let controls = &manifest.matching_controls;
            if controls.operator_location_allowed
                || controls.exact_flight_path_allowed
                || controls.raw_audio_video_allowed
                || controls.private_communications_allowed
                || controls.active_mitigation_allowed
                || controls.third_party_control_allowed
                || controls.vendor_hostile_label_authoritative
            {
                errors.push("comparison_data_or_authority_boundary_violation".into());
            }

            let a_metrics: BTreeSet<&str> = manifest
                .required_track_a_metrics
                .iter()
                .map(String::as_str)
                .collect();
            if a_metrics.len() != REQUIRED_A_METRICS.len()
                || REQUIRED_A_METRICS
                    .iter()
                    .any(|metric| !a_metrics.contains(metric))
            {
                errors.push("track_a_metrics_incomplete".into());
            }

            let b_metrics: BTreeSet<&str> = manifest
                .required_track_b_metrics
                .iter()
                .map(String::as_str)
                .collect();
            if b_metrics.len() != REQUIRED_B_METRICS.len()
                || REQUIRED_B_METRICS
                    .iter()
                    .any(|metric| !b_metrics.contains(metric))
            {
                errors.push("track_b_metrics_incomplete".into());
            }

            let target = &manifest.proposed_target;
            if target.relative_duplicate_unsubstantiated_alert_and_review_workload_reduction_percent
                != 20
                || !target.semantic_state_conformance_non_degradation_required
                || target.privacy_safety_and_authority_violations_may_increase
                || target.status != "falsifiable_hypothesis_only_not_a_result"
            {
                errors.push("proposed_target_claim_ceiling_mismatch".into());
            }

            let gates = &manifest.hard_gates;
            if gates.active_mitigation_count != 0
                || gates.third_party_control_count != 0
                || gates.autonomous_neutralization_count != 0
                || gates.operator_geolocation_export_count != 0
                || gates.vendor_hostile_label_to_ground_truth_count != 0
                || gates.raw_audio_video_export_count != 0
                || gates.privacy_or_retention_violation_count != 0
            {
                errors.push("hard_gate_nonzero".into());
            }

            let review: BTreeSet<&str> = manifest
                .professional_review
                .iter()
                .map(String::as_str)
                .collect();
            if review.len() != REQUIRED_REVIEW.len()
                || REQUIRED_REVIEW.iter().any(|item| !review.contains(item))
            {
                errors.push("professional_review_requirements_incomplete".into());
            }

            if manifest.cases.len() != EXPECTED_CASES.len() {
                errors.push(format!(
                    "case_count_mismatch: expected {}, got {}",
                    EXPECTED_CASES.len(),
                    manifest.cases.len()
                ));
            }

            let mut seen = BTreeSet::new();
            for case in &manifest.cases {
                if case.id.trim().is_empty() || !seen.insert(case.id.as_str()) {
                    errors.push(format!("empty_or_duplicate_case_id: {}", case.id));
                }
                if case.name.trim().is_empty() || case.assertions.is_empty() {
                    errors.push(format!("case_metadata_incomplete: {}", case.id));
                }
                let assertions: BTreeSet<&str> =
                    case.assertions.iter().map(String::as_str).collect();
                if assertions.len() != case.assertions.len() {
                    errors.push(format!("duplicate_assertion: {}", case.id));
                }

                match EXPECTED_CASES.iter().find(|(id, _)| *id == case.id) {
                    None => errors.push(format!("unexpected_case: {}", case.id)),
                    Some((_, expected)) => {
                        if case.expected_disposition != *expected {
                            errors.push(format!("case_disposition_mismatch: {}", case.id));
                        }
                    }
                }
            }

            for (id, _) in EXPECTED_CASES {
                if !seen.contains(id) {
                    errors.push(format!("missing_case: {id}"));
                }
            }
        }
    }

    errors.sort();
    Report {
        status: if errors.is_empty() {
            "CONFORMS"
        } else {
            "REJECTED"
        },
        source_sha256,
        case_count,
        baseline_count,
        errors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!(
        "../../../mycelix-workspace/mycelix-praxis/docs/research/GCPN_AWARENESS_INDUSTRY_COMPARISON_CASES_V0_1.json"
    );

    #[test]
    fn frozen_comparison_manifest_conforms() {
        let report = verify(FIXTURE.as_bytes());
        assert_eq!(report.status, "CONFORMS", "{:?}", report.errors);
        assert_eq!(report.case_count, 10);
        assert_eq!(report.baseline_count, 4);
        assert_eq!(report.source_sha256, EXPECTED_FIXTURE_SHA256);
    }

    #[test]
    fn unknown_root_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"status\": \"synthetic_comparison_protocol_only\"",
            "\"status\": \"synthetic_comparison_protocol_only\", \"unknown_root\": true",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn duplicate_known_json_member_is_rejected() {
        let changed = FIXTURE.replace(
            "\"active_mitigation_count\": 0",
            "\"active_mitigation_count\": 0, \"active_mitigation_count\": 1",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn physical_sensor_track_cannot_be_promoted_to_ready() {
        let changed = FIXTURE.replace(
            "blocked_pending_site_legal_professional_and_ground_truth_approval",
            "synthetic_protocol_ready",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error == "track_b_gate_or_semantics_mismatch")
        );
    }

    #[test]
    fn track_a_cannot_claim_physical_sensor_detection() {
        let changed = FIXTURE.replace(
            "\"scores_physical_sensor_detection\": false",
            "\"scores_physical_sensor_detection\": true",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error == "track_a_semantics_mismatch")
        );
    }

    #[test]
    fn vendor_hostile_label_cannot_be_authoritative() {
        let changed = FIXTURE.replace(
            "\"vendor_hostile_label_authoritative\": false",
            "\"vendor_hostile_label_authoritative\": true",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|error| {
            error == "comparison_data_or_authority_boundary_violation"
                || error.starts_with("vendor_hostile_label_authority_enabled:")
        }));
    }

    #[test]
    fn operator_location_export_is_rejected() {
        let changed = FIXTURE.replace(
            "\"operator_location_allowed\": false",
            "\"operator_location_allowed\": true",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| { error == "comparison_data_or_authority_boundary_violation" })
        );
    }

    #[test]
    fn nonzero_hard_gate_is_rejected() {
        let changed = FIXTURE.replace(
            "\"privacy_or_retention_violation_count\": 0",
            "\"privacy_or_retention_violation_count\": 1",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error == "hard_gate_nonzero")
        );
    }

    #[test]
    fn independent_vendor_performance_cannot_be_fabricated() {
        let changed = FIXTURE.replacen(
            "\"independent_performance_verified\": false",
            "\"independent_performance_verified\": true",
            1,
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|error| {
            error.starts_with("vendor_performance_misrepresented_as_independent:")
        }));
    }

    #[test]
    fn standard_conformity_claim_cannot_be_enabled_without_review() {
        let changed = FIXTURE.replace(
            "\"luminous_conformity_claim\": false",
            "\"luminous_conformity_claim\": true",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| { error == "standard_reference_or_conformity_claim_violation" })
        );
    }

    #[test]
    fn fixture_byte_change_is_rejected_by_digest() {
        let changed = FIXTURE.replace(
            "matched_track_a_event_envelopes",
            "matched_track_a_event_envelopes ",
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| { error.starts_with("fixture_digest_mismatch:") })
        );
    }
}
