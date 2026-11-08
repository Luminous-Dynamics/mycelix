//! Strict contract verifier for the civilian-only autonomous-system awareness corpus.
//!
//! The verifier validates the schema and declared authority/safety semantics. It is
//! not a sensor fusion engine, aircraft tracker, robot controller, threat classifier,
//! interception system, or real-world safety assessor.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const SCHEMA: &str = "luminous.gcpn.civil_autonomous_awareness.v0.1";
pub const STATUS: &str = "synthetic_event_awareness_only";
pub const CLAIM_CEILING: &str = "frozen_synthetic_semantics_only";

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub schema: String,
    pub status: String,
    pub claim_ceiling: String,
    pub scope: String,
    pub authority: Authority,
    pub prohibited_capabilities: Vec<String>,
    pub modalities: Vec<Modality>,
    pub allowed_states: Vec<String>,
    pub required_metrics: Vec<String>,
    pub hard_gates: HardGates,
    pub cases: Vec<Case>,
    pub professional_review: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Authority {
    pub third_party_aircraft_command: bool,
    pub third_party_robot_command: bool,
    pub autonomous_neutralization: bool,
    pub weapon_or_effector_control: bool,
    pub hostile_intent_inference_from_presence: bool,
    pub site_owned_machine_safety_owner: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Modality {
    pub id: String,
    pub role: String,
    pub privacy: String,
    pub effectors_allowed: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardGates {
    pub third_party_command_count: u64,
    pub autonomous_neutralization_count: u64,
    pub presence_to_hostile_escalation_count: u64,
    pub missing_rf_to_aircraft_absent_claim_count: u64,
    pub unconsented_person_identification_count: u64,
    pub unauthorized_public_disclosure_count: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    pub id: String,
    pub name: String,
    pub domain: String,
    pub expected_state: String,
    pub assertions: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub report_schema: &'static str,
    pub status: &'static str,
    pub source_sha256: String,
    pub case_count: usize,
    pub errors: Vec<String>,
    pub claim_ceiling: &'static str,
}

const EXPECTED_CASES: [(&str, &str, &[&str]); 12] = [
    ("CAA-001", "AUTHORIZED_CLAIM_PENDING_SOURCE_VALIDATION", &["identity_claim_is_not_authentication", "authorization_source_and_validity_required", "no_hostile_intent_inference"]),
    ("CAA-002", "HUMAN_REVIEW_REQUIRED", &["emergency_service_claim_preserved", "missing_metadata_remains_unknown", "no_automatic_interference"]),
    ("CAA-003", "OBSERVED_INTENT_UNKNOWN", &["observation_separated_from_intent", "human_review_required", "notification_only"]),
    ("CAA-004", "OBSERVED_CLASS_AMBIGUOUS", &["classification_uncertainty_preserved", "no_hostile_label_from_anomaly", "false_escalation_negative_control"]),
    ("CAA-005", "UNKNOWN_RF_SILENT_OR_OUT_OF_BAND", &["rf_sensor_scope_limited_to_observable_emissions", "missing_rf_is_not_absence", "no_universal_detection_claim"]),
    ("CAA-006", "STALE_UNKNOWN", &["observation_time_preserved", "stale_not_current", "no_automatic_escalation"]),
    ("CAA-007", "SOURCE_UNHEALTHY", &["signature_not_physical_truth", "calibration_state_separate", "no_confidence_promotion"]),
    ("CAA-008", "COMMON_MODE_UNKNOWN", &["sensor_count_not_independence", "dependency_inventory_required", "unknown_when_inventory_incomplete"]),
    ("CAA-009", "IDENTITY_REVOKED_REVIEW_REQUIRED", &["revocation_and_policy_epoch_checked", "stale_identity_not_authorized", "no_third_party_command"]),
    ("CAA-010", "OWNER_TASK_AUTHORIZATION_EXPIRED_HOLD", &["task_scope_and_expiry_checked", "site_owner_and_oem_safety_authority_preserved", "no_new_actuator_authority"]),
    ("CAA-011", "THIRD_PARTY_CONTROL_PROHIBITED", &["no_command_or_takeover_path", "notify_accountable_operator_or_authority", "record_only_authorized_observations"]),
    ("CAA-012", "DATA_USE_BLOCKED", &["purpose_and_recipient_policy_required", "no_intelligible_speech_retention", "privacy_disclosure_gate_applied"]),
];

const EXPECTED_PROHIBITED: [&str; 10] = [
    "high_energy_laser_effectors",
    "harmful_acoustic_or_ultrasonic_effectors",
    "psychotronic_or_involuntary_neuromodulation",
    "rf_jamming_or_gnss_spoofing",
    "third_party_protocol_takeover_or_command_injection",
    "kinetic_or_autonomous_interception",
    "target_selection_or_engagement_control",
    "operator_geolocation_or_person_tracking",
    "facial_recognition_or_person_identification",
    "tactical_sensor_placement_or_defeat_optimization",
];

const REQUIRED_STATES: [&str; 12] = [
    "AUTHORIZED_CLAIM_PENDING_SOURCE_VALIDATION",
    "OBSERVED_INTENT_UNKNOWN",
    "OBSERVED_CLASS_AMBIGUOUS",
    "UNKNOWN_RF_SILENT_OR_OUT_OF_BAND",
    "STALE_UNKNOWN",
    "SOURCE_UNHEALTHY",
    "COMMON_MODE_UNKNOWN",
    "IDENTITY_REVOKED_REVIEW_REQUIRED",
    "OWNER_TASK_AUTHORIZATION_EXPIRED_HOLD",
    "THIRD_PARTY_CONTROL_PROHIBITED",
    "DATA_USE_BLOCKED",
    "HUMAN_REVIEW_REQUIRED",
];

const REQUIRED_METRICS: [&str; 15] = [
    "event_precision_and_recall",
    "false_alerts_per_monitored_hour",
    "detection_to_human_review_latency_p50_p95",
    "calibration_and_selective_risk",
    "unknown_ambiguous_rate_and_correct_abstention",
    "sensor_health_and_stale_input_detection",
    "common_mode_dependency_sensitivity",
    "provenance_completeness",
    "false_escalation_count",
    "unauthorized_command_count",
    "notification_delivery_and_accessibility",
    "human_review_burden",
    "privacy_and_retention_violations",
    "service_downtime_and_verified_recovery",
    "full_lifecycle_cost_per_comparable_service_hour",
];

pub fn verify(bytes: &[u8]) -> Report {
    let source_sha256 = format!("{:x}", Sha256::digest(bytes));
    let mut errors = Vec::new();
    let mut case_count = 0usize;

    match serde_json::from_slice::<Manifest>(bytes) {
        Err(error) => errors.push(format!("strict_json_decode: {error}")),
        Ok(manifest) => {
            case_count = manifest.cases.len();

            if manifest.schema != SCHEMA { errors.push("schema_mismatch".into()); }
            if manifest.status != STATUS { errors.push("status_mismatch".into()); }
            if manifest.claim_ceiling != CLAIM_CEILING { errors.push("claim_ceiling_mismatch".into()); }
            if manifest.scope != "civilian_passive_awareness_warning_and_resilience" {
                errors.push("scope_mismatch".into());
            }

            let a = &manifest.authority;
            if a.third_party_aircraft_command
                || a.third_party_robot_command
                || a.autonomous_neutralization
                || a.weapon_or_effector_control
                || a.hostile_intent_inference_from_presence
                || a.site_owned_machine_safety_owner != "OEM_and_competent_site_authority"
            {
                errors.push("authority_boundary_violation".into());
            }

            let prohibited: BTreeSet<&str> = manifest.prohibited_capabilities.iter().map(String::as_str).collect();
            if prohibited.len() != EXPECTED_PROHIBITED.len()
                || EXPECTED_PROHIBITED.iter().any(|item| !prohibited.contains(item))
            {
                errors.push("prohibited_capability_set_mismatch".into());
            }

            let modality_ids: BTreeSet<&str> = manifest.modalities.iter().map(|m| m.id.as_str()).collect();
            let expected_modalities: BTreeSet<&str> = [
                "OPTICAL", "RADAR", "RF", "ACOUSTIC", "LIDAR", "ROBOT_ONBOARD_SAFETY",
            ].into_iter().collect();
            if modality_ids.len() != 6 || modality_ids != expected_modalities {
                errors.push("modality_set_mismatch".into());
            }
            for modality in &manifest.modalities {
                if modality.effectors_allowed {
                    errors.push(format!("effector_capability_enabled: {}", modality.id));
                }
            }
            if manifest.modalities.iter().any(|m|
                m.id == "ACOUSTIC" && m.privacy != "no_intelligible_speech_retention"
            ) {
                errors.push("acoustic_privacy_boundary_changed".into());
            }
            if manifest.modalities.iter().any(|m|
                m.id == "LIDAR" && m.role != "OEM_certified_eye_safe_measurement"
            ) {
                errors.push("laser_measurement_safety_boundary_changed".into());
            }

            let state_set: BTreeSet<&str> = manifest.allowed_states.iter().map(String::as_str).collect();
            if state_set.len() != REQUIRED_STATES.len()
                || REQUIRED_STATES.iter().any(|s| !state_set.contains(s))
            {
                errors.push("allowed_state_set_mismatch".into());
            }

            let metric_set: BTreeSet<&str> = manifest.required_metrics.iter().map(String::as_str).collect();
            if metric_set.len() != REQUIRED_METRICS.len()
                || REQUIRED_METRICS.iter().any(|m| !metric_set.contains(m))
            {
                errors.push("required_metric_set_mismatch".into());
            }

            let gates = &manifest.hard_gates;
            if gates.third_party_command_count != 0
                || gates.autonomous_neutralization_count != 0
                || gates.presence_to_hostile_escalation_count != 0
                || gates.missing_rf_to_aircraft_absent_claim_count != 0
                || gates.unconsented_person_identification_count != 0
                || gates.unauthorized_public_disclosure_count != 0
            {
                errors.push("hard_gate_nonzero".into());
            }

            if manifest.professional_review.len() < 8 {
                errors.push("professional_review_requirements_incomplete".into());
            }

            if manifest.cases.len() != EXPECTED_CASES.len() {
                errors.push(format!("case_count_mismatch: expected {}, got {}", EXPECTED_CASES.len(), manifest.cases.len()));
            }

            let mut seen = BTreeSet::new();
            for case in &manifest.cases {
                if case.id.trim().is_empty() || !seen.insert(case.id.as_str()) {
                    errors.push(format!("empty_or_duplicate_case_id: {}", case.id));
                }
                if case.name.trim().is_empty() || case.domain.trim().is_empty() {
                    errors.push(format!("incomplete_case: {}", case.id));
                }
                let actual: BTreeSet<&str> = case.assertions.iter().map(String::as_str).collect();
                if actual.len() != case.assertions.len() || actual.is_empty() {
                    errors.push(format!("duplicate_or_empty_assertions: {}", case.id));
                }

                match EXPECTED_CASES.iter().find(|(id, _, _)| *id == case.id) {
                    None => errors.push(format!("unexpected_case: {}", case.id)),
                    Some((_, expected_state, required)) => {
                        if case.expected_state != *expected_state {
                            errors.push(format!("state_mismatch: {}", case.id));
                        }
                        for assertion in *required {
                            if !actual.contains(assertion) {
                                errors.push(format!("required_assertion_missing: {} -> {}", case.id, assertion));
                            }
                        }
                    }
                }
            }
            for (id, _, _) in EXPECTED_CASES {
                if !seen.contains(id) {
                    errors.push(format!("missing_case: {id}"));
                }
            }
        }
    }

    errors.sort();
    Report {
        report_schema: "luminous.gcpn.civil_autonomous_awareness_report.v0.1",
        status: if errors.is_empty() { "CONFORMS" } else { "REJECTED" },
        source_sha256,
        case_count,
        errors,
        claim_ceiling: "synthetic_event_awareness_semantics_only",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!(
        "../../../mycelix-workspace/mycelix-praxis/docs/research/GCPN_CIV_AUTONOMOUS_AWARENESS_CASES_V0_1.json"
    );

    #[test]
    fn frozen_civilian_awareness_fixture_conforms() {
        let report = verify(FIXTURE.as_bytes());
        assert_eq!(report.status, "CONFORMS", "{:?}", report.errors);
        assert_eq!(report.case_count, 12);
        assert_eq!(report.source_sha256.len(), 64);
    }

    #[test]
    fn unknown_root_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"status\": \"synthetic_event_awareness_only\"",
            "\"status\": \"synthetic_event_awareness_only\", \"unknown_root\": true"
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e.starts_with("strict_json_decode:")));
    }

    #[test]
    fn unknown_nested_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"id\": \"OPTICAL\"",
            "\"id\": \"OPTICAL\", \"unknown_nested\": true"
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e.starts_with("strict_json_decode:")));
    }

    #[test]
    fn duplicate_json_member_is_rejected() {
        let changed = FIXTURE.replace(
            "\"third_party_aircraft_command\": false",
            "\"third_party_aircraft_command\": false, \"third_party_aircraft_command\": true"
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e.starts_with("strict_json_decode:")));
    }

    #[test]
    fn presence_cannot_be_promoted_to_hostile_intent() {
        let changed = FIXTURE.replace(
            "\"expected_state\": \"OBSERVED_INTENT_UNKNOWN\"",
            "\"expected_state\": \"HOSTILE_CONFIRMED\""
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e == "state_mismatch: CAA-003"));
    }

    #[test]
    fn rf_silence_cannot_be_promoted_to_aircraft_absence() {
        let changed = FIXTURE.replace(
            "\"expected_state\": \"UNKNOWN_RF_SILENT_OR_OUT_OF_BAND\"",
            "\"expected_state\": \"NO_AIRCRAFT_PRESENT\""
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e == "state_mismatch: CAA-005"));
    }

    #[test]
    fn third_party_control_boundary_cannot_be_enabled() {
        let changed = FIXTURE.replace(
            "\"third_party_robot_command\": false",
            "\"third_party_robot_command\": true"
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e == "authority_boundary_violation"));
    }

    #[test]
    fn modality_cannot_enable_effectors() {
        let changed = FIXTURE.replace(
            "\"effectors_allowed\": false",
            "\"effectors_allowed\": true"
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e.starts_with("effector_capability_enabled:")));
    }

    #[test]
    fn duplicate_case_id_is_rejected() {
        let changed = FIXTURE.replace(
            "\"id\": \"CAA-012\"",
            "\"id\": \"CAA-011\""
        );
        assert_ne!(changed, FIXTURE);
        let report = verify(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.errors.iter().any(|e| e.starts_with("empty_or_duplicate_case_id:")));
        assert!(report.errors.iter().any(|e| e == "missing_case: CAA-012"));
    }
}
