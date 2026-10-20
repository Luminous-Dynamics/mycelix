// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Policy admissibility is deliberately separate from the claim-ceiling meet.
//! A mathematically conservative meet cannot repair an over-permissive policy.

use super::{
    ClaimCeilingV1, TimeWindow, is_no_broader_than, sha256, validate_ceiling,
    validate_json_against_schema, validate_source_commit,
};
use serde::{Deserialize, Serialize};

const POLICY_REQUEST_SCHEMA: &[u8] =
    include_bytes!("../schemas/claim-ceiling-policy-request-v1.schema.json");
const POLICY_RECEIPT_SCHEMA: &[u8] =
    include_bytes!("../schemas/claim-ceiling-policy-receipt-v1.schema.json");

pub const POLICY_REQUEST_SCHEMA_VERSION: &str = "mycelix.claim-ceiling-policy.request.v1";
pub const POLICY_PROFILE_SCHEMA_VERSION: &str = "mycelix.claim-ceiling-policy.profile.v1";
pub const POLICY_RECEIPT_SCHEMA_VERSION: &str = "mycelix.claim-ceiling-policy.receipt.v1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClaimPolicyProfileV1 {
    pub schema_version: String,
    pub profile_id: String,
    /// Maximum allowed scope plus minimum mandatory controls for this profile.
    pub maximum_ceiling: ClaimCeilingV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyRequestV1 {
    pub schema_version: String,
    pub request_id: String,
    pub policy_profile: ClaimPolicyProfileV1,
    pub candidate_ceiling: ClaimCeilingV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolicyDisposition {
    AdmissibleWithinProfile,
    RejectedByProfile,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyViolation {
    /// Stable machine-readable dimension code; values are never inferred from scores.
    pub code: String,
    pub value: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyReceiptV1 {
    pub schema_version: String,
    pub request_id: String,
    pub profile_id: String,
    pub source_commit: String,
    pub source_commit_pinned: bool,
    pub input_sha256: String,
    pub policy_profile_sha256: String,
    pub candidate_ceiling_sha256: String,
    pub request_schema_sha256: String,
    pub receipt_schema_sha256: String,
    pub disposition: PolicyDisposition,
    pub violations: Vec<PolicyViolation>,
    /// This receipt qualifies profile fit only; it is never an admission or grant.
    pub policy_admissibility_only: bool,
    pub action_authority_granted: bool,
    pub nonclaims: Vec<String>,
}

const NONCLAIMS: &[&str] = &[
    "Policy profile fit does not establish claim truth, evidence authenticity, or source independence.",
    "A passing profile check is not canonical evidence admission, user consent, legal authority, or an action grant.",
    "Policy authors remain responsible for choosing a semantically safe allowlist and current authority root.",
    "A digest binds bytes but does not authenticate the person, source, policy publisher, or trusted time.",
];

fn violation(code: &str, value: impl Into<String>) -> PolicyViolation {
    PolicyViolation {
        code: code.to_owned(),
        value: value.into(),
    }
}

fn validate_profile(profile: &ClaimPolicyProfileV1) -> Result<(), String> {
    if profile.schema_version != POLICY_PROFILE_SCHEMA_VERSION {
        return Err(format!(
            "unsupported policy profile schema version: {}",
            profile.schema_version
        ));
    }
    if profile.profile_id.trim().is_empty() {
        return Err("policy_profile.profile_id must not be empty".to_owned());
    }
    validate_ceiling("policy_profile.maximum_ceiling", &profile.maximum_ceiling)
}

fn assess_candidate(candidate: &ClaimCeilingV1, maximum: &ClaimCeilingV1) -> Vec<PolicyViolation> {
    let mut violations = Vec::new();

    for atom in candidate.claim_atoms.difference(&maximum.claim_atoms) {
        violations.push(violation(
            "claim_atom_not_allowed",
            serde_json::to_string(atom).unwrap_or_else(|_| "serialization-error".to_owned()),
        ));
    }
    for subject in candidate.subject_ids.difference(&maximum.subject_ids) {
        violations.push(violation("subject_outside_policy", subject.clone()));
    }
    for context in candidate.context_ids.difference(&maximum.context_ids) {
        violations.push(violation("context_outside_policy", context.clone()));
    }
    if candidate.valid_time.start_unix_seconds < maximum.valid_time.start_unix_seconds
        || candidate.valid_time.end_unix_seconds_exclusive
            > maximum.valid_time.end_unix_seconds_exclusive
    {
        let requested = TimeWindow {
            start_unix_seconds: candidate.valid_time.start_unix_seconds,
            end_unix_seconds_exclusive: candidate.valid_time.end_unix_seconds_exclusive,
        };
        violations.push(violation(
            "time_window_outside_policy",
            format!(
                "{} not contained in {}",
                serde_json::to_string(&requested).unwrap_or_default(),
                serde_json::to_string(&maximum.valid_time).unwrap_or_default()
            ),
        ));
    }
    for action in candidate.action_scopes.difference(&maximum.action_scopes) {
        violations.push(violation("action_scope_not_allowed", action.clone()));
    }
    for qualification in maximum
        .required_qualifications
        .difference(&candidate.required_qualifications)
    {
        violations.push(violation(
            "required_qualification_missing",
            qualification.clone(),
        ));
    }
    for control in maximum
        .required_independence_controls
        .difference(&candidate.required_independence_controls)
    {
        violations.push(violation(
            "required_independence_control_missing",
            format!("{control:?}"),
        ));
    }
    for requirement in maximum
        .required_provenance
        .difference(&candidate.required_provenance)
    {
        violations.push(violation(
            "required_provenance_missing",
            format!("{requirement:?}"),
        ));
    }

    violations.sort();
    violations.dedup();
    violations
}

/// Verify that a candidate is no broader than an explicit, versioned allowlist profile.
///
/// This check is separate from `meet`: intersecting two over-permissive ceilings can
/// still yield an inadmissible combination if both inputs encode a forbidden atom.
pub fn evaluate_policy_request_bytes(
    input: &[u8],
    source_commit: &str,
) -> Result<PolicyReceiptV1, String> {
    let source_commit_pinned = validate_source_commit(source_commit)?;
    validate_json_against_schema(POLICY_REQUEST_SCHEMA, input, "policy request")?;
    let request: PolicyRequestV1 = serde_json::from_slice(input)
        .map_err(|error| format!("policy request deserialization failed: {error}"))?;

    if request.schema_version != POLICY_REQUEST_SCHEMA_VERSION {
        return Err(format!(
            "unsupported policy request schema version: {}",
            request.schema_version
        ));
    }
    if request.request_id.trim().is_empty() {
        return Err("request_id must not be empty".to_owned());
    }
    validate_profile(&request.policy_profile)?;
    validate_ceiling("candidate_ceiling", &request.candidate_ceiling)?;

    let mut violations = assess_candidate(
        &request.candidate_ceiling,
        &request.policy_profile.maximum_ceiling,
    );
    // Defence in depth: the dimension-level explanation and the shared partial-order
    // predicate must agree. A disagreement is an implementation error, never PASS.
    let no_broader = is_no_broader_than(
        &request.candidate_ceiling,
        &request.policy_profile.maximum_ceiling,
    );
    if no_broader != violations.is_empty() {
        return Err(
            "policy assessment invariant mismatch between dimension audit and no-broader predicate"
                .to_owned(),
        );
    }

    let profile_bytes = serde_json::to_vec(&request.policy_profile)
        .map_err(|error| format!("policy profile serialization failed: {error}"))?;
    let candidate_bytes = serde_json::to_vec(&request.candidate_ceiling)
        .map_err(|error| format!("candidate ceiling serialization failed: {error}"))?;
    let receipt = PolicyReceiptV1 {
        schema_version: POLICY_RECEIPT_SCHEMA_VERSION.to_owned(),
        request_id: request.request_id,
        profile_id: request.policy_profile.profile_id,
        source_commit: source_commit.to_owned(),
        source_commit_pinned,
        input_sha256: sha256(input),
        policy_profile_sha256: sha256(&profile_bytes),
        candidate_ceiling_sha256: sha256(&candidate_bytes),
        request_schema_sha256: sha256(POLICY_REQUEST_SCHEMA),
        receipt_schema_sha256: sha256(POLICY_RECEIPT_SCHEMA),
        disposition: if violations.is_empty() {
            PolicyDisposition::AdmissibleWithinProfile
        } else {
            PolicyDisposition::RejectedByProfile
        },
        violations: std::mem::take(&mut violations),
        policy_admissibility_only: true,
        action_authority_granted: false,
        nonclaims: NONCLAIMS.iter().map(|item| (*item).to_owned()).collect(),
    };
    let receipt_bytes = serde_json::to_vec(&receipt)
        .map_err(|error| format!("policy receipt serialization failed: {error}"))?;
    validate_json_against_schema(POLICY_RECEIPT_SCHEMA, &receipt_bytes, "policy receipt")?;
    Ok(receipt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::UNPINNED_LOCAL_RUN;
    use crate::{
        ClaimAtom, ClaimPredicate, DerivationKind, EvidenceClass, IndependenceControl, IntendedUse,
        ProvenanceRequirement, meet,
    };
    use serde_json::json;
    use std::collections::BTreeSet;

    fn atom(
        predicate: ClaimPredicate,
        evidence: EvidenceClass,
        derivation: DerivationKind,
        use_case: IntendedUse,
    ) -> ClaimAtom {
        ClaimAtom {
            predicate,
            evidence_class: evidence,
            derivation,
            intended_use: use_case,
        }
    }

    fn safe_ceiling() -> ClaimCeilingV1 {
        ClaimCeilingV1 {
            claim_atoms: BTreeSet::from([atom(
                ClaimPredicate::Measurement,
                EvidenceClass::InstrumentMeasurement,
                DerivationKind::Direct,
                IntendedUse::Research,
            )]),
            subject_ids: BTreeSet::from(["pump-17".to_owned()]),
            context_ids: BTreeSet::from(["zone-north".to_owned()]),
            valid_time: TimeWindow {
                start_unix_seconds: 100,
                end_unix_seconds_exclusive: 200,
            },
            action_scopes: BTreeSet::from(["publish-report".to_owned()]),
            required_qualifications: BTreeSet::from(["calibration-reviewed".to_owned()]),
            required_independence_controls: BTreeSet::from([
                IndependenceControl::DistinctAcquisition,
            ]),
            required_provenance: BTreeSet::from([ProvenanceRequirement::ExactArtifactDigest]),
        }
    }

    fn profile() -> ClaimPolicyProfileV1 {
        ClaimPolicyProfileV1 {
            schema_version: POLICY_PROFILE_SCHEMA_VERSION.to_owned(),
            profile_id: "mycelix.policy.instrument-measurement.research.v1".to_owned(),
            maximum_ceiling: safe_ceiling(),
        }
    }

    fn evaluate(candidate: ClaimCeilingV1) -> PolicyReceiptV1 {
        let request = PolicyRequestV1 {
            schema_version: POLICY_REQUEST_SCHEMA_VERSION.to_owned(),
            request_id: "policy-test-001".to_owned(),
            policy_profile: profile(),
            candidate_ceiling: candidate,
        };
        evaluate_policy_request_bytes(
            &serde_json::to_vec(&request).expect("serialize policy request"),
            UNPINNED_LOCAL_RUN,
        )
        .expect("valid policy request should emit a receipt")
    }

    #[test]
    fn policy_accepts_candidate_within_every_declared_ceiling_dimension() {
        let receipt = evaluate(safe_ceiling());
        assert_eq!(
            receipt.disposition,
            PolicyDisposition::AdmissibleWithinProfile
        );
        assert!(receipt.violations.is_empty());
        assert!(receipt.policy_admissibility_only);
        assert!(!receipt.action_authority_granted);
        assert!(!receipt.source_commit_pinned);
    }

    #[test]
    fn policy_rejects_forbidden_atom_even_when_a_meet_could_preserve_it() {
        let mut candidate = safe_ceiling();
        let forbidden = atom(
            ClaimPredicate::OwnershipClaim,
            EvidenceClass::SourceAssertion,
            DerivationKind::Direct,
            IntendedUse::OperationalExecution,
        );
        candidate.claim_atoms.insert(forbidden.clone());
        let raw_meet = meet(&candidate, &candidate);
        assert_eq!(raw_meet.0, crate::MeetDisposition::UsableIntersection);
        assert!(
            raw_meet
                .1
                .as_ref()
                .unwrap()
                .claim_atoms
                .contains(&forbidden),
            "the pure meet correctly preserves the shared atom but has no policy knowledge"
        );

        let receipt = evaluate(candidate);
        assert_eq!(receipt.disposition, PolicyDisposition::RejectedByProfile);
        assert!(
            receipt
                .violations
                .iter()
                .any(|v| v.code == "claim_atom_not_allowed")
        );
        // Rejection is scoped policy evidence only; it is not a truth/falsity result.
        assert!(receipt.policy_admissibility_only);
    }

    #[test]
    fn policy_rejects_each_widened_scope_dimension_and_dropped_obligation() {
        let mut candidate = safe_ceiling();
        candidate
            .subject_ids
            .insert("unregistered-subject".to_owned());
        candidate
            .context_ids
            .insert("unregistered-context".to_owned());
        candidate.valid_time = TimeWindow {
            start_unix_seconds: 90,
            end_unix_seconds_exclusive: 210,
        };
        candidate.action_scopes.insert("operate-pump".to_owned());
        candidate.required_qualifications.clear();
        candidate.required_independence_controls.clear();
        candidate.required_provenance.clear();

        let receipt = evaluate(candidate);
        let codes: BTreeSet<_> = receipt.violations.iter().map(|v| v.code.as_str()).collect();
        assert_eq!(receipt.disposition, PolicyDisposition::RejectedByProfile);
        for expected in [
            "subject_outside_policy",
            "context_outside_policy",
            "time_window_outside_policy",
            "action_scope_not_allowed",
            "required_qualification_missing",
            "required_independence_control_missing",
            "required_provenance_missing",
        ] {
            assert!(
                codes.contains(expected),
                "missing policy violation {expected}: {codes:?}"
            );
        }
    }

    #[test]
    fn policy_receipt_is_deterministic_and_binds_raw_input_and_profile() {
        let request = PolicyRequestV1 {
            schema_version: POLICY_REQUEST_SCHEMA_VERSION.to_owned(),
            request_id: "stable-policy-fixture".to_owned(),
            policy_profile: profile(),
            candidate_ceiling: safe_ceiling(),
        };
        let bytes = serde_json::to_vec(&request).expect("serialize request");
        let first = evaluate_policy_request_bytes(&bytes, UNPINNED_LOCAL_RUN).unwrap();
        let second = evaluate_policy_request_bytes(&bytes, UNPINNED_LOCAL_RUN).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.input_sha256, sha256(&bytes));
        assert_eq!(
            first.policy_profile_sha256,
            sha256(&serde_json::to_vec(&request.policy_profile).unwrap())
        );
        assert_eq!(
            first.candidate_ceiling_sha256,
            sha256(&serde_json::to_vec(&request.candidate_ceiling).unwrap())
        );
    }

    #[test]
    fn policy_profile_rejects_unknown_fields_and_invalid_time_window() {
        let mut value = serde_json::to_value(PolicyRequestV1 {
            schema_version: POLICY_REQUEST_SCHEMA_VERSION.to_owned(),
            request_id: "malformed-policy".to_owned(),
            policy_profile: profile(),
            candidate_ceiling: safe_ceiling(),
        })
        .unwrap();
        value["policy_profile"]["unexpected_authority"] = json!(true);
        assert!(
            evaluate_policy_request_bytes(
                &serde_json::to_vec(&value).unwrap(),
                UNPINNED_LOCAL_RUN
            )
            .is_err()
        );

        let mut invalid = PolicyRequestV1 {
            schema_version: POLICY_REQUEST_SCHEMA_VERSION.to_owned(),
            request_id: "invalid-window".to_owned(),
            policy_profile: profile(),
            candidate_ceiling: safe_ceiling(),
        };
        invalid.policy_profile.maximum_ceiling.valid_time = TimeWindow {
            start_unix_seconds: 200,
            end_unix_seconds_exclusive: 200,
        };
        assert!(
            evaluate_policy_request_bytes(
                &serde_json::to_vec(&invalid).unwrap(),
                UNPINNED_LOCAL_RUN
            )
            .is_err()
        );
    }
}
