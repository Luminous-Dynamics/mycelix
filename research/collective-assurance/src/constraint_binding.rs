//! Evidence-constraint binding for open-ended group assistance.
//!
//! This module checks whether every visible evidence item is explicitly
//! accounted for in a response. It is inspired by relational evidence
//! refinement and assistance constraint binding: retrieval is not equivalent
//! to correct use. The module validates a structured contract; it does not
//! parse natural-language responses or authenticate source metadata.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceStatus {
    Current,
    Superseded,
    Disputed,
    Expired,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Applicability {
    Applicable,
    NotApplicable,
    Conditional,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstraintHandling {
    Applied,
    Excluded,
    AppliedWithCondition,
    PreservedConflict,
    Unresolved,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AvailableEvidenceConstraint {
    pub request_id: String,
    pub evidence_id: String,
    pub source_digest: String,
    pub subject_id: String,
    pub scope_id: String,
    pub authorized_responder_id: String,
    pub status: EvidenceStatus,
    pub applicability: Applicability,
    pub verified: bool,
    /// If true, the response must explicitly account for this item.
    pub required: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ResponseConstraintUse {
    pub evidence_id: String,
    pub source_digest: String,
    pub subject_id: String,
    pub scope_id: String,
    pub status: EvidenceStatus,
    pub applicability: Applicability,
    pub handling: ConstraintHandling,
    /// Explains exclusions, unresolved state, conflict preservation, or conditions.
    pub rationale: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ResponseGrounding {
    pub request_id: String,
    pub response_id: String,
    pub responder_id: String,
    pub constraints: Vec<ResponseConstraintUse>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstraintQualificationStatus {
    Pass,
    Fail,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstraintQualificationReport {
    pub status: ConstraintQualificationStatus,
    pub accounted_for: usize,
    pub required_count: usize,
    pub errors: Vec<String>,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| {
            byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)
        })
}

fn needs_rationale(handling: ConstraintHandling) -> bool {
    matches!(
        handling,
        ConstraintHandling::Excluded
            | ConstraintHandling::PreservedConflict
            | ConstraintHandling::Unresolved
            | ConstraintHandling::AppliedWithCondition
    )
}

pub fn qualify_response_grounding(
    available: &[AvailableEvidenceConstraint],
    response: &ResponseGrounding,
) -> ConstraintQualificationReport {
    let mut errors = Vec::new();
    if !nonempty(&response.request_id) || !nonempty(&response.response_id)
        || !nonempty(&response.responder_id)
    {
        errors.push("request, response and responder identities must be non-empty".into());
    }
    if available.is_empty() {
        errors.push("response grounding requires at least one available evidence constraint".into());
    }

    let mut available_by_id = BTreeMap::new();
    let mut required_count = 0usize;
    for item in available {
        if !nonempty(&item.request_id)
            || item.request_id != response.request_id
            || !nonempty(&item.evidence_id)
            || !nonempty(&item.subject_id)
            || !nonempty(&item.scope_id)
            || !nonempty(&item.authorized_responder_id)
            || !sha256_hex(&item.source_digest)
        {
            errors.push(format!("available evidence {} has invalid identity, scope or digest", item.evidence_id));
        }
        if item.authorized_responder_id != response.responder_id {
            errors.push(format!("evidence {} is not authorized for this responder", item.evidence_id));
        }
        if item.required {
            required_count += 1;
        }
        if available_by_id.insert(item.evidence_id.as_str(), item).is_some() {
            errors.push(format!("duplicate available evidence id {}", item.evidence_id));
        }
    }

    let mut uses = BTreeSet::new();
    let mut accounted_for = 0usize;
    for used in &response.constraints {
        if !nonempty(&used.evidence_id) || !uses.insert(used.evidence_id.as_str()) {
            errors.push("response evidence uses must have unique non-empty evidence ids".into());
            continue;
        }
        let Some(item) = available_by_id.get(used.evidence_id.as_str()) else {
            errors.push(format!("response uses unknown or unavailable evidence {}", used.evidence_id));
            continue;
        };
        accounted_for += 1;
        if used.source_digest != item.source_digest
            || used.subject_id != item.subject_id
            || used.scope_id != item.scope_id
            || used.status != item.status
            || used.applicability != item.applicability
        {
            errors.push(format!("response changed the source, subject, scope, status or applicability of {}", used.evidence_id));
        }
        if needs_rationale(used.handling)
            && used.rationale.as_deref().is_none_or(|value| !nonempty(value))
        {
            errors.push(format!("handling {:?} for {} requires a rationale or condition", used.handling, used.evidence_id));
        }

        let valid_handling = if !item.verified {
            used.handling == ConstraintHandling::Unresolved
        } else {
            matches!(
                (item.status, item.applicability, used.handling),
                (
                    EvidenceStatus::Current,
                    Applicability::Applicable,
                    ConstraintHandling::Applied
                ) | (
                    EvidenceStatus::Current,
                    Applicability::NotApplicable,
                    ConstraintHandling::Excluded
                ) | (
                    EvidenceStatus::Current,
                    Applicability::Conditional,
                    ConstraintHandling::AppliedWithCondition
                ) | (
                    EvidenceStatus::Disputed,
                    _,
                    ConstraintHandling::PreservedConflict
                ) | (
                    EvidenceStatus::Superseded | EvidenceStatus::Expired,
                    _,
                    ConstraintHandling::Excluded
                ) | (
                    EvidenceStatus::Unknown,
                    _,
                    ConstraintHandling::Unresolved
                ) | (
                    _,
                    Applicability::Unknown,
                    ConstraintHandling::Unresolved
                )
            )
        };
        if !valid_handling {
            errors.push(format!(
                "handling {:?} is incompatible with verification/status/applicability for {}",
                used.handling, used.evidence_id
            ));
        }
    }

    for item in available.iter().filter(|item| item.required) {
        if !uses.contains(item.evidence_id.as_str()) {
            errors.push(format!("required evidence {} was retrieved but not accounted for", item.evidence_id));
        }
    }

    ConstraintQualificationReport {
        status: if errors.is_empty() {
            ConstraintQualificationStatus::Pass
        } else {
            ConstraintQualificationStatus::Fail
        },
        accounted_for,
        required_count,
        errors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest() -> String {
        "a".repeat(64)
    }

    fn available(
        status: EvidenceStatus,
        applicability: Applicability,
        verified: bool,
    ) -> AvailableEvidenceConstraint {
        AvailableEvidenceConstraint {
            request_id: "request-1".into(),
            evidence_id: "e-1".into(),
            source_digest: digest(),
            subject_id: "trip-date".into(),
            scope_id: "group-trip-2026".into(),
            authorized_responder_id: "assistant-1".into(),
            status,
            applicability,
            verified,
            required: true,
        }
    }

    fn use_item(
        status: EvidenceStatus,
        applicability: Applicability,
        handling: ConstraintHandling,
        rationale: Option<&str>,
    ) -> ResponseConstraintUse {
        ResponseConstraintUse {
            evidence_id: "e-1".into(),
            source_digest: digest(),
            subject_id: "trip-date".into(),
            scope_id: "group-trip-2026".into(),
            status,
            applicability,
            handling,
            rationale: rationale.map(str::to_owned),
        }
    }

    fn response(constraints: Vec<ResponseConstraintUse>) -> ResponseGrounding {
        ResponseGrounding {
            request_id: "request-1".into(),
            response_id: "response-1".into(),
            responder_id: "assistant-1".into(),
            constraints,
        }
    }

    #[test]
    fn evidence_from_another_request_cannot_be_reused() {
        let mut item = available(EvidenceStatus::Current, Applicability::Applicable, true);
        item.request_id = "another-request".into();
        let report = qualify_response_grounding(
            &[item],
            &response(vec![use_item(
                EvidenceStatus::Current,
                Applicability::Applicable,
                ConstraintHandling::Applied,
                None,
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Fail);
    }

    #[test]
    fn one_thousand_constraint_fault_injections_never_false_pass() {
        let valid_available = available(
            EvidenceStatus::Current,
            Applicability::Applicable,
            true,
        );
        let valid_use = use_item(
            EvidenceStatus::Current,
            Applicability::Applicable,
            ConstraintHandling::Applied,
            None,
        );
        for index in 0..1_000 {
            let mut item = valid_available.clone();
            let mut response_record = response(vec![valid_use.clone()]);
            match index % 10 {
                0 => item.request_id = "other-request".into(),
                1 => response_record.request_id = "other-request".into(),
                2 => item.source_digest = "b".repeat(64),
                3 => item.subject_id = "other-subject".into(),
                4 => item.scope_id = "other-scope".into(),
                5 => item.status = EvidenceStatus::Superseded,
                6 => item.applicability = Applicability::Conditional,
                7 => item.authorized_responder_id = "other-responder".into(),
                8 => item.verified = false,
                9 => response_record.constraints.clear(),
                _ => unreachable!(),
            }
            let report = qualify_response_grounding(&[item], &response_record);
            assert_eq!(
                report.status,
                ConstraintQualificationStatus::Fail,
                "fault injection {index} unexpectedly passed: {:?}",
                report.errors
            );
        }
    }

    #[test]
    fn current_applicable_verified_evidence_must_be_applied() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Applicable, true)],
            &response(vec![use_item(
                EvidenceStatus::Current,
                Applicability::Applicable,
                ConstraintHandling::Applied,
                None,
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Pass);
    }

    #[test]
    fn retrieved_required_evidence_cannot_be_silently_omitted() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Applicable, true)],
            &response(vec![]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Fail);
    }

    #[test]
    fn superseded_evidence_cannot_be_applied_as_current() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Superseded, Applicability::Applicable, true)],
            &response(vec![use_item(
                EvidenceStatus::Superseded,
                Applicability::Applicable,
                ConstraintHandling::Applied,
                None,
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Fail);
    }

    #[test]
    fn superseded_evidence_can_be_explicitly_excluded() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Superseded, Applicability::Applicable, true)],
            &response(vec![use_item(
                EvidenceStatus::Superseded,
                Applicability::Applicable,
                ConstraintHandling::Excluded,
                Some("newer record supersedes this value"),
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Pass);
    }

    #[test]
    fn conditional_evidence_requires_a_condition_binding() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Conditional, true)],
            &response(vec![use_item(
                EvidenceStatus::Current,
                Applicability::Conditional,
                ConstraintHandling::AppliedWithCondition,
                Some("applies only when the ferry is operating"),
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Pass);
    }

    #[test]
    fn disputed_evidence_requires_conflict_preservation() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Disputed, Applicability::Applicable, true)],
            &response(vec![use_item(
                EvidenceStatus::Disputed,
                Applicability::Applicable,
                ConstraintHandling::PreservedConflict,
                Some("two source records disagree; do not collapse them"),
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Pass);
    }

    #[test]
    fn unverified_evidence_must_remain_unresolved() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Applicable, false)],
            &response(vec![use_item(
                EvidenceStatus::Current,
                Applicability::Applicable,
                ConstraintHandling::Unresolved,
                Some("verification receipt is absent"),
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Pass);
    }

    #[test]
    fn responder_cannot_use_evidence_outside_its_authorized_view() {
        let mut item = available(EvidenceStatus::Current, Applicability::Applicable, true);
        item.authorized_responder_id = "another-agent".into();
        let report = qualify_response_grounding(
            &[item],
            &response(vec![use_item(
                EvidenceStatus::Current,
                Applicability::Applicable,
                ConstraintHandling::Applied,
                None,
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Fail);
    }

    #[test]
    fn response_cannot_change_source_scope_or_status_metadata() {
        let mut use_record = use_item(
            EvidenceStatus::Current,
            Applicability::Applicable,
            ConstraintHandling::Applied,
            None,
        );
        use_record.scope_id = "another-project".into();
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Applicable, true)],
            &response(vec![use_record]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Fail);
    }

    #[test]
    fn unknown_applicability_must_remain_unresolved() {
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Unknown, true)],
            &response(vec![use_item(
                EvidenceStatus::Current,
                Applicability::Unknown,
                ConstraintHandling::Unresolved,
                Some("scope cannot be established"),
            )]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Pass);
    }

    #[test]
    fn duplicate_response_evidence_is_rejected() {
        let use_record = use_item(
            EvidenceStatus::Current,
            Applicability::Applicable,
            ConstraintHandling::Applied,
            None,
        );
        let report = qualify_response_grounding(
            &[available(EvidenceStatus::Current, Applicability::Applicable, true)],
            &response(vec![use_record.clone(), use_record]),
        );
        assert_eq!(report.status, ConstraintQualificationStatus::Fail);
    }
}
