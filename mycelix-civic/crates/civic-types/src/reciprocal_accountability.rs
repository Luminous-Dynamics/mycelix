// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Reciprocal accountability primitives for person-linked lookups.
//!
//! The core invariant is intentionally structural: a person-linked lookup can
//! be notified immediately or notification can be delayed under an expiring,
//! independently approved authorization. There is no `NeverNotify` state.
//!
//! These types are transport- and storage-agnostic. A Holochain zome can keep
//! the full receipt private to the subject and authorized oversight peers while
//! publishing only commitments/aggregate transparency statistics.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Version of the reciprocal-accountability schema in this module.
pub const RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION: u16 = 1;

/// A pairwise/pseudonymous subject identifier.
///
/// Implementations SHOULD derive this separately for each institutional
/// relationship so the identifier cannot become a universal tracking handle.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub struct PairwiseSubjectId(pub String);

/// The principal that attempted a person-linked lookup.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct RequestingPrincipal {
    pub organization_id: String,
    pub actor_id: String,
    pub role: String,
}

/// Legal or consensual authority asserted for a lookup.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct LegalAuthority {
    pub authority_type: AuthorityType,
    pub authority_id: String,
    pub jurisdiction: String,
}

/// Broad authority categories. Exact legal meaning is supplied by the
/// jurisdiction-specific policy profile, not this protocol crate.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum AuthorityType {
    Consent,
    Statute,
    CourtOrder,
    Warrant,
    Emergency,
    Contractual,
    Other,
}

/// Purpose and scope bound to a lookup authorization.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct PurposeBinding {
    pub purpose_code: String,
    pub plain_language_purpose: String,
    pub matter_id: Option<String>,
    /// Digest of the precise predicate/data scope authorized for the lookup.
    pub scope_digest: String,
    pub expires_at_ms: u64,
}

/// Result of the authorization decision. Denied attempts still produce a
/// receipt because attempted misuse is itself accountability-relevant.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum LookupOutcome {
    Allowed,
    PartiallyAllowed,
    Denied,
}

/// Minimum-disclosure shape returned by a successful lookup.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum DisclosureKind {
    None,
    PredicateOnly,
    RedactedRecord,
    FullRecord,
}

/// Human- and machine-readable summary of what left the data holder.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct DisclosureSummary {
    pub kind: DisclosureKind,
    pub data_classes: Vec<String>,
    pub item_count: u32,
    /// Commitment to the disclosed result, without embedding the result itself.
    pub result_digest: Option<String>,
}

/// A charge against a query/privacy budget, preventing reconstruction of a
/// population through a large number of individually permissible lookups.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct QueryBudgetCharge {
    pub budget_id: String,
    pub units: u32,
    pub remaining_units: u32,
    pub window_ends_at_ms: u64,
}

/// Disclosure about a machine-generated inference concerning the subject.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct InferenceDisclosure {
    pub inference_id: String,
    pub model_id: String,
    pub model_version: String,
    pub plain_language_summary: String,
    /// Integer confidence in parts-per-million, avoiding float canonicalization.
    pub confidence_ppm: u32,
    pub explanation_digest: String,
    pub provenance_receipt_ids: Vec<String>,
    pub significant_effect: bool,
}

/// Reference to a cryptographic proof/attestation produced by another layer
/// (for example Symthaea's ZK stack or an Xenia-signed execution transcript).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct AttestationRef {
    pub scheme: String,
    pub statement_digest: String,
    pub proof_digest: String,
    pub verifier_profile: String,
}

/// Rights advertised by the receipt to its subject.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub enum SubjectRight {
    Know,
    Inspect,
    Contest,
    HumanReview,
    Appeal,
    ProofOfPolicy,
}

/// Narrow, standardized reasons for temporarily delaying subject notice.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub enum DelayReason {
    ActiveInvestigation,
    PreventCrime,
    ProtectVictimOrWitness,
    PublicSafety,
    NationalSecurity,
}

/// One approval of a delayed-notification authorization.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct DelayApproval {
    pub organization_id: String,
    pub approver_id: String,
    pub approval_digest: String,
}

/// Expiring authorization to withhold a notification temporarily.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct DelayedNotificationAuthorization {
    pub authorization_id: String,
    pub reason: DelayReason,
    /// Jurisdiction-specific statute/order/policy reference.
    pub legal_basis: String,
    /// Commitment to the sealed justification reviewed by approvers.
    pub justification_digest: String,
    pub approved_at_ms: u64,
    pub release_at_ms: u64,
    pub required_approvals: u8,
    pub approvals: Vec<DelayApproval>,
}

/// Notification mode for a person-linked lookup.
///
/// Deliberately has no permanent-secret variant.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum NotificationDirective {
    Immediate,
    Delayed(DelayedNotificationAuthorization),
}

/// Canonical accountability record for every attempted person-linked lookup.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct AccessReceipt {
    pub protocol_version: u16,
    pub receipt_id: String,
    pub subject: PairwiseSubjectId,
    pub requester: RequestingPrincipal,
    pub purpose: PurposeBinding,
    pub legal_authority: LegalAuthority,
    pub query_digest: String,
    pub policy_version: String,
    pub occurred_at_ms: u64,
    pub outcome: LookupOutcome,
    pub disclosure: DisclosureSummary,
    pub notification: NotificationDirective,
    pub rights: Vec<SubjectRight>,
    pub query_budget_charge: Option<QueryBudgetCharge>,
    pub inference: Option<InferenceDisclosure>,
    pub attestation: Option<AttestationRef>,
}

/// Jurisdiction/deployment profile used to enforce the protocol's invariants.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct AccountabilityPolicy {
    /// Maximum permissible notice delay. `0` disables delayed notice.
    pub max_delay_ms: u64,
    pub min_delay_approvals: u8,
    pub require_approval_outside_requester_org: bool,
    pub permitted_delay_reasons: Vec<DelayReason>,
    pub required_subject_rights: Vec<SubjectRight>,
    pub require_query_budget_charge: bool,
    pub require_attestation: bool,
}

/// Runtime disposition after validating an access receipt.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum NotificationDisposition {
    DeliverNow,
    EscrowUntil {
        release_at_ms: u64,
        authorization_id: String,
    },
}

/// Fail-closed validation failures for reciprocal accountability.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum AccountabilityError {
    UnsupportedProtocolVersion,
    MissingReceiptId,
    MissingSubjectId,
    MissingRequester,
    MissingPurpose,
    MissingLegalAuthority,
    MissingQueryDigest,
    MissingPolicyVersion,
    PurposeExpiredBeforeLookup,
    InvalidInferenceConfidence,
    DeniedLookupDisclosedData,
    MissingRequiredRight(SubjectRight),
    MissingQueryBudgetCharge,
    MissingAttestation,
    DelayedNoticeDisabled,
    DelayReasonNotPermitted,
    MissingDelayAuthorization,
    InvalidDelayWindow,
    DelayExceedsPolicy,
    DelayAuthorizationAfterLookup,
    InsufficientDelayApprovals,
    DuplicateDelayApprover,
    MissingIndependentApproval,
}

/// Validate a receipt and determine whether notice must be delivered now or
/// can remain in cryptographic escrow until its mandatory release time.
pub fn evaluate_notification(
    receipt: &AccessReceipt,
    now_ms: u64,
    policy: &AccountabilityPolicy,
) -> Result<NotificationDisposition, AccountabilityError> {
    validate_receipt(receipt, policy)?;

    match &receipt.notification {
        NotificationDirective::Immediate => Ok(NotificationDisposition::DeliverNow),
        NotificationDirective::Delayed(auth) if now_ms >= auth.release_at_ms => {
            Ok(NotificationDisposition::DeliverNow)
        }
        NotificationDirective::Delayed(auth) => Ok(NotificationDisposition::EscrowUntil {
            release_at_ms: auth.release_at_ms,
            authorization_id: auth.authorization_id.clone(),
        }),
    }
}

/// Validate all structural and policy invariants of a person-linked lookup
/// receipt. Callers should reject the lookup if they cannot durably commit a
/// valid receipt before releasing protected data.
pub fn validate_receipt(
    receipt: &AccessReceipt,
    policy: &AccountabilityPolicy,
) -> Result<(), AccountabilityError> {
    if receipt.protocol_version != RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION {
        return Err(AccountabilityError::UnsupportedProtocolVersion);
    }
    if receipt.receipt_id.trim().is_empty() {
        return Err(AccountabilityError::MissingReceiptId);
    }
    if receipt.subject.0.trim().is_empty() {
        return Err(AccountabilityError::MissingSubjectId);
    }
    if receipt.requester.organization_id.trim().is_empty()
        || receipt.requester.actor_id.trim().is_empty()
        || receipt.requester.role.trim().is_empty()
    {
        return Err(AccountabilityError::MissingRequester);
    }
    if receipt.purpose.purpose_code.trim().is_empty()
        || receipt.purpose.plain_language_purpose.trim().is_empty()
        || receipt.purpose.scope_digest.trim().is_empty()
    {
        return Err(AccountabilityError::MissingPurpose);
    }
    if receipt.legal_authority.authority_id.trim().is_empty()
        || receipt.legal_authority.jurisdiction.trim().is_empty()
    {
        return Err(AccountabilityError::MissingLegalAuthority);
    }
    if receipt.query_digest.trim().is_empty() {
        return Err(AccountabilityError::MissingQueryDigest);
    }
    if receipt.policy_version.trim().is_empty() {
        return Err(AccountabilityError::MissingPolicyVersion);
    }
    if receipt.purpose.expires_at_ms < receipt.occurred_at_ms {
        return Err(AccountabilityError::PurposeExpiredBeforeLookup);
    }

    if let Some(inference) = &receipt.inference {
        if inference.confidence_ppm > 1_000_000 {
            return Err(AccountabilityError::InvalidInferenceConfidence);
        }
    }

    if receipt.outcome == LookupOutcome::Denied
        && (receipt.disclosure.kind != DisclosureKind::None
            || receipt.disclosure.item_count != 0
            || !receipt.disclosure.data_classes.is_empty()
            || receipt.disclosure.result_digest.is_some())
    {
        return Err(AccountabilityError::DeniedLookupDisclosedData);
    }

    let rights: BTreeSet<_> = receipt.rights.iter().cloned().collect();
    for required in &policy.required_subject_rights {
        if !rights.contains(required) {
            return Err(AccountabilityError::MissingRequiredRight(required.clone()));
        }
    }

    if policy.require_query_budget_charge && receipt.query_budget_charge.is_none() {
        return Err(AccountabilityError::MissingQueryBudgetCharge);
    }
    if policy.require_attestation && receipt.attestation.is_none() {
        return Err(AccountabilityError::MissingAttestation);
    }

    if let NotificationDirective::Delayed(auth) = &receipt.notification {
        validate_delay(receipt, auth, policy)?;
    }

    Ok(())
}

fn validate_delay(
    receipt: &AccessReceipt,
    auth: &DelayedNotificationAuthorization,
    policy: &AccountabilityPolicy,
) -> Result<(), AccountabilityError> {
    if policy.max_delay_ms == 0 {
        return Err(AccountabilityError::DelayedNoticeDisabled);
    }
    if !policy.permitted_delay_reasons.contains(&auth.reason) {
        return Err(AccountabilityError::DelayReasonNotPermitted);
    }
    if auth.authorization_id.trim().is_empty()
        || auth.legal_basis.trim().is_empty()
        || auth.justification_digest.trim().is_empty()
    {
        return Err(AccountabilityError::MissingDelayAuthorization);
    }
    if auth.release_at_ms <= receipt.occurred_at_ms || auth.release_at_ms <= auth.approved_at_ms {
        return Err(AccountabilityError::InvalidDelayWindow);
    }
    if auth.release_at_ms - receipt.occurred_at_ms > policy.max_delay_ms {
        return Err(AccountabilityError::DelayExceedsPolicy);
    }
    // Prevent a secret lookup from being retroactively blessed after it ran.
    if auth.approved_at_ms > receipt.occurred_at_ms {
        return Err(AccountabilityError::DelayAuthorizationAfterLookup);
    }

    let required = usize::from(auth.required_approvals.max(policy.min_delay_approvals));
    if required == 0 || auth.approvals.len() < required {
        return Err(AccountabilityError::InsufficientDelayApprovals);
    }

    let mut approvers = BTreeSet::new();
    for approval in &auth.approvals {
        if approval.approver_id.trim().is_empty()
            || approval.organization_id.trim().is_empty()
            || approval.approval_digest.trim().is_empty()
        {
            return Err(AccountabilityError::MissingDelayAuthorization);
        }
        let identity = (approval.organization_id.clone(), approval.approver_id.clone());
        if !approvers.insert(identity) {
            return Err(AccountabilityError::DuplicateDelayApprover);
        }
    }

    if policy.require_approval_outside_requester_org
        && !auth
            .approvals
            .iter()
            .any(|approval| approval.organization_id != receipt.requester.organization_id)
    {
        return Err(AccountabilityError::MissingIndependentApproval);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const DAY_MS: u64 = 24 * 60 * 60 * 1000;

    fn rights() -> Vec<SubjectRight> {
        vec![
            SubjectRight::Know,
            SubjectRight::Inspect,
            SubjectRight::Contest,
            SubjectRight::HumanReview,
            SubjectRight::Appeal,
            SubjectRight::ProofOfPolicy,
        ]
    }

    fn policy() -> AccountabilityPolicy {
        AccountabilityPolicy {
            max_delay_ms: 30 * DAY_MS,
            min_delay_approvals: 2,
            require_approval_outside_requester_org: true,
            permitted_delay_reasons: vec![
                DelayReason::ActiveInvestigation,
                DelayReason::ProtectVictimOrWitness,
                DelayReason::PublicSafety,
            ],
            required_subject_rights: rights(),
            require_query_budget_charge: true,
            require_attestation: true,
        }
    }

    fn immediate_receipt() -> AccessReceipt {
        let occurred_at_ms = 1_800_000_000_000;
        AccessReceipt {
            protocol_version: RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
            receipt_id: "receipt-001".into(),
            subject: PairwiseSubjectId("pairwise:subject:42".into()),
            requester: RequestingPrincipal {
                organization_id: "agency-a".into(),
                actor_id: "operator-key-7".into(),
                role: "investigator".into(),
            },
            purpose: PurposeBinding {
                purpose_code: "stolen-vehicle".into(),
                plain_language_purpose: "Locate a reported stolen vehicle".into(),
                matter_id: Some("case-123".into()),
                scope_digest: "sha256:scope".into(),
                expires_at_ms: occurred_at_ms + DAY_MS,
            },
            legal_authority: LegalAuthority {
                authority_type: AuthorityType::Statute,
                authority_id: "law:example:17".into(),
                jurisdiction: "example-jurisdiction".into(),
            },
            query_digest: "sha256:query".into(),
            policy_version: "ra-policy-1".into(),
            occurred_at_ms,
            outcome: LookupOutcome::Allowed,
            disclosure: DisclosureSummary {
                kind: DisclosureKind::PredicateOnly,
                data_classes: vec!["vehicle-presence".into()],
                item_count: 1,
                result_digest: Some("sha256:result".into()),
            },
            notification: NotificationDirective::Immediate,
            rights: rights(),
            query_budget_charge: Some(QueryBudgetCharge {
                budget_id: "budget-1".into(),
                units: 1,
                remaining_units: 99,
                window_ends_at_ms: occurred_at_ms + DAY_MS,
            }),
            inference: None,
            attestation: Some(AttestationRef {
                scheme: "zk-or-signed-transcript".into(),
                statement_digest: "sha256:statement".into(),
                proof_digest: "sha256:proof".into(),
                verifier_profile: "sif-v0.1".into(),
            }),
        }
    }

    fn delayed_receipt() -> AccessReceipt {
        let mut receipt = immediate_receipt();
        receipt.notification = NotificationDirective::Delayed(DelayedNotificationAuthorization {
            authorization_id: "delay-001".into(),
            reason: DelayReason::ActiveInvestigation,
            legal_basis: "court-order:456".into(),
            justification_digest: "sha256:sealed-justification".into(),
            approved_at_ms: receipt.occurred_at_ms - 1_000,
            release_at_ms: receipt.occurred_at_ms + 7 * DAY_MS,
            required_approvals: 2,
            approvals: vec![
                DelayApproval {
                    organization_id: "agency-a".into(),
                    approver_id: "supervisor-1".into(),
                    approval_digest: "sha256:approval-a".into(),
                },
                DelayApproval {
                    organization_id: "independent-court".into(),
                    approver_id: "judge-9".into(),
                    approval_digest: "sha256:approval-b".into(),
                },
            ],
        });
        receipt
    }

    #[test]
    fn immediate_lookup_must_notify_now() {
        let receipt = immediate_receipt();
        assert_eq!(
            evaluate_notification(&receipt, receipt.occurred_at_ms, &policy()),
            Ok(NotificationDisposition::DeliverNow)
        );
    }

    #[test]
    fn valid_delayed_notice_is_escrowed_only_until_expiry() {
        let receipt = delayed_receipt();
        let auth = match &receipt.notification {
            NotificationDirective::Delayed(auth) => auth,
            NotificationDirective::Immediate => unreachable!(),
        };
        assert_eq!(
            evaluate_notification(&receipt, receipt.occurred_at_ms, &policy()),
            Ok(NotificationDisposition::EscrowUntil {
                release_at_ms: auth.release_at_ms,
                authorization_id: auth.authorization_id.clone(),
            })
        );
        assert_eq!(
            evaluate_notification(&receipt, auth.release_at_ms, &policy()),
            Ok(NotificationDisposition::DeliverNow)
        );
    }

    #[test]
    fn delayed_notice_requires_independent_approval() {
        let mut receipt = delayed_receipt();
        if let NotificationDirective::Delayed(auth) = &mut receipt.notification {
            auth.approvals[1].organization_id = "agency-a".into();
        }
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::MissingIndependentApproval)
        );
    }

    #[test]
    fn delay_cannot_exceed_policy_maximum() {
        let mut receipt = delayed_receipt();
        if let NotificationDirective::Delayed(auth) = &mut receipt.notification {
            auth.release_at_ms = receipt.occurred_at_ms + 31 * DAY_MS;
        }
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::DelayExceedsPolicy)
        );
    }

    #[test]
    fn delay_cannot_be_authorized_after_lookup() {
        let mut receipt = delayed_receipt();
        if let NotificationDirective::Delayed(auth) = &mut receipt.notification {
            auth.approved_at_ms = receipt.occurred_at_ms + 1;
        }
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::DelayAuthorizationAfterLookup)
        );
    }

    #[test]
    fn denied_lookup_cannot_smuggle_a_disclosure() {
        let mut receipt = immediate_receipt();
        receipt.outcome = LookupOutcome::Denied;
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::DeniedLookupDisclosedData)
        );
    }

    #[test]
    fn required_subject_rights_are_fail_closed() {
        let mut receipt = immediate_receipt();
        receipt.rights.retain(|right| *right != SubjectRight::Contest);
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::MissingRequiredRight(
                SubjectRight::Contest
            ))
        );
    }

    #[test]
    fn identifiable_lookup_can_require_query_budgeting() {
        let mut receipt = immediate_receipt();
        receipt.query_budget_charge = None;
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::MissingQueryBudgetCharge)
        );
    }

    #[test]
    fn inference_confidence_is_canonical_and_bounded() {
        let mut receipt = immediate_receipt();
        receipt.inference = Some(InferenceDisclosure {
            inference_id: "inference-1".into(),
            model_id: "symthaea-model".into(),
            model_version: "v1".into(),
            plain_language_summary: "Vehicle may be associated with the subject".into(),
            confidence_ppm: 1_000_001,
            explanation_digest: "sha256:explanation".into(),
            provenance_receipt_ids: vec!["receipt-prior".into()],
            significant_effect: true,
        });
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::InvalidInferenceConfidence)
        );
    }
}
