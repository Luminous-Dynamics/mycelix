// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Cross-domain reciprocal-accountability primitives for person-linked lookups.
//!
//! The core invariant is intentionally structural: a person-linked lookup can
//! be notified immediately or notification can be delayed under an expiring,
//! independently approved authorization. There is no `NeverNotify` state.
//!
//! These types are transport- and storage-agnostic. Full receipts should remain
//! private to the subject, the originating institution, and authorized oversight
//! peers; public transparency surfaces should publish commitments and aggregates,
//! not person/case identifiers.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

/// Version of the reciprocal-accountability schema in this crate.
pub const RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION: u16 = 1;

/// Hash algorithm used by v1 canonical commitments.
pub const ACCOUNTABILITY_COMMITMENT_ALGORITHM: &str = "blake3-256";

const RECEIPT_COMMITMENT_DOMAIN: &[u8] = b"mycelix:accountability-receipt:pre-attestation:v1";
const PURPOSE_COMMITMENT_DOMAIN: &[u8] = b"mycelix:accountability-purpose:v1";
const POLICY_COMMITMENT_DOMAIN: &[u8] = b"mycelix:accountability-policy:v1";

/// Fixed-size cryptographic commitment used across the Mycelix/Xenia/Symthaea
/// accountability boundary.
#[derive(
    Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash, Default,
)]
pub struct Commitment32(pub [u8; 32]);

impl Commitment32 {
    /// Construct a commitment from raw 32-byte digest bytes.
    pub const fn new(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    /// Return true for the all-zero placeholder, which v1 rejects for required
    /// security-relevant commitments.
    pub const fn is_zero(self) -> bool {
        self.0 == [0u8; 32]
    }

    /// Borrow the raw digest bytes.
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl From<[u8; 32]> for Commitment32 {
    fn from(value: [u8; 32]) -> Self {
        Self(value)
    }
}

/// A pairwise/pseudonymous subject identifier.
///
/// Implementations SHOULD derive this separately for each institutional
/// relationship so the identifier cannot become a universal tracking handle.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub struct PairwiseSubjectId(pub String);

/// The principal that attempted a person-linked lookup.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct RequestingPrincipal {
    /// Organization on whose authority the actor is operating.
    pub organization_id: String,
    /// Human/service identifier meaningful inside that organization.
    pub actor_id: String,
    /// Role asserted for policy evaluation.
    pub role: String,
    /// Xenia/authentication-plane source identifier or equivalent stable
    /// cryptographic principal fingerprint. This is what binds the semantic
    /// Mycelix requester to the authenticated execution transcript.
    pub authenticated_source_id: Commitment32,
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
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
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
    pub scope_digest: Commitment32,
    pub expires_at_ms: u64,
}

/// Result of the authorization decision. Denied attempts still produce a
/// receipt because attempted misuse is itself accountability-relevant.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum LookupOutcome {
    Allowed,
    PartiallyAllowed,
    Denied,
}

/// Minimum-disclosure shape returned by a successful lookup.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
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
    pub result_digest: Option<Commitment32>,
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
    pub explanation_digest: Commitment32,
    pub provenance_receipt_ids: Vec<String>,
    pub significant_effect: bool,
}

/// Reference to a cryptographic proof/attestation produced by another layer
/// (for example Symthaea's ZK stack or an Xenia-signed execution transcript).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct AttestationRef {
    pub scheme: String,
    pub statement_digest: Commitment32,
    pub proof_digest: Commitment32,
    pub verifier_profile: String,
}

/// Rights advertised by the receipt to its subject.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
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
    pub approval_digest: Commitment32,
}

/// Expiring authorization to withhold a notification temporarily.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct DelayedNotificationAuthorization {
    pub authorization_id: String,
    pub reason: DelayReason,
    /// Jurisdiction-specific statute/order/policy reference.
    pub legal_basis: String,
    /// Commitment to the sealed justification reviewed by approvers.
    pub justification_digest: Commitment32,
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
    /// Canonical commitment to the lookup/predicate itself.
    pub query_digest: Commitment32,
    pub policy_version: String,
    pub occurred_at_ms: u64,
    pub outcome: LookupOutcome,
    pub disclosure: DisclosureSummary,
    pub notification: NotificationDirective,
    pub rights: Vec<SubjectRight>,
    pub query_budget_charge: Option<QueryBudgetCharge>,
    pub inference: Option<InferenceDisclosure>,
    /// Added only after the pre-attestation receipt commitment has been formed.
    /// See [`pre_attestation_receipt_commitment`] for the deliberate exclusion
    /// that avoids a receipt ↔ execution-attestation hash cycle.
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

/// How much requester identity is copied into the initial human-facing notice.
/// Full inspection rights may expose more under the governing policy; this
/// controls only the push/inbox notification projection.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum RequesterNoticeLevel {
    OrganizationOnly,
    OrganizationAndRole,
    FullPrincipal,
}

/// Privacy policy for projecting an internal receipt into a subject notice.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SubjectNoticePolicy {
    pub requester_level: RequesterNoticeLevel,
    pub include_authority_id: bool,
    pub include_inference_summary: bool,
}

impl Default for SubjectNoticePolicy {
    fn default() -> Self {
        Self {
            requester_level: RequesterNoticeLevel::OrganizationAndRole,
            include_authority_id: false,
            include_inference_summary: true,
        }
    }
}

/// Requester projection safe for an initial subject-facing notice.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SubjectNoticeRequester {
    pub organization_id: String,
    pub actor_id: Option<String>,
    pub role: Option<String>,
}

/// Machine-inference projection safe for the initial notice surface.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SubjectInferenceNotice {
    pub model_id: String,
    pub model_version: String,
    pub plain_language_summary: String,
    pub confidence_ppm: u32,
    pub significant_effect: bool,
}

/// Delayed-notification metadata shown once an embargo expires. Approver
/// identities and sealed justification material are intentionally not copied
/// into the push/inbox projection.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct DelayedNoticeSummary {
    pub authorization_id: String,
    pub reason: DelayReason,
    pub approved_at_ms: u64,
    pub release_at_ms: u64,
}

/// Minimum safe notification projection derived from an internal receipt.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct SubjectNotice {
    pub receipt_id: String,
    pub occurred_at_ms: u64,
    pub requester: SubjectNoticeRequester,
    pub plain_language_purpose: String,
    pub authority_type: AuthorityType,
    pub authority_id: Option<String>,
    pub jurisdiction: String,
    pub outcome: LookupOutcome,
    pub disclosure: DisclosureSummary,
    pub rights: Vec<SubjectRight>,
    pub inference: Option<SubjectInferenceNotice>,
    pub delayed_notice: Option<DelayedNoticeSummary>,
    pub attestation: Option<AttestationRef>,
}

/// Project a full internal receipt into the smaller initial notice shown to the
/// subject. This prevents the notification channel itself from leaking case IDs,
/// raw query commitments, approver identities, or sealed delay justifications.
pub fn project_subject_notice(
    receipt: &AccessReceipt,
    policy: &SubjectNoticePolicy,
) -> SubjectNotice {
    let requester = match policy.requester_level {
        RequesterNoticeLevel::OrganizationOnly => SubjectNoticeRequester {
            organization_id: receipt.requester.organization_id.clone(),
            actor_id: None,
            role: None,
        },
        RequesterNoticeLevel::OrganizationAndRole => SubjectNoticeRequester {
            organization_id: receipt.requester.organization_id.clone(),
            actor_id: None,
            role: Some(receipt.requester.role.clone()),
        },
        RequesterNoticeLevel::FullPrincipal => SubjectNoticeRequester {
            organization_id: receipt.requester.organization_id.clone(),
            actor_id: Some(receipt.requester.actor_id.clone()),
            role: Some(receipt.requester.role.clone()),
        },
    };

    let inference = if policy.include_inference_summary {
        receipt.inference.as_ref().map(|inference| SubjectInferenceNotice {
            model_id: inference.model_id.clone(),
            model_version: inference.model_version.clone(),
            plain_language_summary: inference.plain_language_summary.clone(),
            confidence_ppm: inference.confidence_ppm,
            significant_effect: inference.significant_effect,
        })
    } else {
        None
    };

    let delayed_notice = match &receipt.notification {
        NotificationDirective::Immediate => None,
        NotificationDirective::Delayed(auth) => Some(DelayedNoticeSummary {
            authorization_id: auth.authorization_id.clone(),
            reason: auth.reason,
            approved_at_ms: auth.approved_at_ms,
            release_at_ms: auth.release_at_ms,
        }),
    };

    SubjectNotice {
        receipt_id: receipt.receipt_id.clone(),
        occurred_at_ms: receipt.occurred_at_ms,
        requester,
        plain_language_purpose: receipt.purpose.plain_language_purpose.clone(),
        authority_type: receipt.legal_authority.authority_type,
        authority_id: policy
            .include_authority_id
            .then(|| receipt.legal_authority.authority_id.clone()),
        jurisdiction: receipt.legal_authority.jurisdiction.clone(),
        outcome: receipt.outcome,
        disclosure: receipt.disclosure.clone(),
        rights: receipt.rights.clone(),
        inference,
        delayed_notice,
        attestation: receipt.attestation.clone(),
    }
}

/// Compute a stable v1 commitment to an access receipt *before* an execution
/// attestation is attached.
///
/// The attestation field is deliberately normalized to `None` before
/// serialization. Xenia signs this receipt commitment, and the resulting Xenia
/// proof reference is then inserted into `AccessReceipt.attestation`; excluding
/// that one field is what prevents a cryptographic hash cycle. Verifiers must
/// recompute this same pre-attestation commitment from the final receipt and
/// compare it with the Xenia execution binding.
pub fn pre_attestation_receipt_commitment(
    receipt: &AccessReceipt,
) -> Result<Commitment32, AccountabilityCommitmentError> {
    let mut body = receipt.clone();
    body.attestation = None;
    hash_serialized(RECEIPT_COMMITMENT_DOMAIN, &body)
}

/// Compute the purpose/scope commitment carried into the Xenia execution
/// binding.
pub fn purpose_commitment(
    purpose: &PurposeBinding,
) -> Result<Commitment32, AccountabilityCommitmentError> {
    hash_serialized(PURPOSE_COMMITMENT_DOMAIN, purpose)
}

/// Compute the policy commitment carried into the Xenia execution binding.
pub fn policy_commitment(
    policy: &AccountabilityPolicy,
) -> Result<Commitment32, AccountabilityCommitmentError> {
    hash_serialized(POLICY_COMMITMENT_DOMAIN, policy)
}

fn hash_serialized<T: Serialize>(
    domain: &[u8],
    value: &T,
) -> Result<Commitment32, AccountabilityCommitmentError> {
    let encoded = bincode::serialize(value)?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(domain);
    hasher.update(&[0]);
    hasher.update(&encoded);
    Ok(Commitment32(*hasher.finalize().as_bytes()))
}

/// Errors from canonical accountability commitments.
#[derive(Debug, Error)]
pub enum AccountabilityCommitmentError {
    #[error("failed to serialize accountability commitment body: {0}")]
    Serialization(#[from] Box<bincode::ErrorKind>),
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
    MissingPolicyVersion,
    ZeroAuthenticatedSourceId,
    ZeroQueryDigest,
    ZeroScopeDigest,
    ZeroDisclosureResultDigest,
    ZeroInferenceExplanationDigest,
    ZeroAttestationDigest,
    PurposeExpiredBeforeLookup,
    InvalidInferenceConfidence,
    DeniedLookupDisclosedData,
    MissingRequiredRight(SubjectRight),
    MissingQueryBudgetCharge,
    InvalidQueryBudgetCharge,
    MissingAttestation,
    DelayedNoticeDisabled,
    DelayReasonNotPermitted,
    MissingDelayAuthorization,
    ZeroDelayJustificationDigest,
    ZeroDelayApprovalDigest,
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
    if receipt.requester.authenticated_source_id.is_zero() {
        return Err(AccountabilityError::ZeroAuthenticatedSourceId);
    }
    if receipt.purpose.purpose_code.trim().is_empty()
        || receipt.purpose.plain_language_purpose.trim().is_empty()
    {
        return Err(AccountabilityError::MissingPurpose);
    }
    if receipt.purpose.scope_digest.is_zero() {
        return Err(AccountabilityError::ZeroScopeDigest);
    }
    if receipt.legal_authority.authority_id.trim().is_empty()
        || receipt.legal_authority.jurisdiction.trim().is_empty()
    {
        return Err(AccountabilityError::MissingLegalAuthority);
    }
    if receipt.query_digest.is_zero() {
        return Err(AccountabilityError::ZeroQueryDigest);
    }
    if receipt.policy_version.trim().is_empty() {
        return Err(AccountabilityError::MissingPolicyVersion);
    }
    if receipt.purpose.expires_at_ms < receipt.occurred_at_ms {
        return Err(AccountabilityError::PurposeExpiredBeforeLookup);
    }

    if let Some(result_digest) = receipt.disclosure.result_digest {
        if result_digest.is_zero() {
            return Err(AccountabilityError::ZeroDisclosureResultDigest);
        }
    }

    if let Some(inference) = &receipt.inference {
        if inference.confidence_ppm > 1_000_000 {
            return Err(AccountabilityError::InvalidInferenceConfidence);
        }
        if inference.explanation_digest.is_zero() {
            return Err(AccountabilityError::ZeroInferenceExplanationDigest);
        }
    }

    if let Some(attestation) = &receipt.attestation {
        if attestation.statement_digest.is_zero() || attestation.proof_digest.is_zero() {
            return Err(AccountabilityError::ZeroAttestationDigest);
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

    let rights: BTreeSet<_> = receipt.rights.iter().copied().collect();
    for required in &policy.required_subject_rights {
        if !rights.contains(required) {
            return Err(AccountabilityError::MissingRequiredRight(*required));
        }
    }

    if policy.require_query_budget_charge && receipt.query_budget_charge.is_none() {
        return Err(AccountabilityError::MissingQueryBudgetCharge);
    }
    if let Some(charge) = &receipt.query_budget_charge {
        if charge.budget_id.trim().is_empty()
            || charge.units == 0
            || charge.window_ends_at_ms <= receipt.occurred_at_ms
        {
            return Err(AccountabilityError::InvalidQueryBudgetCharge);
        }
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
    {
        return Err(AccountabilityError::MissingDelayAuthorization);
    }
    if auth.justification_digest.is_zero() {
        return Err(AccountabilityError::ZeroDelayJustificationDigest);
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
        if approval.approver_id.trim().is_empty() || approval.organization_id.trim().is_empty() {
            return Err(AccountabilityError::MissingDelayAuthorization);
        }
        if approval.approval_digest.is_zero() {
            return Err(AccountabilityError::ZeroDelayApprovalDigest);
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

    fn c(byte: u8) -> Commitment32 {
        Commitment32([byte; 32])
    }

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
                authenticated_source_id: c(9),
            },
            purpose: PurposeBinding {
                purpose_code: "stolen-vehicle".into(),
                plain_language_purpose: "Locate a reported stolen vehicle".into(),
                matter_id: Some("case-123".into()),
                scope_digest: c(10),
                expires_at_ms: occurred_at_ms + DAY_MS,
            },
            legal_authority: LegalAuthority {
                authority_type: AuthorityType::Statute,
                authority_id: "law:example:17".into(),
                jurisdiction: "example-jurisdiction".into(),
            },
            query_digest: c(11),
            policy_version: "ra-policy-1".into(),
            occurred_at_ms,
            outcome: LookupOutcome::Allowed,
            disclosure: DisclosureSummary {
                kind: DisclosureKind::PredicateOnly,
                data_classes: vec!["vehicle-presence".into()],
                item_count: 1,
                result_digest: Some(c(12)),
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
                scheme: "xenia-accountability-execution-attestation-v1".into(),
                statement_digest: c(13),
                proof_digest: c(14),
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
            justification_digest: c(20),
            approved_at_ms: receipt.occurred_at_ms - 1_000,
            release_at_ms: receipt.occurred_at_ms + 7 * DAY_MS,
            required_approvals: 2,
            approvals: vec![
                DelayApproval {
                    organization_id: "agency-a".into(),
                    approver_id: "supervisor-1".into(),
                    approval_digest: c(21),
                },
                DelayApproval {
                    organization_id: "independent-court".into(),
                    approver_id: "judge-9".into(),
                    approval_digest: c(22),
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
            auth.release_at_ms = receipt.occurred_at_ms + DAY_MS;
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
            explanation_digest: c(30),
            provenance_receipt_ids: vec!["receipt-prior".into()],
            significant_effect: true,
        });
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::InvalidInferenceConfidence)
        );
    }

    #[test]
    fn authenticated_source_id_is_required_for_xenia_binding() {
        let mut receipt = immediate_receipt();
        receipt.requester.authenticated_source_id = Commitment32::default();
        assert_eq!(
            validate_receipt(&receipt, &policy()),
            Err(AccountabilityError::ZeroAuthenticatedSourceId)
        );
    }

    #[test]
    fn pre_attestation_commitment_breaks_the_hash_cycle() {
        let mut receipt = immediate_receipt();
        let first = pre_attestation_receipt_commitment(&receipt).expect("commit receipt");
        receipt.attestation = Some(AttestationRef {
            scheme: "a-different-proof-wrapper".into(),
            statement_digest: c(40),
            proof_digest: c(41),
            verifier_profile: "different".into(),
        });
        let second = pre_attestation_receipt_commitment(&receipt).expect("recommit receipt");
        assert_eq!(first, second);

        receipt.purpose.plain_language_purpose.push_str(" (amended)");
        let changed = pre_attestation_receipt_commitment(&receipt).expect("changed receipt");
        assert_ne!(first, changed);
    }

    #[test]
    fn purpose_and_policy_commitments_are_domain_separated() {
        let receipt = immediate_receipt();
        let purpose = purpose_commitment(&receipt.purpose).expect("purpose commitment");
        let policy = policy_commitment(&policy()).expect("policy commitment");
        assert_ne!(purpose, policy);
        assert!(!purpose.is_zero());
        assert!(!policy.is_zero());
    }

    #[test]
    fn default_subject_notice_does_not_expose_actor_or_case_id() {
        let receipt = immediate_receipt();
        let notice = project_subject_notice(&receipt, &SubjectNoticePolicy::default());
        assert_eq!(notice.requester.organization_id, "agency-a");
        assert_eq!(notice.requester.actor_id, None);
        assert_eq!(notice.requester.role.as_deref(), Some("investigator"));
        assert_eq!(notice.authority_id, None);
        assert!(!notice.plain_language_purpose.contains("case-123"));
    }

    #[test]
    fn expired_delayed_notice_projection_discloses_that_notice_was_delayed() {
        let receipt = delayed_receipt();
        let notice = project_subject_notice(&receipt, &SubjectNoticePolicy::default());
        let delayed = notice.delayed_notice.expect("delay summary");
        assert_eq!(delayed.authorization_id, "delay-001");
        assert_eq!(delayed.reason, DelayReason::ActiveInvestigation);
    }
}
