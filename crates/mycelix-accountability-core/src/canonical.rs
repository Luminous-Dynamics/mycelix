// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Language-neutral v1 canonical encoding for accountability commitments.
//!
//! This codec deliberately does not use Serde/bincode. Integers are big-endian,
//! strings are length-prefixed UTF-8, enum tags are fixed by protocol version,
//! options are explicitly tagged, and collections with set semantics are sorted
//! and deduplicated before encoding.

use crate::protocol::*;
use thiserror::Error;

/// Canonical byte codec bound into every v1 accountability commitment.
pub const ACCOUNTABILITY_CANONICAL_CODEC: &str = "mycelix-accountability-canonical-v1";

const RECEIPT_COMMITMENT_DOMAIN_V1: &[u8] = b"mycelix:accountability-receipt:pre-attestation:v1";
const PURPOSE_COMMITMENT_DOMAIN_V1: &[u8] = b"mycelix:accountability-purpose:v1";
const POLICY_COMMITMENT_DOMAIN_V1: &[u8] = b"mycelix:accountability-policy:v1";

/// Errors from canonical accountability commitment encoding.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AccountabilityCommitmentError {
    #[error("accountability collection or string length exceeds canonical v1 limits")]
    LengthOverflow,
}

type CanonicalResult<T> = Result<T, AccountabilityCommitmentError>;

/// Compute the stable v1 commitment to the semantic receipt before proofs are
/// attached. `attestations` are intentionally excluded from the encoded body.
pub fn pre_attestation_receipt_commitment(
    receipt: &AccessReceipt,
) -> CanonicalResult<Commitment32> {
    let encoded = canonical_pre_attestation_receipt_bytes(receipt)?;
    Ok(hash_canonical(RECEIPT_COMMITMENT_DOMAIN_V1, &encoded))
}

/// Compute the purpose/scope commitment carried into execution/proof bindings.
pub fn purpose_commitment(purpose: &PurposeBinding) -> CanonicalResult<Commitment32> {
    let mut encoded = Vec::new();
    encode_purpose(&mut encoded, purpose)?;
    Ok(hash_canonical(PURPOSE_COMMITMENT_DOMAIN_V1, &encoded))
}

/// Compute the policy commitment carried into execution/proof bindings.
pub fn policy_commitment(policy: &AccountabilityPolicy) -> CanonicalResult<Commitment32> {
    let encoded = canonical_policy_bytes(policy)?;
    Ok(hash_canonical(POLICY_COMMITMENT_DOMAIN_V1, &encoded))
}

/// Export exact canonical pre-attestation receipt bytes for independent
/// implementations and golden-vector testing.
pub fn canonical_pre_attestation_receipt_bytes(
    receipt: &AccessReceipt,
) -> CanonicalResult<Vec<u8>> {
    let mut out = Vec::new();
    push_u16(&mut out, receipt.protocol_version);
    push_str(&mut out, &receipt.receipt_id)?;
    push_str(&mut out, &receipt.subject.0)?;
    encode_requester(&mut out, &receipt.requester)?;
    encode_purpose(&mut out, &receipt.purpose)?;
    encode_legal_authority(&mut out, &receipt.legal_authority)?;
    push_commitment(&mut out, receipt.query_digest);
    push_str(&mut out, &receipt.policy_version)?;
    push_u64(&mut out, receipt.occurred_at_ms);
    push_u8(&mut out, lookup_outcome_tag(receipt.outcome));
    encode_disclosure(&mut out, &receipt.disclosure)?;
    encode_notification(&mut out, &receipt.notification)?;

    let mut rights = receipt.rights.clone();
    rights.sort_unstable();
    rights.dedup();
    push_len(&mut out, rights.len())?;
    for right in rights {
        push_u8(&mut out, subject_right_tag(right));
    }

    encode_budget(&mut out, receipt.query_budget_charge.as_ref())?;
    encode_inference(&mut out, receipt.inference.as_ref())?;
    Ok(out)
}

/// Export exact canonical policy bytes for independent implementations and
/// golden-vector testing.
pub fn canonical_policy_bytes(policy: &AccountabilityPolicy) -> CanonicalResult<Vec<u8>> {
    let mut out = Vec::new();
    push_u64(&mut out, policy.max_delay_ms);
    push_u8(&mut out, policy.min_delay_approvals);
    push_bool(&mut out, policy.require_approval_outside_requester_org);

    let mut reasons = policy.permitted_delay_reasons.clone();
    reasons.sort_unstable();
    reasons.dedup();
    push_len(&mut out, reasons.len())?;
    for reason in reasons {
        push_u8(&mut out, delay_reason_tag(reason));
    }

    let mut rights = policy.required_subject_rights.clone();
    rights.sort_unstable();
    rights.dedup();
    push_len(&mut out, rights.len())?;
    for right in rights {
        push_u8(&mut out, subject_right_tag(right));
    }

    push_bool(&mut out, policy.require_query_budget_charge);

    let mut roles = policy.required_attestation_roles.clone();
    roles.sort_unstable();
    roles.dedup();
    push_len(&mut out, roles.len())?;
    for role in roles {
        push_u8(&mut out, attestation_role_tag(role));
    }
    Ok(out)
}

fn hash_canonical(domain: &[u8], encoded: &[u8]) -> Commitment32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(domain);
    hasher.update(&[0]);
    hasher.update(ACCOUNTABILITY_CANONICAL_CODEC.as_bytes());
    hasher.update(&[0]);
    hasher.update(encoded);
    Commitment32(*hasher.finalize().as_bytes())
}

fn encode_requester(out: &mut Vec<u8>, value: &RequestingPrincipal) -> CanonicalResult<()> {
    push_str(out, &value.organization_id)?;
    push_str(out, &value.actor_id)?;
    push_str(out, &value.role)?;
    push_commitment(out, value.authenticated_source_id);
    Ok(())
}

fn encode_purpose(out: &mut Vec<u8>, value: &PurposeBinding) -> CanonicalResult<()> {
    push_str(out, &value.purpose_code)?;
    push_str(out, &value.plain_language_purpose)?;
    push_optional_str(out, value.matter_id.as_deref())?;
    push_commitment(out, value.scope_digest);
    push_u64(out, value.expires_at_ms);
    Ok(())
}

fn encode_legal_authority(out: &mut Vec<u8>, value: &LegalAuthority) -> CanonicalResult<()> {
    push_u8(out, authority_type_tag(value.authority_type));
    push_str(out, &value.authority_id)?;
    push_str(out, &value.jurisdiction)?;
    Ok(())
}

fn encode_disclosure(out: &mut Vec<u8>, value: &DisclosureSummary) -> CanonicalResult<()> {
    push_u8(out, disclosure_kind_tag(value.kind));
    let mut classes = value.data_classes.clone();
    classes.sort();
    classes.dedup();
    push_len(out, classes.len())?;
    for class in classes {
        push_str(out, &class)?;
    }
    push_u32(out, value.item_count);
    push_optional_commitment(out, value.result_digest);
    Ok(())
}

fn encode_notification(out: &mut Vec<u8>, value: &NotificationDirective) -> CanonicalResult<()> {
    match value {
        NotificationDirective::Immediate => push_u8(out, 0),
        NotificationDirective::Delayed(auth) => {
            push_u8(out, 1);
            push_str(out, &auth.authorization_id)?;
            push_u8(out, delay_reason_tag(auth.reason));
            push_str(out, &auth.legal_basis)?;
            push_commitment(out, auth.justification_digest);
            push_u64(out, auth.approved_at_ms);
            push_u64(out, auth.release_at_ms);
            push_u8(out, auth.required_approvals);

            let mut approvals: Vec<_> = auth.approvals.iter().collect();
            approvals.sort_by(|left, right| {
                (
                    &left.organization_id,
                    &left.approver_id,
                    left.approval_digest.0,
                )
                    .cmp(&(
                        &right.organization_id,
                        &right.approver_id,
                        right.approval_digest.0,
                    ))
            });
            push_len(out, approvals.len())?;
            for approval in approvals {
                push_str(out, &approval.organization_id)?;
                push_str(out, &approval.approver_id)?;
                push_commitment(out, approval.approval_digest);
            }
        }
    }
    Ok(())
}

fn encode_budget(out: &mut Vec<u8>, value: Option<&QueryBudgetCharge>) -> CanonicalResult<()> {
    match value {
        None => push_u8(out, 0),
        Some(charge) => {
            push_u8(out, 1);
            push_str(out, &charge.budget_id)?;
            push_u32(out, charge.units);
            push_u32(out, charge.remaining_units);
            push_u64(out, charge.window_ends_at_ms);
        }
    }
    Ok(())
}

fn encode_inference(out: &mut Vec<u8>, value: Option<&InferenceDisclosure>) -> CanonicalResult<()> {
    match value {
        None => push_u8(out, 0),
        Some(inference) => {
            push_u8(out, 1);
            push_str(out, &inference.inference_id)?;
            push_str(out, &inference.model_id)?;
            push_str(out, &inference.model_version)?;
            push_str(out, &inference.plain_language_summary)?;
            push_u32(out, inference.confidence_ppm);
            push_commitment(out, inference.explanation_digest);

            let mut provenance = inference.provenance_receipt_ids.clone();
            provenance.sort();
            provenance.dedup();
            push_len(out, provenance.len())?;
            for receipt_id in provenance {
                push_str(out, &receipt_id)?;
            }
            push_bool(out, inference.significant_effect);
        }
    }
    Ok(())
}

fn push_optional_str(out: &mut Vec<u8>, value: Option<&str>) -> CanonicalResult<()> {
    match value {
        None => push_u8(out, 0),
        Some(value) => {
            push_u8(out, 1);
            push_str(out, value)?;
        }
    }
    Ok(())
}

fn push_optional_commitment(out: &mut Vec<u8>, value: Option<Commitment32>) {
    match value {
        None => push_u8(out, 0),
        Some(value) => {
            push_u8(out, 1);
            push_commitment(out, value);
        }
    }
}

fn push_str(out: &mut Vec<u8>, value: &str) -> CanonicalResult<()> {
    push_len(out, value.len())?;
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_len(out: &mut Vec<u8>, value: usize) -> CanonicalResult<()> {
    let value = u64::try_from(value).map_err(|_| AccountabilityCommitmentError::LengthOverflow)?;
    push_u64(out, value);
    Ok(())
}

fn push_commitment(out: &mut Vec<u8>, value: Commitment32) {
    out.extend_from_slice(&value.0);
}

fn push_bool(out: &mut Vec<u8>, value: bool) {
    push_u8(out, u8::from(value));
}

fn push_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

const fn authority_type_tag(value: AuthorityType) -> u8 {
    match value {
        AuthorityType::Consent => 0,
        AuthorityType::Statute => 1,
        AuthorityType::CourtOrder => 2,
        AuthorityType::Warrant => 3,
        AuthorityType::Emergency => 4,
        AuthorityType::Contractual => 5,
        AuthorityType::Other => 6,
    }
}

const fn lookup_outcome_tag(value: LookupOutcome) -> u8 {
    match value {
        LookupOutcome::Allowed => 0,
        LookupOutcome::PartiallyAllowed => 1,
        LookupOutcome::Denied => 2,
    }
}

const fn disclosure_kind_tag(value: DisclosureKind) -> u8 {
    match value {
        DisclosureKind::None => 0,
        DisclosureKind::PredicateOnly => 1,
        DisclosureKind::RedactedRecord => 2,
        DisclosureKind::FullRecord => 3,
    }
}

const fn subject_right_tag(value: SubjectRight) -> u8 {
    match value {
        SubjectRight::Know => 0,
        SubjectRight::Inspect => 1,
        SubjectRight::Contest => 2,
        SubjectRight::HumanReview => 3,
        SubjectRight::Appeal => 4,
        SubjectRight::ProofOfPolicy => 5,
    }
}

const fn delay_reason_tag(value: DelayReason) -> u8 {
    match value {
        DelayReason::ActiveInvestigation => 0,
        DelayReason::PreventCrime => 1,
        DelayReason::ProtectVictimOrWitness => 2,
        DelayReason::PublicSafety => 3,
        DelayReason::NationalSecurity => 4,
    }
}

const fn attestation_role_tag(value: AttestationRole) -> u8 {
    match value {
        AttestationRole::ExecutionBinding => 0,
        AttestationRole::ComputationProof => 1,
        AttestationRole::PolicyProof => 2,
        AttestationRole::ExternalWitness => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DAY_MS: u64 = 24 * 60 * 60 * 1000;

    fn c(byte: u8) -> Commitment32 {
        Commitment32([byte; 32])
    }

    fn receipt() -> AccessReceipt {
        let occurred_at_ms = 1_800_000_000_000;
        AccessReceipt {
            protocol_version: RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
            receipt_id: "receipt-interop-001".into(),
            subject: PairwiseSubjectId("pairwise:subject:42".into()),
            requester: RequestingPrincipal {
                organization_id: "agency-a".into(),
                actor_id: "operator-key-7".into(),
                role: "investigator".into(),
                authenticated_source_id: c(9),
            },
            purpose: PurposeBinding {
                purpose_code: "minimum-disclosure".into(),
                plain_language_purpose: "Answer a scoped predicate".into(),
                matter_id: Some("matter-123".into()),
                scope_digest: c(10),
                expires_at_ms: occurred_at_ms + DAY_MS,
            },
            legal_authority: LegalAuthority {
                authority_type: AuthorityType::Consent,
                authority_id: "consent-7".into(),
                jurisdiction: "test-jurisdiction".into(),
            },
            query_digest: c(11),
            policy_version: "ra-policy-1".into(),
            occurred_at_ms,
            outcome: LookupOutcome::Allowed,
            disclosure: DisclosureSummary {
                kind: DisclosureKind::PredicateOnly,
                data_classes: vec!["z-class".into(), "a-class".into()],
                item_count: 1,
                result_digest: Some(c(12)),
            },
            notification: NotificationDirective::Immediate,
            rights: vec![
                SubjectRight::Contest,
                SubjectRight::Know,
                SubjectRight::Inspect,
            ],
            query_budget_charge: Some(QueryBudgetCharge {
                budget_id: "budget-1".into(),
                units: 1,
                remaining_units: 99,
                window_ends_at_ms: occurred_at_ms + DAY_MS,
            }),
            inference: Some(InferenceDisclosure {
                inference_id: "inference-1".into(),
                model_id: "model-a".into(),
                model_version: "1".into(),
                plain_language_summary: "Scoped predicate result".into(),
                confidence_ppm: 900_000,
                explanation_digest: c(13),
                provenance_receipt_ids: vec!["receipt-z".into(), "receipt-a".into()],
                significant_effect: false,
            }),
            attestations: Vec::new(),
        }
    }

    fn policy() -> AccountabilityPolicy {
        AccountabilityPolicy {
            max_delay_ms: 30 * DAY_MS,
            min_delay_approvals: 2,
            require_approval_outside_requester_org: true,
            permitted_delay_reasons: vec![
                DelayReason::PublicSafety,
                DelayReason::ActiveInvestigation,
                DelayReason::ProtectVictimOrWitness,
            ],
            required_subject_rights: vec![
                SubjectRight::Contest,
                SubjectRight::Know,
                SubjectRight::Inspect,
            ],
            require_query_budget_charge: true,
            required_attestation_roles: vec![
                AttestationRole::ComputationProof,
                AttestationRole::ExecutionBinding,
            ],
        }
    }

    #[test]
    fn receipt_commitment_is_invariant_to_set_order_and_attestations() {
        let base = receipt();
        let expected = pre_attestation_receipt_commitment(&base).unwrap();

        let mut reordered = base.clone();
        reordered.rights.reverse();
        reordered.disclosure.data_classes.reverse();
        reordered
            .inference
            .as_mut()
            .unwrap()
            .provenance_receipt_ids
            .reverse();
        reordered.attestations.push(AttestationRef {
            role: AttestationRole::ExecutionBinding,
            scheme: "example".into(),
            statement_digest: c(20),
            proof_digest: c(21),
            verifier_profile: "example-v1".into(),
        });

        assert_eq!(
            expected,
            pre_attestation_receipt_commitment(&reordered).unwrap()
        );
    }

    #[test]
    fn policy_commitment_is_invariant_to_set_order() {
        let base = policy();
        let expected = policy_commitment(&base).unwrap();
        let mut reordered = base;
        reordered.permitted_delay_reasons.reverse();
        reordered.required_subject_rights.reverse();
        reordered.required_attestation_roles.reverse();
        assert_eq!(expected, policy_commitment(&reordered).unwrap());
    }

    #[test]
    fn semantic_change_changes_commitment() {
        let base = receipt();
        let expected = pre_attestation_receipt_commitment(&base).unwrap();
        let mut changed = base;
        changed
            .purpose
            .plain_language_purpose
            .push_str(" materially changed");
        assert_ne!(
            expected,
            pre_attestation_receipt_commitment(&changed).unwrap()
        );
    }

    #[test]
    fn canonical_bytes_are_stable_nonempty_domains() {
        let receipt_bytes = canonical_pre_attestation_receipt_bytes(&receipt()).unwrap();
        let policy_bytes = canonical_policy_bytes(&policy()).unwrap();
        assert!(!receipt_bytes.is_empty());
        assert!(!policy_bytes.is_empty());
        assert_ne!(receipt_bytes, policy_bytes);
    }
}
