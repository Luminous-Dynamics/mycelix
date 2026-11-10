#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Effects-disabled reference model for competing payment-channel transitions.
//!
//! This crate deliberately does not talk to Holochain, verify signatures, write
//! entries, contact a payment rail, or report settlement finality. It models
//! deterministic transition shape, policy-claim structure, duplicate delivery,
//! and fail-closed reconciliation of competing branches.

use std::collections::BTreeMap;

pub const CONTRACT_VERSION: &str = "mycelix.payment-channel-fork-reference/0.1";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelTerms {
    pub channel_id: String,
    pub party_a: String,
    pub party_b: String,
    pub currency: String,
    pub initial_balance_a: u64,
    pub initial_balance_b: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelState {
    pub channel_id: String,
    /// Opaque reference to the exact predecessor state/action.
    pub state_ref: String,
    pub sequence: u64,
    pub balance_a: u64,
    pub balance_b: u64,
    pub closed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Party {
    A,
    B,
}

impl Party {
    fn did<'a>(self, terms: &'a ChannelTerms) -> &'a str {
        match self {
            Self::A => &terms.party_a,
            Self::B => &terms.party_b,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChannelOperation {
    Transfer { debit_party: Party, amount: u64 },
    Close,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SequencingClaim {
    None,
    SingleWriter { writer_did: String, epoch: u64 },
    WitnessQuorum { witness_signers: Vec<String> },
}

/// These are claims supplied to the reference model, not cryptographic
/// proofs. The model checks their structure and policy fit, not signatures.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthorizationClaim {
    /// Participant consent claims. Canonical order is lexicographic.
    pub participant_signers: Vec<String>,
    pub sequencing: SequencingClaim,
    /// Opaque reference to the evidence an external verifier would inspect.
    pub evidence_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthorizationPolicy {
    /// A deterministic projection may be useful for display, but has no spend
    /// authority and can never produce an authorized transition disposition.
    ProjectionOnly,
    /// One designated channel participant sequences updates for an epoch.
    /// The debit owner must still be present in participant_signers; a
    /// sequencer claim is not a substitute for the debit owner's consent.
    SingleWriter { writer_did: String, epoch: u64 },
    /// Both channel participants must consent to the exact transition.
    MutualConsent,
    /// A witness set with a quorum whose pairwise intersection is guaranteed
    /// to contain at least one honest witness under the declared fault bound.
    WitnessQuorum {
        witness_roster: Vec<String>,
        threshold: usize,
        max_byzantine_faults: usize,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidateTransition {
    pub channel_id: String,
    /// Exact DHT action reference claimed for this branch.
    pub candidate_ref: String,
    /// Sender-scoped durable operation identity.
    pub operation_id: String,
    /// Must equal the exact predecessor state/action reference.
    pub predecessor_ref: String,
    pub sequence: u64,
    /// The author of the candidate update, not necessarily the debit owner.
    pub author: String,
    pub operation: ChannelOperation,
    pub authorization: AuthorizationClaim,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    NoCandidate,
    Rejected,
    /// The operation and authorization claims are internally consistent.
    /// This is not a committed ledger effect or settlement finality.
    PolicyClaimConsistentPendingFinality,
    Conflicted,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RejectReason {
    EmptyCandidateReference,
    EmptyOperationId,
    EmptyAuthor,
    WrongChannel,
    StalePredecessor,
    SequenceMismatch,
    NonParticipantAuthor,
    ChannelAlreadyClosed,
    ZeroAmount,
    InsufficientBalance,
    BalanceOverflow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReasonCode {
    InvalidBaseState,
    CandidateReferenceCollision,
    CompetingBranches,
    OperationIdentityPayloadConflict,
    ProjectionOnlyHasNoAuthority,
    MissingAuthorizationEvidenceReference,
    NonCanonicalParticipantSigners,
    MissingRequiredParticipantConsent,
    InvalidSequencerClaim,
    InvalidQuorumPolicy,
    InvalidQuorumSigners,
    CandidateRejected,
    AuthorizationClaimNotQualified,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RejectedCandidate {
    pub candidate_ref: String,
    pub reason: RejectReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Resolution {
    pub disposition: Disposition,
    /// Present only for a single, structurally valid, policy-claim-consistent
    /// candidate. It is not a canonical state selection or settlement receipt.
    pub policy_consistent_candidate_ref: Option<String>,
    /// All distinct candidate action references observed in this evaluation.
    pub observed_candidate_refs: Vec<String>,
    /// Exact duplicate deliveries of the same candidate reference and bytes.
    pub duplicate_delivery_count: usize,
    pub rejected_candidates: Vec<RejectedCandidate>,
    pub reason_codes: Vec<ReasonCode>,
}

fn is_sorted_unique(values: &[String]) -> bool {
    !values.iter().any(String::is_empty)
        && values.windows(2).all(|pair| pair[0] < pair[1])
}

fn contains(values: &[String], needle: &str) -> bool {
    values.iter().any(|value| value == needle)
}

fn valid_terms_and_state(terms: &ChannelTerms, state: &ChannelState) -> bool {
    if terms.channel_id.is_empty()
        || terms.party_a.is_empty()
        || terms.party_b.is_empty()
        || terms.party_a == terms.party_b
        || terms.currency.is_empty()
        || state.channel_id != terms.channel_id
        || state.state_ref.is_empty()
    {
        return false;
    }

    let initial_total = terms
        .initial_balance_a
        .checked_add(terms.initial_balance_b);
    let current_total = state.balance_a.checked_add(state.balance_b);
    initial_total.is_some() && initial_total == current_total
}

fn validate_transition(
    terms: &ChannelTerms,
    state: &ChannelState,
    candidate: &CandidateTransition,
) -> Result<(), RejectReason> {
    if candidate.candidate_ref.is_empty() {
        return Err(RejectReason::EmptyCandidateReference);
    }
    if candidate.operation_id.is_empty() {
        return Err(RejectReason::EmptyOperationId);
    }
    if candidate.author.is_empty() {
        return Err(RejectReason::EmptyAuthor);
    }
    if candidate.channel_id != terms.channel_id {
        return Err(RejectReason::WrongChannel);
    }
    if candidate.predecessor_ref != state.state_ref {
        return Err(RejectReason::StalePredecessor);
    }
    let expected_sequence = state
        .sequence
        .checked_add(1)
        .ok_or(RejectReason::SequenceMismatch)?;
    if candidate.sequence != expected_sequence {
        return Err(RejectReason::SequenceMismatch);
    }
    if candidate.author != terms.party_a && candidate.author != terms.party_b {
        return Err(RejectReason::NonParticipantAuthor);
    }
    if state.closed {
        return Err(RejectReason::ChannelAlreadyClosed);
    }

    match &candidate.operation {
        ChannelOperation::Transfer { debit_party, amount } => {
            if *amount == 0 {
                return Err(RejectReason::ZeroAmount);
            }
            match debit_party {
                Party::A => {
                    state
                        .balance_a
                        .checked_sub(*amount)
                        .ok_or(RejectReason::InsufficientBalance)?;
                    state
                        .balance_b
                        .checked_add(*amount)
                        .ok_or(RejectReason::BalanceOverflow)?;
                }
                Party::B => {
                    state
                        .balance_b
                        .checked_sub(*amount)
                        .ok_or(RejectReason::InsufficientBalance)?;
                    state
                        .balance_a
                        .checked_add(*amount)
                        .ok_or(RejectReason::BalanceOverflow)?;
                }
            }
        }
        ChannelOperation::Close => {}
    }

    Ok(())
}

fn required_participant_consents(
    terms: &ChannelTerms,
    operation: &ChannelOperation,
) -> Vec<String> {
    match operation {
        ChannelOperation::Transfer { debit_party, .. } => {
            vec![debit_party.did(terms).to_owned()]
        }
        ChannelOperation::Close => vec![terms.party_a.clone(), terms.party_b.clone()],
    }
}

fn validate_authorization_claim(
    terms: &ChannelTerms,
    candidate: &CandidateTransition,
    policy: &AuthorizationPolicy,
) -> Result<(), ReasonCode> {
    let claim = &candidate.authorization;
    if claim.evidence_ref.is_empty() {
        return Err(ReasonCode::MissingAuthorizationEvidenceReference);
    }
    if !is_sorted_unique(&claim.participant_signers)
        || claim
            .participant_signers
            .iter()
            .any(|signer| signer != &terms.party_a && signer != &terms.party_b)
    {
        return Err(ReasonCode::NonCanonicalParticipantSigners);
    }

    let required = required_participant_consents(terms, &candidate.operation);
    if required
        .iter()
        .any(|required_signer| !contains(&claim.participant_signers, required_signer))
    {
        return Err(ReasonCode::MissingRequiredParticipantConsent);
    }

    match (policy, &claim.sequencing) {
        (AuthorizationPolicy::ProjectionOnly, _) => {
            Err(ReasonCode::ProjectionOnlyHasNoAuthority)
        }
        (
            AuthorizationPolicy::SingleWriter { writer_did, epoch },
            SequencingClaim::SingleWriter {
                writer_did: claimed_writer,
                epoch: claimed_epoch,
            },
        ) => {
            if (writer_did != &terms.party_a && writer_did != &terms.party_b)
                || writer_did != claimed_writer
                || epoch != claimed_epoch
                || candidate.author.as_str() != writer_did.as_str()
            {
                return Err(ReasonCode::InvalidSequencerClaim);
            }
            Ok(())
        }
        (AuthorizationPolicy::MutualConsent, SequencingClaim::None) => {
            if claim.participant_signers.len() == 2
                && contains(&claim.participant_signers, &terms.party_a)
                && contains(&claim.participant_signers, &terms.party_b)
            {
                Ok(())
            } else {
                Err(ReasonCode::MissingRequiredParticipantConsent)
            }
        }
        (
            AuthorizationPolicy::WitnessQuorum {
                witness_roster,
                threshold,
                max_byzantine_faults,
            },
            SequencingClaim::WitnessQuorum { witness_signers },
        ) => {
            if !is_sorted_unique(witness_roster)
                || witness_roster
                    .iter()
                    .any(|witness| witness == &terms.party_a || witness == &terms.party_b)
                || *threshold == 0
                || *threshold > witness_roster.len()
                || *max_byzantine_faults >= witness_roster.len()
                || threshold.saturating_mul(2)
                    <= witness_roster.len().saturating_add(*max_byzantine_faults)
            {
                return Err(ReasonCode::InvalidQuorumPolicy);
            }
            if !is_sorted_unique(witness_signers)
                || witness_signers
                    .iter()
                    .any(|signer| !contains(witness_roster, signer))
                || witness_signers.len() < *threshold
                || (candidate.author != terms.party_a && candidate.author != terms.party_b)
            {
                return Err(ReasonCode::InvalidQuorumSigners);
            }
            Ok(())
        }
        _ => Err(ReasonCode::AuthorizationClaimNotQualified),
    }
}

fn same_operation_intent(
    left: &CandidateTransition,
    right: &CandidateTransition,
) -> bool {
    left.channel_id == right.channel_id
        && left.operation_id == right.operation_id
        && left.predecessor_ref == right.predecessor_ref
        && left.sequence == right.sequence
        && left.author == right.author
        && left.operation == right.operation
}

/// Reconcile all candidate claims observed for one exact channel predecessor.
///
/// The function never picks a branch by timestamp, ActionHash order, arrival
/// order, or candidate-list order. Multiple structurally valid next-state
/// claims produce Conflicted; one candidate can at most become
/// PolicyClaimConsistentPendingFinality.
pub fn reconcile(
    terms: &ChannelTerms,
    state: &ChannelState,
    policy: &AuthorizationPolicy,
    candidates: &[CandidateTransition],
) -> Resolution {
    if !valid_terms_and_state(terms, state) {
        return Resolution {
            disposition: Disposition::Indeterminate,
            policy_consistent_candidate_ref: None,
            observed_candidate_refs: Vec::new(),
            duplicate_delivery_count: 0,
            rejected_candidates: Vec::new(),
            reason_codes: vec![ReasonCode::InvalidBaseState],
        };
    }

    let mut distinct = BTreeMap::<String, CandidateTransition>::new();
    let mut duplicate_delivery_count = 0usize;
    let mut reference_collision = false;

    for candidate in candidates {
        match distinct.get(&candidate.candidate_ref) {
            Some(existing) if existing == candidate => {
                duplicate_delivery_count = duplicate_delivery_count.saturating_add(1);
            }
            Some(_) => reference_collision = true,
            None => {
                distinct.insert(candidate.candidate_ref.clone(), candidate.clone());
            }
        }
    }

    let observed_candidate_refs = distinct.keys().cloned().collect::<Vec<_>>();
    if reference_collision {
        return Resolution {
            disposition: Disposition::Conflicted,
            policy_consistent_candidate_ref: None,
            observed_candidate_refs,
            duplicate_delivery_count,
            rejected_candidates: Vec::new(),
            reason_codes: vec![ReasonCode::CandidateReferenceCollision],
        };
    }

    let unique_candidates = distinct.values().cloned().collect::<Vec<_>>();
    let mut rejected_candidates = Vec::new();
    let mut structurally_valid = Vec::new();

    for candidate in &unique_candidates {
        match validate_transition(terms, state, candidate) {
            Ok(()) => structurally_valid.push(candidate.clone()),
            Err(reason) => rejected_candidates.push(RejectedCandidate {
                candidate_ref: candidate.candidate_ref.clone(),
                reason,
            }),
        }
    }

    if structurally_valid.len() > 1 {
        let mut reason_codes = vec![ReasonCode::CompetingBranches];
        for left in 0..structurally_valid.len() {
            for right in (left + 1)..structurally_valid.len() {
                if structurally_valid[left].operation_id
                    == structurally_valid[right].operation_id
                    && !same_operation_intent(
                        &structurally_valid[left],
                        &structurally_valid[right],
                    )
                {
                    reason_codes.push(ReasonCode::OperationIdentityPayloadConflict);
                }
            }
        }
        reason_codes.sort_by_key(|reason| *reason as u8);
        reason_codes.dedup();
        return Resolution {
            disposition: Disposition::Conflicted,
            policy_consistent_candidate_ref: None,
            observed_candidate_refs,
            duplicate_delivery_count,
            rejected_candidates,
            reason_codes,
        };
    }

    let Some(candidate) = structurally_valid.first() else {
        return Resolution {
            disposition: if rejected_candidates.is_empty() {
                Disposition::NoCandidate
            } else {
                Disposition::Rejected
            },
            policy_consistent_candidate_ref: None,
            observed_candidate_refs,
            duplicate_delivery_count,
            rejected_candidates,
            reason_codes: if candidates.is_empty() {
                Vec::new()
            } else {
                vec![ReasonCode::CandidateRejected]
            },
        };
    };

    if let Err(reason) = validate_authorization_claim(terms, candidate, policy) {
        return Resolution {
            disposition: Disposition::Indeterminate,
            policy_consistent_candidate_ref: None,
            observed_candidate_refs,
            duplicate_delivery_count,
            rejected_candidates,
            reason_codes: vec![reason],
        };
    }

    Resolution {
        disposition: Disposition::PolicyClaimConsistentPendingFinality,
        policy_consistent_candidate_ref: Some(candidate.candidate_ref.clone()),
        observed_candidate_refs,
        duplicate_delivery_count,
        rejected_candidates,
        reason_codes: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terms() -> ChannelTerms {
        ChannelTerms {
            channel_id: "channel-001".into(),
            party_a: "did:mycelix:alice".into(),
            party_b: "did:mycelix:bob".into(),
            currency: "SAP".into(),
            initial_balance_a: 100,
            initial_balance_b: 100,
        }
    }

    fn state() -> ChannelState {
        ChannelState {
            channel_id: "channel-001".into(),
            state_ref: "action:genesis".into(),
            sequence: 0,
            balance_a: 100,
            balance_b: 100,
            closed: false,
        }
    }

    fn single_writer_policy() -> AuthorizationPolicy {
        AuthorizationPolicy::SingleWriter {
            writer_did: "did:mycelix:alice".into(),
            epoch: 7,
        }
    }

    fn authorized_transfer(
        candidate_ref: &str,
        operation_id: &str,
        debit_party: Party,
        amount: u64,
    ) -> CandidateTransition {
        let t = terms();
        let debtor = debit_party.did(&t).to_owned();
        CandidateTransition {
            channel_id: t.channel_id.clone(),
            candidate_ref: candidate_ref.into(),
            operation_id: operation_id.into(),
            predecessor_ref: "action:genesis".into(),
            sequence: 1,
            author: "did:mycelix:alice".into(),
            operation: ChannelOperation::Transfer { debit_party, amount },
            authorization: AuthorizationClaim {
                participant_signers: vec![debtor],
                sequencing: SequencingClaim::SingleWriter {
                    writer_did: "did:mycelix:alice".into(),
                    epoch: 7,
                },
                evidence_ref: format!("synthetic:auth:{candidate_ref}"),
            },
        }
    }

    fn authorized_close(candidate_ref: &str) -> CandidateTransition {
        CandidateTransition {
            channel_id: "channel-001".into(),
            candidate_ref: candidate_ref.into(),
            operation_id: format!("close-{candidate_ref}"),
            predecessor_ref: "action:genesis".into(),
            sequence: 1,
            author: "did:mycelix:alice".into(),
            operation: ChannelOperation::Close,
            authorization: AuthorizationClaim {
                participant_signers: vec![
                    "did:mycelix:alice".into(),
                    "did:mycelix:bob".into(),
                ],
                sequencing: SequencingClaim::SingleWriter {
                    writer_did: "did:mycelix:alice".into(),
                    epoch: 7,
                },
                evidence_ref: format!("synthetic:close:{candidate_ref}"),
            },
        }
    }

    #[test]
    fn empty_candidate_set_has_no_effect() {
        let result = reconcile(&terms(), &state(), &single_writer_policy(), &[]);
        assert_eq!(result.disposition, Disposition::NoCandidate);
        assert!(result.policy_consistent_candidate_ref.is_none());
    }

    #[test]
    fn one_policy_consistent_candidate_is_not_settlement_finality() {
        let candidate = authorized_transfer("action:1", "op-1", Party::A, 10);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(
            result.disposition,
            Disposition::PolicyClaimConsistentPendingFinality
        );
        assert_eq!(
            result.policy_consistent_candidate_ref.as_deref(),
            Some("action:1")
        );
    }

    #[test]
    fn exact_duplicate_delivery_is_idempotent_in_the_reference_projection() {
        let candidate = authorized_transfer("action:1", "op-1", Party::A, 10);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate.clone(), candidate],
        );
        assert_eq!(
            result.disposition,
            Disposition::PolicyClaimConsistentPendingFinality
        );
        assert_eq!(result.duplicate_delivery_count, 1);
        assert_eq!(result.observed_candidate_refs, vec!["action:1"]);
    }

    #[test]
    fn two_distinct_transfers_from_one_predecessor_are_conflicted() {
        let first = authorized_transfer("action:1", "op-1", Party::A, 10);
        let second = authorized_transfer("action:2", "op-2", Party::A, 20);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[second, first],
        );
        assert_eq!(result.disposition, Disposition::Conflicted);
        assert!(result.policy_consistent_candidate_ref.is_none());
        assert!(result.reason_codes.contains(&ReasonCode::CompetingBranches));
    }

    #[test]
    fn same_operation_id_with_changed_amount_is_a_conflict() {
        let first = authorized_transfer("action:1", "op-reused", Party::A, 10);
        let second = authorized_transfer("action:2", "op-reused", Party::A, 11);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[first, second],
        );
        assert_eq!(result.disposition, Disposition::Conflicted);
        assert!(result
            .reason_codes
            .contains(&ReasonCode::OperationIdentityPayloadConflict));
    }

    #[test]
    fn same_payload_under_two_action_references_is_not_silently_deduplicated() {
        let first = authorized_transfer("action:1", "op-1", Party::A, 10);
        let mut second = first.clone();
        second.candidate_ref = "action:2".into();
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[first, second],
        );
        assert_eq!(result.disposition, Disposition::Conflicted);
    }

    #[test]
    fn one_candidate_reference_cannot_name_two_different_payloads() {
        let first = authorized_transfer("action:same", "op-1", Party::A, 10);
        let second = authorized_transfer("action:same", "op-1", Party::A, 11);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[first, second],
        );
        assert_eq!(result.disposition, Disposition::Conflicted);
        assert!(result
            .reason_codes
            .contains(&ReasonCode::CandidateReferenceCollision));
    }

    #[test]
    fn close_racing_a_transfer_is_conflicted_not_hash_ordered() {
        let transfer = authorized_transfer("action:transfer", "op-transfer", Party::A, 10);
        let close = authorized_close("action:close");
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[close, transfer],
        );
        assert_eq!(result.disposition, Disposition::Conflicted);
    }

    #[test]
    fn stale_predecessor_is_rejected_and_never_upgraded() {
        let mut candidate = authorized_transfer("action:stale", "op-stale", Party::A, 10);
        candidate.predecessor_ref = "action:other".into();
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Rejected);
        assert_eq!(
            result.rejected_candidates[0].reason,
            RejectReason::StalePredecessor
        );
    }

    #[test]
    fn wrong_sequence_is_rejected() {
        let mut candidate = authorized_transfer("action:gap", "op-gap", Party::A, 10);
        candidate.sequence = 3;
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Rejected);
        assert_eq!(
            result.rejected_candidates[0].reason,
            RejectReason::SequenceMismatch
        );
    }

    #[test]
    fn zero_transfer_is_rejected() {
        let candidate = authorized_transfer("action:zero", "op-zero", Party::A, 0);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Rejected);
        assert_eq!(
            result.rejected_candidates[0].reason,
            RejectReason::ZeroAmount
        );
    }

    #[test]
    fn insufficient_balance_is_rejected() {
        let candidate = authorized_transfer("action:overdraw", "op-overdraw", Party::A, 101);
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Rejected);
        assert_eq!(
            result.rejected_candidates[0].reason,
            RejectReason::InsufficientBalance
        );
    }

    #[test]
    fn projection_only_policy_cannot_claim_spend_authority() {
        let candidate = authorized_transfer("action:projection", "op-projection", Party::A, 10);
        let result = reconcile(
            &terms(),
            &state(),
            &AuthorizationPolicy::ProjectionOnly,
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
        assert!(result
            .reason_codes
            .contains(&ReasonCode::ProjectionOnlyHasNoAuthority));
    }

    #[test]
    fn single_writer_must_be_the_update_author_and_epoch_must_match() {
        let mut candidate = authorized_transfer("action:wrong-writer", "op-1", Party::A, 10);
        candidate.author = "did:mycelix:bob".into();
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
        assert!(result
            .reason_codes
            .contains(&ReasonCode::InvalidSequencerClaim));
    }

    #[test]
    fn single_writer_does_not_replace_debtor_consent() {
        let mut candidate = authorized_transfer("action:no-debtor-consent", "op-1", Party::B, 10);
        candidate.authorization.participant_signers =
            vec!["did:mycelix:alice".into()];
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
    }

    #[test]
    fn mutual_consent_requires_both_parties() {
        let mut candidate = authorized_transfer("action:mutual", "op-mutual", Party::A, 10);
        candidate.authorization.participant_signers =
            vec!["did:mycelix:alice".into(), "did:mycelix:bob".into()];
        candidate.authorization.sequencing = SequencingClaim::None;
        let result = reconcile(
            &terms(),
            &state(),
            &AuthorizationPolicy::MutualConsent,
            &[candidate],
        );
        assert_eq!(
            result.disposition,
            Disposition::PolicyClaimConsistentPendingFinality
        );
    }

    #[test]
    fn mutual_consent_missing_one_party_is_indeterminate() {
        let mut candidate = authorized_transfer("action:mutual", "op-mutual", Party::A, 10);
        candidate.authorization.sequencing = SequencingClaim::None;
        let result = reconcile(
            &terms(),
            &state(),
            &AuthorizationPolicy::MutualConsent,
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
    }

    fn quorum_policy(threshold: usize) -> AuthorizationPolicy {
        AuthorizationPolicy::WitnessQuorum {
            witness_roster: vec![
                "witness-1".into(),
                "witness-2".into(),
                "witness-3".into(),
                "witness-4".into(),
            ],
            threshold,
            max_byzantine_faults: 1,
        }
    }

    fn quorum_candidate(candidate_ref: &str, operation_id: &str) -> CandidateTransition {
        let mut candidate = authorized_transfer(candidate_ref, operation_id, Party::A, 10);
        candidate.authorization.participant_signers =
            vec!["did:mycelix:alice".into()];
        candidate.authorization.sequencing = SequencingClaim::WitnessQuorum {
            witness_signers: vec![
                "witness-1".into(),
                "witness-2".into(),
                "witness-3".into(),
            ],
        };
        candidate
    }

    #[test]
    fn intersecting_quorum_claim_meets_two_q_greater_than_n_plus_f() {
        let candidate = quorum_candidate("action:q3", "op-q3");
        let result = reconcile(
            &terms(),
            &state(),
            &quorum_policy(3),
            &[candidate],
        );
        assert_eq!(
            result.disposition,
            Disposition::PolicyClaimConsistentPendingFinality
        );
    }

    #[test]
    fn quorum_without_honest_intersection_bound_is_indeterminate() {
        let candidate = quorum_candidate("action:q2", "op-q2");
        let result = reconcile(
            &terms(),
            &state(),
            &quorum_policy(2),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
        assert!(result
            .reason_codes
            .contains(&ReasonCode::InvalidQuorumPolicy));
    }

    #[test]
    fn conflicting_quorum_certified_claims_still_fail_closed() {
        let first = quorum_candidate("action:q1", "op-q1");
        let mut second = quorum_candidate("action:q2", "op-q2");
        second.operation = ChannelOperation::Transfer {
            debit_party: Party::A,
            amount: 15,
        };
        second.authorization.sequencing = SequencingClaim::WitnessQuorum {
            witness_signers: vec![
                "witness-2".into(),
                "witness-3".into(),
                "witness-4".into(),
            ],
        };
        let result = reconcile(
            &terms(),
            &state(),
            &quorum_policy(3),
            &[first, second],
        );
        assert_eq!(result.disposition, Disposition::Conflicted);
    }

    #[test]
    fn close_requires_both_participants_even_under_single_writer_policy() {
        let mut close = authorized_close("action:close");
        close.authorization.participant_signers =
            vec!["did:mycelix:alice".into()];
        let result = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[close],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
    }

    #[test]
    fn closed_state_rejects_all_new_transitions() {
        let mut closed = state();
        closed.closed = true;
        let candidate = authorized_transfer("action:after-close", "op-after-close", Party::A, 1);
        let result = reconcile(
            &terms(),
            &closed,
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Rejected);
        assert_eq!(
            result.rejected_candidates[0].reason,
            RejectReason::ChannelAlreadyClosed
        );
    }

    #[test]
    fn base_state_with_unexplained_balance_delta_is_indeterminate() {
        let mut mutated = state();
        mutated.balance_a += 1;
        let candidate = authorized_transfer("action:1", "op-1", Party::A, 1);
        let result = reconcile(
            &terms(),
            &mutated,
            &single_writer_policy(),
            &[candidate],
        );
        assert_eq!(result.disposition, Disposition::Indeterminate);
        assert!(result.reason_codes.contains(&ReasonCode::InvalidBaseState));
    }

    #[test]
    fn overflow_in_base_total_is_indeterminate() {
        let mut t = terms();
        t.initial_balance_a = u64::MAX;
        t.initial_balance_b = 1;
        let mut s = state();
        s.balance_a = u64::MAX;
        s.balance_b = 1;
        let result = reconcile(&t, &s, &single_writer_policy(), &[]);
        assert_eq!(result.disposition, Disposition::Indeterminate);
    }

    #[test]
    fn deterministic_candidate_order_does_not_change_conflict_disposition() {
        let first = authorized_transfer("action:z", "op-z", Party::A, 10);
        let second = authorized_transfer("action:a", "op-a", Party::A, 20);
        let left = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[first.clone(), second.clone()],
        );
        let right = reconcile(
            &terms(),
            &state(),
            &single_writer_policy(),
            &[second, first],
        );
        assert_eq!(left.disposition, Disposition::Conflicted);
        assert_eq!(right.disposition, Disposition::Conflicted);
        assert_eq!(left.observed_candidate_refs, right.observed_candidate_refs);
    }
}
