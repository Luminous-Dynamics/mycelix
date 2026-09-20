// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B3B1 — process-local atomic spend reference semantics.
//!
//! This crate proves only a narrow process-local state-transition theorem for
//! one canonical query-token spend key. It does not verify Privacy Pass crypto
//! and it deliberately cannot mint a real query credit.

#![forbid(unsafe_code)]

use psi_privacy_pass_credit_core::{
    PrivacyPassChallengeBindingV1, PrivacyPassRedemptionObservationV1, QueryCreditPolicyV1,
    QueryCreditStructuralDisposition, QueryCreditStructuralFailure, Rfc9578TokenType,
    evaluate_redemption_structure_v1,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{collections::BTreeSet, sync::Mutex};

pub const SPEND_KEY_DOMAIN_V1: &str = "mycelix-psi-query-credit-spend-key-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QueryCreditSpendKeyV1 {
    commitment_sha256: String,
    policy_sha256: String,
    challenge_binding_sha256: String,
    token_sha256: String,
    service_domain: String,
    issuer_name: String,
    token_type: Rfc9578TokenType,
    token_key_id_sha256: String,
    budget_epoch: String,
    requested_identifier_count: u32,
}

impl QueryCreditSpendKeyV1 {
    pub fn commitment_sha256(&self) -> &str {
        &self.commitment_sha256
    }

    pub fn policy_sha256(&self) -> &str {
        &self.policy_sha256
    }

    pub fn token_sha256(&self) -> &str {
        &self.token_sha256
    }

    pub fn requested_identifier_count(&self) -> u32 {
        self.requested_identifier_count
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueryCreditSpendFailure {
    StructuralEvaluationFailed(QueryCreditStructuralFailure),
    StructuralIncompatible(QueryCreditStructuralFailure),
    Replay,
    StorePoisoned,
}

pub fn derive_spend_key_v1(
    policy: &QueryCreditPolicyV1,
    challenge: &PrivacyPassChallengeBindingV1,
    observation: &PrivacyPassRedemptionObservationV1,
) -> Result<QueryCreditSpendKeyV1, QueryCreditSpendFailure> {
    let evaluation = evaluate_redemption_structure_v1(policy, challenge, observation)
        .map_err(QueryCreditSpendFailure::StructuralEvaluationFailed)?;

    match evaluation.disposition {
        QueryCreditStructuralDisposition::ReadyForBackendVerification => {}
        QueryCreditStructuralDisposition::Incompatible(reason) => {
            return Err(QueryCreditSpendFailure::StructuralIncompatible(reason));
        }
    }

    let policy_sha256 = policy
        .commitment_sha256()
        .map_err(QueryCreditSpendFailure::StructuralEvaluationFailed)?;
    let challenge_binding_sha256 = challenge
        .commitment_sha256()
        .map_err(QueryCreditSpendFailure::StructuralEvaluationFailed)?;

    let mut bytes = Vec::new();
    append_field(&mut bytes, SPEND_KEY_DOMAIN_V1.as_bytes());
    append_field(&mut bytes, policy_sha256.as_bytes());
    append_field(&mut bytes, challenge_binding_sha256.as_bytes());
    append_field(&mut bytes, observation.token_sha256.as_bytes());
    append_field(&mut bytes, policy.service_domain.as_bytes());
    append_field(&mut bytes, policy.issuer_name.as_bytes());
    append_field(&mut bytes, &policy.token_type.code().to_be_bytes());
    append_field(&mut bytes, policy.token_type.wire_id().as_bytes());
    append_field(&mut bytes, policy.token_key_id_sha256.as_bytes());
    append_field(&mut bytes, policy.budget_epoch.as_bytes());

    Ok(QueryCreditSpendKeyV1 {
        commitment_sha256: sha256_hex(&bytes),
        policy_sha256,
        challenge_binding_sha256,
        token_sha256: observation.token_sha256.clone(),
        service_domain: policy.service_domain.clone(),
        issuer_name: policy.issuer_name.clone(),
        token_type: policy.token_type,
        token_key_id_sha256: policy.token_key_id_sha256.clone(),
        budget_epoch: policy.budget_epoch.clone(),
        requested_identifier_count: observation.requested_identifier_count,
    })
}

#[derive(Debug, Default)]
pub struct ProcessLocalAtomicSpendStoreV1 {
    consumed: Mutex<BTreeSet<String>>,
}

impl ProcessLocalAtomicSpendStoreV1 {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn consume_once(
        &self,
        key: &QueryCreditSpendKeyV1,
    ) -> Result<ProcessLocalConsumedQueryTokenV1, QueryCreditSpendFailure> {
        let mut consumed = self
            .consumed
            .lock()
            .map_err(|_| QueryCreditSpendFailure::StorePoisoned)?;
        if !consumed.insert(key.commitment_sha256.clone()) {
            return Err(QueryCreditSpendFailure::Replay);
        }

        Ok(ProcessLocalConsumedQueryTokenV1 {
            spend_key_sha256: key.commitment_sha256.clone(),
            policy_sha256: key.policy_sha256.clone(),
            challenge_binding_sha256: key.challenge_binding_sha256.clone(),
            token_sha256: key.token_sha256.clone(),
            service_domain: key.service_domain.clone(),
            issuer_name: key.issuer_name.clone(),
            token_type: key.token_type,
            token_key_id_sha256: key.token_key_id_sha256.clone(),
            budget_epoch: key.budget_epoch.clone(),
            requested_identifier_count: key.requested_identifier_count,
        })
    }

    pub fn consumed_count(&self) -> Result<usize, QueryCreditSpendFailure> {
        self.consumed
            .lock()
            .map(|set| set.len())
            .map_err(|_| QueryCreditSpendFailure::StorePoisoned)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProcessLocalConsumedQueryTokenV1 {
    spend_key_sha256: String,
    policy_sha256: String,
    challenge_binding_sha256: String,
    token_sha256: String,
    service_domain: String,
    issuer_name: String,
    token_type: Rfc9578TokenType,
    token_key_id_sha256: String,
    budget_epoch: String,
    requested_identifier_count: u32,
}

impl ProcessLocalConsumedQueryTokenV1 {
    pub fn spend_key_sha256(&self) -> &str {
        &self.spend_key_sha256
    }

    pub fn token_sha256(&self) -> &str {
        &self.token_sha256
    }

    pub const fn process_local_atomic_single_use_established(&self) -> bool {
        true
    }

    pub const fn durable_single_use_established(&self) -> bool {
        false
    }

    pub const fn multi_process_single_use_established(&self) -> bool {
        false
    }

    pub const fn crash_safe_single_use_established(&self) -> bool {
        false
    }

    pub const fn privacy_pass_token_cryptographically_verified(&self) -> bool {
        false
    }

    pub const fn query_credit_granted(&self) -> bool {
        false
    }

    pub const fn anonymous_rate_limit_established(&self) -> bool {
        false
    }

    pub const fn enumeration_resistance_established(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("spend-key semantic fields fit u32");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(field);
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in digest {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Barrier};
    use std::thread;

    fn policy() -> QueryCreditPolicyV1 {
        QueryCreditPolicyV1 {
            service_domain: "contacts.mycelix.test".into(),
            issuer_name: "issuer.mycelix.test".into(),
            issuer_configuration_sha256: "11".repeat(32),
            token_type: Rfc9578TokenType::PrivateVoprfP384Sha384,
            token_key_id_sha256: "22".repeat(32),
            budget_epoch: "epoch-a".into(),
            max_identifiers_per_credit: 256,
            replay_policy: psi_privacy_pass_credit_core::ReplayPolicyV1::AtomicSingleUseRequired,
        }
    }

    fn observation(
        policy: &QueryCreditPolicyV1,
        token_byte: &str,
    ) -> PrivacyPassRedemptionObservationV1 {
        let challenge = policy.challenge_binding().unwrap();
        PrivacyPassRedemptionObservationV1 {
            token_type: policy.token_type,
            issuer_name: policy.issuer_name.clone(),
            token_key_id_sha256: policy.token_key_id_sha256.clone(),
            service_domain: policy.service_domain.clone(),
            budget_epoch: policy.budget_epoch.clone(),
            challenge_binding_sha256: challenge.commitment_sha256().unwrap(),
            token_sha256: token_byte.repeat(32),
            backend_profile: "future-rfc9578-backend-v1".into(),
            backend_receipt_sha256: "44".repeat(32),
            requested_identifier_count: 128,
        }
    }

    #[test]
    fn first_spend_succeeds_and_sequential_replay_fails() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let key = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "33")).unwrap();
        let store = ProcessLocalAtomicSpendStoreV1::new();
        let positive = store.consume_once(&key).unwrap();
        assert!(positive.process_local_atomic_single_use_established());
        assert_eq!(store.consume_once(&key), Err(QueryCreditSpendFailure::Replay));
        assert_eq!(store.consumed_count().unwrap(), 1);
    }

    #[test]
    fn concurrent_race_has_exactly_one_winner() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let key = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "33")).unwrap();
        let store = Arc::new(ProcessLocalAtomicSpendStoreV1::new());
        let barrier = Arc::new(Barrier::new(16));
        let mut joins = Vec::new();
        for _ in 0..16 {
            let store = Arc::clone(&store);
            let barrier = Arc::clone(&barrier);
            let key = key.clone();
            joins.push(thread::spawn(move || {
                barrier.wait();
                store.consume_once(&key).is_ok()
            }));
        }
        let winners = joins.into_iter().filter(|join| join.join().unwrap()).count();
        assert_eq!(winners, 1);
        assert_eq!(store.consumed_count().unwrap(), 1);
    }

    #[test]
    fn token_digest_changes_spend_identity() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let a = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "33")).unwrap();
        let b = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "55")).unwrap();
        assert_ne!(a.commitment_sha256(), b.commitment_sha256());
    }

    #[test]
    fn service_epoch_and_token_key_change_spend_identity() {
        let base = policy();
        let base_challenge = base.challenge_binding().unwrap();
        let base_key = derive_spend_key_v1(&base, &base_challenge, &observation(&base, "33")).unwrap();

        let mut service = base.clone();
        service.service_domain = "other.mycelix.test".into();
        let service_challenge = service.challenge_binding().unwrap();
        let service_key =
            derive_spend_key_v1(&service, &service_challenge, &observation(&service, "33")).unwrap();
        assert_ne!(base_key.commitment_sha256(), service_key.commitment_sha256());

        let mut epoch = base.clone();
        epoch.budget_epoch = "epoch-b".into();
        let epoch_challenge = epoch.challenge_binding().unwrap();
        let epoch_key =
            derive_spend_key_v1(&epoch, &epoch_challenge, &observation(&epoch, "33")).unwrap();
        assert_ne!(base_key.commitment_sha256(), epoch_key.commitment_sha256());

        let mut key_policy = base.clone();
        key_policy.token_key_id_sha256 = "99".repeat(32);
        let key_challenge = key_policy.challenge_binding().unwrap();
        let changed_key = derive_spend_key_v1(
            &key_policy,
            &key_challenge,
            &observation(&key_policy, "33"),
        )
        .unwrap();
        assert_ne!(base_key.commitment_sha256(), changed_key.commitment_sha256());
    }

    #[test]
    fn non_ready_b3a_structure_is_rejected() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let mut observation = observation(&policy, "33");
        observation.token_type = Rfc9578TokenType::PublicBlindRsa2048Sha384;
        assert_eq!(
            derive_spend_key_v1(&policy, &challenge, &observation),
            Err(QueryCreditSpendFailure::StructuralIncompatible(
                QueryCreditStructuralFailure::TokenTypeMismatch
            ))
        );
    }

    #[test]
    fn challenge_cannot_be_borrowed_across_policy() {
        let policy = policy();
        let old_challenge = policy.challenge_binding().unwrap();
        let mut changed = policy.clone();
        changed.max_identifiers_per_credit = 64;
        assert_eq!(
            derive_spend_key_v1(&changed, &old_challenge, &observation(&policy, "33")),
            Err(QueryCreditSpendFailure::StructuralIncompatible(
                QueryCreditStructuralFailure::ChallengeBindingMismatch
            ))
        );
    }

    #[test]
    fn distinct_tokens_may_each_be_consumed_once() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let a = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "33")).unwrap();
        let b = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "55")).unwrap();
        let store = ProcessLocalAtomicSpendStoreV1::new();
        assert!(store.consume_once(&a).is_ok());
        assert!(store.consume_once(&b).is_ok());
        assert_eq!(store.consumed_count().unwrap(), 2);
    }

    #[test]
    fn serialized_positive_preserves_strict_authority_ceiling() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let key = derive_spend_key_v1(&policy, &challenge, &observation(&policy, "33")).unwrap();
        let positive = ProcessLocalAtomicSpendStoreV1::new()
            .consume_once(&key)
            .unwrap();
        let json = serde_json::to_string(&positive).unwrap();
        assert!(json.contains("spend_key_sha256"));
        assert!(positive.process_local_atomic_single_use_established());
        assert!(!positive.durable_single_use_established());
        assert!(!positive.multi_process_single_use_established());
        assert!(!positive.crash_safe_single_use_established());
        assert!(!positive.privacy_pass_token_cryptographically_verified());
        assert!(!positive.query_credit_granted());
        assert!(!positive.anonymous_rate_limit_established());
        assert!(!positive.enumeration_resistance_established());
        assert!(!positive.application_authority_granted());
    }
}
