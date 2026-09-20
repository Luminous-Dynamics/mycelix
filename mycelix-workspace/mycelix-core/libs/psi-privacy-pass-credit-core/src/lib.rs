// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B3A r2 — nonce-correct RFC 9577/9578 structural query-credit semantics.
//!
//! This crate implements no Privacy Pass cryptography. It separates exact
//! token-artifact identity, RFC nonce replay identity, and RFC challenge-digest
//! identity so later adapters can bind all three to one verified token.

#![forbid(unsafe_code)]

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const QUERY_CREDIT_POLICY_DOMAIN_V1: &str = "mycelix-psi-query-credit-policy-v1";
pub const REDEMPTION_CONTEXT_DOMAIN_V1: &str = "mycelix-psi-query-credit-redemption-context-v1";
pub const CHALLENGE_BINDING_DOMAIN_V1: &str = "mycelix-psi-privacy-pass-challenge-binding-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Rfc9578TokenType {
    PrivateVoprfP384Sha384,
    PublicBlindRsa2048Sha384,
}

impl Rfc9578TokenType {
    pub const fn code(self) -> u16 {
        match self {
            Self::PrivateVoprfP384Sha384 => 0x0001,
            Self::PublicBlindRsa2048Sha384 => 0x0002,
        }
    }

    pub const fn wire_id(self) -> &'static str {
        match self {
            Self::PrivateVoprfP384Sha384 => "rfc9578-voprf-p384-sha384-token-type-0001",
            Self::PublicBlindRsa2048Sha384 => "rfc9578-blind-rsa-2048-sha384-token-type-0002",
        }
    }

    pub const fn publicly_verifiable(self) -> bool {
        matches!(self, Self::PublicBlindRsa2048Sha384)
    }

    pub const fn issuer_key_id_profile(self) -> &'static str {
        match self {
            Self::PrivateVoprfP384Sha384 => "sha256-rfc9497-serialize-element-p384-v1",
            Self::PublicBlindRsa2048Sha384 => "sha256-rsassa-pss-spki-v1",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ReplayPolicyV1 {
    AtomicSingleUseRequired,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QueryCreditPolicyV1 {
    pub service_domain: String,
    pub issuer_name: String,
    pub issuer_configuration_sha256: String,
    pub token_type: Rfc9578TokenType,
    pub token_key_id_sha256: String,
    pub budget_epoch: String,
    pub max_identifiers_per_credit: u32,
    pub replay_policy: ReplayPolicyV1,
}

impl QueryCreditPolicyV1 {
    pub fn validate(&self) -> Result<(), QueryCreditStructuralFailure> {
        for value in [&self.service_domain, &self.issuer_name, &self.budget_epoch] {
            if value.trim().is_empty() || !value.is_ascii() {
                return Err(QueryCreditStructuralFailure::InvalidPolicy);
            }
        }
        if self.service_domain.contains(',') || self.issuer_name.contains(',') {
            return Err(QueryCreditStructuralFailure::InvalidPolicy);
        }
        if !is_sha256_hex(&self.issuer_configuration_sha256)
            || !is_sha256_hex(&self.token_key_id_sha256)
        {
            return Err(QueryCreditStructuralFailure::InvalidPolicy);
        }
        if self.max_identifiers_per_credit == 0 {
            return Err(QueryCreditStructuralFailure::ZeroIdentifierBudget);
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, QueryCreditStructuralFailure> {
        self.validate()?;
        let mut out = Vec::new();
        append_field(&mut out, QUERY_CREDIT_POLICY_DOMAIN_V1.as_bytes());
        append_field(&mut out, self.service_domain.as_bytes());
        append_field(&mut out, self.issuer_name.as_bytes());
        append_field(&mut out, self.issuer_configuration_sha256.as_bytes());
        append_field(&mut out, &self.token_type.code().to_be_bytes());
        append_field(&mut out, self.token_type.wire_id().as_bytes());
        append_field(&mut out, self.token_type.issuer_key_id_profile().as_bytes());
        append_field(&mut out, self.token_key_id_sha256.as_bytes());
        append_field(&mut out, self.budget_epoch.as_bytes());
        append_field(&mut out, &self.max_identifiers_per_credit.to_be_bytes());
        append_field(&mut out, b"atomic-single-use-required-v1");
        Ok(sha256_hex(&out))
    }

    pub fn redemption_context_sha256(&self) -> Result<String, QueryCreditStructuralFailure> {
        let policy = self.commitment_sha256()?;
        let mut out = Vec::new();
        append_field(&mut out, REDEMPTION_CONTEXT_DOMAIN_V1.as_bytes());
        append_field(&mut out, self.service_domain.as_bytes());
        append_field(&mut out, self.budget_epoch.as_bytes());
        append_field(&mut out, policy.as_bytes());
        Ok(sha256_hex(&out))
    }

    pub fn challenge_binding(
        &self,
    ) -> Result<PrivacyPassChallengeBindingV1, QueryCreditStructuralFailure> {
        Ok(PrivacyPassChallengeBindingV1 {
            token_type: self.token_type,
            issuer_name: self.issuer_name.clone(),
            redemption_context_sha256: self.redemption_context_sha256()?,
            origin_info: vec![self.service_domain.clone()],
            query_credit_policy_sha256: self.commitment_sha256()?,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivacyPassChallengeBindingV1 {
    pub token_type: Rfc9578TokenType,
    pub issuer_name: String,
    pub redemption_context_sha256: String,
    pub origin_info: Vec<String>,
    pub query_credit_policy_sha256: String,
}

impl PrivacyPassChallengeBindingV1 {
    pub fn validate(&self) -> Result<(), QueryCreditStructuralFailure> {
        if self.issuer_name.trim().is_empty() || !self.issuer_name.is_ascii() {
            return Err(QueryCreditStructuralFailure::MalformedChallengeBinding);
        }
        if !is_sha256_hex(&self.redemption_context_sha256)
            || !is_sha256_hex(&self.query_credit_policy_sha256)
        {
            return Err(QueryCreditStructuralFailure::MalformedChallengeBinding);
        }
        if self.origin_info.len() != 1
            || self.origin_info[0].trim().is_empty()
            || !self.origin_info[0].is_ascii()
        {
            return Err(QueryCreditStructuralFailure::MalformedChallengeBinding);
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, QueryCreditStructuralFailure> {
        self.validate()?;
        let mut out = Vec::new();
        append_field(&mut out, CHALLENGE_BINDING_DOMAIN_V1.as_bytes());
        append_field(&mut out, &self.token_type.code().to_be_bytes());
        append_field(&mut out, self.token_type.wire_id().as_bytes());
        append_field(&mut out, self.issuer_name.as_bytes());
        append_field(&mut out, self.redemption_context_sha256.as_bytes());
        append_field(&mut out, self.origin_info[0].as_bytes());
        append_field(&mut out, self.query_credit_policy_sha256.as_bytes());
        Ok(sha256_hex(&out))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivacyPassRedemptionObservationV1 {
    pub token_type: Rfc9578TokenType,
    pub issuer_name: String,
    pub token_key_id_sha256: String,
    pub service_domain: String,
    pub budget_epoch: String,
    pub challenge_binding_sha256: String,
    pub token_challenge_digest_sha256: String,
    pub token_nonce_sha256: String,
    pub token_sha256: String,
    pub backend_profile: String,
    pub backend_receipt_sha256: String,
    pub requested_identifier_count: u32,
}

impl PrivacyPassRedemptionObservationV1 {
    fn validate(&self) -> Result<(), QueryCreditStructuralFailure> {
        for value in [
            &self.issuer_name,
            &self.service_domain,
            &self.budget_epoch,
            &self.backend_profile,
        ] {
            if value.trim().is_empty() {
                return Err(QueryCreditStructuralFailure::MalformedRedemptionObservation);
            }
        }
        for digest in [
            &self.token_key_id_sha256,
            &self.challenge_binding_sha256,
            &self.token_challenge_digest_sha256,
            &self.token_nonce_sha256,
            &self.token_sha256,
            &self.backend_receipt_sha256,
        ] {
            if !is_sha256_hex(digest) {
                return Err(QueryCreditStructuralFailure::MalformedRedemptionObservation);
            }
        }
        if self.requested_identifier_count == 0 {
            return Err(QueryCreditStructuralFailure::ZeroRequestedIdentifiers);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum QueryCreditStructuralFailure {
    InvalidPolicy,
    ZeroIdentifierBudget,
    MalformedChallengeBinding,
    MalformedRedemptionObservation,
    ZeroRequestedIdentifiers,
    TokenTypeMismatch,
    IssuerMismatch,
    TokenKeyMismatch,
    ServiceDomainMismatch,
    BudgetEpochMismatch,
    ChallengeBindingMismatch,
    IdentifierBudgetExceeded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum QueryCreditStructuralDisposition {
    ReadyForBackendVerification,
    Incompatible(QueryCreditStructuralFailure),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QueryCreditStructuralEvaluationV1 {
    pub disposition: QueryCreditStructuralDisposition,
    pub policy_sha256: String,
    pub challenge_binding_sha256: String,
    pub token_challenge_digest_sha256: String,
    pub token_nonce_sha256: String,
    pub token_sha256: String,
    pub backend_profile: String,
    pub backend_receipt_sha256: String,
    pub requested_identifier_count: u32,
}

impl QueryCreditStructuralEvaluationV1 {
    pub const fn token_cryptographically_verified(&self) -> bool { false }
    pub const fn token_nonce_cryptographically_bound(&self) -> bool { false }
    pub const fn challenge_digest_cryptographically_bound(&self) -> bool { false }
    pub const fn token_unspent_verified(&self) -> bool { false }
    pub const fn token_atomically_consumed(&self) -> bool { false }
    pub const fn query_credit_granted(&self) -> bool { false }
    pub const fn anonymous_rate_limit_established(&self) -> bool { false }
    pub const fn enumeration_resistance_established(&self) -> bool { false }
    pub const fn application_authority_granted(&self) -> bool { false }
}

pub fn evaluate_redemption_structure_v1(
    policy: &QueryCreditPolicyV1,
    challenge: &PrivacyPassChallengeBindingV1,
    observation: &PrivacyPassRedemptionObservationV1,
) -> Result<QueryCreditStructuralEvaluationV1, QueryCreditStructuralFailure> {
    policy.validate()?;
    challenge.validate()?;
    observation.validate()?;

    let policy_sha256 = policy.commitment_sha256()?;
    let expected_challenge = policy.challenge_binding()?;
    let challenge_sha256 = challenge.commitment_sha256()?;
    let expected_challenge_sha256 = expected_challenge.commitment_sha256()?;

    let evaluation = |disposition| QueryCreditStructuralEvaluationV1 {
        disposition,
        policy_sha256: policy_sha256.clone(),
        challenge_binding_sha256: challenge_sha256.clone(),
        token_challenge_digest_sha256: observation.token_challenge_digest_sha256.clone(),
        token_nonce_sha256: observation.token_nonce_sha256.clone(),
        token_sha256: observation.token_sha256.clone(),
        backend_profile: observation.backend_profile.clone(),
        backend_receipt_sha256: observation.backend_receipt_sha256.clone(),
        requested_identifier_count: observation.requested_identifier_count,
    };

    if challenge.token_type != policy.token_type || observation.token_type != policy.token_type {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::TokenTypeMismatch,
        )));
    }
    if challenge.issuer_name != policy.issuer_name || observation.issuer_name != policy.issuer_name {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::IssuerMismatch,
        )));
    }
    if observation.token_key_id_sha256 != policy.token_key_id_sha256 {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::TokenKeyMismatch,
        )));
    }
    if challenge.origin_info != vec![policy.service_domain.clone()]
        || observation.service_domain != policy.service_domain
    {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::ServiceDomainMismatch,
        )));
    }
    if observation.budget_epoch != policy.budget_epoch {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::BudgetEpochMismatch,
        )));
    }
    if challenge.query_credit_policy_sha256 != policy_sha256
        || challenge.redemption_context_sha256 != policy.redemption_context_sha256()?
        || challenge_sha256 != expected_challenge_sha256
        || observation.challenge_binding_sha256 != expected_challenge_sha256
    {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::ChallengeBindingMismatch,
        )));
    }
    if observation.requested_identifier_count > policy.max_identifiers_per_credit {
        return Ok(evaluation(QueryCreditStructuralDisposition::Incompatible(
            QueryCreditStructuralFailure::IdentifierBudgetExceeded,
        )));
    }

    Ok(evaluation(QueryCreditStructuralDisposition::ReadyForBackendVerification))
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("query-credit semantic fields fit u32");
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

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy(token_type: Rfc9578TokenType) -> QueryCreditPolicyV1 {
        QueryCreditPolicyV1 {
            service_domain: "contacts.mycelix.test".into(),
            issuer_name: "issuer.mycelix.test".into(),
            issuer_configuration_sha256: "11".repeat(32),
            token_type,
            token_key_id_sha256: "22".repeat(32),
            budget_epoch: "2026-09-20-a".into(),
            max_identifiers_per_credit: 256,
            replay_policy: ReplayPolicyV1::AtomicSingleUseRequired,
        }
    }

    fn observation(policy: &QueryCreditPolicyV1) -> PrivacyPassRedemptionObservationV1 {
        let challenge = policy.challenge_binding().unwrap();
        PrivacyPassRedemptionObservationV1 {
            token_type: policy.token_type,
            issuer_name: policy.issuer_name.clone(),
            token_key_id_sha256: policy.token_key_id_sha256.clone(),
            service_domain: policy.service_domain.clone(),
            budget_epoch: policy.budget_epoch.clone(),
            challenge_binding_sha256: challenge.commitment_sha256().unwrap(),
            token_challenge_digest_sha256: "33".repeat(32),
            token_nonce_sha256: "44".repeat(32),
            token_sha256: "55".repeat(32),
            backend_profile: "future-rfc9578-backend-v1".into(),
            backend_receipt_sha256: "66".repeat(32),
            requested_identifier_count: 128,
        }
    }

    #[test]
    fn rfc9578_token_type_codes_are_frozen() {
        assert_eq!(Rfc9578TokenType::PrivateVoprfP384Sha384.code(), 0x0001);
        assert_eq!(Rfc9578TokenType::PublicBlindRsa2048Sha384.code(), 0x0002);
        assert!(!Rfc9578TokenType::PrivateVoprfP384Sha384.publicly_verifiable());
        assert!(Rfc9578TokenType::PublicBlindRsa2048Sha384.publicly_verifiable());
    }

    #[test]
    fn policy_changes_change_redemption_context() {
        let a = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let mut b = a.clone();
        b.budget_epoch = "2026-09-20-b".into();
        assert_ne!(a.redemption_context_sha256().unwrap(), b.redemption_context_sha256().unwrap());
        let mut c = a.clone();
        c.max_identifiers_per_credit = 64;
        assert_ne!(a.redemption_context_sha256().unwrap(), c.redemption_context_sha256().unwrap());
    }

    #[test]
    fn challenge_is_single_origin_service_scoped() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        assert_eq!(challenge.origin_info, vec![p.service_domain.clone()]);
        assert_eq!(challenge.redemption_context_sha256.len(), 64);
    }

    #[test]
    fn structurally_matching_redemption_is_not_a_query_credit() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        let result = evaluate_redemption_structure_v1(&p, &challenge, &observation(&p)).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::ReadyForBackendVerification);
        assert!(!result.token_cryptographically_verified());
        assert!(!result.token_nonce_cryptographically_bound());
        assert!(!result.challenge_digest_cryptographically_bound());
        assert!(!result.token_unspent_verified());
        assert!(!result.token_atomically_consumed());
        assert!(!result.query_credit_granted());
    }

    #[test]
    fn token_type_cannot_be_substituted() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        let mut obs = observation(&p);
        obs.token_type = Rfc9578TokenType::PublicBlindRsa2048Sha384;
        let result = evaluate_redemption_structure_v1(&p, &challenge, &obs).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::Incompatible(QueryCreditStructuralFailure::TokenTypeMismatch));
    }

    #[test]
    fn token_key_cannot_be_substituted() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        let mut obs = observation(&p);
        obs.token_key_id_sha256 = "99".repeat(32);
        let result = evaluate_redemption_structure_v1(&p, &challenge, &obs).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::Incompatible(QueryCreditStructuralFailure::TokenKeyMismatch));
    }

    #[test]
    fn cross_service_challenge_is_rejected() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let mut challenge = p.challenge_binding().unwrap();
        challenge.origin_info = vec!["other.mycelix.test".into()];
        let result = evaluate_redemption_structure_v1(&p, &challenge, &observation(&p)).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::Incompatible(QueryCreditStructuralFailure::ServiceDomainMismatch));
    }

    #[test]
    fn old_budget_epoch_cannot_be_reinterpreted() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        let mut obs = observation(&p);
        obs.budget_epoch = "old-epoch".into();
        let result = evaluate_redemption_structure_v1(&p, &challenge, &obs).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::Incompatible(QueryCreditStructuralFailure::BudgetEpochMismatch));
    }

    #[test]
    fn policy_binding_cannot_be_borrowed_after_budget_change() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let old_challenge = p.challenge_binding().unwrap();
        let mut changed = p.clone();
        changed.max_identifiers_per_credit = 64;
        let result = evaluate_redemption_structure_v1(&changed, &old_challenge, &observation(&p)).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::Incompatible(QueryCreditStructuralFailure::ChallengeBindingMismatch));
    }

    #[test]
    fn identifier_budget_is_enforced_structurally() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        let mut obs = observation(&p);
        obs.requested_identifier_count = 257;
        let result = evaluate_redemption_structure_v1(&p, &challenge, &obs).unwrap();
        assert_eq!(result.disposition, QueryCreditStructuralDisposition::Incompatible(QueryCreditStructuralFailure::IdentifierBudgetExceeded));
    }

    #[test]
    fn nonce_challenge_and_token_identities_are_kept_distinct() {
        let p = policy(Rfc9578TokenType::PrivateVoprfP384Sha384);
        let challenge = p.challenge_binding().unwrap();
        let result = evaluate_redemption_structure_v1(&p, &challenge, &observation(&p)).unwrap();
        assert_eq!(result.token_challenge_digest_sha256, "33".repeat(32));
        assert_eq!(result.token_nonce_sha256, "44".repeat(32));
        assert_eq!(result.token_sha256, "55".repeat(32));
        assert_ne!(result.token_nonce_sha256, result.token_sha256);
    }
}
