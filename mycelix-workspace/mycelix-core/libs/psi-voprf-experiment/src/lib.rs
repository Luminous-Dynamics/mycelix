// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002A — synthetic RFC 9497 VOPRF-assisted contact-discovery experiment.
//!
//! This crate is deliberately narrow:
//!
//! - synthetic identifiers only;
//! - exact `voprf = 0.5.0`;
//! - RFC 9497 VOPRF with the Ristretto255/SHA-512 ciphersuite;
//! - two-party set intersection with client-only output;
//! - structural profile bound to canonical PEC-002A subject `41a26efa...`.
//!
//! Governing boundary:
//!
//! ```text
//! VOPRF proof verifies
//!     != PSI security established
//!     != enumeration resistance
//!     != client anonymity
//!     != production admission
//! ```
//!
//! The experiment intentionally exposes an online-guessing oracle in tests to
//! demonstrate why VOPRF alone does not solve low-entropy contact enumeration.

use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageDeclaration, ParticipantModel,
    PrivacyPrimitive, QualificationState,
};
use privacy_protocol_profiles::{
    CollectionSemantics, PsiLeakageProfile, PsiOperation, PsiOutputRecipient, PsiProfile,
    ProtocolProfile,
};
use rand_core::{CryptoRng, RngCore};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use voprf::{CipherSuite, Group, Ristretto255, VoprfClient, VoprfServer};

pub const PROTOCOL_ID: &str = "PSI-002A";
pub const CONSTRUCTION_ID: &str = "voprf-tagged-set-v1";
pub const BACKEND_NAME: &str = "voprf";
pub const BACKEND_VERSION: &str = "0.5.0";
pub const BACKEND_PROFILE: &str = "rfc9497-ristretto255-sha512-voprf-tagged-set-v1";
pub const PEC_PROFILE_SUBJECT: &str = "41a26efa89435fbc328bb5ac68b9e971f4b162cd";
pub const SYNTHETIC_PREFIX: &str = "syn-contact-v1:";
pub const EQUALITY_DOMAIN: &str = "synthetic-contact-id-ascii-lower-v1";
pub const CANONICALIZATION_PROFILE: &str = "ascii-trim-lower-synthetic-contact-v1";
pub const MAX_IDENTIFIER_BYTES: usize = 128;
pub const MAX_DOMAIN_BYTES: usize = 128;
pub const MAX_SET_SIZE: usize = 4096;

type Suite = Ristretto255;
pub type ExperimentServer = VoprfServer<Suite>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExperimentError {
    EmptyServiceDomain,
    EmptySessionDomain,
    InvalidDomain,
    InvalidSyntheticIdentifier,
    IdentifierTooLong,
    TooManyIdentifiers,
    DuplicateCanonicalIdentifier,
    InvalidStructuralProfile,
    VoprfServerSetup,
    VoprfBlind,
    VoprfServerEvaluate,
    VoprfFinalize,
    TagCollision,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SyntheticContactId(String);

impl SyntheticContactId {
    pub fn canonicalize(raw: &str) -> Result<Self, ExperimentError> {
        let trimmed = raw.trim();
        if trimmed.is_empty() || trimmed.len() > MAX_IDENTIFIER_BYTES || !trimmed.is_ascii() {
            return Err(if trimmed.len() > MAX_IDENTIFIER_BYTES {
                ExperimentError::IdentifierTooLong
            } else {
                ExperimentError::InvalidSyntheticIdentifier
            });
        }

        let canonical = trimmed.to_ascii_lowercase();
        let Some(local) = canonical.strip_prefix(SYNTHETIC_PREFIX) else {
            return Err(ExperimentError::InvalidSyntheticIdentifier);
        };
        if local.is_empty()
            || !local
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.'))
        {
            return Err(ExperimentError::InvalidSyntheticIdentifier);
        }

        Ok(Self(canonical))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentConfig {
    pub service_domain: String,
    pub session_domain: String,
}

impl ExperimentConfig {
    pub fn new(
        service_domain: impl Into<String>,
        session_domain: impl Into<String>,
    ) -> Result<Self, ExperimentError> {
        let value = Self {
            service_domain: service_domain.into(),
            session_domain: session_domain.into(),
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ExperimentError> {
        validate_domain(&self.service_domain, true)?;
        validate_domain(&self.session_domain, false)?;
        Ok(())
    }

    pub fn psi_profile(&self) -> Result<PsiProfile, ExperimentError> {
        self.validate()?;
        let profile = PsiProfile {
            backend: BackendIdentity {
                primitive: PrivacyPrimitive::PrivateSetOperation,
                backend: BACKEND_NAME.into(),
                version: BACKEND_VERSION.into(),
                profile: BACKEND_PROFILE.into(),
            },
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 2 },
            operation: PsiOperation::Intersection,
            collection_semantics: CollectionSemantics::Set,
            output_recipient: PsiOutputRecipient::ClientOnly,
            equality_domain: EQUALITY_DOMAIN.into(),
            session_domain: experiment_session_domain(self),
            aggregate: None,
            leakage: PsiLeakageProfile {
                input_sizes: LeakageDeclaration::MayReveal,
                result_cardinality: LeakageDeclaration::MayReveal,
                result_elements: LeakageDeclaration::MayReveal,
                aggregate_value: LeakageDeclaration::Unspecified,
                timing: LeakageDeclaration::MayReveal,
                message_size: LeakageDeclaration::MayReveal,
                abort_behavior: LeakageDeclaration::MayReveal,
                cross_session_linkability: LeakageDeclaration::MayReveal,
            },
            qualification: QualificationState::Experimental,
        };
        profile
            .validate()
            .map_err(|_| ExperimentError::InvalidStructuralProfile)?;
        Ok(profile)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentMetrics {
    pub client_items: usize,
    pub server_items: usize,
    pub intersection_items: usize,
    pub blind_requests: usize,
    pub verified_responses: usize,
    pub client_to_server_bytes: usize,
    pub server_to_client_bytes: usize,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentReceipt {
    pub protocol: String,
    pub construction: String,
    pub backend: String,
    pub backend_version: String,
    pub backend_profile: String,
    pub ciphersuite: String,
    pub pec_profile_subject: String,
    pub canonicalization_profile: String,
    pub equality_domain: String,
    pub service_domain: String,
    pub session_domain: String,
    pub profile_digest_sha256: String,
    pub server_public_key_sha256: String,
    pub client_snapshot_sha256: String,
    pub server_snapshot_sha256: String,
    pub result_elements: Vec<String>,
    pub metrics: ExperimentMetrics,
    pub qualification: QualificationState,
}

impl ExperimentReceipt {
    pub const fn psi_security_established(&self) -> bool {
        false
    }

    pub const fn enumeration_resistance_established(&self) -> bool {
        false
    }

    pub const fn client_anonymity_established(&self) -> bool {
        false
    }

    pub const fn production_admission_granted(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

pub fn deterministic_test_server() -> Result<ExperimentServer, ExperimentError> {
    // Determinism is exclusively for the synthetic evidence corpus. This is not
    // a production key-generation policy.
    let seed = [0x42_u8; 32];
    ExperimentServer::new_from_seed(&seed, BACKEND_PROFILE.as_bytes())
        .map_err(|_| ExperimentError::VoprfServerSetup)
}

pub fn run_synthetic_intersection<R: RngCore + CryptoRng>(
    client_raw: &[&str],
    server_raw: &[&str],
    config: &ExperimentConfig,
    server: &ExperimentServer,
    rng: &mut R,
) -> Result<ExperimentReceipt, ExperimentError> {
    config.validate()?;
    let profile = config.psi_profile()?;
    let client = canonicalize_set(client_raw)?;
    let registry = canonicalize_set(server_raw)?;

    let mut registry_tags = BTreeSet::<Vec<u8>>::new();
    for id in &registry {
        let input = domain_separated_input(id, config)?;
        let tag = server
            .evaluate(&input)
            .map_err(|_| ExperimentError::VoprfServerEvaluate)?
            .to_vec();
        if !registry_tags.insert(tag) {
            return Err(ExperimentError::TagCollision);
        }
    }

    let mut result_elements = Vec::new();
    let mut client_to_server_bytes = 0usize;
    let mut server_to_client_bytes = 0usize;
    let mut verified_responses = 0usize;

    for id in &client {
        let input = domain_separated_input(id, config)?;
        let blind = VoprfClient::<Suite>::blind(&input, rng)
            .map_err(|_| ExperimentError::VoprfBlind)?;
        client_to_server_bytes =
            client_to_server_bytes.saturating_add(blind.message.serialize().len());

        let evaluated = server.blind_evaluate(rng, &blind.message);
        server_to_client_bytes = server_to_client_bytes
            .saturating_add(evaluated.message.serialize().len())
            .saturating_add(evaluated.proof.serialize().len());

        let tag = blind
            .state
            .finalize(
                &input,
                &evaluated.message,
                &evaluated.proof,
                server.get_public_key(),
            )
            .map_err(|_| ExperimentError::VoprfFinalize)?
            .to_vec();
        verified_responses = verified_responses.saturating_add(1);

        if registry_tags.contains(&tag) {
            result_elements.push(id.as_str().to_owned());
        }
    }

    let wrapped = ProtocolProfile::Psi(profile);
    debug_assert!(!wrapped.cryptographic_security_established());
    debug_assert!(!wrapped.production_admission_granted());
    debug_assert!(!wrapped.application_authority_granted());

    Ok(ExperimentReceipt {
        protocol: PROTOCOL_ID.into(),
        construction: CONSTRUCTION_ID.into(),
        backend: BACKEND_NAME.into(),
        backend_version: BACKEND_VERSION.into(),
        backend_profile: BACKEND_PROFILE.into(),
        ciphersuite: <Suite as CipherSuite>::ID.into(),
        pec_profile_subject: PEC_PROFILE_SUBJECT.into(),
        canonicalization_profile: CANONICALIZATION_PROFILE.into(),
        equality_domain: EQUALITY_DOMAIN.into(),
        service_domain: config.service_domain.clone(),
        session_domain: config.session_domain.clone(),
        profile_digest_sha256: profile_digest(config),
        server_public_key_sha256: server_public_key_digest(server),
        client_snapshot_sha256: snapshot_digest(&client),
        server_snapshot_sha256: snapshot_digest(&registry),
        metrics: ExperimentMetrics {
            client_items: client.len(),
            server_items: registry.len(),
            intersection_items: result_elements.len(),
            blind_requests: client.len(),
            verified_responses,
            client_to_server_bytes,
            server_to_client_bytes,
        },
        result_elements,
        qualification: QualificationState::Experimental,
    })
}

/// Demonstrates the online-enumeration boundary: an actor with oracle access can
/// test guesses one by one. A successful match here is evidence that VOPRF alone
/// does not establish enumeration resistance.
pub fn oracle_tag_for_guess<R: RngCore + CryptoRng>(
    raw_guess: &str,
    config: &ExperimentConfig,
    server: &ExperimentServer,
    rng: &mut R,
) -> Result<Vec<u8>, ExperimentError> {
    let id = SyntheticContactId::canonicalize(raw_guess)?;
    let input = domain_separated_input(&id, config)?;
    let blind =
        VoprfClient::<Suite>::blind(&input, rng).map_err(|_| ExperimentError::VoprfBlind)?;
    let evaluated = server.blind_evaluate(rng, &blind.message);
    blind
        .state
        .finalize(
            &input,
            &evaluated.message,
            &evaluated.proof,
            server.get_public_key(),
        )
        .map(|output| output.to_vec())
        .map_err(|_| ExperimentError::VoprfFinalize)
}

pub fn server_tag_for_known_identifier(
    raw: &str,
    config: &ExperimentConfig,
    server: &ExperimentServer,
) -> Result<Vec<u8>, ExperimentError> {
    let id = SyntheticContactId::canonicalize(raw)?;
    let input = domain_separated_input(&id, config)?;
    server
        .evaluate(&input)
        .map(|output| output.to_vec())
        .map_err(|_| ExperimentError::VoprfServerEvaluate)
}

pub fn domain_separated_input(
    id: &SyntheticContactId,
    config: &ExperimentConfig,
) -> Result<Vec<u8>, ExperimentError> {
    config.validate()?;
    let mut out = Vec::new();
    append_field(&mut out, PROTOCOL_ID.as_bytes());
    append_field(&mut out, CONSTRUCTION_ID.as_bytes());
    append_field(&mut out, BACKEND_PROFILE.as_bytes());
    append_field(&mut out, <Suite as CipherSuite>::ID.as_bytes());
    append_field(&mut out, EQUALITY_DOMAIN.as_bytes());
    append_field(&mut out, config.service_domain.as_bytes());
    append_field(&mut out, config.session_domain.as_bytes());
    append_field(&mut out, id.as_str().as_bytes());
    Ok(out)
}

fn validate_domain(value: &str, service: bool) -> Result<(), ExperimentError> {
    if value.is_empty() {
        return Err(if service {
            ExperimentError::EmptyServiceDomain
        } else {
            ExperimentError::EmptySessionDomain
        });
    }
    if value.len() > MAX_DOMAIN_BYTES
        || !value.is_ascii()
        || !value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b':' | b'/'))
    {
        return Err(ExperimentError::InvalidDomain);
    }
    Ok(())
}

fn canonicalize_set(raw: &[&str]) -> Result<Vec<SyntheticContactId>, ExperimentError> {
    if raw.len() > MAX_SET_SIZE {
        return Err(ExperimentError::TooManyIdentifiers);
    }
    let mut set = BTreeSet::new();
    for value in raw {
        let id = SyntheticContactId::canonicalize(value)?;
        if !set.insert(id) {
            return Err(ExperimentError::DuplicateCanonicalIdentifier);
        }
    }
    Ok(set.into_iter().collect())
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("experiment fields are bounded below u32::MAX");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(field);
}

fn snapshot_digest(ids: &[SyntheticContactId]) -> String {
    let mut bytes = Vec::new();
    append_field(&mut bytes, b"psi-002a-snapshot-v1");
    for id in ids {
        append_field(&mut bytes, id.as_str().as_bytes());
    }
    sha256_hex(&bytes)
}

fn experiment_session_domain(config: &ExperimentConfig) -> String {
    let mut bytes = Vec::new();
    append_field(&mut bytes, b"psi-002a-session-domain-v1");
    append_field(&mut bytes, config.service_domain.as_bytes());
    append_field(&mut bytes, config.session_domain.as_bytes());
    format!("psi-002a-scope-sha256:{}", sha256_hex(&bytes))
}

fn profile_digest(config: &ExperimentConfig) -> String {
    let mut bytes = Vec::new();
    for field in [
        PROTOCOL_ID,
        CONSTRUCTION_ID,
        BACKEND_NAME,
        BACKEND_VERSION,
        BACKEND_PROFILE,
        <Suite as CipherSuite>::ID,
        PEC_PROFILE_SUBJECT,
        CANONICALIZATION_PROFILE,
        EQUALITY_DOMAIN,
        &config.service_domain,
        &config.session_domain,
    ] {
        append_field(&mut bytes, field.as_bytes());
    }
    sha256_hex(&bytes)
}

fn server_public_key_digest(server: &ExperimentServer) -> String {
    let bytes = <Suite as Group>::serialize_elem(server.get_public_key());
    sha256_hex(bytes.as_slice())
}

fn sha256_hex(input: &[u8]) -> String {
    let digest = Sha256::digest(input);
    let mut out = String::with_capacity(digest.len() * 2);
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
    use rand::{rngs::StdRng, SeedableRng};
    use voprf::BlindedElement;

    fn config() -> ExperimentConfig {
        ExperimentConfig::new("mycelix.test", "session-001").unwrap()
    }

    fn rng() -> StdRng {
        StdRng::from_seed([0x24; 32])
    }

    #[test]
    fn canonical_profile_is_exactly_experimental_and_non_authoritative() {
        let profile = config().psi_profile().unwrap();
        assert_eq!(profile.operation, PsiOperation::Intersection);
        assert_eq!(profile.collection_semantics, CollectionSemantics::Set);
        assert_eq!(profile.output_recipient, PsiOutputRecipient::ClientOnly);
        assert_eq!(profile.qualification, QualificationState::Experimental);

        let wrapped = ProtocolProfile::Psi(profile);
        assert!(!wrapped.cryptographic_security_established());
        assert!(!wrapped.production_admission_granted());
        assert!(!wrapped.application_authority_granted());
    }

    #[test]
    fn canonicalization_is_deterministic_and_duplicate_aliases_fail_closed() {
        let canonical = SyntheticContactId::canonicalize("  SYN-CONTACT-V1:Alice_01 ").unwrap();
        assert_eq!(canonical.as_str(), "syn-contact-v1:alice_01");

        let duplicate = canonicalize_set(&[
            "syn-contact-v1:alice_01",
            " SYN-CONTACT-V1:ALICE_01 ",
        ]);
        assert_eq!(
            duplicate,
            Err(ExperimentError::DuplicateCanonicalIdentifier)
        );
    }

    #[test]
    fn service_and_session_domains_change_protocol_input() {
        let id = SyntheticContactId::canonicalize("syn-contact-v1:alice").unwrap();
        let a = domain_separated_input(
            &id,
            &ExperimentConfig::new("service-a", "session-1").unwrap(),
        )
        .unwrap();
        let b = domain_separated_input(
            &id,
            &ExperimentConfig::new("service-b", "session-1").unwrap(),
        )
        .unwrap();
        let c = domain_separated_input(
            &id,
            &ExperimentConfig::new("service-a", "session-2").unwrap(),
        )
        .unwrap();
        assert_ne!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn voprf_client_result_matches_server_direct_evaluation() {
        let server = deterministic_test_server().unwrap();
        let config = config();
        let mut rng = rng();
        let online = oracle_tag_for_guess(
            "syn-contact-v1:alice",
            &config,
            &server,
            &mut rng,
        )
        .unwrap();
        let direct =
            server_tag_for_known_identifier("syn-contact-v1:alice", &config, &server).unwrap();
        assert_eq!(online, direct);
    }

    #[test]
    fn synthetic_intersection_is_exact_and_receipt_keeps_claim_ceiling() {
        let server = deterministic_test_server().unwrap();
        let mut rng = rng();
        let receipt = run_synthetic_intersection(
            &[
                "syn-contact-v1:alice",
                "syn-contact-v1:bob",
                "syn-contact-v1:carol",
            ],
            &[
                "syn-contact-v1:dave",
                "syn-contact-v1:bob",
                "syn-contact-v1:carol",
            ],
            &config(),
            &server,
            &mut rng,
        )
        .unwrap();

        assert_eq!(
            receipt.result_elements,
            vec![
                "syn-contact-v1:bob".to_string(),
                "syn-contact-v1:carol".to_string()
            ]
        );
        assert_eq!(receipt.metrics.intersection_items, 2);
        assert_eq!(receipt.metrics.verified_responses, 3);
        assert!(receipt.metrics.client_to_server_bytes > 0);
        assert!(receipt.metrics.server_to_client_bytes > 0);
        assert_eq!(receipt.qualification, QualificationState::Experimental);
        assert!(!receipt.psi_security_established());
        assert!(!receipt.enumeration_resistance_established());
        assert!(!receipt.client_anonymity_established());
        assert!(!receipt.production_admission_granted());
        assert!(!receipt.application_authority_granted());
    }

    #[test]
    fn same_server_key_does_not_reuse_tags_across_service_domains() {
        let server = deterministic_test_server().unwrap();
        let a = server_tag_for_known_identifier(
            "syn-contact-v1:alice",
            &ExperimentConfig::new("service-a", "session-1").unwrap(),
            &server,
        )
        .unwrap();
        let b = server_tag_for_known_identifier(
            "syn-contact-v1:alice",
            &ExperimentConfig::new("service-b", "session-1").unwrap(),
            &server,
        )
        .unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn online_dictionary_guess_can_still_test_low_entropy_identifier() {
        let server = deterministic_test_server().unwrap();
        let config = config();
        let target =
            server_tag_for_known_identifier("syn-contact-v1:bob", &config, &server).unwrap();

        let mut rng = rng();
        let guesses = ["syn-contact-v1:alice", "syn-contact-v1:bob"];
        let mut found = false;
        for guess in guesses {
            if oracle_tag_for_guess(guess, &config, &server, &mut rng).unwrap() == target {
                found = true;
                break;
            }
        }
        assert!(
            found,
            "VOPRF alone must not be treated as enumeration resistance"
        );
    }

    #[test]
    fn snapshot_commitment_changes_when_registry_changes() {
        let server = deterministic_test_server().unwrap();
        let mut rng_a = rng();
        let mut rng_b = rng();
        let a = run_synthetic_intersection(
            &["syn-contact-v1:alice"],
            &["syn-contact-v1:alice"],
            &config(),
            &server,
            &mut rng_a,
        )
        .unwrap();
        let b = run_synthetic_intersection(
            &["syn-contact-v1:alice"],
            &["syn-contact-v1:alice", "syn-contact-v1:bob"],
            &config(),
            &server,
            &mut rng_b,
        )
        .unwrap();
        assert_ne!(a.server_snapshot_sha256, b.server_snapshot_sha256);
    }

    #[test]
    fn malformed_blinded_element_is_rejected() {
        let invalid = [0_u8; 32];
        assert!(BlindedElement::<Suite>::deserialize(&invalid).is_err());
    }

    #[test]
    fn proof_from_different_blinded_input_is_rejected() {
        let server = deterministic_test_server().unwrap();
        let config = config();
        let alice = SyntheticContactId::canonicalize("syn-contact-v1:alice").unwrap();
        let bob = SyntheticContactId::canonicalize("syn-contact-v1:bob").unwrap();
        let alice_input = domain_separated_input(&alice, &config).unwrap();
        let bob_input = domain_separated_input(&bob, &config).unwrap();
        let mut rng = rng();

        let alice_blind = VoprfClient::<Suite>::blind(&alice_input, &mut rng).unwrap();
        let bob_blind = VoprfClient::<Suite>::blind(&bob_input, &mut rng).unwrap();
        let bob_evaluation = server.blind_evaluate(&mut rng, &bob_blind.message);

        let result = alice_blind.state.finalize(
            &alice_input,
            &bob_evaluation.message,
            &bob_evaluation.proof,
            server.get_public_key(),
        );
        assert!(matches!(result, Err(voprf::Error::ProofVerification)));
    }

    #[test]
    fn oversized_query_set_is_rejected_before_protocol_work() {
        let raw = vec!["syn-contact-v1:x"; MAX_SET_SIZE + 1];
        assert_eq!(
            canonicalize_set(&raw),
            Err(ExperimentError::TooManyIdentifiers)
        );
    }
}
