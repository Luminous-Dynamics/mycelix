// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002A: synthetic-only RFC 9497 VOPRF-assisted contact discovery.
//!
//! The client/server role boundary is explicit: raw client identifiers never
//! enter a server API. The client blinds locally, the server evaluates only
//! blinded elements, and the client verifies/finalizes locally against a
//! registry containing only VOPRF-derived tags.
//!
//! This remains an experiment, not a qualified PSI backend.

use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageDeclaration, ParticipantModel,
    PrivacyPrimitive, QualificationState,
};
use privacy_protocol_profiles::{
    CollectionSemantics, PsiLeakageProfile, PsiOperation, PsiOutputRecipient, PsiProfile,
};
use rand_core::{CryptoRng, RngCore};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt;
use voprf::{
    BlindedElement, Group, Ristretto255, VoprfClient, VoprfServer, VoprfServerEvaluateResult,
};

const CONSTRUCTION: &str = "voprf-tagged-set-v2";
const BACKEND: &str = "facebook/voprf";
const BACKEND_VERSION: &str = "0.5.0";
const BACKEND_TAG: &str = "v0.5.0";
const BACKEND_SOURCE_COMMIT: &str = "f0531f0812387cd6be01923b21e2157399a9b295";
const CIPHERSUITE: &str = "ristretto255-SHA512";
const CANONICALIZATION_PROFILE: &str = "synthetic-ascii-lowercase-v1";
const INPUT_DOMAIN: &[u8] = b"mycelix-psi-002a-v2";
const SERVER_KEY_INFO: &[u8] = b"mycelix-psi-002a-server-key-v1";
pub const MAX_IDENTIFIERS: usize = 1_024;
pub const MAX_IDENTIFIER_BYTES: usize = 96;
pub const MAX_DOMAIN_COMPONENT_BYTES: usize = 96;

pub type VoprfPublicKey = <Ristretto255 as Group>::Elem;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DiscoveryError {
    InvalidSyntheticIdentifier,
    InvalidDomain,
    EmptyInputSet,
    TooManyIdentifiers,
    RequestCommitmentMismatch,
    ResponseCountMismatch,
    DomainMismatch,
    ServerKeyMismatch,
    Voprf(String),
}

impl fmt::Display for DiscoveryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSyntheticIdentifier => write!(f, "invalid synthetic identifier"),
            Self::InvalidDomain => write!(f, "invalid discovery domain"),
            Self::EmptyInputSet => write!(f, "input set must not be empty"),
            Self::TooManyIdentifiers => write!(f, "input set exceeds experiment ceiling"),
            Self::RequestCommitmentMismatch => write!(f, "request commitment mismatch"),
            Self::ResponseCountMismatch => write!(f, "response count mismatch"),
            Self::DomainMismatch => write!(f, "domain mismatch"),
            Self::ServerKeyMismatch => write!(f, "server key mismatch"),
            Self::Voprf(error) => write!(f, "VOPRF failure: {error}"),
        }
    }
}

impl std::error::Error for DiscoveryError {}

fn map_voprf<E: fmt::Debug>(error: E) -> DiscoveryError {
    DiscoveryError::Voprf(format!("{error:?}"))
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SyntheticIdentifier(String);

impl SyntheticIdentifier {
    pub fn new(value: impl Into<String>) -> Result<Self, DiscoveryError> {
        let value = value.into();
        let bytes = value.as_bytes();
        if bytes.is_empty()
            || bytes.len() > MAX_IDENTIFIER_BYTES
            || !value.starts_with("synthetic-contact-")
            || value.len() == "synthetic-contact-".len()
            || !bytes.iter().all(|byte| {
                byte.is_ascii_lowercase()
                    || byte.is_ascii_digit()
                    || matches!(*byte, b'-' | b'_')
            })
        {
            return Err(DiscoveryError::InvalidSyntheticIdentifier);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DiscoveryDomain {
    service: String,
    profile: String,
    session: String,
}

impl DiscoveryDomain {
    pub fn new(
        service: impl Into<String>,
        profile: impl Into<String>,
        session: impl Into<String>,
    ) -> Result<Self, DiscoveryError> {
        let domain = Self {
            service: service.into(),
            profile: profile.into(),
            session: session.into(),
        };
        if !valid_domain_component(&domain.service)
            || !valid_domain_component(&domain.profile)
            || !valid_domain_component(&domain.session)
        {
            return Err(DiscoveryError::InvalidDomain);
        }
        Ok(domain)
    }

    pub fn service(&self) -> &str {
        &self.service
    }

    pub fn profile(&self) -> &str {
        &self.profile
    }

    pub fn session(&self) -> &str {
        &self.session
    }

    fn equality_domain(&self) -> String {
        format!("{}:{}", self.service, self.profile)
    }

    fn commitment(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(b"mycelix-psi-002a-domain-v1");
        for component in [&self.service, &self.profile, &self.session] {
            hasher.update((component.len() as u64).to_be_bytes());
            hasher.update(component.as_bytes());
        }
        hasher.finalize().into()
    }
}

fn valid_domain_component(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_DOMAIN_COMPONENT_BYTES
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b':' | b'/')
        })
}

fn push_len_prefixed(buffer: &mut Vec<u8>, value: &[u8]) {
    let length = u16::try_from(value.len()).expect("validated experiment field length fits u16");
    buffer.extend_from_slice(&length.to_be_bytes());
    buffer.extend_from_slice(value);
}

fn protocol_input(domain: &DiscoveryDomain, identifier: &SyntheticIdentifier) -> Vec<u8> {
    let mut input = Vec::with_capacity(
        INPUT_DOMAIN.len()
            + domain.service.len()
            + domain.profile.len()
            + domain.session.len()
            + identifier.0.len()
            + 12,
    );
    push_len_prefixed(&mut input, INPUT_DOMAIN);
    push_len_prefixed(&mut input, CANONICALIZATION_PROFILE.as_bytes());
    push_len_prefixed(&mut input, domain.service.as_bytes());
    push_len_prefixed(&mut input, domain.profile.as_bytes());
    push_len_prefixed(&mut input, domain.session.as_bytes());
    push_len_prefixed(&mut input, identifier.0.as_bytes());
    input
}

fn canonical_set(
    identifiers: &[SyntheticIdentifier],
) -> Result<BTreeSet<SyntheticIdentifier>, DiscoveryError> {
    if identifiers.is_empty() {
        return Err(DiscoveryError::EmptyInputSet);
    }
    if identifiers.len() > MAX_IDENTIFIERS {
        return Err(DiscoveryError::TooManyIdentifiers);
    }
    Ok(identifiers.iter().cloned().collect())
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DiscoveryTag(Vec<u8>);

impl DiscoveryTag {
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }
}

fn tag_set_commitment(tags: &BTreeSet<DiscoveryTag>) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(b"mycelix-psi-002a-tag-set-v1");
    for tag in tags {
        hasher.update((tag.0.len() as u64).to_be_bytes());
        hasher.update(&tag.0);
    }
    hasher.finalize().into()
}

fn public_key_sha256(public_key: VoprfPublicKey) -> [u8; 32] {
    let encoded = <Ristretto255 as Group>::serialize_elem(public_key);
    Sha256::digest(encoded).into()
}

fn request_commitment(messages: &[BlindedElement<Ristretto255>]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(b"mycelix-psi-002a-blinded-request-v1");
    hasher.update((messages.len() as u64).to_be_bytes());
    for message in messages {
        let encoded = message.serialize();
        hasher.update((encoded.len() as u64).to_be_bytes());
        hasher.update(encoded);
    }
    hasher.finalize().into()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreparedRegistry {
    tags: BTreeSet<DiscoveryTag>,
    tag_snapshot_sha256: [u8; 32],
    domain_sha256: [u8; 32],
    server_public_key_sha256: [u8; 32],
    unique_count: usize,
}

impl PreparedRegistry {
    pub fn tag_snapshot_sha256(&self) -> [u8; 32] {
        self.tag_snapshot_sha256
    }

    pub fn domain_sha256(&self) -> [u8; 32] {
        self.domain_sha256
    }

    pub fn server_public_key_sha256(&self) -> [u8; 32] {
        self.server_public_key_sha256
    }

    pub fn unique_count(&self) -> usize {
        self.unique_count
    }

    pub fn contains_tag(&self, tag: &DiscoveryTag) -> bool {
        self.tags.contains(tag)
    }
}

/// Server-side VOPRF key holder. Client raw identifiers are intentionally not
/// accepted by `evaluate_blinded`; the server evaluates only blinded elements.
pub struct SyntheticVoprfServer {
    inner: VoprfServer<Ristretto255>,
}

impl SyntheticVoprfServer {
    /// Deterministic seed construction exists only for reproducible synthetic
    /// evidence. Operational key lifecycle is explicitly out of scope.
    pub fn from_seed(seed: &[u8; 32]) -> Result<Self, DiscoveryError> {
        let inner = VoprfServer::<Ristretto255>::new_from_seed(seed, SERVER_KEY_INFO)
            .map_err(map_voprf)?;
        Ok(Self { inner })
    }

    pub fn public_key(&self) -> VoprfPublicKey {
        self.inner.get_public_key()
    }

    pub fn public_key_sha256(&self) -> [u8; 32] {
        public_key_sha256(self.public_key())
    }

    fn direct_tag(
        &self,
        domain: &DiscoveryDomain,
        identifier: &SyntheticIdentifier,
    ) -> Result<DiscoveryTag, DiscoveryError> {
        let input = protocol_input(domain, identifier);
        let output = self.inner.evaluate(&input).map_err(map_voprf)?;
        Ok(DiscoveryTag(output.to_vec()))
    }

    /// Server-owned registry preparation. Source identifiers are consumed only
    /// to derive tags and are not retained in the resulting registry value.
    pub fn prepare_registry(
        &self,
        domain: &DiscoveryDomain,
        identifiers: &[SyntheticIdentifier],
    ) -> Result<PreparedRegistry, DiscoveryError> {
        let canonical = canonical_set(identifiers)?;
        let mut tags = BTreeSet::new();
        for identifier in &canonical {
            tags.insert(self.direct_tag(domain, identifier)?);
        }
        Ok(PreparedRegistry {
            tag_snapshot_sha256: tag_set_commitment(&tags),
            tags,
            domain_sha256: domain.commitment(),
            server_public_key_sha256: self.public_key_sha256(),
            unique_count: canonical.len(),
        })
    }

    /// Online server phase. This API receives no `SyntheticIdentifier`, raw
    /// protocol input, or client-side VOPRF state.
    pub fn evaluate_blinded<R: RngCore + CryptoRng>(
        &self,
        rng: &mut R,
        request: &BlindedRequest,
    ) -> Result<BlindResponse, DiscoveryError> {
        if request.messages.is_empty() {
            return Err(DiscoveryError::EmptyInputSet);
        }
        if request.messages.len() > MAX_IDENTIFIERS {
            return Err(DiscoveryError::TooManyIdentifiers);
        }
        let observed_commitment = request_commitment(&request.messages);
        if observed_commitment != request.request_sha256 {
            return Err(DiscoveryError::RequestCommitmentMismatch);
        }
        let evaluations = request
            .messages
            .iter()
            .map(|message| self.inner.blind_evaluate(rng, message))
            .collect();
        let public_key = self.public_key();
        Ok(BlindResponse {
            evaluations,
            request_sha256: observed_commitment,
            domain_sha256: request.domain_sha256,
            server_public_key_sha256: public_key_sha256(public_key),
            server_public_key: public_key,
        })
    }
}

/// Server-visible client request. It contains only blinded elements plus
/// non-secret transcript commitments/count metadata.
pub struct BlindedRequest {
    messages: Vec<BlindedElement<Ristretto255>>,
    request_sha256: [u8; 32],
    domain_sha256: [u8; 32],
}

impl BlindedRequest {
    pub fn count(&self) -> usize {
        self.messages.len()
    }

    pub fn request_sha256(&self) -> [u8; 32] {
        self.request_sha256
    }

    pub fn domain_sha256(&self) -> [u8; 32] {
        self.domain_sha256
    }

    pub fn encoded_messages(&self) -> Vec<Vec<u8>> {
        self.messages
            .iter()
            .map(|message| message.serialize().to_vec())
            .collect()
    }
}

struct PendingEntry {
    identifier: SyntheticIdentifier,
    input: Vec<u8>,
    state: VoprfClient<Ristretto255>,
}

/// Client-only pending state. Raw synthetic client identifiers live here and
/// are never embedded into `BlindedRequest` or accepted by server evaluation.
pub struct PendingQuery {
    entries: Vec<PendingEntry>,
    request_sha256: [u8; 32],
    domain_sha256: [u8; 32],
    canonicalization_profile_sha256: [u8; 32],
    client_unique_count: usize,
}

/// Client-side query preparation. Returns local pending state separately from
/// the blinded request that is safe to hand to the VOPRF server.
pub fn prepare_query<R: RngCore + CryptoRng>(
    rng: &mut R,
    domain: &DiscoveryDomain,
    identifiers: &[SyntheticIdentifier],
) -> Result<(PendingQuery, BlindedRequest), DiscoveryError> {
    let canonical = canonical_set(identifiers)?;
    let mut entries = Vec::with_capacity(canonical.len());
    let mut messages = Vec::with_capacity(canonical.len());
    for identifier in canonical {
        let input = protocol_input(domain, &identifier);
        let blind = VoprfClient::<Ristretto255>::blind(&input, rng).map_err(map_voprf)?;
        messages.push(blind.message);
        entries.push(PendingEntry {
            identifier,
            input,
            state: blind.state,
        });
    }
    let request_sha256 = request_commitment(&messages);
    let domain_sha256 = domain.commitment();
    let canonicalization_profile_sha256 = Sha256::digest(CANONICALIZATION_PROFILE.as_bytes()).into();
    let count = entries.len();
    Ok((
        PendingQuery {
            entries,
            request_sha256,
            domain_sha256,
            canonicalization_profile_sha256,
            client_unique_count: count,
        },
        BlindedRequest {
            messages,
            request_sha256,
            domain_sha256,
        },
    ))
}

/// Server response to blinded elements. It contains no raw identifiers.
pub struct BlindResponse {
    evaluations: Vec<VoprfServerEvaluateResult<Ristretto255>>,
    request_sha256: [u8; 32],
    domain_sha256: [u8; 32],
    server_public_key_sha256: [u8; 32],
    server_public_key: VoprfPublicKey,
}

impl BlindResponse {
    pub fn count(&self) -> usize {
        self.evaluations.len()
    }

    pub fn request_sha256(&self) -> [u8; 32] {
        self.request_sha256
    }

    pub fn server_public_key_sha256(&self) -> [u8; 32] {
        self.server_public_key_sha256
    }
}

impl PendingQuery {
    /// Client-side proof verification, VOPRF finalization, and local equality
    /// intersection against the server's pseudorandom registry tags.
    pub fn finalize(
        self,
        response: BlindResponse,
        registry: &PreparedRegistry,
    ) -> Result<DiscoveryOutcome, DiscoveryError> {
        if response.request_sha256 != self.request_sha256 {
            return Err(DiscoveryError::RequestCommitmentMismatch);
        }
        if response.domain_sha256 != self.domain_sha256
            || registry.domain_sha256 != self.domain_sha256
        {
            return Err(DiscoveryError::DomainMismatch);
        }
        if public_key_sha256(response.server_public_key) != response.server_public_key_sha256
            || registry.server_public_key_sha256 != response.server_public_key_sha256
        {
            return Err(DiscoveryError::ServerKeyMismatch);
        }
        if response.evaluations.len() != self.entries.len() {
            return Err(DiscoveryError::ResponseCountMismatch);
        }

        let mut matches = Vec::new();
        for (entry, evaluation) in self.entries.into_iter().zip(response.evaluations) {
            let output = entry
                .state
                .finalize(
                    &entry.input,
                    &evaluation.message,
                    &evaluation.proof,
                    response.server_public_key,
                )
                .map_err(map_voprf)?;
            let tag = DiscoveryTag(output.to_vec());
            if registry.contains_tag(&tag) {
                matches.push(entry.identifier);
            }
        }

        let receipt = ExperimentalReceipt {
            schema: "mycelix.psi.002a.experimental-receipt.v0.2".into(),
            construction: CONSTRUCTION.into(),
            backend: BACKEND.into(),
            backend_version: BACKEND_VERSION.into(),
            backend_tag: BACKEND_TAG.into(),
            backend_source_commit: BACKEND_SOURCE_COMMIT.into(),
            ciphersuite: CIPHERSUITE.into(),
            canonicalization_profile: CANONICALIZATION_PROFILE.into(),
            canonicalization_profile_sha256: self.canonicalization_profile_sha256,
            domain_sha256: self.domain_sha256,
            blinded_request_sha256: self.request_sha256,
            server_public_key_sha256: response.server_public_key_sha256,
            server_tag_snapshot_sha256: registry.tag_snapshot_sha256,
            client_unique_count: self.client_unique_count,
            server_unique_count: registry.unique_count,
            match_count: matches.len(),
            voprf_proof_verification_succeeded: true,
            synthetic_only: true,
            server_request_contains_only_blinded_elements: true,
            raw_client_identifiers_retained_by_registry: false,
            raw_identifier_snapshot_hash_emitted: false,
            offline_enumeration_resistance_established: false,
            online_enumeration_abuse_resistance_established: false,
            client_anonymity_established: false,
            transport_privacy_established: false,
            registry_authenticity_established: false,
            registry_freshness_established: false,
            key_lifecycle_qualified: false,
            wire_format_qualified: false,
            real_data_admitted: false,
            production_admitted: false,
            application_authority_granted: false,
        };
        Ok(DiscoveryOutcome { matches, receipt })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentalReceipt {
    pub schema: String,
    pub construction: String,
    pub backend: String,
    pub backend_version: String,
    pub backend_tag: String,
    pub backend_source_commit: String,
    pub ciphersuite: String,
    pub canonicalization_profile: String,
    pub canonicalization_profile_sha256: [u8; 32],
    pub domain_sha256: [u8; 32],
    pub blinded_request_sha256: [u8; 32],
    pub server_public_key_sha256: [u8; 32],
    pub server_tag_snapshot_sha256: [u8; 32],
    pub client_unique_count: usize,
    pub server_unique_count: usize,
    pub match_count: usize,
    pub voprf_proof_verification_succeeded: bool,
    pub synthetic_only: bool,
    pub server_request_contains_only_blinded_elements: bool,
    pub raw_client_identifiers_retained_by_registry: bool,
    pub raw_identifier_snapshot_hash_emitted: bool,
    pub offline_enumeration_resistance_established: bool,
    pub online_enumeration_abuse_resistance_established: bool,
    pub client_anonymity_established: bool,
    pub transport_privacy_established: bool,
    pub registry_authenticity_established: bool,
    pub registry_freshness_established: bool,
    pub key_lifecycle_qualified: bool,
    pub wire_format_qualified: bool,
    pub real_data_admitted: bool,
    pub production_admitted: bool,
    pub application_authority_granted: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DiscoveryOutcome {
    pub matches: Vec<SyntheticIdentifier>,
    pub receipt: ExperimentalReceipt,
}

pub fn structural_profile(domain: &DiscoveryDomain) -> PsiProfile {
    PsiProfile {
        backend: BackendIdentity {
            primitive: PrivacyPrimitive::PrivateSetOperation,
            backend: BACKEND.into(),
            version: BACKEND_VERSION.into(),
            profile: format!("{CONSTRUCTION}/{CIPHERSUITE}/{}", &BACKEND_SOURCE_COMMIT[..12]),
        },
        participant_model: ParticipantModel::TwoParty,
        adversary_model: AdversaryModel::HonestButCurious,
        interaction_model: InteractionModel::Interactive { rounds: 2 },
        operation: PsiOperation::Intersection,
        collection_semantics: CollectionSemantics::Set,
        output_recipient: PsiOutputRecipient::ClientOnly,
        equality_domain: domain.equality_domain(),
        session_domain: domain.session.clone(),
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
    }
}

/// Deliberately demonstrates the remaining online-enumeration risk while still
/// respecting the client/server message boundary.
pub fn simulate_online_membership_guess<R: RngCore + CryptoRng>(
    client_rng: &mut R,
    server_rng: &mut R,
    server: &SyntheticVoprfServer,
    domain: &DiscoveryDomain,
    guess: &SyntheticIdentifier,
    registry: &PreparedRegistry,
) -> Result<bool, DiscoveryError> {
    let (pending, request) = prepare_query(client_rng, domain, std::slice::from_ref(guess))?;
    let response = server.evaluate_blinded(server_rng, &request)?;
    let outcome = pending.finalize(response, registry)?;
    Ok(!outcome.matches.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_protocol_profiles::ProtocolProfile;
    use rand_chacha::ChaCha20Rng;
    use rand_core::SeedableRng;

    fn domain(session: &str) -> DiscoveryDomain {
        DiscoveryDomain::new("mycelix-test", "contact-discovery-v1", session).unwrap()
    }

    fn id(suffix: &str) -> SyntheticIdentifier {
        SyntheticIdentifier::new(format!("synthetic-contact-{suffix}")).unwrap()
    }

    fn server(seed: u8) -> SyntheticVoprfServer {
        SyntheticVoprfServer::from_seed(&[seed; 32]).unwrap()
    }

    fn rng(byte: u8) -> ChaCha20Rng {
        ChaCha20Rng::from_seed([byte; 32])
    }

    #[test]
    fn rejects_non_synthetic_identifiers() {
        assert_eq!(
            SyntheticIdentifier::new("alice@example.com"),
            Err(DiscoveryError::InvalidSyntheticIdentifier)
        );
        assert_eq!(
            SyntheticIdentifier::new("+15551234567"),
            Err(DiscoveryError::InvalidSyntheticIdentifier)
        );
    }

    #[test]
    fn rejects_empty_or_unsafe_domain_components() {
        assert_eq!(
            DiscoveryDomain::new("", "profile", "session"),
            Err(DiscoveryError::InvalidDomain)
        );
        assert_eq!(
            DiscoveryDomain::new("service with spaces", "profile", "session"),
            Err(DiscoveryError::InvalidDomain)
        );
    }

    #[test]
    fn server_request_contains_only_blinded_message_bytes() {
        let identifier = id("0001");
        let (_, request) = prepare_query(&mut rng(1), &domain("session-a"), std::slice::from_ref(&identifier)).unwrap();
        let raw = identifier.as_str().as_bytes();
        assert!(request
            .encoded_messages()
            .iter()
            .all(|message| !message.windows(raw.len()).any(|window| window == raw)));
    }

    #[test]
    fn client_and_server_roles_produce_exact_intersection() {
        let server = server(7);
        let domain = domain("session-a");
        let registry = server
            .prepare_registry(&domain, &[id("0001"), id("0002"), id("0003")])
            .unwrap();
        let (pending, request) = prepare_query(
            &mut rng(2),
            &domain,
            &[id("0002"), id("0004"), id("0001")],
        )
        .unwrap();
        let response = server.evaluate_blinded(&mut rng(3), &request).unwrap();
        let outcome = pending.finalize(response, &registry).unwrap();
        assert_eq!(outcome.matches, vec![id("0001"), id("0002")]);
        assert_eq!(outcome.receipt.match_count, 2);
        assert!(outcome.receipt.voprf_proof_verification_succeeded);
    }

    #[test]
    fn set_semantics_deduplicate_both_sides() {
        let server = server(7);
        let domain = domain("session-a");
        let registry = server
            .prepare_registry(&domain, &[id("0001"), id("0001"), id("0002")])
            .unwrap();
        assert_eq!(registry.unique_count(), 2);
        let (pending, request) =
            prepare_query(&mut rng(4), &domain, &[id("0001"), id("0001")]).unwrap();
        assert_eq!(request.count(), 1);
        let response = server.evaluate_blinded(&mut rng(5), &request).unwrap();
        let outcome = pending.finalize(response, &registry).unwrap();
        assert_eq!(outcome.matches, vec![id("0001")]);
        assert_eq!(outcome.receipt.client_unique_count, 1);
    }

    #[test]
    fn blinded_request_changes_with_client_randomness() {
        let domain = domain("session-a");
        let identifiers = [id("0001"), id("0002")];
        let (_, a) = prepare_query(&mut rng(10), &domain, &identifiers).unwrap();
        let (_, b) = prepare_query(&mut rng(11), &domain, &identifiers).unwrap();
        assert_ne!(a.request_sha256(), b.request_sha256());
    }

    #[test]
    fn session_domain_separates_registry_tags() {
        let server = server(7);
        let identifier = id("0001");
        let a = server
            .prepare_registry(&domain("session-a"), std::slice::from_ref(&identifier))
            .unwrap();
        let b = server
            .prepare_registry(&domain("session-b"), std::slice::from_ref(&identifier))
            .unwrap();
        assert_ne!(a.tag_snapshot_sha256(), b.tag_snapshot_sha256());
    }

    #[test]
    fn mismatched_domain_fails_closed() {
        let server = server(7);
        let registry = server
            .prepare_registry(&domain("session-b"), &[id("0001")])
            .unwrap();
        let (pending, request) =
            prepare_query(&mut rng(12), &domain("session-a"), &[id("0001")]).unwrap();
        let response = server.evaluate_blinded(&mut rng(13), &request).unwrap();
        assert_eq!(pending.finalize(response, &registry), Err(DiscoveryError::DomainMismatch));
    }

    #[test]
    fn mismatched_server_key_fails_closed() {
        let registry_server = server(7);
        let response_server = server(8);
        let domain = domain("session-a");
        let registry = registry_server.prepare_registry(&domain, &[id("0001")]).unwrap();
        let (pending, request) = prepare_query(&mut rng(14), &domain, &[id("0001")]).unwrap();
        let response = response_server.evaluate_blinded(&mut rng(15), &request).unwrap();
        assert_eq!(pending.finalize(response, &registry), Err(DiscoveryError::ServerKeyMismatch));
    }

    #[test]
    fn registry_snapshot_commits_to_tags_not_raw_identifier_hashes() {
        let server = server(7);
        let domain = domain("session-a");
        let registry = server.prepare_registry(&domain, &[id("0001"), id("0002")]).unwrap();
        let raw_concat_hash: [u8; 32] = Sha256::digest(b"synthetic-contact-0001synthetic-contact-0002").into();
        assert_ne!(registry.tag_snapshot_sha256(), raw_concat_hash);
    }

    #[test]
    fn online_guessing_risk_remains_demonstrable() {
        let server = server(7);
        let domain = domain("session-a");
        let registry = server.prepare_registry(&domain, &[id("0007"), id("0042")]).unwrap();
        assert!(simulate_online_membership_guess(
            &mut rng(20),
            &mut rng(21),
            &server,
            &domain,
            &id("0042"),
            &registry,
        )
        .unwrap());
        assert!(!simulate_online_membership_guess(
            &mut rng(22),
            &mut rng(23),
            &server,
            &domain,
            &id("9999"),
            &registry,
        )
        .unwrap());
    }

    #[test]
    fn receipt_never_promotes_unqualified_properties() {
        let server = server(7);
        let domain = domain("session-a");
        let registry = server.prepare_registry(&domain, &[id("0001")]).unwrap();
        let (pending, request) = prepare_query(&mut rng(30), &domain, &[id("0001")]).unwrap();
        let response = server.evaluate_blinded(&mut rng(31), &request).unwrap();
        let receipt = pending.finalize(response, &registry).unwrap().receipt;
        assert_eq!(receipt.backend_source_commit, BACKEND_SOURCE_COMMIT);
        assert!(receipt.synthetic_only);
        assert!(receipt.server_request_contains_only_blinded_elements);
        assert!(!receipt.raw_client_identifiers_retained_by_registry);
        assert!(!receipt.raw_identifier_snapshot_hash_emitted);
        assert!(!receipt.offline_enumeration_resistance_established);
        assert!(!receipt.online_enumeration_abuse_resistance_established);
        assert!(!receipt.client_anonymity_established);
        assert!(!receipt.transport_privacy_established);
        assert!(!receipt.registry_authenticity_established);
        assert!(!receipt.registry_freshness_established);
        assert!(!receipt.key_lifecycle_qualified);
        assert!(!receipt.wire_format_qualified);
        assert!(!receipt.real_data_admitted);
        assert!(!receipt.production_admitted);
        assert!(!receipt.application_authority_granted);
    }

    #[test]
    fn structural_profile_remains_experimental() {
        let profile = structural_profile(&domain("session-a"));
        assert_eq!(profile.qualification, QualificationState::Experimental);
        assert_eq!(profile.validate(), Ok(()));
        let wrapped = ProtocolProfile::Psi(profile);
        assert!(!wrapped.cryptographic_security_established());
        assert!(!wrapped.production_admission_granted());
        assert!(!wrapped.application_authority_granted());
    }
}