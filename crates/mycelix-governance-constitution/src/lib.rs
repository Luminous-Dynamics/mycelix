// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! DNA-bound constitutional genesis and append-only constitutional lineage.
//!
//! The core rule is deliberately simple:
//!
//! **The first constitution is not whoever writes first.**
//!
//! A Holochain DNA commits immutable properties into the DNA hash. Those
//! properties define the exact semantic genesis constitution. A runtime may
//! persist the matching genesis statement for discoverability, but the writer
//! gains no special authority merely by publishing it.
//!
//! Later constitutional versions form an append-only lineage. Every transition
//! binds the exact parent state, exact child state, exact amendment policy that
//! governed the transition, the binding-vote result, threshold authorization,
//! amendment payload, and a replay nonce.
//!
//! This crate performs no Holochain calls and no signature/proof verification.
//! `TransitionVerificationEvidence` is an adapter boundary: hosts MUST create it
//! only after independently verifying the referenced binding tally and threshold
//! authorization. A deserialized value is evidence-shaped data, not authority by
//! existence.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-constitution-v0.1";
pub const GENESIS_MANIFEST_PROFILE: &str =
    "mycelix-governance-constitution-genesis-v1-blake3-framed";
pub const STATEMENT_PROFILE: &str =
    "mycelix-governance-constitution-statement-v1-blake3-framed";
pub const TRANSITION_PROFILE: &str =
    "mycelix-governance-constitution-transition-v1-blake3-framed";

const MAX_ID_BYTES: usize = 512;
const MAX_PROFILE_BYTES: usize = 128;
const DOMAIN_GENESIS: &[u8] = b"mycelix/governance/constitution/genesis/v1";
const DOMAIN_STATEMENT: &[u8] = b"mycelix/governance/constitution/statement/v1";
const DOMAIN_TRANSITION: &[u8] = b"mycelix/governance/constitution/transition/v1";

#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct Digest32(pub [u8; 32]);

impl Digest32 {
    pub const ZERO: Self = Self([0; 32]);

    pub fn is_zero(&self) -> bool {
        self.0.iter().all(|byte| *byte == 0)
    }

    pub fn to_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(64);
        for byte in self.0 {
            out.push(HEX[(byte >> 4) as usize] as char);
            out.push(HEX[(byte & 0x0f) as usize] as char);
        }
        out
    }
}

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(
            Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
        )]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ConstitutionError> {
                let value = value.into();
                validate_text(&value, $field, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(NetworkId, "network_id");
id_type!(InstitutionId, "institution_id");
id_type!(ConstitutionId, "constitution_id");
id_type!(RulebookId, "rulebook_id");
id_type!(ProposalId, "proposal_id");

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProfiledDigest {
    pub digest: Digest32,
    pub profile: String,
}

impl ProfiledDigest {
    pub fn validate(&self, field: &'static str) -> Result<(), ConstitutionError> {
        require_digest(self.digest, field)?;
        validate_profile(&self.profile, field)
    }
}

/// Immutable network-level constitutional genesis configuration.
///
/// A Holochain adapter should deserialize this from DNA properties. Because DNA
/// properties are integrity modifiers, changing any field creates a different
/// DNA hash/network. There is intentionally no privileged genesis writer field.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstitutionGenesisManifest {
    pub protocol_version: String,
    pub network_id: NetworkId,
    pub institution_id: InstitutionId,
    pub constitution_id: ConstitutionId,
    pub rulebook_id: RulebookId,
    pub rulebook_version: String,
    pub rulebook: ProfiledDigest,
    pub charter: ProfiledDigest,
    pub parameters: ProfiledDigest,
    pub amendment_policy: ProfiledDigest,
    /// Exact binding-vote protocol/profile permitted to ratify amendments.
    pub binding_vote_profile: String,
    /// Exact threshold/institutional authorization profile required after tally.
    pub threshold_authority_profile: String,
    pub effective_from_ms: u64,
}

impl ConstitutionGenesisManifest {
    pub fn validate(&self) -> Result<(), ConstitutionError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.network_id.as_str(), "genesis.network_id", MAX_ID_BYTES)?;
        validate_text(
            self.institution_id.as_str(),
            "genesis.institution_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.constitution_id.as_str(),
            "genesis.constitution_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.rulebook_id.as_str(),
            "genesis.rulebook_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.rulebook_version,
            "genesis.rulebook_version",
            MAX_PROFILE_BYTES,
        )?;
        self.rulebook.validate("genesis.rulebook")?;
        self.charter.validate("genesis.charter")?;
        self.parameters.validate("genesis.parameters")?;
        self.amendment_policy.validate("genesis.amendment_policy")?;
        validate_profile(
            &self.binding_vote_profile,
            "genesis.binding_vote_profile",
        )?;
        validate_profile(
            &self.threshold_authority_profile,
            "genesis.threshold_authority_profile",
        )?;
        if self.effective_from_ms == 0 {
            return Err(ConstitutionError::ZeroTimestamp("genesis.effective_from_ms"));
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ConstitutionError> {
        self.validate()?;
        let mut out = Vec::new();
        put_domain(&mut out, DOMAIN_GENESIS);
        put_str(&mut out, &self.protocol_version);
        put_str(&mut out, self.network_id.as_str());
        put_str(&mut out, self.institution_id.as_str());
        put_str(&mut out, self.constitution_id.as_str());
        put_str(&mut out, self.rulebook_id.as_str());
        put_str(&mut out, &self.rulebook_version);
        put_profiled_digest(&mut out, &self.rulebook);
        put_profiled_digest(&mut out, &self.charter);
        put_profiled_digest(&mut out, &self.parameters);
        put_profiled_digest(&mut out, &self.amendment_policy);
        put_str(&mut out, &self.binding_vote_profile);
        put_str(&mut out, &self.threshold_authority_profile);
        put_u64(&mut out, self.effective_from_ms);
        Ok(out)
    }

    pub fn digest(&self) -> Result<Digest32, ConstitutionError> {
        Ok(hash(&self.canonical_bytes()?))
    }

    pub fn genesis_statement(&self) -> Result<ConstitutionStatement, ConstitutionError> {
        self.validate()?;
        Ok(ConstitutionStatement {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: self.network_id.clone(),
            institution_id: self.institution_id.clone(),
            constitution_id: self.constitution_id.clone(),
            version: 1,
            parent_statement_digest: None,
            rulebook_id: self.rulebook_id.clone(),
            rulebook_version: self.rulebook_version.clone(),
            rulebook: self.rulebook.clone(),
            charter: self.charter.clone(),
            parameters: self.parameters.clone(),
            amendment_policy: self.amendment_policy.clone(),
            binding_vote_profile: self.binding_vote_profile.clone(),
            threshold_authority_profile: self.threshold_authority_profile.clone(),
            effective_from_ms: self.effective_from_ms,
        })
    }

    /// Verify that a persisted genesis statement is exactly the DNA-committed
    /// semantic genesis statement. The publishing author is intentionally absent.
    pub fn verify_genesis_statement(
        &self,
        statement: &ConstitutionStatement,
    ) -> Result<(), ConstitutionError> {
        let expected = self.genesis_statement()?;
        if &expected != statement {
            return Err(ConstitutionError::GenesisStatementMismatch);
        }
        Ok(())
    }
}

/// Semantic constitutional state at one version.
///
/// Authorization receipts are deliberately outside this digest. This avoids a
/// circular hash where a transition authorization commits the child statement
/// while the child statement also commits that authorization.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstitutionStatement {
    pub protocol_version: String,
    pub network_id: NetworkId,
    pub institution_id: InstitutionId,
    pub constitution_id: ConstitutionId,
    pub version: u64,
    pub parent_statement_digest: Option<Digest32>,
    pub rulebook_id: RulebookId,
    pub rulebook_version: String,
    pub rulebook: ProfiledDigest,
    pub charter: ProfiledDigest,
    pub parameters: ProfiledDigest,
    pub amendment_policy: ProfiledDigest,
    pub binding_vote_profile: String,
    pub threshold_authority_profile: String,
    pub effective_from_ms: u64,
}

impl ConstitutionStatement {
    pub fn validate(&self) -> Result<(), ConstitutionError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.network_id.as_str(), "statement.network_id", MAX_ID_BYTES)?;
        validate_text(
            self.institution_id.as_str(),
            "statement.institution_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.constitution_id.as_str(),
            "statement.constitution_id",
            MAX_ID_BYTES,
        )?;
        if self.version == 0 {
            return Err(ConstitutionError::ZeroVersion);
        }
        match (self.version, self.parent_statement_digest) {
            (1, None) => {}
            (1, Some(_)) => return Err(ConstitutionError::GenesisHasParent),
            (_, None) => return Err(ConstitutionError::NonGenesisMissingParent),
            (_, Some(parent)) => require_digest(parent, "statement.parent_statement_digest")?,
        }
        validate_text(
            self.rulebook_id.as_str(),
            "statement.rulebook_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.rulebook_version,
            "statement.rulebook_version",
            MAX_PROFILE_BYTES,
        )?;
        self.rulebook.validate("statement.rulebook")?;
        self.charter.validate("statement.charter")?;
        self.parameters.validate("statement.parameters")?;
        self.amendment_policy.validate("statement.amendment_policy")?;
        validate_profile(
            &self.binding_vote_profile,
            "statement.binding_vote_profile",
        )?;
        validate_profile(
            &self.threshold_authority_profile,
            "statement.threshold_authority_profile",
        )?;
        if self.effective_from_ms == 0 {
            return Err(ConstitutionError::ZeroTimestamp(
                "statement.effective_from_ms",
            ));
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ConstitutionError> {
        self.validate()?;
        let mut out = Vec::new();
        put_domain(&mut out, DOMAIN_STATEMENT);
        put_str(&mut out, &self.protocol_version);
        put_str(&mut out, self.network_id.as_str());
        put_str(&mut out, self.institution_id.as_str());
        put_str(&mut out, self.constitution_id.as_str());
        put_u64(&mut out, self.version);
        put_optional_digest(&mut out, self.parent_statement_digest);
        put_str(&mut out, self.rulebook_id.as_str());
        put_str(&mut out, &self.rulebook_version);
        put_profiled_digest(&mut out, &self.rulebook);
        put_profiled_digest(&mut out, &self.charter);
        put_profiled_digest(&mut out, &self.parameters);
        put_profiled_digest(&mut out, &self.amendment_policy);
        put_str(&mut out, &self.binding_vote_profile);
        put_str(&mut out, &self.threshold_authority_profile);
        put_u64(&mut out, self.effective_from_ms);
        Ok(out)
    }

    pub fn digest(&self) -> Result<Digest32, ConstitutionError> {
        Ok(hash(&self.canonical_bytes()?))
    }
}

/// Exact authorization statement for one parent -> child constitutional change.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstitutionTransitionAuthorization {
    pub protocol_version: String,
    pub network_id: NetworkId,
    pub institution_id: InstitutionId,
    pub constitution_id: ConstitutionId,
    pub from_statement_digest: Digest32,
    pub to_statement_digest: Digest32,
    /// Amendment policy in force on the parent statement.
    pub amendment_policy_digest: Digest32,
    pub amendment_policy_profile: String,
    pub proposal_id: ProposalId,
    pub binding_tally: ProfiledDigest,
    pub threshold_authorization: ProfiledDigest,
    pub amendment_payload: ProfiledDigest,
    pub authorized_at_ms: u64,
    /// Non-zero replay-domain value supplied by the authorization layer.
    pub transition_nonce: Digest32,
}

impl ConstitutionTransitionAuthorization {
    pub fn validate(&self) -> Result<(), ConstitutionError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.network_id.as_str(), "transition.network_id", MAX_ID_BYTES)?;
        validate_text(
            self.institution_id.as_str(),
            "transition.institution_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.constitution_id.as_str(),
            "transition.constitution_id",
            MAX_ID_BYTES,
        )?;
        require_digest(
            self.from_statement_digest,
            "transition.from_statement_digest",
        )?;
        require_digest(self.to_statement_digest, "transition.to_statement_digest")?;
        require_digest(
            self.amendment_policy_digest,
            "transition.amendment_policy_digest",
        )?;
        validate_profile(
            &self.amendment_policy_profile,
            "transition.amendment_policy_profile",
        )?;
        validate_text(self.proposal_id.as_str(), "transition.proposal_id", MAX_ID_BYTES)?;
        self.binding_tally.validate("transition.binding_tally")?;
        self.threshold_authorization
            .validate("transition.threshold_authorization")?;
        self.amendment_payload
            .validate("transition.amendment_payload")?;
        if self.authorized_at_ms == 0 {
            return Err(ConstitutionError::ZeroTimestamp(
                "transition.authorized_at_ms",
            ));
        }
        require_digest(self.transition_nonce, "transition.transition_nonce")
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ConstitutionError> {
        self.validate()?;
        let mut out = Vec::new();
        put_domain(&mut out, DOMAIN_TRANSITION);
        put_str(&mut out, &self.protocol_version);
        put_str(&mut out, self.network_id.as_str());
        put_str(&mut out, self.institution_id.as_str());
        put_str(&mut out, self.constitution_id.as_str());
        put_digest(&mut out, self.from_statement_digest);
        put_digest(&mut out, self.to_statement_digest);
        put_digest(&mut out, self.amendment_policy_digest);
        put_str(&mut out, &self.amendment_policy_profile);
        put_str(&mut out, self.proposal_id.as_str());
        put_profiled_digest(&mut out, &self.binding_tally);
        put_profiled_digest(&mut out, &self.threshold_authorization);
        put_profiled_digest(&mut out, &self.amendment_payload);
        put_u64(&mut out, self.authorized_at_ms);
        put_digest(&mut out, self.transition_nonce);
        Ok(out)
    }

    pub fn digest(&self) -> Result<Digest32, ConstitutionError> {
        Ok(hash(&self.canonical_bytes()?))
    }
}

/// Adapter evidence that the exact transition authorization was externally
/// verified. The proof references are intentionally opaque to this pure crate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransitionVerificationEvidence {
    pub authorization_digest: Digest32,
    pub binding_tally_verification_ref: String,
    pub threshold_verification_ref: String,
    pub verified_at_ms: u64,
}

impl TransitionVerificationEvidence {
    pub fn validate_against(
        &self,
        authorization: &ConstitutionTransitionAuthorization,
    ) -> Result<(), ConstitutionError> {
        let expected = authorization.digest()?;
        if self.authorization_digest != expected {
            return Err(ConstitutionError::VerificationAuthorizationMismatch);
        }
        validate_text(
            &self.binding_tally_verification_ref,
            "verification.binding_tally_verification_ref",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.threshold_verification_ref,
            "verification.threshold_verification_ref",
            MAX_ID_BYTES,
        )?;
        if self.verified_at_ms < authorization.authorized_at_ms {
            return Err(ConstitutionError::VerificationPredatesAuthorization);
        }
        Ok(())
    }
}

/// One candidate child state plus the exact transition that authorized it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedConstitutionTransition {
    pub child: ConstitutionStatement,
    pub authorization: ConstitutionTransitionAuthorization,
    pub verification: TransitionVerificationEvidence,
}

impl VerifiedConstitutionTransition {
    pub fn validate_from(
        &self,
        parent: &ConstitutionStatement,
    ) -> Result<(), ConstitutionError> {
        validate_transition(
            parent,
            &self.child,
            &self.authorization,
            &self.verification,
        )
    }
}

/// Verify all structural and exact-binding invariants for one constitutional
/// transition. This still does not perform the cryptographic verification named
/// by `verification`.
pub fn validate_transition(
    parent: &ConstitutionStatement,
    child: &ConstitutionStatement,
    authorization: &ConstitutionTransitionAuthorization,
    verification: &TransitionVerificationEvidence,
) -> Result<(), ConstitutionError> {
    parent.validate()?;
    child.validate()?;
    authorization.validate()?;
    verification.validate_against(authorization)?;

    if child.version != parent.version.saturating_add(1) {
        return Err(ConstitutionError::NonSequentialVersion);
    }
    let parent_digest = parent.digest()?;
    let child_digest = child.digest()?;
    if child.parent_statement_digest != Some(parent_digest) {
        return Err(ConstitutionError::ChildParentMismatch);
    }
    if authorization.from_statement_digest != parent_digest {
        return Err(ConstitutionError::AuthorizationParentMismatch);
    }
    if authorization.to_statement_digest != child_digest {
        return Err(ConstitutionError::AuthorizationChildMismatch);
    }
    if parent.network_id != child.network_id
        || parent.network_id != authorization.network_id
    {
        return Err(ConstitutionError::NetworkMismatch);
    }
    if parent.institution_id != child.institution_id
        || parent.institution_id != authorization.institution_id
    {
        return Err(ConstitutionError::InstitutionMismatch);
    }
    if parent.constitution_id != child.constitution_id
        || parent.constitution_id != authorization.constitution_id
    {
        return Err(ConstitutionError::ConstitutionMismatch);
    }
    if authorization.amendment_policy_digest != parent.amendment_policy.digest
        || authorization.amendment_policy_profile != parent.amendment_policy.profile
    {
        return Err(ConstitutionError::AmendmentPolicyMismatch);
    }
    if authorization.binding_tally.profile != parent.binding_vote_profile {
        return Err(ConstitutionError::BindingVoteProfileMismatch);
    }
    if authorization.threshold_authorization.profile != parent.threshold_authority_profile {
        return Err(ConstitutionError::ThresholdAuthorityProfileMismatch);
    }
    if authorization.authorized_at_ms < parent.effective_from_ms {
        return Err(ConstitutionError::AuthorizationPredatesParent);
    }
    if child.effective_from_ms < authorization.authorized_at_ms {
        return Err(ConstitutionError::ChildEffectiveBeforeAuthorization);
    }
    Ok(())
}

/// Deterministically project one constitutional lineage from the DNA-committed
/// genesis state and a set of host-verified transitions.
///
/// Duplicate byte-identical transitions are harmless. Two distinct valid child
/// statements for the same current parent are treated as constitutional
/// equivocation and fail closed; timestamp ordering never chooses a winner.
pub fn project_verified_lineage(
    manifest: &ConstitutionGenesisManifest,
    transitions: &[VerifiedConstitutionTransition],
) -> Result<ConstitutionStatement, ConstitutionError> {
    let mut current = manifest.genesis_statement()?;
    let mut consumed = BTreeSet::new();

    loop {
        let current_digest = current.digest()?;
        let mut by_child: BTreeMap<Digest32, &VerifiedConstitutionTransition> = BTreeMap::new();

        for (index, transition) in transitions.iter().enumerate() {
            if consumed.contains(&index) {
                continue;
            }
            if transition.authorization.from_statement_digest != current_digest {
                continue;
            }
            transition.validate_from(&current)?;
            let child_digest = transition.child.digest()?;
            if let Some(existing) = by_child.get(&child_digest) {
                if *existing != transition {
                    return Err(ConstitutionError::DuplicateChildConflictingEvidence);
                }
            } else {
                by_child.insert(child_digest, transition);
            }
        }

        if by_child.is_empty() {
            return Ok(current);
        }
        if by_child.len() > 1 {
            return Err(ConstitutionError::AmbiguousConstitutionalFork);
        }

        let (_, selected) = by_child.into_iter().next().expect("checked non-empty");
        current = selected.child.clone();

        for (index, transition) in transitions.iter().enumerate() {
            if transition.child == current && transition.authorization.from_statement_digest == current_digest {
                consumed.insert(index);
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConstitutionError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidProfile(&'static str),
    ZeroDigest(&'static str),
    ZeroTimestamp(&'static str),
    ZeroVersion,
    GenesisHasParent,
    NonGenesisMissingParent,
    GenesisStatementMismatch,
    VerificationAuthorizationMismatch,
    VerificationPredatesAuthorization,
    NonSequentialVersion,
    ChildParentMismatch,
    AuthorizationParentMismatch,
    AuthorizationChildMismatch,
    NetworkMismatch,
    InstitutionMismatch,
    ConstitutionMismatch,
    AmendmentPolicyMismatch,
    BindingVoteProfileMismatch,
    ThresholdAuthorityProfileMismatch,
    AuthorizationPredatesParent,
    ChildEffectiveBeforeAuthorization,
    DuplicateChildConflictingEvidence,
    AmbiguousConstitutionalFork,
}

impl fmt::Display for ConstitutionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong constitutional protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProfile(field) => write!(f, "{field} is not a canonical profile token"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::ZeroTimestamp(field) => write!(f, "{field} must be non-zero"),
            Self::ZeroVersion => write!(f, "constitution version must be non-zero"),
            Self::GenesisHasParent => write!(f, "genesis constitution must not have a parent"),
            Self::NonGenesisMissingParent => write!(f, "non-genesis constitution requires a parent"),
            Self::GenesisStatementMismatch => write!(f, "persisted genesis differs from DNA-bound genesis"),
            Self::VerificationAuthorizationMismatch => write!(f, "verification evidence targets another transition authorization"),
            Self::VerificationPredatesAuthorization => write!(f, "verification predates transition authorization"),
            Self::NonSequentialVersion => write!(f, "constitutional versions must increment by exactly one"),
            Self::ChildParentMismatch => write!(f, "child constitution does not bind the exact parent statement"),
            Self::AuthorizationParentMismatch => write!(f, "transition authorization targets another parent"),
            Self::AuthorizationChildMismatch => write!(f, "transition authorization targets another child"),
            Self::NetworkMismatch => write!(f, "constitutional transition crosses network identity"),
            Self::InstitutionMismatch => write!(f, "constitutional transition crosses institution identity"),
            Self::ConstitutionMismatch => write!(f, "constitutional transition crosses constitution identity"),
            Self::AmendmentPolicyMismatch => write!(f, "transition was not governed by the parent's exact amendment policy"),
            Self::BindingVoteProfileMismatch => write!(f, "transition uses a binding-vote profile not authorized by the parent"),
            Self::ThresholdAuthorityProfileMismatch => write!(f, "transition uses an authority profile not authorized by the parent"),
            Self::AuthorizationPredatesParent => write!(f, "transition authorization predates parent effectiveness"),
            Self::ChildEffectiveBeforeAuthorization => write!(f, "child constitution becomes effective before authorization"),
            Self::DuplicateChildConflictingEvidence => write!(f, "same constitutional child has conflicting authorization evidence"),
            Self::AmbiguousConstitutionalFork => write!(f, "multiple verified constitutional children exist for one parent"),
        }
    }
}

impl std::error::Error for ConstitutionError {}

fn require_protocol(value: &str) -> Result<(), ConstitutionError> {
    if value == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ConstitutionError::WrongProtocolVersion)
    }
}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), ConstitutionError> {
    if value.trim().is_empty() {
        return Err(ConstitutionError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(ConstitutionError::TooLong(field));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &'static str) -> Result<(), ConstitutionError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(ConstitutionError::InvalidProfile(field));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), ConstitutionError> {
    if digest.is_zero() {
        Err(ConstitutionError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

fn hash(bytes: &[u8]) -> Digest32 {
    Digest32(*blake3::hash(bytes).as_bytes())
}

fn put_domain(out: &mut Vec<u8>, domain: &[u8]) {
    put_bytes(out, domain);
}

fn put_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_bytes(out: &mut Vec<u8>, bytes: &[u8]) {
    let len = u64::try_from(bytes.len()).expect("usize fits into u64 on supported targets");
    put_u64(out, len);
    out.extend_from_slice(bytes);
}

fn put_str(out: &mut Vec<u8>, value: &str) {
    put_bytes(out, value.as_bytes());
}

fn put_digest(out: &mut Vec<u8>, digest: Digest32) {
    out.extend_from_slice(&digest.0);
}

fn put_optional_digest(out: &mut Vec<u8>, digest: Option<Digest32>) {
    match digest {
        None => out.push(0),
        Some(value) => {
            out.push(1);
            put_digest(out, value);
        }
    }
}

fn put_profiled_digest(out: &mut Vec<u8>, digest: &ProfiledDigest) {
    put_str(out, &digest.profile);
    put_digest(out, digest.digest);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn pd(byte: u8, profile: &str) -> ProfiledDigest {
        ProfiledDigest {
            digest: digest(byte),
            profile: profile.into(),
        }
    }

    fn manifest() -> ConstitutionGenesisManifest {
        ConstitutionGenesisManifest {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: NetworkId::new("network:mycelix:test").unwrap(),
            institution_id: InstitutionId::new("institution:mycelix:test").unwrap(),
            constitution_id: ConstitutionId::new("constitution:mycelix:test").unwrap(),
            rulebook_id: RulebookId::new("rulebook:constitution:v1").unwrap(),
            rulebook_version: "1".into(),
            rulebook: pd(1, "mycelix-rulebook-v1"),
            charter: pd(2, "mycelix-charter-v1"),
            parameters: pd(3, "mycelix-parameters-v1"),
            amendment_policy: pd(4, "mycelix-amendment-policy-v1"),
            binding_vote_profile: "mycelix-binding-vote-v2".into(),
            threshold_authority_profile: "mycelix-threshold-authority-v1".into(),
            effective_from_ms: 1_000,
        }
    }

    fn child(parent: &ConstitutionStatement, charter_byte: u8) -> ConstitutionStatement {
        ConstitutionStatement {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: parent.network_id.clone(),
            institution_id: parent.institution_id.clone(),
            constitution_id: parent.constitution_id.clone(),
            version: parent.version + 1,
            parent_statement_digest: Some(parent.digest().unwrap()),
            rulebook_id: parent.rulebook_id.clone(),
            rulebook_version: parent.rulebook_version.clone(),
            rulebook: parent.rulebook.clone(),
            charter: pd(charter_byte, "mycelix-charter-v1"),
            parameters: parent.parameters.clone(),
            amendment_policy: parent.amendment_policy.clone(),
            binding_vote_profile: parent.binding_vote_profile.clone(),
            threshold_authority_profile: parent.threshold_authority_profile.clone(),
            effective_from_ms: parent.effective_from_ms + 1_000,
        }
    }

    fn transition(
        parent: &ConstitutionStatement,
        child: &ConstitutionStatement,
        nonce_byte: u8,
    ) -> VerifiedConstitutionTransition {
        let authorization = ConstitutionTransitionAuthorization {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: parent.network_id.clone(),
            institution_id: parent.institution_id.clone(),
            constitution_id: parent.constitution_id.clone(),
            from_statement_digest: parent.digest().unwrap(),
            to_statement_digest: child.digest().unwrap(),
            amendment_policy_digest: parent.amendment_policy.digest,
            amendment_policy_profile: parent.amendment_policy.profile.clone(),
            proposal_id: ProposalId::new(format!("MIP-{}", child.version)).unwrap(),
            binding_tally: pd(20 + nonce_byte, &parent.binding_vote_profile),
            threshold_authorization: pd(
                40 + nonce_byte,
                &parent.threshold_authority_profile,
            ),
            amendment_payload: pd(60 + nonce_byte, "mycelix-amendment-payload-v1"),
            authorized_at_ms: child.effective_from_ms - 100,
            transition_nonce: digest(80 + nonce_byte),
        };
        let verification = TransitionVerificationEvidence {
            authorization_digest: authorization.digest().unwrap(),
            binding_tally_verification_ref: format!("verify:tally:{nonce_byte}"),
            threshold_verification_ref: format!("verify:threshold:{nonce_byte}"),
            verified_at_ms: authorization.authorized_at_ms + 1,
        };
        VerifiedConstitutionTransition {
            child: child.clone(),
            authorization,
            verification,
        }
    }

    #[test]
    fn dna_genesis_is_exact_not_first_writer() {
        let manifest = manifest();
        let statement = manifest.genesis_statement().unwrap();
        manifest.verify_genesis_statement(&statement).unwrap();

        let mut forged = statement;
        forged.charter = pd(99, "mycelix-charter-v1");
        assert_eq!(
            manifest.verify_genesis_statement(&forged).unwrap_err(),
            ConstitutionError::GenesisStatementMismatch
        );
    }

    #[test]
    fn changing_any_genesis_commitment_changes_manifest_digest() {
        let a = manifest();
        let mut b = a.clone();
        b.parameters = pd(44, "mycelix-parameters-v1");
        assert_ne!(a.digest().unwrap(), b.digest().unwrap());
    }

    #[test]
    fn transition_binds_exact_parent_child_and_policy() {
        let parent = manifest().genesis_statement().unwrap();
        let child = child(&parent, 9);
        transition(&parent, &child, 1).validate_from(&parent).unwrap();

        let mut wrong = transition(&parent, &child, 1);
        wrong.authorization.amendment_policy_digest = digest(99);
        wrong.verification.authorization_digest = wrong.authorization.digest().unwrap();
        assert_eq!(
            wrong.validate_from(&parent).unwrap_err(),
            ConstitutionError::AmendmentPolicyMismatch
        );
    }

    #[test]
    fn skipped_version_is_rejected() {
        let parent = manifest().genesis_statement().unwrap();
        let mut child = child(&parent, 9);
        child.version = 3;
        let t = transition(&parent, &child, 2);
        assert_eq!(
            t.validate_from(&parent).unwrap_err(),
            ConstitutionError::NonSequentialVersion
        );
    }

    #[test]
    fn wrong_binding_vote_profile_is_rejected() {
        let parent = manifest().genesis_statement().unwrap();
        let child = child(&parent, 9);
        let mut t = transition(&parent, &child, 3);
        t.authorization.binding_tally.profile = "legacy-phi-weighted-vote".into();
        t.verification.authorization_digest = t.authorization.digest().unwrap();
        assert_eq!(
            t.validate_from(&parent).unwrap_err(),
            ConstitutionError::BindingVoteProfileMismatch
        );
    }

    #[test]
    fn zero_transition_nonce_is_rejected() {
        let parent = manifest().genesis_statement().unwrap();
        let child = child(&parent, 9);
        let mut t = transition(&parent, &child, 4);
        t.authorization.transition_nonce = Digest32::ZERO;
        assert!(matches!(
            t.validate_from(&parent),
            Err(ConstitutionError::ZeroDigest("transition.transition_nonce"))
        ));
    }

    #[test]
    fn verified_lineage_projects_deterministically() {
        let manifest = manifest();
        let v1 = manifest.genesis_statement().unwrap();
        let v2 = child(&v1, 10);
        let v3 = child(&v2, 11);
        let t2 = transition(&v1, &v2, 5);
        let t3 = transition(&v2, &v3, 6);

        let projected_a = project_verified_lineage(&manifest, &[t2.clone(), t3.clone()]).unwrap();
        let projected_b = project_verified_lineage(&manifest, &[t3, t2]).unwrap();
        assert_eq!(projected_a, v3);
        assert_eq!(projected_a, projected_b);
    }

    #[test]
    fn verified_fork_fails_closed_instead_of_using_timestamp_order() {
        let manifest = manifest();
        let v1 = manifest.genesis_statement().unwrap();
        let a = child(&v1, 10);
        let b = child(&v1, 11);
        let ta = transition(&v1, &a, 7);
        let tb = transition(&v1, &b, 8);

        assert_eq!(
            project_verified_lineage(&manifest, &[ta, tb]).unwrap_err(),
            ConstitutionError::AmbiguousConstitutionalFork
        );
    }

    #[test]
    fn authorization_evidence_cannot_be_replayed_for_another_child() {
        let parent = manifest().genesis_statement().unwrap();
        let child_a = child(&parent, 10);
        let child_b = child(&parent, 11);
        let a = transition(&parent, &child_a, 9);
        let mut b = transition(&parent, &child_b, 10);
        b.verification = a.verification;

        assert_eq!(
            b.validate_from(&parent).unwrap_err(),
            ConstitutionError::VerificationAuthorizationMismatch
        );
    }

    #[test]
    fn serde_round_trip_preserves_canonical_digest() {
        let manifest = manifest();
        let json = serde_json::to_string(&manifest).unwrap();
        let decoded: ConstitutionGenesisManifest = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, manifest);
        assert_eq!(decoded.digest().unwrap(), manifest.digest().unwrap());
    }
}
