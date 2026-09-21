// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Role-safe identity primitives for the Mycelix epistemic evidence substrate.
//!
//! EPI-001 establishes identity framing only. It does not establish truth,
//! source authenticity, evidence validity, semantic entailment, independence,
//! scientific qualification, currentness, or application authority.

use serde::{de, Deserialize, Deserializer, Serialize, Serializer};
use sha2::{Digest, Sha256};
use std::{fmt, marker::PhantomData};

pub const EVIDENCE_IDENTITY_PROFILE_V1: &str = "mycelix:epistemic-identity:v1";
const MAX_NAMESPACE_LEN: usize = 64;
const MAX_LOCAL_ID_LEN: usize = 192;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceAuthorityScopeV1 {
    IdentityOnly,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvidenceIdentityError {
    EmptyNamespace,
    EmptyLocalId,
    NamespaceTooLong,
    LocalIdTooLong,
    InvalidNamespaceCharacter { index: usize, byte: u8 },
    InvalidLocalIdCharacter { index: usize, byte: u8 },
    NonCanonicalWire,
    WrongProfile,
    WrongRole,
    InvalidDigestHex,
}

impl fmt::Display for EvidenceIdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyNamespace => write!(f, "namespace is empty"),
            Self::EmptyLocalId => write!(f, "local id is empty"),
            Self::NamespaceTooLong => write!(f, "namespace exceeds {MAX_NAMESPACE_LEN} bytes"),
            Self::LocalIdTooLong => write!(f, "local id exceeds {MAX_LOCAL_ID_LEN} bytes"),
            Self::InvalidNamespaceCharacter { index, byte } => write!(
                f,
                "invalid namespace character at byte {index}: 0x{byte:02x}"
            ),
            Self::InvalidLocalIdCharacter { index, byte } => {
                write!(f, "invalid local-id character at byte {index}: 0x{byte:02x}")
            }
            Self::NonCanonicalWire => write!(f, "non-canonical evidence identity wire form"),
            Self::WrongProfile => write!(f, "wrong evidence identity profile"),
            Self::WrongRole => write!(f, "wrong evidence identity role"),
            Self::InvalidDigestHex => write!(f, "invalid evidence identity digest hex"),
        }
    }
}

impl std::error::Error for EvidenceIdentityError {}

pub trait EvidenceRole: private::Sealed + Copy + Clone + fmt::Debug + Eq + 'static {
    const TAG: &'static str;
}

macro_rules! define_role {
    ($name:ident, $tag:literal) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name;
        impl private::Sealed for $name {}
        impl EvidenceRole for $name {
            const TAG: &'static str = $tag;
        }
    };
}

define_role!(ArtifactRole, "artifact");
define_role!(ObservationRole, "observation");
define_role!(AssertionRole, "assertion");
define_role!(ClaimRole, "claim");
define_role!(HypothesisRole, "hypothesis");
define_role!(SourceRole, "source");
define_role!(DerivationRole, "derivation");
define_role!(EvidenceRelationRole, "evidence-relation");
define_role!(AssessmentRole, "assessment");

#[derive(Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EvidenceId<R: EvidenceRole> {
    namespace: String,
    local_id: String,
    _role: PhantomData<R>,
}

impl<R: EvidenceRole> fmt::Debug for EvidenceId<R> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EvidenceId")
            .field("role", &R::TAG)
            .field("namespace", &self.namespace)
            .field("local_id", &self.local_id)
            .finish()
    }
}

impl<R: EvidenceRole> EvidenceId<R> {
    pub fn new(
        namespace: impl Into<String>,
        local_id: impl Into<String>,
    ) -> Result<Self, EvidenceIdentityError> {
        let namespace = namespace.into();
        let local_id = local_id.into();
        validate_namespace(&namespace)?;
        validate_local_id(&local_id)?;
        Ok(Self {
            namespace,
            local_id,
            _role: PhantomData,
        })
    }

    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    pub fn local_id(&self) -> &str {
        &self.local_id
    }

    pub const fn role_tag() -> &'static str {
        R::TAG
    }

    pub const fn authority_scope(&self) -> EvidenceAuthorityScopeV1 {
        EvidenceAuthorityScopeV1::IdentityOnly
    }

    pub const fn establishes_truth(&self) -> bool {
        false
    }

    pub const fn establishes_source_authenticity(&self) -> bool {
        false
    }

    pub const fn establishes_evidence_validity(&self) -> bool {
        false
    }

    pub const fn establishes_independence(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(
            EVIDENCE_IDENTITY_PROFILE_V1.len()
                + R::TAG.len()
                + self.namespace.len()
                + self.local_id.len()
                + 32,
        );
        append_field(&mut out, b"profile", EVIDENCE_IDENTITY_PROFILE_V1.as_bytes());
        append_field(&mut out, b"role", R::TAG.as_bytes());
        append_field(&mut out, b"namespace", self.namespace.as_bytes());
        append_field(&mut out, b"local-id", self.local_id.as_bytes());
        out
    }

    pub fn commitment_sha256(&self) -> [u8; 32] {
        Sha256::digest(self.canonical_bytes()).into()
    }

    pub fn commitment_sha256_hex(&self) -> String {
        to_hex(&self.commitment_sha256())
    }

    pub fn to_wire(&self) -> EvidenceIdWireV1 {
        EvidenceIdWireV1 {
            profile: EVIDENCE_IDENTITY_PROFILE_V1.to_string(),
            role: R::TAG.to_string(),
            namespace: self.namespace.clone(),
            local_id: self.local_id.clone(),
            commitment_sha256_hex: self.commitment_sha256_hex(),
        }
    }

    pub fn from_wire(wire: EvidenceIdWireV1) -> Result<Self, EvidenceIdentityError> {
        if wire.profile != EVIDENCE_IDENTITY_PROFILE_V1 {
            return Err(EvidenceIdentityError::WrongProfile);
        }
        if wire.role != R::TAG {
            return Err(EvidenceIdentityError::WrongRole);
        }
        let id = Self::new(wire.namespace, wire.local_id)?;
        if !is_lower_hex_64(&wire.commitment_sha256_hex) {
            return Err(EvidenceIdentityError::InvalidDigestHex);
        }
        if wire.commitment_sha256_hex != id.commitment_sha256_hex() {
            return Err(EvidenceIdentityError::NonCanonicalWire);
        }
        Ok(id)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceIdWireV1 {
    pub profile: String,
    pub role: String,
    pub namespace: String,
    pub local_id: String,
    pub commitment_sha256_hex: String,
}

impl<R: EvidenceRole> Serialize for EvidenceId<R> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_wire().serialize(serializer)
    }
}

impl<'de, R: EvidenceRole> Deserialize<'de> for EvidenceId<R> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = EvidenceIdWireV1::deserialize(deserializer)?;
        Self::from_wire(wire).map_err(de::Error::custom)
    }
}

pub type ArtifactId = EvidenceId<ArtifactRole>;
pub type ObservationId = EvidenceId<ObservationRole>;
pub type AssertionId = EvidenceId<AssertionRole>;
pub type ClaimId = EvidenceId<ClaimRole>;
pub type HypothesisId = EvidenceId<HypothesisRole>;
pub type SourceId = EvidenceId<SourceRole>;
pub type DerivationId = EvidenceId<DerivationRole>;
pub type EvidenceRelationId = EvidenceId<EvidenceRelationRole>;
pub type AssessmentId = EvidenceId<AssessmentRole>;

fn validate_namespace(value: &str) -> Result<(), EvidenceIdentityError> {
    if value.is_empty() {
        return Err(EvidenceIdentityError::EmptyNamespace);
    }
    if value.len() > MAX_NAMESPACE_LEN {
        return Err(EvidenceIdentityError::NamespaceTooLong);
    }
    for (index, byte) in value.bytes().enumerate() {
        let ok = byte.is_ascii_lowercase()
            || byte.is_ascii_digit()
            || matches!(byte, b'-' | b'.' | b'_' | b'/');
        if !ok {
            return Err(EvidenceIdentityError::InvalidNamespaceCharacter { index, byte });
        }
    }
    Ok(())
}

fn validate_local_id(value: &str) -> Result<(), EvidenceIdentityError> {
    if value.is_empty() {
        return Err(EvidenceIdentityError::EmptyLocalId);
    }
    if value.len() > MAX_LOCAL_ID_LEN {
        return Err(EvidenceIdentityError::LocalIdTooLong);
    }
    for (index, byte) in value.bytes().enumerate() {
        let ok = byte.is_ascii_alphanumeric()
            || matches!(byte, b'-' | b'.' | b'_' | b':' | b'/' | b'@' | b'+');
        if !ok {
            return Err(EvidenceIdentityError::InvalidLocalIdCharacter { index, byte });
        }
    }
    Ok(())
}

fn append_field(out: &mut Vec<u8>, name: &[u8], value: &[u8]) {
    out.extend_from_slice(&(name.len() as u16).to_be_bytes());
    out.extend_from_slice(name);
    out.extend_from_slice(&(value.len() as u32).to_be_bytes());
    out.extend_from_slice(value);
}

fn to_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        out.push(HEX[(b >> 4) as usize] as char);
        out.push(HEX[(b & 0x0f) as usize] as char);
    }
    out
}

fn is_lower_hex_64(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_digit() || matches!(b, b'a'..=b'f'))
}

mod private {
    pub trait Sealed {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn role_safe_ids_do_not_share_wire_role() {
        let artifact = ArtifactId::new("web", "example.org/report:v1").unwrap();
        let claim = ClaimId::new("web", "example.org/report:v1").unwrap();
        assert_eq!(artifact.namespace(), claim.namespace());
        assert_eq!(artifact.local_id(), claim.local_id());
        assert_ne!(ArtifactRole::TAG, ClaimRole::TAG);
        assert_ne!(artifact.canonical_bytes(), claim.canonical_bytes());
        assert_ne!(artifact.commitment_sha256(), claim.commitment_sha256());
    }

    #[test]
    fn invalid_identity_shapes_fail_closed() {
        assert_eq!(
            ArtifactId::new("", "x").unwrap_err(),
            EvidenceIdentityError::EmptyNamespace
        );
        assert!(matches!(
            ArtifactId::new("Web", "x").unwrap_err(),
            EvidenceIdentityError::InvalidNamespaceCharacter { .. }
        ));
        assert!(matches!(
            ArtifactId::new("web", "contains whitespace").unwrap_err(),
            EvidenceIdentityError::InvalidLocalIdCharacter { .. }
        ));
        assert!(ArtifactId::new("web", "é").is_err());
    }

    #[test]
    fn wire_round_trip_revalidates_profile_role_and_commitment() {
        let id = AssertionId::new("research/web", "assertion:0001").unwrap();
        let wire = id.to_wire();
        assert_eq!(AssertionId::from_wire(wire.clone()).unwrap(), id);

        let mut wrong_role = wire.clone();
        wrong_role.role = ClaimRole::TAG.to_string();
        assert_eq!(
            AssertionId::from_wire(wrong_role).unwrap_err(),
            EvidenceIdentityError::WrongRole
        );

        let mut wrong_profile = wire.clone();
        wrong_profile.profile = "mycelix:epistemic-identity:v2".to_string();
        assert_eq!(
            AssertionId::from_wire(wrong_profile).unwrap_err(),
            EvidenceIdentityError::WrongProfile
        );

        let mut tampered = wire;
        tampered.local_id = "assertion:0002".to_string();
        assert_eq!(
            AssertionId::from_wire(tampered).unwrap_err(),
            EvidenceIdentityError::NonCanonicalWire
        );
    }

    #[test]
    fn canonical_commitment_is_deterministic_and_separator_safe() {
        let a = SourceId::new("cti", "ab:c").unwrap();
        let b = SourceId::new("cti", "a:bc").unwrap();
        assert_ne!(a.canonical_bytes(), b.canonical_bytes());
        assert_ne!(a.commitment_sha256(), b.commitment_sha256());
        assert_eq!(a.commitment_sha256(), a.commitment_sha256());
    }

    #[test]
    fn identity_is_machine_readably_non_authoritative() {
        let id = EvidenceRelationId::new("investigation", "rel:7").unwrap();
        assert_eq!(id.authority_scope(), EvidenceAuthorityScopeV1::IdentityOnly);
        assert!(!id.establishes_truth());
        assert!(!id.establishes_source_authenticity());
        assert!(!id.establishes_evidence_validity());
        assert!(!id.establishes_independence());
        assert!(!id.grants_application_authority());
    }

    #[test]
    fn digest_hex_is_lowercase_canonical() {
        let id = AssessmentId::new("science", "assessment:alpha").unwrap();
        let hex = id.commitment_sha256_hex();
        assert_eq!(hex.len(), 64);
        assert!(hex
            .bytes()
            .all(|b| b.is_ascii_digit() || matches!(b, b'a'..=b'f')));
    }
}
