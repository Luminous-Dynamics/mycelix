use std::collections::{BTreeMap, BTreeSet};

use mycelix_infrastructure_types::{PartyIdV1, StableIdV1};

use crate::ReaderEnrollmentErrorV1;

const CREDENTIAL_DOMAIN_V1: &str = "content-fabric/xenia-reader-credential";
const ENROLLMENT_DOMAIN_V1: &str = "content-fabric/reader-enrollment";
const REGISTRY_DOMAIN_V1: &str = "content-fabric/reader-enrollment-registry";
const SCHEMA_VERSION_V1: u16 = 1;
const ED25519_SUITE_V1: &[u8] = b"ed25519-rfc8032";
const ML_DSA_65_SUITE_V1: &[u8] = b"ml-dsa-65-fips204";

/// Exact Ed25519 public-key length used by Xenia's current peer identity.
pub const XENIA_ED25519_PUBLIC_KEY_LEN_V1: usize = 32;
/// Exact ML-DSA-65 verifying-key length used by Xenia's current FIPS 204 profile.
pub const XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1: usize = 1952;

/// Exact hybrid public-key credential used for Content Fabric reader enrollment.
///
/// This is **identity data, not authentication evidence**. Any caller can supply
/// public keys. A later transport adapter must prove that Xenia authenticated the
/// same pair before using an enrollment for request authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct XeniaHybridReaderCredentialV1 {
    id: StableIdV1,
    ed25519_public_key: [u8; XENIA_ED25519_PUBLIC_KEY_LEN_V1],
    ml_dsa_65_public_key: Vec<u8>,
}

impl XeniaHybridReaderCredentialV1 {
    /// Construct and commit an exact Xenia Ed25519 + ML-DSA-65 public-key pair.
    ///
    /// This validates representation length only. Cryptographic key validity and
    /// proof of possession remain the responsibility of Xenia's handshake.
    pub fn new(
        ed25519_public_key: [u8; XENIA_ED25519_PUBLIC_KEY_LEN_V1],
        ml_dsa_65_public_key: impl Into<Vec<u8>>,
    ) -> Result<Self, ReaderEnrollmentErrorV1> {
        let ml_dsa_65_public_key = ml_dsa_65_public_key.into();
        validate_ml_dsa_65_length(&ml_dsa_65_public_key)?;
        let id = derive_credential_id(&ed25519_public_key, &ml_dsa_65_public_key)?;
        Ok(Self {
            id,
            ed25519_public_key,
            ml_dsa_65_public_key,
        })
    }

    /// Reconstruct a stored/wire credential while verifying its claimed ID.
    pub fn from_stable_id(
        ed25519_public_key: [u8; XENIA_ED25519_PUBLIC_KEY_LEN_V1],
        ml_dsa_65_public_key: impl Into<Vec<u8>>,
        claimed_id: StableIdV1,
    ) -> Result<Self, ReaderEnrollmentErrorV1> {
        let credential = Self::new(ed25519_public_key, ml_dsa_65_public_key)?;
        if credential.id != claimed_id {
            return Err(ReaderEnrollmentErrorV1::CredentialCommitmentMismatch);
        }
        Ok(credential)
    }

    /// Stable domain-separated commitment to the exact hybrid key pair.
    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    /// Exact enrolled Ed25519 verifying key.
    pub fn ed25519_public_key(&self) -> &[u8; XENIA_ED25519_PUBLIC_KEY_LEN_V1] {
        &self.ed25519_public_key
    }

    /// Exact enrolled ML-DSA-65 verifying key.
    pub fn ml_dsa_65_public_key(&self) -> &[u8] {
        &self.ml_dsa_65_public_key
    }
}

/// Explicit mapping from one exact hybrid peer credential to a Mycelix reader
/// principal and zero or more groups.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReaderEnrollmentV1 {
    id: StableIdV1,
    credential: XeniaHybridReaderCredentialV1,
    principal: PartyIdV1,
    groups: Vec<StableIdV1>,
}

impl ReaderEnrollmentV1 {
    /// Create one enrollment, canonicalizing group order and duplicates.
    pub fn new(
        credential: XeniaHybridReaderCredentialV1,
        principal: PartyIdV1,
        groups: impl IntoIterator<Item = StableIdV1>,
    ) -> Result<Self, ReaderEnrollmentErrorV1> {
        if principal == PartyIdV1::ZERO {
            return Err(ReaderEnrollmentErrorV1::ZeroPrincipal);
        }
        let groups = groups.into_iter().collect::<BTreeSet<_>>();
        if groups.contains(&StableIdV1::ZERO) {
            return Err(ReaderEnrollmentErrorV1::ZeroGroup);
        }
        let groups = groups.into_iter().collect::<Vec<_>>();
        let id = derive_enrollment_id(credential.id(), principal, &groups)?;
        Ok(Self {
            id,
            credential,
            principal,
            groups,
        })
    }

    /// Reconstruct one enrollment while verifying the claimed enrollment ID.
    pub fn from_stable_id(
        credential: XeniaHybridReaderCredentialV1,
        principal: PartyIdV1,
        groups: impl IntoIterator<Item = StableIdV1>,
        claimed_id: StableIdV1,
    ) -> Result<Self, ReaderEnrollmentErrorV1> {
        let enrollment = Self::new(credential, principal, groups)?;
        if enrollment.id != claimed_id {
            return Err(ReaderEnrollmentErrorV1::EnrollmentCommitmentMismatch);
        }
        Ok(enrollment)
    }

    /// Stable commitment to credential, application principal, and canonical groups.
    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    /// Exact enrolled hybrid credential.
    pub fn credential(&self) -> &XeniaHybridReaderCredentialV1 {
        &self.credential
    }

    /// Explicit Mycelix application principal assigned by enrollment policy.
    pub fn principal(&self) -> PartyIdV1 {
        self.principal
    }

    /// Canonical sorted, duplicate-free group membership snapshot.
    pub fn groups(&self) -> &[StableIdV1] {
        &self.groups
    }
}

/// Immutable canonical snapshot of reader enrollments indexed by exact hybrid
/// credential commitment.
///
/// The registry is a directory, not an authenticator. `lookup_keys` answers only
/// "what identity did policy enroll for this exact key pair?" It does not prove
/// that a live peer possesses either private key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReaderEnrollmentRegistryV1 {
    id: StableIdV1,
    by_credential: BTreeMap<StableIdV1, ReaderEnrollmentV1>,
}

impl ReaderEnrollmentRegistryV1 {
    /// Construct a deterministic registry snapshot.
    ///
    /// More than one credential may map to the same principal to support
    /// explicit key-rotation overlap. The same credential may appear only once.
    pub fn from_enrollments(
        enrollments: impl IntoIterator<Item = ReaderEnrollmentV1>,
    ) -> Result<Self, ReaderEnrollmentErrorV1> {
        let mut by_credential = BTreeMap::new();
        for enrollment in enrollments {
            let credential_id = enrollment.credential().id();
            if by_credential.insert(credential_id, enrollment).is_some() {
                return Err(ReaderEnrollmentErrorV1::DuplicateCredential(credential_id));
            }
        }
        let id = derive_registry_id(by_credential.values())?;
        Ok(Self { id, by_credential })
    }

    /// Stable commitment to the complete canonical enrollment snapshot.
    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    /// Number of exact hybrid credentials enrolled in this snapshot.
    pub fn len(&self) -> usize {
        self.by_credential.len()
    }

    /// Whether the snapshot contains no enrolled hybrid credentials.
    pub fn is_empty(&self) -> bool {
        self.by_credential.is_empty()
    }

    /// Look up the policy enrollment for an exact hybrid public-key pair.
    ///
    /// Length-valid public keys still carry no authentication claim. The later
    /// Xenia adapter must call this only with keys taken from sealed handshake
    /// evidence bound to the live connection generation.
    pub fn lookup_keys(
        &self,
        ed25519_public_key: &[u8; XENIA_ED25519_PUBLIC_KEY_LEN_V1],
        ml_dsa_65_public_key: &[u8],
    ) -> Result<Option<&ReaderEnrollmentV1>, ReaderEnrollmentErrorV1> {
        validate_ml_dsa_65_length(ml_dsa_65_public_key)?;
        let credential_id = derive_credential_id(ed25519_public_key, ml_dsa_65_public_key)?;
        let Some(enrollment) = self.by_credential.get(&credential_id) else {
            return Ok(None);
        };
        let enrolled = enrollment.credential();
        if enrolled.ed25519_public_key() != ed25519_public_key
            || enrolled.ml_dsa_65_public_key() != ml_dsa_65_public_key
        {
            // Stable-ID collision or corrupted in-memory state: never authorize a
            // merely colliding commitment as the enrolled exact key pair.
            return Ok(None);
        }
        Ok(Some(enrollment))
    }
}

fn validate_ml_dsa_65_length(key: &[u8]) -> Result<(), ReaderEnrollmentErrorV1> {
    if key.len() != XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1 {
        return Err(ReaderEnrollmentErrorV1::InvalidMlDsa65PublicKeyLength {
            actual: key.len(),
            expected: XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1,
        });
    }
    Ok(())
}

fn derive_credential_id(
    ed25519_public_key: &[u8; XENIA_ED25519_PUBLIC_KEY_LEN_V1],
    ml_dsa_65_public_key: &[u8],
) -> Result<StableIdV1, ReaderEnrollmentErrorV1> {
    Ok(StableIdV1::derive(
        CREDENTIAL_DOMAIN_V1,
        SCHEMA_VERSION_V1,
        &[
            ED25519_SUITE_V1,
            ed25519_public_key,
            ML_DSA_65_SUITE_V1,
            ml_dsa_65_public_key,
        ],
    )?)
}

fn derive_enrollment_id(
    credential_id: StableIdV1,
    principal: PartyIdV1,
    groups: &[StableIdV1],
) -> Result<StableIdV1, ReaderEnrollmentErrorV1> {
    let mut fields = Vec::with_capacity(2 + groups.len());
    fields.push(credential_id.0.to_vec());
    fields.push(principal.0.to_vec());
    fields.extend(groups.iter().map(|group| group.0.to_vec()));
    let refs = fields.iter().map(Vec::as_slice).collect::<Vec<_>>();
    Ok(StableIdV1::derive(
        ENROLLMENT_DOMAIN_V1,
        SCHEMA_VERSION_V1,
        &refs,
    )?)
}

fn derive_registry_id<'a>(
    enrollments: impl IntoIterator<Item = &'a ReaderEnrollmentV1>,
) -> Result<StableIdV1, ReaderEnrollmentErrorV1> {
    let fields = enrollments
        .into_iter()
        .map(|enrollment| enrollment.id().0.to_vec())
        .collect::<Vec<_>>();
    let refs = fields.iter().map(Vec::as_slice).collect::<Vec<_>>();
    Ok(StableIdV1::derive(
        REGISTRY_DOMAIN_V1,
        SCHEMA_VERSION_V1,
        &refs,
    )?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ml(byte: u8) -> Vec<u8> {
        vec![byte; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1]
    }

    fn party(byte: u8) -> PartyIdV1 {
        PartyIdV1([byte; 32])
    }

    fn group(byte: u8) -> StableIdV1 {
        StableIdV1([byte; 32])
    }

    fn credential(ed: u8, pq: u8) -> XeniaHybridReaderCredentialV1 {
        XeniaHybridReaderCredentialV1::new([ed; 32], ml(pq)).unwrap()
    }

    #[test]
    fn credential_commitment_binds_both_signature_keys() {
        let base = credential(1, 2);
        let changed_ed = credential(3, 2);
        let changed_pq = credential(1, 4);
        assert_ne!(base.id(), changed_ed.id());
        assert_ne!(base.id(), changed_pq.id());
    }

    #[test]
    fn credential_rejects_wrong_ml_dsa_length() {
        assert!(matches!(
            XeniaHybridReaderCredentialV1::new([1; 32], vec![2; 10]),
            Err(ReaderEnrollmentErrorV1::InvalidMlDsa65PublicKeyLength { .. })
        ));
    }

    #[test]
    fn credential_reconstruction_rejects_wrong_commitment() {
        let source = credential(1, 2);
        assert!(matches!(
            XeniaHybridReaderCredentialV1::from_stable_id(
                [1; 32],
                ml(2),
                StableIdV1([9; 32]),
            ),
            Err(ReaderEnrollmentErrorV1::CredentialCommitmentMismatch)
        ));
        assert_ne!(source.id(), StableIdV1([9; 32]));
    }

    #[test]
    fn enrollment_canonicalizes_groups_and_rejects_zero_authority_values() {
        let enrollment = ReaderEnrollmentV1::new(
            credential(1, 2),
            party(7),
            [group(9), group(8), group(9)],
        )
        .unwrap();
        assert_eq!(enrollment.groups(), &[group(8), group(9)]);

        assert!(matches!(
            ReaderEnrollmentV1::new(credential(1, 2), PartyIdV1::ZERO, []),
            Err(ReaderEnrollmentErrorV1::ZeroPrincipal)
        ));
        assert!(matches!(
            ReaderEnrollmentV1::new(credential(1, 2), party(7), [StableIdV1::ZERO]),
            Err(ReaderEnrollmentErrorV1::ZeroGroup)
        ));
    }

    #[test]
    fn enrollment_commitment_changes_with_principal_or_groups() {
        let a = ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8)]).unwrap();
        let b = ReaderEnrollmentV1::new(credential(1, 2), party(6), [group(8)]).unwrap();
        let c = ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(9)]).unwrap();
        assert_ne!(a.id(), b.id());
        assert_ne!(a.id(), c.id());
    }

    #[test]
    fn registry_requires_exact_hybrid_pair_not_ed25519_alone() {
        let enrolled = ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8)]).unwrap();
        let registry = ReaderEnrollmentRegistryV1::from_enrollments([enrolled]).unwrap();

        let exact = registry.lookup_keys(&[1; 32], &ml(2)).unwrap().unwrap();
        assert_eq!(exact.principal(), party(7));
        assert!(registry.lookup_keys(&[1; 32], &ml(3)).unwrap().is_none());
        assert!(registry.lookup_keys(&[4; 32], &ml(2)).unwrap().is_none());
    }

    #[test]
    fn duplicate_credential_enrollment_fails_closed() {
        let a = ReaderEnrollmentV1::new(credential(1, 2), party(7), []).unwrap();
        let b = ReaderEnrollmentV1::new(credential(1, 2), party(8), []).unwrap();
        assert!(matches!(
            ReaderEnrollmentRegistryV1::from_enrollments([a, b]),
            Err(ReaderEnrollmentErrorV1::DuplicateCredential(_))
        ));
    }

    #[test]
    fn multiple_credentials_may_explicitly_map_to_same_principal() {
        let a = ReaderEnrollmentV1::new(credential(1, 2), party(7), []).unwrap();
        let b = ReaderEnrollmentV1::new(credential(3, 4), party(7), []).unwrap();
        let registry = ReaderEnrollmentRegistryV1::from_enrollments([a, b]).unwrap();
        assert_eq!(registry.len(), 2);
        assert_eq!(
            registry.lookup_keys(&[1; 32], &ml(2)).unwrap().unwrap().principal(),
            party(7)
        );
        assert_eq!(
            registry.lookup_keys(&[3; 32], &ml(4)).unwrap().unwrap().principal(),
            party(7)
        );
    }

    #[test]
    fn registry_commitment_is_input_order_independent() {
        let a = ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8)]).unwrap();
        let b = ReaderEnrollmentV1::new(credential(3, 4), party(9), [group(10)]).unwrap();
        let first = ReaderEnrollmentRegistryV1::from_enrollments([a.clone(), b.clone()]).unwrap();
        let second = ReaderEnrollmentRegistryV1::from_enrollments([b, a]).unwrap();
        assert_eq!(first.id(), second.id());
    }

    #[test]
    fn empty_registry_is_valid_deny_by_absence_snapshot() {
        let registry = ReaderEnrollmentRegistryV1::from_enrollments([]).unwrap();
        assert!(registry.is_empty());
        assert_eq!(registry.len(), 0);
        assert!(registry.lookup_keys(&[1; 32], &ml(2)).unwrap().is_none());
    }
}
