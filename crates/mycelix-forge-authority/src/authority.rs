// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Capability-scoped, version-linked project authority.
//!
//! Authority epochs describe *who may do what* for one project at one point
//! in its authority history. They deliberately do not contain signatures;
//! later Forge layers bind signed authorization evidence to these exact epoch
//! subjects.

use crate::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::{collections::BTreeSet, fmt};
use thiserror::Error;

const AUTHORITY_EPOCH_DOMAIN_V1: &[u8] = b"mycelix-forge/authority-epoch/v1\0";

/// Typed authority capabilities understood by Forge protocol v1.
///
/// Numeric codes are part of the canonical representation. Existing codes
/// MUST NOT be reassigned in a later implementation.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub enum Capability {
    #[serde(rename = "review-source")]
    ReviewSource,
    #[serde(rename = "merge-protected")]
    MergeProtected,
    #[serde(rename = "manage-policy")]
    ManagePolicy,
    #[serde(rename = "manage-authority")]
    ManageAuthority,
    #[serde(rename = "qualify-build")]
    QualifyBuild,
    #[serde(rename = "release")]
    Release,
    #[serde(rename = "recover-authority")]
    RecoverAuthority,
    #[serde(rename = "witness")]
    Witness,
}

impl Capability {
    /// Stable v1 canonical code.
    pub const fn code(self) -> u16 {
        match self {
            Self::ReviewSource => 1,
            Self::MergeProtected => 2,
            Self::ManagePolicy => 3,
            Self::ManageAuthority => 4,
            Self::QualifyBuild => 5,
            Self::Release => 6,
            Self::RecoverAuthority => 7,
            Self::Witness => 8,
        }
    }
}

/// Provider-neutral principal identifier.
///
/// The digest may commit to a public key, DID document, Xenia identity root,
/// or another adapter-defined identity object. Forge core does not assume the
/// identity provider.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct PrincipalId(Digest);

impl PrincipalId {
    pub fn new(commitment: Digest) -> Self {
        Self(commitment)
    }

    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

impl fmt::Display for PrincipalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "principal:{}", self.0)
    }
}

/// Capabilities granted to one principal in an authority epoch.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PrincipalGrant {
    principal: PrincipalId,
    capabilities: BTreeSet<Capability>,
}

impl PrincipalGrant {
    pub fn new(
        principal: PrincipalId,
        capabilities: impl IntoIterator<Item = Capability>,
    ) -> Result<Self, AuthorityError> {
        let capabilities = capabilities.into_iter().collect::<BTreeSet<_>>();
        if capabilities.is_empty() {
            return Err(AuthorityError::EmptyCapabilityGrant(principal));
        }

        Ok(Self {
            principal,
            capabilities,
        })
    }

    pub fn principal(&self) -> &PrincipalId {
        &self.principal
    }

    pub fn capabilities(&self) -> &BTreeSet<Capability> {
        &self.capabilities
    }

    pub fn has(&self, capability: Capability) -> bool {
        self.capabilities.contains(&capability)
    }
}

impl<'de> Deserialize<'de> for PrincipalGrant {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireGrant {
            principal: PrincipalId,
            capabilities: BTreeSet<Capability>,
        }

        let wire = WireGrant::deserialize(deserializer)?;
        Self::new(wire.principal, wire.capabilities).map_err(D::Error::custom)
    }
}

/// Threshold required for one capability.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct CapabilityRule {
    capability: Capability,
    threshold: u16,
}

impl CapabilityRule {
    pub fn new(capability: Capability, threshold: u16) -> Result<Self, AuthorityError> {
        if threshold == 0 {
            return Err(AuthorityError::ZeroThreshold(capability));
        }
        Ok(Self {
            capability,
            threshold,
        })
    }

    pub const fn capability(self) -> Capability {
        self.capability
    }

    pub const fn threshold(self) -> u16 {
        self.threshold
    }
}

impl<'de> Deserialize<'de> for CapabilityRule {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireRule {
            capability: Capability,
            threshold: u16,
        }

        let wire = WireRule::deserialize(deserializer)?;
        Self::new(wire.capability, wire.threshold).map_err(D::Error::custom)
    }
}

/// Constructor input for a validated [`AuthorityEpoch`].
#[derive(Clone, Debug)]
pub struct AuthorityEpochParts {
    pub project: ProjectIdentity,
    pub sequence: u64,
    pub previous: Option<Digest>,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: Option<u64>,
    pub grants: Vec<PrincipalGrant>,
    pub thresholds: Vec<CapabilityRule>,
    pub revoked_principals: Vec<PrincipalId>,
}

/// A normalized, validated authority state for one project.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AuthorityEpoch {
    version: ProtocolVersion,
    project: ProjectIdentity,
    sequence: u64,
    previous: Option<Digest>,
    valid_from_unix_ms: u64,
    valid_until_unix_ms: Option<u64>,
    grants: Vec<PrincipalGrant>,
    thresholds: Vec<CapabilityRule>,
    revoked_principals: Vec<PrincipalId>,
}

impl AuthorityEpoch {
    /// Construct and normalize an authority epoch, rejecting ambiguous or
    /// impossible authority states.
    pub fn new(mut parts: AuthorityEpochParts) -> Result<Self, AuthorityError> {
        match (parts.sequence, parts.previous.is_some()) {
            (0, true) => return Err(AuthorityError::GenesisHasPrevious),
            (0, false) => {}
            (_, false) => return Err(AuthorityError::NonGenesisMissingPrevious),
            (_, true) => {}
        }

        if let Some(valid_until) = parts.valid_until_unix_ms {
            if valid_until <= parts.valid_from_unix_ms {
                return Err(AuthorityError::InvalidValidityWindow {
                    valid_from_unix_ms: parts.valid_from_unix_ms,
                    valid_until_unix_ms: valid_until,
                });
            }
        }

        if parts.grants.is_empty() {
            return Err(AuthorityError::NoPrincipals);
        }

        parts.grants.sort_by(|a, b| a.principal.cmp(&b.principal));
        if let Some(duplicate) = adjacent_duplicate(parts.grants.iter().map(|grant| &grant.principal)) {
            return Err(AuthorityError::DuplicatePrincipal(duplicate.clone()));
        }

        parts.revoked_principals.sort();
        if let Some(duplicate) = adjacent_duplicate(parts.revoked_principals.iter()) {
            return Err(AuthorityError::DuplicateRevocation(duplicate.clone()));
        }

        for revoked in &parts.revoked_principals {
            if parts.grants.iter().any(|grant| &grant.principal == revoked) {
                return Err(AuthorityError::RevokedPrincipalStillGranted(revoked.clone()));
            }
        }

        parts
            .thresholds
            .sort_by_key(|rule| (rule.capability.code(), rule.threshold));
        if let Some(duplicate) = adjacent_duplicate(parts.thresholds.iter().map(|rule| &rule.capability)) {
            return Err(AuthorityError::DuplicateCapabilityRule(*duplicate));
        }

        let granted_capabilities = parts
            .grants
            .iter()
            .flat_map(|grant| grant.capabilities.iter().copied())
            .collect::<BTreeSet<_>>();

        for capability in &granted_capabilities {
            let Some(rule) = parts
                .thresholds
                .iter()
                .find(|rule| rule.capability == *capability)
            else {
                return Err(AuthorityError::MissingCapabilityRule(*capability));
            };

            let eligible = parts
                .grants
                .iter()
                .filter(|grant| grant.has(*capability))
                .count();
            if usize::from(rule.threshold) > eligible {
                return Err(AuthorityError::ThresholdExceedsEligible {
                    capability: *capability,
                    threshold: rule.threshold,
                    eligible,
                });
            }
        }

        for rule in &parts.thresholds {
            if !granted_capabilities.contains(&rule.capability) {
                return Err(AuthorityError::RuleWithoutEligiblePrincipal(rule.capability));
            }
        }

        if !granted_capabilities.contains(&Capability::ManageAuthority) {
            return Err(AuthorityError::MissingManageAuthority);
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: parts.project,
            sequence: parts.sequence,
            previous: parts.previous,
            valid_from_unix_ms: parts.valid_from_unix_ms,
            valid_until_unix_ms: parts.valid_until_unix_ms,
            grants: parts.grants,
            thresholds: parts.thresholds,
            revoked_principals: parts.revoked_principals,
        })
    }

    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn previous(&self) -> Option<&Digest> {
        self.previous.as_ref()
    }

    pub const fn valid_from_unix_ms(&self) -> u64 {
        self.valid_from_unix_ms
    }

    pub const fn valid_until_unix_ms(&self) -> Option<u64> {
        self.valid_until_unix_ms
    }

    pub fn grants(&self) -> &[PrincipalGrant] {
        &self.grants
    }

    pub fn thresholds(&self) -> &[CapabilityRule] {
        &self.thresholds
    }

    pub fn revoked_principals(&self) -> &[PrincipalId] {
        &self.revoked_principals
    }

    /// Whether this epoch is temporally valid at `unix_ms`.
    pub fn is_valid_at(&self, unix_ms: u64) -> bool {
        let before_end = match self.valid_until_unix_ms {
            Some(valid_until) => unix_ms < valid_until,
            None => true,
        };
        unix_ms >= self.valid_from_unix_ms && before_end
    }

    /// Whether a principal is individually eligible for a capability in this
    /// epoch. Threshold satisfaction is intentionally a separate decision.
    pub fn is_principal_eligible(
        &self,
        principal: &PrincipalId,
        capability: Capability,
        unix_ms: u64,
    ) -> bool {
        self.is_valid_at(unix_ms)
            && self
                .grants
                .iter()
                .find(|grant| grant.principal() == principal)
                .is_some_and(|grant| grant.has(capability))
    }

    /// Required number of distinct eligible principals for a capability.
    pub fn threshold_for(&self, capability: Capability) -> Option<u16> {
        self.thresholds
            .iter()
            .find(|rule| rule.capability == capability)
            .map(|rule| rule.threshold)
    }

    /// Deterministic v1 bytes used to identify this exact authority state.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AuthorityError> {
        if self.version.get() != CURRENT_PROTOCOL_VERSION {
            return Err(AuthorityError::UnsupportedProtocolVersion(self.version.get()));
        }

        let mut out = Vec::new();
        out.extend_from_slice(AUTHORITY_EPOCH_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        out.extend_from_slice(&self.sequence.to_be_bytes());
        push_optional_digest(&mut out, self.previous.as_ref())?;
        out.extend_from_slice(&self.valid_from_unix_ms.to_be_bytes());
        match self.valid_until_unix_ms {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value.to_be_bytes());
            }
            None => out.push(0),
        }

        push_u32_len(&mut out, self.grants.len(), "grants")?;
        for grant in &self.grants {
            push_principal(&mut out, grant.principal())?;
            push_u16_len(&mut out, grant.capabilities.len(), "capabilities")?;
            for capability in &grant.capabilities {
                out.extend_from_slice(&capability.code().to_be_bytes());
            }
        }

        push_u16_len(&mut out, self.thresholds.len(), "thresholds")?;
        for rule in &self.thresholds {
            out.extend_from_slice(&rule.capability.code().to_be_bytes());
            out.extend_from_slice(&rule.threshold.to_be_bytes());
        }

        push_u32_len(
            &mut out,
            self.revoked_principals.len(),
            "revoked_principals",
        )?;
        for principal in &self.revoked_principals {
            push_principal(&mut out, principal)?;
        }

        Ok(out)
    }

    /// Digest identifying this exact normalized authority epoch.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, AuthorityError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }

    /// Validate direct lineage from `self` to `next`.
    ///
    /// The digest suite used for the predecessor commitment is selected by the
    /// `next.previous` digest itself, allowing explicit hash-suite migration.
    pub fn validate_successor(&self, next: &Self) -> Result<(), AuthorityError> {
        if self.project != next.project {
            return Err(AuthorityError::ProjectChangedAcrossEpoch);
        }

        let expected_sequence = self
            .sequence
            .checked_add(1)
            .ok_or(AuthorityError::SequenceOverflow)?;
        if next.sequence != expected_sequence {
            return Err(AuthorityError::NonContiguousSequence {
                expected: expected_sequence,
                actual: next.sequence,
            });
        }

        let previous = next
            .previous
            .as_ref()
            .ok_or(AuthorityError::NonGenesisMissingPrevious)?;
        let expected_previous = self.digest(previous.algorithm())?;
        if *previous != expected_previous {
            return Err(AuthorityError::PreviousEpochMismatch);
        }

        if next.valid_from_unix_ms < self.valid_from_unix_ms {
            return Err(AuthorityError::ValidityMovedBackwards {
                previous_valid_from_unix_ms: self.valid_from_unix_ms,
                next_valid_from_unix_ms: next.valid_from_unix_ms,
            });
        }

        for previously_revoked in &self.revoked_principals {
            if !next.revoked_principals.contains(previously_revoked) {
                return Err(AuthorityError::RevocationRemoved(previously_revoked.clone()));
            }
        }

        Ok(())
    }
}

impl<'de> Deserialize<'de> for AuthorityEpoch {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireAuthorityEpoch {
            version: ProtocolVersion,
            project: ProjectIdentity,
            sequence: u64,
            previous: Option<Digest>,
            valid_from_unix_ms: u64,
            valid_until_unix_ms: Option<u64>,
            grants: Vec<PrincipalGrant>,
            thresholds: Vec<CapabilityRule>,
            revoked_principals: Vec<PrincipalId>,
        }

        let wire = WireAuthorityEpoch::deserialize(deserializer)?;
        if wire.version != ProtocolVersion::CURRENT {
            return Err(D::Error::custom(AuthorityError::UnsupportedProtocolVersion(
                wire.version.get(),
            )));
        }

        Self::new(AuthorityEpochParts {
            project: wire.project,
            sequence: wire.sequence,
            previous: wire.previous,
            valid_from_unix_ms: wire.valid_from_unix_ms,
            valid_until_unix_ms: wire.valid_until_unix_ms,
            grants: wire.grants,
            thresholds: wire.thresholds,
            revoked_principals: wire.revoked_principals,
        })
        .map_err(D::Error::custom)
    }
}

/// Validation failures for project authority state.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AuthorityError {
    #[error("authority genesis epoch must not name a predecessor")]
    GenesisHasPrevious,
    #[error("non-genesis authority epoch must name a predecessor")]
    NonGenesisMissingPrevious,
    #[error("authority epoch must contain at least one principal grant")]
    NoPrincipals,
    #[error("principal {0} has an empty capability grant")]
    EmptyCapabilityGrant(PrincipalId),
    #[error("duplicate principal grant: {0}")]
    DuplicatePrincipal(PrincipalId),
    #[error("duplicate revocation: {0}")]
    DuplicateRevocation(PrincipalId),
    #[error("revoked principal is still granted capabilities: {0}")]
    RevokedPrincipalStillGranted(PrincipalId),
    #[error("capability {0:?} has a zero threshold")]
    ZeroThreshold(Capability),
    #[error("duplicate threshold rule for capability {0:?}")]
    DuplicateCapabilityRule(Capability),
    #[error("capability {0:?} is granted but has no threshold rule")]
    MissingCapabilityRule(Capability),
    #[error("threshold rule for {0:?} has no eligible principal")]
    RuleWithoutEligiblePrincipal(Capability),
    #[error(
        "threshold for {capability:?} exceeds eligible principals: threshold={threshold}, eligible={eligible}"
    )]
    ThresholdExceedsEligible {
        capability: Capability,
        threshold: u16,
        eligible: usize,
    },
    #[error("authority epoch must grant ManageAuthority to at least one principal")]
    MissingManageAuthority,
    #[error(
        "invalid authority validity window: from={valid_from_unix_ms}, until={valid_until_unix_ms}"
    )]
    InvalidValidityWindow {
        valid_from_unix_ms: u64,
        valid_until_unix_ms: u64,
    },
    #[error("canonical collection {field} is too large: {len} > {max}")]
    CanonicalCollectionTooLarge {
        field: &'static str,
        len: usize,
        max: usize,
    },
    #[error("unsupported Forge authority protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    #[error("project identity changed across authority epochs")]
    ProjectChangedAcrossEpoch,
    #[error("authority epoch sequence overflow")]
    SequenceOverflow,
    #[error("authority sequence is not contiguous: expected {expected}, got {actual}")]
    NonContiguousSequence { expected: u64, actual: u64 },
    #[error("authority predecessor digest does not match the prior epoch")]
    PreviousEpochMismatch,
    #[error(
        "authority validity moved backwards: previous={previous_valid_from_unix_ms}, next={next_valid_from_unix_ms}"
    )]
    ValidityMovedBackwards {
        previous_valid_from_unix_ms: u64,
        next_valid_from_unix_ms: u64,
    },
    #[error("previously revoked principal was removed from revocation history: {0}")]
    RevocationRemoved(PrincipalId),
}

fn adjacent_duplicate<'a, T: Ord + 'a>(
    mut values: impl Iterator<Item = &'a T>,
) -> Option<&'a T> {
    let mut previous = values.next()?;
    for value in values {
        if value == previous {
            return Some(value);
        }
        previous = value;
    }
    None
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), AuthorityError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_principal(out: &mut Vec<u8>, principal: &PrincipalId) -> Result<(), AuthorityError> {
    push_digest(out, principal.commitment())
}

fn push_optional_digest(out: &mut Vec<u8>, digest: Option<&Digest>) -> Result<(), AuthorityError> {
    match digest {
        Some(digest) => {
            out.push(1);
            push_digest(out, digest)
        }
        None => {
            out.push(0);
            Ok(())
        }
    }
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), AuthorityError> {
    let algorithm = digest.algorithm().id().as_bytes();
    push_u16_len(out, algorithm.len(), "digest_algorithm")?;
    out.extend_from_slice(algorithm);
    push_u32_len(out, digest.as_bytes().len(), "digest_bytes")?;
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_u16_len(out: &mut Vec<u8>, len: usize, field: &'static str) -> Result<(), AuthorityError> {
    let value = u16::try_from(len).map_err(|_| AuthorityError::CanonicalCollectionTooLarge {
        field,
        len,
        max: u16::MAX as usize,
    })?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn push_u32_len(out: &mut Vec<u8>, len: usize, field: &'static str) -> Result<(), AuthorityError> {
    let value = u32::try_from(len).map_err(|_| AuthorityError::CanonicalCollectionTooLarge {
        field,
        len,
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        let seed = ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22));
        ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn grant(principal: PrincipalId, capabilities: &[Capability]) -> PrincipalGrant {
        PrincipalGrant::new(principal, capabilities.iter().copied()).unwrap()
    }

    fn genesis() -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: None,
            grants: vec![
                grant(
                    principal(1),
                    &[Capability::ManageAuthority, Capability::ReviewSource],
                ),
                grant(principal(2), &[Capability::ManageAuthority]),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 2).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    #[test]
    fn genesis_is_normalized_and_authorizes_by_capability() {
        let epoch = genesis();
        assert_eq!(epoch.sequence(), 0);
        assert_eq!(epoch.threshold_for(Capability::ManageAuthority), Some(2));
        assert!(epoch.is_principal_eligible(&principal(1), Capability::ReviewSource, 1_000));
        assert!(!epoch.is_principal_eligible(&principal(2), Capability::ReviewSource, 1_000));
    }

    #[test]
    fn non_genesis_requires_predecessor() {
        let error = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 1,
            previous: None,
            valid_from_unix_ms: 2_000,
            valid_until_unix_ms: None,
            grants: vec![grant(principal(1), &[Capability::ManageAuthority])],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap_err();
        assert_eq!(error, AuthorityError::NonGenesisMissingPrevious);
    }

    #[test]
    fn duplicate_principals_fail_closed() {
        let same = grant(principal(1), &[Capability::ManageAuthority]);
        let error = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 0,
            valid_until_unix_ms: None,
            grants: vec![same.clone(), same],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap_err();
        assert_eq!(error, AuthorityError::DuplicatePrincipal(principal(1)));
    }

    #[test]
    fn thresholds_cannot_exceed_eligible_principals() {
        let error = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 0,
            valid_until_unix_ms: None,
            grants: vec![grant(principal(1), &[Capability::ManageAuthority])],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 2).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap_err();
        assert_eq!(
            error,
            AuthorityError::ThresholdExceedsEligible {
                capability: Capability::ManageAuthority,
                threshold: 2,
                eligible: 1,
            }
        );
    }

    #[test]
    fn revoked_principal_cannot_remain_granted() {
        let p = principal(1);
        let error = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 0,
            valid_until_unix_ms: None,
            grants: vec![grant(p.clone(), &[Capability::ManageAuthority])],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![p.clone()],
        })
        .unwrap_err();
        assert_eq!(error, AuthorityError::RevokedPrincipalStillGranted(p));
    }

    #[test]
    fn validity_window_is_start_inclusive_end_exclusive() {
        let epoch = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 10,
            valid_until_unix_ms: Some(20),
            grants: vec![grant(principal(1), &[Capability::ManageAuthority])],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap();
        assert!(!epoch.is_valid_at(9));
        assert!(epoch.is_valid_at(10));
        assert!(epoch.is_valid_at(19));
        assert!(!epoch.is_valid_at(20));
    }

    #[test]
    fn canonicalization_is_independent_of_input_order() {
        let a = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: None,
            grants: vec![
                grant(principal(2), &[Capability::ManageAuthority]),
                grant(
                    principal(1),
                    &[Capability::ReviewSource, Capability::ManageAuthority],
                ),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
                CapabilityRule::new(Capability::ManageAuthority, 2).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap();
        let b = genesis();
        assert_eq!(a.canonical_bytes().unwrap(), b.canonical_bytes().unwrap());
        assert_eq!(
            a.digest(DigestAlgorithm::Sha256).unwrap(),
            b.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn exact_authority_mutation_changes_epoch_digest() {
        let original = genesis();
        let mutated = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_001,
            valid_until_unix_ms: None,
            grants: original.grants().to_vec(),
            thresholds: original.thresholds().to_vec(),
            revoked_principals: vec![],
        })
        .unwrap();
        assert_ne!(
            original.digest(DigestAlgorithm::Sha256).unwrap(),
            mutated.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn successor_must_link_exact_prior_epoch_and_can_change_hash_suite() {
        let first = genesis();
        let previous = first.digest(DigestAlgorithm::Blake3_256).unwrap();
        let next = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 1,
            previous: Some(previous),
            valid_from_unix_ms: 2_000,
            valid_until_unix_ms: None,
            grants: first.grants().to_vec(),
            thresholds: first.thresholds().to_vec(),
            revoked_principals: vec![],
        })
        .unwrap();
        assert_eq!(first.validate_successor(&next), Ok(()));
    }

    #[test]
    fn revocation_history_is_monotonic() {
        let first = genesis();
        let revoked = principal(9);
        let second = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 1,
            previous: Some(first.digest(DigestAlgorithm::Sha256).unwrap()),
            valid_from_unix_ms: 2_000,
            valid_until_unix_ms: None,
            grants: first.grants().to_vec(),
            thresholds: first.thresholds().to_vec(),
            revoked_principals: vec![revoked.clone()],
        })
        .unwrap();
        first.validate_successor(&second).unwrap();

        let third = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 2,
            previous: Some(second.digest(DigestAlgorithm::Sha256).unwrap()),
            valid_from_unix_ms: 3_000,
            valid_until_unix_ms: None,
            grants: second.grants().to_vec(),
            thresholds: second.thresholds().to_vec(),
            revoked_principals: vec![],
        })
        .unwrap();

        assert_eq!(
            second.validate_successor(&third).unwrap_err(),
            AuthorityError::RevocationRemoved(revoked)
        );
    }

    #[test]
    fn serde_round_trip_revalidates_and_normalizes_epoch() {
        let epoch = genesis();
        let encoded = serde_json::to_string(&epoch).unwrap();
        let decoded: AuthorityEpoch = serde_json::from_str(&encoded).unwrap();
        assert_eq!(epoch, decoded);
        assert_eq!(
            epoch.canonical_bytes().unwrap(),
            decoded.canonical_bytes().unwrap()
        );
    }
}
