//! Request-scoped causal authority cuts.
//!
//! A cut is the exact set of authority dependencies required by one request.
//! Completeness is not inferred from the dependencies the caller happened to
//! provide: an independent resolver must attest that the dependency set is
//! complete for the exact request and policy snapshot. This module checks the
//! binding and frontier semantics after that external verification boundary.
//!
//! No signatures, inclusion proofs, policy language, or resolver correctness
//! are implemented here.

use super::{
    append_identifier, append_position, AuthorityPosition,
    AuthorityPositionCoverage, Digest, FrontierEvidence, RealmId, MAX_IDENTIFIER_BYTES,
};

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct DependencyId(String);

impl DependencyId {
    pub fn new(value: impl Into<String>) -> Result<Self, CutError> {
        let value = value.into();
        if value.trim().is_empty() {
            return Err(CutError::EmptyIdentifier);
        }
        if value.len() > MAX_IDENTIFIER_BYTES {
            return Err(CutError::IdentifierTooLong);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
#[repr(u8)]
pub enum DependencyKind {
    RealmPolicy = 1,
    SubjectMembership = 2,
    IssuerAdmission = 3,
    CredentialStatus = 4,
    GroupGraph = 5,
    RoleBinding = 6,
    Delegation = 7,
    FederationContract = 8,
    DevicePosture = 9,
    WorkloadTrust = 10,
    ResourcePolicy = 11,
    Revocation = 12,
    ExplicitDeny = 13,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorityDependency {
    id: DependencyId,
    kind: DependencyKind,
    minimum_position: AuthorityPosition,
}

impl AuthorityDependency {
    pub fn new(
        id: DependencyId,
        kind: DependencyKind,
        minimum_position: AuthorityPosition,
    ) -> Self {
        Self {
            id,
            kind,
            minimum_position,
        }
    }

    pub fn id(&self) -> &DependencyId {
        &self.id
    }

    pub fn kind(&self) -> DependencyKind {
        self.kind
    }

    pub fn minimum_position(&self) -> &AuthorityPosition {
        &self.minimum_position
    }

    fn append_canonical(&self, bytes: &mut Vec<u8>) -> Result<(), CutError> {
        append_identifier(bytes, self.id.as_str()).map_err(|_| CutError::IdentifierTooLong)?;
        bytes.push(self.kind as u8);
        append_position(bytes, &self.minimum_position)
            .map_err(|_| CutError::CanonicalEncodingFailure)?;
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CutError {
    EmptyIdentifier,
    IdentifierTooLong,
    EmptyDependencySet,
    DuplicateDependencyId,
    ZeroDigest,
    CanonicalEncodingFailure,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RequestScopedCausalCut {
    realm: RealmId,
    request_digest: Digest,
    policy_digest: Digest,
    resolver_profile_digest: Digest,
    dependencies: Vec<AuthorityDependency>,
}

impl RequestScopedCausalCut {
    pub fn new(
        realm: RealmId,
        request_digest: Digest,
        policy_digest: Digest,
        resolver_profile_digest: Digest,
        mut dependencies: Vec<AuthorityDependency>,
    ) -> Result<Self, CutError> {
        if request_digest == [0; 32]
            || policy_digest == [0; 32]
            || resolver_profile_digest == [0; 32]
        {
            return Err(CutError::ZeroDigest);
        }
        if dependencies.is_empty() {
            return Err(CutError::EmptyDependencySet);
        }

        dependencies.sort_by(|left, right| left.id.cmp(&right.id));
        for pair in dependencies.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(CutError::DuplicateDependencyId);
            }
        }

        Ok(Self {
            realm,
            request_digest,
            policy_digest,
            resolver_profile_digest,
            dependencies,
        })
    }

    pub fn realm(&self) -> &RealmId {
        &self.realm
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn resolver_profile_digest(&self) -> &Digest {
        &self.resolver_profile_digest
    }

    pub fn dependencies(&self) -> &[AuthorityDependency] {
        &self.dependencies
    }

    /// Deterministic, domain-separated bytes. The caller must hash these exact
    /// bytes with the pinned digest profile to obtain the cut commitment.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, CutError> {
        let count = u32::try_from(self.dependencies.len())
            .map_err(|_| CutError::CanonicalEncodingFailure)?;
        let mut bytes = b"MYC-DIR-REQUEST-SCOPED-CAUSAL-CUT\0\x01".to_vec();
        append_identifier(&mut bytes, self.realm.as_str())
            .map_err(|_| CutError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.policy_digest);
        bytes.extend_from_slice(&self.resolver_profile_digest);
        bytes.extend_from_slice(&count.to_be_bytes());
        for dependency in &self.dependencies {
            dependency.append_canonical(&mut bytes)?;
        }
        Ok(bytes)
    }
}

/// This status is supplied only after an external, pinned resolver/verifier
/// checks the dependency-closure proof. Setting it to VerifiedComplete without
/// that verification would be an authority-boundary violation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ClosureVerificationStatus {
    Unverified = 0,
    VerifiedComplete = 1,
    Incomplete = 2,
    Rejected = 3,
    Unavailable = 4,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DependencyClosureReceipt {
    request_digest: Digest,
    realm: RealmId,
    policy_digest: Digest,
    cut_digest: Digest,
    resolver_profile_digest: Digest,
    verifier_id: String,
    verifier_build_digest: Digest,
    proof_digest: Digest,
    status: ClosureVerificationStatus,
}

impl DependencyClosureReceipt {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        request_digest: Digest,
        realm: RealmId,
        policy_digest: Digest,
        cut_digest: Digest,
        resolver_profile_digest: Digest,
        verifier_id: impl Into<String>,
        verifier_build_digest: Digest,
        proof_digest: Digest,
        status: ClosureVerificationStatus,
    ) -> Result<Self, CutError> {
        let verifier_id = verifier_id.into();
        if verifier_id.trim().is_empty() {
            return Err(CutError::EmptyIdentifier);
        }
        if verifier_id.len() > MAX_IDENTIFIER_BYTES {
            return Err(CutError::IdentifierTooLong);
        }
        if request_digest == [0; 32]
            || policy_digest == [0; 32]
            || cut_digest == [0; 32]
            || resolver_profile_digest == [0; 32]
            || verifier_build_digest == [0; 32]
            || proof_digest == [0; 32]
        {
            return Err(CutError::ZeroDigest);
        }
        Ok(Self {
            request_digest,
            realm,
            policy_digest,
            cut_digest,
            resolver_profile_digest,
            verifier_id,
            verifier_build_digest,
            proof_digest,
            status,
        })
    }

    pub fn status(&self) -> ClosureVerificationStatus {
        self.status
    }

    /// Canonical commitment of the closure attestation claim. This is not a
    /// signature and does not authenticate the verifier.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, CutError> {
        let mut bytes = b"MYC-DIR-CLOSURE-RECEIPT\0\x01".to_vec();
        bytes.extend_from_slice(&self.request_digest);
        append_identifier(&mut bytes, self.realm.as_str())
            .map_err(|_| CutError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.policy_digest);
        bytes.extend_from_slice(&self.cut_digest);
        bytes.extend_from_slice(&self.resolver_profile_digest);
        append_identifier(&mut bytes, &self.verifier_id)
            .map_err(|_| CutError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.verifier_build_digest);
        bytes.extend_from_slice(&self.proof_digest);
        bytes.push(self.status as u8);
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CausalCutAssessment {
    Sufficient,
    ClosureUnverified,
    ClosureIncomplete,
    ClosureRejected,
    ClosureUnavailable,
    RequestBindingMismatch,
    RealmBindingMismatch,
    PolicyBindingMismatch,
    ResolverProfileMismatch,
    CutDigestMismatch,
    BehindRequiredDependency,
    IncomparableRequiredDependency,
}

/// Checks that an independently verified dependency closure matches the exact
/// request/policy/cut and that every required per-authority position is covered
/// by the local snapshot. This is a freshness/completeness precondition only;
/// it never returns an authorization Allow.
pub fn assess_request_scoped_cut(
    cut: &RequestScopedCausalCut,
    closure: &DependencyClosureReceipt,
    expected_cut_digest: &Digest,
    local_frontier: &FrontierEvidence,
) -> CausalCutAssessment {
    match closure.status {
        ClosureVerificationStatus::Unverified => {
            return CausalCutAssessment::ClosureUnverified
        }
        ClosureVerificationStatus::Incomplete => {
            return CausalCutAssessment::ClosureIncomplete
        }
        ClosureVerificationStatus::Rejected => {
            return CausalCutAssessment::ClosureRejected
        }
        ClosureVerificationStatus::Unavailable => {
            return CausalCutAssessment::ClosureUnavailable
        }
        ClosureVerificationStatus::VerifiedComplete => {}
    }

    if closure.request_digest != cut.request_digest {
        return CausalCutAssessment::RequestBindingMismatch;
    }
    if closure.realm != cut.realm {
        return CausalCutAssessment::RealmBindingMismatch;
    }
    if closure.policy_digest != cut.policy_digest {
        return CausalCutAssessment::PolicyBindingMismatch;
    }
    if closure.resolver_profile_digest != cut.resolver_profile_digest {
        return CausalCutAssessment::ResolverProfileMismatch;
    }
    if &closure.cut_digest != expected_cut_digest {
        return CausalCutAssessment::CutDigestMismatch;
    }

    let mut behind = false;
    let mut incomparable = false;
    for dependency in &cut.dependencies {
        match local_frontier.coverage_of(&dependency.minimum_position) {
            AuthorityPositionCoverage::Exact
            | AuthorityPositionCoverage::CoveredByVerifiedAncestry => {}
            AuthorityPositionCoverage::Behind => behind = true,
            AuthorityPositionCoverage::Incomparable => incomparable = true,
        }
    }

    if incomparable {
        CausalCutAssessment::IncomparableRequiredDependency
    } else if behind {
        CausalCutAssessment::BehindRequiredDependency
    } else {
        CausalCutAssessment::Sufficient
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthorityFrontier, AuthorityId, AuthorityPosition, FrontierEvidence, RealmId,
    };

    fn realm() -> RealmId {
        RealmId::new("realm-a").unwrap()
    }

    fn authority(value: &str) -> AuthorityId {
        AuthorityId::new(value).unwrap()
    }

    fn position(authority_id: &str, sequence: u64, marker: u8) -> AuthorityPosition {
        let mut digest = [0; 32];
        if sequence > 0 {
            digest[0] = marker;
        }
        AuthorityPosition::new(authority(authority_id), sequence, digest).unwrap()
    }

    fn cut(dependencies: Vec<AuthorityDependency>) -> RequestScopedCausalCut {
        RequestScopedCausalCut::new(
            realm(),
            [1; 32],
            [2; 32],
            [3; 32],
            dependencies,
        )
        .unwrap()
    }

    fn dependency(id: &str, authority_id: &str, seq: u64, marker: u8) -> AuthorityDependency {
        AuthorityDependency::new(
            DependencyId::new(id).unwrap(),
            DependencyKind::Revocation,
            position(authority_id, seq, marker),
        )
    }

    fn closure(cut: &RequestScopedCausalCut, cut_digest: Digest, status: ClosureVerificationStatus) -> DependencyClosureReceipt {
        DependencyClosureReceipt::new(
            *cut.request_digest(),
            cut.realm().clone(),
            *cut.policy_digest(),
            cut_digest,
            *cut.resolver_profile_digest(),
            "resolver-v1",
            [4; 32],
            [5; 32],
            status,
        )
        .unwrap()
    }

    #[test]
    fn request_cut_encoding_is_order_independent() {
        let first = cut(vec![
            dependency("membership", "realm-admin", 4, 4),
            dependency("issuer", "issuer-a", 3, 3),
        ]);
        let second = cut(vec![
            dependency("issuer", "issuer-a", 3, 3),
            dependency("membership", "realm-admin", 4, 4),
        ]);
        assert_eq!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
    }

    #[test]
    fn duplicate_dependency_ids_are_rejected() {
        let result = RequestScopedCausalCut::new(
            realm(),
            [1; 32],
            [2; 32],
            [3; 32],
            vec![
                dependency("issuer", "issuer-a", 3, 3),
                dependency("issuer", "issuer-b", 4, 4),
            ],
        );
        assert_eq!(result, Err(CutError::DuplicateDependencyId));
    }

    #[test]
    fn empty_or_zero_bound_dependency_cut_is_rejected() {
        assert_eq!(
            RequestScopedCausalCut::new(realm(), [1; 32], [2; 32], [3; 32], vec![]),
            Err(CutError::EmptyDependencySet)
        );
        assert_eq!(
            RequestScopedCausalCut::new(realm(), [0; 32], [2; 32], [3; 32], vec![dependency("p", "a", 1, 1)]),
            Err(CutError::ZeroDigest)
        );
    }

    #[test]
    fn verified_complete_cut_requires_every_dependency_position() {
        let current = FrontierEvidence::new(
            AuthorityFrontier::new(vec![position("realm-admin", 5, 5), position("issuer-a", 4, 4)]).unwrap(),
            vec![position("realm-admin", 4, 4), position("issuer-a", 3, 3)],
        ).unwrap();
        let cut = cut(vec![
            dependency("membership", "realm-admin", 4, 4),
            dependency("issuer", "issuer-a", 3, 3),
        ]);
        let receipt = closure(&cut, [9; 32], ClosureVerificationStatus::VerifiedComplete);
        assert_eq!(assess_request_scoped_cut(&cut, &receipt, &[9; 32], &current), CausalCutAssessment::Sufficient);
    }

    #[test]
    fn unrelated_authority_progress_does_not_mask_a_behind_dependency() {
        let current = FrontierEvidence::new(
            AuthorityFrontier::new(vec![position("realm-admin", 2, 2), position("issuer-a", 9, 9)]).unwrap(),
            vec![],
        ).unwrap();
        let cut = cut(vec![
            dependency("membership", "realm-admin", 3, 3),
            dependency("issuer", "issuer-a", 1, 1),
        ]);
        let receipt = closure(&cut, [9; 32], ClosureVerificationStatus::VerifiedComplete);
        assert_eq!(assess_request_scoped_cut(&cut, &receipt, &[9; 32], &current), CausalCutAssessment::BehindRequiredDependency);
    }

    #[test]
    fn unproven_higher_sequence_is_incomparable_for_a_required_dependency() {
        let current = FrontierEvidence::new(
            AuthorityFrontier::new(vec![position("issuer-a", 4, 4)]).unwrap(),
            vec![],
        ).unwrap();
        let cut = cut(vec![dependency("issuer", "issuer-a", 3, 3)]);
        let receipt = closure(&cut, [9; 32], ClosureVerificationStatus::VerifiedComplete);
        assert_eq!(assess_request_scoped_cut(&cut, &receipt, &[9; 32], &current), CausalCutAssessment::IncomparableRequiredDependency);
    }

    #[test]
    fn incomplete_or_unverified_dependency_inventory_never_satisfies_a_cut() {
        let current = FrontierEvidence::new(AuthorityFrontier::new(vec![position("issuer-a", 3, 3)]).unwrap(), vec![]).unwrap();
        let cut = cut(vec![dependency("issuer", "issuer-a", 3, 3)]);
        for (status, expected) in [
            (ClosureVerificationStatus::Unverified, CausalCutAssessment::ClosureUnverified),
            (ClosureVerificationStatus::Incomplete, CausalCutAssessment::ClosureIncomplete),
            (ClosureVerificationStatus::Rejected, CausalCutAssessment::ClosureRejected),
            (ClosureVerificationStatus::Unavailable, CausalCutAssessment::ClosureUnavailable),
        ] {
            let receipt = closure(&cut, [9; 32], status);
            assert_eq!(assess_request_scoped_cut(&cut, &receipt, &[9; 32], &current), expected);
        }
    }

    #[test]
    fn request_policy_realm_and_cut_substitution_are_detected() {
        let current = FrontierEvidence::new(AuthorityFrontier::new(vec![position("issuer-a", 3, 3)]).unwrap(), vec![]).unwrap();
        let cut = cut(vec![dependency("issuer", "issuer-a", 3, 3)]);
        let receipt = closure(&cut, [9; 32], ClosureVerificationStatus::VerifiedComplete);
        assert_eq!(
            assess_request_scoped_cut(&cut, &receipt, &[8; 32], &current),
            CausalCutAssessment::CutDigestMismatch
        );
        let wrong_request = DependencyClosureReceipt::new(
            [7; 32], realm(), [2; 32], [9; 32], [3; 32], "resolver-v1", [4; 32], [5; 32],
            ClosureVerificationStatus::VerifiedComplete
        ).unwrap();
        assert_eq!(
            assess_request_scoped_cut(&cut, &wrong_request, &[9; 32], &current),
            CausalCutAssessment::RequestBindingMismatch
        );
    }

    #[test]
    fn one_authority_can_supply_multiple_independently_required_dependencies() {
        let current = FrontierEvidence::new(
            AuthorityFrontier::new(vec![position("realm-admin", 7, 7)]).unwrap(),
            vec![position("realm-admin", 4, 4), position("realm-admin", 6, 6)],
        ).unwrap();
        let cut = cut(vec![
            dependency("membership", "realm-admin", 4, 4),
            dependency("role-policy", "realm-admin", 6, 6),
        ]);
        let receipt = closure(&cut, [9; 32], ClosureVerificationStatus::VerifiedComplete);
        assert_eq!(assess_request_scoped_cut(&cut, &receipt, &[9; 32], &current), CausalCutAssessment::Sufficient);
    }
}
