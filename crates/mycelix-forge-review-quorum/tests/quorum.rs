use mycelix_forge_authentication::{
    bind_principal_authentication, AuthenticationObservation, PrincipalAuthenticationRequest,
    PrincipalBinding,
};
use mycelix_forge_authority::{
    AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, Digest, DigestAlgorithm,
    PrincipalGrant, PrincipalId, ProjectIdentity, ProjectIdentitySeed, QuorumError,
    GENESIS_NONCE_LEN,
};
use mycelix_forge_proposal::ChangeProposal;
use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
use mycelix_forge_review::{
    bind_authenticated_review, qualify_review_authority, ReviewDecision, ReviewStatement,
    StructurallyAuthorizedReview,
};
use mycelix_forge_review_quorum::{qualify_approval_quorum, ApprovalQuorumError};

fn digest(byte: u8) -> Digest {
    Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
}

fn principal(byte: u8) -> PrincipalId {
    PrincipalId::new(digest(byte))
}

fn project() -> ProjectIdentity {
    ProjectIdentity::derive(
        &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
        DigestAlgorithm::Sha256,
    )
    .unwrap()
}

fn git(byte: u8) -> GitObjectId {
    GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
}

fn epoch(project: ProjectIdentity) -> AuthorityEpoch {
    AuthorityEpoch::new(AuthorityEpochParts {
        project,
        sequence: 0,
        previous: None,
        valid_from_unix_ms: 100,
        valid_until_unix_ms: Some(10_000),
        grants: vec![
            PrincipalGrant::new(
                principal(1),
                [Capability::ReviewSource, Capability::ManageAuthority],
            )
            .unwrap(),
            PrincipalGrant::new(
                principal(2),
                [Capability::ReviewSource, Capability::ManageAuthority],
            )
            .unwrap(),
        ],
        thresholds: vec![
            CapabilityRule::new(Capability::ReviewSource, 2).unwrap(),
            CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
        ],
        revoked_principals: vec![],
    })
    .unwrap()
}

fn proposal(epoch: &AuthorityEpoch) -> ChangeProposal {
    ChangeProposal::new(
        epoch.project().clone(),
        principal(9),
        epoch.digest(DigestAlgorithm::Sha256).unwrap(),
        digest(0x31),
        RepositoryRef::new("refs/heads/main").unwrap(),
        git(0x40),
        git(0x41),
        git(0x42),
        digest(0x50),
        vec![],
    )
    .unwrap()
}

fn authorized_review(
    proposal: &ChangeProposal,
    epoch: &AuthorityEpoch,
    reviewer: PrincipalId,
    decision: ReviewDecision,
    context_byte: u8,
) -> StructurallyAuthorizedReview {
    let statement = ReviewStatement::new(
        proposal.proposal_id(DigestAlgorithm::Sha256).unwrap(),
        reviewer.clone(),
        proposal.authority_epoch().clone(),
        proposal.repository_policy_state().clone(),
        decision,
        digest(context_byte),
    );
    let binding = PrincipalBinding::new(
        digest(context_byte.wrapping_add(1)),
        digest(context_byte.wrapping_add(2)),
        reviewer.clone(),
        digest(context_byte.wrapping_add(3)),
    );
    let request = PrincipalAuthenticationRequest::new(
        proposal.project().clone(),
        proposal.authority_epoch().clone(),
        reviewer,
        binding.digest(DigestAlgorithm::Sha256).unwrap(),
        Capability::ReviewSource,
        statement.digest(DigestAlgorithm::Sha256).unwrap(),
        [context_byte; 32],
    );
    let observation = AuthenticationObservation::new(
        request.digest(DigestAlgorithm::Sha256).unwrap(),
        digest(context_byte.wrapping_add(4)),
        digest(context_byte.wrapping_add(5)),
    );
    let auth = bind_principal_authentication(request, binding, epoch, observation).unwrap();
    let authenticated = bind_authenticated_review(proposal, statement, &auth).unwrap();
    qualify_review_authority(authenticated, epoch, 1_000).unwrap()
}

#[test]
fn two_distinct_approvals_satisfy_threshold() {
    let epoch = epoch(project());
    let proposal = proposal(&epoch);
    let reviews = vec![
        authorized_review(&proposal, &epoch, principal(2), ReviewDecision::Approve, 0x70),
        authorized_review(&proposal, &epoch, principal(1), ReviewDecision::Approve, 0x60),
    ];
    let quorum = qualify_approval_quorum(&proposal, &epoch, 1_000, reviews).unwrap();
    assert_eq!(quorum.threshold(), 2);
    assert_eq!(quorum.reviewers(), &[principal(1), principal(2)]);
}

#[test]
fn one_approval_is_below_threshold() {
    let epoch = epoch(project());
    let proposal = proposal(&epoch);
    let error = qualify_approval_quorum(
        &proposal,
        &epoch,
        1_000,
        vec![authorized_review(
            &proposal,
            &epoch,
            principal(1),
            ReviewDecision::Approve,
            0x60,
        )],
    )
    .unwrap_err();
    assert_eq!(
        error,
        ApprovalQuorumError::ThresholdNotSatisfied {
            threshold: 2,
            approvals: 1,
        }
    );
}

#[test]
fn duplicate_reviewer_cannot_be_counted_twice() {
    let epoch = epoch(project());
    let proposal = proposal(&epoch);
    let error = qualify_approval_quorum(
        &proposal,
        &epoch,
        1_000,
        vec![
            authorized_review(&proposal, &epoch, principal(1), ReviewDecision::Approve, 0x60),
            authorized_review(&proposal, &epoch, principal(1), ReviewDecision::Approve, 0x61),
        ],
    )
    .unwrap_err();
    assert_eq!(
        error,
        ApprovalQuorumError::Quorum(QuorumError::DuplicateAuthenticatedPrincipal(principal(1)))
    );
}

#[test]
fn request_changes_cannot_count_as_approval() {
    let epoch = epoch(project());
    let proposal = proposal(&epoch);
    let error = qualify_approval_quorum(
        &proposal,
        &epoch,
        1_000,
        vec![
            authorized_review(&proposal, &epoch, principal(1), ReviewDecision::Approve, 0x60),
            authorized_review(
                &proposal,
                &epoch,
                principal(2),
                ReviewDecision::RequestChanges,
                0x70,
            ),
        ],
    )
    .unwrap_err();
    assert_eq!(error, ApprovalQuorumError::NonApprovalDecision);
}
