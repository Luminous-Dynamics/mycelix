use ed25519_dalek::{Signer as _, SigningKey};
use ml_dsa::{
    signature::{Keypair as _, Signer as _}, B32, MlDsa65,
    Signature as MlDsaSignature, SigningKey as MlDsaSigningKey,
};
use mycelix_forge_authentication::{
    bind_principal_authentication, EvidenceBoundPrincipalAuthentication,
    PrincipalAuthenticationRequest, PrincipalBinding,
};
use mycelix_forge_authentication_policy::bind_project_policy_trusted_xenia_authentication_v1;
use mycelix_forge_authority::{
    AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant, PrincipalId,
};
use mycelix_forge_checkpoint_review_set::{
    project_witness_qualified_checkpoint_review_set_v1, CheckpointReviewSetError,
};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
};
use mycelix_forge_project_policy::{
    AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
};
use mycelix_forge_proposal::ChangeProposal;
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
};
use mycelix_forge_review_head_snapshot::{
    bind_review_head_snapshot_observation_v1, EvidenceBoundReviewHeadSnapshotV1,
    ReviewHeadCompletenessObservationV1, ReviewHeadCoverageClaimV1, ReviewHeadSnapshotClaimV1,
    ReviewHeadV1,
};
use mycelix_forge_review_head_witness::{
    bind_witnessed_review_head_attestation_v1, evaluate_review_head_witness_quorum_v1,
    ReviewHeadCompletenessWitnessStatementV1, WitnessQualifiedReviewHeadSnapshotV1,
};
use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewRevisionV1};
use mycelix_forge_review_state_auth::{
    bind_project_policy_trusted_review_revision_v1,
    bind_review_revision_authentication_v1, bind_xenia_provider_verified_review_revision_v1,
    qualify_review_revision_eligibility_v1, ProjectPolicyTrustedReviewRevisionV1,
};
use mycelix_forge_xenia::{
    xenia_challenge_commitment, xenia_key_lineage_commitment, xenia_operator_id_commitment,
    xenia_provider_namespace, XeniaHybridSuite, XeniaVerificationReceiptV1,
};
use mycelix_forge_xenia_provider::{
    provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
    ProviderVerifiedXeniaAuthenticationV1, TrustedXeniaVerifierV1,
    XeniaProviderAttestedReceiptV1,
};
use serde::Serialize;

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

struct ProviderIdentity {
    ed: SigningKey,
    ml: MlDsaSigningKey<MlDsa65>,
}

impl ProviderIdentity {
    fn new() -> Self {
        let seed: B32 = [0x82; 32].into();
        Self {
            ed: SigningKey::from_bytes(&[0x81; 32]),
            ml: MlDsaSigningKey::<MlDsa65>::from_seed(&seed),
        }
    }

    fn trusted(&self) -> TrustedXeniaVerifierV1 {
        TrustedXeniaVerifierV1::new(
            self.ed.verifying_key().to_bytes(),
            self.ml.verifying_key().encode().as_slice().to_vec(),
        )
        .unwrap()
    }

    fn envelope(&self, receipt: XeniaVerificationReceiptV1) -> XeniaProviderAttestedReceiptV1 {
        #[derive(Serialize)]
        struct WireEnvelope {
            receipt: XeniaVerificationReceiptV1,
            verifier_identity_commitment: Digest,
            ed25519_signature: Vec<u8>,
            ml_dsa_65_signature: Vec<u8>,
        }

        let trusted = self.trusted();
        let transcript =
            provider_attestation_transcript_v1(&receipt, trusted.identity_commitment()).unwrap();
        let ml_signature: MlDsaSignature<MlDsa65> = self.ml.sign(&transcript);
        let wire = WireEnvelope {
            receipt,
            verifier_identity_commitment: trusted.identity_commitment().clone(),
            ed25519_signature: self.ed.sign(&transcript).to_bytes().to_vec(),
            ml_dsa_65_signature: ml_signature.encode().as_slice().to_vec(),
        };
        serde_json::from_slice(&serde_json::to_vec(&wire).unwrap()).unwrap()
    }
}

struct Context {
    provider: ProviderIdentity,
    authority: AuthorityEpoch,
    project_policy: ProjectPolicyStateV1,
    provider_trust: AuthenticationProviderTrustPolicyV1,
    proposal: ChangeProposal,
}

fn context() -> Context {
    let provider = ProviderIdentity::new();
    let provider_trust = AuthenticationProviderTrustPolicyV1::new(
        project(),
        vec![TrustedProviderVerifierV1::new(
            xenia_provider_namespace(),
            provider.trusted().identity_commitment().clone(),
        )],
    )
    .unwrap();
    let project_policy = ProjectPolicyStateV1::new(
        project(),
        0,
        None,
        &provider_trust,
        DigestAlgorithm::Sha256,
    )
    .unwrap();
    let authority = AuthorityEpoch::new(AuthorityEpochParts {
        project: project(),
        sequence: 0,
        previous: None,
        valid_from_unix_ms: 1_000,
        valid_until_unix_ms: Some(5_000),
        grants: vec![
            PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
            PrincipalGrant::new(principal(0x20), [Capability::ReviewSource]).unwrap(),
            PrincipalGrant::new(principal(0x21), [Capability::ReviewSource]).unwrap(),
            PrincipalGrant::new(principal(0x30), [Capability::Witness]).unwrap(),
            PrincipalGrant::new(principal(0x31), [Capability::Witness]).unwrap(),
        ],
        thresholds: vec![
            CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
            CapabilityRule::new(Capability::ReviewSource, 2).unwrap(),
            CapabilityRule::new(Capability::Witness, 2).unwrap(),
        ],
        revoked_principals: vec![],
    })
    .unwrap();
    let repository_policy =
        RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
    let proposal = ChangeProposal::new(
        principal(0x40),
        &authority,
        &project_policy,
        &repository_policy,
        RepositoryRef::new("refs/heads/main").unwrap(),
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x41; 20]).unwrap(),
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x42; 20]).unwrap(),
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x43; 20]).unwrap(),
        digest(0x50),
        vec![],
        DigestAlgorithm::Sha256,
    )
    .unwrap();

    Context {
        provider,
        authority,
        project_policy,
        provider_trust,
        proposal,
    }
}

struct RawAuth {
    authentication: EvidenceBoundPrincipalAuthentication,
    provider_verified: ProviderVerifiedXeniaAuthenticationV1,
}

fn raw_auth(
    context: &Context,
    actor: PrincipalId,
    capability: Capability,
    action_subject: Digest,
    marker: u8,
) -> RawAuth {
    let operator =
        xenia_operator_id_commitment(&format!("operator:{marker:02x}")).unwrap();
    let lineage = xenia_key_lineage_commitment(
        &[marker.wrapping_add(1); 32],
        &[marker.wrapping_add(2); 64],
        None,
    )
    .unwrap();
    let binding = PrincipalBinding::new(
        xenia_provider_namespace(),
        operator.clone(),
        actor.clone(),
        lineage.clone(),
    );
    let request = PrincipalAuthenticationRequest::new(
        project(),
        context.authority.digest(DigestAlgorithm::Sha256).unwrap(),
        actor,
        binding.digest(DigestAlgorithm::Sha256).unwrap(),
        capability,
        action_subject,
        [marker; 32],
    );
    let receipt = XeniaVerificationReceiptV1::new(
        request.digest(DigestAlgorithm::Sha256).unwrap(),
        operator,
        lineage,
        xenia_challenge_commitment(request.challenge()),
        XeniaHybridSuite::Ed25519MlDsa65V1,
        digest(marker.wrapping_add(3)),
        digest(marker.wrapping_add(4)),
        digest(marker.wrapping_add(5)),
        1_797_000_000,
    );
    let verifier = context.provider.trusted();
    let envelope = context.provider.envelope(receipt);
    let provider_verified =
        verify_xenia_provider_authentication_v1(&verifier, &request, &binding, &envelope).unwrap();
    let authentication = bind_principal_authentication(
        request,
        binding,
        &context.authority,
        provider_verified.observation().clone(),
    )
    .unwrap();
    RawAuth {
        authentication,
        provider_verified,
    }
}

fn trusted_review(
    context: &Context,
    reviewer_byte: u8,
    decision: ReviewRevisionDecision,
    marker: u8,
    observed_at: u64,
) -> ProjectPolicyTrustedReviewRevisionV1 {
    let reviewer = principal(reviewer_byte);
    let revision = ReviewRevisionV1::genesis(
        &context.proposal,
        reviewer.clone(),
        decision,
        digest(marker),
        DigestAlgorithm::Sha256,
    )
    .unwrap();
    let revision_id = revision.revision_id(DigestAlgorithm::Sha256).unwrap();
    let raw = raw_auth(
        context,
        reviewer,
        Capability::ReviewSource,
        revision_id.commitment().clone(),
        marker.wrapping_add(20),
    );
    let bound = bind_review_revision_authentication_v1(
        &context.proposal,
        revision,
        &raw.authentication,
        &context.authority,
    )
    .unwrap();
    let eligible =
        qualify_review_revision_eligibility_v1(bound, &context.authority, observed_at).unwrap();
    let provider = bind_xenia_provider_verified_review_revision_v1(
        eligible,
        &raw.authentication,
        &raw.provider_verified,
    )
    .unwrap();
    bind_project_policy_trusted_review_revision_v1(
        &context.proposal,
        &context.project_policy,
        &context.provider_trust,
        provider,
    )
    .unwrap()
}

fn exact_revision(trusted: &ProjectPolicyTrustedReviewRevisionV1) -> &ReviewRevisionV1 {
    trusted.revision().revision().revision().revision()
}

fn snapshot(
    context: &Context,
    reviews: &[ProjectPolicyTrustedReviewRevisionV1],
) -> EvidenceBoundReviewHeadSnapshotV1 {
    let heads = reviews
        .iter()
        .map(|trusted| {
            let revision = exact_revision(trusted);
            ReviewHeadV1::new(
                revision.reviewer().clone(),
                revision.revision_id(DigestAlgorithm::Sha256).unwrap(),
                revision.sequence(),
            )
        })
        .collect();
    let claim = ReviewHeadSnapshotClaimV1::new(
        &context.proposal,
        digest(0x60),
        heads,
        DigestAlgorithm::Sha256,
    )
    .unwrap();
    let observation = ReviewHeadCompletenessObservationV1::new(
        claim.digest(DigestAlgorithm::Sha256).unwrap(),
        digest(0x61),
        ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal,
        digest(0x62),
        digest(0x63),
        digest(0x64),
    );
    bind_review_head_snapshot_observation_v1(claim, observation).unwrap()
}

fn witnessed_attestation(
    context: &Context,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    witness_byte: u8,
    marker: u8,
    observed_at: u64,
) -> mycelix_forge_review_head_witness::WitnessedReviewHeadAttestationV1 {
    let witness = principal(witness_byte);
    let statement = ReviewHeadCompletenessWitnessStatementV1::new(
        snapshot,
        witness.clone(),
        digest(marker),
    )
    .unwrap();
    let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
    let raw = raw_auth(
        context,
        witness,
        Capability::Witness,
        statement_id.commitment().clone(),
        marker.wrapping_add(30),
    );
    let trusted_auth = bind_project_policy_trusted_xenia_authentication_v1(
        &context.proposal,
        &context.project_policy,
        &context.provider_trust,
        &raw.authentication,
        &raw.provider_verified,
    )
    .unwrap();
    bind_witnessed_review_head_attestation_v1(
        &context.proposal,
        snapshot,
        statement,
        &trusted_auth,
        &context.authority,
        observed_at,
    )
    .unwrap()
}

fn checkpoint(
    context: &Context,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    checkpoint_time: u64,
) -> WitnessQualifiedReviewHeadSnapshotV1 {
    let a = witnessed_attestation(context, snapshot, 0x30, 0x70, checkpoint_time - 100);
    let b = witnessed_attestation(context, snapshot, 0x31, 0x71, checkpoint_time - 50);
    evaluate_review_head_witness_quorum_v1(
        &context.proposal,
        snapshot,
        &context.authority,
        &[b, a],
        checkpoint_time,
    )
    .unwrap()
}

#[test]
fn exact_trusted_revision_set_projects_authenticated_checkpoint_decisions() {
    let context = context();
    let approve = trusted_review(
        &context,
        0x20,
        ReviewRevisionDecision::Approve,
        0x80,
        1_300,
    );
    let changes = trusted_review(
        &context,
        0x21,
        ReviewRevisionDecision::RequestChanges,
        0x81,
        1_350,
    );
    let reviews = vec![changes, approve];
    let snapshot = snapshot(&context, &reviews);
    let checkpoint = checkpoint(&context, &snapshot, 1_500);

    let projected = project_witness_qualified_checkpoint_review_set_v1(
        &context.proposal,
        &snapshot,
        &checkpoint,
        &reviews,
    )
    .unwrap();

    assert_eq!(projected.reviews().len(), 2);
    assert_eq!(projected.reviews()[0].reviewer(), &principal(0x20));
    assert_eq!(projected.reviews()[0].decision(), ReviewRevisionDecision::Approve);
    assert_eq!(projected.reviews()[1].reviewer(), &principal(0x21));
    assert_eq!(
        projected.reviews()[1].decision(),
        ReviewRevisionDecision::RequestChanges
    );
}

#[test]
fn missing_trusted_revision_fails_closed() {
    let context = context();
    let a = trusted_review(
        &context,
        0x20,
        ReviewRevisionDecision::Approve,
        0x82,
        1_300,
    );
    let b = trusted_review(
        &context,
        0x21,
        ReviewRevisionDecision::Approve,
        0x83,
        1_300,
    );
    let snapshot_reviews = vec![a.clone(), b];
    let snapshot = snapshot(&context, &snapshot_reviews);
    let checkpoint = checkpoint(&context, &snapshot, 1_500);

    assert_eq!(
        project_witness_qualified_checkpoint_review_set_v1(
            &context.proposal,
            &snapshot,
            &checkpoint,
            &[a],
        )
        .unwrap_err(),
        CheckpointReviewSetError::RevisionCountMismatch {
            heads: 2,
            revisions: 1,
        }
    );
}

#[test]
fn sibling_revision_cannot_substitute_for_witnessed_head() {
    let context = context();
    let witnessed = trusted_review(
        &context,
        0x20,
        ReviewRevisionDecision::Approve,
        0x84,
        1_300,
    );
    let substitute = trusted_review(
        &context,
        0x20,
        ReviewRevisionDecision::RequestChanges,
        0x85,
        1_300,
    );
    let snapshot = snapshot(&context, &[witnessed]);
    let checkpoint = checkpoint(&context, &snapshot, 1_500);

    assert_eq!(
        project_witness_qualified_checkpoint_review_set_v1(
            &context.proposal,
            &snapshot,
            &checkpoint,
            &[substitute],
        )
        .unwrap_err(),
        CheckpointReviewSetError::RevisionIdentityMismatch(principal(0x20))
    );
}

#[test]
fn revision_observed_after_checkpoint_cannot_be_inserted_retroactively() {
    let context = context();
    let late = trusted_review(
        &context,
        0x20,
        ReviewRevisionDecision::Approve,
        0x86,
        1_600,
    );
    let snapshot = snapshot(&context, &[late.clone()]);
    let checkpoint = checkpoint(&context, &snapshot, 1_500);

    assert_eq!(
        project_witness_qualified_checkpoint_review_set_v1(
            &context.proposal,
            &snapshot,
            &checkpoint,
            &[late],
        )
        .unwrap_err(),
        CheckpointReviewSetError::RevisionObservedAfterCheckpoint {
            reviewer: principal(0x20),
            revision_observed_at: 1_600,
            checkpoint_observed_at: 1_500,
        }
    );
}
