use std::collections::BTreeSet;

use sha2::{Digest, Sha256};
use time::{OffsetDateTime, format_description::well_known::Rfc3339};

use crate::{
    AuthenticationFreshnessPolicyV1, GitObjectIdV1, GitHubParsedVerificationCandidateV1,
    GitHubResolvedTrustInstanceStatusV1, GitHubTrustedBuilderBindingV1,
    GitHubTrustedBuilderPolicyV1, GitHubVerifiedTimestampWitnessKindV1,
    QualificationReceiptCanonicalizationV1, QualificationResultV1,
    ReceiptAuthenticationPolicyV1, Sha256DigestV1, TransparencyPolicyV1,
    VerifierProcessOutcomeV1, VerifierTrustRootModeV1, WorkflowRevisionPolicyV1,
};

pub const GITHUB_AUTHENTICATION_CURRENTNESS_PROFILE_V1: &str =
    "mycelix-github-authentication-currentness-v1";
pub const AUTHENTICATION_POLICY_DIGEST_PROFILE_V1: &str =
    "mycelix-receipt-authentication-policy-digest-v1";
pub const TRUSTED_BUILDER_POLICY_DIGEST_PROFILE_V1: &str =
    "mycelix-github-trusted-builder-policy-digest-v1";
pub const AUTHENTICATION_EVIDENCE_DIGEST_PROFILE_V1: &str =
    "mycelix-github-authentication-evidence-digest-v1";
pub const VERIFIED_WITNESS_TIME_PROFILE_V1: &str = "rfc3339-public-good-tlog-v1";

const AUTH_POLICY_DIGEST_DOMAIN_V1: &[u8] =
    b"mycelix:receipt-authentication-policy:digest:v1\0";
const BUILDER_POLICY_DIGEST_DOMAIN_V1: &[u8] =
    b"mycelix:github-trusted-builder-policy:digest:v1\0";
const AUTH_EVIDENCE_DIGEST_DOMAIN_V1: &[u8] =
    b"mycelix:github-authentication-evidence:digest:v1\0";
const NANOS_PER_SECOND: i128 = 1_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubAuthenticationCurrentnessAuthorityV1 {
    CurrentnessEvidenceOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubAuthenticationCurrentnessErrorV1 {
    InvalidExecutionReceipt,
    ProcessDidNotSucceed,
    WrongTrustRootMode,
    CurrentTrustMaterialNotAcquiredInsideExecution,
    NoQualifyingPublicGoodTimestamp,
    MalformedVerifiedTimestamp { uri: String },
    TimestampBeforeUnixEpoch { uri: String },
    TimestampAfterExecution { uri: String },
    MaximumAgeExceeded { maximum: u64, actual: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubVerifiedWitnessTimeV1 {
    profile_id: &'static str,
    uri: String,
    raw_timestamp: String,
    unix_seconds: u64,
    nanosecond: u32,
}

impl GitHubVerifiedWitnessTimeV1 {
    pub const fn profile_id(&self) -> &'static str {
        self.profile_id
    }

    pub fn uri(&self) -> &str {
        &self.uri
    }

    pub fn raw_timestamp(&self) -> &str {
        &self.raw_timestamp
    }

    pub const fn unix_seconds(&self) -> u64 {
        self.unix_seconds
    }

    pub const fn nanosecond(&self) -> u32 {
        self.nanosecond
    }
}

/// Non-authoritative currentness and age evidence derived from one exact trusted-builder
/// binding. This type deliberately cannot mint or deserialize receipt-authentication
/// authority.
#[derive(Clone, Debug, PartialEq)]
pub struct GitHubAuthenticationCurrentnessV1 {
    profile_id: &'static str,
    trusted_builder_binding: GitHubTrustedBuilderBindingV1,
    evaluated_at_unix_seconds: u64,
    current_trust_material_at_verification: bool,
    conservative_witness_time: Option<GitHubVerifiedWitnessTimeV1>,
    age_seconds: Option<u64>,
    authentication_policy_digest: Sha256DigestV1,
    trusted_builder_policy_digest: Sha256DigestV1,
    authentication_evidence_digest: Sha256DigestV1,
    authority: GitHubAuthenticationCurrentnessAuthorityV1,
}

impl GitHubAuthenticationCurrentnessV1 {
    pub const fn profile_id(&self) -> &'static str {
        self.profile_id
    }

    pub fn trusted_builder_binding(&self) -> &GitHubTrustedBuilderBindingV1 {
        &self.trusted_builder_binding
    }

    pub const fn evaluated_at_unix_seconds(&self) -> u64 {
        self.evaluated_at_unix_seconds
    }

    pub const fn current_trust_material_at_verification(&self) -> bool {
        self.current_trust_material_at_verification
    }

    pub fn conservative_witness_time(&self) -> Option<&GitHubVerifiedWitnessTimeV1> {
        self.conservative_witness_time.as_ref()
    }

    pub const fn age_seconds(&self) -> Option<u64> {
        self.age_seconds
    }

    pub const fn authentication_policy_digest(&self) -> Sha256DigestV1 {
        self.authentication_policy_digest
    }

    pub const fn trusted_builder_policy_digest(&self) -> Sha256DigestV1 {
        self.trusted_builder_policy_digest
    }

    pub const fn authentication_evidence_digest(&self) -> Sha256DigestV1 {
        self.authentication_evidence_digest
    }

    pub const fn authority_scope(&self) -> GitHubAuthenticationCurrentnessAuthorityV1 {
        self.authority
    }

    pub const fn establishes_current_trust_material_at_verification(&self) -> bool {
        self.current_trust_material_at_verification
    }

    pub const fn establishes_attestation_age(&self) -> bool {
        self.age_seconds.is_some()
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }
}

/// Canonical identity of the backend-neutral authentication policy.
///
/// This function computes identity only; callers still need the policy's own validation
/// and an admitted verifier path before the digest carries authority.
pub fn authentication_policy_digest_v1(policy: &ReceiptAuthenticationPolicyV1) -> Sha256DigestV1 {
    let mut hasher = Sha256::new();
    hasher.update(AUTH_POLICY_DIGEST_DOMAIN_V1);
    put_str(&mut hasher, AUTHENTICATION_POLICY_DIGEST_PROFILE_V1);
    put_str(&mut hasher, &policy.profile_id);
    put_str(&mut hasher, &policy.verifier_profile.profile_id);
    put_str(&mut hasher, &policy.verifier_profile.backend_family);
    put_str(&mut hasher, &policy.verifier_profile.backend_version);
    put_str(&mut hasher, &policy.verifier_profile.verification_profile);
    put_str(&mut hasher, &policy.expected_attestation_subject_name);
    put_str(&mut hasher, &policy.expected_predicate_type);
    put_str(&mut hasher, &policy.expected_predicate_schema);
    put_str(&mut hasher, &policy.trusted_root_profile);
    put_digest(&mut hasher, policy.trusted_root_digest);
    put_str(&mut hasher, &policy.oidc_issuer);
    put_str(&mut hasher, &policy.source_repository);
    put_str(&mut hasher, &policy.source_repository_owner);
    put_str(&mut hasher, &policy.signer_workflow);
    put_workflow_revision_policy(&mut hasher, &policy.signer_workflow_revision);
    put_git_object_id(&mut hasher, policy.source_revision.exact_commit);
    put_optional_str(&mut hasher, policy.source_revision.exact_git_ref.as_deref());
    put_transparency_policy(&mut hasher, policy.transparency_policy);
    put_freshness_policy(&mut hasher, policy.freshness_policy);
    Sha256DigestV1::from_bytes(hasher.finalize().into())
}

/// Canonical identity of the GitHub-specific trusted-builder policy.
pub fn trusted_builder_policy_digest_v1(
    policy: &GitHubTrustedBuilderPolicyV1,
) -> Sha256DigestV1 {
    let mut hasher = Sha256::new();
    hasher.update(BUILDER_POLICY_DIGEST_DOMAIN_V1);
    put_str(&mut hasher, TRUSTED_BUILDER_POLICY_DIGEST_PROFILE_V1);
    put_str(&mut hasher, policy.profile_id());
    put_str(&mut hasher, policy.source_repository_identifier());
    put_str(&mut hasher, policy.source_repository_owner_identifier());
    put_str(&mut hasher, policy.expected_statement_type());
    put_str(&mut hasher, policy.required_runner_environment());
    put_str(&mut hasher, policy.required_repository_visibility());
    put_bool(&mut hasher, policy.require_build_config_identity());
    put_bool(&mut hasher, policy.require_run_invocation_identity());
    Sha256DigestV1::from_bytes(hasher.finalize().into())
}

/// Evaluate currentness using only the exact verification event retained by the binding.
/// No wall-clock value is accepted from the caller.
pub fn evaluate_github_authentication_currentness_v1(
    binding: &GitHubTrustedBuilderBindingV1,
) -> Result<GitHubAuthenticationCurrentnessV1, GitHubAuthenticationCurrentnessErrorV1> {
    let classification = binding.root_classification();
    let execution = classification.execution_receipt();
    execution
        .validate()
        .map_err(|_| GitHubAuthenticationCurrentnessErrorV1::InvalidExecutionReceipt)?;
    if !execution.process_exited_successfully() {
        return Err(GitHubAuthenticationCurrentnessErrorV1::ProcessDidNotSucceed);
    }
    if execution.trust_root_mode != VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 {
        return Err(GitHubAuthenticationCurrentnessErrorV1::WrongTrustRootMode);
    }
    if execution.trusted_root_acquired_at_unix_seconds < execution.execution_started_at_unix_seconds
        || execution.trusted_root_acquired_at_unix_seconds
            > execution.execution_completed_at_unix_seconds
    {
        return Err(
            GitHubAuthenticationCurrentnessErrorV1::CurrentTrustMaterialNotAcquiredInsideExecution,
        );
    }

    let authentication_policy = binding.authentication_policy();
    let (conservative_witness_time, age_seconds) = match authentication_policy.freshness_policy {
        AuthenticationFreshnessPolicyV1::CurrentAtVerification => (None, None),
        AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(maximum) => {
            let witness = oldest_qualifying_public_good_witness(
                binding,
                execution.execution_completed_at_unix_seconds,
            )?;
            let age = conservative_age_seconds(
                &witness,
                execution.execution_completed_at_unix_seconds,
            )?;
            if age > maximum {
                return Err(GitHubAuthenticationCurrentnessErrorV1::MaximumAgeExceeded {
                    maximum,
                    actual: age,
                });
            }
            (Some(witness), Some(age))
        }
    };

    let authentication_policy_digest = authentication_policy_digest_v1(authentication_policy);
    let trusted_builder_policy_digest =
        trusted_builder_policy_digest_v1(binding.trusted_builder_policy());
    let authentication_evidence_digest = authentication_evidence_digest_v1(
        binding,
        authentication_policy_digest,
        trusted_builder_policy_digest,
        conservative_witness_time.as_ref(),
        age_seconds,
    );

    Ok(GitHubAuthenticationCurrentnessV1 {
        profile_id: GITHUB_AUTHENTICATION_CURRENTNESS_PROFILE_V1,
        trusted_builder_binding: binding.clone(),
        evaluated_at_unix_seconds: execution.execution_completed_at_unix_seconds,
        current_trust_material_at_verification: true,
        conservative_witness_time,
        age_seconds,
        authentication_policy_digest,
        trusted_builder_policy_digest,
        authentication_evidence_digest,
        authority: GitHubAuthenticationCurrentnessAuthorityV1::CurrentnessEvidenceOnly,
    })
}

fn oldest_qualifying_public_good_witness(
    binding: &GitHubTrustedBuilderBindingV1,
    execution_completed_at_unix_seconds: u64,
) -> Result<GitHubVerifiedWitnessTimeV1, GitHubAuthenticationCurrentnessErrorV1> {
    let accepted_uris: BTreeSet<&str> = binding
        .transparency_resolution()
        .matched_public_good_tlog_uris()
        .iter()
        .map(String::as_str)
        .collect();
    if accepted_uris.is_empty()
        || binding.transparency_resolution().status()
            != GitHubResolvedTrustInstanceStatusV1::PublicGoodTransparencyConfirmed
    {
        return Err(GitHubAuthenticationCurrentnessErrorV1::NoQualifyingPublicGoodTimestamp);
    }

    let completion_nanos = i128::from(execution_completed_at_unix_seconds) * NANOS_PER_SECOND;
    let mut oldest: Option<(i128, GitHubVerifiedWitnessTimeV1)> = None;
    for witness in binding.selected_candidate().verified_timestamps() {
        if witness.kind() != GitHubVerifiedTimestampWitnessKindV1::TransparencyLog
            || !accepted_uris.contains(witness.uri())
        {
            continue;
        }
        let parsed = parse_verified_witness_time(witness.uri(), witness.timestamp())?;
        let nanos = witness_time_nanos(&parsed);
        if nanos > completion_nanos {
            return Err(GitHubAuthenticationCurrentnessErrorV1::TimestampAfterExecution {
                uri: parsed.uri.clone(),
            });
        }
        if oldest.as_ref().is_none_or(|(current, _)| nanos < *current) {
            oldest = Some((nanos, parsed));
        }
    }

    oldest
        .map(|(_, witness)| witness)
        .ok_or(GitHubAuthenticationCurrentnessErrorV1::NoQualifyingPublicGoodTimestamp)
}

fn parse_verified_witness_time(
    uri: &str,
    raw_timestamp: &str,
) -> Result<GitHubVerifiedWitnessTimeV1, GitHubAuthenticationCurrentnessErrorV1> {
    let parsed = OffsetDateTime::parse(raw_timestamp, &Rfc3339).map_err(|_| {
        GitHubAuthenticationCurrentnessErrorV1::MalformedVerifiedTimestamp {
            uri: uri.to_owned(),
        }
    })?;
    let unix_seconds = parsed.unix_timestamp();
    if unix_seconds < 0 {
        return Err(GitHubAuthenticationCurrentnessErrorV1::TimestampBeforeUnixEpoch {
            uri: uri.to_owned(),
        });
    }
    Ok(GitHubVerifiedWitnessTimeV1 {
        profile_id: VERIFIED_WITNESS_TIME_PROFILE_V1,
        uri: uri.to_owned(),
        raw_timestamp: raw_timestamp.to_owned(),
        unix_seconds: unix_seconds as u64,
        nanosecond: parsed.nanosecond(),
    })
}

fn conservative_age_seconds(
    witness: &GitHubVerifiedWitnessTimeV1,
    execution_completed_at_unix_seconds: u64,
) -> Result<u64, GitHubAuthenticationCurrentnessErrorV1> {
    let witness_nanos = witness_time_nanos(witness);
    let completion_nanos = i128::from(execution_completed_at_unix_seconds) * NANOS_PER_SECOND;
    if witness_nanos > completion_nanos {
        return Err(GitHubAuthenticationCurrentnessErrorV1::TimestampAfterExecution {
            uri: witness.uri.clone(),
        });
    }
    let delta = completion_nanos - witness_nanos;
    let age = (delta + NANOS_PER_SECOND - 1) / NANOS_PER_SECOND;
    Ok(age as u64)
}

fn witness_time_nanos(witness: &GitHubVerifiedWitnessTimeV1) -> i128 {
    i128::from(witness.unix_seconds) * NANOS_PER_SECOND + i128::from(witness.nanosecond)
}

fn authentication_evidence_digest_v1(
    binding: &GitHubTrustedBuilderBindingV1,
    authentication_policy_digest: Sha256DigestV1,
    trusted_builder_policy_digest: Sha256DigestV1,
    conservative_witness: Option<&GitHubVerifiedWitnessTimeV1>,
    age_seconds: Option<u64>,
) -> Sha256DigestV1 {
    let mut hasher = Sha256::new();
    hasher.update(AUTH_EVIDENCE_DIGEST_DOMAIN_V1);
    put_str(&mut hasher, AUTHENTICATION_EVIDENCE_DIGEST_PROFILE_V1);
    put_str(&mut hasher, GITHUB_AUTHENTICATION_CURRENTNESS_PROFILE_V1);
    put_digest(&mut hasher, binding.receipt_digest().sha256);
    put_u8(
        &mut hasher,
        match binding.receipt_digest().canonicalization {
            QualificationReceiptCanonicalizationV1::BinaryV1 => 1,
        },
    );
    put_digest(&mut hasher, authentication_policy_digest);
    put_digest(&mut hasher, trusted_builder_policy_digest);

    let classification = binding.root_classification();
    put_str(&mut hasher, classification.profile_id());
    put_str(&mut hasher, classification.retained_root_digest_profile_id());
    put_digest(&mut hasher, classification.retained_root_digest());
    put_digest(&mut hasher, classification.public_good_root_digest());
    put_digest(&mut hasher, classification.github_private_root_digest());
    put_sorted_strings(&mut hasher, classification.public_good_tlog_uris());
    put_sorted_strings(&mut hasher, classification.public_good_ca_uris());
    put_sorted_strings(&mut hasher, classification.github_private_tlog_uris());
    put_sorted_strings(&mut hasher, classification.github_private_ca_uris());
    put_str(&mut hasher, classification.execution_policy().profile_id());
    put_str(
        &mut hasher,
        classification.execution_policy().authentication_policy_id(),
    );
    put_str(
        &mut hasher,
        classification.execution_policy().executable_path(),
    );

    let command_plan = classification.command_plan();
    put_u64(&mut hasher, command_plan.timeout_seconds());
    put_u64(&mut hasher, command_plan.max_stdout_bytes() as u64);
    put_u64(&mut hasher, command_plan.max_stderr_bytes() as u64);
    put_u32(&mut hasher, command_plan.expected_max_results());

    let execution = classification.execution_receipt();
    put_u32(&mut hasher, execution.evidence_version);
    put_str(&mut hasher, &execution.verifier.profile_id);
    put_str(&mut hasher, &execution.verifier.backend_family);
    put_str(&mut hasher, &execution.verifier.semantic_version);
    put_digest(&mut hasher, execution.verifier.executable_sha256);
    put_str(&mut hasher, &execution.verifier.platform_profile);
    put_str(&mut hasher, &execution.verifier.command_profile_id);
    match &execution.verifier.nix_closure {
        Some(nix) => {
            put_u8(&mut hasher, 1);
            put_str(&mut hasher, &nix.store_path);
            put_digest(&mut hasher, nix.closure_digest);
        }
        None => put_u8(&mut hasher, 0),
    }
    put_digest(&mut hasher, execution.canonical_receipt_digest.sha256);
    put_digest(&mut hasher, execution.attestation_bundle_digest);
    put_u8(
        &mut hasher,
        match execution.trust_root_mode {
            VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 => 1,
            VerifierTrustRootModeV1::OfflinePinnedRootV1 => 2,
        },
    );
    put_digest(&mut hasher, execution.trusted_root_material_digest);
    put_u64(&mut hasher, execution.trusted_root_acquired_at_unix_seconds);
    put_digest(&mut hasher, execution.command_arguments_digest);
    put_digest(&mut hasher, execution.environment_profile_digest);
    put_digest(&mut hasher, execution.verifier_stdout_digest);
    put_digest(&mut hasher, execution.verifier_stderr_digest);
    put_u64(&mut hasher, execution.execution_started_at_unix_seconds);
    put_u64(&mut hasher, execution.execution_completed_at_unix_seconds);
    match execution.process_outcome {
        VerifierProcessOutcomeV1::ExitCode(code) => {
            put_u8(&mut hasher, 1);
            put_i32(&mut hasher, code);
        }
        VerifierProcessOutcomeV1::TerminatedBySignal(signal) => {
            put_u8(&mut hasher, 2);
            put_i32(&mut hasher, signal);
        }
        VerifierProcessOutcomeV1::TimedOut => put_u8(&mut hasher, 3),
    }
    put_u32(&mut hasher, execution.parsed_result_count);

    put_u64(&mut hasher, binding.selected_candidate_index() as u64);
    put_git_object_id(&mut hasher, binding.selected_signer_revision());
    put_git_object_id(&mut hasher, binding.selected_source_revision());
    match binding.selected_run_identity() {
        Some(run) => {
            put_u8(&mut hasher, 1);
            put_u64(&mut hasher, run.run_id());
            put_u64(&mut hasher, run.run_attempt());
        }
        None => put_u8(&mut hasher, 0),
    }

    let predicate = binding.predicate();
    put_u32(&mut hasher, predicate.predicate_version);
    put_str(&mut hasher, &predicate.predicate_schema);
    put_digest(&mut hasher, predicate.receipt_digest.sha256);
    put_str(&mut hasher, &predicate.qualification_profile);
    put_git_object_id(&mut hasher, predicate.subject);
    put_digest(&mut hasher, predicate.coherence_result_digest);
    put_qualification_result(&mut hasher, predicate.result);

    put_selected_candidate(&mut hasher, binding.selected_candidate());
    put_transparency_resolution(&mut hasher, binding);

    put_u64(
        &mut hasher,
        classification
            .execution_receipt()
            .execution_completed_at_unix_seconds,
    );
    put_bool(&mut hasher, true);
    match conservative_witness {
        Some(witness) => {
            put_u8(&mut hasher, 1);
            put_str(&mut hasher, witness.profile_id());
            put_str(&mut hasher, witness.uri());
            put_str(&mut hasher, witness.raw_timestamp());
            put_u64(&mut hasher, witness.unix_seconds());
            put_u32(&mut hasher, witness.nanosecond());
        }
        None => put_u8(&mut hasher, 0),
    }
    put_optional_u64(&mut hasher, age_seconds);

    Sha256DigestV1::from_bytes(hasher.finalize().into())
}

fn put_selected_candidate(hasher: &mut Sha256, candidate: &GitHubParsedVerificationCandidateV1) {
    let certificate = candidate.certificate();
    put_str(hasher, certificate.certificate_issuer());
    put_str(hasher, certificate.subject_alternative_name());
    for value in [
        certificate.oidc_issuer(),
        certificate.build_signer_uri(),
        certificate.build_signer_digest(),
        certificate.runner_environment(),
        certificate.source_repository_uri(),
        certificate.source_repository_digest(),
        certificate.source_repository_ref(),
        certificate.source_repository_identifier(),
        certificate.source_repository_owner_uri(),
        certificate.source_repository_owner_identifier(),
        certificate.build_config_uri(),
        certificate.build_config_digest(),
        certificate.build_trigger(),
        certificate.run_invocation_uri(),
        certificate.source_repository_visibility_at_signing(),
    ] {
        put_optional_str(hasher, value);
    }
    put_str(hasher, candidate.statement_type());
    put_u64(hasher, candidate.subjects().len() as u64);
    for subject in candidate.subjects() {
        put_str(hasher, subject.name());
        put_digest(hasher, subject.sha256());
    }
    put_str(hasher, candidate.predicate().predicate_type());
    put_u64(hasher, candidate.verified_timestamps().len() as u64);
    for timestamp in candidate.verified_timestamps() {
        put_u8(
            hasher,
            match timestamp.kind() {
                GitHubVerifiedTimestampWitnessKindV1::TransparencyLog => 1,
                GitHubVerifiedTimestampWitnessKindV1::TimestampAuthority => 2,
                GitHubVerifiedTimestampWitnessKindV1::OtherVerifiedWitness => 3,
            },
        );
        put_str(hasher, timestamp.raw_type());
        put_str(hasher, timestamp.uri());
        put_str(hasher, timestamp.timestamp());
    }
}

fn put_transparency_resolution(hasher: &mut Sha256, binding: &GitHubTrustedBuilderBindingV1) {
    let resolution = binding.transparency_resolution();
    put_u8(
        hasher,
        match resolution.status() {
            GitHubResolvedTrustInstanceStatusV1::PublicGoodTransparencyConfirmed => 1,
            GitHubResolvedTrustInstanceStatusV1::GitHubPrivateTransparencyOnly => 2,
            GitHubResolvedTrustInstanceStatusV1::TimestampAuthorityOnly => 3,
            GitHubResolvedTrustInstanceStatusV1::NoVerifiedTimestamps => 4,
            GitHubResolvedTrustInstanceStatusV1::UnresolvedTrustInstance => 5,
            GitHubResolvedTrustInstanceStatusV1::MixedTrustDomains => 6,
        },
    );
    put_sorted_strings(hasher, resolution.matched_public_good_tlog_uris());
    put_sorted_strings(hasher, resolution.matched_github_private_tlog_uris());
    put_sorted_strings(hasher, resolution.unresolved_tlog_uris());
    put_digest(hasher, resolution.retained_root_digest());
}

fn put_workflow_revision_policy(hasher: &mut Sha256, policy: &WorkflowRevisionPolicyV1) {
    match policy {
        WorkflowRevisionPolicyV1::Exact(revision) => {
            put_u8(hasher, 1);
            put_git_object_id(hasher, *revision);
        }
        WorkflowRevisionPolicyV1::Allowed(revisions) => {
            put_u8(hasher, 2);
            let mut canonical: Vec<String> = revisions.iter().map(GitObjectIdV1::to_wire).collect();
            canonical.sort();
            put_u64(hasher, canonical.len() as u64);
            for revision in canonical {
                put_str(hasher, &revision);
            }
        }
    }
}

fn put_transparency_policy(hasher: &mut Sha256, policy: TransparencyPolicyV1) {
    put_u8(
        hasher,
        match policy {
            TransparencyPolicyV1::PublicTransparencyRequired => 1,
            TransparencyPolicyV1::TimestampRequired => 2,
            TransparencyPolicyV1::PublicTransparencyAndTimestampRequired => 3,
        },
    );
}

fn put_freshness_policy(hasher: &mut Sha256, policy: AuthenticationFreshnessPolicyV1) {
    match policy {
        AuthenticationFreshnessPolicyV1::CurrentAtVerification => put_u8(hasher, 1),
        AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(seconds) => {
            put_u8(hasher, 2);
            put_u64(hasher, seconds);
        }
    }
}

fn put_qualification_result(hasher: &mut Sha256, result: QualificationResultV1) {
    put_u8(
        hasher,
        match result {
            QualificationResultV1::Pass => 1,
            QualificationResultV1::Fail => 2,
            QualificationResultV1::RecordedOnly => 3,
        },
    );
}

fn put_sorted_strings(hasher: &mut Sha256, values: &[String]) {
    let mut values = values.to_vec();
    values.sort();
    put_u64(hasher, values.len() as u64);
    for value in values {
        put_str(hasher, &value);
    }
}

fn put_git_object_id(hasher: &mut Sha256, value: GitObjectIdV1) {
    put_str(hasher, &value.to_wire());
}

fn put_digest(hasher: &mut Sha256, digest: Sha256DigestV1) {
    hasher.update(digest.as_bytes());
}

fn put_str(hasher: &mut Sha256, value: &str) {
    put_u64(hasher, value.len() as u64);
    hasher.update(value.as_bytes());
}

fn put_optional_str(hasher: &mut Sha256, value: Option<&str>) {
    match value {
        Some(value) => {
            put_u8(hasher, 1);
            put_str(hasher, value);
        }
        None => put_u8(hasher, 0),
    }
}

fn put_optional_u64(hasher: &mut Sha256, value: Option<u64>) {
    match value {
        Some(value) => {
            put_u8(hasher, 1);
            put_u64(hasher, value);
        }
        None => put_u8(hasher, 0),
    }
}

fn put_bool(hasher: &mut Sha256, value: bool) {
    put_u8(hasher, u8::from(value));
}

fn put_u8(hasher: &mut Sha256, value: u8) {
    hasher.update([value]);
}

fn put_u32(hasher: &mut Sha256, value: u32) {
    hasher.update(value.to_be_bytes());
}

fn put_u64(hasher: &mut Sha256, value: u64) {
    hasher.update(value.to_be_bytes());
}

fn put_i32(hasher: &mut Sha256, value: i32) {
    hasher.update(value.to_be_bytes());
}
