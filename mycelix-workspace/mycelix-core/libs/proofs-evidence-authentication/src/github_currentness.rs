use std::collections::BTreeSet;

use chrono::DateTime;
use sha2::{Digest, Sha256};

use crate::{
    AuthenticationFreshnessPolicyV1, GitHubResolvedTrustInstanceStatusV1,
    GitHubTrustedBuilderBindingV1, GitHubTrustedBuilderPolicyV1,
    GitHubVerifiedTimestampEvidenceV1, GitHubVerifiedTimestampWitnessKindV1,
    QualificationAttestationPredicateV1, QualificationResultV1,
    ReceiptAuthenticationPolicyV1, Sha256DigestV1, TransparencyPolicyV1,
    VerifierProcessOutcomeV1, VerifierTrustRootModeV1, WorkflowRevisionPolicyV1,
};

pub const RECEIPT_AUTHENTICATION_POLICY_DIGEST_PROFILE_V1: &str =
    "mycelix-receipt-authentication-policy-digest-v1";
pub const GITHUB_TRUSTED_BUILDER_POLICY_DIGEST_PROFILE_V1: &str =
    "mycelix-github-trusted-builder-policy-digest-v1";
pub const GITHUB_AUTHENTICATION_CURRENTNESS_PROFILE_V1: &str =
    "mycelix-github-authentication-currentness-v1";
pub const GITHUB_AUTHENTICATION_EVIDENCE_DIGEST_PROFILE_V1: &str =
    "mycelix-github-authentication-evidence-digest-v1";

const AUTH_POLICY_DOMAIN_V1: &[u8] = b"mycelix:receipt-authentication-policy:v1\0";
const TRUSTED_BUILDER_POLICY_DOMAIN_V1: &[u8] =
    b"mycelix:github-trusted-builder-policy:v1\0";
const AUTH_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix:github-authentication-evidence:v1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubAuthenticationCurrentnessAuthorityV1 {
    CurrentnessEvidenceOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubAuthenticationCurrentnessErrorV1 {
    InvalidAuthenticationPolicy,
    InvalidTrustedBuilderPolicy,
    InvalidExecutionReceipt,
    ProcessDidNotSucceed,
    WrongTrustRootMode,
    TrustedRootDigestMismatch,
    MissingQualifiedWitness,
    MalformedQualifiedWitnessTimestamp { uri: String },
    QualifiedWitnessBeforeUnixEpoch { uri: String },
    QualifiedWitnessInFuture {
        uri: String,
        witness_unix_seconds: u64,
        evaluated_at_unix_seconds: u64,
    },
    MaximumAgeExceeded { maximum: u64, actual: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubQualifiedWitnessTimeV1 {
    kind: GitHubVerifiedTimestampWitnessKindV1,
    uri: String,
    raw_timestamp: String,
    unix_seconds: u64,
}

impl GitHubQualifiedWitnessTimeV1 {
    pub const fn kind(&self) -> GitHubVerifiedTimestampWitnessKindV1 {
        self.kind
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
}

/// Opaque evidence that one already-bound GitHub trusted-builder attestation was
/// evaluated under current trust material and the exact freshness policy carried
/// by its authentication policy.
///
/// This is deliberately not serializable/deserializable authority and cannot mint
/// `AuthenticatedQualificationReceiptV1`. A later concrete backend must map this
/// sealed evidence into the crate-private verified capability parts.
#[derive(Clone, Debug, PartialEq)]
pub struct GitHubAuthenticationCurrentnessV1 {
    trusted_builder_binding: GitHubTrustedBuilderBindingV1,
    evaluated_at_unix_seconds: u64,
    current_trust_material_at_verification: bool,
    conservative_witness_time: Option<GitHubQualifiedWitnessTimeV1>,
    age_seconds: Option<u64>,
    authentication_policy_digest: Sha256DigestV1,
    trusted_builder_policy_digest: Sha256DigestV1,
    authentication_evidence_digest: Sha256DigestV1,
    authority: GitHubAuthenticationCurrentnessAuthorityV1,
}

impl GitHubAuthenticationCurrentnessV1 {
    pub fn trusted_builder_binding(&self) -> &GitHubTrustedBuilderBindingV1 {
        &self.trusted_builder_binding
    }

    pub const fn evaluated_at_unix_seconds(&self) -> u64 {
        self.evaluated_at_unix_seconds
    }

    pub const fn current_trust_material_at_verification(&self) -> bool {
        self.current_trust_material_at_verification
    }

    pub fn conservative_witness_time(&self) -> Option<&GitHubQualifiedWitnessTimeV1> {
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

    pub const fn establishes_currentness_evidence(&self) -> bool {
        true
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

pub fn receipt_authentication_policy_digest_v1(
    policy: &ReceiptAuthenticationPolicyV1,
) -> Result<Sha256DigestV1, GitHubAuthenticationCurrentnessErrorV1> {
    policy
        .validate()
        .map_err(|_| GitHubAuthenticationCurrentnessErrorV1::InvalidAuthenticationPolicy)?;

    let mut out = Vec::with_capacity(1024);
    out.extend_from_slice(AUTH_POLICY_DOMAIN_V1);
    put_str(&mut out, RECEIPT_AUTHENTICATION_POLICY_DIGEST_PROFILE_V1);
    put_str(&mut out, &policy.profile_id);
    put_str(&mut out, &policy.verifier_profile.profile_id);
    put_str(&mut out, &policy.verifier_profile.backend_family);
    put_str(&mut out, &policy.verifier_profile.backend_version);
    put_str(&mut out, &policy.verifier_profile.verification_profile);
    put_str(&mut out, &policy.expected_attestation_subject_name);
    put_str(&mut out, &policy.expected_predicate_type);
    put_str(&mut out, &policy.expected_predicate_schema);
    put_str(&mut out, &policy.trusted_root_profile);
    put_digest(&mut out, policy.trusted_root_digest);
    put_str(&mut out, &policy.oidc_issuer);
    put_str(&mut out, &policy.source_repository);
    put_str(&mut out, &policy.source_repository_owner);
    put_str(&mut out, &policy.signer_workflow);
    put_workflow_revision_policy(&mut out, &policy.signer_workflow_revision);
    put_git_object_id(&mut out, policy.source_revision.exact_commit);
    put_opt_str(&mut out, policy.source_revision.exact_git_ref.as_deref());
    put_transparency_policy(&mut out, policy.transparency_policy);
    put_freshness_policy(&mut out, policy.freshness_policy);
    Ok(sha256(&out))
}

pub fn github_trusted_builder_policy_digest_v1(
    policy: &GitHubTrustedBuilderPolicyV1,
) -> Result<Sha256DigestV1, GitHubAuthenticationCurrentnessErrorV1> {
    policy
        .validate()
        .map_err(|_| GitHubAuthenticationCurrentnessErrorV1::InvalidTrustedBuilderPolicy)?;

    let mut out = Vec::with_capacity(512);
    out.extend_from_slice(TRUSTED_BUILDER_POLICY_DOMAIN_V1);
    put_str(&mut out, GITHUB_TRUSTED_BUILDER_POLICY_DIGEST_PROFILE_V1);
    put_str(&mut out, policy.profile_id());
    put_str(&mut out, policy.source_repository_identifier());
    put_str(&mut out, policy.source_repository_owner_identifier());
    put_str(&mut out, policy.expected_statement_type());
    put_str(&mut out, policy.required_runner_environment());
    put_str(&mut out, policy.required_repository_visibility());
    put_bool(&mut out, policy.require_build_config_identity());
    put_bool(&mut out, policy.require_run_invocation_identity());
    Ok(sha256(&out))
}

pub fn establish_github_authentication_currentness_v1(
    binding: &GitHubTrustedBuilderBindingV1,
) -> Result<GitHubAuthenticationCurrentnessV1, GitHubAuthenticationCurrentnessErrorV1> {
    let authentication_policy = binding.authentication_policy();
    authentication_policy
        .validate()
        .map_err(|_| GitHubAuthenticationCurrentnessErrorV1::InvalidAuthenticationPolicy)?;
    binding
        .trusted_builder_policy()
        .validate()
        .map_err(|_| GitHubAuthenticationCurrentnessErrorV1::InvalidTrustedBuilderPolicy)?;

    let execution = binding.root_classification().execution_receipt();
    execution
        .validate()
        .map_err(|_| GitHubAuthenticationCurrentnessErrorV1::InvalidExecutionReceipt)?;
    if !execution.process_exited_successfully() {
        return Err(GitHubAuthenticationCurrentnessErrorV1::ProcessDidNotSucceed);
    }
    if execution.trust_root_mode != VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 {
        return Err(GitHubAuthenticationCurrentnessErrorV1::WrongTrustRootMode);
    }
    if binding.root_classification().retained_root_digest()
        != authentication_policy.trusted_root_digest
        || execution.trusted_root_material_digest != authentication_policy.trusted_root_digest
    {
        return Err(GitHubAuthenticationCurrentnessErrorV1::TrustedRootDigestMismatch);
    }

    let evaluated_at_unix_seconds = execution.execution_completed_at_unix_seconds;
    let (conservative_witness_time, age_seconds) = match authentication_policy.freshness_policy {
        AuthenticationFreshnessPolicyV1::CurrentAtVerification => (None, None),
        AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(maximum) => {
            let witness = conservative_policy_qualified_witness(
                binding,
                evaluated_at_unix_seconds,
            )?;
            let age = evaluated_at_unix_seconds - witness.unix_seconds;
            if age > maximum {
                return Err(GitHubAuthenticationCurrentnessErrorV1::MaximumAgeExceeded {
                    maximum,
                    actual: age,
                });
            }
            (Some(witness), Some(age))
        }
    };

    let authentication_policy_digest =
        receipt_authentication_policy_digest_v1(authentication_policy)?;
    let trusted_builder_policy_digest =
        github_trusted_builder_policy_digest_v1(binding.trusted_builder_policy())?;
    let authentication_evidence_digest = authentication_evidence_digest_v1(
        binding,
        authentication_policy_digest,
        trusted_builder_policy_digest,
        evaluated_at_unix_seconds,
        conservative_witness_time.as_ref(),
        age_seconds,
    );

    Ok(GitHubAuthenticationCurrentnessV1 {
        trusted_builder_binding: binding.clone(),
        evaluated_at_unix_seconds,
        current_trust_material_at_verification: true,
        conservative_witness_time,
        age_seconds,
        authentication_policy_digest,
        trusted_builder_policy_digest,
        authentication_evidence_digest,
        authority: GitHubAuthenticationCurrentnessAuthorityV1::CurrentnessEvidenceOnly,
    })
}

fn conservative_policy_qualified_witness(
    binding: &GitHubTrustedBuilderBindingV1,
    evaluated_at_unix_seconds: u64,
) -> Result<GitHubQualifiedWitnessTimeV1, GitHubAuthenticationCurrentnessErrorV1> {
    let policy = binding.authentication_policy().transparency_policy;
    let public_good_uris: BTreeSet<&str> = binding
        .transparency_resolution()
        .matched_public_good_tlog_uris()
        .iter()
        .map(String::as_str)
        .collect();

    let mut qualified = Vec::new();
    for evidence in binding.selected_candidate().verified_timestamps() {
        if !witness_qualifies(policy, evidence, &public_good_uris) {
            continue;
        }
        let unix_seconds = parse_witness_time(evidence)?;
        if unix_seconds > evaluated_at_unix_seconds {
            return Err(GitHubAuthenticationCurrentnessErrorV1::QualifiedWitnessInFuture {
                uri: evidence.uri().to_string(),
                witness_unix_seconds: unix_seconds,
                evaluated_at_unix_seconds,
            });
        }
        qualified.push(GitHubQualifiedWitnessTimeV1 {
            kind: evidence.kind(),
            uri: evidence.uri().to_string(),
            raw_timestamp: evidence.timestamp().to_string(),
            unix_seconds,
        });
    }

    qualified
        .into_iter()
        .min_by(|left, right| {
            (
                left.unix_seconds,
                left.uri.as_str(),
                left.raw_timestamp.as_str(),
            )
                .cmp(&(
                    right.unix_seconds,
                    right.uri.as_str(),
                    right.raw_timestamp.as_str(),
                ))
        })
        .ok_or(GitHubAuthenticationCurrentnessErrorV1::MissingQualifiedWitness)
}

fn witness_qualifies(
    policy: TransparencyPolicyV1,
    evidence: &GitHubVerifiedTimestampEvidenceV1,
    public_good_uris: &BTreeSet<&str>,
) -> bool {
    match policy {
        TransparencyPolicyV1::PublicTransparencyRequired
        | TransparencyPolicyV1::PublicTransparencyAndTimestampRequired => {
            evidence.kind() == GitHubVerifiedTimestampWitnessKindV1::TransparencyLog
                && public_good_uris.contains(evidence.uri())
        }
        TransparencyPolicyV1::TimestampRequired => matches!(
            evidence.kind(),
            GitHubVerifiedTimestampWitnessKindV1::TransparencyLog
                | GitHubVerifiedTimestampWitnessKindV1::TimestampAuthority
        ),
    }
}

fn parse_witness_time(
    evidence: &GitHubVerifiedTimestampEvidenceV1,
) -> Result<u64, GitHubAuthenticationCurrentnessErrorV1> {
    let parsed = DateTime::parse_from_rfc3339(evidence.timestamp()).map_err(|_| {
        GitHubAuthenticationCurrentnessErrorV1::MalformedQualifiedWitnessTimestamp {
            uri: evidence.uri().to_string(),
        }
    })?;
    let seconds = parsed.timestamp();
    if seconds < 0 {
        return Err(
            GitHubAuthenticationCurrentnessErrorV1::QualifiedWitnessBeforeUnixEpoch {
                uri: evidence.uri().to_string(),
            },
        );
    }
    Ok(seconds as u64)
}

fn authentication_evidence_digest_v1(
    binding: &GitHubTrustedBuilderBindingV1,
    authentication_policy_digest: Sha256DigestV1,
    trusted_builder_policy_digest: Sha256DigestV1,
    evaluated_at_unix_seconds: u64,
    conservative_witness_time: Option<&GitHubQualifiedWitnessTimeV1>,
    age_seconds: Option<u64>,
) -> Sha256DigestV1 {
    let mut out = Vec::with_capacity(4096);
    out.extend_from_slice(AUTH_EVIDENCE_DOMAIN_V1);
    put_str(&mut out, GITHUB_AUTHENTICATION_EVIDENCE_DIGEST_PROFILE_V1);
    put_str(&mut out, GITHUB_AUTHENTICATION_CURRENTNESS_PROFILE_V1);
    put_str(
        &mut out,
        binding.receipt_digest().canonicalization_profile(),
    );
    put_digest(&mut out, binding.receipt_digest().sha256);
    put_digest(&mut out, authentication_policy_digest);
    put_digest(&mut out, trusted_builder_policy_digest);

    let root = binding.root_classification();
    put_str(&mut out, root.profile_id());
    put_str(&mut out, root.retained_root_digest_profile_id());
    put_digest(&mut out, root.retained_root_digest());
    put_digest(&mut out, root.public_good_root_digest());
    put_digest(&mut out, root.github_private_root_digest());
    put_sorted_strings(&mut out, root.public_good_tlog_uris());
    put_sorted_strings(&mut out, root.public_good_ca_uris());
    put_sorted_strings(&mut out, root.github_private_tlog_uris());
    put_sorted_strings(&mut out, root.github_private_ca_uris());

    let execution_policy = root.execution_policy();
    put_str(&mut out, execution_policy.profile_id());
    put_str(&mut out, execution_policy.authentication_policy_id());
    put_str(&mut out, execution_policy.executable_path());

    let execution = root.execution_receipt();
    put_u32(&mut out, execution.evidence_version);
    put_str(&mut out, &execution.verifier.profile_id);
    put_str(&mut out, &execution.verifier.backend_family);
    put_str(&mut out, &execution.verifier.semantic_version);
    put_digest(&mut out, execution.verifier.executable_sha256);
    put_str(&mut out, &execution.verifier.platform_profile);
    put_str(&mut out, &execution.verifier.command_profile_id);
    match &execution.verifier.nix_closure {
        Some(nix) => {
            put_bool(&mut out, true);
            put_str(&mut out, &nix.store_path);
            put_digest(&mut out, nix.closure_digest);
        }
        None => put_bool(&mut out, false),
    }
    put_digest(&mut out, execution.attestation_bundle_digest);
    put_u8(
        &mut out,
        match execution.trust_root_mode {
            VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 => 1,
            VerifierTrustRootModeV1::OfflinePinnedRootV1 => 2,
        },
    );
    put_digest(&mut out, execution.trusted_root_material_digest);
    put_u64(&mut out, execution.trusted_root_acquired_at_unix_seconds);
    put_digest(&mut out, execution.command_arguments_digest);
    put_digest(&mut out, execution.environment_profile_digest);
    put_digest(&mut out, execution.verifier_stdout_digest);
    put_digest(&mut out, execution.verifier_stderr_digest);
    put_u64(&mut out, execution.execution_started_at_unix_seconds);
    put_u64(&mut out, execution.execution_completed_at_unix_seconds);
    put_process_outcome(&mut out, execution.process_outcome);
    put_u32(&mut out, execution.parsed_result_count);

    let parsed = binding.parsed_output();
    put_str(&mut out, parsed.parser_profile_id());
    put_str(&mut out, parsed.stdout_digest_profile_id());
    put_digest(&mut out, parsed.raw_stdout_digest());
    put_u32(&mut out, parsed.candidates().len() as u32);
    put_u64(&mut out, binding.selected_candidate_index() as u64);
    put_git_object_id(&mut out, binding.selected_signer_revision());
    put_git_object_id(&mut out, binding.selected_source_revision());
    match binding.selected_run_identity() {
        Some(run) => {
            put_bool(&mut out, true);
            put_u64(&mut out, run.run_id());
            put_u64(&mut out, run.run_attempt());
        }
        None => put_bool(&mut out, false),
    }

    let candidate = binding.selected_candidate();
    let certificate = candidate.certificate();
    put_str(&mut out, certificate.certificate_issuer());
    put_str(&mut out, certificate.subject_alternative_name());
    put_opt_str(&mut out, certificate.oidc_issuer());
    put_opt_str(&mut out, certificate.build_signer_uri());
    put_opt_str(&mut out, certificate.build_signer_digest());
    put_opt_str(&mut out, certificate.runner_environment());
    put_opt_str(&mut out, certificate.source_repository_uri());
    put_opt_str(&mut out, certificate.source_repository_digest());
    put_opt_str(&mut out, certificate.source_repository_ref());
    put_opt_str(&mut out, certificate.source_repository_identifier());
    put_opt_str(&mut out, certificate.source_repository_owner_uri());
    put_opt_str(&mut out, certificate.source_repository_owner_identifier());
    put_opt_str(&mut out, certificate.build_config_uri());
    put_opt_str(&mut out, certificate.build_config_digest());
    put_opt_str(&mut out, certificate.build_trigger());
    put_opt_str(&mut out, certificate.run_invocation_uri());
    put_opt_str(
        &mut out,
        certificate.source_repository_visibility_at_signing(),
    );
    put_str(&mut out, candidate.statement_type());
    put_u32(&mut out, candidate.subjects().len() as u32);
    for subject in candidate.subjects() {
        put_str(&mut out, subject.name());
        put_digest(&mut out, subject.sha256());
    }
    put_timestamp_set(&mut out, candidate.verified_timestamps());

    put_predicate(&mut out, binding.predicate());
    let transparency = binding.transparency_resolution();
    put_u8(&mut out, transparency_status_tag(transparency.status()));
    put_sorted_strings(&mut out, transparency.matched_public_good_tlog_uris());
    put_sorted_strings(&mut out, transparency.matched_github_private_tlog_uris());
    put_sorted_strings(&mut out, transparency.unresolved_tlog_uris());
    put_digest(&mut out, transparency.retained_root_digest());

    put_u64(&mut out, evaluated_at_unix_seconds);
    put_bool(&mut out, true);
    match conservative_witness_time {
        Some(witness) => {
            put_bool(&mut out, true);
            put_u8(&mut out, timestamp_kind_tag(witness.kind));
            put_str(&mut out, &witness.uri);
            put_str(&mut out, &witness.raw_timestamp);
            put_u64(&mut out, witness.unix_seconds);
        }
        None => put_bool(&mut out, false),
    }
    match age_seconds {
        Some(age) => {
            put_bool(&mut out, true);
            put_u64(&mut out, age);
        }
        None => put_bool(&mut out, false),
    }

    sha256(&out)
}

fn put_predicate(out: &mut Vec<u8>, predicate: &QualificationAttestationPredicateV1) {
    put_u32(out, predicate.predicate_version);
    put_str(out, &predicate.predicate_schema);
    put_str(out, predicate.receipt_digest.canonicalization_profile());
    put_digest(out, predicate.receipt_digest.sha256);
    put_str(out, &predicate.qualification_profile);
    put_git_object_id(out, predicate.subject);
    put_digest(out, predicate.coherence_result_digest);
    put_u8(out, qualification_result_tag(predicate.result));
}

fn put_workflow_revision_policy(out: &mut Vec<u8>, policy: &WorkflowRevisionPolicyV1) {
    match policy {
        WorkflowRevisionPolicyV1::Exact(revision) => {
            put_u8(out, 1);
            put_git_object_id(out, *revision);
        }
        WorkflowRevisionPolicyV1::Allowed(revisions) => {
            put_u8(out, 2);
            let mut normalized: Vec<String> = revisions.iter().map(|r| r.to_wire()).collect();
            normalized.sort();
            put_u32(out, normalized.len() as u32);
            for revision in normalized {
                put_str(out, &revision);
            }
        }
    }
}

fn put_transparency_policy(out: &mut Vec<u8>, policy: TransparencyPolicyV1) {
    put_u8(
        out,
        match policy {
            TransparencyPolicyV1::PublicTransparencyRequired => 1,
            TransparencyPolicyV1::TimestampRequired => 2,
            TransparencyPolicyV1::PublicTransparencyAndTimestampRequired => 3,
        },
    );
}

fn put_freshness_policy(out: &mut Vec<u8>, policy: AuthenticationFreshnessPolicyV1) {
    match policy {
        AuthenticationFreshnessPolicyV1::CurrentAtVerification => put_u8(out, 1),
        AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(maximum) => {
            put_u8(out, 2);
            put_u64(out, maximum);
        }
    }
}

fn put_process_outcome(out: &mut Vec<u8>, outcome: VerifierProcessOutcomeV1) {
    match outcome {
        VerifierProcessOutcomeV1::ExitCode(code) => {
            put_u8(out, 1);
            out.extend_from_slice(&code.to_be_bytes());
        }
        VerifierProcessOutcomeV1::TerminatedBySignal(signal) => {
            put_u8(out, 2);
            out.extend_from_slice(&signal.to_be_bytes());
        }
        VerifierProcessOutcomeV1::TimedOut => put_u8(out, 3),
    }
}

fn put_timestamp_set(out: &mut Vec<u8>, timestamps: &[GitHubVerifiedTimestampEvidenceV1]) {
    let mut normalized: Vec<(u8, &str, &str, &str)> = timestamps
        .iter()
        .map(|value| {
            (
                timestamp_kind_tag(value.kind()),
                value.raw_type(),
                value.uri(),
                value.timestamp(),
            )
        })
        .collect();
    normalized.sort();
    put_u32(out, normalized.len() as u32);
    for (kind, raw_type, uri, timestamp) in normalized {
        put_u8(out, kind);
        put_str(out, raw_type);
        put_str(out, uri);
        put_str(out, timestamp);
    }
}

fn put_sorted_strings(out: &mut Vec<u8>, values: &[String]) {
    let mut normalized: Vec<&str> = values.iter().map(String::as_str).collect();
    normalized.sort();
    put_u32(out, normalized.len() as u32);
    for value in normalized {
        put_str(out, value);
    }
}

fn timestamp_kind_tag(kind: GitHubVerifiedTimestampWitnessKindV1) -> u8 {
    match kind {
        GitHubVerifiedTimestampWitnessKindV1::TransparencyLog => 1,
        GitHubVerifiedTimestampWitnessKindV1::TimestampAuthority => 2,
        GitHubVerifiedTimestampWitnessKindV1::OtherVerifiedWitness => 3,
    }
}

fn transparency_status_tag(status: GitHubResolvedTrustInstanceStatusV1) -> u8 {
    match status {
        GitHubResolvedTrustInstanceStatusV1::PublicGoodTransparencyConfirmed => 1,
        GitHubResolvedTrustInstanceStatusV1::GitHubPrivateTransparencyOnly => 2,
        GitHubResolvedTrustInstanceStatusV1::TimestampAuthorityOnly => 3,
        GitHubResolvedTrustInstanceStatusV1::NoVerifiedTimestamps => 4,
        GitHubResolvedTrustInstanceStatusV1::UnresolvedTrustInstance => 5,
        GitHubResolvedTrustInstanceStatusV1::MixedTrustDomains => 6,
    }
}

fn qualification_result_tag(result: QualificationResultV1) -> u8 {
    match result {
        QualificationResultV1::Pass => 1,
        QualificationResultV1::Fail => 2,
        QualificationResultV1::RecordedOnly => 3,
    }
}

fn put_git_object_id(out: &mut Vec<u8>, value: crate::GitObjectIdV1) {
    put_str(out, &value.to_wire());
}

fn put_digest(out: &mut Vec<u8>, value: Sha256DigestV1) {
    out.extend_from_slice(value.as_bytes());
}

fn put_opt_str(out: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            put_bool(out, true);
            put_str(out, value);
        }
        None => put_bool(out, false),
    }
}

fn put_str(out: &mut Vec<u8>, value: &str) {
    put_u32(out, value.len() as u32);
    out.extend_from_slice(value.as_bytes());
}

fn put_bool(out: &mut Vec<u8>, value: bool) {
    put_u8(out, u8::from(value));
}

fn put_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn sha256(bytes: &[u8]) -> Sha256DigestV1 {
    Sha256DigestV1::from_bytes(Sha256::digest(bytes).into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        GitObjectIdV1, SourceRevisionPolicyV1, VerifierProfileV1,
        GITHUB_HOSTED_RUNNER_ENVIRONMENT_V1, GITHUB_PUBLIC_REPOSITORY_VISIBILITY_V1,
    };

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn authentication_policy(revisions: Vec<GitObjectIdV1>) -> ReceiptAuthenticationPolicyV1 {
        ReceiptAuthenticationPolicyV1 {
            profile_id: "github-public-v1".into(),
            verifier_profile: VerifierProfileV1 {
                profile_id: "github-attestation-verifier-v1".into(),
                backend_family: "github-cli-attestation".into(),
                backend_version: "2.101.0".into(),
                verification_profile: "public-sigstore-qualification-v1".into(),
            },
            expected_attestation_subject_name: "mycelix-qualification-receipt".into(),
            expected_predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            expected_predicate_schema: "mycelix-qualification-attestation-predicate-v1".into(),
            trusted_root_profile: "sigstore-public-good-v1".into(),
            trusted_root_digest: digest(9),
            oidc_issuer: "https://token.actions.githubusercontent.com".into(),
            source_repository: "Luminous-Dynamics/mycelix".into(),
            source_repository_owner: "Luminous-Dynamics".into(),
            signer_workflow: ".github/workflows/qualify.yml".into(),
            signer_workflow_revision: WorkflowRevisionPolicyV1::Allowed(revisions),
            source_revision: SourceRevisionPolicyV1 {
                exact_commit: GitObjectIdV1::sha1([0xaa; 20]),
                exact_git_ref: Some("refs/heads/main".into()),
            },
            transparency_policy: TransparencyPolicyV1::PublicTransparencyRequired,
            freshness_policy: AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(3600),
        }
    }

    #[test]
    fn allowed_revision_set_order_does_not_change_policy_digest() {
        let a = GitObjectIdV1::sha1([0x11; 20]);
        let b = GitObjectIdV1::sha1([0x22; 20]);
        let left = receipt_authentication_policy_digest_v1(&authentication_policy(vec![a, b]))
            .unwrap();
        let right = receipt_authentication_policy_digest_v1(&authentication_policy(vec![b, a]))
            .unwrap();
        assert_eq!(left, right);
    }

    #[test]
    fn trusted_builder_policy_digest_binds_load_bearing_flags() {
        let left = GitHubTrustedBuilderPolicyV1::new(
            "builder-v1",
            "1176351975",
            "216969177",
            "https://in-toto.io/Statement/v1",
            GITHUB_HOSTED_RUNNER_ENVIRONMENT_V1,
            GITHUB_PUBLIC_REPOSITORY_VISIBILITY_V1,
            true,
            true,
        )
        .unwrap();
        let right = GitHubTrustedBuilderPolicyV1::new(
            "builder-v1",
            "1176351975",
            "216969177",
            "https://in-toto.io/Statement/v1",
            GITHUB_HOSTED_RUNNER_ENVIRONMENT_V1,
            GITHUB_PUBLIC_REPOSITORY_VISIBILITY_V1,
            false,
            true,
        )
        .unwrap();
        assert_ne!(
            github_trusted_builder_policy_digest_v1(&left).unwrap(),
            github_trusted_builder_policy_digest_v1(&right).unwrap()
        );
    }

    #[test]
    fn rfc3339_parser_handles_offsets_and_rejects_pre_epoch() {
        let z = DateTime::parse_from_rfc3339("2026-09-20T12:00:00Z")
            .unwrap()
            .timestamp();
        let offset = DateTime::parse_from_rfc3339("2026-09-20T14:00:00+02:00")
            .unwrap()
            .timestamp();
        assert_eq!(z, offset);
        assert!(DateTime::parse_from_rfc3339("not-a-time").is_err());
        assert!(DateTime::parse_from_rfc3339("1969-12-31T23:59:59Z")
            .unwrap()
            .timestamp()
            < 0);
    }
}
