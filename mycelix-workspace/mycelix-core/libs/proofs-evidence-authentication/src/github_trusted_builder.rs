use crate::{
    GitObjectIdV1, GitHubParsedVerificationCandidateV1,
    GitHubResolvedTrustInstanceStatusV1, GitHubTransparencyResolutionV1,
    GitHubTrustedRootClassificationV1, ParsedGitHubAttestationVerifierOutputV1,
    QualificationAttestationPredicateV1, QualificationReceiptDigestV1,
    QualificationReceiptV1, ReceiptAuthenticationPolicyV1, ReceiptCanonicalizationErrorV1,
    TransparencyPolicyV1, UntrustedWireParseErrorV1, WorkflowRevisionPolicyV1,
    parse_untrusted_qualification_predicate_json_v1,
    resolve_github_candidate_transparency_v1,
};

pub const MAX_GITHUB_TRUSTED_BUILDER_POLICY_TEXT_BYTES_V1: usize = 4096;
pub const GITHUB_HOSTED_RUNNER_ENVIRONMENT_V1: &str = "github-hosted";
pub const GITHUB_PUBLIC_REPOSITORY_VISIBILITY_V1: &str = "public";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubTrustedBuilderPolicyAuthorityV1 {
    StructuralPolicyOnly,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubTrustedBuilderBindingAuthorityV1 {
    TrustedBuilderBindingOnly,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GitHubActionsRunIdentityV1 {
    run_id: u64,
    run_attempt: u64,
}

impl GitHubActionsRunIdentityV1 {
    pub const fn run_id(&self) -> u64 {
        self.run_id
    }

    pub const fn run_attempt(&self) -> u64 {
        self.run_attempt
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubTrustedBuilderPolicyErrorV1 {
    EmptyField {
        field: &'static str,
    },
    FieldTooLong {
        field: &'static str,
        maximum: usize,
        actual: usize,
    },
    InvalidRepositoryIdentifier,
    InvalidOwnerIdentifier,
}

/// GitHub-specific immutable identity requirements layered over the backend-neutral
/// receipt authentication policy.
///
/// Constructing this policy grants no authority. A later qualification/profile
/// admission theorem decides which exact policy bytes are acceptable for production.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubTrustedBuilderPolicyV1 {
    profile_id: String,
    source_repository_identifier: String,
    source_repository_owner_identifier: String,
    expected_statement_type: String,
    required_runner_environment: String,
    required_repository_visibility: String,
    require_build_config_identity: bool,
    require_run_invocation_identity: bool,
}

impl GitHubTrustedBuilderPolicyV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        profile_id: impl Into<String>,
        source_repository_identifier: impl Into<String>,
        source_repository_owner_identifier: impl Into<String>,
        expected_statement_type: impl Into<String>,
        required_runner_environment: impl Into<String>,
        required_repository_visibility: impl Into<String>,
        require_build_config_identity: bool,
        require_run_invocation_identity: bool,
    ) -> Result<Self, GitHubTrustedBuilderPolicyErrorV1> {
        let policy = Self {
            profile_id: profile_id.into(),
            source_repository_identifier: source_repository_identifier.into(),
            source_repository_owner_identifier: source_repository_owner_identifier.into(),
            expected_statement_type: expected_statement_type.into(),
            required_runner_environment: required_runner_environment.into(),
            required_repository_visibility: required_repository_visibility.into(),
            require_build_config_identity,
            require_run_invocation_identity,
        };
        policy.validate()?;
        Ok(policy)
    }

    pub fn validate(&self) -> Result<(), GitHubTrustedBuilderPolicyErrorV1> {
        for (field, value) in [
            ("profile_id", self.profile_id.as_str()),
            (
                "source_repository_identifier",
                self.source_repository_identifier.as_str(),
            ),
            (
                "source_repository_owner_identifier",
                self.source_repository_owner_identifier.as_str(),
            ),
            ("expected_statement_type", self.expected_statement_type.as_str()),
            (
                "required_runner_environment",
                self.required_runner_environment.as_str(),
            ),
            (
                "required_repository_visibility",
                self.required_repository_visibility.as_str(),
            ),
        ] {
            check_policy_text(field, value)?;
        }

        if !is_positive_decimal_identifier(&self.source_repository_identifier) {
            return Err(GitHubTrustedBuilderPolicyErrorV1::InvalidRepositoryIdentifier);
        }
        if !is_positive_decimal_identifier(&self.source_repository_owner_identifier) {
            return Err(GitHubTrustedBuilderPolicyErrorV1::InvalidOwnerIdentifier);
        }
        Ok(())
    }

    pub fn profile_id(&self) -> &str {
        &self.profile_id
    }

    pub fn source_repository_identifier(&self) -> &str {
        &self.source_repository_identifier
    }

    pub fn source_repository_owner_identifier(&self) -> &str {
        &self.source_repository_owner_identifier
    }

    pub fn expected_statement_type(&self) -> &str {
        &self.expected_statement_type
    }

    pub fn required_runner_environment(&self) -> &str {
        &self.required_runner_environment
    }

    pub fn required_repository_visibility(&self) -> &str {
        &self.required_repository_visibility
    }

    pub const fn require_build_config_identity(&self) -> bool {
        self.require_build_config_identity
    }

    pub const fn require_run_invocation_identity(&self) -> bool {
        self.require_run_invocation_identity
    }

    pub const fn authority_scope(&self) -> GitHubTrustedBuilderPolicyAuthorityV1 {
        GitHubTrustedBuilderPolicyAuthorityV1::StructuralPolicyOnly
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubTrustedBuilderCandidateMismatchV1 {
    MissingCertificateField {
        field: &'static str,
    },
    CertificateFieldMismatch {
        field: &'static str,
    },
    MalformedGitObjectId {
        field: &'static str,
    },
    SignerRevisionNotAllowed,
    InvalidBuildSignerUri,
    SubjectAlternativeNameMismatch,
    InvalidBuildConfigIdentity,
    InvalidRunInvocationIdentity,
    StatementTypeMismatch,
    StatementSubjectCountMismatch {
        actual: usize,
    },
    StatementSubjectNameMismatch,
    StatementSubjectDigestMismatch,
    PredicateTypeMismatch,
    PredicateSerializationFailed,
    PredicateStrictParseFailed,
    PredicateSchemaMismatch,
    PredicateReceiptDigestMismatch,
    PredicateQualificationProfileMismatch,
    PredicateSubjectMismatch,
    PredicateCoherenceDigestMismatch,
    PredicateResultMismatch,
    TransparencyPolicyNotMet,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubTrustedBuilderCandidateRejectionV1 {
    candidate_index: usize,
    reason: GitHubTrustedBuilderCandidateMismatchV1,
}

impl GitHubTrustedBuilderCandidateRejectionV1 {
    pub const fn candidate_index(&self) -> usize {
        self.candidate_index
    }

    pub const fn reason(&self) -> &GitHubTrustedBuilderCandidateMismatchV1 {
        &self.reason
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubTrustedBuilderBindingErrorV1 {
    InvalidAuthenticationPolicy,
    InvalidTrustedBuilderPolicy,
    MissingExactSourceRef,
    ReceiptCanonicalization(ReceiptCanonicalizationErrorV1),
    ParserReceiptDigestMismatch,
    RootReceiptDigestMismatch,
    ExecutionLineageMismatch,
    AuthenticationPolicyLineageMismatch,
    TrustedRootPolicyDigestMismatch,
    NoMatchingCandidate {
        rejections: Vec<GitHubTrustedBuilderCandidateRejectionV1>,
    },
    AmbiguousMatchingCandidates {
        count: usize,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct GitHubTrustedBuilderBindingV1 {
    receipt_digest: QualificationReceiptDigestV1,
    authentication_policy: ReceiptAuthenticationPolicyV1,
    trusted_builder_policy: GitHubTrustedBuilderPolicyV1,
    parsed_output: ParsedGitHubAttestationVerifierOutputV1,
    root_classification: GitHubTrustedRootClassificationV1,
    selected_candidate_index: usize,
    selected_signer_revision: GitObjectIdV1,
    selected_source_revision: GitObjectIdV1,
    selected_run_identity: Option<GitHubActionsRunIdentityV1>,
    predicate: QualificationAttestationPredicateV1,
    transparency_resolution: GitHubTransparencyResolutionV1,
    authority: GitHubTrustedBuilderBindingAuthorityV1,
}

impl GitHubTrustedBuilderBindingV1 {
    pub const fn receipt_digest(&self) -> QualificationReceiptDigestV1 {
        self.receipt_digest
    }

    pub fn authentication_policy(&self) -> &ReceiptAuthenticationPolicyV1 {
        &self.authentication_policy
    }

    pub fn trusted_builder_policy(&self) -> &GitHubTrustedBuilderPolicyV1 {
        &self.trusted_builder_policy
    }

    pub fn parsed_output(&self) -> &ParsedGitHubAttestationVerifierOutputV1 {
        &self.parsed_output
    }

    pub fn root_classification(&self) -> &GitHubTrustedRootClassificationV1 {
        &self.root_classification
    }

    pub const fn selected_candidate_index(&self) -> usize {
        self.selected_candidate_index
    }

    pub fn selected_candidate(&self) -> &GitHubParsedVerificationCandidateV1 {
        &self.parsed_output.candidates()[self.selected_candidate_index]
    }

    pub const fn selected_signer_revision(&self) -> GitObjectIdV1 {
        self.selected_signer_revision
    }

    pub const fn selected_source_revision(&self) -> GitObjectIdV1 {
        self.selected_source_revision
    }

    pub const fn selected_run_identity(&self) -> Option<GitHubActionsRunIdentityV1> {
        self.selected_run_identity
    }

    pub fn predicate(&self) -> &QualificationAttestationPredicateV1 {
        &self.predicate
    }

    pub fn transparency_resolution(&self) -> &GitHubTransparencyResolutionV1 {
        &self.transparency_resolution
    }

    pub const fn authority_scope(&self) -> GitHubTrustedBuilderBindingAuthorityV1 {
        self.authority
    }

    pub const fn establishes_trusted_builder_binding(&self) -> bool {
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

struct CandidateMatchV1 {
    signer_revision: GitObjectIdV1,
    source_revision: GitObjectIdV1,
    run_identity: Option<GitHubActionsRunIdentityV1>,
    predicate: QualificationAttestationPredicateV1,
    transparency_resolution: GitHubTransparencyResolutionV1,
}

pub fn bind_unique_github_trusted_builder_candidate_v1(
    receipt: &QualificationReceiptV1,
    authentication_policy: &ReceiptAuthenticationPolicyV1,
    trusted_builder_policy: &GitHubTrustedBuilderPolicyV1,
    parsed_output: &ParsedGitHubAttestationVerifierOutputV1,
    root_classification: &GitHubTrustedRootClassificationV1,
) -> Result<GitHubTrustedBuilderBindingV1, GitHubTrustedBuilderBindingErrorV1> {
    authentication_policy
        .validate()
        .map_err(|_| GitHubTrustedBuilderBindingErrorV1::InvalidAuthenticationPolicy)?;
    trusted_builder_policy
        .validate()
        .map_err(|_| GitHubTrustedBuilderBindingErrorV1::InvalidTrustedBuilderPolicy)?;
    let expected_source_ref = authentication_policy
        .source_revision
        .exact_git_ref
        .as_deref()
        .ok_or(GitHubTrustedBuilderBindingErrorV1::MissingExactSourceRef)?;

    let receipt_digest = receipt
        .digest()
        .map_err(GitHubTrustedBuilderBindingErrorV1::ReceiptCanonicalization)?;
    if parsed_output.canonical_receipt_digest() != receipt_digest {
        return Err(GitHubTrustedBuilderBindingErrorV1::ParserReceiptDigestMismatch);
    }
    if root_classification.execution_receipt().canonical_receipt_digest != receipt_digest
        || root_classification.command_plan().canonical_receipt_digest() != receipt_digest
    {
        return Err(GitHubTrustedBuilderBindingErrorV1::RootReceiptDigestMismatch);
    }
    if parsed_output.execution_receipt() != root_classification.execution_receipt() {
        return Err(GitHubTrustedBuilderBindingErrorV1::ExecutionLineageMismatch);
    }
    if root_classification.execution_policy().authentication_policy_id()
        != authentication_policy.profile_id
    {
        return Err(GitHubTrustedBuilderBindingErrorV1::AuthenticationPolicyLineageMismatch);
    }
    if root_classification.retained_root_digest() != authentication_policy.trusted_root_digest {
        return Err(GitHubTrustedBuilderBindingErrorV1::TrustedRootPolicyDigestMismatch);
    }

    let mut matches = Vec::new();
    let mut rejections = Vec::new();
    for (candidate_index, candidate) in parsed_output.candidates().iter().enumerate() {
        match match_candidate(
            candidate,
            receipt,
            receipt_digest,
            authentication_policy,
            trusted_builder_policy,
            root_classification,
            expected_source_ref,
        ) {
            Ok(candidate_match) => matches.push((candidate_index, candidate_match)),
            Err(reason) => rejections.push(GitHubTrustedBuilderCandidateRejectionV1 {
                candidate_index,
                reason,
            }),
        }
    }

    if matches.is_empty() {
        return Err(GitHubTrustedBuilderBindingErrorV1::NoMatchingCandidate { rejections });
    }
    if matches.len() != 1 {
        return Err(GitHubTrustedBuilderBindingErrorV1::AmbiguousMatchingCandidates {
            count: matches.len(),
        });
    }

    let (selected_candidate_index, selected) = matches.pop().expect("exactly one match");
    Ok(GitHubTrustedBuilderBindingV1 {
        receipt_digest,
        authentication_policy: authentication_policy.clone(),
        trusted_builder_policy: trusted_builder_policy.clone(),
        parsed_output: parsed_output.clone(),
        root_classification: root_classification.clone(),
        selected_candidate_index,
        selected_signer_revision: selected.signer_revision,
        selected_source_revision: selected.source_revision,
        selected_run_identity: selected.run_identity,
        predicate: selected.predicate,
        transparency_resolution: selected.transparency_resolution,
        authority: GitHubTrustedBuilderBindingAuthorityV1::TrustedBuilderBindingOnly,
    })
}

#[allow(clippy::too_many_arguments)]
fn match_candidate(
    candidate: &GitHubParsedVerificationCandidateV1,
    receipt: &QualificationReceiptV1,
    receipt_digest: QualificationReceiptDigestV1,
    authentication_policy: &ReceiptAuthenticationPolicyV1,
    trusted_builder_policy: &GitHubTrustedBuilderPolicyV1,
    root_classification: &GitHubTrustedRootClassificationV1,
    expected_source_ref: &str,
) -> Result<CandidateMatchV1, GitHubTrustedBuilderCandidateMismatchV1> {
    let certificate = candidate.certificate();

    require_certificate_field_eq(
        "issuer",
        certificate.oidc_issuer(),
        &authentication_policy.oidc_issuer,
    )?;

    let expected_repository_uri = format!(
        "https://github.com/{}",
        authentication_policy.source_repository
    );
    require_certificate_field_eq(
        "sourceRepositoryURI",
        certificate.source_repository_uri(),
        &expected_repository_uri,
    )?;
    require_certificate_field_eq(
        "sourceRepositoryIdentifier",
        certificate.source_repository_identifier(),
        trusted_builder_policy.source_repository_identifier(),
    )?;

    let expected_owner_uri = format!(
        "https://github.com/{}",
        authentication_policy.source_repository_owner
    );
    require_certificate_field_eq(
        "sourceRepositoryOwnerURI",
        certificate.source_repository_owner_uri(),
        &expected_owner_uri,
    )?;
    require_certificate_field_eq(
        "sourceRepositoryOwnerIdentifier",
        certificate.source_repository_owner_identifier(),
        trusted_builder_policy.source_repository_owner_identifier(),
    )?;

    require_certificate_field_eq(
        "runnerEnvironment",
        certificate.runner_environment(),
        trusted_builder_policy.required_runner_environment(),
    )?;
    require_certificate_field_eq(
        "sourceRepositoryVisibilityAtSigning",
        certificate.source_repository_visibility_at_signing(),
        trusted_builder_policy.required_repository_visibility(),
    )?;
    require_certificate_field_eq(
        "sourceRepositoryRef",
        certificate.source_repository_ref(),
        expected_source_ref,
    )?;

    let source_revision = parse_required_git_object_id(
        "sourceRepositoryDigest",
        certificate.source_repository_digest(),
    )?;
    if source_revision != authentication_policy.source_revision.exact_commit {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::CertificateFieldMismatch {
            field: "sourceRepositoryDigest",
        });
    }

    let build_signer_uri = certificate.build_signer_uri().ok_or(
        GitHubTrustedBuilderCandidateMismatchV1::MissingCertificateField {
            field: "buildSignerURI",
        },
    )?;
    let signer_prefix = format!(
        "https://github.com/{}/{}@",
        authentication_policy.source_repository, authentication_policy.signer_workflow
    );
    if !build_signer_uri.starts_with(&signer_prefix)
        || build_signer_uri.len() == signer_prefix.len()
    {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::InvalidBuildSignerUri);
    }
    if certificate.subject_alternative_name() != build_signer_uri {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::SubjectAlternativeNameMismatch);
    }

    let signer_revision = parse_required_git_object_id(
        "buildSignerDigest",
        certificate.build_signer_digest(),
    )?;
    if !workflow_revision_allowed(
        &authentication_policy.signer_workflow_revision,
        signer_revision,
    ) {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::SignerRevisionNotAllowed);
    }

    if trusted_builder_policy.require_build_config_identity()
        && !valid_build_config_identity(certificate, &authentication_policy.source_repository)
    {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::InvalidBuildConfigIdentity);
    }

    let run_identity = parse_run_invocation_uri(
        certificate.run_invocation_uri(),
        &authentication_policy.source_repository,
        trusted_builder_policy.require_run_invocation_identity(),
    )?;

    if candidate.statement_type() != trusted_builder_policy.expected_statement_type() {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::StatementTypeMismatch);
    }
    if candidate.subjects().len() != 1 {
        return Err(
            GitHubTrustedBuilderCandidateMismatchV1::StatementSubjectCountMismatch {
                actual: candidate.subjects().len(),
            },
        );
    }
    let subject = &candidate.subjects()[0];
    if subject.name() != authentication_policy.expected_attestation_subject_name {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::StatementSubjectNameMismatch);
    }
    if subject.sha256() != receipt_digest.sha256 {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::StatementSubjectDigestMismatch);
    }

    if candidate.predicate().predicate_type() != authentication_policy.expected_predicate_type {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::PredicateTypeMismatch);
    }
    let predicate_json = candidate
        .predicate()
        .predicate_json()
        .map_err(|_| GitHubTrustedBuilderCandidateMismatchV1::PredicateSerializationFailed)?;
    let predicate = parse_untrusted_qualification_predicate_json_v1(&predicate_json)
        .map_err(map_predicate_parse_failure)?;
    if predicate.predicate_schema != authentication_policy.expected_predicate_schema {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::PredicateSchemaMismatch);
    }
    if predicate.receipt_digest != receipt_digest {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::PredicateReceiptDigestMismatch);
    }
    if predicate.qualification_profile != receipt.qualification_profile {
        return Err(
            GitHubTrustedBuilderCandidateMismatchV1::PredicateQualificationProfileMismatch,
        );
    }
    if predicate.subject != receipt.subject {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::PredicateSubjectMismatch);
    }
    if predicate.coherence_result_digest != receipt.coherence_result_digest {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::PredicateCoherenceDigestMismatch);
    }
    if predicate.result != receipt.result {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::PredicateResultMismatch);
    }

    let transparency_resolution =
        resolve_github_candidate_transparency_v1(root_classification, candidate);
    if !transparency_policy_met(
        authentication_policy.transparency_policy,
        &transparency_resolution,
        candidate,
    ) {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::TransparencyPolicyNotMet);
    }

    Ok(CandidateMatchV1 {
        signer_revision,
        source_revision,
        run_identity,
        predicate,
        transparency_resolution,
    })
}

fn require_certificate_field_eq(
    field: &'static str,
    actual: Option<&str>,
    expected: &str,
) -> Result<(), GitHubTrustedBuilderCandidateMismatchV1> {
    let actual = actual.ok_or(
        GitHubTrustedBuilderCandidateMismatchV1::MissingCertificateField { field },
    )?;
    if actual != expected {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::CertificateFieldMismatch {
            field,
        });
    }
    Ok(())
}

fn parse_required_git_object_id(
    field: &'static str,
    value: Option<&str>,
) -> Result<GitObjectIdV1, GitHubTrustedBuilderCandidateMismatchV1> {
    let value = value.ok_or(
        GitHubTrustedBuilderCandidateMismatchV1::MissingCertificateField { field },
    )?;
    parse_git_object_id_hex(value)
        .ok_or(GitHubTrustedBuilderCandidateMismatchV1::MalformedGitObjectId { field })
}

fn parse_git_object_id_hex(value: &str) -> Option<GitObjectIdV1> {
    match value.len() {
        40 => GitObjectIdV1::sha1_from_hex(value).ok(),
        64 => GitObjectIdV1::sha256_from_hex(value).ok(),
        _ => None,
    }
}

fn workflow_revision_allowed(policy: &WorkflowRevisionPolicyV1, actual: GitObjectIdV1) -> bool {
    match policy {
        WorkflowRevisionPolicyV1::Exact(expected) => *expected == actual,
        WorkflowRevisionPolicyV1::Allowed(allowed) => allowed.contains(&actual),
    }
}

fn valid_build_config_identity(
    certificate: &crate::GitHubCertificateIdentityFactsV1,
    repository: &str,
) -> bool {
    let Some(uri) = certificate.build_config_uri() else {
        return false;
    };
    let Some(digest) = certificate.build_config_digest() else {
        return false;
    };
    let prefix = format!("https://github.com/{repository}/.github/workflows/");
    let Some(rest) = uri.strip_prefix(&prefix) else {
        return false;
    };
    let Some((workflow_path, workflow_ref)) = rest.rsplit_once('@') else {
        return false;
    };
    !workflow_path.is_empty()
        && !workflow_ref.is_empty()
        && parse_git_object_id_hex(digest).is_some()
}

fn parse_run_invocation_uri(
    value: Option<&str>,
    repository: &str,
    required: bool,
) -> Result<Option<GitHubActionsRunIdentityV1>, GitHubTrustedBuilderCandidateMismatchV1> {
    let Some(value) = value else {
        return if required {
            Err(GitHubTrustedBuilderCandidateMismatchV1::InvalidRunInvocationIdentity)
        } else {
            Ok(None)
        };
    };

    let prefix = format!("https://github.com/{repository}/actions/runs/");
    let Some(tail) = value.strip_prefix(&prefix) else {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::InvalidRunInvocationIdentity);
    };
    let Some((run_id, attempt_tail)) = tail.split_once("/attempts/") else {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::InvalidRunInvocationIdentity);
    };
    if run_id.is_empty() || attempt_tail.is_empty() || attempt_tail.contains('/') {
        return Err(GitHubTrustedBuilderCandidateMismatchV1::InvalidRunInvocationIdentity);
    }
    let run_id = run_id
        .parse::<u64>()
        .ok()
        .filter(|value| *value > 0)
        .ok_or(GitHubTrustedBuilderCandidateMismatchV1::InvalidRunInvocationIdentity)?;
    let run_attempt = attempt_tail
        .parse::<u64>()
        .ok()
        .filter(|value| *value > 0)
        .ok_or(GitHubTrustedBuilderCandidateMismatchV1::InvalidRunInvocationIdentity)?;

    Ok(Some(GitHubActionsRunIdentityV1 {
        run_id,
        run_attempt,
    }))
}

fn transparency_policy_met(
    policy: TransparencyPolicyV1,
    resolution: &GitHubTransparencyResolutionV1,
    candidate: &GitHubParsedVerificationCandidateV1,
) -> bool {
    let public_good =
        resolution.status() == GitHubResolvedTrustInstanceStatusV1::PublicGoodTransparencyConfirmed;
    let timestamp = !candidate.verified_timestamps().is_empty();
    match policy {
        TransparencyPolicyV1::PublicTransparencyRequired => public_good,
        TransparencyPolicyV1::TimestampRequired => timestamp,
        TransparencyPolicyV1::PublicTransparencyAndTimestampRequired => public_good && timestamp,
    }
}

fn map_predicate_parse_failure(
    _error: UntrustedWireParseErrorV1,
) -> GitHubTrustedBuilderCandidateMismatchV1 {
    GitHubTrustedBuilderCandidateMismatchV1::PredicateStrictParseFailed
}

fn is_positive_decimal_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.bytes().all(|byte| byte.is_ascii_digit())
        && value.parse::<u64>().ok().is_some_and(|identifier| identifier > 0)
}

fn check_policy_text(
    field: &'static str,
    value: &str,
) -> Result<(), GitHubTrustedBuilderPolicyErrorV1> {
    if value.trim().is_empty() {
        return Err(GitHubTrustedBuilderPolicyErrorV1::EmptyField { field });
    }
    if value.len() > MAX_GITHUB_TRUSTED_BUILDER_POLICY_TEXT_BYTES_V1 {
        return Err(GitHubTrustedBuilderPolicyErrorV1::FieldTooLong {
            field,
            maximum: MAX_GITHUB_TRUSTED_BUILDER_POLICY_TEXT_BYTES_V1,
            actual: value.len(),
        });
    }
    Ok(())
}
