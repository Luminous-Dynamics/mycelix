use sha2::{Digest, Sha256};

use crate::{
    GitObjectIdV1, QualificationReceiptDigestV1, QualificationReceiptV1,
    ReceiptAuthenticationPolicyV1, ReceiptCanonicalizationErrorV1, Sha256DigestV1,
    VerifierExecutableIdentityV1, WorkflowRevisionPolicyV1,
};

pub const GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1: &str =
    "mycelix-github-public-qualification-verifier-command-v1";
pub const GITHUB_PUBLIC_BACKEND_FAMILY_V1: &str = "github-cli-attestation";
pub const GITHUB_PUBLIC_HOSTNAME_V1: &str = "github.com";
pub const GITHUB_ACTIONS_OIDC_ISSUER_V1: &str =
    "https://token.actions.githubusercontent.com";
pub const MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1: &str =
    "https://mycelix.org/attestations/qualification/v1";
pub const MYCELIX_QUALIFICATION_SUBJECT_NAME_V1: &str =
    "mycelix-qualification-receipt";
pub const MYCELIX_REPOSITORY_V1: &str = "Luminous-Dynamics/mycelix";
pub const MYCELIX_REPOSITORY_OWNER_V1: &str = "Luminous-Dynamics";
pub const GITHUB_PUBLIC_PLATFORM_PROFILE_V1: &str = "linux-x86_64";
pub const GITHUB_PUBLIC_MAX_ATTESTATIONS_V1: u32 = 8;
pub const GITHUB_PUBLIC_PROCESS_TIMEOUT_SECONDS_V1: u64 = 60;
pub const GITHUB_PUBLIC_MAX_STDOUT_BYTES_V1: usize = 4 * 1024 * 1024;
pub const GITHUB_PUBLIC_MAX_STDERR_BYTES_V1: usize = 1024 * 1024;

const COMMAND_DIGEST_DOMAIN_V1: &[u8] = b"mycelix:github-verifier-command-plan:v1\0";
const ENVIRONMENT_DIGEST_DOMAIN_V1: &[u8] = b"mycelix:github-verifier-environment:v1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubCommandPlanAuthorityV1 {
    CommandPlanOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubCommandPlanErrorV1 {
    InvalidAuthenticationPolicy,
    InvalidVerifierIdentity,
    WrongRepository,
    WrongRepositoryOwner,
    WrongOidcIssuer,
    WrongPredicateType,
    WrongAttestationSubjectName,
    WrongVerifierBackendFamily,
    WrongVerifierCommandProfile,
    WrongPlatformProfile,
    InvalidSignerWorkflowPath,
    SignerRevisionNotAllowed,
    MissingExactSourceRef,
    ReceiptSubjectDoesNotMatchSourceRevision,
    InvalidAbsolutePath { field: &'static str },
    NixExecutablePathMismatch,
    ReceiptCanonicalization(ReceiptCanonicalizationErrorV1),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FixedEnvironmentVariableV1 {
    key: String,
    value: String,
}

impl FixedEnvironmentVariableV1 {
    pub fn key(&self) -> &str {
        &self.key
    }

    pub fn value(&self) -> &str {
        &self.value
    }
}

/// Deterministic, non-authoritative command plan for the initial public GitHub
/// qualification-attestation verifier profile.
///
/// Fields are private so external callers cannot manufacture a plan that looks
/// admitted while bypassing [`build_github_public_command_plan_v1`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubPublicCommandPlanV1 {
    executable_path: String,
    working_directory: String,
    subject_path: String,
    bundle_path: String,
    trusted_root_path: String,
    download_args: Vec<String>,
    trusted_root_args: Vec<String>,
    verify_args: Vec<String>,
    clear_environment: bool,
    fixed_environment: Vec<FixedEnvironmentVariableV1>,
    inherited_secret_environment_keys: Vec<String>,
    timeout_seconds: u64,
    max_stdout_bytes: usize,
    max_stderr_bytes: usize,
    expected_max_results: u32,
    canonical_receipt_digest: QualificationReceiptDigestV1,
}

impl GitHubPublicCommandPlanV1 {
    pub fn executable_path(&self) -> &str {
        &self.executable_path
    }

    pub fn working_directory(&self) -> &str {
        &self.working_directory
    }

    pub fn subject_path(&self) -> &str {
        &self.subject_path
    }

    pub fn bundle_path(&self) -> &str {
        &self.bundle_path
    }

    pub fn trusted_root_path(&self) -> &str {
        &self.trusted_root_path
    }

    pub fn download_args(&self) -> &[String] {
        &self.download_args
    }

    pub fn trusted_root_args(&self) -> &[String] {
        &self.trusted_root_args
    }

    pub fn verify_args(&self) -> &[String] {
        &self.verify_args
    }

    pub const fn clear_environment(&self) -> bool {
        self.clear_environment
    }

    pub fn fixed_environment(&self) -> &[FixedEnvironmentVariableV1] {
        &self.fixed_environment
    }

    pub fn inherited_secret_environment_keys(&self) -> &[String] {
        &self.inherited_secret_environment_keys
    }

    pub const fn timeout_seconds(&self) -> u64 {
        self.timeout_seconds
    }

    pub const fn max_stdout_bytes(&self) -> usize {
        self.max_stdout_bytes
    }

    pub const fn max_stderr_bytes(&self) -> usize {
        self.max_stderr_bytes
    }

    pub const fn expected_max_results(&self) -> u32 {
        self.expected_max_results
    }

    pub const fn canonical_receipt_digest(&self) -> QualificationReceiptDigestV1 {
        self.canonical_receipt_digest
    }

    pub const fn authority_scope(&self) -> GitHubCommandPlanAuthorityV1 {
        GitHubCommandPlanAuthorityV1::CommandPlanOnly
    }

    pub const fn executes_processes(&self) -> bool {
        false
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub fn command_arguments_digest(&self) -> Sha256DigestV1 {
        let mut hasher = Sha256::new();
        hasher.update(COMMAND_DIGEST_DOMAIN_V1);
        hash_str(&mut hasher, GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1);
        hash_str(&mut hasher, &self.executable_path);
        hash_str(&mut hasher, &self.working_directory);
        hash_str(&mut hasher, &self.subject_path);
        hash_str(&mut hasher, &self.bundle_path);
        hash_str(&mut hasher, &self.trusted_root_path);
        hash_vec(&mut hasher, "download", &self.download_args);
        hash_vec(&mut hasher, "trusted-root", &self.trusted_root_args);
        hash_vec(&mut hasher, "verify", &self.verify_args);
        Sha256DigestV1::from_bytes(hasher.finalize().into())
    }

    /// Hash only the environment *policy*. Secret values such as `GH_TOKEN` are
    /// deliberately excluded; the allowed secret key names remain bound.
    pub fn environment_profile_digest(&self) -> Sha256DigestV1 {
        let mut hasher = Sha256::new();
        hasher.update(ENVIRONMENT_DIGEST_DOMAIN_V1);
        hasher.update([u8::from(self.clear_environment)]);
        for assignment in &self.fixed_environment {
            hash_str(&mut hasher, &assignment.key);
            hash_str(&mut hasher, &assignment.value);
        }
        for key in &self.inherited_secret_environment_keys {
            hash_str(&mut hasher, key);
        }
        Sha256DigestV1::from_bytes(hasher.finalize().into())
    }
}

pub fn build_github_public_command_plan_v1(
    policy: &ReceiptAuthenticationPolicyV1,
    receipt: &QualificationReceiptV1,
    verifier: &VerifierExecutableIdentityV1,
    selected_signer_revision: GitObjectIdV1,
    executable_path: &str,
    working_directory: &str,
) -> Result<GitHubPublicCommandPlanV1, GitHubCommandPlanErrorV1> {
    policy
        .validate()
        .map_err(|_| GitHubCommandPlanErrorV1::InvalidAuthenticationPolicy)?;
    verifier
        .validate()
        .map_err(|_| GitHubCommandPlanErrorV1::InvalidVerifierIdentity)?;

    if policy.source_repository != MYCELIX_REPOSITORY_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongRepository);
    }
    if policy.source_repository_owner != MYCELIX_REPOSITORY_OWNER_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongRepositoryOwner);
    }
    if policy.oidc_issuer != GITHUB_ACTIONS_OIDC_ISSUER_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongOidcIssuer);
    }
    if policy.expected_predicate_type != MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongPredicateType);
    }
    if policy.expected_attestation_subject_name != MYCELIX_QUALIFICATION_SUBJECT_NAME_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongAttestationSubjectName);
    }
    if verifier.backend_family != GITHUB_PUBLIC_BACKEND_FAMILY_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongVerifierBackendFamily);
    }
    if verifier.command_profile_id != GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongVerifierCommandProfile);
    }
    if verifier.platform_profile != GITHUB_PUBLIC_PLATFORM_PROFILE_V1 {
        return Err(GitHubCommandPlanErrorV1::WrongPlatformProfile);
    }
    if !valid_workflow_path(&policy.signer_workflow) {
        return Err(GitHubCommandPlanErrorV1::InvalidSignerWorkflowPath);
    }
    if !workflow_revision_allowed(&policy.signer_workflow_revision, selected_signer_revision) {
        return Err(GitHubCommandPlanErrorV1::SignerRevisionNotAllowed);
    }
    let source_ref = policy
        .source_revision
        .exact_git_ref
        .as_deref()
        .ok_or(GitHubCommandPlanErrorV1::MissingExactSourceRef)?;
    if receipt.subject != policy.source_revision.exact_commit {
        return Err(GitHubCommandPlanErrorV1::ReceiptSubjectDoesNotMatchSourceRevision);
    }

    validate_absolute_path("executable_path", executable_path)?;
    validate_absolute_path("working_directory", working_directory)?;
    if let Some(nix) = &verifier.nix_closure {
        if nix.store_path != executable_path {
            return Err(GitHubCommandPlanErrorV1::NixExecutablePathMismatch);
        }
    }

    let receipt_digest = receipt
        .digest()
        .map_err(GitHubCommandPlanErrorV1::ReceiptCanonicalization)?;
    let work = working_directory.trim_end_matches('/');
    let subject_path = format!("{work}/qualification-receipt.bin");
    let bundle_path = format!("{work}/sha256:{}.jsonl", receipt_digest.sha256.to_hex());
    let trusted_root_path = format!("{work}/trusted_root.jsonl");
    let gh_config_dir = format!("{work}/gh-config");
    let home_dir = format!("{work}/home");
    let signer_workflow = format!(
        "{}/{}/{}",
        GITHUB_PUBLIC_HOSTNAME_V1, MYCELIX_REPOSITORY_V1, policy.signer_workflow
    );
    let limit = GITHUB_PUBLIC_MAX_ATTESTATIONS_V1.to_string();

    let download_args = vec![
        "attestation".into(),
        "download".into(),
        subject_path.clone(),
        "--repo".into(),
        MYCELIX_REPOSITORY_V1.into(),
        "--predicate-type".into(),
        MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1.into(),
        "--limit".into(),
        limit.clone(),
        "--hostname".into(),
        GITHUB_PUBLIC_HOSTNAME_V1.into(),
    ];

    let trusted_root_args = vec![
        "attestation".into(),
        "trusted-root".into(),
        "--hostname".into(),
        GITHUB_PUBLIC_HOSTNAME_V1.into(),
    ];

    let mut verify_args = vec![
        "attestation".into(),
        "verify".into(),
        subject_path.clone(),
        "--repo".into(),
        MYCELIX_REPOSITORY_V1.into(),
        "--bundle".into(),
        bundle_path.clone(),
        "--custom-trusted-root".into(),
        trusted_root_path.clone(),
        "--predicate-type".into(),
        MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1.into(),
        "--cert-oidc-issuer".into(),
        GITHUB_ACTIONS_OIDC_ISSUER_V1.into(),
        "--signer-workflow".into(),
        signer_workflow,
        "--signer-digest".into(),
        selected_signer_revision.to_hex(),
        "--source-digest".into(),
        policy.source_revision.exact_commit.to_hex(),
        "--source-ref".into(),
        source_ref.into(),
        "--deny-self-hosted-runners".into(),
        "--limit".into(),
        limit,
        "--hostname".into(),
        GITHUB_PUBLIC_HOSTNAME_V1.into(),
        "--format".into(),
        "json".into(),
    ];
    // Keep deterministic ordering even if optional fields are added in future versions.
    verify_args.shrink_to_fit();

    Ok(GitHubPublicCommandPlanV1 {
        executable_path: executable_path.into(),
        working_directory: work.into(),
        subject_path,
        bundle_path,
        trusted_root_path,
        download_args,
        trusted_root_args,
        verify_args,
        clear_environment: true,
        fixed_environment: vec![
            FixedEnvironmentVariableV1 {
                key: "GH_CONFIG_DIR".into(),
                value: gh_config_dir,
            },
            FixedEnvironmentVariableV1 {
                key: "GH_HOST".into(),
                value: GITHUB_PUBLIC_HOSTNAME_V1.into(),
            },
            FixedEnvironmentVariableV1 {
                key: "GH_PROMPT_DISABLED".into(),
                value: "1".into(),
            },
            FixedEnvironmentVariableV1 {
                key: "HOME".into(),
                value: home_dir,
            },
            FixedEnvironmentVariableV1 {
                key: "NO_COLOR".into(),
                value: "1".into(),
            },
        ],
        inherited_secret_environment_keys: vec!["GH_TOKEN".into()],
        timeout_seconds: GITHUB_PUBLIC_PROCESS_TIMEOUT_SECONDS_V1,
        max_stdout_bytes: GITHUB_PUBLIC_MAX_STDOUT_BYTES_V1,
        max_stderr_bytes: GITHUB_PUBLIC_MAX_STDERR_BYTES_V1,
        expected_max_results: GITHUB_PUBLIC_MAX_ATTESTATIONS_V1,
        canonical_receipt_digest: receipt_digest,
    })
}

fn validate_absolute_path(
    field: &'static str,
    value: &str,
) -> Result<(), GitHubCommandPlanErrorV1> {
    if !value.starts_with('/') || value.contains('\0') || value.contains('\n') || value.contains('\r') {
        return Err(GitHubCommandPlanErrorV1::InvalidAbsolutePath { field });
    }
    Ok(())
}

fn valid_workflow_path(value: &str) -> bool {
    value.starts_with(".github/workflows/")
        && !value.contains("..")
        && !value.contains('\0')
        && !value.contains('\n')
        && !value.contains('\r')
}

fn workflow_revision_allowed(policy: &WorkflowRevisionPolicyV1, actual: GitObjectIdV1) -> bool {
    match policy {
        WorkflowRevisionPolicyV1::Exact(expected) => *expected == actual,
        WorkflowRevisionPolicyV1::Allowed(allowed) => allowed.contains(&actual),
    }
}

fn hash_vec(hasher: &mut Sha256, label: &str, values: &[String]) {
    hash_str(hasher, label);
    hasher.update((values.len() as u64).to_be_bytes());
    for value in values {
        hash_str(hasher, value);
    }
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthenticationFreshnessPolicyV1, NixVerifierClosureIdentityV1,
        QualificationResultV1, SourceRevisionPolicyV1, TransparencyPolicyV1,
        VerifierProfileV1,
    };

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn receipt() -> QualificationReceiptV1 {
        QualificationReceiptV1 {
            receipt_version: 1,
            qualification_profile: "myc-zkp-range-001aq".into(),
            statement_profile: "range-membership-v1".into(),
            theorem_profile: "range-membership-v1-air".into(),
            subject: GitObjectIdV1::sha1([0xaa; 20]),
            dependency_graph_digest: digest(1),
            measured_security_receipt_digest: digest(2),
            coherence_policy_id: "coherence-v1".into(),
            coherence_result_digest: digest(3),
            qualification_corpus_digest: digest(4),
            execution_capsule_digest: digest(5),
            result: QualificationResultV1::Pass,
            nonclaims: vec!["application authority not granted".into()],
        }
    }

    fn policy() -> ReceiptAuthenticationPolicyV1 {
        ReceiptAuthenticationPolicyV1 {
            profile_id: "github-public-v1".into(),
            verifier_profile: VerifierProfileV1 {
                profile_id: "github-attestation-verifier-v1".into(),
                backend_family: GITHUB_PUBLIC_BACKEND_FAMILY_V1.into(),
                backend_version: "2.101.0".into(),
                verification_profile: "public-sigstore-qualification-v1".into(),
            },
            expected_attestation_subject_name: MYCELIX_QUALIFICATION_SUBJECT_NAME_V1.into(),
            expected_predicate_type: MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1.into(),
            expected_predicate_schema: "mycelix-qualification-attestation-predicate-v1".into(),
            trusted_root_profile: "sigstore-public-good-v1".into(),
            trusted_root_digest: digest(9),
            oidc_issuer: GITHUB_ACTIONS_OIDC_ISSUER_V1.into(),
            source_repository: MYCELIX_REPOSITORY_V1.into(),
            source_repository_owner: MYCELIX_REPOSITORY_OWNER_V1.into(),
            signer_workflow: ".github/workflows/qualify.yml".into(),
            signer_workflow_revision: WorkflowRevisionPolicyV1::Exact(GitObjectIdV1::sha1([
                0xbb; 20
            ])),
            source_revision: SourceRevisionPolicyV1 {
                exact_commit: GitObjectIdV1::sha1([0xaa; 20]),
                exact_git_ref: Some("refs/heads/main".into()),
            },
            transparency_policy: TransparencyPolicyV1::PublicTransparencyRequired,
            freshness_policy: AuthenticationFreshnessPolicyV1::CurrentAtVerification,
        }
    }

    fn verifier() -> VerifierExecutableIdentityV1 {
        VerifierExecutableIdentityV1 {
            profile_id: "gh-2.101.0-linux-x86_64".into(),
            backend_family: GITHUB_PUBLIC_BACKEND_FAMILY_V1.into(),
            semantic_version: "2.101.0".into(),
            executable_sha256: digest(7),
            platform_profile: GITHUB_PUBLIC_PLATFORM_PROFILE_V1.into(),
            command_profile_id: GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1.into(),
            nix_closure: Some(NixVerifierClosureIdentityV1 {
                store_path: "/nix/store/example-gh/bin/gh".into(),
                closure_digest: digest(8),
            }),
        }
    }

    fn plan() -> GitHubPublicCommandPlanV1 {
        build_github_public_command_plan_v1(
            &policy(),
            &receipt(),
            &verifier(),
            GitObjectIdV1::sha1([0xbb; 20]),
            "/nix/store/example-gh/bin/gh",
            "/tmp/mycelix-gh-verify",
        )
        .unwrap()
    }

    #[test]
    fn exact_profile_builds_deterministic_explicit_plan() {
        let a = plan();
        let b = plan();
        assert_eq!(a, b);
        assert_eq!(a.authority_scope(), GitHubCommandPlanAuthorityV1::CommandPlanOnly);
        assert!(!a.executes_processes());
        assert!(!a.establishes_receipt_authentication());
        assert!(!a.grants_production_authority());
        assert!(a.clear_environment());
        assert_eq!(a.inherited_secret_environment_keys(), &["GH_TOKEN"]);
        assert!(a.verify_args().iter().any(|arg| arg == "--deny-self-hosted-runners"));
        assert!(a.verify_args().iter().any(|arg| arg == "--signer-digest"));
        assert!(a.verify_args().iter().any(|arg| arg == "--source-digest"));
        assert!(a.verify_args().iter().any(|arg| arg == "--source-ref"));
        assert!(a.verify_args().iter().any(|arg| arg == "--format"));
        assert_eq!(a.command_arguments_digest(), b.command_arguments_digest());
        assert_eq!(a.environment_profile_digest(), b.environment_profile_digest());
    }

    #[test]
    fn receipt_source_and_signer_revision_substitution_fail() {
        let mut wrong_receipt = receipt();
        wrong_receipt.subject = GitObjectIdV1::sha1([0xcc; 20]);
        assert_eq!(
            build_github_public_command_plan_v1(
                &policy(),
                &wrong_receipt,
                &verifier(),
                GitObjectIdV1::sha1([0xbb; 20]),
                "/nix/store/example-gh/bin/gh",
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubCommandPlanErrorV1::ReceiptSubjectDoesNotMatchSourceRevision)
        );

        assert_eq!(
            build_github_public_command_plan_v1(
                &policy(),
                &receipt(),
                &verifier(),
                GitObjectIdV1::sha1([0xcc; 20]),
                "/nix/store/example-gh/bin/gh",
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubCommandPlanErrorV1::SignerRevisionNotAllowed)
        );
    }

    #[test]
    fn repository_backend_and_path_substitution_fail() {
        let mut wrong = policy();
        wrong.source_repository = "attacker/repo".into();
        assert_eq!(
            build_github_public_command_plan_v1(
                &wrong,
                &receipt(),
                &verifier(),
                GitObjectIdV1::sha1([0xbb; 20]),
                "/nix/store/example-gh/bin/gh",
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubCommandPlanErrorV1::WrongRepository)
        );

        let mut wrong_verifier = verifier();
        wrong_verifier.command_profile_id = "other-profile".into();
        assert_eq!(
            build_github_public_command_plan_v1(
                &policy(),
                &receipt(),
                &wrong_verifier,
                GitObjectIdV1::sha1([0xbb; 20]),
                "/nix/store/example-gh/bin/gh",
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubCommandPlanErrorV1::WrongVerifierCommandProfile)
        );

        assert_eq!(
            build_github_public_command_plan_v1(
                &policy(),
                &receipt(),
                &verifier(),
                GitObjectIdV1::sha1([0xbb; 20]),
                "gh",
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubCommandPlanErrorV1::InvalidAbsolutePath {
                field: "executable_path"
            })
        );
    }

    #[test]
    fn secret_values_are_not_part_of_environment_profile() {
        let plan = plan();
        assert_eq!(plan.inherited_secret_environment_keys(), &["GH_TOKEN"]);
        for assignment in plan.fixed_environment() {
            assert_ne!(assignment.key(), "GH_TOKEN");
        }
    }
}
