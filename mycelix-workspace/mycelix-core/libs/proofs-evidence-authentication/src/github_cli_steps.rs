use crate::github_cli_plan::{
    FixedEnvironmentVariableV1, GitHubPublicCommandPlanV1,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubVerifierStepPurposeV1 {
    DownloadAttestations,
    FetchTrustedRoot,
    VerifyAttestation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubVerifierStdoutDispositionV1 {
    CaptureBounded { maximum_bytes: usize },
    WriteFileBounded { path: String, maximum_bytes: usize },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubExecutionStepAuthorityV1 {
    ExecutionPlanOnly,
}

/// One deterministic process step derived from a sealed GitHub command plan.
///
/// The type binds cwd, argv, environment policy, timeout, stdout routing,
/// stderr bound, and expected output files. It executes nothing and grants no
/// authentication or production authority.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubVerifierExecutionStepV1 {
    purpose: GitHubVerifierStepPurposeV1,
    executable_path: String,
    working_directory: String,
    args: Vec<String>,
    clear_environment: bool,
    fixed_environment: Vec<FixedEnvironmentVariableV1>,
    inherited_secret_environment_keys: Vec<String>,
    timeout_seconds: u64,
    stdout: GitHubVerifierStdoutDispositionV1,
    max_stderr_bytes: usize,
    expected_output_files: Vec<String>,
}

impl GitHubVerifierExecutionStepV1 {
    pub const fn purpose(&self) -> GitHubVerifierStepPurposeV1 {
        self.purpose
    }

    pub fn executable_path(&self) -> &str {
        &self.executable_path
    }

    pub fn working_directory(&self) -> &str {
        &self.working_directory
    }

    pub fn args(&self) -> &[String] {
        &self.args
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

    pub fn stdout_disposition(&self) -> &GitHubVerifierStdoutDispositionV1 {
        &self.stdout
    }

    pub const fn max_stderr_bytes(&self) -> usize {
        self.max_stderr_bytes
    }

    pub fn expected_output_files(&self) -> &[String] {
        &self.expected_output_files
    }

    pub const fn authority_scope(&self) -> GitHubExecutionStepAuthorityV1 {
        GitHubExecutionStepAuthorityV1::ExecutionPlanOnly
    }

    pub const fn executes_process(&self) -> bool {
        false
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

/// Expand one sealed command plan into its exact three-process execution graph.
pub fn github_public_verifier_execution_steps_v1(
    plan: &GitHubPublicCommandPlanV1,
) -> [GitHubVerifierExecutionStepV1; 3] {
    let common = |purpose, args, stdout, expected_output_files| GitHubVerifierExecutionStepV1 {
        purpose,
        executable_path: plan.executable_path().into(),
        working_directory: plan.working_directory().into(),
        args,
        clear_environment: plan.clear_environment(),
        fixed_environment: plan.fixed_environment().to_vec(),
        inherited_secret_environment_keys: plan.inherited_secret_environment_keys().to_vec(),
        timeout_seconds: plan.timeout_seconds(),
        stdout,
        max_stderr_bytes: plan.max_stderr_bytes(),
        expected_output_files,
    };

    [
        common(
            GitHubVerifierStepPurposeV1::DownloadAttestations,
            plan.download_args().to_vec(),
            GitHubVerifierStdoutDispositionV1::CaptureBounded {
                maximum_bytes: plan.max_stdout_bytes(),
            },
            vec![plan.bundle_path().into()],
        ),
        common(
            GitHubVerifierStepPurposeV1::FetchTrustedRoot,
            plan.trusted_root_args().to_vec(),
            GitHubVerifierStdoutDispositionV1::WriteFileBounded {
                path: plan.trusted_root_path().into(),
                maximum_bytes: plan.max_stdout_bytes(),
            },
            vec![plan.trusted_root_path().into()],
        ),
        common(
            GitHubVerifierStepPurposeV1::VerifyAttestation,
            plan.verify_args().to_vec(),
            GitHubVerifierStdoutDispositionV1::CaptureBounded {
                maximum_bytes: plan.max_stdout_bytes(),
            },
            Vec::new(),
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthenticationFreshnessPolicyV1, GitObjectIdV1,
        GitHubPublicVerifierExecutionPolicyV1, NixVerifierClosureIdentityV1,
        QualificationReceiptV1, QualificationResultV1, ReceiptAuthenticationPolicyV1,
        Sha256DigestV1, SourceRevisionPolicyV1, TransparencyPolicyV1,
        VerifierExecutableIdentityV1, VerifierProfileV1, WorkflowRevisionPolicyV1,
        build_github_public_command_plan_with_execution_policy_v1,
    };
    use crate::github_cli_plan::{
        GITHUB_ACTIONS_OIDC_ISSUER_V1, GITHUB_PUBLIC_BACKEND_FAMILY_V1,
        GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1, GITHUB_PUBLIC_PLATFORM_PROFILE_V1,
        MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1, MYCELIX_QUALIFICATION_SUBJECT_NAME_V1,
        MYCELIX_REPOSITORY_OWNER_V1, MYCELIX_REPOSITORY_V1,
    };

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn verifier() -> VerifierExecutableIdentityV1 {
        VerifierExecutableIdentityV1 {
            profile_id: "gh-exact-v1".into(),
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
        let execution_policy = GitHubPublicVerifierExecutionPolicyV1::new(
            "github-public-exact-executable-v1",
            "github-public-v1",
            "github-attestation-verifier-v1",
            "public-sigstore-qualification-v1",
            verifier(),
            "/nix/store/example-gh/bin/gh",
        )
        .unwrap();
        let authentication_policy = ReceiptAuthenticationPolicyV1 {
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
        };
        let receipt = QualificationReceiptV1 {
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
        };
        build_github_public_command_plan_with_execution_policy_v1(
            &execution_policy,
            &authentication_policy,
            &receipt,
            &verifier(),
            GitObjectIdV1::sha1([0xbb; 20]),
            "/tmp/mycelix-gh-verify",
        )
        .unwrap()
    }

    #[test]
    fn execution_graph_binds_trusted_root_stdout_to_exact_file() {
        let plan = plan();
        let steps = github_public_verifier_execution_steps_v1(&plan);
        assert_eq!(steps[0].purpose(), GitHubVerifierStepPurposeV1::DownloadAttestations);
        assert_eq!(steps[1].purpose(), GitHubVerifierStepPurposeV1::FetchTrustedRoot);
        assert_eq!(steps[2].purpose(), GitHubVerifierStepPurposeV1::VerifyAttestation);
        assert_eq!(steps[0].expected_output_files(), &[plan.bundle_path()]);
        assert_eq!(steps[1].expected_output_files(), &[plan.trusted_root_path()]);
        assert!(matches!(
            steps[1].stdout_disposition(),
            GitHubVerifierStdoutDispositionV1::WriteFileBounded { path, .. }
                if path == plan.trusted_root_path()
        ));
        assert!(matches!(
            steps[2].stdout_disposition(),
            GitHubVerifierStdoutDispositionV1::CaptureBounded { .. }
        ));
        for step in &steps {
            assert_eq!(step.authority_scope(), GitHubExecutionStepAuthorityV1::ExecutionPlanOnly);
            assert!(!step.executes_process());
            assert!(!step.establishes_receipt_authentication());
            assert!(!step.grants_production_authority());
            assert!(!step.grants_application_authority());
        }
    }
}
