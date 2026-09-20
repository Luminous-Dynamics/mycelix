use crate::{
    GitObjectIdV1, QualificationReceiptV1, ReceiptAuthenticationPolicyV1,
    VerifierExecutableIdentityV1,
};
use crate::github_cli_plan::{
    GITHUB_PUBLIC_BACKEND_FAMILY_V1, GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1,
    GITHUB_PUBLIC_PLATFORM_PROFILE_V1, GitHubCommandPlanErrorV1,
    GitHubPublicCommandPlanV1, build_github_public_command_plan_v1,
};

pub const MAX_GITHUB_EXECUTION_POLICY_TEXT_BYTES_V1: usize = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubVerifierExecutionPolicyAuthorityV1 {
    StructuralPolicyOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubVerifierExecutionPolicyErrorV1 {
    EmptyField { field: &'static str },
    FieldTooLong {
        field: &'static str,
        maximum: usize,
        actual: usize,
    },
    InvalidExpectedVerifier,
    WrongExpectedBackendFamily,
    WrongExpectedCommandProfile,
    WrongExpectedPlatformProfile,
    InvalidExecutablePath,
    NixExecutablePathMismatch,
    AuthenticationPolicyIdMismatch,
    AuthenticationVerifierProfileIdMismatch,
    AuthenticationVerificationProfileMismatch,
    AuthenticationBackendFamilyMismatch,
    AuthenticationBackendVersionMismatch,
    ObservedVerifierIdentityMismatch,
    CommandPlan(GitHubCommandPlanErrorV1),
}

/// Structural policy that admits one exact verifier executable identity for the
/// initial GitHub public-attestation command profile.
///
/// This is policy data, not an authentication capability. External callers may
/// construct policy through [`Self::new`], but doing so grants no authority. A
/// later production-admission theorem must bind the selected policy itself to a
/// qualified configuration/release lineage.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubPublicVerifierExecutionPolicyV1 {
    profile_id: String,
    authentication_policy_id: String,
    authentication_verifier_profile_id: String,
    authentication_verification_profile: String,
    expected_verifier: VerifierExecutableIdentityV1,
    executable_path: String,
}

impl GitHubPublicVerifierExecutionPolicyV1 {
    pub fn new(
        profile_id: impl Into<String>,
        authentication_policy_id: impl Into<String>,
        authentication_verifier_profile_id: impl Into<String>,
        authentication_verification_profile: impl Into<String>,
        expected_verifier: VerifierExecutableIdentityV1,
        executable_path: impl Into<String>,
    ) -> Result<Self, GitHubVerifierExecutionPolicyErrorV1> {
        let policy = Self {
            profile_id: profile_id.into(),
            authentication_policy_id: authentication_policy_id.into(),
            authentication_verifier_profile_id: authentication_verifier_profile_id.into(),
            authentication_verification_profile: authentication_verification_profile.into(),
            expected_verifier,
            executable_path: executable_path.into(),
        };
        policy.validate()?;
        Ok(policy)
    }

    pub fn validate(&self) -> Result<(), GitHubVerifierExecutionPolicyErrorV1> {
        check_text("profile_id", &self.profile_id)?;
        check_text("authentication_policy_id", &self.authentication_policy_id)?;
        check_text(
            "authentication_verifier_profile_id",
            &self.authentication_verifier_profile_id,
        )?;
        check_text(
            "authentication_verification_profile",
            &self.authentication_verification_profile,
        )?;
        check_text("executable_path", &self.executable_path)?;

        self.expected_verifier
            .validate()
            .map_err(|_| GitHubVerifierExecutionPolicyErrorV1::InvalidExpectedVerifier)?;

        if self.expected_verifier.backend_family != GITHUB_PUBLIC_BACKEND_FAMILY_V1 {
            return Err(GitHubVerifierExecutionPolicyErrorV1::WrongExpectedBackendFamily);
        }
        if self.expected_verifier.command_profile_id != GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1 {
            return Err(GitHubVerifierExecutionPolicyErrorV1::WrongExpectedCommandProfile);
        }
        if self.expected_verifier.platform_profile != GITHUB_PUBLIC_PLATFORM_PROFILE_V1 {
            return Err(GitHubVerifierExecutionPolicyErrorV1::WrongExpectedPlatformProfile);
        }
        if !valid_absolute_path(&self.executable_path) {
            return Err(GitHubVerifierExecutionPolicyErrorV1::InvalidExecutablePath);
        }
        if let Some(nix) = &self.expected_verifier.nix_closure {
            if nix.store_path != self.executable_path {
                return Err(GitHubVerifierExecutionPolicyErrorV1::NixExecutablePathMismatch);
            }
        }
        Ok(())
    }

    pub fn profile_id(&self) -> &str {
        &self.profile_id
    }

    pub fn authentication_policy_id(&self) -> &str {
        &self.authentication_policy_id
    }

    pub fn expected_verifier(&self) -> &VerifierExecutableIdentityV1 {
        &self.expected_verifier
    }

    pub fn executable_path(&self) -> &str {
        &self.executable_path
    }

    pub const fn authority_scope(&self) -> GitHubVerifierExecutionPolicyAuthorityV1 {
        GitHubVerifierExecutionPolicyAuthorityV1::StructuralPolicyOnly
    }

    pub const fn establishes_verifier_authenticity(&self) -> bool {
        false
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }
}

/// Build the deterministic GitHub CLI plan only after the observed verifier
/// identity exactly matches the structural execution policy.
pub fn build_github_public_command_plan_with_execution_policy_v1(
    execution_policy: &GitHubPublicVerifierExecutionPolicyV1,
    authentication_policy: &ReceiptAuthenticationPolicyV1,
    receipt: &QualificationReceiptV1,
    observed_verifier: &VerifierExecutableIdentityV1,
    selected_signer_revision: GitObjectIdV1,
    working_directory: &str,
) -> Result<GitHubPublicCommandPlanV1, GitHubVerifierExecutionPolicyErrorV1> {
    execution_policy.validate()?;

    if authentication_policy.profile_id != execution_policy.authentication_policy_id {
        return Err(GitHubVerifierExecutionPolicyErrorV1::AuthenticationPolicyIdMismatch);
    }
    if authentication_policy.verifier_profile.profile_id
        != execution_policy.authentication_verifier_profile_id
    {
        return Err(
            GitHubVerifierExecutionPolicyErrorV1::AuthenticationVerifierProfileIdMismatch,
        );
    }
    if authentication_policy.verifier_profile.verification_profile
        != execution_policy.authentication_verification_profile
    {
        return Err(
            GitHubVerifierExecutionPolicyErrorV1::AuthenticationVerificationProfileMismatch,
        );
    }
    if authentication_policy.verifier_profile.backend_family
        != execution_policy.expected_verifier.backend_family
    {
        return Err(
            GitHubVerifierExecutionPolicyErrorV1::AuthenticationBackendFamilyMismatch,
        );
    }
    if authentication_policy.verifier_profile.backend_version
        != execution_policy.expected_verifier.semantic_version
    {
        return Err(
            GitHubVerifierExecutionPolicyErrorV1::AuthenticationBackendVersionMismatch,
        );
    }
    if observed_verifier != &execution_policy.expected_verifier {
        return Err(GitHubVerifierExecutionPolicyErrorV1::ObservedVerifierIdentityMismatch);
    }

    build_github_public_command_plan_v1(
        authentication_policy,
        receipt,
        observed_verifier,
        selected_signer_revision,
        &execution_policy.executable_path,
        working_directory,
    )
    .map_err(GitHubVerifierExecutionPolicyErrorV1::CommandPlan)
}

fn check_text(
    field: &'static str,
    value: &str,
) -> Result<(), GitHubVerifierExecutionPolicyErrorV1> {
    if value.trim().is_empty() {
        return Err(GitHubVerifierExecutionPolicyErrorV1::EmptyField { field });
    }
    let actual = value.len();
    if actual > MAX_GITHUB_EXECUTION_POLICY_TEXT_BYTES_V1 {
        return Err(GitHubVerifierExecutionPolicyErrorV1::FieldTooLong {
            field,
            maximum: MAX_GITHUB_EXECUTION_POLICY_TEXT_BYTES_V1,
            actual,
        });
    }
    Ok(())
}

fn valid_absolute_path(value: &str) -> bool {
    value.starts_with('/')
        && !value.contains('\0')
        && !value.contains('\n')
        && !value.contains('\r')
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthenticationFreshnessPolicyV1, NixVerifierClosureIdentityV1,
        QualificationResultV1, Sha256DigestV1, SourceRevisionPolicyV1,
        TransparencyPolicyV1, VerifierProfileV1, WorkflowRevisionPolicyV1,
    };
    use crate::github_cli_plan::{
        GITHUB_ACTIONS_OIDC_ISSUER_V1, MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1,
        MYCELIX_QUALIFICATION_SUBJECT_NAME_V1, MYCELIX_REPOSITORY_OWNER_V1,
        MYCELIX_REPOSITORY_V1,
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

    fn execution_policy() -> GitHubPublicVerifierExecutionPolicyV1 {
        GitHubPublicVerifierExecutionPolicyV1::new(
            "github-public-exact-executable-v1",
            "github-public-v1",
            "github-attestation-verifier-v1",
            "public-sigstore-qualification-v1",
            verifier(),
            "/nix/store/example-gh/bin/gh",
        )
        .unwrap()
    }

    fn authentication_policy() -> ReceiptAuthenticationPolicyV1 {
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

    #[test]
    fn exact_observed_verifier_builds_structural_command_plan() {
        let policy = execution_policy();
        let plan = build_github_public_command_plan_with_execution_policy_v1(
            &policy,
            &authentication_policy(),
            &receipt(),
            &verifier(),
            GitObjectIdV1::sha1([0xbb; 20]),
            "/tmp/mycelix-gh-verify",
        )
        .unwrap();
        assert_eq!(
            policy.authority_scope(),
            GitHubVerifierExecutionPolicyAuthorityV1::StructuralPolicyOnly
        );
        assert!(!policy.establishes_verifier_authenticity());
        assert!(!policy.establishes_receipt_authentication());
        assert!(!policy.grants_production_authority());
        assert!(!plan.executes_processes());
        assert!(!plan.establishes_receipt_authentication());
    }

    #[test]
    fn same_version_different_binary_digest_is_rejected() {
        let mut observed = verifier();
        observed.executable_sha256 = digest(0xee);
        assert_eq!(
            build_github_public_command_plan_with_execution_policy_v1(
                &execution_policy(),
                &authentication_policy(),
                &receipt(),
                &observed,
                GitObjectIdV1::sha1([0xbb; 20]),
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubVerifierExecutionPolicyErrorV1::ObservedVerifierIdentityMismatch)
        );
    }

    #[test]
    fn authentication_backend_version_must_match_exact_executable_version() {
        let mut authentication = authentication_policy();
        authentication.verifier_profile.backend_version = "2.102.0".into();
        assert_eq!(
            build_github_public_command_plan_with_execution_policy_v1(
                &execution_policy(),
                &authentication,
                &receipt(),
                &verifier(),
                GitObjectIdV1::sha1([0xbb; 20]),
                "/tmp/mycelix-gh-verify",
            ),
            Err(GitHubVerifierExecutionPolicyErrorV1::AuthenticationBackendVersionMismatch)
        );
    }

    #[test]
    fn nix_execution_policy_binds_exact_executable_path() {
        assert_eq!(
            GitHubPublicVerifierExecutionPolicyV1::new(
                "github-public-exact-executable-v1",
                "github-public-v1",
                "github-attestation-verifier-v1",
                "public-sigstore-qualification-v1",
                verifier(),
                "/nix/store/other-gh/bin/gh",
            ),
            Err(GitHubVerifierExecutionPolicyErrorV1::NixExecutablePathMismatch)
        );
    }
}
