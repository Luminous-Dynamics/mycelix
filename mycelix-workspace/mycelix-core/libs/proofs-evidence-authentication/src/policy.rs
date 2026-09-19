use std::collections::HashSet;

use serde::{Deserialize, Serialize};

use crate::{GitObjectIdV1, Sha256DigestV1};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierProfileV1 {
    pub profile_id: String,
    pub backend_family: String,
    pub backend_version: String,
    pub verification_profile: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum WorkflowRevisionPolicyV1 {
    Exact(GitObjectIdV1),
    Allowed(Vec<GitObjectIdV1>),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceRevisionPolicyV1 {
    pub exact_commit: GitObjectIdV1,
    /// `None` means policy does not constrain the ref name; commit identity is still exact.
    pub exact_git_ref: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TransparencyPolicyV1 {
    PublicTransparencyRequired,
    TimestampRequired,
    PublicTransparencyAndTimestampRequired,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthenticationFreshnessPolicyV1 {
    CurrentAtVerification,
    MaximumAgeSeconds(u64),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReceiptAuthenticationPolicyV1 {
    pub profile_id: String,
    pub verifier_profile: VerifierProfileV1,
    pub expected_predicate_type: String,
    pub expected_predicate_schema: String,
    pub trusted_root_profile: String,
    pub trusted_root_digest: Sha256DigestV1,
    pub oidc_issuer: String,
    pub source_repository: String,
    pub source_repository_owner: String,
    pub signer_workflow: String,
    pub signer_workflow_revision: WorkflowRevisionPolicyV1,
    pub source_revision: SourceRevisionPolicyV1,
    pub transparency_policy: TransparencyPolicyV1,
    pub freshness_policy: AuthenticationFreshnessPolicyV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthenticationPolicyErrorV1 {
    EmptyField { field: &'static str },
    EmptyWorkflowRevisionSet,
    DuplicateWorkflowRevision,
    ZeroMaximumAge,
}

impl ReceiptAuthenticationPolicyV1 {
    pub fn validate(&self) -> Result<(), AuthenticationPolicyErrorV1> {
        for (field, value) in [
            ("profile_id", self.profile_id.as_str()),
            (
                "verifier_profile.profile_id",
                self.verifier_profile.profile_id.as_str(),
            ),
            (
                "verifier_profile.backend_family",
                self.verifier_profile.backend_family.as_str(),
            ),
            (
                "verifier_profile.backend_version",
                self.verifier_profile.backend_version.as_str(),
            ),
            (
                "verifier_profile.verification_profile",
                self.verifier_profile.verification_profile.as_str(),
            ),
            ("expected_predicate_type", self.expected_predicate_type.as_str()),
            (
                "expected_predicate_schema",
                self.expected_predicate_schema.as_str(),
            ),
            ("trusted_root_profile", self.trusted_root_profile.as_str()),
            ("oidc_issuer", self.oidc_issuer.as_str()),
            ("source_repository", self.source_repository.as_str()),
            (
                "source_repository_owner",
                self.source_repository_owner.as_str(),
            ),
            ("signer_workflow", self.signer_workflow.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(AuthenticationPolicyErrorV1::EmptyField { field });
            }
        }

        if self
            .source_revision
            .exact_git_ref
            .as_ref()
            .is_some_and(|value| value.trim().is_empty())
        {
            return Err(AuthenticationPolicyErrorV1::EmptyField {
                field: "source_revision.exact_git_ref",
            });
        }

        if let WorkflowRevisionPolicyV1::Allowed(revisions) = &self.signer_workflow_revision {
            if revisions.is_empty() {
                return Err(AuthenticationPolicyErrorV1::EmptyWorkflowRevisionSet);
            }
            let unique: HashSet<_> = revisions.iter().collect();
            if unique.len() != revisions.len() {
                return Err(AuthenticationPolicyErrorV1::DuplicateWorkflowRevision);
            }
        }

        if matches!(
            self.freshness_policy,
            AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(0)
        ) {
            return Err(AuthenticationPolicyErrorV1::ZeroMaximumAge);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn policy() -> ReceiptAuthenticationPolicyV1 {
        ReceiptAuthenticationPolicyV1 {
            profile_id: "github-public-v1".into(),
            verifier_profile: VerifierProfileV1 {
                profile_id: "github-attestation-verifier-v1".into(),
                backend_family: "github-artifact-attestation".into(),
                backend_version: "unqualified-placeholder".into(),
                verification_profile: "public-sigstore-qualification-v1".into(),
            },
            expected_predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            expected_predicate_schema: "mycelix-qualification-v1".into(),
            trusted_root_profile: "sigstore-public-good-v1".into(),
            trusted_root_digest: digest(9),
            oidc_issuer: "https://token.actions.githubusercontent.com".into(),
            source_repository: "Luminous-Dynamics/mycelix".into(),
            source_repository_owner: "Luminous-Dynamics".into(),
            signer_workflow: ".github/workflows/proofs.yml".into(),
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

    #[test]
    fn exact_policy_is_valid() {
        assert_eq!(policy().validate(), Ok(()));
    }

    #[test]
    fn empty_and_duplicate_revision_sets_fail() {
        let mut p = policy();
        p.signer_workflow_revision = WorkflowRevisionPolicyV1::Allowed(Vec::new());
        assert_eq!(
            p.validate(),
            Err(AuthenticationPolicyErrorV1::EmptyWorkflowRevisionSet)
        );

        let mut p = policy();
        let revision = GitObjectIdV1::sha1([0xbb; 20]);
        p.signer_workflow_revision = WorkflowRevisionPolicyV1::Allowed(vec![revision, revision]);
        assert_eq!(
            p.validate(),
            Err(AuthenticationPolicyErrorV1::DuplicateWorkflowRevision)
        );
    }
}
