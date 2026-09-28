// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009F: exact Git `update-ref --stdin -z` transaction planning.
//!
//! This adapter turns one positive FORGE-009E atomic-consumption intent into an
//! exact, inspectable Git transaction plan. It deliberately does not spawn Git
//! or mutate a repository.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProtocolVersion};
use mycelix_forge_protected_ref_consumption::AtomicProtectedRefConsumptionIntentV1;
use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
use serde::Serialize;
use thiserror::Error;

const GIT_REF_TRANSACTION_PLAN_DOMAIN_V1: &[u8] =
    b"mycelix-forge/git-ref-transaction-plan/v1\0";

/// Exact argv passed after the configured absolute Git executable and optional
/// repository selection arguments.
pub const GIT_UPDATE_REF_TRANSACTION_ARGS: &[&str] =
    &["update-ref", "--no-deref", "--stdin", "-z"];

/// Exact non-authoritative Git transaction plan derived from a positive
/// FORGE-009E intent.
///
/// The plan has no `Deserialize` implementation and exposes only bytes that a
/// later sealed execution adapter may pass to Git after merge authorization is
/// established.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GitRefTransactionPlanV1 {
    version: ProtocolVersion,
    atomic_intent: Digest,
    target_ref: RepositoryRef,
    expected_base: GitObjectId,
    proposed_revision: GitObjectId,
    consumption_marker_ref: RepositoryRef,
    consumption_marker_value: GitObjectId,
    stdin_digest: Digest,
    stdin_bytes: Vec<u8>,
    evidence_commitment: Digest,
}

impl GitRefTransactionPlanV1 {
    /// Build an exact Git transaction plan from the positive FORGE-009E intent.
    pub fn new(
        intent: &AtomicProtectedRefConsumptionIntentV1,
    ) -> Result<Self, GitRefTransactionPlanError> {
        let algorithm = intent.protected_merge_request().commitment().algorithm();
        let atomic_intent = intent.digest(algorithm)?;
        Self::from_parts(
            atomic_intent,
            intent.target_ref().clone(),
            intent.expected_base().clone(),
            intent.proposed_revision().clone(),
            intent.consumption_marker_ref().clone(),
            intent.consumption_marker_value().clone(),
            algorithm,
        )
    }

    fn from_parts(
        atomic_intent: Digest,
        target_ref: RepositoryRef,
        expected_base: GitObjectId,
        proposed_revision: GitObjectId,
        consumption_marker_ref: RepositoryRef,
        consumption_marker_value: GitObjectId,
        algorithm: DigestAlgorithm,
    ) -> Result<Self, GitRefTransactionPlanError> {
        if expected_base == proposed_revision {
            return Err(GitRefTransactionPlanError::NoOpTransition);
        }
        let expected_algorithm = expected_base.algorithm();
        for (field, object) in [
            ("proposed_revision", &proposed_revision),
            ("consumption_marker_value", &consumption_marker_value),
        ] {
            if object.algorithm() != expected_algorithm {
                return Err(GitRefTransactionPlanError::GitObjectAlgorithmMismatch {
                    field,
                    expected: expected_algorithm,
                    actual: object.algorithm(),
                });
            }
        }
        if consumption_marker_value != proposed_revision {
            return Err(GitRefTransactionPlanError::MarkerValueMismatch);
        }
        if consumption_marker_ref == target_ref {
            return Err(GitRefTransactionPlanError::MarkerAliasesTargetRef);
        }

        let stdin_bytes = build_stdin_bytes(
            &target_ref,
            &expected_base,
            &proposed_revision,
            &consumption_marker_ref,
            &consumption_marker_value,
        );
        let stdin_digest = Digest::of_bytes(algorithm, &stdin_bytes);

        let mut canonical = Vec::new();
        canonical.extend_from_slice(GIT_REF_TRANSACTION_PLAN_DOMAIN_V1);
        canonical.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
        push_digest(&mut canonical, &atomic_intent)?;
        push_ref(&mut canonical, &target_ref)?;
        push_git_object(&mut canonical, &expected_base)?;
        push_git_object(&mut canonical, &proposed_revision)?;
        push_ref(&mut canonical, &consumption_marker_ref)?;
        push_git_object(&mut canonical, &consumption_marker_value)?;
        push_digest(&mut canonical, &stdin_digest)?;
        let evidence_commitment = Digest::of_bytes(algorithm, &canonical);

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            atomic_intent,
            target_ref,
            expected_base,
            proposed_revision,
            consumption_marker_ref,
            consumption_marker_value,
            stdin_digest,
            stdin_bytes,
            evidence_commitment,
        })
    }

    /// Exact FORGE-009E atomic-consumption intent commitment.
    pub fn atomic_intent(&self) -> &Digest {
        &self.atomic_intent
    }

    /// Protected target ref.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Exact old target value used as the update precondition.
    pub fn expected_base(&self) -> &GitObjectId {
        &self.expected_base
    }

    /// Exact new target value.
    pub fn proposed_revision(&self) -> &GitObjectId {
        &self.proposed_revision
    }

    /// Canonical per-request consumption-marker ref.
    pub fn consumption_marker_ref(&self) -> &RepositoryRef {
        &self.consumption_marker_ref
    }

    /// Exact value created in the marker ref.
    pub fn consumption_marker_value(&self) -> &GitObjectId {
        &self.consumption_marker_value
    }

    /// Exact argv passed to `git` after executable/repository selection.
    pub const fn command_args(&self) -> &'static [&'static str] {
        GIT_UPDATE_REF_TRANSACTION_ARGS
    }

    /// Exact NUL-framed stdin transaction bytes.
    pub fn stdin_bytes(&self) -> &[u8] {
        &self.stdin_bytes
    }

    /// Commitment to the exact stdin bytes.
    pub fn stdin_digest(&self) -> &Digest {
        &self.stdin_digest
    }

    /// Aggregate plan evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }

    /// Protocol version for the plan.
    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }
}

fn build_stdin_bytes(
    target_ref: &RepositoryRef,
    expected_base: &GitObjectId,
    proposed_revision: &GitObjectId,
    marker_ref: &RepositoryRef,
    marker_value: &GitObjectId,
) -> Vec<u8> {
    let expected_hex = hex::encode(expected_base.as_bytes());
    let proposed_hex = hex::encode(proposed_revision.as_bytes());
    let marker_hex = hex::encode(marker_value.as_bytes());

    let mut out = Vec::new();
    push_simple_command(&mut out, b"start");
    push_update_command(
        &mut out,
        target_ref.as_str(),
        &proposed_hex,
        &expected_hex,
    );
    push_create_command(&mut out, marker_ref.as_str(), &marker_hex);
    push_simple_command(&mut out, b"prepare");
    push_simple_command(&mut out, b"commit");
    out
}

fn push_simple_command(out: &mut Vec<u8>, command: &[u8]) {
    out.extend_from_slice(command);
    out.push(0);
}

fn push_update_command(out: &mut Vec<u8>, reference: &str, new_oid: &str, old_oid: &str) {
    out.extend_from_slice(b"update ");
    out.extend_from_slice(reference.as_bytes());
    out.push(0);
    out.extend_from_slice(new_oid.as_bytes());
    out.push(0);
    out.extend_from_slice(old_oid.as_bytes());
    out.push(0);
}

fn push_create_command(out: &mut Vec<u8>, reference: &str, new_oid: &str) {
    out.extend_from_slice(b"create ");
    out.extend_from_slice(reference.as_bytes());
    out.push(0);
    out.extend_from_slice(new_oid.as_bytes());
    out.push(0);
}

fn push_ref(out: &mut Vec<u8>, reference: &RepositoryRef) -> Result<(), GitRefTransactionPlanError> {
    push_bytes(out, "repository_ref", reference.as_str().as_bytes())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), GitRefTransactionPlanError> {
    out.push(object.algorithm().code());
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), GitRefTransactionPlanError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), GitRefTransactionPlanError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        GitRefTransactionPlanError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Git transaction-plan construction failures.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum GitRefTransactionPlanError {
    /// Target update is a no-op.
    #[error("Git ref transaction is a no-op")]
    NoOpTransition,
    /// A Git object field uses another hash algorithm.
    #[error("field {field} uses {actual:?}, expected {expected:?}")]
    GitObjectAlgorithmMismatch {
        /// Field containing the mismatch.
        field: &'static str,
        /// Expected algorithm.
        expected: GitObjectAlgorithm,
        /// Actual algorithm.
        actual: GitObjectAlgorithm,
    },
    /// Marker value differs from the proposed revision.
    #[error("consumption marker value differs from proposed revision")]
    MarkerValueMismatch,
    /// Marker ref aliases the protected target ref.
    #[error("consumption marker ref aliases protected target ref")]
    MarkerAliasesTargetRef,
    /// Canonical field exceeded v1 bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Actual byte length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// FORGE-009E intent canonicalization failed.
    #[error(transparent)]
    Consumption(#[from] mycelix_forge_protected_ref_consumption::ProtectedRefConsumptionError),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn plan() -> GitRefTransactionPlanV1 {
        GitRefTransactionPlanV1::from_parts(
            digest(0x10),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x20),
            git(0x21),
            RepositoryRef::new(
                "refs/mycelix/forge/consumed/sha256/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            )
            .unwrap(),
            git(0x21),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    #[test]
    fn exact_git_argv_is_fixed_and_shell_free() {
        let plan = plan();
        assert_eq!(
            plan.command_args(),
            ["update-ref", "--no-deref", "--stdin", "-z"]
        );
        assert!(!plan.command_args().iter().any(|arg| *arg == "sh" || *arg == "-c"));
    }

    #[test]
    fn exact_nul_framed_transaction_contains_update_create_prepare_commit() {
        let plan = plan();
        let target = "refs/heads/main";
        let marker = "refs/mycelix/forge/consumed/sha256/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let old_hex = hex::encode(git(0x20).as_bytes());
        let new_hex = hex::encode(git(0x21).as_bytes());

        let mut expected = Vec::new();
        push_simple_command(&mut expected, b"start");
        push_update_command(&mut expected, target, &new_hex, &old_hex);
        push_create_command(&mut expected, marker, &new_hex);
        push_simple_command(&mut expected, b"prepare");
        push_simple_command(&mut expected, b"commit");

        assert_eq!(plan.stdin_bytes(), expected);
        assert!(!plan.stdin_bytes().contains(&b'\n'));
    }

    #[test]
    fn stdin_digest_is_bound_into_plan_evidence() {
        let plan = plan();
        assert_eq!(
            plan.stdin_digest(),
            &Digest::of_bytes(DigestAlgorithm::Sha256, plan.stdin_bytes())
        );
        assert_ne!(plan.stdin_digest(), plan.evidence_commitment());
    }

    #[test]
    fn marker_alias_fails_closed() {
        let target = RepositoryRef::new("refs/heads/main").unwrap();
        let error = GitRefTransactionPlanV1::from_parts(
            digest(0x10),
            target.clone(),
            git(0x20),
            git(0x21),
            target,
            git(0x21),
            DigestAlgorithm::Sha256,
        )
        .unwrap_err();
        assert_eq!(error, GitRefTransactionPlanError::MarkerAliasesTargetRef);
    }

    #[test]
    fn marker_value_must_equal_proposed_revision() {
        let error = GitRefTransactionPlanV1::from_parts(
            digest(0x10),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x20),
            git(0x21),
            RepositoryRef::new("refs/mycelix/forge/consumed/sha256/marker").unwrap(),
            git(0x22),
            DigestAlgorithm::Sha256,
        )
        .unwrap_err();
        assert_eq!(error, GitRefTransactionPlanError::MarkerValueMismatch);
    }

    #[allow(dead_code)]
    fn constructor_requires_positive_forge_009e_intent(
        intent: &AtomicProtectedRefConsumptionIntentV1,
    ) {
        let _ = GitRefTransactionPlanV1::new(intent);
    }
}
