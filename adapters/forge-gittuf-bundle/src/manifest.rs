// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_gittuf_adapter::REQUIRED_GITTUF_VERSION;
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    RepositoryVerificationError, RepositoryVerificationRequest,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

pub const OFFLINE_BUNDLE_SCHEMA_VERSION: u16 = 1;
pub const REQUIRED_BUNDLE_FORMAT: u8 = 3;
const MANIFEST_DOMAIN_V1: &[u8] = b"mycelix-forge/offline-gittuf-bundle/v1\0";

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct BundleRef {
    reference: RepositoryRef,
    tip: GitObjectId,
}

impl BundleRef {
    pub fn new(reference: RepositoryRef, tip: GitObjectId) -> Self {
        Self { reference, tip }
    }

    pub fn reference(&self) -> &RepositoryRef {
        &self.reference
    }

    pub fn tip(&self) -> &GitObjectId {
        &self.tip
    }
}

/// Canonical description of one self-contained offline verification artifact.
///
/// The Git bundle bytes are intentionally external to this object and are
/// bound by `bundle_digest` + `bundle_size`. The manifest embeds the complete
/// translated repository-policy state so an offline verifier has no hidden
/// policy-state dependency.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct OfflineBundleManifest {
    schema_version: u16,
    bundle_format: u8,
    adapter_version: String,
    request_digest: Digest,
    policy_state: RepositoryPolicyState,
    policy_state_digest: Digest,
    local_receipt_commitment: Digest,
    bundle_digest: Digest,
    bundle_size: u64,
    object_format: GitObjectAlgorithm,
    refs: Vec<BundleRef>,
}

impl OfflineBundleManifest {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        request: &RepositoryVerificationRequest,
        policy_state: RepositoryPolicyState,
        local_receipt_commitment: Digest,
        bundle_digest: Digest,
        bundle_size: u64,
        object_format: GitObjectAlgorithm,
        refs: Vec<BundleRef>,
    ) -> Result<Self, OfflineManifestError> {
        let request_digest = request.digest(DigestAlgorithm::Sha256)?;
        let policy_state_digest = policy_state.digest(request.repository_policy_state().algorithm())?;
        if &policy_state_digest != request.repository_policy_state() {
            return Err(OfflineManifestError::PolicyStateMismatch);
        }
        if policy_state.sequence() != request.repository_policy_sequence() {
            return Err(OfflineManifestError::PolicySequenceMismatch {
                request: request.repository_policy_sequence(),
                supplied: policy_state.sequence(),
            });
        }

        let manifest = Self {
            schema_version: OFFLINE_BUNDLE_SCHEMA_VERSION,
            bundle_format: REQUIRED_BUNDLE_FORMAT,
            adapter_version: REQUIRED_GITTUF_VERSION.to_owned(),
            request_digest,
            policy_state,
            policy_state_digest,
            local_receipt_commitment,
            bundle_digest,
            bundle_size,
            object_format,
            refs,
        };
        manifest.validate_internal()?;
        Ok(manifest)
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn policy_state(&self) -> &RepositoryPolicyState {
        &self.policy_state
    }

    pub fn policy_state_digest(&self) -> &Digest {
        &self.policy_state_digest
    }

    pub fn local_receipt_commitment(&self) -> &Digest {
        &self.local_receipt_commitment
    }

    pub fn bundle_digest(&self) -> &Digest {
        &self.bundle_digest
    }

    pub const fn bundle_size(&self) -> u64 {
        self.bundle_size
    }

    pub const fn object_format(&self) -> GitObjectAlgorithm {
        self.object_format
    }

    pub fn refs(&self) -> &[BundleRef] {
        &self.refs
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, OfflineManifestError> {
        self.validate_internal()?;
        let mut out = Vec::new();
        out.extend_from_slice(MANIFEST_DOMAIN_V1);
        out.extend_from_slice(&self.schema_version.to_be_bytes());
        out.push(self.bundle_format);
        push_string(&mut out, &self.adapter_version, "adapter_version")?;
        push_digest(&mut out, &self.request_digest)?;
        push_digest(&mut out, &self.policy_state_digest)?;
        push_digest(&mut out, &self.local_receipt_commitment)?;
        push_digest(&mut out, &self.bundle_digest)?;
        out.extend_from_slice(&self.bundle_size.to_be_bytes());
        out.push(object_algorithm_code(self.object_format));
        let count = u16::try_from(self.refs.len())
            .map_err(|_| OfflineManifestError::CanonicalFieldTooLarge("refs"))?;
        out.extend_from_slice(&count.to_be_bytes());
        for entry in &self.refs {
            push_string(&mut out, entry.reference.as_str(), "ref")?;
            push_git_object(&mut out, &entry.tip)?;
        }
        Ok(out)
    }

    pub fn commitment(&self, algorithm: DigestAlgorithm) -> Result<Digest, OfflineManifestError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }

    pub fn validate_for_request(
        &self,
        request: &RepositoryVerificationRequest,
    ) -> Result<(), OfflineManifestError> {
        self.validate_internal()?;
        let expected_request = request.digest(self.request_digest.algorithm())?;
        if expected_request != self.request_digest {
            return Err(OfflineManifestError::RequestMismatch);
        }
        if request.repository_policy_state() != &self.policy_state_digest {
            return Err(OfflineManifestError::PolicyStateMismatch);
        }
        if request.repository_policy_sequence() != self.policy_state.sequence() {
            return Err(OfflineManifestError::PolicySequenceMismatch {
                request: request.repository_policy_sequence(),
                supplied: self.policy_state.sequence(),
            });
        }
        Ok(())
    }

    fn validate_internal(&self) -> Result<(), OfflineManifestError> {
        if self.schema_version != OFFLINE_BUNDLE_SCHEMA_VERSION {
            return Err(OfflineManifestError::UnsupportedSchema(self.schema_version));
        }
        if self.bundle_format != REQUIRED_BUNDLE_FORMAT {
            return Err(OfflineManifestError::UnsupportedBundleFormat(self.bundle_format));
        }
        if self.adapter_version != REQUIRED_GITTUF_VERSION {
            return Err(OfflineManifestError::UnexpectedAdapterVersion(
                self.adapter_version.clone(),
            ));
        }
        if self.bundle_size == 0 {
            return Err(OfflineManifestError::EmptyBundle);
        }
        if self.bundle_digest.algorithm() != DigestAlgorithm::Sha256 {
            return Err(OfflineManifestError::BundleDigestMustBeSha256);
        }
        if self.local_receipt_commitment.algorithm() != DigestAlgorithm::Sha256 {
            return Err(OfflineManifestError::ReceiptCommitmentMustBeSha256);
        }

        let expected_policy_state = self
            .policy_state
            .digest(self.policy_state_digest.algorithm())?;
        if expected_policy_state != self.policy_state_digest {
            return Err(OfflineManifestError::PolicyStateMismatch);
        }

        if self.refs.is_empty() {
            return Err(OfflineManifestError::EmptyRefSet);
        }

        let mut previous: Option<&RepositoryRef> = None;
        let mut seen = BTreeSet::new();
        for entry in &self.refs {
            if entry.tip.algorithm() != self.object_format {
                return Err(OfflineManifestError::ObjectFormatMismatch);
            }
            if let Some(prior) = previous {
                if prior > entry.reference() {
                    return Err(OfflineManifestError::RefsNotCanonical);
                }
            }
            if !seen.insert(entry.reference().clone()) {
                return Err(OfflineManifestError::DuplicateRef(
                    entry.reference().to_string(),
                ));
            }
            previous = Some(entry.reference());
        }
        Ok(())
    }
}

impl<'de> Deserialize<'de> for OfflineBundleManifest {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireManifest {
            schema_version: u16,
            bundle_format: u8,
            adapter_version: String,
            request_digest: Digest,
            policy_state: RepositoryPolicyState,
            policy_state_digest: Digest,
            local_receipt_commitment: Digest,
            bundle_digest: Digest,
            bundle_size: u64,
            object_format: GitObjectAlgorithm,
            refs: Vec<BundleRef>,
        }

        let wire = WireManifest::deserialize(deserializer)?;
        let manifest = Self {
            schema_version: wire.schema_version,
            bundle_format: wire.bundle_format,
            adapter_version: wire.adapter_version,
            request_digest: wire.request_digest,
            policy_state: wire.policy_state,
            policy_state_digest: wire.policy_state_digest,
            local_receipt_commitment: wire.local_receipt_commitment,
            bundle_digest: wire.bundle_digest,
            bundle_size: wire.bundle_size,
            object_format: wire.object_format,
            refs: wire.refs,
        };
        manifest.validate_internal().map_err(D::Error::custom)?;
        Ok(manifest)
    }
}

pub fn canonicalize_refs(mut refs: Vec<BundleRef>) -> Result<Vec<BundleRef>, OfflineManifestError> {
    refs.sort();
    for pair in refs.windows(2) {
        if pair[0].reference == pair[1].reference {
            return Err(OfflineManifestError::DuplicateRef(
                pair[0].reference.to_string(),
            ));
        }
    }
    Ok(refs)
}

fn object_algorithm_code(algorithm: GitObjectAlgorithm) -> u8 {
    match algorithm {
        GitObjectAlgorithm::Sha1 => 1,
        GitObjectAlgorithm::Sha256 => 2,
    }
}

fn push_git_object(out: &mut Vec<u8>, object: &GitObjectId) -> Result<(), OfflineManifestError> {
    out.push(object_algorithm_code(object.algorithm()));
    let len = u16::try_from(object.as_bytes().len())
        .map_err(|_| OfflineManifestError::CanonicalFieldTooLarge("git_object"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(object.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), OfflineManifestError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| OfflineManifestError::CanonicalFieldTooLarge("digest_algorithm"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| OfflineManifestError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), OfflineManifestError> {
    let bytes = value.as_bytes();
    let len = u16::try_from(bytes.len())
        .map_err(|_| OfflineManifestError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum OfflineManifestError {
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error("unsupported offline-bundle manifest schema {0}")]
    UnsupportedSchema(u16),
    #[error("unsupported Git bundle format {0}")]
    UnsupportedBundleFormat(u8),
    #[error("unexpected gittuf adapter version {0}")]
    UnexpectedAdapterVersion(String),
    #[error("offline bundle may not be empty")]
    EmptyBundle,
    #[error("offline bundle artifact digest must use SHA-256")]
    BundleDigestMustBeSha256,
    #[error("local receipt commitment must use SHA-256")]
    ReceiptCommitmentMustBeSha256,
    #[error("embedded repository-policy state does not match its digest")]
    PolicyStateMismatch,
    #[error("repository-policy sequence mismatch: request={request}, supplied={supplied}")]
    PolicySequenceMismatch { request: u64, supplied: u64 },
    #[error("manifest does not bind the exact repository-verification request")]
    RequestMismatch,
    #[error("offline bundle must advertise at least one ref")]
    EmptyRefSet,
    #[error("duplicate bundle ref {0}")]
    DuplicateRef(String),
    #[error("bundle refs are not in canonical lexical order")]
    RefsNotCanonical,
    #[error("Git object format differs from manifest object format")]
    ObjectFormatMismatch,
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_repository::{RepositoryAdoption, RepositoryTip};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn request_and_policy() -> (RepositoryVerificationRequest, RepositoryPolicyState) {
        let policy = RepositoryPolicyState::new(project(), 0, None, digest(0x66)).unwrap();
        let adoption = RepositoryAdoption::new(
            project(),
            RepositoryTip::new(RepositoryRef::new("refs/heads/main").unwrap(), git(0x33)),
            digest(0x44),
            digest(0x55),
            digest(0x66),
            1000,
        );
        let request = RepositoryVerificationRequest::new(
            &adoption,
            git(0x33),
            git(0x77),
            digest(0x44),
            digest(0x55),
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        (request, policy)
    }

    fn manifest() -> OfflineBundleManifest {
        let (request, policy) = request_and_policy();
        let refs = canonicalize_refs(vec![
            BundleRef::new(RepositoryRef::new("refs/gittuf/policy").unwrap(), git(0x99)),
            BundleRef::new(request.reference().clone(), request.to().clone()),
        ])
        .unwrap();
        OfflineBundleManifest::new(
            &request,
            policy,
            digest(0xaa),
            digest(0xbb),
            1234,
            GitObjectAlgorithm::Sha1,
            refs,
        )
        .unwrap()
    }

    #[test]
    fn canonical_ref_order_is_stable() {
        let refs = canonicalize_refs(vec![
            BundleRef::new(RepositoryRef::new("refs/z").unwrap(), git(1)),
            BundleRef::new(RepositoryRef::new("refs/a").unwrap(), git(2)),
        ])
        .unwrap();
        assert_eq!(refs[0].reference().as_str(), "refs/a");
        assert_eq!(refs[1].reference().as_str(), "refs/z");
    }

    #[test]
    fn duplicate_refs_are_rejected() {
        let reference = RepositoryRef::new("refs/heads/main").unwrap();
        assert!(matches!(
            canonicalize_refs(vec![
                BundleRef::new(reference.clone(), git(1)),
                BundleRef::new(reference, git(2)),
            ]),
            Err(OfflineManifestError::DuplicateRef(_))
        ));
    }

    #[test]
    fn deserialization_revalidates_artifact_digest_algorithm() {
        let manifest = manifest();
        let mut value = serde_json::to_value(&manifest).unwrap();
        value["bundle_digest"]["algorithm"] = serde_json::json!("blake3-256");
        assert!(serde_json::from_value::<OfflineBundleManifest>(value).is_err());
    }

    #[test]
    fn manifest_rebinds_to_exact_request() {
        let manifest = manifest();
        let (request, _) = request_and_policy();
        manifest.validate_for_request(&request).unwrap();
    }
}
