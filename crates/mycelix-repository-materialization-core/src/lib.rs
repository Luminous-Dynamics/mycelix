#![deny(unsafe_code)]

//! Pure semantic-publication and recovery protocol for bounded repository materialization.
//!
//! This crate deliberately performs no hashing, filesystem I/O, process execution,
//! network access, repository mutation, or remote observation. It canonicalizes the
//! exact semantic publication preimage and resolves already-authenticated publication
//! observations into typed outcomes.
//!
//! Protocol V1 defines:
//!
//! `SemanticPublicationIdV1 = SHA-256(canonical_preimage_v1)`
//!
//! Computing and authenticating that digest belongs to an enclosing verifier.

use std::fmt;

pub const SEMANTIC_PUBLICATION_PROFILE_V1: u16 = 1;
pub const SEMANTIC_PUBLICATION_DOMAIN_V1: &[u8] =
    b"mycelix/repository-materialization/semantic-publication/v1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RepositoryObjectAlgorithmV1 {
    GitSha1,
    GitSha256,
}

impl RepositoryObjectAlgorithmV1 {
    const fn tag(self) -> u8 {
        match self {
            Self::GitSha1 => 1,
            Self::GitSha256 => 2,
        }
    }

    const fn digest_len(self) -> usize {
        match self {
            Self::GitSha1 => 20,
            Self::GitSha256 => 32,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RepositoryObjectIdV1 {
    algorithm: RepositoryObjectAlgorithmV1,
    digest: Vec<u8>,
}

impl RepositoryObjectIdV1 {
    pub fn new(
        algorithm: RepositoryObjectAlgorithmV1,
        digest: Vec<u8>,
    ) -> Result<Self, MaterializationProtocolError> {
        if digest.len() != algorithm.digest_len() {
            return Err(MaterializationProtocolError::InvalidObjectIdLength);
        }
        Ok(Self { algorithm, digest })
    }

    pub fn git_sha1(digest: [u8; 20]) -> Self {
        Self {
            algorithm: RepositoryObjectAlgorithmV1::GitSha1,
            digest: digest.to_vec(),
        }
    }

    pub fn git_sha256(digest: [u8; 32]) -> Self {
        Self {
            algorithm: RepositoryObjectAlgorithmV1::GitSha256,
            digest: digest.to_vec(),
        }
    }

    pub fn algorithm(&self) -> RepositoryObjectAlgorithmV1 {
        self.algorithm
    }

    pub fn digest(&self) -> &[u8] {
        &self.digest
    }
}

/// Exact semantic change to one repository path.
///
/// Replacement identity binds both the repository's native object ID and an
/// independent SHA-256 of the exact file/symlink bytes. The native object ID is
/// required for repository plumbing; the independent content commitment prevents
/// semantic publication identity from relying on Git SHA-1 alone.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PathMutationV1 {
    path: String,
    mode: Option<u32>,
    object: Option<RepositoryObjectIdV1>,
    content_sha256: Option<[u8; 32]>,
}

impl PathMutationV1 {
    pub fn replace(
        path: impl Into<String>,
        mode: u32,
        object: RepositoryObjectIdV1,
        content_sha256: [u8; 32],
    ) -> Result<Self, MaterializationProtocolError> {
        let path = path.into();
        validate_repository_path(&path)?;
        if !matches!(mode, 0o100644 | 0o100755 | 0o120000) {
            return Err(MaterializationProtocolError::InvalidGitMode);
        }
        Ok(Self {
            path,
            mode: Some(mode),
            object: Some(object),
            content_sha256: Some(content_sha256),
        })
    }

    pub fn delete(path: impl Into<String>) -> Result<Self, MaterializationProtocolError> {
        let path = path.into();
        validate_repository_path(&path)?;
        Ok(Self {
            path,
            mode: None,
            object: None,
            content_sha256: None,
        })
    }

    pub fn path(&self) -> &str {
        &self.path
    }

    pub fn mode(&self) -> Option<u32> {
        self.mode
    }

    pub fn object(&self) -> Option<&RepositoryObjectIdV1> {
        self.object.as_ref()
    }

    pub fn content_sha256(&self) -> Option<&[u8; 32]> {
        self.content_sha256.as_ref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SemanticPublicationDescriptorV1 {
    repository: String,
    target_ref: String,
    expected_parent: RepositoryObjectIdV1,
    mutation_class: String,
    policy_commitment: [u8; 32],
    mutations: Vec<PathMutationV1>,
}

impl SemanticPublicationDescriptorV1 {
    pub fn new(
        repository: impl Into<String>,
        target_ref: impl Into<String>,
        expected_parent: RepositoryObjectIdV1,
        mutation_class: impl Into<String>,
        policy_commitment: [u8; 32],
        mutations: Vec<PathMutationV1>,
    ) -> Result<Self, MaterializationProtocolError> {
        let repository = repository.into();
        let target_ref = target_ref.into();
        let mutation_class = mutation_class.into();
        validate_repository_name(&repository)?;
        validate_target_ref(&target_ref)?;
        validate_semantic_token(&mutation_class)?;
        let mutations = normalize_mutations(mutations)?;
        if mutations.is_empty() {
            return Err(MaterializationProtocolError::EmptyMutationSet);
        }
        for mutation in &mutations {
            if let Some(object) = mutation.object() {
                if object.algorithm() != expected_parent.algorithm() {
                    return Err(MaterializationProtocolError::ObjectAlgorithmMismatch);
                }
            }
        }
        Ok(Self {
            repository,
            target_ref,
            expected_parent,
            mutation_class,
            policy_commitment,
            mutations,
        })
    }

    pub fn repository(&self) -> &str {
        &self.repository
    }

    pub fn target_ref(&self) -> &str {
        &self.target_ref
    }

    pub fn expected_parent(&self) -> &RepositoryObjectIdV1 {
        &self.expected_parent
    }

    pub fn mutation_class(&self) -> &str {
        &self.mutation_class
    }

    pub fn policy_commitment(&self) -> &[u8; 32] {
        &self.policy_commitment
    }

    pub fn mutations(&self) -> &[PathMutationV1] {
        &self.mutations
    }

    /// Canonical language-neutral preimage for `SemanticPublicationIdV1`.
    ///
    /// The enclosing verifier computes SHA-256 over these exact bytes. Physical
    /// commit metadata and the preferred commit ID are deliberately absent.
    pub fn canonical_preimage_v1(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(SEMANTIC_PUBLICATION_DOMAIN_V1);
        push_u16(&mut bytes, SEMANTIC_PUBLICATION_PROFILE_V1);
        push_frame(&mut bytes, self.repository.as_bytes());
        push_frame(&mut bytes, self.target_ref.as_bytes());
        push_object_id(&mut bytes, &self.expected_parent);
        push_frame(&mut bytes, self.mutation_class.as_bytes());
        bytes.extend_from_slice(&self.policy_commitment);
        push_u32(&mut bytes, self.mutations.len() as u32);
        for mutation in &self.mutations {
            push_frame(&mut bytes, mutation.path.as_bytes());
            match (
                &mutation.mode,
                &mutation.object,
                &mutation.content_sha256,
            ) {
                (Some(mode), Some(object), Some(content_sha256)) => {
                    bytes.push(1);
                    push_u32(&mut bytes, *mode);
                    push_object_id(&mut bytes, object);
                    bytes.extend_from_slice(content_sha256);
                }
                (None, None, None) => bytes.push(0),
                _ => unreachable!("PathMutationV1 constructors preserve replacement fields"),
            }
        }
        bytes
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidatePublicationV1 {
    descriptor: SemanticPublicationDescriptorV1,
    preferred_commit: Option<RepositoryObjectIdV1>,
}

impl CandidatePublicationV1 {
    pub fn new(
        descriptor: SemanticPublicationDescriptorV1,
        preferred_commit: Option<RepositoryObjectIdV1>,
    ) -> Result<Self, MaterializationProtocolError> {
        if let Some(commit) = &preferred_commit {
            if commit.algorithm() != descriptor.expected_parent().algorithm() {
                return Err(MaterializationProtocolError::ObjectAlgorithmMismatch);
            }
        }
        Ok(Self {
            descriptor,
            preferred_commit,
        })
    }

    pub fn descriptor(&self) -> &SemanticPublicationDescriptorV1 {
        &self.descriptor
    }

    pub fn preferred_commit(&self) -> Option<&RepositoryObjectIdV1> {
        self.preferred_commit.as_ref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RemoteCommitObservationV1 {
    commit_id: RepositoryObjectIdV1,
    parents: Vec<RepositoryObjectIdV1>,
    mutations: Vec<PathMutationV1>,
}

impl RemoteCommitObservationV1 {
    pub fn new(
        commit_id: RepositoryObjectIdV1,
        parents: Vec<RepositoryObjectIdV1>,
        mutations: Vec<PathMutationV1>,
    ) -> Result<Self, MaterializationProtocolError> {
        for parent in &parents {
            if parent.algorithm() != commit_id.algorithm() {
                return Err(MaterializationProtocolError::ObjectAlgorithmMismatch);
            }
        }
        let mutations = normalize_mutations(mutations)?;
        for mutation in &mutations {
            if let Some(object) = mutation.object() {
                if object.algorithm() != commit_id.algorithm() {
                    return Err(MaterializationProtocolError::ObjectAlgorithmMismatch);
                }
            }
        }
        Ok(Self {
            commit_id,
            parents,
            mutations,
        })
    }

    pub fn commit_id(&self) -> &RepositoryObjectIdV1 {
        &self.commit_id
    }

    pub fn parents(&self) -> &[RepositoryObjectIdV1] {
        &self.parents
    }

    pub fn mutations(&self) -> &[PathMutationV1] {
        &self.mutations
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RemotePublicationMatchV1 {
    ExactPhysicalPublication,
    SemanticallyEquivalentPublication,
    Conflict,
}

pub fn classify_remote_publication_v1(
    candidate: &CandidatePublicationV1,
    remote: &RemoteCommitObservationV1,
) -> RemotePublicationMatchV1 {
    let descriptor = candidate.descriptor();
    if remote.parents().len() != 1
        || remote.parents().first() != Some(descriptor.expected_parent())
        || remote.mutations() != descriptor.mutations()
    {
        return RemotePublicationMatchV1::Conflict;
    }
    if candidate.preferred_commit() == Some(remote.commit_id()) {
        RemotePublicationMatchV1::ExactPhysicalPublication
    } else {
        RemotePublicationMatchV1::SemanticallyEquivalentPublication
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateAvailabilityV1 {
    Available,
    Unavailable,
}

/// Result of the mutation attempt itself, before independent post-state observation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttemptDispositionV1 {
    NotAttempted,
    ConfirmedApplied,
    RejectedBeforeMutation,
    UnknownAfterAttempt,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PostPublicationStateV1 {
    ObservationUnavailable,
    AtExpectedParent,
    Commit(RemoteCommitObservationV1),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PublicationOutcomeV1 {
    Published,
    PublishedRecovered,
    PublishedEquivalent,
    PublishedRecoveredEquivalent,
    DefinitelyNotPublished,
    Conflict,
    IndeterminatePublication,
    CandidateUnavailable,
}

/// Resolve one bounded materialization transaction.
///
/// A transport/push error is represented by `UnknownAfterAttempt`; it can never
/// become `DefinitelyNotPublished` merely because the next observation still sees
/// the expected parent. Positive semantic equivalence requires exact parent and
/// mutation equality via `classify_remote_publication_v1`.
pub fn resolve_publication_v1(
    candidate: &CandidatePublicationV1,
    availability: CandidateAvailabilityV1,
    attempt: AttemptDispositionV1,
    post_state: &PostPublicationStateV1,
) -> PublicationOutcomeV1 {
    if let PostPublicationStateV1::Commit(remote) = post_state {
        return match classify_remote_publication_v1(candidate, remote) {
            RemotePublicationMatchV1::Conflict => PublicationOutcomeV1::Conflict,
            RemotePublicationMatchV1::ExactPhysicalPublication => match attempt {
                AttemptDispositionV1::ConfirmedApplied => PublicationOutcomeV1::Published,
                AttemptDispositionV1::UnknownAfterAttempt => {
                    PublicationOutcomeV1::PublishedRecovered
                }
                AttemptDispositionV1::NotAttempted
                | AttemptDispositionV1::RejectedBeforeMutation => {
                    PublicationOutcomeV1::PublishedEquivalent
                }
            },
            RemotePublicationMatchV1::SemanticallyEquivalentPublication => match attempt {
                AttemptDispositionV1::UnknownAfterAttempt => {
                    PublicationOutcomeV1::PublishedRecoveredEquivalent
                }
                _ => PublicationOutcomeV1::PublishedEquivalent,
            },
        };
    }

    if availability == CandidateAvailabilityV1::Unavailable
        && attempt == AttemptDispositionV1::NotAttempted
    {
        return PublicationOutcomeV1::CandidateUnavailable;
    }

    match attempt {
        AttemptDispositionV1::NotAttempted | AttemptDispositionV1::RejectedBeforeMutation => {
            PublicationOutcomeV1::DefinitelyNotPublished
        }
        AttemptDispositionV1::ConfirmedApplied | AttemptDispositionV1::UnknownAfterAttempt => {
            PublicationOutcomeV1::IndeterminatePublication
        }
    }
}

fn normalize_mutations(
    mut mutations: Vec<PathMutationV1>,
) -> Result<Vec<PathMutationV1>, MaterializationProtocolError> {
    mutations.sort_by(|left, right| left.path.cmp(&right.path));
    if mutations
        .windows(2)
        .any(|pair| pair[0].path == pair[1].path)
    {
        return Err(MaterializationProtocolError::DuplicateMutationPath);
    }
    Ok(mutations)
}

fn validate_repository_name(value: &str) -> Result<(), MaterializationProtocolError> {
    let Some((owner, repo)) = value.split_once('/') else {
        return Err(MaterializationProtocolError::InvalidRepositoryName);
    };
    if owner.is_empty()
        || repo.is_empty()
        || repo.contains('/')
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b'/'))
    {
        return Err(MaterializationProtocolError::InvalidRepositoryName);
    }
    Ok(())
}

fn validate_target_ref(value: &str) -> Result<(), MaterializationProtocolError> {
    let Some(name) = value.strip_prefix("refs/heads/") else {
        return Err(MaterializationProtocolError::InvalidTargetRef);
    };
    if name.is_empty()
        || name.ends_with('/')
        || name.ends_with('.')
        || name.contains("..")
        || name.contains("//")
        || name.contains("@{")
        || name.bytes().any(|byte| {
            byte <= 0x20
                || byte == 0x7f
                || matches!(byte, b'~' | b'^' | b':' | b'?' | b'*' | b'[' | b'\\')
        })
    {
        return Err(MaterializationProtocolError::InvalidTargetRef);
    }
    Ok(())
}

fn validate_semantic_token(value: &str) -> Result<(), MaterializationProtocolError> {
    if value.is_empty()
        || value.len() > 128
        || !value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b':' | b'/')
        })
    {
        return Err(MaterializationProtocolError::InvalidSemanticToken);
    }
    Ok(())
}

fn validate_repository_path(value: &str) -> Result<(), MaterializationProtocolError> {
    if value.is_empty()
        || value.starts_with('/')
        || value.ends_with('/')
        || value.contains('\\')
        || value.bytes().any(|byte| byte == 0 || byte.is_ascii_control())
        || value
            .split('/')
            .any(|segment| segment.is_empty() || segment == "." || segment == "..")
    {
        return Err(MaterializationProtocolError::InvalidRepositoryPath);
    }
    Ok(())
}

fn push_object_id(bytes: &mut Vec<u8>, object: &RepositoryObjectIdV1) {
    bytes.push(object.algorithm.tag());
    bytes.push(object.digest.len() as u8);
    bytes.extend_from_slice(&object.digest);
}

fn push_frame(bytes: &mut Vec<u8>, value: &[u8]) {
    push_u32(bytes, value.len() as u32);
    bytes.extend_from_slice(value);
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MaterializationProtocolError {
    InvalidObjectIdLength,
    InvalidGitMode,
    InvalidRepositoryName,
    InvalidTargetRef,
    InvalidSemanticToken,
    InvalidRepositoryPath,
    EmptyMutationSet,
    DuplicateMutationPath,
    ObjectAlgorithmMismatch,
}

impl fmt::Display for MaterializationProtocolError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::InvalidObjectIdLength => "repository object ID has the wrong digest length",
            Self::InvalidGitMode => "replacement uses a non-canonical Git file/symlink mode",
            Self::InvalidRepositoryName => "repository must be a bounded owner/name identifier",
            Self::InvalidTargetRef => "target must be a canonical refs/heads/* branch ref",
            Self::InvalidSemanticToken => "semantic token is empty, too long, or non-canonical",
            Self::InvalidRepositoryPath => "repository path is not a canonical relative UTF-8 path",
            Self::EmptyMutationSet => "semantic publication must mutate at least one path",
            Self::DuplicateMutationPath => "semantic publication mutates one path more than once",
            Self::ObjectAlgorithmMismatch => "repository object algorithms do not match",
        })
    }
}

impl std::error::Error for MaterializationProtocolError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn sha1(byte: u8) -> RepositoryObjectIdV1 {
        RepositoryObjectIdV1::git_sha1([byte; 20])
    }

    fn replace(path: &str, object_byte: u8, content_byte: u8) -> PathMutationV1 {
        PathMutationV1::replace(
            path,
            0o100644,
            sha1(object_byte),
            [content_byte; 32],
        )
        .unwrap()
    }

    fn descriptor_with(mutations: Vec<PathMutationV1>) -> SemanticPublicationDescriptorV1 {
        SemanticPublicationDescriptorV1::new(
            "Luminous-Dynamics/mycelix",
            "refs/heads/fix/example",
            sha1(0x11),
            "lock-only",
            [0x22; 32],
            mutations,
        )
        .unwrap()
    }

    fn one_file_descriptor() -> SemanticPublicationDescriptorV1 {
        descriptor_with(vec![replace("Cargo.lock", 0x33, 0x44)])
    }

    fn hex(bytes: &[u8]) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut output = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            output.push(HEX[(byte >> 4) as usize] as char);
            output.push(HEX[(byte & 0x0f) as usize] as char);
        }
        output
    }

    #[test]
    fn canonical_preimage_matches_frozen_language_neutral_vector() {
        let bytes = one_file_descriptor().canonical_preimage_v1();
        assert_eq!(bytes.len(), 260);
        assert_eq!(
            hex(&bytes),
            "6d7963656c69782f7265706f7369746f72792d6d6174657269616c697a6174696f6e2f73656d616e7469632d7075626c69636174696f6e2f7631000001000000194c756d696e6f75732d44796e616d6963732f6d7963656c697800000016726566732f68656164732f6669782f6578616d706c6501141111111111111111111111111111111111111111000000096c6f636b2d6f6e6c792222222222222222222222222222222222222222222222222222222222222222000000010000000a436172676f2e6c6f636b01000081a4011433333333333333333333333333333333333333334444444444444444444444444444444444444444444444444444444444444444"
        );
    }

    #[test]
    fn mutation_input_order_does_not_change_semantic_preimage() {
        let left = descriptor_with(vec![replace("b", 2, 12), replace("a", 1, 11)]);
        let right = descriptor_with(vec![replace("a", 1, 11), replace("b", 2, 12)]);
        assert_eq!(left.canonical_preimage_v1(), right.canonical_preimage_v1());
    }

    #[test]
    fn independent_content_commitment_changes_semantic_preimage() {
        let left = descriptor_with(vec![replace("Cargo.lock", 0x33, 0x44)]);
        let right = descriptor_with(vec![replace("Cargo.lock", 0x33, 0x45)]);
        assert_ne!(left.canonical_preimage_v1(), right.canonical_preimage_v1());
    }

    #[test]
    fn physical_commit_identity_is_not_part_of_semantic_identity() {
        let descriptor = one_file_descriptor();
        let left = CandidatePublicationV1::new(descriptor.clone(), Some(sha1(0x55))).unwrap();
        let right = CandidatePublicationV1::new(descriptor, Some(sha1(0x66))).unwrap();
        assert_eq!(
            left.descriptor().canonical_preimage_v1(),
            right.descriptor().canonical_preimage_v1()
        );
    }

    #[test]
    fn duplicate_path_fails_closed() {
        let result = SemanticPublicationDescriptorV1::new(
            "Luminous-Dynamics/mycelix",
            "refs/heads/fix/example",
            sha1(1),
            "two-files",
            [0; 32],
            vec![PathMutationV1::delete("same").unwrap(), replace("same", 2, 3)],
        );
        assert_eq!(
            result.unwrap_err(),
            MaterializationProtocolError::DuplicateMutationPath
        );
    }

    #[test]
    fn equivalent_remote_requires_exact_parent_and_mutations() {
        let descriptor = one_file_descriptor();
        let candidate = CandidatePublicationV1::new(descriptor.clone(), Some(sha1(0x55))).unwrap();
        let equivalent = RemoteCommitObservationV1::new(
            sha1(0x66),
            vec![descriptor.expected_parent().clone()],
            descriptor.mutations().to_vec(),
        )
        .unwrap();
        assert_eq!(
            classify_remote_publication_v1(&candidate, &equivalent),
            RemotePublicationMatchV1::SemanticallyEquivalentPublication
        );

        let wrong_parent = RemoteCommitObservationV1::new(
            sha1(0x66),
            vec![sha1(0x77)],
            descriptor.mutations().to_vec(),
        )
        .unwrap();
        assert_eq!(
            classify_remote_publication_v1(&candidate, &wrong_parent),
            RemotePublicationMatchV1::Conflict
        );

        let wrong_content = RemoteCommitObservationV1::new(
            sha1(0x66),
            vec![descriptor.expected_parent().clone()],
            vec![replace("Cargo.lock", 0x33, 0x45)],
        )
        .unwrap();
        assert_eq!(
            classify_remote_publication_v1(&candidate, &wrong_content),
            RemotePublicationMatchV1::Conflict
        );
    }

    #[test]
    fn confirmed_exact_publication_resolves_published() {
        let descriptor = one_file_descriptor();
        let candidate = CandidatePublicationV1::new(descriptor.clone(), Some(sha1(0x55))).unwrap();
        let remote = RemoteCommitObservationV1::new(
            sha1(0x55),
            vec![descriptor.expected_parent().clone()],
            descriptor.mutations().to_vec(),
        )
        .unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Available,
                AttemptDispositionV1::ConfirmedApplied,
                &PostPublicationStateV1::Commit(remote),
            ),
            PublicationOutcomeV1::Published
        );
    }

    #[test]
    fn unknown_attempt_plus_exact_observation_recovers_publication() {
        let descriptor = one_file_descriptor();
        let candidate = CandidatePublicationV1::new(descriptor.clone(), Some(sha1(0x55))).unwrap();
        let remote = RemoteCommitObservationV1::new(
            sha1(0x55),
            vec![descriptor.expected_parent().clone()],
            descriptor.mutations().to_vec(),
        )
        .unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Available,
                AttemptDispositionV1::UnknownAfterAttempt,
                &PostPublicationStateV1::Commit(remote),
            ),
            PublicationOutcomeV1::PublishedRecovered
        );
    }

    #[test]
    fn unknown_attempt_plus_equivalent_observation_recovers_equivalence() {
        let descriptor = one_file_descriptor();
        let candidate = CandidatePublicationV1::new(descriptor.clone(), Some(sha1(0x55))).unwrap();
        let remote = RemoteCommitObservationV1::new(
            sha1(0x66),
            vec![descriptor.expected_parent().clone()],
            descriptor.mutations().to_vec(),
        )
        .unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Available,
                AttemptDispositionV1::UnknownAfterAttempt,
                &PostPublicationStateV1::Commit(remote),
            ),
            PublicationOutcomeV1::PublishedRecoveredEquivalent
        );
    }

    #[test]
    fn push_error_cannot_be_laundered_into_definitely_not_published() {
        let candidate =
            CandidatePublicationV1::new(one_file_descriptor(), Some(sha1(0x55))).unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Available,
                AttemptDispositionV1::UnknownAfterAttempt,
                &PostPublicationStateV1::AtExpectedParent,
            ),
            PublicationOutcomeV1::IndeterminatePublication
        );
    }

    #[test]
    fn explicit_pre_mutation_rejection_is_definitely_not_published() {
        let candidate =
            CandidatePublicationV1::new(one_file_descriptor(), Some(sha1(0x55))).unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Available,
                AttemptDispositionV1::RejectedBeforeMutation,
                &PostPublicationStateV1::AtExpectedParent,
            ),
            PublicationOutcomeV1::DefinitelyNotPublished
        );
    }

    #[test]
    fn unavailable_candidate_is_not_misclassified_as_product_failure() {
        let candidate =
            CandidatePublicationV1::new(one_file_descriptor(), Some(sha1(0x55))).unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Unavailable,
                AttemptDispositionV1::NotAttempted,
                &PostPublicationStateV1::AtExpectedParent,
            ),
            PublicationOutcomeV1::CandidateUnavailable
        );
    }

    #[test]
    fn confirmed_apply_without_observable_post_state_is_indeterminate() {
        let candidate =
            CandidatePublicationV1::new(one_file_descriptor(), Some(sha1(0x55))).unwrap();
        assert_eq!(
            resolve_publication_v1(
                &candidate,
                CandidateAvailabilityV1::Available,
                AttemptDispositionV1::ConfirmedApplied,
                &PostPublicationStateV1::ObservationUnavailable,
            ),
            PublicationOutcomeV1::IndeterminatePublication
        );
    }
}
