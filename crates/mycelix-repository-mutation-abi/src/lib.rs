#![deny(unsafe_code)]

//! Pure structural ABI for repository mutation subjects.
//!
//! This crate is intentionally dependency-free and grants no repository write
//! authority. It canonicalizes the *subject* of a proposed Git mutation while
//! keeping raw-file SHA-256, Git SHA-1 object identity, semantic commitments,
//! repository identity, ref leases, tree identity, and deterministic commit
//! metadata as distinct types.
//!
//! G3A deliberately does not:
//! - hash the canonical subject into a cryptographic candidate commitment;
//! - reconstruct or authenticate a Git tree;
//! - publish a Git ref;
//! - reconcile an uncertain publication;
//! - qualify the semantic correctness of mutated product bytes.
//!
//! Those are later QUAL-001G3 / CORE-COMMIT-001 adapter theorems.

use std::collections::BTreeSet;
use std::fmt;

pub const SUBJECT_PROFILE_REVISION_V1: u16 = 1;
pub const MAX_MUTATIONS_V1: usize = 4_096;
pub const MAX_REPOSITORY_COMPONENT_BYTES_V1: usize = 100;
pub const MAX_REF_BYTES_V1: usize = 512;
pub const MAX_PATH_BYTES_V1: usize = 4_096;
pub const MAX_PROFILE_ID_BYTES_V1: usize = 128;
pub const MAX_COMMIT_MESSAGE_BYTES_V1: usize = 65_536;

const SUBJECT_DOMAIN_V1: &[u8] = b"mycelix/repository-mutation-subject/v1\0";
const EXACT_PATH_POLICY_V1: &[u8] = b"exact-path-manifest-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GitSha1ObjectId([u8; 20]);

impl GitSha1ObjectId {
    pub const fn from_bytes(bytes: [u8; 20]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 20] {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RawFileSha256([u8; 32]);

impl RawFileSha256 {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SemanticCommitmentDigest([u8; 32]);

impl SemanticCommitmentDigest {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProfileId(String);

impl ProfileId {
    pub fn new(value: impl Into<String>) -> Result<Self, SubjectError> {
        let value = value.into();
        if !is_valid_ascii_token(&value, MAX_PROFILE_ID_BYTES_V1) {
            return Err(SubjectError::InvalidProfileId);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SemanticContentCommitmentV1 {
    profile: ProfileId,
    digest: SemanticCommitmentDigest,
}

impl SemanticContentCommitmentV1 {
    pub fn new(profile: ProfileId, digest: SemanticCommitmentDigest) -> Self {
        Self { profile, digest }
    }

    pub fn profile(&self) -> &ProfileId {
        &self.profile
    }

    pub const fn digest(&self) -> SemanticCommitmentDigest {
        self.digest
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RepositoryProviderV1 {
    GitHub,
}

impl RepositoryProviderV1 {
    const fn tag(self) -> u8 {
        match self {
            Self::GitHub => 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RepositoryIdentityV1 {
    provider: RepositoryProviderV1,
    owner: String,
    repository: String,
    repository_id: u64,
}

impl RepositoryIdentityV1 {
    pub fn github(
        owner: impl Into<String>,
        repository: impl Into<String>,
        repository_id: u64,
    ) -> Result<Self, SubjectError> {
        let owner = owner.into();
        let repository = repository.into();
        validate_repository_component(&owner)?;
        validate_repository_component(&repository)?;
        if repository_id == 0 {
            return Err(SubjectError::InvalidRepositoryIdentity);
        }
        Ok(Self {
            provider: RepositoryProviderV1::GitHub,
            owner,
            repository,
            repository_id,
        })
    }

    pub const fn provider(&self) -> RepositoryProviderV1 {
        self.provider
    }

    pub fn owner(&self) -> &str {
        &self.owner
    }

    pub fn repository(&self) -> &str {
        &self.repository
    }

    pub const fn repository_id(&self) -> u64 {
        self.repository_id
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GitHeadRefV1(String);

impl GitHeadRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, SubjectError> {
        let value = value.into();
        validate_head_ref(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RepositoryPathV1(String);

impl RepositoryPathV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, SubjectError> {
        let value = value.into();
        validate_repository_path(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GitFileModeV1 {
    Regular,
    Executable,
}

impl GitFileModeV1 {
    pub const fn numeric(self) -> u32 {
        match self {
            Self::Regular => 0o100644,
            Self::Executable => 0o100755,
        }
    }

    const fn tag(self) -> u8 {
        match self {
            Self::Regular => 1,
            Self::Executable => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FileMutationOperationV1 {
    Add,
    Replace,
    Delete,
}

impl FileMutationOperationV1 {
    const fn tag(self) -> u8 {
        match self {
            Self::Add => 1,
            Self::Replace => 2,
            Self::Delete => 3,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FileMutationV1 {
    path: RepositoryPathV1,
    operation: FileMutationOperationV1,
    mode: GitFileModeV1,
    prior_blob: Option<GitSha1ObjectId>,
    new_blob: Option<GitSha1ObjectId>,
    raw_content_sha256: Option<RawFileSha256>,
    semantic_content_commitment: Option<SemanticContentCommitmentV1>,
}

impl FileMutationV1 {
    pub fn add(
        path: RepositoryPathV1,
        mode: GitFileModeV1,
        new_blob: GitSha1ObjectId,
        raw_content_sha256: RawFileSha256,
        semantic_content_commitment: Option<SemanticContentCommitmentV1>,
    ) -> Self {
        Self {
            path,
            operation: FileMutationOperationV1::Add,
            mode,
            prior_blob: None,
            new_blob: Some(new_blob),
            raw_content_sha256: Some(raw_content_sha256),
            semantic_content_commitment,
        }
    }

    pub fn replace(
        path: RepositoryPathV1,
        mode: GitFileModeV1,
        prior_blob: GitSha1ObjectId,
        new_blob: GitSha1ObjectId,
        raw_content_sha256: RawFileSha256,
        semantic_content_commitment: Option<SemanticContentCommitmentV1>,
    ) -> Result<Self, SubjectError> {
        if prior_blob == new_blob {
            return Err(SubjectError::NoOpReplacement);
        }
        Ok(Self {
            path,
            operation: FileMutationOperationV1::Replace,
            mode,
            prior_blob: Some(prior_blob),
            new_blob: Some(new_blob),
            raw_content_sha256: Some(raw_content_sha256),
            semantic_content_commitment,
        })
    }

    pub fn delete(
        path: RepositoryPathV1,
        mode: GitFileModeV1,
        prior_blob: GitSha1ObjectId,
    ) -> Self {
        Self {
            path,
            operation: FileMutationOperationV1::Delete,
            mode,
            prior_blob: Some(prior_blob),
            new_blob: None,
            raw_content_sha256: None,
            semantic_content_commitment: None,
        }
    }

    pub fn path(&self) -> &RepositoryPathV1 {
        &self.path
    }

    pub const fn operation(&self) -> FileMutationOperationV1 {
        self.operation
    }

    pub const fn prior_blob(&self) -> Option<GitSha1ObjectId> {
        self.prior_blob
    }

    pub const fn new_blob(&self) -> Option<GitSha1ObjectId> {
        self.new_blob
    }

    pub const fn raw_content_sha256(&self) -> Option<RawFileSha256> {
        self.raw_content_sha256
    }

    fn validate_shape(&self) -> Result<(), SubjectError> {
        match self.operation {
            FileMutationOperationV1::Add => {
                if self.prior_blob.is_some()
                    || self.new_blob.is_none()
                    || self.raw_content_sha256.is_none()
                {
                    return Err(SubjectError::MutationShapeMismatch);
                }
            }
            FileMutationOperationV1::Replace => {
                let Some(prior) = self.prior_blob else {
                    return Err(SubjectError::MutationShapeMismatch);
                };
                let Some(new) = self.new_blob else {
                    return Err(SubjectError::MutationShapeMismatch);
                };
                if self.raw_content_sha256.is_none() || prior == new {
                    return Err(SubjectError::MutationShapeMismatch);
                }
            }
            FileMutationOperationV1::Delete => {
                if self.prior_blob.is_none()
                    || self.new_blob.is_some()
                    || self.raw_content_sha256.is_some()
                    || self.semantic_content_commitment.is_some()
                {
                    return Err(SubjectError::MutationShapeMismatch);
                }
            }
        }
        Ok(())
    }

    fn encode_into(&self, bytes: &mut Vec<u8>) {
        push_str(bytes, self.path.as_str());
        bytes.push(self.operation.tag());
        bytes.push(self.mode.tag());
        push_optional_git_id(bytes, self.prior_blob);
        push_optional_git_id(bytes, self.new_blob);
        push_optional_raw_sha256(bytes, self.raw_content_sha256);
        match &self.semantic_content_commitment {
            Some(value) => {
                bytes.push(1);
                push_str(bytes, value.profile.as_str());
                bytes.extend_from_slice(value.digest.as_bytes());
            }
            None => bytes.push(0),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitActorV1 {
    name: String,
    email: String,
    unix_seconds: i64,
    timezone_offset_minutes: i16,
}

impl GitActorV1 {
    pub fn new(
        name: impl Into<String>,
        email: impl Into<String>,
        unix_seconds: i64,
        timezone_offset_minutes: i16,
    ) -> Result<Self, SubjectError> {
        let name = name.into();
        let email = email.into();
        validate_git_actor_field(&name)?;
        validate_git_actor_field(&email)?;
        if email.contains('<') || email.contains('>') || !email.contains('@') {
            return Err(SubjectError::InvalidGitActor);
        }
        if !(-840..=840).contains(&timezone_offset_minutes) {
            return Err(SubjectError::InvalidGitActor);
        }
        Ok(Self {
            name,
            email,
            unix_seconds,
            timezone_offset_minutes,
        })
    }

    fn encode_into(&self, bytes: &mut Vec<u8>) {
        push_str(bytes, &self.name);
        push_str(bytes, &self.email);
        bytes.extend_from_slice(&self.unix_seconds.to_be_bytes());
        bytes.extend_from_slice(&self.timezone_offset_minutes.to_be_bytes());
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeterministicCommitProfileV1 {
    parent_commit: GitSha1ObjectId,
    candidate_tree: GitSha1ObjectId,
    message: String,
    author: GitActorV1,
    committer: GitActorV1,
}

impl DeterministicCommitProfileV1 {
    pub fn new(
        parent_commit: GitSha1ObjectId,
        candidate_tree: GitSha1ObjectId,
        message: impl Into<String>,
        author: GitActorV1,
        committer: GitActorV1,
    ) -> Result<Self, SubjectError> {
        let message = message.into();
        if message.is_empty()
            || message.len() > MAX_COMMIT_MESSAGE_BYTES_V1
            || message.as_bytes().contains(&0)
            || message.contains('\r')
        {
            return Err(SubjectError::InvalidCommitMessage);
        }
        Ok(Self {
            parent_commit,
            candidate_tree,
            message,
            author,
            committer,
        })
    }

    pub const fn parent_commit(&self) -> GitSha1ObjectId {
        self.parent_commit
    }

    pub const fn candidate_tree(&self) -> GitSha1ObjectId {
        self.candidate_tree
    }

    fn encode_into(&self, bytes: &mut Vec<u8>) {
        bytes.extend_from_slice(self.parent_commit.as_bytes());
        bytes.extend_from_slice(self.candidate_tree.as_bytes());
        push_str(bytes, &self.message);
        self.author.encode_into(bytes);
        self.committer.encode_into(bytes);
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RepositoryMutationTargetV1 {
    repository: RepositoryIdentityV1,
    target_ref: GitHeadRefV1,
    expected_old_ref_state: GitSha1ObjectId,
    expected_old_tree: GitSha1ObjectId,
}

impl RepositoryMutationTargetV1 {
    pub fn new(
        repository: RepositoryIdentityV1,
        target_ref: GitHeadRefV1,
        expected_old_ref_state: GitSha1ObjectId,
        expected_old_tree: GitSha1ObjectId,
    ) -> Self {
        Self {
            repository,
            target_ref,
            expected_old_ref_state,
            expected_old_tree,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidateMutationSetV1 {
    candidate_tree: GitSha1ObjectId,
    mutations: Vec<FileMutationV1>,
}

impl CandidateMutationSetV1 {
    pub fn new(candidate_tree: GitSha1ObjectId, mutations: Vec<FileMutationV1>) -> Self {
        Self {
            candidate_tree,
            mutations,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawRepositoryMutationSubjectV1 {
    profile_revision: u16,
    target: RepositoryMutationTargetV1,
    candidate: CandidateMutationSetV1,
    derivation_profile: ProfileId,
    commit_profile: DeterministicCommitProfileV1,
}

impl RawRepositoryMutationSubjectV1 {
    pub fn new(
        profile_revision: u16,
        target: RepositoryMutationTargetV1,
        candidate: CandidateMutationSetV1,
        derivation_profile: ProfileId,
        commit_profile: DeterministicCommitProfileV1,
    ) -> Self {
        Self {
            profile_revision,
            target,
            candidate,
            derivation_profile,
            commit_profile,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedRepositoryMutationSubjectV1 {
    repository: RepositoryIdentityV1,
    target_ref: GitHeadRefV1,
    expected_old_ref_state: GitSha1ObjectId,
    subject_parent_commit: GitSha1ObjectId,
    expected_old_tree: GitSha1ObjectId,
    candidate_tree: GitSha1ObjectId,
    mutations: Vec<FileMutationV1>,
    derivation_profile: ProfileId,
    commit_profile: DeterministicCommitProfileV1,
    canonical_preimage: Vec<u8>,
}

impl QualifiedRepositoryMutationSubjectV1 {
    pub fn repository(&self) -> &RepositoryIdentityV1 {
        &self.repository
    }

    pub fn target_ref(&self) -> &GitHeadRefV1 {
        &self.target_ref
    }

    pub const fn expected_old_ref_state(&self) -> GitSha1ObjectId {
        self.expected_old_ref_state
    }

    pub const fn subject_parent_commit(&self) -> GitSha1ObjectId {
        self.subject_parent_commit
    }

    pub const fn expected_old_tree(&self) -> GitSha1ObjectId {
        self.expected_old_tree
    }

    pub const fn candidate_tree(&self) -> GitSha1ObjectId {
        self.candidate_tree
    }

    pub fn mutations(&self) -> &[FileMutationV1] {
        &self.mutations
    }

    pub fn derivation_profile(&self) -> &ProfileId {
        &self.derivation_profile
    }

    pub fn commit_profile(&self) -> &DeterministicCommitProfileV1 {
        &self.commit_profile
    }

    pub fn canonical_preimage(&self) -> &[u8] {
        &self.canonical_preimage
    }

    pub const fn candidate_commitment_qualified_here(&self) -> bool {
        false
    }

    pub const fn git_tree_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn git_commit_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn grants_repository_write_authority(&self) -> bool {
        false
    }

    pub const fn publication_performed_here(&self) -> bool {
        false
    }

    pub const fn semantic_product_qualification_performed_here(&self) -> bool {
        false
    }
}

pub fn qualify_repository_mutation_subject_v1(
    raw: RawRepositoryMutationSubjectV1,
) -> Result<QualifiedRepositoryMutationSubjectV1, SubjectError> {
    if raw.profile_revision != SUBJECT_PROFILE_REVISION_V1 {
        return Err(SubjectError::UnsupportedProfileRevision);
    }
    if raw.candidate.mutations.is_empty() {
        return Err(SubjectError::EmptyMutationSet);
    }
    if raw.candidate.mutations.len() > MAX_MUTATIONS_V1 {
        return Err(SubjectError::TooManyMutations);
    }
    if raw.target.expected_old_tree == raw.candidate.candidate_tree {
        return Err(SubjectError::UnchangedCandidateTree);
    }
    if raw.commit_profile.parent_commit != raw.target.expected_old_ref_state {
        return Err(SubjectError::CommitParentMismatch);
    }
    if raw.commit_profile.candidate_tree != raw.candidate.candidate_tree {
        return Err(SubjectError::CommitTreeMismatch);
    }

    let mut mutations = raw.candidate.mutations;
    for mutation in &mutations {
        mutation.validate_shape()?;
    }
    mutations.sort_by(|left, right| left.path.cmp(&right.path));

    let mut seen = BTreeSet::new();
    for mutation in &mutations {
        if !seen.insert(mutation.path.clone()) {
            return Err(SubjectError::DuplicatePath);
        }
    }

    let canonical_preimage = canonical_subject_preimage(
        &raw.target,
        raw.candidate.candidate_tree,
        &mutations,
        &raw.derivation_profile,
        &raw.commit_profile,
    );

    Ok(QualifiedRepositoryMutationSubjectV1 {
        repository: raw.target.repository,
        target_ref: raw.target.target_ref,
        expected_old_ref_state: raw.target.expected_old_ref_state,
        subject_parent_commit: raw.target.expected_old_ref_state,
        expected_old_tree: raw.target.expected_old_tree,
        candidate_tree: raw.candidate.candidate_tree,
        mutations,
        derivation_profile: raw.derivation_profile,
        commit_profile: raw.commit_profile,
        canonical_preimage,
    })
}

fn canonical_subject_preimage(
    target: &RepositoryMutationTargetV1,
    candidate_tree: GitSha1ObjectId,
    mutations: &[FileMutationV1],
    derivation_profile: &ProfileId,
    commit_profile: &DeterministicCommitProfileV1,
) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(SUBJECT_DOMAIN_V1);
    bytes.extend_from_slice(&SUBJECT_PROFILE_REVISION_V1.to_be_bytes());

    bytes.push(target.repository.provider.tag());
    push_str(&mut bytes, &target.repository.owner);
    push_str(&mut bytes, &target.repository.repository);
    bytes.extend_from_slice(&target.repository.repository_id.to_be_bytes());

    push_str(&mut bytes, target.target_ref.as_str());
    bytes.extend_from_slice(target.expected_old_ref_state.as_bytes());
    bytes.extend_from_slice(target.expected_old_ref_state.as_bytes());
    bytes.extend_from_slice(target.expected_old_tree.as_bytes());
    bytes.extend_from_slice(candidate_tree.as_bytes());

    push_bytes(&mut bytes, EXACT_PATH_POLICY_V1);
    bytes.extend_from_slice(&(mutations.len() as u32).to_be_bytes());
    for mutation in mutations {
        mutation.encode_into(&mut bytes);
    }

    push_str(&mut bytes, derivation_profile.as_str());
    commit_profile.encode_into(&mut bytes);
    bytes
}

fn validate_repository_component(value: &str) -> Result<(), SubjectError> {
    if value.is_empty()
        || value.len() > MAX_REPOSITORY_COMPONENT_BYTES_V1
        || value == "."
        || value == ".."
        || value.ends_with(".git")
        || value.starts_with('.')
        || value.ends_with('.')
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.'))
    {
        return Err(SubjectError::InvalidRepositoryIdentity);
    }
    Ok(())
}

fn validate_head_ref(value: &str) -> Result<(), SubjectError> {
    if value.len() > MAX_REF_BYTES_V1 || !value.starts_with("refs/heads/") {
        return Err(SubjectError::InvalidHeadRef);
    }
    let suffix = &value["refs/heads/".len()..];
    if suffix.is_empty()
        || suffix.starts_with('/')
        || suffix.ends_with('/')
        || suffix.contains("//")
        || suffix.contains("..")
        || suffix.contains("@{")
        || suffix.ends_with(".lock")
        || suffix.bytes().any(|byte| {
            byte < 0x20
                || byte == 0x7f
                || matches!(
                    byte,
                    b' ' | b'~' | b'^' | b':' | b'?' | b'*' | b'[' | b'\\'
                )
        })
    {
        return Err(SubjectError::InvalidHeadRef);
    }
    for segment in suffix.split('/') {
        if segment.is_empty()
            || segment == "."
            || segment == ".."
            || segment.starts_with('.')
            || segment.ends_with('.')
            || segment.ends_with(".lock")
        {
            return Err(SubjectError::InvalidHeadRef);
        }
    }
    Ok(())
}

fn validate_repository_path(value: &str) -> Result<(), SubjectError> {
    if value.is_empty()
        || value.len() > MAX_PATH_BYTES_V1
        || value.starts_with('/')
        || value.ends_with('/')
        || value.contains('\\')
        || value.as_bytes().contains(&0)
        || value.bytes().any(|byte| byte < 0x20 || byte == 0x7f)
    {
        return Err(SubjectError::InvalidRepositoryPath);
    }

    for segment in value.split('/') {
        if segment.is_empty()
            || segment == "."
            || segment == ".."
            || segment.eq_ignore_ascii_case(".git")
        {
            return Err(SubjectError::InvalidRepositoryPath);
        }
    }
    Ok(())
}

fn is_valid_ascii_token(value: &str, maximum: usize) -> bool {
    !value.is_empty()
        && value.len() <= maximum
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b'/'))
}

fn validate_git_actor_field(value: &str) -> Result<(), SubjectError> {
    if value.is_empty()
        || value.len() > 320
        || value
            .bytes()
            .any(|byte| byte == 0 || byte == b'\n' || byte == b'\r')
    {
        return Err(SubjectError::InvalidGitActor);
    }
    Ok(())
}

fn push_str(bytes: &mut Vec<u8>, value: &str) {
    push_bytes(bytes, value.as_bytes());
}

fn push_bytes(bytes: &mut Vec<u8>, value: &[u8]) {
    let length = u32::try_from(value.len()).expect("V1 bounds fit in u32");
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value);
}

fn push_optional_git_id(bytes: &mut Vec<u8>, value: Option<GitSha1ObjectId>) {
    match value {
        Some(value) => {
            bytes.push(1);
            bytes.extend_from_slice(value.as_bytes());
        }
        None => bytes.push(0),
    }
}

fn push_optional_raw_sha256(bytes: &mut Vec<u8>, value: Option<RawFileSha256>) {
    match value {
        Some(value) => {
            bytes.push(1);
            bytes.extend_from_slice(value.as_bytes());
        }
        None => bytes.push(0),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubjectError {
    UnsupportedProfileRevision,
    InvalidRepositoryIdentity,
    InvalidHeadRef,
    InvalidRepositoryPath,
    InvalidProfileId,
    InvalidGitActor,
    InvalidCommitMessage,
    EmptyMutationSet,
    TooManyMutations,
    MutationShapeMismatch,
    NoOpReplacement,
    DuplicatePath,
    UnchangedCandidateTree,
    CommitParentMismatch,
    CommitTreeMismatch,
}

impl fmt::Display for SubjectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::UnsupportedProfileRevision => "unsupported repository mutation subject profile",
            Self::InvalidRepositoryIdentity => "invalid repository identity",
            Self::InvalidHeadRef => "invalid or non-canonical refs/heads Git ref",
            Self::InvalidRepositoryPath => "invalid or unsafe repository path",
            Self::InvalidProfileId => "invalid semantic profile identifier",
            Self::InvalidGitActor => "invalid deterministic Git actor",
            Self::InvalidCommitMessage => "invalid deterministic Git commit message",
            Self::EmptyMutationSet => "repository mutation set is empty",
            Self::TooManyMutations => "repository mutation count exceeds V1 bound",
            Self::MutationShapeMismatch => "file mutation operation does not match its typed fields",
            Self::NoOpReplacement => "replacement reuses the prior Git blob identity",
            Self::DuplicatePath => "repository mutation manifest contains a duplicate path",
            Self::UnchangedCandidateTree => "candidate tree equals the old tree despite a mutation set",
            Self::CommitParentMismatch => {
                "deterministic commit parent differs from expected old ref state"
            }
            Self::CommitTreeMismatch => "deterministic commit tree differs from candidate tree",
        };
        f.write_str(message)
    }
}

impl std::error::Error for SubjectError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn git(byte: u8) -> GitSha1ObjectId {
        GitSha1ObjectId::from_bytes([byte; 20])
    }

    fn raw(byte: u8) -> RawFileSha256 {
        RawFileSha256::from_bytes([byte; 32])
    }

    fn path(value: &str) -> RepositoryPathV1 {
        RepositoryPathV1::new(value).expect("static valid path")
    }

    fn actor() -> GitActorV1 {
        GitActorV1::new("Qualification Bot", "qual@example.invalid", 1_789_700_000, 0)
            .expect("static actor")
    }

    fn mutation(path_value: &str, byte: u8) -> FileMutationV1 {
        FileMutationV1::replace(
            path(path_value),
            GitFileModeV1::Regular,
            git(byte),
            git(byte.wrapping_add(1)),
            raw(byte),
            None,
        )
        .expect("valid replacement")
    }

    fn subject_with(mutations: Vec<FileMutationV1>) -> RawRepositoryMutationSubjectV1 {
        let parent = git(10);
        let candidate_tree = git(12);
        RawRepositoryMutationSubjectV1::new(
            SUBJECT_PROFILE_REVISION_V1,
            RepositoryMutationTargetV1::new(
                RepositoryIdentityV1::github("Luminous-Dynamics", "mycelix", 1_176_351_975)
                    .expect("repository"),
                GitHeadRefV1::new("refs/heads/fix/example").expect("ref"),
                parent,
                git(11),
            ),
            CandidateMutationSetV1::new(candidate_tree, mutations),
            ProfileId::new("qual-001g3a-test-v1").expect("profile"),
            DeterministicCommitProfileV1::new(
                parent,
                candidate_tree,
                "fix: deterministic candidate",
                actor(),
                actor(),
            )
            .expect("commit profile"),
        )
    }

    #[test]
    fn manifest_order_does_not_change_canonical_preimage() {
        let first = qualify_repository_mutation_subject_v1(subject_with(vec![
            mutation("b/file.rs", 30),
            mutation("a/file.rs", 20),
        ]))
        .expect("valid subject");

        let second = qualify_repository_mutation_subject_v1(subject_with(vec![
            mutation("a/file.rs", 20),
            mutation("b/file.rs", 30),
        ]))
        .expect("valid subject");

        assert_eq!(first.canonical_preimage(), second.canonical_preimage());
        assert_eq!(first.mutations()[0].path().as_str(), "a/file.rs");
    }

    #[test]
    fn duplicate_path_fails_closed() {
        let result = qualify_repository_mutation_subject_v1(subject_with(vec![
            mutation("src/lib.rs", 20),
            mutation("src/lib.rs", 30),
        ]));
        assert_eq!(result, Err(SubjectError::DuplicatePath));
    }

    #[test]
    fn unsafe_paths_fail_closed() {
        for candidate in [
            "",
            "/absolute",
            "a//b",
            "a/../b",
            "a/./b",
            "a\\b",
            ".git/config",
            "a/.GIT/config",
        ] {
            assert!(
                RepositoryPathV1::new(candidate).is_err(),
                "accepted unsafe path {candidate:?}"
            );
        }
    }

    #[test]
    fn ambiguous_or_non_head_refs_fail_closed() {
        for candidate in [
            "main",
            "refs/tags/v1",
            "refs/heads/a..b",
            "refs/heads/a//b",
            "refs/heads/.hidden",
            "refs/heads/a.lock",
            "refs/heads/a b",
        ] {
            assert!(
                GitHeadRefV1::new(candidate).is_err(),
                "accepted unsafe ref {candidate:?}"
            );
        }
    }

    #[test]
    fn raw_file_digest_and_git_object_identity_are_distinct_types() {
        let raw_digest = RawFileSha256::from_bytes([7; 32]);
        let git_id = GitSha1ObjectId::from_bytes([7; 20]);
        assert_eq!(raw_digest.as_bytes().len(), 32);
        assert_eq!(git_id.as_bytes().len(), 20);
    }

    #[test]
    fn no_op_replace_is_rejected() {
        assert_eq!(
            FileMutationV1::replace(
                path("src/lib.rs"),
                GitFileModeV1::Regular,
                git(1),
                git(1),
                raw(1),
                None,
            ),
            Err(SubjectError::NoOpReplacement)
        );
    }

    #[test]
    fn commit_parent_must_equal_expected_old_ref_state() {
        let mut raw_subject = subject_with(vec![mutation("src/lib.rs", 20)]);
        raw_subject.target.expected_old_ref_state = git(99);
        assert_eq!(
            qualify_repository_mutation_subject_v1(raw_subject),
            Err(SubjectError::CommitParentMismatch)
        );
    }

    #[test]
    fn commit_tree_must_equal_candidate_tree() {
        let mut raw_subject = subject_with(vec![mutation("src/lib.rs", 20)]);
        raw_subject.commit_profile.candidate_tree = git(99);
        assert_eq!(
            qualify_repository_mutation_subject_v1(raw_subject),
            Err(SubjectError::CommitTreeMismatch)
        );
    }

    #[test]
    fn changed_target_ref_changes_canonical_subject() {
        let first = qualify_repository_mutation_subject_v1(subject_with(vec![mutation(
            "src/lib.rs",
            20,
        )]))
        .expect("first");

        let mut second_raw = subject_with(vec![mutation("src/lib.rs", 20)]);
        second_raw.target.target_ref = GitHeadRefV1::new("refs/heads/fix/other").expect("ref");
        let second = qualify_repository_mutation_subject_v1(second_raw).expect("second");

        assert_ne!(first.canonical_preimage(), second.canonical_preimage());
    }

    #[test]
    fn changed_file_identity_changes_canonical_subject() {
        let first = qualify_repository_mutation_subject_v1(subject_with(vec![mutation(
            "src/lib.rs",
            20,
        )]))
        .expect("first");
        let second = qualify_repository_mutation_subject_v1(subject_with(vec![mutation(
            "src/lib.rs",
            21,
        )]))
        .expect("second");

        assert_ne!(first.canonical_preimage(), second.canonical_preimage());
    }

    #[test]
    fn changed_commit_metadata_changes_canonical_subject() {
        let first_raw = subject_with(vec![mutation("src/lib.rs", 20)]);
        let mut second_raw = first_raw.clone();
        second_raw.commit_profile.committer.unix_seconds += 1;

        let first = qualify_repository_mutation_subject_v1(first_raw).expect("first");
        let second = qualify_repository_mutation_subject_v1(second_raw).expect("second");

        assert_ne!(first.canonical_preimage(), second.canonical_preimage());
    }

    #[test]
    fn positive_subject_grants_no_later_authority() {
        let qualified = qualify_repository_mutation_subject_v1(subject_with(vec![mutation(
            "src/lib.rs",
            20,
        )]))
        .expect("qualified");

        assert!(!qualified.candidate_commitment_qualified_here());
        assert!(!qualified.git_tree_reconstructed_here());
        assert!(!qualified.git_commit_reconstructed_here());
        assert!(!qualified.grants_repository_write_authority());
        assert!(!qualified.publication_performed_here());
        assert!(!qualified.semantic_product_qualification_performed_here());
    }
}
