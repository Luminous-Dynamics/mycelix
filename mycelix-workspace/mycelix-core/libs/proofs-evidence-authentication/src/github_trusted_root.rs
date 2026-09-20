use std::collections::BTreeSet;

use serde::Deserialize;
use sha2::{Digest, Sha256};

use crate::{
    GitHubParsedVerificationCandidateV1, GitHubPublicCommandPlanV1,
    GitHubPublicVerifierExecutionPolicyV1, GitHubVerifiedTimestampWitnessKindV1,
    Sha256DigestV1, VerifierExecutionReceiptV1, VerifierTrustRootModeV1,
};
use crate::github_cli_plan::{GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1, GITHUB_PUBLIC_HOSTNAME_V1};

pub const GITHUB_DEFAULT_DUAL_TRUSTED_ROOT_PROFILE_V1: &str =
    "github-default-dual-trusted-root-jsonl-v1";
pub const RETAINED_TRUSTED_ROOT_DIGEST_PROFILE_V1: &str = "raw-sha256-v1";
pub const SIGSTORE_TRUSTED_ROOT_MEDIA_TYPE_V1: &str =
    "application/vnd.dev.sigstore.trustedroot+json;version=0.1";
pub const SIGSTORE_PUBLIC_GOOD_FULCIO_URI_V1: &str = "https://fulcio.sigstore.dev";
pub const MAX_GITHUB_TRUSTED_ROOT_JSONL_BYTES_V1: usize = 1024 * 1024;
pub const MAX_GITHUB_TRUSTED_ROOT_TLOGS_V1: usize = 32;
pub const MAX_GITHUB_TRUSTED_ROOT_CAS_V1: usize = 32;
pub const MAX_GITHUB_TRUSTED_ROOT_URI_BYTES_V1: usize = 4096;
pub const GITHUB_DEFAULT_DUAL_TRUSTED_ROOT_RECORDS_V1: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubTrustedRootClassificationAuthorityV1 {
    TrustRootClassificationOnly,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubTransparencyResolutionAuthorityV1 {
    TransparencyResolutionOnly,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubResolvedTrustInstanceStatusV1 {
    PublicGoodTransparencyConfirmed,
    GitHubPrivateTransparencyOnly,
    TimestampAuthorityOnly,
    NoVerifiedTimestamps,
    UnresolvedTrustInstance,
    MixedTrustDomains,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubTrustedRootClassificationErrorV1 {
    TrustedRootTooLarge { maximum: usize, actual: usize },
    InvalidExecutionReceipt,
    ProcessDidNotSucceed,
    WrongTrustRootMode,
    InvalidExecutionPolicy,
    VerifierIdentityMismatch,
    WrongCommandProfile,
    ExecutablePathMismatch,
    CommandArgumentsDigestMismatch,
    EnvironmentProfileDigestMismatch,
    CanonicalReceiptDigestMismatch,
    TrustedRootDigestMismatch,
    WrongTrustedRootCommand,
    WrongRootRecordCount { expected: usize, actual: usize },
    EmptyRootRecord { index: usize },
    NonCompactRootRecord { index: usize },
    InvalidRootJson { index: usize },
    WrongRootMediaType { index: usize },
    TooManyTransparencyLogs { index: usize, maximum: usize, actual: usize },
    TooManyCertificateAuthorities { index: usize, maximum: usize, actual: usize },
    EmptyServiceUri { index: usize, field: &'static str },
    ServiceUriTooLong {
        index: usize,
        field: &'static str,
        maximum: usize,
        actual: usize,
    },
    DuplicateTransparencyLogUri { index: usize, uri: String },
    DuplicateCertificateAuthorityUri { index: usize, uri: String },
    DuplicateSemanticRoots,
    MissingPublicGoodFulcio,
    EmptyPublicGoodTransparencyLogs,
    AmbiguousTransparencyLogAcrossTrustDomains { uri: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubTrustedRootClassificationV1 {
    profile_id: &'static str,
    retained_root_digest_profile_id: &'static str,
    retained_root_digest: Sha256DigestV1,
    public_good_root_digest: Sha256DigestV1,
    github_private_root_digest: Sha256DigestV1,
    public_good_tlog_uris: Vec<String>,
    public_good_ca_uris: Vec<String>,
    github_private_tlog_uris: Vec<String>,
    github_private_ca_uris: Vec<String>,
    verifier_executable_sha256: Sha256DigestV1,
    execution_policy_profile_id: String,
    command_profile_id: &'static str,
    execution_receipt: VerifierExecutionReceiptV1,
    execution_policy: GitHubPublicVerifierExecutionPolicyV1,
    command_plan: GitHubPublicCommandPlanV1,
    authority: GitHubTrustedRootClassificationAuthorityV1,
}

impl GitHubTrustedRootClassificationV1 {
    pub const fn profile_id(&self) -> &'static str {
        self.profile_id
    }

    pub const fn retained_root_digest_profile_id(&self) -> &'static str {
        self.retained_root_digest_profile_id
    }

    pub const fn retained_root_digest(&self) -> Sha256DigestV1 {
        self.retained_root_digest
    }

    pub const fn public_good_root_digest(&self) -> Sha256DigestV1 {
        self.public_good_root_digest
    }

    pub const fn github_private_root_digest(&self) -> Sha256DigestV1 {
        self.github_private_root_digest
    }

    pub fn public_good_tlog_uris(&self) -> &[String] {
        &self.public_good_tlog_uris
    }

    pub fn public_good_ca_uris(&self) -> &[String] {
        &self.public_good_ca_uris
    }

    pub fn github_private_tlog_uris(&self) -> &[String] {
        &self.github_private_tlog_uris
    }

    pub fn github_private_ca_uris(&self) -> &[String] {
        &self.github_private_ca_uris
    }

    pub const fn verifier_executable_sha256(&self) -> Sha256DigestV1 {
        self.verifier_executable_sha256
    }

    pub fn execution_policy_profile_id(&self) -> &str {
        &self.execution_policy_profile_id
    }

    pub const fn command_profile_id(&self) -> &'static str {
        self.command_profile_id
    }

    pub fn execution_receipt(&self) -> &VerifierExecutionReceiptV1 {
        &self.execution_receipt
    }

    pub fn execution_policy(&self) -> &GitHubPublicVerifierExecutionPolicyV1 {
        &self.execution_policy
    }

    pub fn command_plan(&self) -> &GitHubPublicCommandPlanV1 {
        &self.command_plan
    }

    pub const fn authority_scope(&self) -> GitHubTrustedRootClassificationAuthorityV1 {
        self.authority
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubTransparencyResolutionV1 {
    status: GitHubResolvedTrustInstanceStatusV1,
    public_good_tlog_uris: Vec<String>,
    github_private_tlog_uris: Vec<String>,
    unresolved_tlog_uris: Vec<String>,
    retained_root_digest: Sha256DigestV1,
    authority: GitHubTransparencyResolutionAuthorityV1,
}

impl GitHubTransparencyResolutionV1 {
    pub const fn status(&self) -> GitHubResolvedTrustInstanceStatusV1 {
        self.status
    }

    pub fn matched_public_good_tlog_uris(&self) -> &[String] {
        &self.public_good_tlog_uris
    }

    pub fn matched_github_private_tlog_uris(&self) -> &[String] {
        &self.github_private_tlog_uris
    }

    pub fn unresolved_tlog_uris(&self) -> &[String] {
        &self.unresolved_tlog_uris
    }

    pub const fn retained_root_digest(&self) -> Sha256DigestV1 {
        self.retained_root_digest
    }

    pub const fn authority_scope(&self) -> GitHubTransparencyResolutionAuthorityV1 {
        self.authority
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

#[derive(Debug, Deserialize)]
struct RawTrustedRootV1 {
    #[serde(rename = "mediaType")]
    media_type: String,
    tlogs: Vec<RawTransparencyLogV1>,
    #[serde(rename = "certificateAuthorities")]
    certificate_authorities: Vec<RawCertificateAuthorityV1>,
}

#[derive(Debug, Deserialize)]
struct RawTransparencyLogV1 {
    #[serde(rename = "baseUrl")]
    base_url: String,
}

#[derive(Debug, Deserialize)]
struct RawCertificateAuthorityV1 {
    uri: String,
}

struct NormalizedRootV1 {
    digest: Sha256DigestV1,
    tlog_uris: Vec<String>,
    ca_uris: Vec<String>,
}

pub fn classify_github_default_dual_trusted_root_v1(
    retained_root_bytes: &[u8],
    execution_receipt: &VerifierExecutionReceiptV1,
    execution_policy: &GitHubPublicVerifierExecutionPolicyV1,
    command_plan: &GitHubPublicCommandPlanV1,
) -> Result<GitHubTrustedRootClassificationV1, GitHubTrustedRootClassificationErrorV1> {
    if retained_root_bytes.len() > MAX_GITHUB_TRUSTED_ROOT_JSONL_BYTES_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::TrustedRootTooLarge {
            maximum: MAX_GITHUB_TRUSTED_ROOT_JSONL_BYTES_V1,
            actual: retained_root_bytes.len(),
        });
    }

    execution_receipt
        .validate()
        .map_err(|_| GitHubTrustedRootClassificationErrorV1::InvalidExecutionReceipt)?;
    if !execution_receipt.process_exited_successfully() {
        return Err(GitHubTrustedRootClassificationErrorV1::ProcessDidNotSucceed);
    }
    if execution_receipt.trust_root_mode != VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 {
        return Err(GitHubTrustedRootClassificationErrorV1::WrongTrustRootMode);
    }
    execution_policy
        .validate()
        .map_err(|_| GitHubTrustedRootClassificationErrorV1::InvalidExecutionPolicy)?;

    if &execution_receipt.verifier != execution_policy.expected_verifier() {
        return Err(GitHubTrustedRootClassificationErrorV1::VerifierIdentityMismatch);
    }
    if execution_receipt.verifier.command_profile_id != GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::WrongCommandProfile);
    }
    if command_plan.executable_path() != execution_policy.executable_path() {
        return Err(GitHubTrustedRootClassificationErrorV1::ExecutablePathMismatch);
    }
    if command_plan.command_arguments_digest() != execution_receipt.command_arguments_digest {
        return Err(GitHubTrustedRootClassificationErrorV1::CommandArgumentsDigestMismatch);
    }
    if command_plan.environment_profile_digest() != execution_receipt.environment_profile_digest {
        return Err(GitHubTrustedRootClassificationErrorV1::EnvironmentProfileDigestMismatch);
    }
    if command_plan.canonical_receipt_digest() != execution_receipt.canonical_receipt_digest {
        return Err(GitHubTrustedRootClassificationErrorV1::CanonicalReceiptDigestMismatch);
    }

    let retained_root_digest = sha256(retained_root_bytes);
    if retained_root_digest != execution_receipt.trusted_root_material_digest {
        return Err(GitHubTrustedRootClassificationErrorV1::TrustedRootDigestMismatch);
    }

    if !args_equal(
        command_plan.trusted_root_args(),
        &["attestation", "trusted-root", "--hostname", GITHUB_PUBLIC_HOSTNAME_V1],
    ) {
        return Err(GitHubTrustedRootClassificationErrorV1::WrongTrustedRootCommand);
    }

    let records = split_jsonl_records(retained_root_bytes)?;
    if records.len() != GITHUB_DEFAULT_DUAL_TRUSTED_ROOT_RECORDS_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::WrongRootRecordCount {
            expected: GITHUB_DEFAULT_DUAL_TRUSTED_ROOT_RECORDS_V1,
            actual: records.len(),
        });
    }

    let public_good = parse_root_record(0, records[0])?;
    let github_private = parse_root_record(1, records[1])?;

    if roots_semantically_equal(&public_good, &github_private) {
        return Err(GitHubTrustedRootClassificationErrorV1::DuplicateSemanticRoots);
    }
    if !public_good
        .ca_uris
        .iter()
        .any(|uri| uri == SIGSTORE_PUBLIC_GOOD_FULCIO_URI_V1)
    {
        return Err(GitHubTrustedRootClassificationErrorV1::MissingPublicGoodFulcio);
    }
    if public_good.tlog_uris.is_empty() {
        return Err(GitHubTrustedRootClassificationErrorV1::EmptyPublicGoodTransparencyLogs);
    }

    let private_tlogs: BTreeSet<&str> = github_private.tlog_uris.iter().map(String::as_str).collect();
    for uri in &public_good.tlog_uris {
        if private_tlogs.contains(uri.as_str()) {
            return Err(
                GitHubTrustedRootClassificationErrorV1::AmbiguousTransparencyLogAcrossTrustDomains {
                    uri: uri.clone(),
                },
            );
        }
    }

    Ok(GitHubTrustedRootClassificationV1 {
        profile_id: GITHUB_DEFAULT_DUAL_TRUSTED_ROOT_PROFILE_V1,
        retained_root_digest_profile_id: RETAINED_TRUSTED_ROOT_DIGEST_PROFILE_V1,
        retained_root_digest,
        public_good_root_digest: public_good.digest,
        github_private_root_digest: github_private.digest,
        public_good_tlog_uris: public_good.tlog_uris,
        public_good_ca_uris: public_good.ca_uris,
        github_private_tlog_uris: github_private.tlog_uris,
        github_private_ca_uris: github_private.ca_uris,
        verifier_executable_sha256: execution_receipt.verifier.executable_sha256,
        execution_policy_profile_id: execution_policy.profile_id().into(),
        command_profile_id: GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1,
        execution_receipt: execution_receipt.clone(),
        execution_policy: execution_policy.clone(),
        command_plan: command_plan.clone(),
        authority: GitHubTrustedRootClassificationAuthorityV1::TrustRootClassificationOnly,
    })
}

pub fn resolve_github_candidate_transparency_v1(
    classification: &GitHubTrustedRootClassificationV1,
    candidate: &GitHubParsedVerificationCandidateV1,
) -> GitHubTransparencyResolutionV1 {
    let public_good: BTreeSet<&str> = classification
        .public_good_tlog_uris
        .iter()
        .map(String::as_str)
        .collect();
    let github_private: BTreeSet<&str> = classification
        .github_private_tlog_uris
        .iter()
        .map(String::as_str)
        .collect();

    let mut public_matches = BTreeSet::new();
    let mut private_matches = BTreeSet::new();
    let mut unresolved = BTreeSet::new();
    let mut saw_tsa = false;
    let mut saw_other = false;

    for witness in candidate.verified_timestamps() {
        match witness.kind() {
            GitHubVerifiedTimestampWitnessKindV1::TransparencyLog => {
                if public_good.contains(witness.uri()) {
                    public_matches.insert(witness.uri().to_owned());
                } else if github_private.contains(witness.uri()) {
                    private_matches.insert(witness.uri().to_owned());
                } else {
                    unresolved.insert(witness.uri().to_owned());
                }
            }
            GitHubVerifiedTimestampWitnessKindV1::TimestampAuthority => saw_tsa = true,
            GitHubVerifiedTimestampWitnessKindV1::OtherVerifiedWitness => saw_other = true,
        }
    }

    let status = if candidate.verified_timestamps().is_empty() {
        GitHubResolvedTrustInstanceStatusV1::NoVerifiedTimestamps
    } else if !public_matches.is_empty() && !private_matches.is_empty() {
        GitHubResolvedTrustInstanceStatusV1::MixedTrustDomains
    } else if !unresolved.is_empty() || saw_other {
        GitHubResolvedTrustInstanceStatusV1::UnresolvedTrustInstance
    } else if !public_matches.is_empty() {
        GitHubResolvedTrustInstanceStatusV1::PublicGoodTransparencyConfirmed
    } else if !private_matches.is_empty() {
        GitHubResolvedTrustInstanceStatusV1::GitHubPrivateTransparencyOnly
    } else if saw_tsa {
        GitHubResolvedTrustInstanceStatusV1::TimestampAuthorityOnly
    } else {
        GitHubResolvedTrustInstanceStatusV1::UnresolvedTrustInstance
    };

    GitHubTransparencyResolutionV1 {
        status,
        public_good_tlog_uris: public_matches.into_iter().collect(),
        github_private_tlog_uris: private_matches.into_iter().collect(),
        unresolved_tlog_uris: unresolved.into_iter().collect(),
        retained_root_digest: classification.retained_root_digest,
        authority: GitHubTransparencyResolutionAuthorityV1::TransparencyResolutionOnly,
    }
}

fn split_jsonl_records(
    bytes: &[u8],
) -> Result<Vec<&[u8]>, GitHubTrustedRootClassificationErrorV1> {
    let mut records: Vec<&[u8]> = bytes.split(|byte| *byte == b'\n').collect();
    if matches!(records.last(), Some(last) if last.is_empty()) {
        records.pop();
    }
    for (index, record) in records.iter().enumerate() {
        if record.is_empty() {
            return Err(GitHubTrustedRootClassificationErrorV1::EmptyRootRecord { index });
        }
        if record.first().is_some_and(u8::is_ascii_whitespace)
            || record.last().is_some_and(u8::is_ascii_whitespace)
        {
            return Err(GitHubTrustedRootClassificationErrorV1::NonCompactRootRecord { index });
        }
    }
    Ok(records)
}

fn parse_root_record(
    index: usize,
    bytes: &[u8],
) -> Result<NormalizedRootV1, GitHubTrustedRootClassificationErrorV1> {
    let raw: RawTrustedRootV1 = serde_json::from_slice(bytes)
        .map_err(|_| GitHubTrustedRootClassificationErrorV1::InvalidRootJson { index })?;
    if raw.media_type != SIGSTORE_TRUSTED_ROOT_MEDIA_TYPE_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::WrongRootMediaType { index });
    }
    if raw.tlogs.len() > MAX_GITHUB_TRUSTED_ROOT_TLOGS_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::TooManyTransparencyLogs {
            index,
            maximum: MAX_GITHUB_TRUSTED_ROOT_TLOGS_V1,
            actual: raw.tlogs.len(),
        });
    }
    if raw.certificate_authorities.len() > MAX_GITHUB_TRUSTED_ROOT_CAS_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::TooManyCertificateAuthorities {
            index,
            maximum: MAX_GITHUB_TRUSTED_ROOT_CAS_V1,
            actual: raw.certificate_authorities.len(),
        });
    }

    let mut tlog_uris = Vec::with_capacity(raw.tlogs.len());
    let mut seen_tlogs = BTreeSet::new();
    for tlog in raw.tlogs {
        check_uri(index, "tlogs[].baseUrl", &tlog.base_url)?;
        if !seen_tlogs.insert(tlog.base_url.clone()) {
            return Err(GitHubTrustedRootClassificationErrorV1::DuplicateTransparencyLogUri {
                index,
                uri: tlog.base_url,
            });
        }
        tlog_uris.push(tlog.base_url);
    }

    let mut ca_uris = Vec::with_capacity(raw.certificate_authorities.len());
    let mut seen_cas = BTreeSet::new();
    for ca in raw.certificate_authorities {
        check_uri(index, "certificateAuthorities[].uri", &ca.uri)?;
        if !seen_cas.insert(ca.uri.clone()) {
            return Err(GitHubTrustedRootClassificationErrorV1::DuplicateCertificateAuthorityUri {
                index,
                uri: ca.uri,
            });
        }
        ca_uris.push(ca.uri);
    }

    Ok(NormalizedRootV1 {
        digest: sha256(bytes),
        tlog_uris,
        ca_uris,
    })
}

fn roots_semantically_equal(a: &NormalizedRootV1, b: &NormalizedRootV1) -> bool {
    let a_tlogs: BTreeSet<&str> = a.tlog_uris.iter().map(String::as_str).collect();
    let b_tlogs: BTreeSet<&str> = b.tlog_uris.iter().map(String::as_str).collect();
    let a_cas: BTreeSet<&str> = a.ca_uris.iter().map(String::as_str).collect();
    let b_cas: BTreeSet<&str> = b.ca_uris.iter().map(String::as_str).collect();
    a_tlogs == b_tlogs && a_cas == b_cas
}

fn check_uri(
    index: usize,
    field: &'static str,
    value: &str,
) -> Result<(), GitHubTrustedRootClassificationErrorV1> {
    if value.trim().is_empty() {
        return Err(GitHubTrustedRootClassificationErrorV1::EmptyServiceUri { index, field });
    }
    if value.len() > MAX_GITHUB_TRUSTED_ROOT_URI_BYTES_V1 {
        return Err(GitHubTrustedRootClassificationErrorV1::ServiceUriTooLong {
            index,
            field,
            maximum: MAX_GITHUB_TRUSTED_ROOT_URI_BYTES_V1,
            actual: value.len(),
        });
    }
    Ok(())
}

fn args_equal(actual: &[String], expected: &[&str]) -> bool {
    actual.len() == expected.len()
        && actual
            .iter()
            .zip(expected.iter())
            .all(|(actual, expected)| actual == expected)
}

fn sha256(bytes: &[u8]) -> Sha256DigestV1 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    Sha256DigestV1::from_bytes(hasher.finalize().into())
}
