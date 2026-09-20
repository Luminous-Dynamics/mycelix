use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::{QualificationReceiptDigestV1, Sha256DigestV1, VerifierExecutionReceiptV1};

pub const GITHUB_VERIFICATION_RESULT_PARSER_PROFILE_V1: &str =
    "github-cli-attestation-processing-result-json-v1";
pub const SIGSTORE_VERIFICATION_RESULT_MEDIA_TYPE_V1: &str =
    "application/vnd.dev.sigstore.verificationresult+json;version=0.1";
pub const VERIFIER_STDOUT_DIGEST_PROFILE_V1: &str = "raw-sha256-v1";
pub const MAX_GITHUB_VERIFICATION_STDOUT_BYTES_V1: usize = 4 * 1024 * 1024;
pub const MAX_GITHUB_VERIFICATION_RESULTS_V1: usize = 8;
pub const MAX_GITHUB_STATEMENT_SUBJECTS_V1: usize = 16;
pub const MAX_GITHUB_VERIFIED_TIMESTAMPS_V1: usize = 16;
pub const MAX_GITHUB_RESULT_TEXT_BYTES_V1: usize = 4 * 1024;
pub const MAX_GITHUB_PREDICATE_JSON_BYTES_V1: usize = 256 * 1024;
pub const MAX_GITHUB_CERTIFICATE_JSON_BYTES_V1: usize = 256 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubAttestationFactProvenanceV1 {
    CertificateDerived,
    VerifiedTimestampDerived,
    WorkflowControlledSignedClaim,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubVerifiedTimestampWitnessKindV1 {
    TransparencyLog,
    TimestampAuthority,
    OtherVerifiedWitness,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubTrustInstanceStatusV1 {
    NoVerifiedTimestamps,
    TimestampAuthorityOnly,
    UnresolvedTrustInstance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitHubParsedVerifierOutputAuthorityV1 {
    ParsedVerifierOutputOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubVerificationResultParseErrorV1 {
    StdoutTooLarge { maximum: usize, actual: usize },
    InvalidExecutionReceipt,
    ProcessDidNotSucceed,
    StdoutDigestMismatch,
    InvalidJson,
    EmptyResults,
    TooManyResults { maximum: usize, actual: usize },
    ExecutionResultCountMismatch { expected: u32, actual: usize },
    AttestationMustBeObject { index: usize },
    WrongVerificationMediaType { index: usize },
    MissingCertificate { index: usize },
    CertificateTooLarge { index: usize, maximum: usize, actual: usize },
    TooManyVerifiedTimestamps { index: usize, maximum: usize, actual: usize },
    TooManyStatementSubjects { index: usize, maximum: usize, actual: usize },
    PredicateTooLarge { index: usize, maximum: usize, actual: usize },
    EmptyRequiredText { index: usize, field: &'static str },
    TextFieldTooLong {
        index: usize,
        field: &'static str,
        maximum: usize,
        actual: usize,
    },
    MissingSha256SubjectDigest { index: usize, subject_index: usize },
    MalformedSha256SubjectDigest { index: usize, subject_index: usize },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubCertificateIdentityFactsV1 {
    certificate_issuer: String,
    subject_alternative_name: String,
    oidc_issuer: Option<String>,
    build_signer_uri: Option<String>,
    build_signer_digest: Option<String>,
    runner_environment: Option<String>,
    source_repository_uri: Option<String>,
    source_repository_digest: Option<String>,
    source_repository_ref: Option<String>,
    source_repository_identifier: Option<String>,
    source_repository_owner_uri: Option<String>,
    source_repository_owner_identifier: Option<String>,
    build_config_uri: Option<String>,
    build_config_digest: Option<String>,
    build_trigger: Option<String>,
    run_invocation_uri: Option<String>,
    source_repository_visibility_at_signing: Option<String>,
}

impl GitHubCertificateIdentityFactsV1 {
    pub const fn provenance(&self) -> GitHubAttestationFactProvenanceV1 {
        GitHubAttestationFactProvenanceV1::CertificateDerived
    }

    pub fn certificate_issuer(&self) -> &str { &self.certificate_issuer }
    pub fn subject_alternative_name(&self) -> &str { &self.subject_alternative_name }
    pub fn oidc_issuer(&self) -> Option<&str> { self.oidc_issuer.as_deref() }
    pub fn build_signer_uri(&self) -> Option<&str> { self.build_signer_uri.as_deref() }
    pub fn build_signer_digest(&self) -> Option<&str> { self.build_signer_digest.as_deref() }
    pub fn runner_environment(&self) -> Option<&str> { self.runner_environment.as_deref() }
    pub fn source_repository_uri(&self) -> Option<&str> { self.source_repository_uri.as_deref() }
    pub fn source_repository_digest(&self) -> Option<&str> { self.source_repository_digest.as_deref() }
    pub fn source_repository_ref(&self) -> Option<&str> { self.source_repository_ref.as_deref() }
    pub fn source_repository_identifier(&self) -> Option<&str> { self.source_repository_identifier.as_deref() }
    pub fn source_repository_owner_uri(&self) -> Option<&str> { self.source_repository_owner_uri.as_deref() }
    pub fn source_repository_owner_identifier(&self) -> Option<&str> { self.source_repository_owner_identifier.as_deref() }
    pub fn build_config_uri(&self) -> Option<&str> { self.build_config_uri.as_deref() }
    pub fn build_config_digest(&self) -> Option<&str> { self.build_config_digest.as_deref() }
    pub fn build_trigger(&self) -> Option<&str> { self.build_trigger.as_deref() }
    pub fn run_invocation_uri(&self) -> Option<&str> { self.run_invocation_uri.as_deref() }
    pub fn source_repository_visibility_at_signing(&self) -> Option<&str> {
        self.source_repository_visibility_at_signing.as_deref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubVerifiedTimestampEvidenceV1 {
    kind: GitHubVerifiedTimestampWitnessKindV1,
    raw_type: String,
    uri: String,
    timestamp: String,
}

impl GitHubVerifiedTimestampEvidenceV1 {
    pub const fn provenance(&self) -> GitHubAttestationFactProvenanceV1 {
        GitHubAttestationFactProvenanceV1::VerifiedTimestampDerived
    }
    pub const fn kind(&self) -> GitHubVerifiedTimestampWitnessKindV1 { self.kind }
    pub fn raw_type(&self) -> &str { &self.raw_type }
    pub fn uri(&self) -> &str { &self.uri }
    pub fn timestamp(&self) -> &str { &self.timestamp }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubSignedStatementSubjectV1 {
    name: String,
    sha256: Sha256DigestV1,
}

impl GitHubSignedStatementSubjectV1 {
    pub const fn provenance(&self) -> GitHubAttestationFactProvenanceV1 {
        GitHubAttestationFactProvenanceV1::WorkflowControlledSignedClaim
    }
    pub fn name(&self) -> &str { &self.name }
    pub const fn sha256(&self) -> Sha256DigestV1 { self.sha256 }
}

#[derive(Clone, Debug, PartialEq)]
pub struct GitHubSignedPredicateClaimV1 {
    predicate_type: String,
    predicate: Value,
}

impl GitHubSignedPredicateClaimV1 {
    pub const fn provenance(&self) -> GitHubAttestationFactProvenanceV1 {
        GitHubAttestationFactProvenanceV1::WorkflowControlledSignedClaim
    }
    pub fn predicate_type(&self) -> &str { &self.predicate_type }
    pub fn predicate(&self) -> &Value { &self.predicate }
    pub fn predicate_json(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(&self.predicate)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct GitHubParsedVerificationCandidateV1 {
    certificate: GitHubCertificateIdentityFactsV1,
    verified_timestamps: Vec<GitHubVerifiedTimestampEvidenceV1>,
    trust_instance_status: GitHubTrustInstanceStatusV1,
    statement_type: String,
    subjects: Vec<GitHubSignedStatementSubjectV1>,
    predicate: GitHubSignedPredicateClaimV1,
}

impl GitHubParsedVerificationCandidateV1 {
    pub fn certificate(&self) -> &GitHubCertificateIdentityFactsV1 { &self.certificate }
    pub fn verified_timestamps(&self) -> &[GitHubVerifiedTimestampEvidenceV1] { &self.verified_timestamps }
    pub const fn trust_instance_status(&self) -> GitHubTrustInstanceStatusV1 { self.trust_instance_status }
    pub fn statement_type(&self) -> &str { &self.statement_type }
    pub fn subjects(&self) -> &[GitHubSignedStatementSubjectV1] { &self.subjects }
    pub fn predicate(&self) -> &GitHubSignedPredicateClaimV1 { &self.predicate }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParsedGitHubAttestationVerifierOutputV1 {
    parser_profile_id: &'static str,
    stdout_digest_profile_id: &'static str,
    raw_stdout_digest: Sha256DigestV1,
    canonical_receipt_digest: QualificationReceiptDigestV1,
    execution_receipt: VerifierExecutionReceiptV1,
    candidates: Vec<GitHubParsedVerificationCandidateV1>,
    authority: GitHubParsedVerifierOutputAuthorityV1,
}

impl ParsedGitHubAttestationVerifierOutputV1 {
    pub const fn parser_profile_id(&self) -> &'static str { self.parser_profile_id }
    pub const fn stdout_digest_profile_id(&self) -> &'static str { self.stdout_digest_profile_id }
    pub const fn raw_stdout_digest(&self) -> Sha256DigestV1 { self.raw_stdout_digest }
    pub const fn canonical_receipt_digest(&self) -> QualificationReceiptDigestV1 { self.canonical_receipt_digest }
    pub fn execution_receipt(&self) -> &VerifierExecutionReceiptV1 { &self.execution_receipt }
    pub fn candidates(&self) -> &[GitHubParsedVerificationCandidateV1] { &self.candidates }
    pub const fn authority_scope(&self) -> GitHubParsedVerifierOutputAuthorityV1 { self.authority }
    pub const fn establishes_trusted_builder_identity(&self) -> bool { false }
    pub const fn establishes_public_good_transparency(&self) -> bool { false }
    pub const fn establishes_receipt_authentication(&self) -> bool { false }
    pub const fn grants_production_authority(&self) -> bool { false }
    pub const fn grants_application_authority(&self) -> bool { false }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OuterProcessingResultV1 {
    attestation: Value,
    #[serde(rename = "verificationResult")]
    verification_result: RawVerificationResultV1,
}

// Nested Sigstore result data is extension-tolerant by design. Required authority
// paths remain typed and mandatory for this parser profile.
#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawVerificationResultV1 {
    media_type: String,
    statement: RawStatementV1,
    signature: RawSignatureV1,
    verified_timestamps: Vec<RawTimestampV1>,
}

#[derive(Debug, Deserialize)]
struct RawSignatureV1 {
    certificate: Option<RawCertificateSummaryV1>,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct RawCertificateSummaryV1 {
    certificate_issuer: String,
    subject_alternative_name: String,
    #[serde(default)]
    issuer: Option<String>,
    #[serde(default, rename = "buildSignerURI")]
    build_signer_uri: Option<String>,
    #[serde(default)]
    build_signer_digest: Option<String>,
    #[serde(default)]
    runner_environment: Option<String>,
    #[serde(default, rename = "sourceRepositoryURI")]
    source_repository_uri: Option<String>,
    #[serde(default)]
    source_repository_digest: Option<String>,
    #[serde(default)]
    source_repository_ref: Option<String>,
    #[serde(default)]
    source_repository_identifier: Option<String>,
    #[serde(default, rename = "sourceRepositoryOwnerURI")]
    source_repository_owner_uri: Option<String>,
    #[serde(default)]
    source_repository_owner_identifier: Option<String>,
    #[serde(default, rename = "buildConfigURI")]
    build_config_uri: Option<String>,
    #[serde(default)]
    build_config_digest: Option<String>,
    #[serde(default)]
    build_trigger: Option<String>,
    #[serde(default, rename = "runInvocationURI")]
    run_invocation_uri: Option<String>,
    #[serde(default)]
    source_repository_visibility_at_signing: Option<String>,
}

#[derive(Debug, Deserialize)]
struct RawTimestampV1 {
    #[serde(rename = "type")]
    witness_type: String,
    uri: String,
    timestamp: String,
}

#[derive(Debug, Deserialize)]
struct RawStatementV1 {
    #[serde(rename = "_type")]
    statement_type: String,
    subject: Vec<RawSubjectV1>,
    #[serde(rename = "predicateType")]
    predicate_type: String,
    predicate: Value,
}

#[derive(Debug, Deserialize)]
struct RawSubjectV1 {
    name: String,
    digest: BTreeMap<String, String>,
}

pub fn parse_github_attestation_verifier_output_v1(
    raw_stdout: &[u8],
    execution_receipt: &VerifierExecutionReceiptV1,
) -> Result<ParsedGitHubAttestationVerifierOutputV1, GitHubVerificationResultParseErrorV1> {
    if raw_stdout.len() > MAX_GITHUB_VERIFICATION_STDOUT_BYTES_V1 {
        return Err(GitHubVerificationResultParseErrorV1::StdoutTooLarge {
            maximum: MAX_GITHUB_VERIFICATION_STDOUT_BYTES_V1,
            actual: raw_stdout.len(),
        });
    }
    execution_receipt
        .validate()
        .map_err(|_| GitHubVerificationResultParseErrorV1::InvalidExecutionReceipt)?;
    if !execution_receipt.process_exited_successfully() {
        return Err(GitHubVerificationResultParseErrorV1::ProcessDidNotSucceed);
    }

    let raw_stdout_digest = github_verifier_stdout_digest_v1(raw_stdout);
    if raw_stdout_digest != execution_receipt.verifier_stdout_digest {
        return Err(GitHubVerificationResultParseErrorV1::StdoutDigestMismatch);
    }

    let raw: Vec<OuterProcessingResultV1> =
        serde_json::from_slice(raw_stdout).map_err(|_| GitHubVerificationResultParseErrorV1::InvalidJson)?;
    if raw.is_empty() {
        return Err(GitHubVerificationResultParseErrorV1::EmptyResults);
    }
    if raw.len() > MAX_GITHUB_VERIFICATION_RESULTS_V1 {
        return Err(GitHubVerificationResultParseErrorV1::TooManyResults {
            maximum: MAX_GITHUB_VERIFICATION_RESULTS_V1,
            actual: raw.len(),
        });
    }
    if raw.len() != execution_receipt.parsed_result_count as usize {
        return Err(GitHubVerificationResultParseErrorV1::ExecutionResultCountMismatch {
            expected: execution_receipt.parsed_result_count,
            actual: raw.len(),
        });
    }

    let mut candidates = Vec::with_capacity(raw.len());
    for (index, item) in raw.into_iter().enumerate() {
        if !item.attestation.is_object() {
            return Err(GitHubVerificationResultParseErrorV1::AttestationMustBeObject { index });
        }
        let vr = item.verification_result;
        if vr.media_type != SIGSTORE_VERIFICATION_RESULT_MEDIA_TYPE_V1 {
            return Err(GitHubVerificationResultParseErrorV1::WrongVerificationMediaType { index });
        }
        check_text(index, "verificationResult.mediaType", &vr.media_type)?;
        check_text(index, "statement._type", &vr.statement.statement_type)?;
        check_text(index, "statement.predicateType", &vr.statement.predicate_type)?;

        if vr.verified_timestamps.len() > MAX_GITHUB_VERIFIED_TIMESTAMPS_V1 {
            return Err(GitHubVerificationResultParseErrorV1::TooManyVerifiedTimestamps {
                index,
                maximum: MAX_GITHUB_VERIFIED_TIMESTAMPS_V1,
                actual: vr.verified_timestamps.len(),
            });
        }
        if vr.statement.subject.len() > MAX_GITHUB_STATEMENT_SUBJECTS_V1 {
            return Err(GitHubVerificationResultParseErrorV1::TooManyStatementSubjects {
                index,
                maximum: MAX_GITHUB_STATEMENT_SUBJECTS_V1,
                actual: vr.statement.subject.len(),
            });
        }

        let predicate_bytes = serde_json::to_vec(&vr.statement.predicate)
            .map_err(|_| GitHubVerificationResultParseErrorV1::InvalidJson)?;
        if predicate_bytes.len() > MAX_GITHUB_PREDICATE_JSON_BYTES_V1 {
            return Err(GitHubVerificationResultParseErrorV1::PredicateTooLarge {
                index,
                maximum: MAX_GITHUB_PREDICATE_JSON_BYTES_V1,
                actual: predicate_bytes.len(),
            });
        }

        let certificate = vr
            .signature
            .certificate
            .ok_or(GitHubVerificationResultParseErrorV1::MissingCertificate { index })?;
        let certificate_bytes = serde_json::to_vec(&certificate)
            .map_err(|_| GitHubVerificationResultParseErrorV1::InvalidJson)?;
        if certificate_bytes.len() > MAX_GITHUB_CERTIFICATE_JSON_BYTES_V1 {
            return Err(GitHubVerificationResultParseErrorV1::CertificateTooLarge {
                index,
                maximum: MAX_GITHUB_CERTIFICATE_JSON_BYTES_V1,
                actual: certificate_bytes.len(),
            });
        }
        validate_certificate_text(index, &certificate)?;

        let mut timestamps = Vec::with_capacity(vr.verified_timestamps.len());
        let mut has_tlog = false;
        let mut has_tsa = false;
        for timestamp in vr.verified_timestamps {
            check_text(index, "verifiedTimestamps.type", &timestamp.witness_type)?;
            check_text(index, "verifiedTimestamps.uri", &timestamp.uri)?;
            check_text(index, "verifiedTimestamps.timestamp", &timestamp.timestamp)?;
            let kind = match timestamp.witness_type.as_str() {
                "Tlog" => {
                    has_tlog = true;
                    GitHubVerifiedTimestampWitnessKindV1::TransparencyLog
                }
                "TimestampAuthority" => {
                    has_tsa = true;
                    GitHubVerifiedTimestampWitnessKindV1::TimestampAuthority
                }
                _ => GitHubVerifiedTimestampWitnessKindV1::OtherVerifiedWitness,
            };
            timestamps.push(GitHubVerifiedTimestampEvidenceV1 {
                kind,
                raw_type: timestamp.witness_type,
                uri: timestamp.uri,
                timestamp: timestamp.timestamp,
            });
        }
        let trust_instance_status = if has_tlog {
            // Tlog proves the verifier accepted a transparency-log witness; it does not by
            // itself identify which retained trust-root instance admitted that witness.
            GitHubTrustInstanceStatusV1::UnresolvedTrustInstance
        } else if has_tsa {
            GitHubTrustInstanceStatusV1::TimestampAuthorityOnly
        } else {
            GitHubTrustInstanceStatusV1::NoVerifiedTimestamps
        };

        let mut subjects = Vec::with_capacity(vr.statement.subject.len());
        for (subject_index, subject) in vr.statement.subject.into_iter().enumerate() {
            check_text(index, "statement.subject.name", &subject.name)?;
            let sha256 = subject.digest.get("sha256").ok_or(
                GitHubVerificationResultParseErrorV1::MissingSha256SubjectDigest {
                    index,
                    subject_index,
                },
            )?;
            let sha256 = Sha256DigestV1::from_hex(sha256).map_err(|_| {
                GitHubVerificationResultParseErrorV1::MalformedSha256SubjectDigest {
                    index,
                    subject_index,
                }
            })?;
            subjects.push(GitHubSignedStatementSubjectV1 {
                name: subject.name,
                sha256,
            });
        }

        candidates.push(GitHubParsedVerificationCandidateV1 {
            certificate: GitHubCertificateIdentityFactsV1 {
                certificate_issuer: certificate.certificate_issuer,
                subject_alternative_name: certificate.subject_alternative_name,
                oidc_issuer: certificate.issuer,
                build_signer_uri: certificate.build_signer_uri,
                build_signer_digest: certificate.build_signer_digest,
                runner_environment: certificate.runner_environment,
                source_repository_uri: certificate.source_repository_uri,
                source_repository_digest: certificate.source_repository_digest,
                source_repository_ref: certificate.source_repository_ref,
                source_repository_identifier: certificate.source_repository_identifier,
                source_repository_owner_uri: certificate.source_repository_owner_uri,
                source_repository_owner_identifier: certificate.source_repository_owner_identifier,
                build_config_uri: certificate.build_config_uri,
                build_config_digest: certificate.build_config_digest,
                build_trigger: certificate.build_trigger,
                run_invocation_uri: certificate.run_invocation_uri,
                source_repository_visibility_at_signing: certificate.source_repository_visibility_at_signing,
            },
            verified_timestamps: timestamps,
            trust_instance_status,
            statement_type: vr.statement.statement_type,
            subjects,
            predicate: GitHubSignedPredicateClaimV1 {
                predicate_type: vr.statement.predicate_type,
                predicate: vr.statement.predicate,
            },
        });
    }

    Ok(ParsedGitHubAttestationVerifierOutputV1 {
        parser_profile_id: GITHUB_VERIFICATION_RESULT_PARSER_PROFILE_V1,
        stdout_digest_profile_id: VERIFIER_STDOUT_DIGEST_PROFILE_V1,
        raw_stdout_digest,
        canonical_receipt_digest: execution_receipt.canonical_receipt_digest,
        execution_receipt: execution_receipt.clone(),
        candidates,
        authority: GitHubParsedVerifierOutputAuthorityV1::ParsedVerifierOutputOnly,
    })
}

/// Raw SHA-256 of the exact verifier stdout bytes.
pub fn github_verifier_stdout_digest_v1(raw_stdout: &[u8]) -> Sha256DigestV1 {
    let mut hasher = Sha256::new();
    hasher.update(raw_stdout);
    Sha256DigestV1::from_bytes(hasher.finalize().into())
}

fn check_text(
    index: usize,
    field: &'static str,
    value: &str,
) -> Result<(), GitHubVerificationResultParseErrorV1> {
    if value.trim().is_empty() {
        return Err(GitHubVerificationResultParseErrorV1::EmptyRequiredText { index, field });
    }
    let actual = value.len();
    if actual > MAX_GITHUB_RESULT_TEXT_BYTES_V1 {
        return Err(GitHubVerificationResultParseErrorV1::TextFieldTooLong {
            index,
            field,
            maximum: MAX_GITHUB_RESULT_TEXT_BYTES_V1,
            actual,
        });
    }
    Ok(())
}

fn check_optional_text(
    index: usize,
    field: &'static str,
    value: Option<&str>,
) -> Result<(), GitHubVerificationResultParseErrorV1> {
    if let Some(value) = value {
        check_text(index, field, value)?;
    }
    Ok(())
}

fn validate_certificate_text(
    index: usize,
    certificate: &RawCertificateSummaryV1,
) -> Result<(), GitHubVerificationResultParseErrorV1> {
    check_text(
        index,
        "signature.certificate.certificateIssuer",
        &certificate.certificate_issuer,
    )?;
    check_text(
        index,
        "signature.certificate.subjectAlternativeName",
        &certificate.subject_alternative_name,
    )?;
    for (field, value) in [
        ("signature.certificate.issuer", certificate.issuer.as_deref()),
        ("signature.certificate.buildSignerURI", certificate.build_signer_uri.as_deref()),
        ("signature.certificate.buildSignerDigest", certificate.build_signer_digest.as_deref()),
        ("signature.certificate.runnerEnvironment", certificate.runner_environment.as_deref()),
        ("signature.certificate.sourceRepositoryURI", certificate.source_repository_uri.as_deref()),
        ("signature.certificate.sourceRepositoryDigest", certificate.source_repository_digest.as_deref()),
        ("signature.certificate.sourceRepositoryRef", certificate.source_repository_ref.as_deref()),
        ("signature.certificate.sourceRepositoryIdentifier", certificate.source_repository_identifier.as_deref()),
        ("signature.certificate.sourceRepositoryOwnerURI", certificate.source_repository_owner_uri.as_deref()),
        ("signature.certificate.sourceRepositoryOwnerIdentifier", certificate.source_repository_owner_identifier.as_deref()),
        ("signature.certificate.buildConfigURI", certificate.build_config_uri.as_deref()),
        ("signature.certificate.buildConfigDigest", certificate.build_config_digest.as_deref()),
        ("signature.certificate.buildTrigger", certificate.build_trigger.as_deref()),
        ("signature.certificate.runInvocationURI", certificate.run_invocation_uri.as_deref()),
        (
            "signature.certificate.sourceRepositoryVisibilityAtSigning",
            certificate.source_repository_visibility_at_signing.as_deref(),
        ),
    ] {
        check_optional_text(index, field, value)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        NixVerifierClosureIdentityV1, QualificationReceiptCanonicalizationV1,
        VerifierExecutableIdentityV1, VerifierProcessOutcomeV1, VerifierTrustRootModeV1,
        VERIFIER_EXECUTION_EVIDENCE_VERSION_V1,
    };

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn candidate_json(timestamp_type: Option<&str>) -> Value {
        let timestamps = timestamp_type.map_or_else(Vec::new, |timestamp_type| {
            vec![serde_json::json!({
                "type": timestamp_type,
                "uri": if timestamp_type == "Tlog" {
                    "https://rekor.sigstore.dev"
                } else {
                    "https://timestamp.example"
                },
                "timestamp": "2026-09-20T00:00:00Z"
            })]
        });
        serde_json::json!({
            "attestation": {"bundle": "retained-elsewhere"},
            "verificationResult": {
                "mediaType": SIGSTORE_VERIFICATION_RESULT_MEDIA_TYPE_V1,
                "statement": {
                    "_type": "https://in-toto.io/Statement/v1",
                    "subject": [{
                        "name": "mycelix-qualification-receipt",
                        "digest": {"sha256": "11".repeat(32)}
                    }],
                    "predicateType": "https://mycelix.org/attestations/qualification/v1",
                    "predicate": {
                        "predicate_version": 1,
                        "predicate_schema": "mycelix-qualification-attestation-predicate-v1"
                    }
                },
                "signature": {
                    "certificate": {
                        "certificateIssuer": "CN=sigstore-intermediate,O=sigstore.dev",
                        "subjectAlternativeName": "https://github.com/Luminous-Dynamics/mycelix/.github/workflows/qualify.yml@refs/heads/main",
                        "issuer": "https://token.actions.githubusercontent.com",
                        "buildSignerURI": "https://github.com/Luminous-Dynamics/mycelix/.github/workflows/qualify.yml@refs/heads/main",
                        "buildSignerDigest": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
                        "runnerEnvironment": "github-hosted",
                        "sourceRepositoryURI": "https://github.com/Luminous-Dynamics/mycelix",
                        "sourceRepositoryDigest": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                        "sourceRepositoryRef": "refs/heads/main",
                        "sourceRepositoryIdentifier": "1176351975",
                        "sourceRepositoryOwnerURI": "https://github.com/Luminous-Dynamics",
                        "sourceRepositoryOwnerIdentifier": "example-owner-id",
                        "buildConfigURI": "https://github.com/Luminous-Dynamics/mycelix/.github/workflows/qualify.yml@refs/heads/main",
                        "buildConfigDigest": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
                        "buildTrigger": "pull_request",
                        "runInvocationURI": "https://github.com/Luminous-Dynamics/mycelix/actions/runs/1",
                        "sourceRepositoryVisibilityAtSigning": "public"
                    }
                },
                "verifiedTimestamps": timestamps,
                "futureNestedField": {"allowed": true}
            }
        })
    }

    fn execution_receipt(stdout: &[u8], count: u32) -> VerifierExecutionReceiptV1 {
        VerifierExecutionReceiptV1 {
            evidence_version: VERIFIER_EXECUTION_EVIDENCE_VERSION_V1,
            verifier: VerifierExecutableIdentityV1 {
                profile_id: "gh-exact-v1".into(),
                backend_family: "github-cli-attestation".into(),
                semantic_version: "2.101.0".into(),
                executable_sha256: digest(1),
                platform_profile: "linux-x86_64".into(),
                command_profile_id: "mycelix-github-public-qualification-verifier-command-v1".into(),
                nix_closure: Some(NixVerifierClosureIdentityV1 {
                    store_path: "/nix/store/example-gh/bin/gh".into(),
                    closure_digest: digest(2),
                }),
            },
            canonical_receipt_digest: QualificationReceiptDigestV1 {
                canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
                sha256: digest(3),
            },
            attestation_bundle_digest: digest(4),
            trust_root_mode: VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1,
            trusted_root_material_digest: digest(5),
            trusted_root_acquired_at_unix_seconds: 1_010,
            command_arguments_digest: digest(6),
            environment_profile_digest: digest(7),
            verifier_stdout_digest: github_verifier_stdout_digest_v1(stdout),
            verifier_stderr_digest: digest(9),
            execution_started_at_unix_seconds: 1_000,
            execution_completed_at_unix_seconds: 1_020,
            process_outcome: VerifierProcessOutcomeV1::ExitCode(0),
            parsed_result_count: count,
        }
    }

    #[test]
    fn tlog_output_parses_but_trust_instance_remains_unresolved() {
        let raw = serde_json::to_vec(&vec![candidate_json(Some("Tlog"))]).unwrap();
        let parsed = parse_github_attestation_verifier_output_v1(
            &raw,
            &execution_receipt(&raw, 1),
        )
        .unwrap();
        let candidate = &parsed.candidates()[0];
        assert_eq!(
            candidate.trust_instance_status(),
            GitHubTrustInstanceStatusV1::UnresolvedTrustInstance
        );
        assert_eq!(
            candidate.verified_timestamps()[0].kind(),
            GitHubVerifiedTimestampWitnessKindV1::TransparencyLog
        );
        assert_eq!(
            candidate.certificate().build_signer_uri(),
            Some("https://github.com/Luminous-Dynamics/mycelix/.github/workflows/qualify.yml@refs/heads/main")
        );
        assert_eq!(
            candidate.certificate().source_repository_uri(),
            Some("https://github.com/Luminous-Dynamics/mycelix")
        );
        assert!(!parsed.establishes_public_good_transparency());
        assert!(!parsed.establishes_receipt_authentication());
    }

    #[test]
    fn tsa_only_and_empty_timestamps_are_distinct() {
        let raw = serde_json::to_vec(&vec![candidate_json(Some("TimestampAuthority"))]).unwrap();
        let parsed = parse_github_attestation_verifier_output_v1(
            &raw,
            &execution_receipt(&raw, 1),
        )
        .unwrap();
        assert_eq!(
            parsed.candidates()[0].trust_instance_status(),
            GitHubTrustInstanceStatusV1::TimestampAuthorityOnly
        );

        let raw = serde_json::to_vec(&vec![candidate_json(None)]).unwrap();
        let parsed = parse_github_attestation_verifier_output_v1(
            &raw,
            &execution_receipt(&raw, 1),
        )
        .unwrap();
        assert_eq!(
            parsed.candidates()[0].trust_instance_status(),
            GitHubTrustInstanceStatusV1::NoVerifiedTimestamps
        );
    }

    #[test]
    fn detached_or_modified_stdout_fails_lineage_binding() {
        let raw = serde_json::to_vec(&vec![candidate_json(Some("Tlog"))]).unwrap();
        let receipt = execution_receipt(&raw, 1);
        let mut changed = raw.clone();
        changed.push(b' ');
        assert_eq!(
            parse_github_attestation_verifier_output_v1(&changed, &receipt),
            Err(GitHubVerificationResultParseErrorV1::StdoutDigestMismatch)
        );
    }

    #[test]
    fn signal_timeout_and_nonzero_exit_fail_before_parse() {
        let raw = serde_json::to_vec(&vec![candidate_json(Some("Tlog"))]).unwrap();
        for outcome in [
            VerifierProcessOutcomeV1::TerminatedBySignal(9),
            VerifierProcessOutcomeV1::TimedOut,
            VerifierProcessOutcomeV1::ExitCode(1),
        ] {
            let mut receipt = execution_receipt(&raw, 1);
            receipt.process_outcome = outcome;
            assert_eq!(
                parse_github_attestation_verifier_output_v1(&raw, &receipt),
                Err(GitHubVerificationResultParseErrorV1::ProcessDidNotSucceed)
            );
        }
    }

    #[test]
    fn outer_envelope_is_strict_but_nested_extensions_are_tolerated() {
        let mut candidate = candidate_json(Some("Tlog"));
        candidate
            .as_object_mut()
            .unwrap()
            .insert("unexpectedOuter".into(), Value::Bool(true));
        let raw = serde_json::to_vec(&vec![candidate]).unwrap();
        assert_eq!(
            parse_github_attestation_verifier_output_v1(&raw, &execution_receipt(&raw, 1)),
            Err(GitHubVerificationResultParseErrorV1::InvalidJson)
        );

        let raw = serde_json::to_vec(&vec![candidate_json(Some("Tlog"))]).unwrap();
        assert!(parse_github_attestation_verifier_output_v1(
            &raw,
            &execution_receipt(&raw, 1),
        )
        .is_ok());
    }

    #[test]
    fn result_count_is_bound_to_execution_receipt() {
        let raw = serde_json::to_vec(&vec![candidate_json(Some("Tlog"))]).unwrap();
        assert_eq!(
            parse_github_attestation_verifier_output_v1(&raw, &execution_receipt(&raw, 2)),
            Err(GitHubVerificationResultParseErrorV1::ExecutionResultCountMismatch {
                expected: 2,
                actual: 1,
            })
        );
    }

    #[test]
    fn output_is_parsed_evidence_only() {
        let raw = serde_json::to_vec(&vec![candidate_json(Some("Tlog"))]).unwrap();
        let parsed = parse_github_attestation_verifier_output_v1(
            &raw,
            &execution_receipt(&raw, 1),
        )
        .unwrap();
        assert_eq!(
            parsed.authority_scope(),
            GitHubParsedVerifierOutputAuthorityV1::ParsedVerifierOutputOnly
        );
        assert_eq!(parsed.stdout_digest_profile_id(), VERIFIER_STDOUT_DIGEST_PROFILE_V1);
        assert!(!parsed.establishes_trusted_builder_identity());
        assert!(!parsed.establishes_public_good_transparency());
        assert!(!parsed.establishes_receipt_authentication());
        assert!(!parsed.grants_production_authority());
        assert!(!parsed.grants_application_authority());
    }
}
