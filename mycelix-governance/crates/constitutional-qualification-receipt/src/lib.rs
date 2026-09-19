use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

pub const RECEIPT_SCHEMA_VERSION: u16 = 1;
pub const RECEIPT_COMMITMENT_PREFIX: &str = "blake3-256:";
pub const ARTIFACT_DIGEST_PREFIX: &str = "sha256:";
pub const MAX_DEPENDENCY_ID_LEN: usize = 256;
pub const MAX_ISSUER_ID_LEN: usize = 256;
pub const MAX_NAVIGATION_TEXT_LEN: usize = 512;

const RECEIPT_COMMITMENT_DOMAIN: &[u8] = b"MYCELIX-QUALIFICATION-RECEIPT\0V1\0";

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ReceiptError {
    #[error("invalid {field}: {message}")]
    InvalidField {
        field: &'static str,
        message: String,
    },
    #[error("qualification receipt commitment mismatch")]
    CommitmentMismatch,
    #[error("duplicate qualification requirement for {0}")]
    DuplicateRequirement(String),
    #[error("duplicate qualification evidence for {0}")]
    DuplicateEvidence(String),
    #[error("unexpected qualification evidence for {0}")]
    UnexpectedEvidence(String),
    #[error("missing qualification evidence for {0}")]
    MissingEvidence(String),
    #[error("issuer mismatch for {dependency_id}: expected {expected}, got {actual}")]
    IssuerMismatch {
        dependency_id: String,
        expected: String,
        actual: String,
    },
    #[error("semantic head mismatch for {dependency_id}: expected {expected}, got {actual}")]
    SemanticHeadMismatch {
        dependency_id: String,
        expected: String,
        actual: String,
    },
    #[error("verifier head mismatch for {dependency_id}: expected {expected}, got {actual}")]
    VerifierHeadMismatch {
        dependency_id: String,
        expected: String,
        actual: String,
    },
    #[error("qualification dependency remains pending: {0}")]
    DependencyPending(String),
    #[error("qualification dependency failed: {dependency_id} ({evidence_id})")]
    DependencyFailed {
        dependency_id: String,
        evidence_id: String,
    },
}

pub type ReceiptResult<T> = Result<T, ReceiptError>;

fn invalid(field: &'static str, message: impl Into<String>) -> ReceiptError {
    ReceiptError::InvalidField {
        field,
        message: message.into(),
    }
}

fn require_opaque(field: &'static str, value: &str, max_len: usize) -> ReceiptResult<()> {
    if value.is_empty() || value.len() > max_len || value.trim() != value {
        return Err(invalid(
            field,
            format!("must be non-empty, trimmed, and <= {max_len} bytes"),
        ));
    }
    Ok(())
}

fn is_lower_hex(value: &str, len: usize) -> bool {
    value.len() == len
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn require_git_head(field: &'static str, value: &str) -> ReceiptResult<()> {
    if !is_lower_hex(value, 40) {
        return Err(invalid(
            field,
            "must be exactly 40 lowercase hexadecimal characters",
        ));
    }
    Ok(())
}

fn require_tagged_digest(
    field: &'static str,
    value: &str,
    prefix: &str,
) -> ReceiptResult<()> {
    let Some(hex) = value.strip_prefix(prefix) else {
        return Err(invalid(
            field,
            format!("must use {prefix}<64-lowercase-hex>"),
        ));
    };
    if !is_lower_hex(hex, 64) {
        return Err(invalid(
            field,
            format!("must use {prefix}<64-lowercase-hex>"),
        ));
    }
    Ok(())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn tagged_receipt_hash(hash: blake3::Hash) -> String {
    format!("{RECEIPT_COMMITMENT_PREFIX}{}", hash.to_hex())
}

/// Human/operator navigation metadata.
///
/// These fields are intentionally excluded from qualification authority and
/// from the deterministic receipt commitment. They may help locate retained
/// evidence, but workflow renames, PR metadata, or artifact storage IDs must not
/// redefine which exact qualification tuple earned authority.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReceiptNavigationV1 {
    pub job_id: Option<u64>,
    pub artifact_id: Option<u64>,
    pub workflow_name: Option<String>,
    pub evidence_label: Option<String>,
}

impl ReceiptNavigationV1 {
    pub fn validate(&self) -> ReceiptResult<()> {
        if self.job_id == Some(0) {
            return Err(invalid("navigation.job_id", "must be non-zero when present"));
        }
        if self.artifact_id == Some(0) {
            return Err(invalid(
                "navigation.artifact_id",
                "must be non-zero when present",
            ));
        }
        if let Some(value) = &self.workflow_name {
            require_opaque(
                "navigation.workflow_name",
                value,
                MAX_NAVIGATION_TEXT_LEN,
            )?;
        }
        if let Some(value) = &self.evidence_label {
            require_opaque(
                "navigation.evidence_label",
                value,
                MAX_NAVIGATION_TEXT_LEN,
            )?;
        }
        Ok(())
    }
}

/// Authority-bearing input used to construct a receipt without an oversized
/// positional constructor.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationReceiptIdentityV1 {
    pub dependency_id: String,
    pub issuer_id: String,
    pub semantic_head: String,
    pub verifier_head: String,
    pub run_id: u64,
    pub run_attempt: u32,
    pub artifact_digest: String,
}

impl QualificationReceiptIdentityV1 {
    pub fn validate(&self) -> ReceiptResult<()> {
        require_opaque(
            "dependency_id",
            &self.dependency_id,
            MAX_DEPENDENCY_ID_LEN,
        )?;
        require_opaque("issuer_id", &self.issuer_id, MAX_ISSUER_ID_LEN)?;
        require_git_head("semantic_head", &self.semantic_head)?;
        require_git_head("verifier_head", &self.verifier_head)?;
        if self.run_id == 0 {
            return Err(invalid("run_id", "must be non-zero"));
        }
        if self.run_attempt == 0 {
            return Err(invalid("run_attempt", "must be non-zero"));
        }
        require_tagged_digest(
            "artifact_digest",
            &self.artifact_digest,
            ARTIFACT_DIGEST_PREFIX,
        )?;
        Ok(())
    }
}

/// Exact, deterministic identity of a successful qualification receipt.
///
/// This proves only receipt *shape and identity*. Constructing or deserializing
/// this value is not proof that the named evidence authority actually issued it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationReceiptV1 {
    pub schema_version: u16,
    pub dependency_id: String,
    pub issuer_id: String,
    pub semantic_head: String,
    pub verifier_head: String,
    pub run_id: u64,
    pub run_attempt: u32,
    pub artifact_digest: String,
    pub receipt_commitment: String,
    #[serde(default)]
    pub navigation: ReceiptNavigationV1,
}

impl QualificationReceiptV1 {
    pub fn new(
        identity: QualificationReceiptIdentityV1,
        navigation: ReceiptNavigationV1,
    ) -> ReceiptResult<Self> {
        identity.validate()?;
        navigation.validate()?;
        let mut receipt = Self {
            schema_version: RECEIPT_SCHEMA_VERSION,
            dependency_id: identity.dependency_id,
            issuer_id: identity.issuer_id,
            semantic_head: identity.semantic_head,
            verifier_head: identity.verifier_head,
            run_id: identity.run_id,
            run_attempt: identity.run_attempt,
            artifact_digest: identity.artifact_digest,
            receipt_commitment: String::new(),
            navigation,
        };
        receipt.receipt_commitment = receipt.compute_authority_commitment();
        Ok(receipt)
    }

    pub fn validate(&self) -> ReceiptResult<()> {
        self.identity().validate()?;
        self.navigation.validate()?;
        require_tagged_digest(
            "receipt_commitment",
            &self.receipt_commitment,
            RECEIPT_COMMITMENT_PREFIX,
        )?;
        if self.receipt_commitment != self.compute_authority_commitment() {
            return Err(ReceiptError::CommitmentMismatch);
        }
        Ok(())
    }

    pub fn identity(&self) -> QualificationReceiptIdentityV1 {
        QualificationReceiptIdentityV1 {
            dependency_id: self.dependency_id.clone(),
            issuer_id: self.issuer_id.clone(),
            semantic_head: self.semantic_head.clone(),
            verifier_head: self.verifier_head.clone(),
            run_id: self.run_id,
            run_attempt: self.run_attempt,
            artifact_digest: self.artifact_digest.clone(),
        }
    }

    pub fn recompute_authority_commitment(&self) -> ReceiptResult<String> {
        self.identity().validate()?;
        Ok(self.compute_authority_commitment())
    }

    fn compute_authority_commitment(&self) -> String {
        let mut hasher = blake3::Hasher::new();
        hasher.update(RECEIPT_COMMITMENT_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.dependency_id);
        push_str(&mut hasher, &self.issuer_id);
        push_str(&mut hasher, &self.semantic_head);
        push_str(&mut hasher, &self.verifier_head);
        hasher.update(&self.run_id.to_be_bytes());
        hasher.update(&self.run_attempt.to_be_bytes());
        push_str(&mut hasher, &self.artifact_digest);
        tagged_receipt_hash(hasher.finalize())
    }
}

/// Exact qualification lineage required by an activation consumer.
///
/// The requirement pins the trusted evidence issuer and exact semantic/verifier
/// lineage. The accepted receipt still records the exact run attempt and artifact
/// that produced the evidence.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationRequirementV1 {
    pub dependency_id: String,
    pub issuer_id: String,
    pub semantic_head: String,
    pub verifier_head: String,
}

impl QualificationRequirementV1 {
    pub fn new(
        dependency_id: String,
        issuer_id: String,
        semantic_head: String,
        verifier_head: String,
    ) -> ReceiptResult<Self> {
        let requirement = Self {
            dependency_id,
            issuer_id,
            semantic_head,
            verifier_head,
        };
        requirement.validate()?;
        Ok(requirement)
    }

    pub fn validate(&self) -> ReceiptResult<()> {
        require_opaque(
            "requirement.dependency_id",
            &self.dependency_id,
            MAX_DEPENDENCY_ID_LEN,
        )?;
        require_opaque(
            "requirement.issuer_id",
            &self.issuer_id,
            MAX_ISSUER_ID_LEN,
        )?;
        require_git_head("requirement.semantic_head", &self.semantic_head)?;
        require_git_head("requirement.verifier_head", &self.verifier_head)?;
        Ok(())
    }

    pub fn validate_receipt(&self, receipt: &QualificationReceiptV1) -> ReceiptResult<()> {
        self.validate()?;
        receipt.validate()?;
        if receipt.dependency_id != self.dependency_id {
            return Err(ReceiptError::UnexpectedEvidence(
                receipt.dependency_id.clone(),
            ));
        }
        self.validate_identity(
            &receipt.issuer_id,
            &receipt.semantic_head,
            &receipt.verifier_head,
        )
    }

    fn validate_identity(
        &self,
        issuer_id: &str,
        semantic_head: &str,
        verifier_head: &str,
    ) -> ReceiptResult<()> {
        if issuer_id != self.issuer_id {
            return Err(ReceiptError::IssuerMismatch {
                dependency_id: self.dependency_id.clone(),
                expected: self.issuer_id.clone(),
                actual: issuer_id.to_string(),
            });
        }
        if semantic_head != self.semantic_head {
            return Err(ReceiptError::SemanticHeadMismatch {
                dependency_id: self.dependency_id.clone(),
                expected: self.semantic_head.clone(),
                actual: semantic_head.to_string(),
            });
        }
        if verifier_head != self.verifier_head {
            return Err(ReceiptError::VerifierHeadMismatch {
                dependency_id: self.dependency_id.clone(),
                expected: self.verifier_head.clone(),
                actual: verifier_head.to_string(),
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PendingQualificationV1 {
    pub dependency_id: String,
    pub issuer_id: String,
    pub semantic_head: String,
    pub verifier_head: String,
}

impl PendingQualificationV1 {
    fn validate(&self) -> ReceiptResult<()> {
        QualificationRequirementV1 {
            dependency_id: self.dependency_id.clone(),
            issuer_id: self.issuer_id.clone(),
            semantic_head: self.semantic_head.clone(),
            verifier_head: self.verifier_head.clone(),
        }
        .validate()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FailedQualificationV1 {
    pub dependency_id: String,
    pub issuer_id: String,
    pub semantic_head: String,
    pub verifier_head: String,
    pub evidence_id: String,
}

impl FailedQualificationV1 {
    fn validate(&self) -> ReceiptResult<()> {
        QualificationRequirementV1 {
            dependency_id: self.dependency_id.clone(),
            issuer_id: self.issuer_id.clone(),
            semantic_head: self.semantic_head.clone(),
            verifier_head: self.verifier_head.clone(),
        }
        .validate()?;
        require_opaque(
            "failed.evidence_id",
            &self.evidence_id,
            MAX_NAVIGATION_TEXT_LEN,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", content = "evidence")]
pub enum QualificationEvidenceV1 {
    Qualified(QualificationReceiptV1),
    Pending(PendingQualificationV1),
    Failed(FailedQualificationV1),
}

impl QualificationEvidenceV1 {
    pub fn dependency_id(&self) -> &str {
        match self {
            Self::Qualified(receipt) => &receipt.dependency_id,
            Self::Pending(pending) => &pending.dependency_id,
            Self::Failed(failed) => &failed.dependency_id,
        }
    }

    pub fn validate_shape(&self) -> ReceiptResult<()> {
        match self {
            Self::Qualified(receipt) => receipt.validate(),
            Self::Pending(pending) => pending.validate(),
            Self::Failed(failed) => failed.validate(),
        }
    }
}

/// Require an exact dependency census and accept only exact qualified receipts.
///
/// Missing, extra, duplicate, pending, failed, wrong-issuer, wrong-semantic, or
/// wrong-verifier evidence fails closed.
pub fn validate_exact_qualified_census(
    requirements: &[QualificationRequirementV1],
    evidence: &[QualificationEvidenceV1],
) -> ReceiptResult<()> {
    let mut requirement_ids = BTreeSet::new();
    for requirement in requirements {
        requirement.validate()?;
        if !requirement_ids.insert(requirement.dependency_id.as_str()) {
            return Err(ReceiptError::DuplicateRequirement(
                requirement.dependency_id.clone(),
            ));
        }
    }

    let mut evidence_ids = BTreeSet::new();
    for item in evidence {
        item.validate_shape()?;
        let dependency_id = item.dependency_id();
        if !evidence_ids.insert(dependency_id.to_string()) {
            return Err(ReceiptError::DuplicateEvidence(dependency_id.to_string()));
        }

        let Some(requirement) = requirements
            .iter()
            .find(|requirement| requirement.dependency_id == dependency_id)
        else {
            return Err(ReceiptError::UnexpectedEvidence(dependency_id.to_string()));
        };

        match item {
            QualificationEvidenceV1::Qualified(receipt) => {
                requirement.validate_receipt(receipt)?;
            }
            QualificationEvidenceV1::Pending(pending) => {
                requirement.validate_identity(
                    &pending.issuer_id,
                    &pending.semantic_head,
                    &pending.verifier_head,
                )?;
                return Err(ReceiptError::DependencyPending(dependency_id.to_string()));
            }
            QualificationEvidenceV1::Failed(failed) => {
                requirement.validate_identity(
                    &failed.issuer_id,
                    &failed.semantic_head,
                    &failed.verifier_head,
                )?;
                return Err(ReceiptError::DependencyFailed {
                    dependency_id: dependency_id.to_string(),
                    evidence_id: failed.evidence_id.clone(),
                });
            }
        }
    }

    for requirement in requirements {
        if !evidence_ids.contains(requirement.dependency_id.as_str()) {
            return Err(ReceiptError::MissingEvidence(
                requirement.dependency_id.clone(),
            ));
        }
    }

    Ok(())
}

/// Reserved representation for evidence that has passed a separately-qualified
/// authenticity/ingestion boundary.
///
/// This type deliberately has no public constructor and does not implement
/// Serialize, Deserialize, Clone, or Copy. QREC-001 defines exact receipt
/// identity only; a future qualification-ingestion tranche must earn the right
/// to construct this wrapper.
#[derive(Debug)]
pub struct VerifiedQualificationReceiptV1 {
    receipt: QualificationReceiptV1,
    _authenticated_ingestion: VerifiedIngestionSeal,
}

#[derive(Debug)]
struct VerifiedIngestionSeal;

impl VerifiedQualificationReceiptV1 {
    pub fn receipt(&self) -> &QualificationReceiptV1 {
        &self.receipt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ISSUER: &str = "github-repository:1176351975";

    fn git_hex(ch: char) -> String {
        ch.to_string().repeat(40)
    }

    fn sha256_hex(ch: char) -> String {
        format!("sha256:{}", ch.to_string().repeat(64))
    }

    fn nav(label: &str) -> ReceiptNavigationV1 {
        ReceiptNavigationV1 {
            job_id: Some(1001),
            artifact_id: Some(2002),
            workflow_name: Some("Exact qualification".into()),
            evidence_label: Some(label.into()),
        }
    }

    fn receipt_from(
        dependency_id: &str,
        issuer_id: &str,
        semantic_ch: char,
        verifier_ch: char,
        run_id: u64,
        run_attempt: u32,
        digest_ch: char,
    ) -> QualificationReceiptV1 {
        QualificationReceiptV1::new(
            QualificationReceiptIdentityV1 {
                dependency_id: dependency_id.into(),
                issuer_id: issuer_id.into(),
                semantic_head: git_hex(semantic_ch),
                verifier_head: git_hex(verifier_ch),
                run_id,
                run_attempt,
                artifact_digest: sha256_hex(digest_ch),
            },
            nav("test evidence"),
        )
        .unwrap()
    }

    fn receipt(
        dependency_id: &str,
        semantic_ch: char,
        verifier_ch: char,
        run_id: u64,
        digest_ch: char,
    ) -> QualificationReceiptV1 {
        receipt_from(
            dependency_id,
            ISSUER,
            semantic_ch,
            verifier_ch,
            run_id,
            1,
            digest_ch,
        )
    }

    fn requirement(
        dependency_id: &str,
        semantic_ch: char,
        verifier_ch: char,
    ) -> QualificationRequirementV1 {
        QualificationRequirementV1::new(
            dependency_id.into(),
            ISSUER.into(),
            git_hex(semantic_ch),
            git_hex(verifier_ch),
        )
        .unwrap()
    }

    #[test]
    fn valid_receipt_round_trips_and_revalidates() {
        let receipt = receipt("MYC-CONST-TEST-A", 'a', 'b', 42, 'c');
        receipt.validate().unwrap();
        let encoded = serde_json::to_string(&receipt).unwrap();
        let decoded: QualificationReceiptV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, receipt);
        decoded.validate().unwrap();
        assert!(decoded
            .receipt_commitment
            .starts_with(RECEIPT_COMMITMENT_PREFIX));
    }

    #[test]
    fn navigation_metadata_is_not_qualification_authority() {
        let first = receipt("MYC-CONST-TEST-A", 'a', 'b', 42, 'c');
        let mut second = first.clone();
        second.navigation = ReceiptNavigationV1 {
            job_id: Some(9999),
            artifact_id: Some(8888),
            workflow_name: Some("Renamed workflow".into()),
            evidence_label: Some("different human label".into()),
        };
        second.validate().unwrap();
        assert_eq!(
            first.receipt_commitment,
            second.receipt_commitment,
            "navigation metadata must not redefine qualification identity"
        );
    }

    #[test]
    fn every_authority_tuple_field_changes_the_commitment() {
        let baseline = receipt("MYC-CONST-TEST-A", 'a', 'b', 42, 'c');
        let changed_dependency = receipt("MYC-CONST-TEST-B", 'a', 'b', 42, 'c');
        let changed_issuer = receipt_from(
            "MYC-CONST-TEST-A",
            "github-repository:999",
            'a',
            'b',
            42,
            1,
            'c',
        );
        let changed_semantic = receipt("MYC-CONST-TEST-A", 'd', 'b', 42, 'c');
        let changed_verifier = receipt("MYC-CONST-TEST-A", 'a', 'e', 42, 'c');
        let changed_run = receipt("MYC-CONST-TEST-A", 'a', 'b', 43, 'c');
        let changed_attempt = receipt_from(
            "MYC-CONST-TEST-A",
            ISSUER,
            'a',
            'b',
            42,
            2,
            'c',
        );
        let changed_digest = receipt("MYC-CONST-TEST-A", 'a', 'b', 42, 'd');

        for changed in [
            changed_dependency,
            changed_issuer,
            changed_semantic,
            changed_verifier,
            changed_run,
            changed_attempt,
            changed_digest,
        ] {
            assert_ne!(baseline.receipt_commitment, changed.receipt_commitment);
        }
    }

    #[test]
    fn zero_run_id_zero_attempt_and_malformed_digest_fail_closed() {
        let base = QualificationReceiptIdentityV1 {
            dependency_id: "MYC-CONST-TEST-A".into(),
            issuer_id: ISSUER.into(),
            semantic_head: git_hex('a'),
            verifier_head: git_hex('b'),
            run_id: 1,
            run_attempt: 1,
            artifact_digest: sha256_hex('c'),
        };

        let mut zero_run = base.clone();
        zero_run.run_id = 0;
        assert!(QualificationReceiptV1::new(zero_run, ReceiptNavigationV1::default()).is_err());

        let mut zero_attempt = base.clone();
        zero_attempt.run_attempt = 0;
        assert!(
            QualificationReceiptV1::new(zero_attempt, ReceiptNavigationV1::default()).is_err()
        );

        let mut malformed = base;
        malformed.artifact_digest = "sha256:not-a-digest".into();
        assert!(QualificationReceiptV1::new(malformed, ReceiptNavigationV1::default()).is_err());
    }

    #[test]
    fn stored_commitment_mutation_is_detected() {
        let mut receipt = receipt("MYC-CONST-TEST-A", 'a', 'b', 42, 'c');
        receipt.run_id = 777;
        assert_eq!(receipt.validate(), Err(ReceiptError::CommitmentMismatch));
    }

    #[test]
    fn requirement_rejects_wrong_issuer_semantic_and_verifier_heads() {
        let requirement = requirement("MYC-CONST-TEST-A", 'a', 'b');
        let wrong_issuer = receipt_from(
            "MYC-CONST-TEST-A",
            "github-repository:999",
            'a',
            'b',
            42,
            1,
            'd',
        );
        let wrong_semantic = receipt("MYC-CONST-TEST-A", 'c', 'b', 42, 'd');
        let wrong_verifier = receipt("MYC-CONST-TEST-A", 'a', 'c', 42, 'd');

        assert!(matches!(
            requirement.validate_receipt(&wrong_issuer),
            Err(ReceiptError::IssuerMismatch { .. })
        ));
        assert!(matches!(
            requirement.validate_receipt(&wrong_semantic),
            Err(ReceiptError::SemanticHeadMismatch { .. })
        ));
        assert!(matches!(
            requirement.validate_receipt(&wrong_verifier),
            Err(ReceiptError::VerifierHeadMismatch { .. })
        ));
    }

    #[test]
    fn exact_census_accepts_only_exact_qualified_receipts() {
        let requirements = vec![
            requirement("MYC-CONST-TEST-A", 'a', 'b'),
            requirement("MYC-CONST-TEST-B", 'c', 'd'),
        ];
        let evidence = vec![
            QualificationEvidenceV1::Qualified(receipt(
                "MYC-CONST-TEST-A",
                'a',
                'b',
                11,
                'e',
            )),
            QualificationEvidenceV1::Qualified(receipt(
                "MYC-CONST-TEST-B",
                'c',
                'd',
                12,
                'f',
            )),
        ];
        validate_exact_qualified_census(&requirements, &evidence).unwrap();
    }

    #[test]
    fn exact_census_rejects_duplicate_missing_and_extra_evidence() {
        let requirements = vec![
            requirement("MYC-CONST-TEST-A", 'a', 'b'),
            requirement("MYC-CONST-TEST-B", 'c', 'd'),
        ];
        let a = QualificationEvidenceV1::Qualified(receipt(
            "MYC-CONST-TEST-A",
            'a',
            'b',
            11,
            'e',
        ));

        assert!(matches!(
            validate_exact_qualified_census(&requirements, &[a.clone(), a.clone()]),
            Err(ReceiptError::DuplicateEvidence(_))
        ));
        assert!(matches!(
            validate_exact_qualified_census(&requirements, std::slice::from_ref(&a)),
            Err(ReceiptError::MissingEvidence(_))
        ));

        let extra = QualificationEvidenceV1::Qualified(receipt(
            "MYC-CONST-TEST-C",
            'e',
            'f',
            13,
            'a',
        ));
        assert!(matches!(
            validate_exact_qualified_census(&requirements[..1], &[a, extra]),
            Err(ReceiptError::UnexpectedEvidence(_))
        ));
    }

    #[test]
    fn exact_census_rejects_pending_and_failed_dependencies() {
        let requirements = vec![requirement("MYC-CONST-TEST-A", 'a', 'b')];
        let pending = QualificationEvidenceV1::Pending(PendingQualificationV1 {
            dependency_id: "MYC-CONST-TEST-A".into(),
            issuer_id: ISSUER.into(),
            semantic_head: git_hex('a'),
            verifier_head: git_hex('b'),
        });
        assert_eq!(
            validate_exact_qualified_census(&requirements, &[pending]),
            Err(ReceiptError::DependencyPending(
                "MYC-CONST-TEST-A".into()
            ))
        );

        let failed = QualificationEvidenceV1::Failed(FailedQualificationV1 {
            dependency_id: "MYC-CONST-TEST-A".into(),
            issuer_id: ISSUER.into(),
            semantic_head: git_hex('a'),
            verifier_head: git_hex('b'),
            evidence_id: "failed-run-123".into(),
        });
        assert!(matches!(
            validate_exact_qualified_census(&requirements, &[failed]),
            Err(ReceiptError::DependencyFailed { .. })
        ));
    }

    #[test]
    fn exact_census_rejects_duplicate_requirements() {
        let requirement = requirement("MYC-CONST-TEST-A", 'a', 'b');
        let evidence = QualificationEvidenceV1::Qualified(receipt(
            "MYC-CONST-TEST-A",
            'a',
            'b',
            11,
            'e',
        ));
        assert!(matches!(
            validate_exact_qualified_census(&[requirement.clone(), requirement], &[evidence]),
            Err(ReceiptError::DuplicateRequirement(_))
        ));
    }
}
