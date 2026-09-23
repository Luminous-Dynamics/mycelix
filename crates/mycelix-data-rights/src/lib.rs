#![forbid(unsafe_code)]

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

/// Intended-use dimensions are independent. Compatibility for one dimension
/// grants nothing about any other dimension.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum IntendedUseDimension {
    LocalStorage,
    ResearchAnalysis,
    Training,
    Evaluation,
    CommercialResearch,
    CommercialProductDevelopment,
    DerivedModelUse,
    DerivedModelDistribution,
    DerivedDatasetCreation,
    DerivedDatasetRedistribution,
    RawRedistribution,
    PublicationExcerpt,
    PublicDemo,
    RemoteProcessing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum DimensionDisposition {
    Compatible,
    Incompatible,
    Unknown,
    OutOfScope,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum TermsCurrentness {
    Current,
    Expired,
    Superseded,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum DatasetSplitRole {
    Training,
    Validation,
    EvaluationOnly,
    BenchmarkHoldout,
    Unspecified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum RetainedObligation {
    Attribution,
    NoticeRetention,
    NonCommercialUseOnly,
    NoRawRedistribution,
    SourceLineageRetention,
    ReviewOnTermsChange,
    DeletionOnRevocation,
    NoDerivedDatasetRedistribution,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum ExternalPrerequisite {
    PrivacyConsentAssessment,
}

/// Raw reviewed evidence is deserializable evidence input. It is not an
/// authority token and does not by itself create a compatibility assessment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewedTermsEvidenceV1 {
    pub subject: String,
    pub evidence_ref: String,
    pub currentness: TermsCurrentness,
    pub dimensions: BTreeMap<IntendedUseDimension, DimensionDisposition>,
    #[serde(default)]
    pub obligations: BTreeSet<RetainedObligation>,
}

/// Caller-requested use. This expresses what should be assessed; it cannot
/// rewrite reviewed evidence.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntendedDatasetUseV1 {
    pub subjects: BTreeSet<String>,
    pub dataset_split_role: DatasetSplitRole,
    pub dimensions: BTreeSet<IntendedUseDimension>,
    #[serde(default)]
    pub external_prerequisites: BTreeSet<ExternalPrerequisite>,
}

/// Aggregate state is descriptive compatibility under reviewed evidence only.
/// It is deliberately not an allow/deny execution token.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "PascalCase")]
pub enum AssessmentState {
    CompatibleWithReviewedTermsProfile,
    IncompatibleWithReviewedTermsProfile,
    HumanReviewRequired,
    TermsEvidenceConflict,
    TermsProfileExpiredOrSuperseded,
    IntendedUseOutOfProfile,
}

impl AssessmentState {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::CompatibleWithReviewedTermsProfile => "CompatibleWithReviewedTermsProfile",
            Self::IncompatibleWithReviewedTermsProfile => "IncompatibleWithReviewedTermsProfile",
            Self::HumanReviewRequired => "HumanReviewRequired",
            Self::TermsEvidenceConflict => "TermsEvidenceConflict",
            Self::TermsProfileExpiredOrSuperseded => "TermsProfileExpiredOrSuperseded",
            Self::IntendedUseOutOfProfile => "IntendedUseOutOfProfile",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct RetainedObligationEvidenceV1 {
    subject: String,
    evidence_ref: String,
    obligation: RetainedObligation,
}

impl RetainedObligationEvidenceV1 {
    pub fn subject(&self) -> &str {
        &self.subject
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }

    pub const fn obligation(&self) -> RetainedObligation {
        self.obligation
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DimensionEvidenceV1 {
    subject: String,
    dimension: IntendedUseDimension,
    dispositions: BTreeSet<DimensionDisposition>,
    evidence_refs: BTreeSet<String>,
}

impl DimensionEvidenceV1 {
    pub fn subject(&self) -> &str {
        &self.subject
    }

    pub const fn dimension(&self) -> IntendedUseDimension {
        self.dimension
    }

    pub fn dispositions(&self) -> &BTreeSet<DimensionDisposition> {
        &self.dispositions
    }

    pub fn evidence_refs(&self) -> &BTreeSet<String> {
        &self.evidence_refs
    }
}

/// Constructor-controlled output. It intentionally implements Serialize but
/// not Deserialize, so arbitrary JSON cannot mint an assessment object.
///
/// ```compile_fail
/// use mycelix_data_rights::DatasetTermsCompatibilityAssessmentV1;
/// let _: DatasetTermsCompatibilityAssessmentV1 = serde_json::from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DatasetTermsCompatibilityAssessmentV1 {
    state: AssessmentState,
    requested_subjects: BTreeSet<String>,
    requested_dimensions: BTreeSet<IntendedUseDimension>,
    dimension_evidence: Vec<DimensionEvidenceV1>,
    retained_obligations: Vec<RetainedObligationEvidenceV1>,
    external_prerequisites: BTreeSet<ExternalPrerequisite>,
}

impl DatasetTermsCompatibilityAssessmentV1 {
    pub const fn state(&self) -> AssessmentState {
        self.state
    }

    pub fn requested_subjects(&self) -> &BTreeSet<String> {
        &self.requested_subjects
    }

    pub fn requested_dimensions(&self) -> &BTreeSet<IntendedUseDimension> {
        &self.requested_dimensions
    }

    pub fn dimension_evidence(&self) -> &[DimensionEvidenceV1] {
        &self.dimension_evidence
    }

    pub fn retained_obligations(&self) -> &[RetainedObligationEvidenceV1] {
        &self.retained_obligations
    }

    pub fn external_prerequisites(&self) -> &BTreeSet<ExternalPrerequisite> {
        &self.external_prerequisites
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvidenceError {
    EmptyRequestedSubjects,
    EmptyRequestedDimensions,
    EmptySubject,
    EmptyEvidenceRef,
    ConflictingReuseOfEvidenceRef(String),
}

impl fmt::Display for EvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyRequestedSubjects => write!(f, "requested subject set is empty"),
            Self::EmptyRequestedDimensions => write!(f, "requested dimension set is empty"),
            Self::EmptySubject => write!(f, "subject identifiers must be non-empty"),
            Self::EmptyEvidenceRef => write!(f, "evidence references must be non-empty"),
            Self::ConflictingReuseOfEvidenceRef(reference) => {
                write!(
                    f,
                    "evidence reference is reused with conflicting content: {reference}"
                )
            }
        }
    }
}

impl Error for EvidenceError {}

fn retained_obligations(evidence: &[ReviewedTermsEvidenceV1]) -> Vec<RetainedObligationEvidenceV1> {
    let mut retained = BTreeSet::new();
    for row in evidence {
        for obligation in &row.obligations {
            retained.insert(RetainedObligationEvidenceV1 {
                subject: row.subject.clone(),
                evidence_ref: row.evidence_ref.clone(),
                obligation: *obligation,
            });
        }
    }
    retained.into_iter().collect()
}

fn build_dimension_evidence(
    evidence: &[ReviewedTermsEvidenceV1],
    request: &IntendedDatasetUseV1,
) -> Vec<DimensionEvidenceV1> {
    let mut out = Vec::new();
    for subject in &request.subjects {
        for dimension in &request.dimensions {
            let mut dispositions = BTreeSet::new();
            let mut evidence_refs = BTreeSet::new();
            for row in evidence.iter().filter(|row| &row.subject == subject) {
                if let Some(disposition) = row.dimensions.get(dimension) {
                    dispositions.insert(*disposition);
                    evidence_refs.insert(row.evidence_ref.clone());
                }
            }
            if dispositions.is_empty() {
                dispositions.insert(DimensionDisposition::OutOfScope);
            }
            out.push(DimensionEvidenceV1 {
                subject: subject.clone(),
                dimension: *dimension,
                dispositions,
                evidence_refs,
            });
        }
    }
    out
}

fn make_assessment(
    state: AssessmentState,
    evidence: &[ReviewedTermsEvidenceV1],
    request: &IntendedDatasetUseV1,
) -> DatasetTermsCompatibilityAssessmentV1 {
    DatasetTermsCompatibilityAssessmentV1 {
        state,
        requested_subjects: request.subjects.clone(),
        requested_dimensions: request.dimensions.clone(),
        dimension_evidence: build_dimension_evidence(evidence, request),
        retained_obligations: retained_obligations(evidence),
        external_prerequisites: request.external_prerequisites.clone(),
    }
}

fn validate_and_normalize_evidence(
    evidence: &[ReviewedTermsEvidenceV1],
) -> Result<Vec<ReviewedTermsEvidenceV1>, EvidenceError> {
    let mut by_ref: BTreeMap<String, ReviewedTermsEvidenceV1> = BTreeMap::new();
    for row in evidence {
        if row.subject.trim().is_empty() {
            return Err(EvidenceError::EmptySubject);
        }
        if row.evidence_ref.trim().is_empty() {
            return Err(EvidenceError::EmptyEvidenceRef);
        }
        match by_ref.get(&row.evidence_ref) {
            Some(existing) if existing != row => {
                return Err(EvidenceError::ConflictingReuseOfEvidenceRef(
                    row.evidence_ref.clone(),
                ));
            }
            Some(_) => {}
            None => {
                by_ref.insert(row.evidence_ref.clone(), row.clone());
            }
        }
    }
    Ok(by_ref.into_values().collect())
}

/// Evaluate exact supplied reviewed-terms evidence for the requested use.
///
/// This function reports compatibility semantics only. It does not grant legal
/// permission, provider approval, dataset access, privacy/consent approval,
/// training authority, publication authority, or physical execution authority.
pub fn assess_terms_compatibility(
    evidence: &[ReviewedTermsEvidenceV1],
    request: &IntendedDatasetUseV1,
) -> Result<DatasetTermsCompatibilityAssessmentV1, EvidenceError> {
    if request.subjects.is_empty() {
        return Err(EvidenceError::EmptyRequestedSubjects);
    }
    if request.dimensions.is_empty() {
        return Err(EvidenceError::EmptyRequestedDimensions);
    }
    if request
        .subjects
        .iter()
        .any(|subject| subject.trim().is_empty())
    {
        return Err(EvidenceError::EmptySubject);
    }

    let evidence = validate_and_normalize_evidence(evidence)?;
    if evidence.is_empty() {
        return Ok(make_assessment(
            AssessmentState::HumanReviewRequired,
            &evidence,
            request,
        ));
    }

    let observed_subjects: BTreeSet<_> = evidence.iter().map(|row| row.subject.clone()).collect();
    if observed_subjects != request.subjects {
        return Ok(make_assessment(
            AssessmentState::IntendedUseOutOfProfile,
            &evidence,
            request,
        ));
    }

    let mut currentness_by_subject: BTreeMap<String, BTreeSet<TermsCurrentness>> = BTreeMap::new();
    for row in &evidence {
        currentness_by_subject
            .entry(row.subject.clone())
            .or_default()
            .insert(row.currentness);
    }
    if currentness_by_subject
        .values()
        .any(|states| states.len() > 1)
    {
        return Ok(make_assessment(
            AssessmentState::TermsEvidenceConflict,
            &evidence,
            request,
        ));
    }
    if evidence.iter().any(|row| {
        matches!(
            row.currentness,
            TermsCurrentness::Expired | TermsCurrentness::Superseded
        )
    }) {
        return Ok(make_assessment(
            AssessmentState::TermsProfileExpiredOrSuperseded,
            &evidence,
            request,
        ));
    }
    if evidence
        .iter()
        .any(|row| row.currentness == TermsCurrentness::Unknown)
    {
        return Ok(make_assessment(
            AssessmentState::HumanReviewRequired,
            &evidence,
            request,
        ));
    }

    if request.dimensions.contains(&IntendedUseDimension::Training)
        && matches!(
            request.dataset_split_role,
            DatasetSplitRole::EvaluationOnly | DatasetSplitRole::BenchmarkHoldout
        )
    {
        return Ok(make_assessment(
            AssessmentState::IncompatibleWithReviewedTermsProfile,
            &evidence,
            request,
        ));
    }

    let dimension_evidence = build_dimension_evidence(&evidence, request);
    if dimension_evidence
        .iter()
        .any(|item| item.dispositions.len() > 1)
    {
        return Ok(make_assessment(
            AssessmentState::TermsEvidenceConflict,
            &evidence,
            request,
        ));
    }

    let observed: Vec<_> = dimension_evidence
        .iter()
        .filter_map(|item| item.dispositions.iter().next().copied())
        .collect();

    let state = if observed.contains(&DimensionDisposition::Incompatible) {
        AssessmentState::IncompatibleWithReviewedTermsProfile
    } else if observed.contains(&DimensionDisposition::Unknown) {
        AssessmentState::HumanReviewRequired
    } else if observed.contains(&DimensionDisposition::OutOfScope) {
        AssessmentState::IntendedUseOutOfProfile
    } else {
        AssessmentState::CompatibleWithReviewedTermsProfile
    };

    Ok(make_assessment(state, &evidence, request))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn training_request(subject: &str) -> IntendedDatasetUseV1 {
        IntendedDatasetUseV1 {
            subjects: [subject.to_string()].into_iter().collect(),
            dataset_split_role: DatasetSplitRole::Training,
            dimensions: [IntendedUseDimension::Training].into_iter().collect(),
            external_prerequisites: BTreeSet::new(),
        }
    }

    fn training_evidence(
        subject: &str,
        evidence_ref: &str,
        disposition: DimensionDisposition,
    ) -> ReviewedTermsEvidenceV1 {
        ReviewedTermsEvidenceV1 {
            subject: subject.to_string(),
            evidence_ref: evidence_ref.to_string(),
            currentness: TermsCurrentness::Current,
            dimensions: [(IntendedUseDimension::Training, disposition)]
                .into_iter()
                .collect(),
            obligations: BTreeSet::new(),
        }
    }

    #[test]
    fn duplicate_identical_evidence_ref_normalizes() {
        let row = training_evidence("dataset:a:v1", "review:a", DimensionDisposition::Compatible);
        let result =
            assess_terms_compatibility(&[row.clone(), row], &training_request("dataset:a:v1"))
                .expect("identical duplicate should normalize");
        assert_eq!(
            result.state(),
            AssessmentState::CompatibleWithReviewedTermsProfile
        );
        assert_eq!(result.dimension_evidence()[0].evidence_refs().len(), 1);
    }

    #[test]
    fn conflicting_reuse_of_same_evidence_ref_is_malformed() {
        let a = training_evidence("dataset:a:v1", "review:a", DimensionDisposition::Compatible);
        let b = training_evidence(
            "dataset:a:v1",
            "review:a",
            DimensionDisposition::Incompatible,
        );
        assert_eq!(
            assess_terms_compatibility(&[a, b], &training_request("dataset:a:v1")),
            Err(EvidenceError::ConflictingReuseOfEvidenceRef(
                "review:a".to_string()
            ))
        );
    }

    #[test]
    fn distinct_review_refs_can_expose_terms_conflict() {
        let a = training_evidence("dataset:a:v1", "review:a", DimensionDisposition::Compatible);
        let b = training_evidence(
            "dataset:a:v1",
            "review:b",
            DimensionDisposition::Incompatible,
        );
        let result =
            assess_terms_compatibility(&[a, b], &training_request("dataset:a:v1")).unwrap();
        assert_eq!(result.state(), AssessmentState::TermsEvidenceConflict);
    }

    #[test]
    fn compatibility_for_one_dimension_does_not_imply_another() {
        let evidence = ReviewedTermsEvidenceV1 {
            subject: "dataset:a:v1".to_string(),
            evidence_ref: "review:a".to_string(),
            currentness: TermsCurrentness::Current,
            dimensions: [(
                IntendedUseDimension::Training,
                DimensionDisposition::Compatible,
            )]
            .into_iter()
            .collect(),
            obligations: BTreeSet::new(),
        };
        let request = IntendedDatasetUseV1 {
            subjects: ["dataset:a:v1".to_string()].into_iter().collect(),
            dataset_split_role: DatasetSplitRole::Training,
            dimensions: [IntendedUseDimension::DerivedModelDistribution]
                .into_iter()
                .collect(),
            external_prerequisites: BTreeSet::new(),
        };
        let result = assess_terms_compatibility(&[evidence], &request).unwrap();
        assert_eq!(result.state(), AssessmentState::IntendedUseOutOfProfile);
    }

    #[test]
    fn external_privacy_prerequisite_survives_terms_compatibility() {
        let evidence =
            training_evidence("dataset:a:v1", "review:a", DimensionDisposition::Compatible);
        let mut request = training_request("dataset:a:v1");
        request
            .external_prerequisites
            .insert(ExternalPrerequisite::PrivacyConsentAssessment);
        let result = assess_terms_compatibility(&[evidence], &request).unwrap();
        assert_eq!(
            result.state(),
            AssessmentState::CompatibleWithReviewedTermsProfile
        );
        assert!(
            result
                .external_prerequisites()
                .contains(&ExternalPrerequisite::PrivacyConsentAssessment)
        );
    }
}
