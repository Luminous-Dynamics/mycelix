#![forbid(unsafe_code)]

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum PrivacyPurpose {
    PerceptionResearch,
    ActionForecasting,
    GazeAffordanceResearch,
    ImitationLearning,
    RobotPolicyTraining,
    BenchmarkEvaluation,
    HumanBehaviorResearch,
    PublicVisualization,
    RawDataSharing,
    DerivedRepresentationSharing,
    IdentityRecognition,
    BiometricIdentification,
    SensitiveAttributeInference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum Modality {
    RgbVideo,
    Audio,
    Gaze,
    HeadPose,
    BodyPose,
    HandPose,
    ObjectPose,
    Depth,
    Imu,
    SpatialMap,
    DerivedEmbedding,
    DerivedPoseOnly,
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
pub enum ParticipantEvidenceState {
    Supported,
    Unknown,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum BystanderEvidenceState {
    Supported,
    Unknown,
    NotApplicable,
    HandledByReviewedTransformation,
    NotPresentInRequestedRepresentation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum PrivacyPurposeCurrentness {
    Current,
    Expired,
    Superseded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum PurposeDisposition {
    Compatible,
    Incompatible,
    Unknown,
    OutOfScope,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum PrivacyObligation {
    SourcePrivacyLineageRetention,
    DeleteOnRevocation,
    ReviewOnProfileChange,
    NoRawHumanMediaExport,
    RetainTransformationProvenance,
    NoIdentityInference,
    NoSensitiveAttributeInference,
    NoPublicVisualization,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum PrivacyNonclaim {
    ReidentificationImpossible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum ExternalPrivacyPrerequisite {
    ReviewedPrivacyConsentEvidence,
    SeparatePublicationAdmission,
    SeparatePhysicalExecutionAdmission,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum TransformReviewState {
    ReviewedForDeclaredDerivedOutput,
}

/// Raw reviewed privacy/purpose evidence. This is deserializable evidence input,
/// not a consent, legal-compliance, publication, training, or robot-control token.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewedPrivacyPurposeEvidenceV1 {
    pub subject: String,
    pub evidence_ref: String,
    pub currentness: PrivacyPurposeCurrentness,
    pub participant_evidence: ParticipantEvidenceState,
    pub bystander_evidence: BystanderEvidenceState,
    pub purpose_dispositions: BTreeMap<PrivacyPurpose, PurposeDisposition>,
    #[serde(default)]
    pub obligations: BTreeSet<PrivacyObligation>,
    #[serde(default)]
    pub nonclaims: BTreeSet<PrivacyNonclaim>,
    #[serde(default)]
    pub transformation_refs: BTreeSet<String>,
}

/// Exact reviewed transformation evidence supplied with the request. A reviewed
/// transform is scoped to its declared output modalities only.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewedTransformationV1 {
    pub transform_ref: String,
    pub review_state: TransformReviewState,
    pub output_modalities: BTreeSet<Modality>,
}

/// Caller-requested use. Source availability does not imply every modality is
/// requested or should be materialized.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntendedPrivacyPurposeUseV1 {
    pub subjects: BTreeSet<String>,
    pub dataset_split_role: DatasetSplitRole,
    pub source_modalities: BTreeSet<Modality>,
    pub requested_purpose: PrivacyPurpose,
    pub requested_modalities: BTreeSet<Modality>,
    #[serde(default)]
    pub excluded_modalities: BTreeSet<Modality>,
    #[serde(default)]
    pub reviewed_transformations: Vec<ReviewedTransformationV1>,
    #[serde(default)]
    pub external_prerequisites: BTreeSet<ExternalPrivacyPrerequisite>,
}

/// Descriptive compatibility under exact reviewed privacy/purpose evidence.
/// These states are not execution or permission tokens.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "PascalCase")]
pub enum PrivacyPurposeAssessmentState {
    CompatibleWithReviewedPrivacyPurposeProfile,
    IncompatibleWithReviewedPrivacyPurposeProfile,
    HumanReviewRequired,
    ConsentEvidenceUnavailable,
    ConsentOrPurposeProfileExpiredOrSuperseded,
    PurposeOutOfReviewedScope,
    EvidenceConflict,
}

impl PrivacyPurposeAssessmentState {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::CompatibleWithReviewedPrivacyPurposeProfile => {
                "CompatibleWithReviewedPrivacyPurposeProfile"
            }
            Self::IncompatibleWithReviewedPrivacyPurposeProfile => {
                "IncompatibleWithReviewedPrivacyPurposeProfile"
            }
            Self::HumanReviewRequired => "HumanReviewRequired",
            Self::ConsentEvidenceUnavailable => "ConsentEvidenceUnavailable",
            Self::ConsentOrPurposeProfileExpiredOrSuperseded => {
                "ConsentOrPurposeProfileExpiredOrSuperseded"
            }
            Self::PurposeOutOfReviewedScope => "PurposeOutOfReviewedScope",
            Self::EvidenceConflict => "EvidenceConflict",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct RetainedPrivacyObligationEvidenceV1 {
    subject: String,
    evidence_ref: String,
    obligation: PrivacyObligation,
}

impl RetainedPrivacyObligationEvidenceV1 {
    pub fn subject(&self) -> &str {
        &self.subject
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }

    pub const fn obligation(&self) -> PrivacyObligation {
        self.obligation
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct RetainedPrivacyNonclaimEvidenceV1 {
    subject: String,
    evidence_ref: String,
    nonclaim: PrivacyNonclaim,
}

impl RetainedPrivacyNonclaimEvidenceV1 {
    pub fn subject(&self) -> &str {
        &self.subject
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }

    pub const fn nonclaim(&self) -> PrivacyNonclaim {
        self.nonclaim
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct PrivacyPurposeEvidenceV1 {
    subject: String,
    purpose: PrivacyPurpose,
    dispositions: BTreeSet<PurposeDisposition>,
    evidence_refs: BTreeSet<String>,
    participant_states: BTreeSet<ParticipantEvidenceState>,
    bystander_states: BTreeSet<BystanderEvidenceState>,
    currentness_states: BTreeSet<PrivacyPurposeCurrentness>,
}

impl PrivacyPurposeEvidenceV1 {
    pub fn subject(&self) -> &str {
        &self.subject
    }

    pub const fn purpose(&self) -> PrivacyPurpose {
        self.purpose
    }

    pub fn dispositions(&self) -> &BTreeSet<PurposeDisposition> {
        &self.dispositions
    }

    pub fn evidence_refs(&self) -> &BTreeSet<String> {
        &self.evidence_refs
    }

    pub fn participant_states(&self) -> &BTreeSet<ParticipantEvidenceState> {
        &self.participant_states
    }

    pub fn bystander_states(&self) -> &BTreeSet<BystanderEvidenceState> {
        &self.bystander_states
    }

    pub fn currentness_states(&self) -> &BTreeSet<PrivacyPurposeCurrentness> {
        &self.currentness_states
    }
}

/// Constructor-controlled output. It intentionally implements Serialize but
/// not Deserialize, so arbitrary JSON cannot mint an assessment object.
///
/// ```compile_fail
/// use mycelix_data_privacy::PrivacyPurposeCompatibilityAssessmentV1;
/// let _: PrivacyPurposeCompatibilityAssessmentV1 = serde_json::from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct PrivacyPurposeCompatibilityAssessmentV1 {
    state: PrivacyPurposeAssessmentState,
    requested_subjects: BTreeSet<String>,
    requested_purpose: PrivacyPurpose,
    source_modalities: BTreeSet<Modality>,
    requested_modalities: BTreeSet<Modality>,
    excluded_modalities: BTreeSet<Modality>,
    purpose_evidence: Vec<PrivacyPurposeEvidenceV1>,
    retained_obligations: Vec<RetainedPrivacyObligationEvidenceV1>,
    retained_nonclaims: Vec<RetainedPrivacyNonclaimEvidenceV1>,
    reviewed_transform_refs: BTreeSet<String>,
    external_prerequisites: BTreeSet<ExternalPrivacyPrerequisite>,
}

impl PrivacyPurposeCompatibilityAssessmentV1 {
    pub const fn state(&self) -> PrivacyPurposeAssessmentState {
        self.state
    }

    pub fn requested_subjects(&self) -> &BTreeSet<String> {
        &self.requested_subjects
    }

    pub const fn requested_purpose(&self) -> PrivacyPurpose {
        self.requested_purpose
    }

    pub fn source_modalities(&self) -> &BTreeSet<Modality> {
        &self.source_modalities
    }

    pub fn requested_modalities(&self) -> &BTreeSet<Modality> {
        &self.requested_modalities
    }

    pub fn excluded_modalities(&self) -> &BTreeSet<Modality> {
        &self.excluded_modalities
    }

    pub fn purpose_evidence(&self) -> &[PrivacyPurposeEvidenceV1] {
        &self.purpose_evidence
    }

    pub fn retained_obligations(&self) -> &[RetainedPrivacyObligationEvidenceV1] {
        &self.retained_obligations
    }

    pub fn retained_nonclaims(&self) -> &[RetainedPrivacyNonclaimEvidenceV1] {
        &self.retained_nonclaims
    }

    pub fn reviewed_transform_refs(&self) -> &BTreeSet<String> {
        &self.reviewed_transform_refs
    }

    pub fn external_prerequisites(&self) -> &BTreeSet<ExternalPrivacyPrerequisite> {
        &self.external_prerequisites
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PrivacyEvidenceError {
    EmptyRequestedSubjects,
    EmptyRequestedModalities,
    EmptySubject,
    EmptyEvidenceRef,
    EmptyTransformationRef,
    ConflictingReuseOfEvidenceRef(String),
    ConflictingReuseOfTransformationRef(String),
    RequestedAndExcludedOverlap(Modality),
    ExcludedModalityNotInSource(Modality),
    RequestedModalityUnavailable(Modality),
    MissingReviewedTransformation(String),
    RequestedModalityOutsideReviewedTransformation(Modality),
}

impl fmt::Display for PrivacyEvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyRequestedSubjects => write!(f, "requested subject set is empty"),
            Self::EmptyRequestedModalities => write!(f, "requested modality set is empty"),
            Self::EmptySubject => write!(f, "subject identifiers must be non-empty"),
            Self::EmptyEvidenceRef => write!(f, "evidence references must be non-empty"),
            Self::EmptyTransformationRef => {
                write!(f, "transformation references must be non-empty")
            }
            Self::ConflictingReuseOfEvidenceRef(reference) => write!(
                f,
                "evidence reference is reused with conflicting content: {reference}"
            ),
            Self::ConflictingReuseOfTransformationRef(reference) => write!(
                f,
                "transformation reference is reused with conflicting content: {reference}"
            ),
            Self::RequestedAndExcludedOverlap(modality) => {
                write!(f, "requested modality is also excluded: {modality:?}")
            }
            Self::ExcludedModalityNotInSource(modality) => {
                write!(
                    f,
                    "excluded modality is not present in source: {modality:?}"
                )
            }
            Self::RequestedModalityUnavailable(modality) => {
                write!(f, "requested modality is unavailable: {modality:?}")
            }
            Self::MissingReviewedTransformation(reference) => {
                write!(f, "reviewed transformation is missing: {reference}")
            }
            Self::RequestedModalityOutsideReviewedTransformation(modality) => write!(
                f,
                "requested modality is outside reviewed transformation outputs: {modality:?}"
            ),
        }
    }
}

impl Error for PrivacyEvidenceError {}

fn validate_and_normalize_evidence(
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
) -> Result<Vec<ReviewedPrivacyPurposeEvidenceV1>, PrivacyEvidenceError> {
    let mut by_ref: BTreeMap<String, ReviewedPrivacyPurposeEvidenceV1> = BTreeMap::new();
    for row in evidence {
        if row.subject.trim().is_empty() {
            return Err(PrivacyEvidenceError::EmptySubject);
        }
        if row.evidence_ref.trim().is_empty() {
            return Err(PrivacyEvidenceError::EmptyEvidenceRef);
        }
        if row
            .transformation_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err(PrivacyEvidenceError::EmptyTransformationRef);
        }
        match by_ref.get(&row.evidence_ref) {
            Some(existing) if existing != row => {
                return Err(PrivacyEvidenceError::ConflictingReuseOfEvidenceRef(
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

fn normalize_transformations(
    request: &IntendedPrivacyPurposeUseV1,
) -> Result<BTreeMap<String, ReviewedTransformationV1>, PrivacyEvidenceError> {
    let mut by_ref = BTreeMap::new();
    for transform in &request.reviewed_transformations {
        if transform.transform_ref.trim().is_empty() {
            return Err(PrivacyEvidenceError::EmptyTransformationRef);
        }
        match by_ref.get(&transform.transform_ref) {
            Some(existing) if existing != transform => {
                return Err(PrivacyEvidenceError::ConflictingReuseOfTransformationRef(
                    transform.transform_ref.clone(),
                ));
            }
            Some(_) => {}
            None => {
                by_ref.insert(transform.transform_ref.clone(), transform.clone());
            }
        }
    }
    Ok(by_ref)
}

fn validate_request(
    request: &IntendedPrivacyPurposeUseV1,
    transforms: &BTreeMap<String, ReviewedTransformationV1>,
) -> Result<(), PrivacyEvidenceError> {
    if request.subjects.is_empty() {
        return Err(PrivacyEvidenceError::EmptyRequestedSubjects);
    }
    if request.requested_modalities.is_empty() {
        return Err(PrivacyEvidenceError::EmptyRequestedModalities);
    }
    if request
        .subjects
        .iter()
        .any(|subject| subject.trim().is_empty())
    {
        return Err(PrivacyEvidenceError::EmptySubject);
    }

    if let Some(modality) = request
        .requested_modalities
        .intersection(&request.excluded_modalities)
        .next()
    {
        return Err(PrivacyEvidenceError::RequestedAndExcludedOverlap(*modality));
    }
    if let Some(modality) = request
        .excluded_modalities
        .difference(&request.source_modalities)
        .next()
    {
        return Err(PrivacyEvidenceError::ExcludedModalityNotInSource(*modality));
    }

    let mut available = request.source_modalities.clone();
    for transform in transforms.values() {
        available.extend(transform.output_modalities.iter().copied());
    }
    if let Some(modality) = request.requested_modalities.difference(&available).next() {
        return Err(PrivacyEvidenceError::RequestedModalityUnavailable(
            *modality,
        ));
    }
    Ok(())
}

fn retained_obligations(
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
) -> Vec<RetainedPrivacyObligationEvidenceV1> {
    let mut retained = BTreeSet::new();
    for row in evidence {
        for obligation in &row.obligations {
            retained.insert(RetainedPrivacyObligationEvidenceV1 {
                subject: row.subject.clone(),
                evidence_ref: row.evidence_ref.clone(),
                obligation: *obligation,
            });
        }
    }
    retained.into_iter().collect()
}

fn retained_nonclaims(
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
) -> Vec<RetainedPrivacyNonclaimEvidenceV1> {
    let mut retained = BTreeSet::new();
    for row in evidence {
        for nonclaim in &row.nonclaims {
            retained.insert(RetainedPrivacyNonclaimEvidenceV1 {
                subject: row.subject.clone(),
                evidence_ref: row.evidence_ref.clone(),
                nonclaim: *nonclaim,
            });
        }
    }
    retained.into_iter().collect()
}

fn build_purpose_evidence(
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
    request: &IntendedPrivacyPurposeUseV1,
) -> Vec<PrivacyPurposeEvidenceV1> {
    let mut out = Vec::new();
    for subject in &request.subjects {
        let rows: Vec<_> = evidence
            .iter()
            .filter(|row| &row.subject == subject)
            .collect();
        let mut dispositions = BTreeSet::new();
        let mut evidence_refs = BTreeSet::new();
        let mut participant_states = BTreeSet::new();
        let mut bystander_states = BTreeSet::new();
        let mut currentness_states = BTreeSet::new();
        for row in rows {
            dispositions.insert(
                row.purpose_dispositions
                    .get(&request.requested_purpose)
                    .copied()
                    .unwrap_or(PurposeDisposition::OutOfScope),
            );
            evidence_refs.insert(row.evidence_ref.clone());
            participant_states.insert(row.participant_evidence);
            bystander_states.insert(row.bystander_evidence);
            currentness_states.insert(row.currentness);
        }
        if dispositions.is_empty() {
            dispositions.insert(PurposeDisposition::OutOfScope);
        }
        out.push(PrivacyPurposeEvidenceV1 {
            subject: subject.clone(),
            purpose: request.requested_purpose,
            dispositions,
            evidence_refs,
            participant_states,
            bystander_states,
            currentness_states,
        });
    }
    out
}

fn make_assessment(
    state: PrivacyPurposeAssessmentState,
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
    request: &IntendedPrivacyPurposeUseV1,
    transforms: &BTreeMap<String, ReviewedTransformationV1>,
) -> PrivacyPurposeCompatibilityAssessmentV1 {
    PrivacyPurposeCompatibilityAssessmentV1 {
        state,
        requested_subjects: request.subjects.clone(),
        requested_purpose: request.requested_purpose,
        source_modalities: request.source_modalities.clone(),
        requested_modalities: request.requested_modalities.clone(),
        excluded_modalities: request.excluded_modalities.clone(),
        purpose_evidence: build_purpose_evidence(evidence, request),
        retained_obligations: retained_obligations(evidence),
        retained_nonclaims: retained_nonclaims(evidence),
        reviewed_transform_refs: transforms.keys().cloned().collect(),
        external_prerequisites: request.external_prerequisites.clone(),
    }
}

fn validate_transformation_binding(
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
    request: &IntendedPrivacyPurposeUseV1,
    transforms: &BTreeMap<String, ReviewedTransformationV1>,
) -> Result<(), PrivacyEvidenceError> {
    let mut required_outputs = BTreeSet::new();
    let mut requires_transform = false;

    for row in evidence {
        if row.bystander_evidence == BystanderEvidenceState::HandledByReviewedTransformation {
            requires_transform = true;
            if row.transformation_refs.is_empty() {
                return Err(PrivacyEvidenceError::MissingReviewedTransformation(
                    row.evidence_ref.clone(),
                ));
            }
            for reference in &row.transformation_refs {
                let transform = transforms.get(reference).ok_or_else(|| {
                    PrivacyEvidenceError::MissingReviewedTransformation(reference.clone())
                })?;
                required_outputs.extend(transform.output_modalities.iter().copied());
            }
        }
    }

    if requires_transform
        && let Some(modality) = request
            .requested_modalities
            .difference(&required_outputs)
            .next()
    {
        return Err(
            PrivacyEvidenceError::RequestedModalityOutsideReviewedTransformation(*modality),
        );
    }
    Ok(())
}

/// Evaluate exact supplied reviewed privacy/purpose evidence for a requested use.
///
/// This function reports evidence compatibility only. It does not establish a
/// real person's consent, legal/privacy compliance, ethics approval,
/// de-identification effectiveness, training/publication authority, identity or
/// biometric permission, or physical robot execution authority.
pub fn assess_privacy_purpose_compatibility(
    evidence: &[ReviewedPrivacyPurposeEvidenceV1],
    request: &IntendedPrivacyPurposeUseV1,
) -> Result<PrivacyPurposeCompatibilityAssessmentV1, PrivacyEvidenceError> {
    let evidence = validate_and_normalize_evidence(evidence)?;
    let transforms = normalize_transformations(request)?;
    validate_request(request, &transforms)?;

    if evidence.is_empty() {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::HumanReviewRequired,
            &evidence,
            request,
            &transforms,
        ));
    }

    let observed_subjects: BTreeSet<_> = evidence.iter().map(|row| row.subject.clone()).collect();
    if observed_subjects != request.subjects {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::PurposeOutOfReviewedScope,
            &evidence,
            request,
            &transforms,
        ));
    }

    let purpose_evidence = build_purpose_evidence(&evidence, request);
    if purpose_evidence.iter().any(|item| {
        item.currentness_states.len() > 1
            || item.participant_states.len() > 1
            || item.bystander_states.len() > 1
            || item.dispositions.len() > 1
    }) {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::EvidenceConflict,
            &evidence,
            request,
            &transforms,
        ));
    }

    if evidence.iter().any(|row| {
        matches!(
            row.currentness,
            PrivacyPurposeCurrentness::Expired | PrivacyPurposeCurrentness::Superseded
        )
    }) {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::ConsentOrPurposeProfileExpiredOrSuperseded,
            &evidence,
            request,
            &transforms,
        ));
    }

    if evidence
        .iter()
        .any(|row| row.participant_evidence == ParticipantEvidenceState::Unavailable)
    {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::ConsentEvidenceUnavailable,
            &evidence,
            request,
            &transforms,
        ));
    }

    if evidence.iter().any(|row| {
        row.participant_evidence == ParticipantEvidenceState::Unknown
            || row.bystander_evidence == BystanderEvidenceState::Unknown
    }) {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::HumanReviewRequired,
            &evidence,
            request,
            &transforms,
        ));
    }

    if evidence.iter().any(|row| {
        row.bystander_evidence == BystanderEvidenceState::NotPresentInRequestedRepresentation
    }) && request
        .requested_modalities
        .iter()
        .any(|modality| matches!(modality, Modality::RgbVideo | Modality::Audio))
    {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::HumanReviewRequired,
            &evidence,
            request,
            &transforms,
        ));
    }

    validate_transformation_binding(&evidence, request, &transforms)?;

    if request.requested_purpose == PrivacyPurpose::RobotPolicyTraining
        && matches!(
            request.dataset_split_role,
            DatasetSplitRole::EvaluationOnly | DatasetSplitRole::BenchmarkHoldout
        )
    {
        return Ok(make_assessment(
            PrivacyPurposeAssessmentState::IncompatibleWithReviewedPrivacyPurposeProfile,
            &evidence,
            request,
            &transforms,
        ));
    }

    let observed: Vec<_> = purpose_evidence
        .iter()
        .filter_map(|item| item.dispositions.iter().next().copied())
        .collect();

    let state = if observed.contains(&PurposeDisposition::Incompatible) {
        PrivacyPurposeAssessmentState::IncompatibleWithReviewedPrivacyPurposeProfile
    } else if observed.contains(&PurposeDisposition::Unknown) {
        PrivacyPurposeAssessmentState::HumanReviewRequired
    } else if observed.contains(&PurposeDisposition::OutOfScope) {
        PrivacyPurposeAssessmentState::PurposeOutOfReviewedScope
    } else {
        PrivacyPurposeAssessmentState::CompatibleWithReviewedPrivacyPurposeProfile
    };

    Ok(make_assessment(state, &evidence, request, &transforms))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(subjects: &[&str], purpose: PrivacyPurpose) -> IntendedPrivacyPurposeUseV1 {
        IntendedPrivacyPurposeUseV1 {
            subjects: subjects.iter().map(|value| (*value).to_owned()).collect(),
            dataset_split_role: DatasetSplitRole::Training,
            source_modalities: [Modality::Gaze, Modality::HandPose].into_iter().collect(),
            requested_purpose: purpose,
            requested_modalities: [Modality::Gaze, Modality::HandPose].into_iter().collect(),
            excluded_modalities: BTreeSet::new(),
            reviewed_transformations: Vec::new(),
            external_prerequisites: BTreeSet::new(),
        }
    }

    fn evidence(
        subject: &str,
        evidence_ref: &str,
        disposition: PurposeDisposition,
    ) -> ReviewedPrivacyPurposeEvidenceV1 {
        ReviewedPrivacyPurposeEvidenceV1 {
            subject: subject.to_owned(),
            evidence_ref: evidence_ref.to_owned(),
            currentness: PrivacyPurposeCurrentness::Current,
            participant_evidence: ParticipantEvidenceState::Supported,
            bystander_evidence: BystanderEvidenceState::NotApplicable,
            purpose_dispositions: [(PrivacyPurpose::GazeAffordanceResearch, disposition)]
                .into_iter()
                .collect(),
            obligations: BTreeSet::new(),
            nonclaims: BTreeSet::new(),
            transformation_refs: BTreeSet::new(),
        }
    }

    #[test]
    fn identical_duplicate_evidence_normalizes() {
        let row = evidence("a", "review:a", PurposeDisposition::Compatible);
        let result = assess_privacy_purpose_compatibility(
            &[row.clone(), row],
            &request(&["a"], PrivacyPurpose::GazeAffordanceResearch),
        )
        .expect("identical duplicate should normalize");
        assert_eq!(
            result.state(),
            PrivacyPurposeAssessmentState::CompatibleWithReviewedPrivacyPurposeProfile
        );
        assert_eq!(result.purpose_evidence()[0].evidence_refs().len(), 1);
    }

    #[test]
    fn conflicting_reuse_of_evidence_ref_is_malformed_input() {
        let compatible = evidence("a", "review:a", PurposeDisposition::Compatible);
        let incompatible = evidence("a", "review:a", PurposeDisposition::Incompatible);
        assert_eq!(
            assess_privacy_purpose_compatibility(
                &[compatible, incompatible],
                &request(&["a"], PrivacyPurpose::GazeAffordanceResearch),
            ),
            Err(PrivacyEvidenceError::ConflictingReuseOfEvidenceRef(
                "review:a".to_owned()
            ))
        );
    }

    #[test]
    fn same_subject_distinct_reviews_can_form_evidence_conflict() {
        let compatible = evidence("a", "review:a", PurposeDisposition::Compatible);
        let incompatible = evidence("a", "review:b", PurposeDisposition::Incompatible);
        let result = assess_privacy_purpose_compatibility(
            &[compatible, incompatible],
            &request(&["a"], PrivacyPurpose::GazeAffordanceResearch),
        )
        .expect("well-formed contradiction should assess");
        assert_eq!(
            result.state(),
            PrivacyPurposeAssessmentState::EvidenceConflict
        );
    }

    #[test]
    fn different_subject_restriction_is_composition_not_conflict() {
        let compatible = evidence("a", "review:a", PurposeDisposition::Compatible);
        let incompatible = evidence("b", "review:b", PurposeDisposition::Incompatible);
        let result = assess_privacy_purpose_compatibility(
            &[compatible, incompatible],
            &request(&["a", "b"], PrivacyPurpose::GazeAffordanceResearch),
        )
        .expect("well-formed multi-source evidence should assess");
        assert_eq!(
            result.state(),
            PrivacyPurposeAssessmentState::IncompatibleWithReviewedPrivacyPurposeProfile
        );
    }

    #[test]
    fn reviewed_bystander_transform_must_cover_requested_representation() {
        let mut row = evidence("a", "review:a", PurposeDisposition::Compatible);
        row.bystander_evidence = BystanderEvidenceState::HandledByReviewedTransformation;
        row.transformation_refs.insert("transform:a".to_owned());
        row.obligations
            .insert(PrivacyObligation::RetainTransformationProvenance);

        let mut req = request(&["a"], PrivacyPurpose::GazeAffordanceResearch);
        req.reviewed_transformations.push(ReviewedTransformationV1 {
            transform_ref: "transform:a".to_owned(),
            review_state: TransformReviewState::ReviewedForDeclaredDerivedOutput,
            output_modalities: [Modality::Gaze].into_iter().collect(),
        });

        assert_eq!(
            assess_privacy_purpose_compatibility(&[row], &req),
            Err(
                PrivacyEvidenceError::RequestedModalityOutsideReviewedTransformation(
                    Modality::HandPose
                )
            )
        );
    }
}
