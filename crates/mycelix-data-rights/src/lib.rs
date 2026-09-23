#![forbid(unsafe_code)]
//! Pure, deterministic compatibility semantics for reviewed dataset-terms evidence.
//!
//! This crate deliberately has no provider SDK, network, storage, Holochain, model,
//! training-runtime, or robot-control dependency. It answers one narrow question:
//! whether exact reviewed terms evidence is compatible with an exact intended-use
//! profile under the frozen EMB-DATA-RIGHTS-001 v0.1 semantics.
//!
//! A positive assessment is constructor-controlled: safe downstream Rust cannot
//! mint one with a struct literal.
//!
//! ```compile_fail
//! use mycelix_data_rights::CompatibleTermsAssessmentV1;
//! let _forged = CompatibleTermsAssessmentV1 {};
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum UseDimension {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SplitRole {
    Training,
    Validation,
    EvaluationOnly,
    BenchmarkHoldout,
    Unspecified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EvidenceDisposition {
    Compatible,
    Incompatible,
    Unknown,
    OutOfScope,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TermsProfileCurrentness {
    Current,
    Unknown,
    Expired,
    Superseded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Obligation {
    Attribution,
    NoticeRetention,
    NonCommercialUseOnly,
    NoRawRedistribution,
    SourceLineageRetention,
    ReviewOnTermsChange,
    DeletionOnRevocation,
    NoDerivedDatasetRedistribution,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ExternalPrerequisite {
    PrivacyConsentAssessment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssessmentState {
    CompatibleWithReviewedTermsProfile,
    IncompatibleWithReviewedTermsProfile,
    HumanReviewRequired,
    TermsEvidenceConflict,
    TermsProfileExpiredOrSuperseded,
    IntendedUseOutOfProfile,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SubjectId(String);

impl SubjectId {
    pub fn new(value: impl Into<String>) -> Result<Self, InputError> {
        let value = value.into();
        if value.trim().is_empty() {
            return Err(InputError::EmptySubject);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EvidenceRef(String);

impl EvidenceRef {
    pub fn new(value: impl Into<String>) -> Result<Self, InputError> {
        let value = value.into();
        if value.trim().is_empty() {
            return Err(InputError::EmptyEvidenceRef);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InputError {
    EmptySubject,
    EmptyEvidenceRef,
    NoSubjects,
    DuplicateSubject(String),
    NoIntendedUseDimensions,
    NoEvidenceRows,
    NoEvidenceDimensions(String),
    DuplicateEvidenceDimension {
        subject: String,
        dimension: UseDimension,
    },
}

impl fmt::Display for InputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptySubject => write!(f, "subject identity must not be empty"),
            Self::EmptyEvidenceRef => write!(f, "evidence reference must not be empty"),
            Self::NoSubjects => write!(f, "intended-use profile must contain at least one subject"),
            Self::DuplicateSubject(subject) => {
                write!(f, "duplicate intended-use subject: {subject}")
            }
            Self::NoIntendedUseDimensions => {
                write!(f, "intended-use profile must request at least one dimension")
            }
            Self::NoEvidenceRows => write!(f, "reviewed terms evidence must contain at least one row"),
            Self::NoEvidenceDimensions(subject) => {
                write!(f, "evidence row for {subject} has no reviewed dimensions")
            }
            Self::DuplicateEvidenceDimension { subject, dimension } => {
                write!(f, "duplicate evidence dimension {dimension:?} for {subject}")
            }
        }
    }
}

impl Error for InputError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IntendedUseProfileV1 {
    subjects: Vec<SubjectId>,
    split_role: SplitRole,
    dimensions: BTreeSet<UseDimension>,
    external_prerequisites: BTreeSet<ExternalPrerequisite>,
}

impl IntendedUseProfileV1 {
    pub fn new(
        subjects: Vec<SubjectId>,
        split_role: SplitRole,
        dimensions: impl IntoIterator<Item = UseDimension>,
        external_prerequisites: impl IntoIterator<Item = ExternalPrerequisite>,
    ) -> Result<Self, InputError> {
        if subjects.is_empty() {
            return Err(InputError::NoSubjects);
        }
        let mut seen = BTreeSet::new();
        for subject in &subjects {
            if !seen.insert(subject.clone()) {
                return Err(InputError::DuplicateSubject(subject.as_str().to_owned()));
            }
        }
        let dimensions: BTreeSet<_> = dimensions.into_iter().collect();
        if dimensions.is_empty() {
            return Err(InputError::NoIntendedUseDimensions);
        }
        Ok(Self {
            subjects,
            split_role,
            dimensions,
            external_prerequisites: external_prerequisites.into_iter().collect(),
        })
    }

    pub fn subjects(&self) -> &[SubjectId] {
        &self.subjects
    }

    pub fn split_role(&self) -> SplitRole {
        self.split_role
    }

    pub fn dimensions(&self) -> &BTreeSet<UseDimension> {
        &self.dimensions
    }

    pub fn external_prerequisites(&self) -> &BTreeSet<ExternalPrerequisite> {
        &self.external_prerequisites
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceTermsEvidenceV1 {
    subject: SubjectId,
    evidence_ref: Option<EvidenceRef>,
    dimensions: BTreeMap<UseDimension, EvidenceDisposition>,
    obligations: BTreeSet<Obligation>,
}

impl SourceTermsEvidenceV1 {
    pub fn new(
        subject: SubjectId,
        evidence_ref: Option<EvidenceRef>,
        dimensions: impl IntoIterator<Item = (UseDimension, EvidenceDisposition)>,
        obligations: impl IntoIterator<Item = Obligation>,
    ) -> Result<Self, InputError> {
        let mut dimension_map = BTreeMap::new();
        for (dimension, disposition) in dimensions {
            if dimension_map.insert(dimension, disposition).is_some() {
                return Err(InputError::DuplicateEvidenceDimension {
                    subject: subject.as_str().to_owned(),
                    dimension,
                });
            }
        }
        if dimension_map.is_empty() {
            return Err(InputError::NoEvidenceDimensions(subject.as_str().to_owned()));
        }
        Ok(Self {
            subject,
            evidence_ref,
            dimensions: dimension_map,
            obligations: obligations.into_iter().collect(),
        })
    }

    pub fn subject(&self) -> &SubjectId {
        &self.subject
    }

    pub fn evidence_ref(&self) -> Option<&EvidenceRef> {
        self.evidence_ref.as_ref()
    }

    pub fn dimensions(&self) -> &BTreeMap<UseDimension, EvidenceDisposition> {
        &self.dimensions
    }

    pub fn obligations(&self) -> &BTreeSet<Obligation> {
        &self.obligations
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReviewedTermsEvidenceV1 {
    currentness: TermsProfileCurrentness,
    rows: Vec<SourceTermsEvidenceV1>,
}

impl ReviewedTermsEvidenceV1 {
    pub fn new(
        currentness: TermsProfileCurrentness,
        rows: Vec<SourceTermsEvidenceV1>,
    ) -> Result<Self, InputError> {
        if rows.is_empty() {
            return Err(InputError::NoEvidenceRows);
        }
        Ok(Self { currentness, rows })
    }

    pub fn currentness(&self) -> TermsProfileCurrentness {
        self.currentness
    }

    pub fn rows(&self) -> &[SourceTermsEvidenceV1] {
        &self.rows
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssessmentError {
    MalformedConflictEvidence {
        subject: SubjectId,
        dimension: UseDimension,
        reason: ConflictEvidenceError,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConflictEvidenceError {
    MissingEvidenceRef,
    DuplicateEvidenceRef,
}

impl fmt::Display for AssessmentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedConflictEvidence {
                subject,
                dimension,
                reason,
            } => write!(
                f,
                "malformed conflicting evidence for {} / {dimension:?}: {reason:?}",
                subject.as_str()
            ),
        }
    }
}

impl Error for AssessmentError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssessmentContextV1 {
    subjects: Vec<SubjectId>,
    dimensions: BTreeSet<UseDimension>,
    obligations: BTreeSet<Obligation>,
    external_prerequisites: BTreeSet<ExternalPrerequisite>,
}

impl AssessmentContextV1 {
    fn from_input(
        intended: &IntendedUseProfileV1,
        obligations: BTreeSet<Obligation>,
    ) -> Self {
        Self {
            subjects: intended.subjects.clone(),
            dimensions: intended.dimensions.clone(),
            obligations,
            external_prerequisites: intended.external_prerequisites.clone(),
        }
    }

    pub fn subjects(&self) -> &[SubjectId] {
        &self.subjects
    }

    pub fn dimensions(&self) -> &BTreeSet<UseDimension> {
        &self.dimensions
    }

    pub fn obligations(&self) -> &BTreeSet<Obligation> {
        &self.obligations
    }

    pub fn external_prerequisites(&self) -> &BTreeSet<ExternalPrerequisite> {
        &self.external_prerequisites
    }
}

/// A positive terms-compatibility result minted only by [`assess_terms_compatibility`].
///
/// Fields are private and there is no public constructor or `Default` implementation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompatibleTermsAssessmentV1 {
    context: AssessmentContextV1,
}

impl CompatibleTermsAssessmentV1 {
    pub fn context(&self) -> &AssessmentContextV1 {
        &self.context
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConflictCoordinateV1 {
    subject: SubjectId,
    dimension: UseDimension,
    evidence_refs: BTreeSet<EvidenceRef>,
}

impl ConflictCoordinateV1 {
    pub fn subject(&self) -> &SubjectId {
        &self.subject
    }

    pub fn dimension(&self) -> UseDimension {
        self.dimension
    }

    pub fn evidence_refs(&self) -> &BTreeSet<EvidenceRef> {
        &self.evidence_refs
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TermsEvidenceConflictV1 {
    context: AssessmentContextV1,
    conflicts: Vec<ConflictCoordinateV1>,
}

impl TermsEvidenceConflictV1 {
    pub fn context(&self) -> &AssessmentContextV1 {
        &self.context
    }

    pub fn conflicts(&self) -> &[ConflictCoordinateV1] {
        &self.conflicts
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TermsAssessmentV1 {
    Compatible(CompatibleTermsAssessmentV1),
    Incompatible(AssessmentContextV1),
    HumanReviewRequired(AssessmentContextV1),
    TermsEvidenceConflict(TermsEvidenceConflictV1),
    TermsProfileExpiredOrSuperseded(AssessmentContextV1),
    IntendedUseOutOfProfile(AssessmentContextV1),
}

impl TermsAssessmentV1 {
    pub fn state(&self) -> AssessmentState {
        match self {
            Self::Compatible(_) => AssessmentState::CompatibleWithReviewedTermsProfile,
            Self::Incompatible(_) => AssessmentState::IncompatibleWithReviewedTermsProfile,
            Self::HumanReviewRequired(_) => AssessmentState::HumanReviewRequired,
            Self::TermsEvidenceConflict(_) => AssessmentState::TermsEvidenceConflict,
            Self::TermsProfileExpiredOrSuperseded(_) => {
                AssessmentState::TermsProfileExpiredOrSuperseded
            }
            Self::IntendedUseOutOfProfile(_) => AssessmentState::IntendedUseOutOfProfile,
        }
    }

    pub fn context(&self) -> &AssessmentContextV1 {
        match self {
            Self::Compatible(value) => value.context(),
            Self::Incompatible(context)
            | Self::HumanReviewRequired(context)
            | Self::TermsProfileExpiredOrSuperseded(context)
            | Self::IntendedUseOutOfProfile(context) => context,
            Self::TermsEvidenceConflict(value) => value.context(),
        }
    }
}

fn union_obligations(evidence: &ReviewedTermsEvidenceV1) -> BTreeSet<Obligation> {
    evidence
        .rows
        .iter()
        .flat_map(|row| row.obligations.iter().copied())
        .collect()
}

fn context(
    intended: &IntendedUseProfileV1,
    evidence: &ReviewedTermsEvidenceV1,
) -> AssessmentContextV1 {
    AssessmentContextV1::from_input(intended, union_obligations(evidence))
}

/// Evaluate exact reviewed terms evidence against an exact intended-use profile.
///
/// This function does not perform legal interpretation, network access, provider
/// authentication, privacy/consent review, training, publication, or physical effects.
pub fn assess_terms_compatibility(
    intended: &IntendedUseProfileV1,
    evidence: &ReviewedTermsEvidenceV1,
) -> Result<TermsAssessmentV1, AssessmentError> {
    let base_context = context(intended, evidence);

    match evidence.currentness {
        TermsProfileCurrentness::Unknown => {
            return Ok(TermsAssessmentV1::HumanReviewRequired(base_context));
        }
        TermsProfileCurrentness::Expired | TermsProfileCurrentness::Superseded => {
            return Ok(TermsAssessmentV1::TermsProfileExpiredOrSuperseded(
                base_context,
            ));
        }
        TermsProfileCurrentness::Current => {}
    }

    if intended.dimensions.contains(&UseDimension::Training)
        && matches!(
            intended.split_role,
            SplitRole::EvaluationOnly | SplitRole::BenchmarkHoldout
        )
    {
        return Ok(TermsAssessmentV1::Incompatible(base_context));
    }

    let intended_subjects: BTreeSet<_> = intended.subjects.iter().cloned().collect();
    let evidence_subjects: BTreeSet<_> = evidence
        .rows
        .iter()
        .map(|row| row.subject.clone())
        .collect();
    if intended_subjects != evidence_subjects {
        return Ok(TermsAssessmentV1::IntendedUseOutOfProfile(base_context));
    }

    let mut grouped: BTreeMap<
        (SubjectId, UseDimension),
        Vec<(EvidenceDisposition, Option<EvidenceRef>)>,
    > = BTreeMap::new();
    for row in &evidence.rows {
        for dimension in &intended.dimensions {
            let disposition = row
                .dimensions
                .get(dimension)
                .copied()
                .unwrap_or(EvidenceDisposition::OutOfScope);
            grouped
                .entry((row.subject.clone(), *dimension))
                .or_default()
                .push((disposition, row.evidence_ref.clone()));
        }
    }

    let mut conflicts = Vec::new();
    for ((subject, dimension), records) in &grouped {
        let dispositions: BTreeSet<_> = records.iter().map(|(value, _)| *value).collect();
        if dispositions.len() <= 1 {
            continue;
        }
        let mut refs = BTreeSet::new();
        for (_, evidence_ref) in records {
            let Some(evidence_ref) = evidence_ref else {
                return Err(AssessmentError::MalformedConflictEvidence {
                    subject: subject.clone(),
                    dimension: *dimension,
                    reason: ConflictEvidenceError::MissingEvidenceRef,
                });
            };
            if !refs.insert(evidence_ref.clone()) {
                return Err(AssessmentError::MalformedConflictEvidence {
                    subject: subject.clone(),
                    dimension: *dimension,
                    reason: ConflictEvidenceError::DuplicateEvidenceRef,
                });
            }
        }
        conflicts.push(ConflictCoordinateV1 {
            subject: subject.clone(),
            dimension: *dimension,
            evidence_refs: refs,
        });
    }

    if !conflicts.is_empty() {
        return Ok(TermsAssessmentV1::TermsEvidenceConflict(
            TermsEvidenceConflictV1 {
                context: base_context,
                conflicts,
            },
        ));
    }

    let mut observed = Vec::new();
    for subject in &intended.subjects {
        for dimension in &intended.dimensions {
            let records = grouped
                .get(&(subject.clone(), *dimension))
                .expect("subject set and requested dimensions were materialized above");
            let dispositions: BTreeSet<_> = records.iter().map(|(value, _)| *value).collect();
            debug_assert_eq!(dispositions.len(), 1);
            observed.push(*dispositions.iter().next().expect("non-empty evidence group"));
        }
    }

    if observed.contains(&EvidenceDisposition::Incompatible) {
        return Ok(TermsAssessmentV1::Incompatible(base_context));
    }
    if observed.contains(&EvidenceDisposition::Unknown) {
        return Ok(TermsAssessmentV1::HumanReviewRequired(base_context));
    }
    if observed.contains(&EvidenceDisposition::OutOfScope) {
        return Ok(TermsAssessmentV1::IntendedUseOutOfProfile(base_context));
    }

    debug_assert!(
        !observed.is_empty()
            && observed
                .iter()
                .all(|value| *value == EvidenceDisposition::Compatible)
    );
    Ok(TermsAssessmentV1::Compatible(
        CompatibleTermsAssessmentV1 {
            context: base_context,
        },
    ))
}
