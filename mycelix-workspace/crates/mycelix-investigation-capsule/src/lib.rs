// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Record-only investigation capsule semantics for Mycelix EPI-012.
//!
//! This crate preserves investigation history and cross-references only. It has no network,
//! persistence, Holochain, reasoning, model, search, connector, OPSEC-permit, Xenia, or action API.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

pub const INVESTIGATION_CAPSULE_PROFILE_V1: &str = "mycelix:epi:investigation-capsule:v1";
pub const MAX_REF_BYTES: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvestigationCapsuleAuthorityV1 {
    RecordOnly,
}

#[derive(Clone, PartialEq, Eq)]
pub enum CapsuleError {
    InvalidReference { role: &'static str, reason: &'static str },
    DuplicateIdentity { role: &'static str, value: String },
    MissingPriorFrontier(String),
    DanglingReference { role: &'static str, value: String },
    BrokenAssumptionSupersession(String),
    IncoherentNegativeSearch(String),
    NegativeSearchHasResults(String),
    PlannerExecutionAuthority(String),
    PlannerSelfDomination(String),
    InvalidProtectedOmission(String),
}

impl fmt::Display for CapsuleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidReference { role, reason } => write!(f, "invalid {role}: {reason}"),
            Self::DuplicateIdentity { role, .. } => write!(f, "duplicate {role}: <redacted>"),
            Self::MissingPriorFrontier(_) => {
                write!(f, "frontier prior reference is missing: <redacted>")
            }
            Self::DanglingReference { role, .. } => {
                write!(f, "dangling {role} reference: <redacted>")
            }
            Self::BrokenAssumptionSupersession(_) => {
                write!(f, "assumption supersession is invalid: <redacted>")
            }
            Self::IncoherentNegativeSearch(_) => {
                write!(f, "negative-search coverage/interpretation is incoherent: <redacted>")
            }
            Self::NegativeSearchHasResults(_) => {
                write!(f, "negative-search record contains positive results: <redacted>")
            }
            Self::PlannerExecutionAuthority(_) => {
                write!(f, "planner trace exceeds candidate-only authority: <redacted>")
            }
            Self::PlannerSelfDomination(_) => {
                write!(f, "planner domination witness is self-referential: <redacted>")
            }
            Self::InvalidProtectedOmission(_) => {
                write!(f, "protected omission embeds raw content: <redacted>")
            }
        }
    }
}

impl fmt::Debug for CapsuleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl Error for CapsuleError {}

fn validate_ref(role: &'static str, value: &str) -> Result<(), CapsuleError> {
    if value.is_empty() {
        return Err(CapsuleError::InvalidReference { role, reason: "empty" });
    }
    if value.len() > MAX_REF_BYTES {
        return Err(CapsuleError::InvalidReference { role, reason: "too long" });
    }
    if !value.bytes().all(|b| (0x21..=0x7e).contains(&b)) {
        return Err(CapsuleError::InvalidReference {
            role,
            reason: "graphic ASCII only",
        });
    }
    Ok(())
}

macro_rules! role_ref {
    ($name:ident) => {
        #[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, CapsuleError> {
                let value = value.into();
                validate_ref(stringify!($name), &value)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}(<redacted>)", stringify!($name))
            }
        }
    };
}

role_ref!(CapsuleId);
role_ref!(FrontierRef);
role_ref!(ArtifactRef);
role_ref!(AssumptionRef);
role_ref!(AssumptionAssessmentRef);
role_ref!(DependencyGroupRef);
role_ref!(SearchRef);
role_ref!(CandidateRef);
role_ref!(PlannerTraceRef);
role_ref!(ProposalRef);
role_ref!(OmissionRef);
role_ref!(PresentationRef);
role_ref!(CommitmentRef);
role_ref!(PropositionRef);
role_ref!(ProfileRef);
role_ref!(QuestionRef);
role_ref!(PurposeRef);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArtifactRecordV1 {
    pub id: ArtifactRef,
    pub content_commitment: CommitmentRef,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssumptionStatusV1 {
    DeclaredWorkingAssumption,
    InvalidatedWithinProfile,
    Retired,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AssumptionAssessmentV1 {
    pub id: AssumptionAssessmentRef,
    pub assumption_id: AssumptionRef,
    pub frontier_ref: FrontierRef,
    pub proposition_ref: PropositionRef,
    pub status: AssumptionStatusV1,
    pub supersedes: Option<AssumptionAssessmentRef>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DependencyGroupV1 {
    pub id: DependencyGroupRef,
    pub frontier_ref: FrontierRef,
    pub artifact_refs: Vec<ArtifactRef>,
    pub scope_profile_ref: ProfileRef,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SearchCoverageV1 {
    UnknownCoverage,
    ExhaustiveWithinDeclaredFiniteCorpus,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NegativeSearchInterpretationV1 {
    UnresolvedDueToUnknownCoverage,
    AbsentWithinExactFiniteCorpus,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NegativeSearchRecordV1 {
    pub id: SearchRef,
    pub frontier_ref: FrontierRef,
    pub result_count: usize,
    pub coverage: SearchCoverageV1,
    pub finite_corpus_commitment: Option<CommitmentRef>,
    pub interpretation: NegativeSearchInterpretationV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateStatusV1 {
    CandidateUnadmitted,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExternalCandidateRecordV1 {
    pub id: CandidateRef,
    pub frontier_ref: FrontierRef,
    pub external_profile_ref: ProfileRef,
    pub status: CandidateStatusV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BlockedPlannerProposalV1 {
    pub proposal_ref: ProposalRef,
    pub reason_ref: ProfileRef,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DominatedPlannerProposalV1 {
    pub proposal_ref: ProposalRef,
    pub dominated_by: ProposalRef,
    pub witness_profile_ref: ProfileRef,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlannerTraceV1 {
    pub id: PlannerTraceRef,
    pub frontier_ref: FrontierRef,
    pub preferred_hypothesis_ref: PropositionRef,
    pub profile_ref: ProfileRef,
    pub disconfirmation_candidate_refs: Vec<ProposalRef>,
    pub eligible_pareto_front_refs: Vec<ProposalRef>,
    pub blocked_but_analytically_useful: Vec<BlockedPlannerProposalV1>,
    pub dominated_candidates: Vec<DominatedPlannerProposalV1>,
    pub execution_authority: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MethodologyExecutionStateV1 {
    NotExecuted,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MethodologyRecordV1 {
    pub frontier_ref: FrontierRef,
    pub selected_profile_refs: Vec<ProfileRef>,
    pub execution_state: MethodologyExecutionStateV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProtectedOmissionV1 {
    pub id: OmissionRef,
    pub subject_ref: PropositionRef,
    pub commitment_ref: CommitmentRef,
    pub reason_ref: ProfileRef,
    pub raw_content_embedded: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PresentationAuthorityV1 {
    RenderingOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PresentationProjectionV1 {
    pub id: PresentationRef,
    pub frontier_ref: FrontierRef,
    pub authority: PresentationAuthorityV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FrontierRecordV1 {
    pub id: FrontierRef,
    pub prior: Option<FrontierRef>,
    pub artifact_refs: Vec<ArtifactRef>,
    pub assumption_assessment_refs: Vec<AssumptionAssessmentRef>,
    pub dependency_group_refs: Vec<DependencyGroupRef>,
    pub search_refs: Vec<SearchRef>,
    pub candidate_refs: Vec<CandidateRef>,
    pub planner_trace_refs: Vec<PlannerTraceRef>,
}

#[derive(Clone, PartialEq, Eq)]
pub struct InvestigationCapsuleV1 {
    pub capsule_id: CapsuleId,
    pub question_ref: QuestionRef,
    pub purpose_ref: PurposeRef,
    pub profile_ref: ProfileRef,
    pub artifacts: Vec<ArtifactRecordV1>,
    pub frontiers: Vec<FrontierRecordV1>,
    pub assumptions: Vec<AssumptionAssessmentV1>,
    pub dependency_groups: Vec<DependencyGroupV1>,
    pub searches: Vec<NegativeSearchRecordV1>,
    pub external_candidates: Vec<ExternalCandidateRecordV1>,
    pub planner_traces: Vec<PlannerTraceV1>,
    pub methodology_records: Vec<MethodologyRecordV1>,
    pub protected_omissions: Vec<ProtectedOmissionV1>,
    pub presentation_projection: PresentationProjectionV1,
}

impl InvestigationCapsuleV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        capsule_id: CapsuleId,
        question_ref: QuestionRef,
        purpose_ref: PurposeRef,
        profile_ref: ProfileRef,
        artifacts: Vec<ArtifactRecordV1>,
        frontiers: Vec<FrontierRecordV1>,
        assumptions: Vec<AssumptionAssessmentV1>,
        dependency_groups: Vec<DependencyGroupV1>,
        searches: Vec<NegativeSearchRecordV1>,
        external_candidates: Vec<ExternalCandidateRecordV1>,
        planner_traces: Vec<PlannerTraceV1>,
        methodology_records: Vec<MethodologyRecordV1>,
        protected_omissions: Vec<ProtectedOmissionV1>,
        presentation_projection: PresentationProjectionV1,
    ) -> Result<Self, CapsuleError> {
        unique("artifact", artifacts.iter().map(|x| x.id.as_str()))?;
        unique("frontier", frontiers.iter().map(|x| x.id.as_str()))?;
        unique(
            "assumption-assessment",
            assumptions.iter().map(|x| x.id.as_str()),
        )?;
        unique(
            "dependency-group",
            dependency_groups.iter().map(|x| x.id.as_str()),
        )?;
        unique("search", searches.iter().map(|x| x.id.as_str()))?;
        unique(
            "candidate",
            external_candidates.iter().map(|x| x.id.as_str()),
        )?;
        unique(
            "planner-trace",
            planner_traces.iter().map(|x| x.id.as_str()),
        )?;
        unique(
            "omission",
            protected_omissions.iter().map(|x| x.id.as_str()),
        )?;

        validate_frontiers(&frontiers)?;
        validate_assumptions(&assumptions)?;
        validate_searches(&searches)?;
        validate_planners(&planner_traces)?;
        validate_methodologies(&methodology_records)?;
        validate_omissions(&protected_omissions)?;
        validate_cross_references(
            &artifacts,
            &frontiers,
            &assumptions,
            &dependency_groups,
            &searches,
            &external_candidates,
            &planner_traces,
            &methodology_records,
            &presentation_projection,
        )?;

        Ok(Self {
            capsule_id,
            question_ref,
            purpose_ref,
            profile_ref,
            artifacts,
            frontiers,
            assumptions,
            dependency_groups,
            searches,
            external_candidates,
            planner_traces,
            methodology_records,
            protected_omissions,
            presentation_projection,
        })
    }

    pub fn authority_scope(&self) -> InvestigationCapsuleAuthorityV1 {
        InvestigationCapsuleAuthorityV1::RecordOnly
    }

    pub fn reasoning_authority(&self) -> bool {
        false
    }

    pub fn collection_authority(&self) -> bool {
        false
    }

    pub fn action_authority(&self) -> bool {
        false
    }
}

impl fmt::Debug for InvestigationCapsuleV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InvestigationCapsuleV1")
            .field("artifacts", &self.artifacts.len())
            .field("frontiers", &self.frontiers.len())
            .field("searches", &self.searches.len())
            .field("methodology_records", &self.methodology_records.len())
            .field("authority", &self.authority_scope())
            .finish()
    }
}

fn unique<'a>(
    role: &'static str,
    values: impl Iterator<Item = &'a str>,
) -> Result<(), CapsuleError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value) {
            return Err(CapsuleError::DuplicateIdentity {
                role,
                value: value.to_string(),
            });
        }
    }
    Ok(())
}

fn validate_frontiers(frontiers: &[FrontierRecordV1]) -> Result<(), CapsuleError> {
    let mut known = BTreeSet::new();
    for frontier in frontiers {
        if let Some(prior) = &frontier.prior {
            if !known.contains(prior.as_str()) {
                return Err(CapsuleError::MissingPriorFrontier(
                    prior.as_str().to_string(),
                ));
            }
        }
        known.insert(frontier.id.as_str());
    }
    Ok(())
}

fn validate_assumptions(assumptions: &[AssumptionAssessmentV1]) -> Result<(), CapsuleError> {
    let mut latest: BTreeMap<&str, &str> = BTreeMap::new();
    for assessment in assumptions {
        match latest.get(assessment.assumption_id.as_str()) {
            None if assessment.supersedes.is_some() => {
                return Err(CapsuleError::BrokenAssumptionSupersession(
                    assessment.id.as_str().to_string(),
                ));
            }
            Some(expected)
                if assessment.supersedes.as_ref().map(|x| x.as_str()) != Some(*expected) =>
            {
                return Err(CapsuleError::BrokenAssumptionSupersession(
                    assessment.id.as_str().to_string(),
                ));
            }
            _ => {}
        }
        latest.insert(assessment.assumption_id.as_str(), assessment.id.as_str());
    }
    Ok(())
}

fn validate_searches(searches: &[NegativeSearchRecordV1]) -> Result<(), CapsuleError> {
    for search in searches {
        if search.result_count != 0 {
            return Err(CapsuleError::NegativeSearchHasResults(
                search.id.as_str().to_string(),
            ));
        }

        let coherent = match (search.coverage, search.interpretation) {
            (
                SearchCoverageV1::UnknownCoverage,
                NegativeSearchInterpretationV1::UnresolvedDueToUnknownCoverage,
            ) => search.finite_corpus_commitment.is_none(),
            (
                SearchCoverageV1::ExhaustiveWithinDeclaredFiniteCorpus,
                NegativeSearchInterpretationV1::AbsentWithinExactFiniteCorpus,
            ) => search.finite_corpus_commitment.is_some(),
            _ => false,
        };

        if !coherent {
            return Err(CapsuleError::IncoherentNegativeSearch(
                search.id.as_str().to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_planners(planners: &[PlannerTraceV1]) -> Result<(), CapsuleError> {
    for planner in planners {
        if planner.execution_authority {
            return Err(CapsuleError::PlannerExecutionAuthority(
                planner.id.as_str().to_string(),
            ));
        }
        unique(
            "planner-disconfirmation-proposal",
            planner
                .disconfirmation_candidate_refs
                .iter()
                .map(|x| x.as_str()),
        )?;
        unique(
            "planner-pareto-proposal",
            planner
                .eligible_pareto_front_refs
                .iter()
                .map(|x| x.as_str()),
        )?;
        unique(
            "planner-blocked-proposal",
            planner
                .blocked_but_analytically_useful
                .iter()
                .map(|x| x.proposal_ref.as_str()),
        )?;
        unique(
            "planner-dominated-proposal",
            planner
                .dominated_candidates
                .iter()
                .map(|x| x.proposal_ref.as_str()),
        )?;
        for dominated in &planner.dominated_candidates {
            if dominated.proposal_ref == dominated.dominated_by {
                return Err(CapsuleError::PlannerSelfDomination(
                    dominated.proposal_ref.as_str().to_string(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_methodologies(records: &[MethodologyRecordV1]) -> Result<(), CapsuleError> {
    for record in records {
        unique(
            "selected-methodology-profile",
            record.selected_profile_refs.iter().map(|x| x.as_str()),
        )?;
        match record.execution_state {
            MethodologyExecutionStateV1::NotExecuted => {}
        }
    }
    Ok(())
}

fn validate_omissions(omissions: &[ProtectedOmissionV1]) -> Result<(), CapsuleError> {
    for omission in omissions {
        if omission.raw_content_embedded {
            return Err(CapsuleError::InvalidProtectedOmission(
                omission.id.as_str().to_string(),
            ));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_cross_references(
    artifacts: &[ArtifactRecordV1],
    frontiers: &[FrontierRecordV1],
    assumptions: &[AssumptionAssessmentV1],
    dependency_groups: &[DependencyGroupV1],
    searches: &[NegativeSearchRecordV1],
    external_candidates: &[ExternalCandidateRecordV1],
    planner_traces: &[PlannerTraceV1],
    methodology_records: &[MethodologyRecordV1],
    presentation: &PresentationProjectionV1,
) -> Result<(), CapsuleError> {
    let artifact_ids: BTreeSet<_> = artifacts.iter().map(|x| x.id.as_str()).collect();
    let frontier_ids: BTreeSet<_> = frontiers.iter().map(|x| x.id.as_str()).collect();
    let assumption_ids: BTreeSet<_> = assumptions.iter().map(|x| x.id.as_str()).collect();
    let dependency_ids: BTreeSet<_> = dependency_groups.iter().map(|x| x.id.as_str()).collect();
    let search_ids: BTreeSet<_> = searches.iter().map(|x| x.id.as_str()).collect();
    let candidate_ids: BTreeSet<_> = external_candidates.iter().map(|x| x.id.as_str()).collect();
    let planner_ids: BTreeSet<_> = planner_traces.iter().map(|x| x.id.as_str()).collect();

    for frontier in frontiers {
        ensure_all("frontier-artifact", frontier.artifact_refs.iter().map(|x| x.as_str()), &artifact_ids)?;
        ensure_all(
            "frontier-assumption",
            frontier.assumption_assessment_refs.iter().map(|x| x.as_str()),
            &assumption_ids,
        )?;
        ensure_all(
            "frontier-dependency",
            frontier.dependency_group_refs.iter().map(|x| x.as_str()),
            &dependency_ids,
        )?;
        ensure_all("frontier-search", frontier.search_refs.iter().map(|x| x.as_str()), &search_ids)?;
        ensure_all(
            "frontier-candidate",
            frontier.candidate_refs.iter().map(|x| x.as_str()),
            &candidate_ids,
        )?;
        ensure_all(
            "frontier-planner",
            frontier.planner_trace_refs.iter().map(|x| x.as_str()),
            &planner_ids,
        )?;
    }

    for assumption in assumptions {
        ensure_one("assumption-frontier", assumption.frontier_ref.as_str(), &frontier_ids)?;
    }
    for group in dependency_groups {
        ensure_one("dependency-frontier", group.frontier_ref.as_str(), &frontier_ids)?;
        ensure_all(
            "dependency-artifact",
            group.artifact_refs.iter().map(|x| x.as_str()),
            &artifact_ids,
        )?;
    }
    for search in searches {
        ensure_one("search-frontier", search.frontier_ref.as_str(), &frontier_ids)?;
    }
    for candidate in external_candidates {
        ensure_one("candidate-frontier", candidate.frontier_ref.as_str(), &frontier_ids)?;
    }
    for planner in planner_traces {
        ensure_one("planner-frontier", planner.frontier_ref.as_str(), &frontier_ids)?;
    }
    for methodology in methodology_records {
        ensure_one(
            "methodology-frontier",
            methodology.frontier_ref.as_str(),
            &frontier_ids,
        )?;
    }
    ensure_one(
        "presentation-frontier",
        presentation.frontier_ref.as_str(),
        &frontier_ids,
    )?;

    Ok(())
}

fn ensure_one(
    role: &'static str,
    value: &str,
    known: &BTreeSet<&str>,
) -> Result<(), CapsuleError> {
    if known.contains(value) {
        Ok(())
    } else {
        Err(CapsuleError::DanglingReference {
            role,
            value: value.to_string(),
        })
    }
}

fn ensure_all<'a>(
    role: &'static str,
    values: impl Iterator<Item = &'a str>,
    known: &BTreeSet<&str>,
) -> Result<(), CapsuleError> {
    for value in values {
        ensure_one(role, value, known)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r<T>(result: Result<T, CapsuleError>) -> T {
        result.unwrap()
    }

    fn reservoir_capsule() -> Result<InvestigationCapsuleV1, CapsuleError> {
        let ar1 = r(ArtifactRef::new("AR1"));
        let ar2 = r(ArtifactRef::new("AR2"));
        let ar3 = r(ArtifactRef::new("AR3"));
        let ar4 = r(ArtifactRef::new("AR4"));
        let f1 = r(FrontierRef::new("F1"));
        let f2 = r(FrontierRef::new("F2"));
        let a1f1 = r(AssumptionAssessmentRef::new("ASSUMP1:F1"));
        let a1f2 = r(AssumptionAssessmentRef::new("ASSUMP1:F2"));
        let dep = r(DependencyGroupRef::new("DEP:G1"));
        let s1 = r(SearchRef::new("S1"));
        let s2 = r(SearchRef::new("S2"));
        let c1 = r(CandidateRef::new("SYMCAND:F1"));
        let c2 = r(CandidateRef::new("SYMCAND:F2"));
        let planner = r(PlannerTraceRef::new("SYMPLAN:F2"));

        InvestigationCapsuleV1::new(
            r(CapsuleId::new("capsule:synthetic-reservoir:001")),
            r(QuestionRef::new("question:reservoir-level-change")),
            r(PurposeRef::new("purpose:synthetic-qualification")),
            r(ProfileRef::new(INVESTIGATION_CAPSULE_PROFILE_V1)),
            vec![
                ArtifactRecordV1 { id: ar1.clone(), content_commitment: r(CommitmentRef::new("fixture:ar1")) },
                ArtifactRecordV1 { id: ar2.clone(), content_commitment: r(CommitmentRef::new("fixture:ar2")) },
                ArtifactRecordV1 { id: ar3.clone(), content_commitment: r(CommitmentRef::new("fixture:ar3")) },
                ArtifactRecordV1 { id: ar4.clone(), content_commitment: r(CommitmentRef::new("fixture:ar4")) },
            ],
            vec![
                FrontierRecordV1 {
                    id: f1.clone(), prior: None,
                    artifact_refs: vec![ar1.clone(), ar2.clone(), ar3.clone()],
                    assumption_assessment_refs: vec![a1f1.clone()],
                    dependency_group_refs: vec![], search_refs: vec![],
                    candidate_refs: vec![c1.clone()], planner_trace_refs: vec![],
                },
                FrontierRecordV1 {
                    id: f2.clone(), prior: Some(f1.clone()),
                    artifact_refs: vec![ar1.clone(), ar2.clone(), ar3.clone(), ar4.clone()],
                    assumption_assessment_refs: vec![a1f2.clone()],
                    dependency_group_refs: vec![dep.clone()],
                    search_refs: vec![s1.clone(), s2.clone()],
                    candidate_refs: vec![c2.clone()], planner_trace_refs: vec![planner.clone()],
                },
            ],
            vec![
                AssumptionAssessmentV1 {
                    id: a1f1.clone(), assumption_id: r(AssumptionRef::new("ASSUMP1")),
                    frontier_ref: f1.clone(), proposition_ref: r(PropositionRef::new("assumption:distinct-publications-independent")),
                    status: AssumptionStatusV1::DeclaredWorkingAssumption, supersedes: None,
                },
                AssumptionAssessmentV1 {
                    id: a1f2, assumption_id: r(AssumptionRef::new("ASSUMP1")),
                    frontier_ref: f2.clone(), proposition_ref: r(PropositionRef::new("assumption:distinct-publications-independent")),
                    status: AssumptionStatusV1::InvalidatedWithinProfile, supersedes: Some(a1f1),
                },
            ],
            vec![DependencyGroupV1 {
                id: dep, frontier_ref: f2.clone(), artifact_refs: vec![ar1, ar2, ar3],
                scope_profile_ref: r(ProfileRef::new("dependency:synthetic-explicit-lineage:v1")),
            }],
            vec![
                NegativeSearchRecordV1 {
                    id: s1, frontier_ref: f2.clone(), result_count: 0,
                    coverage: SearchCoverageV1::UnknownCoverage, finite_corpus_commitment: None,
                    interpretation: NegativeSearchInterpretationV1::UnresolvedDueToUnknownCoverage,
                },
                NegativeSearchRecordV1 {
                    id: s2, frontier_ref: f2.clone(), result_count: 0,
                    coverage: SearchCoverageV1::ExhaustiveWithinDeclaredFiniteCorpus,
                    finite_corpus_commitment: Some(r(CommitmentRef::new("fixture:manual-override-corpus:v1"))),
                    interpretation: NegativeSearchInterpretationV1::AbsentWithinExactFiniteCorpus,
                },
            ],
            vec![
                ExternalCandidateRecordV1 {
                    id: c1, frontier_ref: f1,
                    external_profile_ref: r(ProfileRef::new("symthaea:bounded-investigation:v1")),
                    status: CandidateStatusV1::CandidateUnadmitted,
                },
                ExternalCandidateRecordV1 {
                    id: c2, frontier_ref: f2.clone(),
                    external_profile_ref: r(ProfileRef::new("symthaea:closed-world-investigation-loop:v1")),
                    status: CandidateStatusV1::CandidateUnadmitted,
                },
            ],
            vec![PlannerTraceV1 {
                id: planner, frontier_ref: f2.clone(),
                preferred_hypothesis_ref: r(PropositionRef::new("H1")),
                profile_ref: r(ProfileRef::new("symthaea:next-information:pareto-front:v1")),
                disconfirmation_candidate_refs: vec![r(ProposalRef::new("D1")), r(ProposalRef::new("D2")), r(ProposalRef::new("D4"))],
                eligible_pareto_front_refs: vec![r(ProposalRef::new("D1")), r(ProposalRef::new("D2")), r(ProposalRef::new("D3"))],
                blocked_but_analytically_useful: vec![BlockedPlannerProposalV1 {
                    proposal_ref: r(ProposalRef::new("D4")),
                    reason_ref: r(ProfileRef::new("privacy:blocked")),
                }],
                dominated_candidates: vec![DominatedPlannerProposalV1 {
                    proposal_ref: r(ProposalRef::new("D5")),
                    dominated_by: r(ProposalRef::new("D2")),
                    witness_profile_ref: r(ProfileRef::new("symthaea:next-information:pareto-front:v1")),
                }],
                execution_authority: false,
            }],
            vec![MethodologyRecordV1 {
                frontier_ref: f2.clone(),
                selected_profile_refs: vec![r(ProfileRef::new("T_WEB_PUBLIC_TOPK"))],
                execution_state: MethodologyExecutionStateV1::NotExecuted,
            }],
            vec![ProtectedOmissionV1 {
                id: r(OmissionRef::new("OMIT1")),
                subject_ref: r(PropositionRef::new("planner-proposal:D4")),
                commitment_ref: r(CommitmentRef::new("protected:synthetic-d4-detail")),
                reason_ref: r(ProfileRef::new("ProtectedInformationNotEmbedded")),
                raw_content_embedded: false,
            }],
            PresentationProjectionV1 {
                id: r(PresentationRef::new("ATLAS:F2")),
                frontier_ref: f2,
                authority: PresentationAuthorityV1::RenderingOnly,
            },
        )
    }

    #[test]
    fn reservoir_capsule_preserves_history_and_non_authority() {
        let capsule = reservoir_capsule().unwrap();
        assert_eq!(capsule.authority_scope(), InvestigationCapsuleAuthorityV1::RecordOnly);
        assert!(!capsule.reasoning_authority());
        assert!(!capsule.collection_authority());
        assert!(!capsule.action_authority());
        assert_eq!(capsule.frontiers.len(), 2);
        assert_eq!(capsule.dependency_groups[0].artifact_refs.len(), 3);
        assert_eq!(capsule.searches[0].interpretation, NegativeSearchInterpretationV1::UnresolvedDueToUnknownCoverage);
        assert_eq!(capsule.searches[1].interpretation, NegativeSearchInterpretationV1::AbsentWithinExactFiniteCorpus);
        assert_eq!(capsule.methodology_records[0].execution_state, MethodologyExecutionStateV1::NotExecuted);
        assert_eq!(capsule.methodology_records[0].selected_profile_refs[0].as_str(), "T_WEB_PUBLIC_TOPK");
        assert!(!format!("{capsule:?}").contains("ASSUMP1"));
        assert!(!format!("{capsule:?}").contains("T_WEB_PUBLIC_TOPK"));
    }

    #[test]
    fn incoherent_negative_search_rejects() {
        let search = NegativeSearchRecordV1 {
            id: r(SearchRef::new("S")), frontier_ref: r(FrontierRef::new("F1")), result_count: 0,
            coverage: SearchCoverageV1::UnknownCoverage,
            finite_corpus_commitment: Some(r(CommitmentRef::new("fixture:wrong"))),
            interpretation: NegativeSearchInterpretationV1::AbsentWithinExactFiniteCorpus,
        };
        assert!(matches!(
            validate_searches(&[search]),
            Err(CapsuleError::IncoherentNegativeSearch(_))
        ));
    }

    #[test]
    fn negative_search_with_results_rejects() {
        let search = NegativeSearchRecordV1 {
            id: r(SearchRef::new("S")), frontier_ref: r(FrontierRef::new("F1")), result_count: 1,
            coverage: SearchCoverageV1::UnknownCoverage, finite_corpus_commitment: None,
            interpretation: NegativeSearchInterpretationV1::UnresolvedDueToUnknownCoverage,
        };
        assert!(matches!(
            validate_searches(&[search]),
            Err(CapsuleError::NegativeSearchHasResults(_))
        ));
    }

    #[test]
    fn planner_execution_authority_and_self_domination_reject() {
        let mut trace = PlannerTraceV1 {
            id: r(PlannerTraceRef::new("P")), frontier_ref: r(FrontierRef::new("F1")),
            preferred_hypothesis_ref: r(PropositionRef::new("H1")),
            profile_ref: r(ProfileRef::new("profile:p")),
            disconfirmation_candidate_refs: vec![], eligible_pareto_front_refs: vec![],
            blocked_but_analytically_useful: vec![], dominated_candidates: vec![],
            execution_authority: true,
        };
        assert!(matches!(
            validate_planners(&[trace.clone()]),
            Err(CapsuleError::PlannerExecutionAuthority(_))
        ));
        trace.execution_authority = false;
        trace.dominated_candidates.push(DominatedPlannerProposalV1 {
            proposal_ref: r(ProposalRef::new("D1")),
            dominated_by: r(ProposalRef::new("D1")),
            witness_profile_ref: r(ProfileRef::new("profile:p")),
        });
        assert!(matches!(
            validate_planners(&[trace]),
            Err(CapsuleError::PlannerSelfDomination(_))
        ));
    }
}
