//! Evidence-reconstructible after-event review (AER) benchmark.
//!
//! The engine evaluates synthetic review artefacts. It is non-punitive by
//! construction: it models system stages and owner roles, not individual
//! blame, rankings or personal suitability.
//!
//! Historical evidence is reconstructed by transaction/knowledge time.
//! Ground-truth causal labels are evaluator-only and must not be exposed to a
//! reviewer as hindsight evidence.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ReviewStage {
    RiskKnowledge,
    ObservationForecast,
    AuthorityDecision,
    WarningDissemination,
    AccessibilityComprehension,
    Preparedness,
    Response,
    OutcomeEvaluation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedMilestone {
    pub milestone_id: String,
    pub stage: ReviewStage,
    pub planned_at: u64,
    pub plan_version: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceEvent {
    pub evidence_id: String,
    pub milestone_id: String,
    pub stage: ReviewStage,
    /// Event/valid time: when the activity or observation is said to occur.
    pub event_at: u64,
    /// Transaction/knowledge time: when the review record became available.
    pub recorded_at: u64,
    pub source_id: String,
    pub plan_version: String,
    pub verified_under_fixture_policy: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TimelineSnapshot {
    pub as_of_transaction_time: u64,
    pub events: Vec<EvidenceEvent>,
}

pub fn reconstruct_as_of(events: &[EvidenceEvent], cutoff: u64) -> TimelineSnapshot {
    let mut visible: Vec<EvidenceEvent> = events
        .iter()
        .filter(|event| event.recorded_at <= cutoff)
        .cloned()
        .collect();
    visible.sort_by(|a, b| {
        a.event_at
            .cmp(&b.event_at)
            .then(a.evidence_id.cmp(&b.evidence_id))
    });
    TimelineSnapshot {
        as_of_transaction_time: cutoff,
        events: visible,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MilestoneStatus {
    OnTime,
    Late,
    Missing,
    Unverified,
    StageOrPlanVersionMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MilestoneFinding {
    pub milestone_id: String,
    pub stage: ReviewStage,
    pub status: MilestoneStatus,
    pub planned_at: u64,
    pub observed_at: Option<u64>,
    pub evidence_ids: Vec<String>,
}

pub fn evaluate_plan(plan: &[PlannedMilestone], snapshot: &TimelineSnapshot) -> Vec<MilestoneFinding> {
    plan.iter()
        .map(|milestone| {
            let mut all_matching: Vec<&EvidenceEvent> = snapshot
                .events
                .iter()
                .filter(|event| event.milestone_id == milestone.milestone_id)
                .collect();
            all_matching.sort_by_key(|event| event.event_at);
            if all_matching.is_empty() {
                return MilestoneFinding {
                    milestone_id: milestone.milestone_id.clone(),
                    stage: milestone.stage,
                    status: MilestoneStatus::Missing,
                    planned_at: milestone.planned_at,
                    observed_at: None,
                    evidence_ids: Vec::new(),
                };
            }

            let mut plan_matching: Vec<&EvidenceEvent> = all_matching
                .iter()
                .copied()
                .filter(|event| {
                    event.stage == milestone.stage
                        && event.plan_version == milestone.plan_version
                })
                .collect();
            if plan_matching.is_empty() {
                return MilestoneFinding {
                    milestone_id: milestone.milestone_id.clone(),
                    stage: milestone.stage,
                    status: MilestoneStatus::StageOrPlanVersionMismatch,
                    planned_at: milestone.planned_at,
                    observed_at: all_matching.first().map(|event| event.event_at),
                    evidence_ids: all_matching.iter().map(|event| event.evidence_id.clone()).collect(),
                };
            }

            plan_matching.sort_by_key(|event| event.event_at);
            let verified: Vec<&EvidenceEvent> = plan_matching
                .iter()
                .copied()
                .filter(|event| event.verified_under_fixture_policy)
                .collect();

            if verified.is_empty() {
                return MilestoneFinding {
                    milestone_id: milestone.milestone_id.clone(),
                    stage: milestone.stage,
                    status: MilestoneStatus::Unverified,
                    planned_at: milestone.planned_at,
                    observed_at: plan_matching.first().map(|event| event.event_at),
                    evidence_ids: plan_matching.iter().map(|event| event.evidence_id.clone()).collect(),
                };
            }

            let first_verified = verified[0];
            let status = if first_verified.event_at <= milestone.planned_at {
                MilestoneStatus::OnTime
            } else {
                MilestoneStatus::Late
            };
            MilestoneFinding {
                milestone_id: milestone.milestone_id.clone(),
                stage: milestone.stage,
                status,
                planned_at: milestone.planned_at,
                observed_at: Some(first_verified.event_at),
                evidence_ids: verified.iter().map(|event| event.evidence_id.clone()).collect(),
            }
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CausalClaimLevel {
    Chronology,
    Association,
    ProcessContribution,
    CausalEffect,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CausalClaim {
    pub claim_id: String,
    pub level: CausalClaimLevel,
    pub target_question: Option<String>,
    pub causal_assumptions: Option<String>,
    pub confounders_considered: bool,
    pub outcome_measurement: Option<String>,
    pub counterfactual_definition: Option<String>,
    pub data_limitations: Option<String>,
    pub evidence_ids: Vec<String>,
    pub proposed_cause: String,
    pub proposed_effect: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CausalAssessment {
    DescriptiveOnly,
    ProcessEvidenceSupported,
    CausalFieldsIncomplete,
    CausalClaimUnsupportedByKnownDgp,
    CausalClaimConsistentWithKnownDgp,
    CausalEffectUnidentified,
    CausalEvidenceNotVisibleAsOf,
    CausalEvidenceUnverified,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KnownDgpTruth {
    /// Evaluator-only truth; do not disclose this field to the review author.
    pub causal_effect_identified: bool,
    /// Synthetic directed causal links known to the evaluator.
    pub supported_links: BTreeSet<(String, String)>,
}

pub fn assess_causal_claim_as_of(
    claim: &CausalClaim,
    truth: &KnownDgpTruth,
    snapshot: &TimelineSnapshot,
) -> CausalAssessment {
    if claim.evidence_ids.is_empty() {
        return CausalAssessment::CausalFieldsIncomplete;
    }
    let visible: BTreeMap<&str, bool> = snapshot
        .events
        .iter()
        .map(|event| (event.evidence_id.as_str(), event.verified_under_fixture_policy))
        .collect();
    if claim.evidence_ids.iter().any(|id| !visible.contains_key(id.as_str())) {
        return CausalAssessment::CausalEvidenceNotVisibleAsOf;
    }
    if claim.evidence_ids.iter().any(|id| !visible.get(id.as_str()).copied().unwrap_or(false)) {
        return CausalAssessment::CausalEvidenceUnverified;
    }
    assess_causal_claim_against_known_dgp(claim, truth)
}

fn assess_causal_claim_against_known_dgp(claim: &CausalClaim, truth: &KnownDgpTruth) -> CausalAssessment {
    match claim.level {
        CausalClaimLevel::Chronology | CausalClaimLevel::Association => {
            CausalAssessment::DescriptiveOnly
        }
        CausalClaimLevel::ProcessContribution => {
            if claim.evidence_ids.is_empty()
                || claim.proposed_cause.trim().is_empty()
                || claim.proposed_effect.trim().is_empty()
            {
                CausalAssessment::CausalFieldsIncomplete
            } else {
                CausalAssessment::ProcessEvidenceSupported
            }
        }
        CausalClaimLevel::CausalEffect => {
            let complete = claim
                .target_question
                .as_ref()
                .is_some_and(|value| !value.trim().is_empty())
                && claim
                    .causal_assumptions
                    .as_ref()
                    .is_some_and(|value| !value.trim().is_empty())
                && claim.confounders_considered
                && claim
                    .outcome_measurement
                    .as_ref()
                    .is_some_and(|value| !value.trim().is_empty())
                && claim
                    .counterfactual_definition
                    .as_ref()
                    .is_some_and(|value| !value.trim().is_empty())
                && claim
                    .data_limitations
                    .as_ref()
                    .is_some_and(|value| !value.trim().is_empty())
                && !claim.evidence_ids.is_empty()
                && !claim.proposed_cause.trim().is_empty()
                && !claim.proposed_effect.trim().is_empty();
            if !complete {
                return CausalAssessment::CausalFieldsIncomplete;
            }
            if !truth.causal_effect_identified {
                return CausalAssessment::CausalEffectUnidentified;
            }
            if truth
                .supported_links
                .contains(&(claim.proposed_cause.clone(), claim.proposed_effect.clone()))
            {
                CausalAssessment::CausalClaimConsistentWithKnownDgp
            } else {
                CausalAssessment::CausalClaimUnsupportedByKnownDgp
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SystemFinding {
    pub finding_id: String,
    pub system_issue: String,
    pub stage: ReviewStage,
    pub evidence_ids: Vec<String>,
    pub alternative_explanations: Vec<String>,
    pub unresolved: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Recommendation {
    pub recommendation_id: String,
    pub system_issue: String,
    /// Responsible institutional/functional role, not a personal identity.
    pub owner_role: String,
    pub evidence_ids: Vec<String>,
    pub follow_up_due: Option<u64>,
    pub measurable_follow_up: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecommendationStatus {
    TraceableAndActionable,
    MissingEvidence,
    UnknownEvidence,
    UnverifiedEvidence,
    MissingSystemIssue,
    MissingOwnerRole,
    MissingFollowUp,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecommendationFinding {
    pub recommendation_id: String,
    pub status: RecommendationStatus,
}

pub fn validate_recommendations(
    recommendations: &[Recommendation],
    snapshot: &TimelineSnapshot,
) -> Vec<RecommendationFinding> {
    let visible: BTreeMap<&str, bool> = snapshot
        .events
        .iter()
        .map(|event| (event.evidence_id.as_str(), event.verified_under_fixture_policy))
        .collect();
    recommendations
        .iter()
        .map(|recommendation| {
            let status = if recommendation.evidence_ids.is_empty() {
                RecommendationStatus::MissingEvidence
            } else if recommendation
                .evidence_ids
                .iter()
                .any(|id| !visible.contains_key(id.as_str()))
            {
                RecommendationStatus::UnknownEvidence
            } else if recommendation
                .evidence_ids
                .iter()
                .any(|id| !visible.get(id.as_str()).copied().unwrap_or(false))
            {
                RecommendationStatus::UnverifiedEvidence
            } else if recommendation.system_issue.trim().is_empty() {
                RecommendationStatus::MissingSystemIssue
            } else if recommendation.owner_role.trim().is_empty() {
                RecommendationStatus::MissingOwnerRole
            } else if recommendation.follow_up_due.is_none()
                || recommendation
                    .measurable_follow_up
                    .as_ref()
                    .is_none_or(|value| value.trim().is_empty())
            {
                RecommendationStatus::MissingFollowUp
            } else {
                RecommendationStatus::TraceableAndActionable
            };
            RecommendationFinding {
                recommendation_id: recommendation.recommendation_id.clone(),
                status,
            }
        })
        .collect()
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ReviewReport {
    pub milestone_count: u32,
    pub on_time: u32,
    pub late: u32,
    pub missing: u32,
    pub unverified: u32,
    pub stage_or_plan_version_mismatch: u32,
    pub causal_claims_incomplete: u32,
    pub causal_claims_unsupported: u32,
    pub causal_effects_unidentified: u32,
    pub recommendation_count: u32,
    pub recommendations_traceable: u32,
    pub recommendations_incomplete: u32,
    pub unresolved_findings: u32,
}

pub fn summarize_review(
    milestones: &[MilestoneFinding],
    causal_assessments: &[CausalAssessment],
    recommendations: &[RecommendationFinding],
    findings: &[SystemFinding],
) -> ReviewReport {
    let mut report = ReviewReport {
        milestone_count: milestones.len() as u32,
        recommendation_count: recommendations.len() as u32,
        ..ReviewReport::default()
    };
    for milestone in milestones {
        match milestone.status {
            MilestoneStatus::OnTime => report.on_time += 1,
            MilestoneStatus::Late => report.late += 1,
            MilestoneStatus::Missing => report.missing += 1,
            MilestoneStatus::Unverified => report.unverified += 1,
            MilestoneStatus::StageOrPlanVersionMismatch => report.stage_or_plan_version_mismatch += 1,
        }
    }
    for assessment in causal_assessments {
        match assessment {
            CausalAssessment::CausalFieldsIncomplete => report.causal_claims_incomplete += 1,
            CausalAssessment::CausalClaimUnsupportedByKnownDgp => report.causal_claims_unsupported += 1,
            CausalAssessment::CausalEffectUnidentified => report.causal_effects_unidentified += 1,
            CausalAssessment::CausalEvidenceNotVisibleAsOf | CausalAssessment::CausalEvidenceUnverified => report.causal_claims_incomplete += 1,
            _ => {}
        }
    }
    for recommendation in recommendations {
        if recommendation.status == RecommendationStatus::TraceableAndActionable {
            report.recommendations_traceable += 1;
        } else {
            report.recommendations_incomplete += 1;
        }
    }
    report.unresolved_findings = findings.iter().filter(|finding| finding.unresolved).count() as u32;
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(id: &str, milestone: &str, event_at: u64, recorded_at: u64, verified: bool) -> EvidenceEvent {
        EvidenceEvent {
            evidence_id: id.into(),
            milestone_id: milestone.into(),
            stage: ReviewStage::WarningDissemination,
            event_at,
            recorded_at,
            source_id: "synthetic-source".into(),
            plan_version: "v1".into(),
            verified_under_fixture_policy: verified,
        }
    }

    fn complete_causal_claim() -> CausalClaim {
        CausalClaim {
            claim_id: "claim-1".into(),
            level: CausalClaimLevel::CausalEffect,
            target_question: Some("Did delayed warning change timely reach?".into()),
            causal_assumptions: Some("Synthetic randomized scenario assignment".into()),
            confounders_considered: true,
            outcome_measurement: Some("Aggregate timely reach".into()),
            counterfactual_definition: Some("Same scenario with on-time warning".into()),
            data_limitations: Some("Synthetic cohort-level outcomes only".into()),
            evidence_ids: vec!["evidence-1".into()],
            proposed_cause: "warning_delay".into(),
            proposed_effect: "reduced_timely_reach".into(),
        }
    }

    #[test]
    fn later_recorded_evidence_is_excluded_from_historical_snapshot() {
        let records = vec![event("e-late", "warning", 100, 300, true)];
        let before = reconstruct_as_of(&records, 200);
        let after = reconstruct_as_of(&records, 400);
        assert!(before.events.is_empty());
        assert_eq!(after.events.len(), 1);
        assert_eq!(after.events[0].event_at, 100);
    }

    #[test]
    fn replay_is_deterministic_and_sorted_by_event_time() {
        let records = vec![
            event("e2", "m2", 200, 201, true),
            event("e1", "m1", 100, 101, true),
        ];
        assert_eq!(reconstruct_as_of(&records, 300), reconstruct_as_of(&records, 300));
        assert_eq!(reconstruct_as_of(&records, 300).events[0].evidence_id, "e1");
    }

    #[test]
    fn plan_comparison_distinguishes_on_time_late_and_missing(){
        let plan = vec![
            PlannedMilestone{milestone_id:"on-time".into(),stage:ReviewStage::WarningDissemination,planned_at:100,plan_version:"v1".into()},
            PlannedMilestone{milestone_id:"late".into(),stage:ReviewStage::WarningDissemination,planned_at:100,plan_version:"v1".into()},
            PlannedMilestone{milestone_id:"missing".into(),stage:ReviewStage::Preparedness,planned_at:100,plan_version:"v1".into()},
        ];
        let snapshot = reconstruct_as_of(&[
            event("e1","on-time",90,91,true),
            event("e2","late",120,121,true),
        ],200);
        let result=evaluate_plan(&plan,&snapshot);
        assert_eq!(result[0].status,MilestoneStatus::OnTime);
        assert_eq!(result[1].status,MilestoneStatus::Late);
        assert_eq!(result[2].status,MilestoneStatus::Missing);
    }

    #[test]
    fn evidence_recorded_after_cutoff_cannot_support_causal_claim(){
        let claim=complete_causal_claim();
        let truth=KnownDgpTruth{causal_effect_identified:true,supported_links:BTreeSet::from([("warning_delay".into(),"reduced_timely_reach".into())])};
        let snapshot=reconstruct_as_of(&[event("evidence-1","m1",100,300,true)],200);
        assert_eq!(assess_causal_claim_as_of(&claim,&truth,&snapshot),CausalAssessment::CausalEvidenceNotVisibleAsOf);
    }

    #[test]
    fn causal_claim_without_any_evidence_id_is_incomplete(){
        let mut claim=complete_causal_claim();
        claim.evidence_ids.clear();
        let truth=KnownDgpTruth{causal_effect_identified:true,supported_links:BTreeSet::from([("warning_delay".into(),"reduced_timely_reach".into())])};
        let snapshot=reconstruct_as_of(&[],500);
        assert_eq!(assess_causal_claim_as_of(&claim,&truth,&snapshot),CausalAssessment::CausalFieldsIncomplete);
    }

    #[test]
    fn unverified_as_of_evidence_cannot_support_causal_claim(){
        let claim=complete_causal_claim();
        let truth=KnownDgpTruth{causal_effect_identified:true,supported_links:BTreeSet::from([("warning_delay".into(),"reduced_timely_reach".into())])};
        let snapshot=reconstruct_as_of(&[event("evidence-1","m1",100,101,false)],200);
        assert_eq!(assess_causal_claim_as_of(&claim,&truth,&snapshot),CausalAssessment::CausalEvidenceUnverified);
    }

    #[test]
    fn recommendations_do_not_treat_unverified_evidence_as_traceable(){
        let snapshot=reconstruct_as_of(&[event("e1","m1",100,101,false)],200);
        let recommendation=Recommendation{recommendation_id:"r".into(),system_issue:"system issue".into(),owner_role:"coordination role".into(),evidence_ids:vec!["e1".into()],follow_up_due:Some(500),measurable_follow_up:Some("measure handoff time".into())};
        let result=validate_recommendations(&[recommendation],&snapshot);
        assert_eq!(result[0].status,RecommendationStatus::UnverifiedEvidence);
    }

    #[test]
    fn plan_version_or_stage_mismatch_is_not_scored_as_success(){
        let plan=vec![PlannedMilestone{milestone_id:"m".into(),stage:ReviewStage::WarningDissemination,planned_at:100,plan_version:"v2".into()}];
        let snapshot=reconstruct_as_of(&[event("e","m",90,91,true)],200);
        assert_eq!(evaluate_plan(&plan,&snapshot)[0].status,MilestoneStatus::StageOrPlanVersionMismatch);
    }

    #[test]
    fn earlier_wrong_plan_version_does_not_hide_later_matching_evidence() {
        let plan = vec![PlannedMilestone {
            milestone_id: "m".into(),
            stage: ReviewStage::WarningDissemination,
            planned_at: 100,
            plan_version: "v1".into(),
        }];
        let mut wrong = event("wrong-version", "m", 80, 81, true);
        wrong.plan_version = "v0".into();
        let right = event("matching-version", "m", 90, 91, true);
        let snapshot = reconstruct_as_of(&[wrong, right], 200);
        let finding = &evaluate_plan(&plan, &snapshot)[0];
        assert_eq!(finding.status, MilestoneStatus::OnTime);
        assert_eq!(finding.evidence_ids, vec!["matching-version"]);
    }

    #[test]
    fn earlier_unverified_record_does_not_override_later_verified_record() {
        let plan = vec![PlannedMilestone {
            milestone_id: "m".into(),
            stage: ReviewStage::WarningDissemination,
            planned_at: 100,
            plan_version: "v1".into(),
        }];
        let unverified = event("unverified-early", "m", 80, 81, false);
        let verified_late = event("verified-late", "m", 120, 121, true);
        let snapshot = reconstruct_as_of(&[unverified, verified_late], 200);
        let finding = &evaluate_plan(&plan, &snapshot)[0];
        assert_eq!(finding.status, MilestoneStatus::Late);
        assert_eq!(finding.observed_at, Some(120));
        assert_eq!(finding.evidence_ids, vec!["verified-late"]);
    }

    #[test]
    fn unverified_event_does_not_count_as_on_time(){
        let plan=vec![PlannedMilestone{milestone_id:"m".into(),stage:ReviewStage::WarningDissemination,planned_at:100,plan_version:"v1".into()}];
        let snapshot=reconstruct_as_of(&[event("e","m",90,91,false)],200);
        assert_eq!(evaluate_plan(&plan,&snapshot)[0].status,MilestoneStatus::Unverified);
    }

    #[test]
    fn descriptive_claim_is_not_promoted_to_causality(){
        let mut claim=complete_causal_claim();
        claim.level=CausalClaimLevel::Chronology;
        let truth=KnownDgpTruth{causal_effect_identified:true,supported_links:BTreeSet::new()};
        assert_eq!(assess_causal_claim_against_known_dgp(&claim,&truth),CausalAssessment::DescriptiveOnly);
    }

    #[test]
    fn missing_causal_fields_are_rejected(){
        let mut claim=complete_causal_claim();
        claim.counterfactual_definition=None;
        let truth=KnownDgpTruth{causal_effect_identified:true,supported_links:BTreeSet::from([("warning_delay".into(),"reduced_timely_reach".into())])};
        assert_eq!(assess_causal_claim(&claim,&truth),CausalAssessment::CausalFieldsIncomplete);
    }

    #[test]
    fn unknown_causal_effect_must_remain_unidentified(){
        let claim=complete_causal_claim();
        let truth=KnownDgpTruth{causal_effect_identified:false,supported_links:BTreeSet::new()};
        assert_eq!(assess_causal_claim(&claim,&truth),CausalAssessment::CausalEffectUnidentified);
    }

    #[test]
    fn unsupported_causal_link_is_not_accepted(){
        let mut claim=complete_causal_claim();
        claim.proposed_cause="wrong_cause".into();
        let truth=KnownDgpTruth{causal_effect_identified:true,supported_links:BTreeSet::from([("warning_delay".into(),"reduced_timely_reach".into())])};
        assert_eq!(assess_causal_claim(&claim,&truth),CausalAssessment::CausalClaimUnsupportedByKnownDgp);
    }

    #[test]
    fn recommendations_require_visible_evidence_system_issue_owner_and_followup(){
        let snapshot=reconstruct_as_of(&[event("e1","m1",100,101,true)],200);
        let good=Recommendation{recommendation_id:"r1".into(),system_issue:"late dissemination handoff".into(),owner_role:"regional coordination function".into(),evidence_ids:vec!["e1".into()],follow_up_due:Some(500),measurable_follow_up:Some("exercise median handoff time".into())};
        let bad=Recommendation{recommendation_id:"r2".into(),system_issue:"".into(),owner_role:"".into(),evidence_ids:vec!["missing".into()],follow_up_due:None,measurable_follow_up:None};
        let findings=validate_recommendations(&[good,bad],&snapshot);
        assert_eq!(findings[0].status,RecommendationStatus::TraceableAndActionable);
        assert_eq!(findings[1].status,RecommendationStatus::UnknownEvidence);
    }

    #[test]
    fn summaries_report_unresolved_and_incomplete_without_composite_score(){
        let milestones=vec![MilestoneFinding{milestone_id:"m".into(),stage:ReviewStage::Response,status:MilestoneStatus::Missing,planned_at:10,observed_at:None,evidence_ids:vec![]}];
        let recommendations=vec![RecommendationFinding{recommendation_id:"r".into(),status:RecommendationStatus::MissingFollowUp}];
        let findings=vec![SystemFinding{finding_id:"f".into(),system_issue:"coordination gap".into(),stage:ReviewStage::AuthorityDecision,evidence_ids:vec!["e".into()],alternative_explanations:vec!["source delay".into()],unresolved:true}];
        let report=summarize_review(&milestones,&[CausalAssessment::CausalEffectUnidentified],&recommendations,&findings);
        assert_eq!(report.missing,1);
        assert_eq!(report.causal_effects_unidentified,1);
        assert_eq!(report.recommendations_incomplete,1);
        assert_eq!(report.unresolved_findings,1);
    }
}
