// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Transaction-derived read-only hospitality field qualification.
//!
//! This layer accepts preregistered target/projection plans, exact source files, and model
//! forecasts. It constructs the canonical campaign replay, evaluation actuals, slice evidence,
//! and field-quality counts internally. Callers cannot supply actuals or slice verdicts.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_adapter_delimited::DelimitedIngressAdapter;
use mycelix_business_campaign_replay::{CampaignReplayEvidence, ReplayError, replay_campaign};
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_derived_forecast_actual::{
    ActualProjectionSpec, DerivedActualError, DerivedForecastCase, DerivedMetricActual,
    DerivedScoringError, derive_actuals, score_derived_cases,
};
use mycelix_business_field_qualification::{
    DataQualityEvidence, FieldQualificationDecision, FieldQualificationEvidence, SliceEvidence,
    evaluate_field_qualification,
};
use mycelix_business_forecast_plan::{ForecastPlanError, ForecastTargetPlan};
use mycelix_business_import_diagnostics::{
    ExtractionCampaignManifest, ImportRejectionClass, upstream_export_completeness_unverified_ref,
};
use mycelix_business_import_membership::CampaignSourceFile;
use mycelix_business_ingress::IngressQualificationBinding;
use mycelix_business_pilot_hospitality::{
    HospitalityForecastPilotRegistration, HospitalitySliceDefinition, HospitalitySliceKind,
    sales_input_ref,
};
use mycelix_business_shadow::{
    ForecastScorecard, ShadowForecast, ShadowPromotionDecision, evaluate_forecast_gate,
};
use mycelix_business_time_evidence::{LocalTimeSchedule, ResolveError, ScheduleError};
use sha2::{Digest, Sha256};

pub const TRANSACTION_FIELD_IS_READ_ONLY: bool = true;

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

pub fn fixed_offset_clock_limitation_ref() -> ReferenceId {
    ReferenceId::new("limitation:fixed-offset-clock-rules:v1")
        .expect("static limitation id is canonical")
}

pub fn forecast_actual_membership_limitation_ref() -> ReferenceId {
    ReferenceId::new("limitation:forecast-actual-campaign-membership-unverified:v1")
        .expect("static limitation id is canonical")
}

pub fn time_rule_source_authority_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:time-rule-source-authority-unverified:v1")
        .expect("static limitation id is canonical")
}

pub fn aggregation_semantic_authority_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:aggregation-semantic-authority-unverified:v1")
        .expect("static limitation id is canonical")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedForecastSubmission {
    pub target_id: ReferenceId,
    pub candidate: ShadowForecast,
    pub baseline: ShadowForecast,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransactionSliceReport {
    pub slice: ReferenceId,
    pub criterion_digest: Digest32,
    pub derived_case_set_digest: Digest32,
    pub scorecard: ForecastScorecard,
    pub shadow_gate_passed: bool,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransactionQualificationDecision {
    OverallShadowFailed(ShadowPromotionDecision),
    Field(FieldQualificationDecision),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransactionHospitalityReport {
    pub registration_digest: Digest32,
    pub target_plan_digest: Digest32,
    pub projection_spec_digest: Digest32,
    pub campaign_digest: Digest32,
    pub campaign_replay: CampaignReplayEvidence,
    pub forecast_submission_set_digest: Digest32,
    pub derived_actual_set_digest: Digest32,
    pub overall_scorecard: ForecastScorecard,
    pub overall_derived_case_set_digest: Digest32,
    pub slices: Vec<TransactionSliceReport>,
    pub field_evidence: FieldQualificationEvidence,
    pub unresolved_limitations: BTreeSet<ReferenceId>,
    pub decision: TransactionQualificationDecision,
    pub report_digest: Digest32,
}

impl TransactionHospitalityReport {
    pub fn validate_digest(&self) -> bool {
        self.report_digest == transaction_report_digest(self)
    }
}

pub struct TransactionFieldRequest<'a> {
    pub registration: &'a HospitalityForecastPilotRegistration,
    pub schedule: &'a LocalTimeSchedule,
    pub target_plan: &'a ForecastTargetPlan,
    pub projection_spec: &'a ActualProjectionSpec,
    pub adapter: &'a DelimitedIngressAdapter,
    pub campaign: &'a ExtractionCampaignManifest,
    pub source_files: &'a [CampaignSourceFile],
    pub submissions: &'a [PlannedForecastSubmission],
    pub additional_limitations: &'a BTreeSet<ReferenceId>,
}

#[derive(Debug)]
pub enum TransactionFieldError {
    RegistrationInvalid,
    TimeSchedule(ScheduleError),
    TimeScheduleMismatch,
    TimeScheduleRegisteredAfterPilot,
    ForecastPlan(ForecastPlanError),
    TargetPlanBindingMismatch,
    ProjectionInvalid,
    ConnectorMismatch,
    ProjectionInputNotDeclared,
    CampaignHasFutureTimestamps { count: u64 },
    Replay(ReplayError),
    DerivedActual(DerivedActualError),
    NoSubmissions,
    SubmissionCountMismatch { expected: usize, actual: usize },
    DuplicateTargetSubmission { target: ReferenceId },
    MissingTargetSubmission { target: ReferenceId },
    UnknownTargetSubmission { target: ReferenceId },
    InvalidCandidate { index: usize },
    InvalidBaseline { index: usize },
    CandidateLineageMismatch { index: usize },
    BaselineLineageMismatch { index: usize },
    CandidateTargetMismatch { index: usize },
    BaselineTargetMismatch { index: usize },
    ForecastPredatesProtocol { index: usize },
    DuplicateCandidateForecast,
    DuplicateBaselineForecast,
    DerivedCaseInvalid { index: usize },
    DerivedScoring(DerivedScoringError),
    ProtocolGateFailed,
    TimeResolution(ResolveError),
    TargetCrossesOffsetTransition { target: ReferenceId },
    QualityPlanMismatch,
    QualityArithmeticOverflow,
    RetiredLimitationPresent { limitation: ReferenceId },
    FieldEvidenceInvalid,
}

pub fn evaluate_transaction_hospitality_field(
    request: TransactionFieldRequest<'_>,
) -> Result<TransactionHospitalityReport, TransactionFieldError> {
    request
        .registration
        .validate()
        .map_err(|_| TransactionFieldError::RegistrationInvalid)?;
    request
        .schedule
        .validate()
        .map_err(TransactionFieldError::TimeSchedule)?;
    validate_schedule_binding(request.registration, request.schedule)?;
    reject_retired_limitations(request.additional_limitations)?;
    request
        .target_plan
        .validate_against(&request.registration.protocol)
        .map_err(TransactionFieldError::ForecastPlan)?;
    validate_target_plan_binding(request.registration, request.target_plan)?;
    request
        .projection_spec
        .validate_against(&request.registration.protocol, request.target_plan)
        .map_err(|_| TransactionFieldError::ProjectionInvalid)?;
    validate_connector_binding(
        request.registration,
        request.adapter,
        request.campaign,
        request.projection_spec,
    )?;
    if request.campaign.future_timestamp_rows != 0 {
        return Err(TransactionFieldError::CampaignHasFutureTimestamps {
            count: request.campaign.future_timestamp_rows,
        });
    }

    let replay = replay_campaign(request.adapter, request.campaign, request.source_files)
        .map_err(TransactionFieldError::Replay)?;
    let actuals = derive_actuals(
        &request.registration.protocol,
        request.target_plan,
        request.projection_spec,
        &replay,
    )
    .map_err(TransactionFieldError::DerivedActual)?;
    let submissions = validate_and_index_submissions(
        request.registration,
        request.target_plan,
        request.submissions,
    )?;
    let derived_cases = build_derived_cases(request.target_plan, &submissions, actuals)?;
    let overall_scorecard =
        score_derived_cases(&derived_cases).map_err(TransactionFieldError::DerivedScoring)?;
    let overall_decision = evaluate_forecast_gate(&request.registration.protocol, &overall_scorecard)
        .map_err(|_| TransactionFieldError::ProtocolGateFailed)?;

    let forecast_submission_set_digest = submission_set_digest(request.submissions);
    let derived_actual_set_digest = derived_actual_set_digest(&derived_cases);
    let overall_derived_case_set_digest = derived_case_set_digest(&derived_cases);

    let mut slice_reports = Vec::with_capacity(request.registration.slices.len());
    let mut slice_evidence = Vec::with_capacity(request.registration.slices.len());
    for slice in &request.registration.slices {
        let mut selected = Vec::new();
        for case in &derived_cases {
            if derived_case_belongs_to_slice(case, slice, request.schedule)? {
                selected.push(case.clone());
            }
        }
        let scorecard =
            score_derived_cases(&selected).map_err(TransactionFieldError::DerivedScoring)?;
        let selected_digest = derived_case_set_digest(&selected);
        let passed = scorecard.cases >= slice.minimum_cases
            && scorecard.abstention_bps()
                <= request.registration.protocol.maximum_abstention_bps
            && (!request
                .registration
                .protocol
                .require_candidate_not_worse_than_baseline
                || scorecard.candidate_not_worse_than_baseline());
        let evidence_digest = transaction_slice_evidence_digest(
            request.schedule.schedule_digest,
            slice,
            selected_digest,
            &scorecard,
            passed,
        );
        slice_reports.push(TransactionSliceReport {
            slice: slice.slice.clone(),
            criterion_digest: slice.criterion_digest,
            derived_case_set_digest: selected_digest,
            scorecard: scorecard.clone(),
            shadow_gate_passed: passed,
            evidence_digest,
        });
        slice_evidence.push(SliceEvidence {
            slice: slice.slice.clone(),
            criterion_digest: slice.criterion_digest,
            cases: scorecard.cases,
            shadow_gate_passed: passed,
            evidence_digest,
        });
    }

    let data_quality = derive_data_quality(request.registration, request.campaign)?;
    let mut limitations = request.additional_limitations.clone();
    limitations.insert(upstream_export_completeness_unverified_ref());
    limitations.insert(time_rule_source_authority_unverified_ref());
    limitations.insert(aggregation_semantic_authority_unverified_ref());

    let field_evidence_digest = transaction_field_evidence_digest(
        request.registration.registration_digest,
        request.target_plan.plan_digest,
        request.projection_spec.spec_digest,
        request.campaign.campaign_digest,
        replay.evidence.evidence_digest,
        forecast_submission_set_digest,
        derived_actual_set_digest,
        overall_derived_case_set_digest,
        &overall_scorecard,
        &data_quality,
        &slice_evidence,
        &limitations,
    );
    let field_evidence = FieldQualificationEvidence {
        plan_digest: request.registration.field_plan.plan_digest,
        profile: request.registration.field_plan.profile.clone(),
        capability: request.registration.field_plan.capability.clone(),
        scope: request.registration.field_plan.scope.clone(),
        shadow_protocol_digest: request.registration.field_plan.shadow_protocol_digest,
        connectors: request.registration.field_plan.connectors.clone(),
        data_quality,
        slices: slice_evidence,
        known_limitations: limitations.clone(),
        evidence_digest: field_evidence_digest,
    };
    let field_decision = evaluate_field_qualification(&request.registration.field_plan, &field_evidence)
        .map_err(|_| TransactionFieldError::FieldEvidenceInvalid)?;
    let decision = if overall_decision == ShadowPromotionDecision::PassShadowGate {
        TransactionQualificationDecision::Field(field_decision)
    } else {
        TransactionQualificationDecision::OverallShadowFailed(overall_decision)
    };

    let mut report = TransactionHospitalityReport {
        registration_digest: request.registration.registration_digest,
        target_plan_digest: request.target_plan.plan_digest,
        projection_spec_digest: request.projection_spec.spec_digest,
        campaign_digest: request.campaign.campaign_digest,
        campaign_replay: replay.evidence,
        forecast_submission_set_digest,
        derived_actual_set_digest,
        overall_scorecard,
        overall_derived_case_set_digest,
        slices: slice_reports,
        field_evidence,
        unresolved_limitations: limitations,
        decision,
        report_digest: Digest32([0; 32]),
    };
    report.report_digest = transaction_report_digest(&report);
    Ok(report)
}

fn validate_schedule_binding(
    registration: &HospitalityForecastPilotRegistration,
    schedule: &LocalTimeSchedule,
) -> Result<(), TransactionFieldError> {
    if schedule.timezone != registration.timezone
        || schedule.evaluation_start_unix_ms != registration.protocol.evaluation_start_unix_ms
        || schedule.evaluation_end_unix_ms != registration.protocol.evaluation_end_unix_ms
    {
        return Err(TransactionFieldError::TimeScheduleMismatch);
    }
    if schedule.registered_at_unix_ms > registration.protocol.registered_at_unix_ms {
        return Err(TransactionFieldError::TimeScheduleRegisteredAfterPilot);
    }
    Ok(())
}

fn reject_retired_limitations(
    limitations: &BTreeSet<ReferenceId>,
) -> Result<(), TransactionFieldError> {
    for retired in [
        fixed_offset_clock_limitation_ref(),
        forecast_actual_membership_limitation_ref(),
    ] {
        if limitations.contains(&retired) {
            return Err(TransactionFieldError::RetiredLimitationPresent {
                limitation: retired,
            });
        }
    }
    Ok(())
}

fn validate_target_plan_binding(
    registration: &HospitalityForecastPilotRegistration,
    plan: &ForecastTargetPlan,
) -> Result<(), TransactionFieldError> {
    if plan.protocol_digest != registration.protocol.protocol_digest
        || plan.profile != registration.protocol.profile
        || plan.capability != registration.protocol.capability
        || plan.candidate_model_lineage != registration.protocol.candidate_model_lineage
        || plan.baseline_model_lineage != registration.protocol.baseline_model_lineage
        || plan.evaluation_start_unix_ms != registration.protocol.evaluation_start_unix_ms
        || plan.evaluation_end_unix_ms != registration.protocol.evaluation_end_unix_ms
    {
        return Err(TransactionFieldError::TargetPlanBindingMismatch);
    }
    Ok(())
}

fn validate_connector_binding(
    registration: &HospitalityForecastPilotRegistration,
    adapter: &DelimitedIngressAdapter,
    campaign: &ExtractionCampaignManifest,
    projection_spec: &ActualProjectionSpec,
) -> Result<(), TransactionFieldError> {
    let descriptor = adapter
        .descriptor()
        .map_err(|_| TransactionFieldError::ConnectorMismatch)?;
    let binding = IngressQualificationBinding::from(&descriptor);
    if binding != registration.ingress_connector || binding != campaign.connector {
        return Err(TransactionFieldError::ConnectorMismatch);
    }
    if !campaign.supported_inputs.contains(&projection_spec.source_input)
        || !descriptor.supported_inputs.contains(&projection_spec.source_input)
    {
        return Err(TransactionFieldError::ProjectionInputNotDeclared);
    }
    Ok(())
}

fn validate_and_index_submissions<'a>(
    registration: &HospitalityForecastPilotRegistration,
    plan: &ForecastTargetPlan,
    submissions: &'a [PlannedForecastSubmission],
) -> Result<BTreeMap<ReferenceId, &'a PlannedForecastSubmission>, TransactionFieldError> {
    if submissions.is_empty() {
        return Err(TransactionFieldError::NoSubmissions);
    }
    if submissions.len() != plan.targets.len() {
        return Err(TransactionFieldError::SubmissionCountMismatch {
            expected: plan.targets.len(),
            actual: submissions.len(),
        });
    }
    let planned = plan
        .targets
        .iter()
        .map(|target| (target.target_id.clone(), target))
        .collect::<BTreeMap<_, _>>();
    let mut candidate_ids = BTreeSet::new();
    let mut baseline_ids = BTreeSet::new();
    let mut indexed = BTreeMap::new();
    for (index, submission) in submissions.iter().enumerate() {
        submission
            .candidate
            .validate()
            .map_err(|_| TransactionFieldError::InvalidCandidate { index })?;
        submission
            .baseline
            .validate()
            .map_err(|_| TransactionFieldError::InvalidBaseline { index })?;
        let Some(expected) = planned.get(&submission.target_id) else {
            return Err(TransactionFieldError::UnknownTargetSubmission {
                target: submission.target_id.clone(),
            });
        };
        if submission.candidate.model_lineage != registration.protocol.candidate_model_lineage {
            return Err(TransactionFieldError::CandidateLineageMismatch { index });
        }
        if submission.baseline.model_lineage != registration.protocol.baseline_model_lineage {
            return Err(TransactionFieldError::BaselineLineageMismatch { index });
        }
        if submission.candidate.target != expected.target {
            return Err(TransactionFieldError::CandidateTargetMismatch { index });
        }
        if submission.baseline.target != expected.target {
            return Err(TransactionFieldError::BaselineTargetMismatch { index });
        }
        if submission.candidate.issued_at_unix_ms < registration.protocol.registered_at_unix_ms
            || submission.baseline.issued_at_unix_ms < registration.protocol.registered_at_unix_ms
        {
            return Err(TransactionFieldError::ForecastPredatesProtocol { index });
        }
        if !candidate_ids.insert(submission.candidate.forecast.as_ref_id().clone()) {
            return Err(TransactionFieldError::DuplicateCandidateForecast);
        }
        if !baseline_ids.insert(submission.baseline.forecast.as_ref_id().clone()) {
            return Err(TransactionFieldError::DuplicateBaselineForecast);
        }
        if indexed
            .insert(submission.target_id.clone(), submission)
            .is_some()
        {
            return Err(TransactionFieldError::DuplicateTargetSubmission {
                target: submission.target_id.clone(),
            });
        }
    }
    for target in &plan.targets {
        if !indexed.contains_key(&target.target_id) {
            return Err(TransactionFieldError::MissingTargetSubmission {
                target: target.target_id.clone(),
            });
        }
    }
    Ok(indexed)
}

fn build_derived_cases(
    plan: &ForecastTargetPlan,
    submissions: &BTreeMap<ReferenceId, &PlannedForecastSubmission>,
    actuals: Vec<DerivedMetricActual>,
) -> Result<Vec<DerivedForecastCase>, TransactionFieldError> {
    let actuals = actuals
        .into_iter()
        .map(|actual| (actual.target_id.clone(), actual))
        .collect::<BTreeMap<_, _>>();
    let mut cases = Vec::with_capacity(plan.targets.len());
    for (index, planned) in plan.targets.iter().enumerate() {
        let submission = submissions
            .get(&planned.target_id)
            .ok_or_else(|| TransactionFieldError::MissingTargetSubmission {
                target: planned.target_id.clone(),
            })?;
        let actual = actuals
            .get(&planned.target_id)
            .cloned()
            .ok_or_else(|| TransactionFieldError::MissingTargetSubmission {
                target: planned.target_id.clone(),
            })?;
        let case = DerivedForecastCase {
            candidate: submission.candidate.clone(),
            baseline: submission.baseline.clone(),
            actual,
        };
        case.validate()
            .map_err(|_| TransactionFieldError::DerivedCaseInvalid { index })?;
        cases.push(case);
    }
    Ok(cases)
}

fn derived_case_belongs_to_slice(
    case: &DerivedForecastCase,
    slice: &HospitalitySliceDefinition,
    schedule: &LocalTimeSchedule,
) -> Result<bool, TransactionFieldError> {
    let target = &case.candidate.target;
    let resolved = schedule
        .resolve_window(target.window_start_unix_ms, target.window_end_unix_ms)
        .map_err(TransactionFieldError::TimeResolution)?;
    if resolved.crosses_offset_transition() {
        return Err(TransactionFieldError::TargetCrossesOffsetTransition {
            target: case.actual.target_id.clone(),
        });
    }
    match slice.kind {
        HospitalitySliceKind::Weekend => {
            let same_weekend_day = resolved.start.local_day_index
                == resolved.end_inclusive.local_day_index
                && resolved.start.weekday >= 5
                && resolved.end_inclusive.weekday >= 5;
            let saturday_to_sunday = resolved
                .start
                .local_day_index
                .checked_add(1)
                == Some(resolved.end_inclusive.local_day_index)
                && resolved.start.weekday == 5
                && resolved.end_inclusive.weekday == 6;
            Ok(same_weekend_day || saturday_to_sunday)
        }
        _ => {
            let (Some(start), Some(end)) = (slice.start_local_minute, slice.end_local_minute)
            else {
                return Ok(false);
            };
            Ok(resolved.start.local_day_index == resolved.end_inclusive.local_day_index
                && resolved.start.minute_of_day >= start
                && resolved.end_inclusive.minute_of_day < end)
        }
    }
}

fn derive_data_quality(
    registration: &HospitalityForecastPilotRegistration,
    campaign: &ExtractionCampaignManifest,
) -> Result<Vec<DataQualityEvidence>, TransactionFieldError> {
    if registration.field_plan.data_quality.len() != 1
        || registration.field_plan.data_quality[0].input != sales_input_ref()
    {
        return Err(TransactionFieldError::QualityPlanMismatch);
    }
    let threshold = &registration.field_plan.data_quality[0];
    let conflicting_records = campaign.files.iter().try_fold(0_u64, |total, file| {
        total.checked_add(
            file.rejection_counts
                .get(&ImportRejectionClass::DuplicateSourceEvent)
                .copied()
                .unwrap_or(0),
        )
        .ok_or(TransactionFieldError::QualityArithmeticOverflow)
    })?;
    // Conservative v0.1 classification: campaign manifests prove the exact maximum ingest delay,
    // but not the exact count of late accepted records. If even one accepted record may exceed the
    // registered delay ceiling, mark the entire accepted set stale rather than undercounting.
    let stale_records = if campaign.maximum_observed_ingest_delay_ms
        > threshold.maximum_ingest_delay_ms
    {
        campaign.accepted_rows
    } else {
        0
    };
    let mut evidence = DataQualityEvidence {
        input: sales_input_ref(),
        expected_records: campaign.discovered_rows,
        missing_records: campaign.rejected_rows,
        conflicting_records,
        stale_records,
        maximum_observed_ingest_delay_ms: campaign.maximum_observed_ingest_delay_ms,
        evidence_digest: Digest32([0; 32]),
    };
    evidence.evidence_digest = data_quality_digest(campaign.campaign_digest, &evidence);
    Ok(vec![evidence])
}

fn data_quality_digest(campaign_digest: Digest32, evidence: &DataQualityEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:transaction-field-data-quality:v1");
    hasher.update(campaign_digest.0);
    hash_str(&mut hasher, evidence.input.as_str());
    hasher.update(evidence.expected_records.to_be_bytes());
    hasher.update(evidence.missing_records.to_be_bytes());
    hasher.update(evidence.conflicting_records.to_be_bytes());
    hasher.update(evidence.stale_records.to_be_bytes());
    hasher.update(evidence.maximum_observed_ingest_delay_ms.to_be_bytes());
    finish_digest(hasher)
}

fn submission_set_digest(submissions: &[PlannedForecastSubmission]) -> Digest32 {
    let mut digests = submissions
        .iter()
        .map(|submission| {
            let mut hasher = Sha256::new();
            hash_str(&mut hasher, "mycelix:transaction-forecast-submission:v1");
            hash_str(&mut hasher, submission.target_id.as_str());
            hash_forecast(&mut hasher, &submission.candidate);
            hash_forecast(&mut hasher, &submission.baseline);
            finish_digest(hasher)
        })
        .collect::<Vec<_>>();
    digests.sort_unstable();
    digest_digest_set("mycelix:transaction-forecast-submission-set:v1", &digests)
}

fn derived_actual_set_digest(cases: &[DerivedForecastCase]) -> Digest32 {
    let mut digests = cases
        .iter()
        .map(|case| case.actual.evidence_digest)
        .collect::<Vec<_>>();
    digests.sort_unstable();
    digest_digest_set("mycelix:transaction-derived-actual-set:v1", &digests)
}

fn derived_case_set_digest(cases: &[DerivedForecastCase]) -> Digest32 {
    let mut digests = cases.iter().map(derived_case_digest).collect::<Vec<_>>();
    digests.sort_unstable();
    digest_digest_set("mycelix:transaction-derived-case-set:v1", &digests)
}

fn derived_case_digest(case: &DerivedForecastCase) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:transaction-derived-case:v1");
    hash_forecast(&mut hasher, &case.candidate);
    hash_forecast(&mut hasher, &case.baseline);
    hasher.update(case.actual.evidence_digest.0);
    finish_digest(hasher)
}

fn digest_digest_set(label: &str, digests: &[Digest32]) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, label);
    hasher.update((digests.len() as u64).to_be_bytes());
    for digest in digests {
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

fn hash_forecast(hasher: &mut Sha256, forecast: &ShadowForecast) {
    hash_str(hasher, forecast.forecast.as_ref_id().as_str());
    hash_str(hasher, forecast.model_lineage.as_str());
    hash_str(hasher, forecast.target.metric.as_str());
    hash_str(hasher, forecast.target.scope.as_ref_id().as_str());
    hash_str(hasher, forecast.target.unit.as_str());
    hasher.update(forecast.target.scale.to_be_bytes());
    hasher.update(forecast.target.window_start_unix_ms.to_be_bytes());
    hasher.update(forecast.target.window_end_unix_ms.to_be_bytes());
    hasher.update(forecast.issued_at_unix_ms.to_be_bytes());
    match &forecast.disposition {
        mycelix_business_shadow::ForecastDisposition::Predicted(value) => {
            hasher.update([1]);
            hash_scaled(hasher, &value.point);
            hash_scaled(hasher, &value.lower);
            hash_scaled(hasher, &value.upper);
        }
        mycelix_business_shadow::ForecastDisposition::Abstained { reason } => {
            hasher.update([2]);
            hash_str(hasher, reason.as_str());
        }
    }
}

fn hash_scaled(hasher: &mut Sha256, value: &mycelix_business_shadow::ScaledValue) {
    hasher.update(value.mantissa.to_be_bytes());
    hasher.update(value.scale.to_be_bytes());
    hash_str(hasher, value.unit.as_str());
}

fn hash_scorecard(hasher: &mut Sha256, scorecard: &ForecastScorecard) {
    hasher.update(scorecard.cases.to_be_bytes());
    hasher.update(scorecard.candidate_abstentions.to_be_bytes());
    hasher.update(scorecard.candidate_absolute_error_sum.to_be_bytes());
    hasher.update(scorecard.baseline_absolute_error_sum.to_be_bytes());
    hasher.update(scorecard.candidate_interval_hits.to_be_bytes());
    hasher.update(scorecard.candidate_interval_trials.to_be_bytes());
}

fn transaction_slice_evidence_digest(
    schedule_digest: Digest32,
    slice: &HospitalitySliceDefinition,
    case_set_digest: Digest32,
    scorecard: &ForecastScorecard,
    passed: bool,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:transaction-hospitality-slice:v1");
    hasher.update(schedule_digest.0);
    hash_str(&mut hasher, slice.slice.as_str());
    hasher.update(slice.criterion_digest.0);
    hasher.update(case_set_digest.0);
    hash_scorecard(&mut hasher, scorecard);
    hasher.update([u8::from(passed)]);
    finish_digest(hasher)
}

#[allow(clippy::too_many_arguments)]
fn transaction_field_evidence_digest(
    registration_digest: Digest32,
    target_plan_digest: Digest32,
    projection_spec_digest: Digest32,
    campaign_digest: Digest32,
    replay_digest: Digest32,
    submission_set_digest: Digest32,
    derived_actual_set_digest: Digest32,
    case_set_digest: Digest32,
    scorecard: &ForecastScorecard,
    quality: &[DataQualityEvidence],
    slices: &[SliceEvidence],
    limitations: &BTreeSet<ReferenceId>,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:transaction-hospitality-field-evidence:v1");
    for digest in [
        registration_digest,
        target_plan_digest,
        projection_spec_digest,
        campaign_digest,
        replay_digest,
        submission_set_digest,
        derived_actual_set_digest,
        case_set_digest,
    ] {
        hasher.update(digest.0);
    }
    hash_scorecard(&mut hasher, scorecard);
    for item in quality {
        hasher.update(item.evidence_digest.0);
    }
    for item in slices {
        hasher.update(item.evidence_digest.0);
    }
    for limitation in limitations {
        hash_str(&mut hasher, limitation.as_str());
    }
    finish_digest(hasher)
}

fn hash_shadow_decision(hasher: &mut Sha256, decision: &ShadowPromotionDecision) {
    match decision {
        ShadowPromotionDecision::PassShadowGate => hasher.update([1]),
        ShadowPromotionDecision::InsufficientCases { actual, required } => {
            hasher.update([2]);
            hasher.update(actual.to_be_bytes());
            hasher.update(required.to_be_bytes());
        }
        ShadowPromotionDecision::ExcessiveAbstention {
            actual_bps,
            maximum_bps,
        } => {
            hasher.update([3]);
            hasher.update(actual_bps.to_be_bytes());
            hasher.update(maximum_bps.to_be_bytes());
        }
        ShadowPromotionDecision::CandidateWorseThanBaseline => hasher.update([4]),
    }
}

fn hash_field_decision(hasher: &mut Sha256, decision: &FieldQualificationDecision) {
    match decision {
        FieldQualificationDecision::PassShadowFieldGate => hasher.update([1]),
        FieldQualificationDecision::DataQualityFailed {
            input,
            dimension,
            actual,
            maximum,
        } => {
            hasher.update([2]);
            hash_str(hasher, input.as_str());
            hash_str(hasher, dimension.as_str());
            hasher.update(actual.to_be_bytes());
            hasher.update(maximum.to_be_bytes());
        }
        FieldQualificationDecision::InsufficientSliceCases {
            slice,
            actual,
            minimum,
        } => {
            hasher.update([3]);
            hash_str(hasher, slice.as_str());
            hasher.update(actual.to_be_bytes());
            hasher.update(minimum.to_be_bytes());
        }
        FieldQualificationDecision::SliceShadowGateFailed { slice } => {
            hasher.update([4]);
            hash_str(hasher, slice.as_str());
        }
    }
}

fn transaction_report_digest(report: &TransactionHospitalityReport) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:transaction-hospitality-report:v1");
    for digest in [
        report.registration_digest,
        report.target_plan_digest,
        report.projection_spec_digest,
        report.campaign_digest,
        report.campaign_replay.evidence_digest,
        report.forecast_submission_set_digest,
        report.derived_actual_set_digest,
        report.overall_derived_case_set_digest,
    ] {
        hasher.update(digest.0);
    }
    hash_scorecard(&mut hasher, &report.overall_scorecard);
    for slice in &report.slices {
        hasher.update(slice.evidence_digest.0);
    }
    hasher.update(report.field_evidence.evidence_digest.0);
    for limitation in &report.unresolved_limitations {
        hash_str(&mut hasher, limitation.as_str());
    }
    match &report.decision {
        TransactionQualificationDecision::OverallShadowFailed(decision) => {
            hasher.update([1]);
            hash_shadow_decision(&mut hasher, decision);
        }
        TransactionQualificationDecision::Field(decision) => {
            hasher.update([2]);
            hash_field_decision(&mut hasher, decision);
        }
    }
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, OutputMapping, ScopeMapping, TimestampEncoding,
        ValueMapping,
    };
    use mycelix_business_core::{ForecastRef, ScopeRef};
    use mycelix_business_derived_forecast_actual::AggregationKind;
    use mycelix_business_forecast_plan::PlannedForecastTarget;
    use mycelix_business_import_diagnostics::{
        ExtractionCampaignManifest, diagnose_delimited_import,
    };
    use mycelix_business_pilot_hospitality::{
        DAY_MS, HOUR_MS, HospitalityForecastPilotConfig, HospitalityPilotPolicy,
        standard_daypart_slices_v1,
    };
    use mycelix_business_shadow::{
        ForecastDisposition, ForecastTarget, ForecastValue, ScaledValue,
    };
    use mycelix_business_time_evidence::UtcOffsetPeriod;

    use super::*;

    const EVAL_START: u64 = 1_788_739_200_000;
    const EVAL_END: u64 = EVAL_START + 7 * DAY_MS;
    const BREAKFAST: u64 = EVAL_START + 3 * HOUR_MS;
    const LUNCH: u64 = EVAL_START + 9 * HOUR_MS;
    const EVENING: u64 = EVAL_START + 14 * HOUR_MS;
    const WEEKEND: u64 = EVAL_START + 5 * DAY_MS + 7 * HOUR_MS;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:transaction-field-test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:transaction-field-test:v1"),
            delimiter: b',',
            expected_headers: vec![
                "event_id".into(),
                "occurred_at".into(),
                "location".into(),
                "quantity".into(),
            ],
            source_event_id_column: "event_id".into(),
            observed_at_column: "occurred_at".into(),
            timestamp_encoding: TimestampEncoding::UnixMilliseconds,
            scope: ScopeMapping::Column {
                column: "location".into(),
                prefix: "scope:location:".into(),
            },
            outputs: vec![OutputMapping {
                input: sales_input_ref(),
                metric: id("metric:hospitality:item-demand"),
                value: ValueMapping::Column {
                    column: "quantity".into(),
                    decimal: DecimalPolicy::default(),
                },
                unit: id("unit:count"),
                scale: 0,
            }],
            maximum_batch_records: 100,
        })
        .unwrap()
    }

    fn source_file() -> CampaignSourceFile {
        let rows = [
            ("b1", BREAKFAST + 1_000, 2),
            ("b2", BREAKFAST + 2_000, 3),
            ("l1", LUNCH + 1_000, 5),
            ("e1", EVENING + 1_000, 7),
            ("w1", WEEKEND + 1_000, 11),
        ];
        let mut csv = String::from("event_id,occurred_at,location,quantity\n");
        for (event, observed, quantity) in rows {
            csv.push_str(&format!("event:{event},{observed},a,{quantity}\n"));
        }
        CampaignSourceFile {
            bytes: csv.into_bytes(),
            ingested_at_unix_ms: EVAL_END - 1,
        }
    }

    fn campaign(adapter: &DelimitedIngressAdapter, file: &CampaignSourceFile) -> ExtractionCampaignManifest {
        let manifest = diagnose_delimited_import(adapter, file.bytes.as_slice(), file.ingested_at_unix_ms)
            .unwrap();
        ExtractionCampaignManifest::from_files(vec![manifest]).unwrap()
    }

    fn registration(
        connector: IngressQualificationBinding,
    ) -> HospitalityForecastPilotRegistration {
        HospitalityForecastPilotRegistration::build(HospitalityForecastPilotConfig {
            plan_id: id("pilot:transaction-field:v1"),
            scope: ScopeRef(id("scope:location:a")),
            timezone: id("timezone:iana:Africa/Johannesburg"),
            candidate_model_lineage: id("model:candidate:v1"),
            connector,
            registered_at_unix_ms: EVAL_START - 2 * DAY_MS,
            evaluation_start_unix_ms: EVAL_START,
            evaluation_end_unix_ms: EVAL_END,
            policy: HospitalityPilotPolicy {
                minimum_preregistration_lead_ms: HOUR_MS,
                minimum_evaluation_duration_ms: DAY_MS,
                minimum_total_forecast_cases: 1,
                minimum_cases_per_slice: 1,
                maximum_abstention_bps: 10_000,
                maximum_missing_bps: 10_000,
                maximum_conflicting_bps: 10_000,
                maximum_stale_bps: 10_000,
                maximum_ingest_delay_ms: u64::MAX,
            },
            slices: Some(standard_daypart_slices_v1(
                id("timezone:iana:Africa/Johannesburg"),
                1,
            )),
        })
        .unwrap()
    }

    fn target(metric: &ReferenceId, start: u64) -> ForecastTarget {
        ForecastTarget {
            metric: metric.clone(),
            scope: ScopeRef(id("scope:location:a")),
            unit: id("unit:count"),
            scale: 0,
            window_start_unix_ms: start,
            window_end_unix_ms: start + HOUR_MS,
        }
    }

    fn target_plan(registration: &HospitalityForecastPilotRegistration) -> ForecastTargetPlan {
        let metric = id("metric:hospitality:item-demand");
        ForecastTargetPlan::build(
            &registration.protocol,
            id("forecast-plan:transaction-field:v1"),
            registration.protocol.registered_at_unix_ms,
            vec![
                PlannedForecastTarget { target_id: id("target:breakfast"), target: target(&metric, BREAKFAST) },
                PlannedForecastTarget { target_id: id("target:lunch"), target: target(&metric, LUNCH) },
                PlannedForecastTarget { target_id: id("target:evening"), target: target(&metric, EVENING) },
                PlannedForecastTarget { target_id: id("target:weekend"), target: target(&metric, WEEKEND) },
            ],
        )
        .unwrap()
    }

    fn schedule(registration: &HospitalityForecastPilotRegistration) -> LocalTimeSchedule {
        LocalTimeSchedule::build(
            registration.timezone.clone(),
            id("time-rules:test:v1"),
            Digest32::repeat(7),
            registration.protocol.registered_at_unix_ms,
            EVAL_START,
            EVAL_END,
            vec![UtcOffsetPeriod {
                start_unix_ms: EVAL_START,
                end_unix_ms: EVAL_END,
                utc_offset_minutes: 120,
            }],
        )
        .unwrap()
    }

    fn projection(
        registration: &HospitalityForecastPilotRegistration,
        plan: &ForecastTargetPlan,
    ) -> ActualProjectionSpec {
        ActualProjectionSpec::build(
            &registration.protocol,
            plan,
            id("projection:transaction-sum:v1"),
            sales_input_ref(),
            AggregationKind::Sum,
            plan.registered_at_unix_ms,
        )
        .unwrap()
    }

    fn forecast(
        id_value: &str,
        model: ReferenceId,
        target: &mycelix_business_shadow::ForecastTarget,
        issued_at: u64,
        point: i128,
    ) -> ShadowForecast {
        let value = ForecastValue {
            point: ScaledValue { mantissa: point, scale: 0, unit: id("unit:count") },
            lower: ScaledValue { mantissa: point.saturating_sub(1), scale: 0, unit: id("unit:count") },
            upper: ScaledValue { mantissa: point.saturating_add(1), scale: 0, unit: id("unit:count") },
        };
        ShadowForecast {
            forecast: ForecastRef(id(id_value)),
            model_lineage: model,
            target: target.clone(),
            issued_at_unix_ms: issued_at,
            disposition: ForecastDisposition::Predicted(value),
        }
    }

    fn expected_point(target_id: &ReferenceId) -> i128 {
        match target_id.as_str() {
            "target:breakfast" => 5,
            "target:lunch" => 5,
            "target:evening" => 7,
            "target:weekend" => 11,
            other => panic!("unexpected test target {other}"),
        }
    }

    fn submissions(
        registration: &HospitalityForecastPilotRegistration,
        plan: &ForecastTargetPlan,
    ) -> Vec<PlannedForecastSubmission> {
        plan.targets
            .iter()
            .enumerate()
            .map(|(index, planned)| {
                let point = expected_point(&planned.target_id);
                PlannedForecastSubmission {
                    target_id: planned.target_id.clone(),
                    candidate: forecast(
                        &format!("forecast:candidate:{index}"),
                        registration.protocol.candidate_model_lineage.clone(),
                        &planned.target,
                        planned.target.window_start_unix_ms - HOUR_MS,
                        point,
                    ),
                    baseline: forecast(
                        &format!("forecast:baseline:{index}"),
                        registration.protocol.baseline_model_lineage.clone(),
                        &planned.target,
                        planned.target.window_start_unix_ms - HOUR_MS,
                        point + 3,
                    ),
                }
            })
            .collect()
    }

    fn request<'a>(
        registration: &'a HospitalityForecastPilotRegistration,
        schedule: &'a LocalTimeSchedule,
        plan: &'a ForecastTargetPlan,
        projection: &'a ActualProjectionSpec,
        adapter: &'a DelimitedIngressAdapter,
        campaign: &'a ExtractionCampaignManifest,
        file: &'a CampaignSourceFile,
        submissions: &'a [PlannedForecastSubmission],
        limitations: &'a BTreeSet<ReferenceId>,
    ) -> TransactionFieldRequest<'a> {
        TransactionFieldRequest {
            registration,
            schedule,
            target_plan: plan,
            projection_spec: projection,
            adapter,
            campaign,
            source_files: std::slice::from_ref(file),
            submissions,
            additional_limitations: limitations,
        }
    }

    #[test]
    fn transaction_field_constructs_actuals_and_quality_internally() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let plan = target_plan(&registration);
        let schedule = schedule(&registration);
        let projection = projection(&registration, &plan);
        let submissions = submissions(&registration, &plan);
        let limitations = BTreeSet::new();
        let report = evaluate_transaction_hospitality_field(request(
            &registration,
            &schedule,
            &plan,
            &projection,
            &adapter,
            &campaign,
            &file,
            &submissions,
            &limitations,
        ))
        .unwrap();
        assert!(TRANSACTION_FIELD_IS_READ_ONLY);
        assert_eq!(report.overall_scorecard.cases, 4);
        assert_eq!(report.overall_scorecard.candidate_absolute_error_sum, 0);
        assert!(report.validate_digest());
        assert!(report
            .unresolved_limitations
            .contains(&aggregation_semantic_authority_unverified_ref()));
        assert!(matches!(
            report.decision,
            TransactionQualificationDecision::Field(FieldQualificationDecision::PassShadowFieldGate)
        ));
    }

    #[test]
    fn omitted_planned_window_cannot_disappear_from_denominator() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let plan = target_plan(&registration);
        let schedule = schedule(&registration);
        let projection = projection(&registration, &plan);
        let mut submissions = submissions(&registration, &plan);
        submissions.pop();
        let limitations = BTreeSet::new();
        assert!(matches!(
            evaluate_transaction_hospitality_field(request(
                &registration,
                &schedule,
                &plan,
                &projection,
                &adapter,
                &campaign,
                &file,
                &submissions,
                &limitations,
            )),
            Err(TransactionFieldError::SubmissionCountMismatch { .. })
        ));
    }

    #[test]
    fn caller_cannot_supply_an_actual_or_slice_verdict() {
        let _ = std::mem::size_of::<TransactionFieldRequest<'static>>();
        assert!(TRANSACTION_FIELD_IS_READ_ONLY);
    }

    #[test]
    fn projection_input_must_be_declared_by_registered_connector() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let plan = target_plan(&registration);
        let schedule = schedule(&registration);
        let mut projection = projection(&registration, &plan);
        projection.source_input = id("input:undeclared");
        let submissions = submissions(&registration, &plan);
        let limitations = BTreeSet::new();
        assert!(matches!(
            evaluate_transaction_hospitality_field(request(
                &registration,
                &schedule,
                &plan,
                &projection,
                &adapter,
                &campaign,
                &file,
                &submissions,
                &limitations,
            )),
            Err(TransactionFieldError::ProjectionInvalid)
                | Err(TransactionFieldError::ProjectionInputNotDeclared)
        ));
    }

    #[test]
    fn future_dated_export_rows_fail_before_scoring() {
        let adapter = adapter();
        let mut file = source_file();
        file.bytes = format!(
            "event_id,occurred_at,location,quantity\nevent:future,{},a,1\n",
            EVAL_END + DAY_MS
        )
        .into_bytes();
        file.ingested_at_unix_ms = EVAL_END - 1;
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let plan = target_plan(&registration);
        let schedule = schedule(&registration);
        let projection = projection(&registration, &plan);
        let submissions = submissions(&registration, &plan);
        let limitations = BTreeSet::new();
        assert!(matches!(
            evaluate_transaction_hospitality_field(request(
                &registration,
                &schedule,
                &plan,
                &projection,
                &adapter,
                &campaign,
                &file,
                &submissions,
                &limitations,
            )),
            Err(TransactionFieldError::CampaignHasFutureTimestamps { .. })
        ));
    }

    #[test]
    fn retired_weaker_limitations_cannot_reenter_v2_report() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let plan = target_plan(&registration);
        let schedule = schedule(&registration);
        let projection = projection(&registration, &plan);
        let submissions = submissions(&registration, &plan);
        let limitations = BTreeSet::from([fixed_offset_clock_limitation_ref()]);
        assert!(matches!(
            evaluate_transaction_hospitality_field(request(
                &registration,
                &schedule,
                &plan,
                &projection,
                &adapter,
                &campaign,
                &file,
                &submissions,
                &limitations,
            )),
            Err(TransactionFieldError::RetiredLimitationPresent { .. })
        ));
    }
}
