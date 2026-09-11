// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Strong read-only hospitality field qualification.
//!
//! This layer composes independently checkable evidence for campaign integrity, exact normalized
//! actual membership, preregistered transition-aware local-time rules, forecast scoring, slice
//! qualification, and field data quality. It grants no authority and introduces no write path.

use std::collections::BTreeSet;

use mycelix_business_adapter_delimited::DelimitedIngressAdapter;
use mycelix_business_campaign_integrity::{
    CampaignIntegrityEvidence, IntegrityError, verify_campaign_integrity,
};
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_field_qualification::{
    DataQualityEvidence, FieldQualificationDecision, FieldQualificationEvidence, SliceEvidence,
};
use mycelix_business_import_diagnostics::{
    ExtractionBoundHospitalityEvidence, ExtractionBoundPilotError, ExtractionCampaignManifest,
    evaluate_extraction_bound_hospitality_pilot, upstream_export_completeness_unverified_ref,
};
use mycelix_business_import_membership::{
    CampaignSourceFile, MembershipError, ObservationMembershipEvidence,
    verify_observation_membership,
};
use mycelix_business_pilot_hospitality::{
    HospitalityForecastPilotRegistration, HospitalityPilotEvidence, HospitalitySliceDefinition,
    HospitalitySliceKind,
};
use mycelix_business_shadow::{
    ForecastCase, ForecastDisposition, ForecastScorecard, ScaledValue, ShadowForecast,
    ShadowPromotionDecision, evaluate_forecast_gate,
};
use mycelix_business_time_evidence::{
    LocalTimeSchedule, ResolveError, ScheduleError,
};
use sha2::{Digest, Sha256};

pub const STRONG_HOSPITALITY_FIELD_IS_READ_ONLY: bool = true;

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StrongSliceReport {
    pub slice: ReferenceId,
    pub criterion_digest: Digest32,
    pub case_set_digest: Digest32,
    pub scorecard: ForecastScorecard,
    pub shadow_gate_passed: bool,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StrongQualificationDecision {
    OverallShadowFailed(ShadowPromotionDecision),
    Field(FieldQualificationDecision),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StrongHospitalityReport {
    pub registration_digest: Digest32,
    pub campaign_digest: Digest32,
    pub campaign_integrity: CampaignIntegrityEvidence,
    pub actual_membership: ObservationMembershipEvidence,
    pub time_schedule_digest: Digest32,
    pub overall_scorecard: ForecastScorecard,
    pub overall_case_set_digest: Digest32,
    pub slices: Vec<StrongSliceReport>,
    pub field_evidence: FieldQualificationEvidence,
    pub unresolved_limitations: BTreeSet<ReferenceId>,
    pub decision: StrongQualificationDecision,
    pub report_digest: Digest32,
}

impl StrongHospitalityReport {
    pub fn validate_digest(&self) -> bool {
        self.report_digest == strong_report_digest(self)
    }
}

#[derive(Debug)]
pub enum StrongFieldError {
    RegistrationInvalid,
    TimeSchedule(ScheduleError),
    TimeScheduleMismatch,
    TimeScheduleRegisteredAfterPilot,
    CampaignIntegrity(IntegrityError),
    Membership(MembershipError),
    EmptyCaseSet,
    TooManyCases,
    InvalidCase { index: usize },
    CandidateLineageMismatch { index: usize },
    BaselineLineageMismatch { index: usize },
    CaseOutsideEvaluationWindow { index: usize },
    ForecastPredatesProtocol { index: usize },
    ScopeMismatch { index: usize },
    ActualSourceMismatch { index: usize },
    ActualMappingMismatch { index: usize },
    DuplicateCandidateForecast,
    DuplicateBaselineForecast,
    DuplicateActualObservation,
    DuplicateSourceEvent,
    DuplicateTargetWindow,
    TimeResolution(ResolveError),
    TargetCrossesOffsetTransition,
    ScoringFailed,
    ProtocolGateFailed,
    RetiredLimitationPresent { limitation: ReferenceId },
    Extraction(ExtractionBoundPilotError),
}

/// Compose the strong read-only hospitality field theorem.
///
/// The function intentionally recomputes campaign-integrity and actual-membership evidence from
/// the exact source files instead of trusting externally supplied proof objects. A passing result is
/// still qualification evidence only; it grants no business authority.
pub fn evaluate_strong_hospitality_field(
    registration: &HospitalityForecastPilotRegistration,
    schedule: &LocalTimeSchedule,
    adapter: &DelimitedIngressAdapter,
    campaign: ExtractionCampaignManifest,
    source_files: &[CampaignSourceFile],
    cases: Vec<ForecastCase>,
    data_quality: Vec<DataQualityEvidence>,
    mut unresolved_limitations: BTreeSet<ReferenceId>,
) -> Result<StrongHospitalityReport, StrongFieldError> {
    registration
        .validate()
        .map_err(|_| StrongFieldError::RegistrationInvalid)?;
    schedule.validate().map_err(StrongFieldError::TimeSchedule)?;
    validate_schedule_binding(registration, schedule)?;
    reject_retired_limitations(&unresolved_limitations)?;
    if cases.is_empty() {
        return Err(StrongFieldError::EmptyCaseSet);
    }
    validate_cases(registration, &cases)?;

    let campaign_integrity = verify_campaign_integrity(adapter, &campaign, source_files)
        .map_err(StrongFieldError::CampaignIntegrity)?;
    let actuals = cases
        .iter()
        .map(|case| case.actual.clone())
        .collect::<Vec<_>>();
    let actual_membership = verify_observation_membership(
        adapter,
        &campaign,
        source_files,
        &actuals,
    )
    .map_err(StrongFieldError::Membership)?;

    let overall_scorecard =
        ForecastScorecard::score(&cases).map_err(|_| StrongFieldError::ScoringFailed)?;
    let overall_case_set_digest = case_set_digest(&cases);
    let overall_decision = evaluate_forecast_gate(&registration.protocol, &overall_scorecard)
        .map_err(|_| StrongFieldError::ProtocolGateFailed)?;

    let mut slice_reports = Vec::with_capacity(registration.slices.len());
    let mut slice_evidence = Vec::with_capacity(registration.slices.len());
    for slice in &registration.slices {
        let mut selected = Vec::new();
        for case in &cases {
            if case_belongs_to_slice(case, slice, schedule)? {
                selected.push(case.clone());
            }
        }
        let scorecard =
            ForecastScorecard::score(&selected).map_err(|_| StrongFieldError::ScoringFailed)?;
        let selected_digest = case_set_digest(&selected);
        let passed = scorecard.cases >= slice.minimum_cases
            && scorecard.abstention_bps() <= registration.protocol.maximum_abstention_bps
            && (!registration.protocol.require_candidate_not_worse_than_baseline
                || scorecard.candidate_not_worse_than_baseline());
        let evidence_digest = slice_evidence_digest(
            schedule.schedule_digest,
            slice,
            selected_digest,
            &scorecard,
            passed,
        );
        slice_reports.push(StrongSliceReport {
            slice: slice.slice.clone(),
            criterion_digest: slice.criterion_digest,
            case_set_digest: selected_digest,
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

    // These are the two limitations this stronger path still cannot discharge by construction.
    // Campaign membership proves what is in the export, not that the provider exported everything.
    // A digest-bound civil-time schedule proves stable semantics, not that the rule source is an
    // independently authoritative timezone/jurisdictional source.
    unresolved_limitations.insert(upstream_export_completeness_unverified_ref());
    unresolved_limitations.insert(time_rule_source_authority_unverified_ref());

    let field_evidence_digest = field_evidence_digest(
        registration.registration_digest,
        campaign.campaign_digest,
        campaign_integrity.evidence_digest,
        actual_membership.evidence_digest,
        schedule.schedule_digest,
        overall_case_set_digest,
        &overall_scorecard,
        &data_quality,
        &slice_evidence,
        &unresolved_limitations,
    );
    let field_evidence = FieldQualificationEvidence {
        plan_digest: registration.field_plan.plan_digest,
        profile: registration.field_plan.profile.clone(),
        capability: registration.field_plan.capability.clone(),
        scope: registration.field_plan.scope.clone(),
        shadow_protocol_digest: registration.field_plan.shadow_protocol_digest,
        connectors: registration.field_plan.connectors.clone(),
        data_quality,
        slices: slice_evidence,
        known_limitations: unresolved_limitations.clone(),
        evidence_digest: field_evidence_digest,
    };

    // Field/extraction validation always runs even when the model gate itself fails. A weak model
    // cannot hide denominator, connector, or data-quality defects.
    let pilot_evidence = HospitalityPilotEvidence {
        registration_digest: registration.registration_digest,
        ingress_connector: registration.ingress_connector.clone(),
        field_evidence: field_evidence.clone(),
    };
    let extraction_evidence = ExtractionBoundHospitalityEvidence {
        campaign: campaign.clone(),
        pilot_evidence,
    };
    let field_decision = evaluate_extraction_bound_hospitality_pilot(
        registration,
        &extraction_evidence,
    )
    .map_err(StrongFieldError::Extraction)?;

    let decision = if overall_decision == ShadowPromotionDecision::PassShadowGate {
        StrongQualificationDecision::Field(field_decision)
    } else {
        StrongQualificationDecision::OverallShadowFailed(overall_decision)
    };

    let mut report = StrongHospitalityReport {
        registration_digest: registration.registration_digest,
        campaign_digest: campaign.campaign_digest,
        campaign_integrity,
        actual_membership,
        time_schedule_digest: schedule.schedule_digest,
        overall_scorecard,
        overall_case_set_digest,
        slices: slice_reports,
        field_evidence,
        unresolved_limitations,
        decision,
        report_digest: Digest32([0; 32]),
    };
    report.report_digest = strong_report_digest(&report);
    Ok(report)
}

fn validate_schedule_binding(
    registration: &HospitalityForecastPilotRegistration,
    schedule: &LocalTimeSchedule,
) -> Result<(), StrongFieldError> {
    if schedule.timezone != registration.timezone
        || schedule.evaluation_start_unix_ms != registration.protocol.evaluation_start_unix_ms
        || schedule.evaluation_end_unix_ms != registration.protocol.evaluation_end_unix_ms
    {
        return Err(StrongFieldError::TimeScheduleMismatch);
    }
    // The local-time interpretation must be frozen no later than the pilot protocol itself.
    if schedule.registered_at_unix_ms > registration.protocol.registered_at_unix_ms {
        return Err(StrongFieldError::TimeScheduleRegisteredAfterPilot);
    }
    Ok(())
}

fn reject_retired_limitations(
    limitations: &BTreeSet<ReferenceId>,
) -> Result<(), StrongFieldError> {
    for retired in [
        fixed_offset_clock_limitation_ref(),
        forecast_actual_membership_limitation_ref(),
    ] {
        if limitations.contains(&retired) {
            return Err(StrongFieldError::RetiredLimitationPresent {
                limitation: retired,
            });
        }
    }
    Ok(())
}

fn validate_cases(
    registration: &HospitalityForecastPilotRegistration,
    cases: &[ForecastCase],
) -> Result<(), StrongFieldError> {
    if cases.len() > u32::MAX as usize {
        return Err(StrongFieldError::TooManyCases);
    }
    let mut candidate_ids = BTreeSet::new();
    let mut baseline_ids = BTreeSet::new();
    let mut actual_ids = BTreeSet::new();
    let mut source_events = BTreeSet::new();
    let mut targets = BTreeSet::new();

    for (index, case) in cases.iter().enumerate() {
        case.validate()
            .map_err(|_| StrongFieldError::InvalidCase { index })?;
        if case.candidate.model_lineage != registration.protocol.candidate_model_lineage {
            return Err(StrongFieldError::CandidateLineageMismatch { index });
        }
        if case.baseline.model_lineage != registration.protocol.baseline_model_lineage {
            return Err(StrongFieldError::BaselineLineageMismatch { index });
        }
        let target = &case.candidate.target;
        if target.window_start_unix_ms < registration.protocol.evaluation_start_unix_ms
            || target.window_end_unix_ms > registration.protocol.evaluation_end_unix_ms
        {
            return Err(StrongFieldError::CaseOutsideEvaluationWindow { index });
        }
        if case.candidate.issued_at_unix_ms < registration.protocol.registered_at_unix_ms
            || case.baseline.issued_at_unix_ms < registration.protocol.registered_at_unix_ms
        {
            return Err(StrongFieldError::ForecastPredatesProtocol { index });
        }
        if target.scope != registration.field_plan.scope {
            return Err(StrongFieldError::ScopeMismatch { index });
        }
        if case.actual.source_system != registration.ingress_connector.source_system {
            return Err(StrongFieldError::ActualSourceMismatch { index });
        }
        if case.actual.mapping_digest != registration.ingress_connector.mapping_digest {
            return Err(StrongFieldError::ActualMappingMismatch { index });
        }
        if !candidate_ids.insert(case.candidate.forecast.as_ref_id().clone()) {
            return Err(StrongFieldError::DuplicateCandidateForecast);
        }
        if !baseline_ids.insert(case.baseline.forecast.as_ref_id().clone()) {
            return Err(StrongFieldError::DuplicateBaselineForecast);
        }
        if !actual_ids.insert(case.actual.observation.as_ref_id().clone()) {
            return Err(StrongFieldError::DuplicateActualObservation);
        }
        if !source_events.insert(case.actual.source_event_id.clone()) {
            return Err(StrongFieldError::DuplicateSourceEvent);
        }
        let target_identity = (
            target.metric.clone(),
            target.scope.clone(),
            target.window_start_unix_ms,
            target.window_end_unix_ms,
        );
        if !targets.insert(target_identity) {
            return Err(StrongFieldError::DuplicateTargetWindow);
        }
    }
    Ok(())
}

fn case_belongs_to_slice(
    case: &ForecastCase,
    slice: &HospitalitySliceDefinition,
    schedule: &LocalTimeSchedule,
) -> Result<bool, StrongFieldError> {
    let target = &case.candidate.target;
    let resolved = schedule
        .resolve_window(target.window_start_unix_ms, target.window_end_unix_ms)
        .map_err(StrongFieldError::TimeResolution)?;
    if resolved.crosses_offset_transition() {
        return Err(StrongFieldError::TargetCrossesOffsetTransition);
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

fn compute_case_digest(case: &ForecastCase) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:strong-hospitality-forecast-case:v1");
    hash_forecast(&mut hasher, &case.candidate);
    hash_forecast(&mut hasher, &case.baseline);
    hash_str(&mut hasher, case.actual.observation.as_ref_id().as_str());
    hash_str(&mut hasher, case.actual.source_system.as_str());
    hash_str(&mut hasher, case.actual.source_event_id.as_str());
    hasher.update(case.actual.source_payload_digest.0);
    hasher.update(case.actual.mapping_digest.0);
    hash_str(&mut hasher, case.actual.metric.as_str());
    hash_str(&mut hasher, case.actual.scope.as_ref_id().as_str());
    hash_scaled(&mut hasher, &case.actual.value);
    hasher.update(case.actual.observed_at_unix_ms.to_be_bytes());
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
        ForecastDisposition::Predicted(value) => {
            hasher.update([1]);
            hash_scaled(hasher, &value.point);
            hash_scaled(hasher, &value.lower);
            hash_scaled(hasher, &value.upper);
        }
        ForecastDisposition::Abstained { reason } => {
            hasher.update([2]);
            hash_str(hasher, reason.as_str());
        }
    }
}

fn hash_scaled(hasher: &mut Sha256, value: &ScaledValue) {
    hasher.update(value.mantissa.to_be_bytes());
    hasher.update(value.scale.to_be_bytes());
    hash_str(hasher, value.unit.as_str());
}

fn case_set_digest(cases: &[ForecastCase]) -> Digest32 {
    let mut digests = cases.iter().map(compute_case_digest).collect::<Vec<_>>();
    digests.sort_unstable();
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:strong-hospitality-case-set:v1");
    hasher.update((digests.len() as u64).to_be_bytes());
    for digest in digests {
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

fn hash_scorecard(hasher: &mut Sha256, scorecard: &ForecastScorecard) {
    hasher.update(scorecard.cases.to_be_bytes());
    hasher.update(scorecard.candidate_abstentions.to_be_bytes());
    hasher.update(scorecard.candidate_absolute_error_sum.to_be_bytes());
    hasher.update(scorecard.baseline_absolute_error_sum.to_be_bytes());
    hasher.update(scorecard.candidate_interval_hits.to_be_bytes());
    hasher.update(scorecard.candidate_interval_trials.to_be_bytes());
}

fn slice_evidence_digest(
    schedule_digest: Digest32,
    slice: &HospitalitySliceDefinition,
    case_set_digest: Digest32,
    scorecard: &ForecastScorecard,
    passed: bool,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:strong-hospitality-slice-evidence:v1");
    hasher.update(schedule_digest.0);
    hash_str(&mut hasher, slice.slice.as_str());
    hasher.update(slice.criterion_digest.0);
    hasher.update(case_set_digest.0);
    hash_scorecard(&mut hasher, scorecard);
    hasher.update([u8::from(passed)]);
    finish_digest(hasher)
}

#[allow(clippy::too_many_arguments)]
fn field_evidence_digest(
    registration_digest: Digest32,
    campaign_digest: Digest32,
    campaign_integrity_digest: Digest32,
    membership_digest: Digest32,
    schedule_digest: Digest32,
    case_set_digest: Digest32,
    overall_scorecard: &ForecastScorecard,
    data_quality: &[DataQualityEvidence],
    slices: &[SliceEvidence],
    limitations: &BTreeSet<ReferenceId>,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:strong-hospitality-field-evidence:v1");
    hasher.update(registration_digest.0);
    hasher.update(campaign_digest.0);
    hasher.update(campaign_integrity_digest.0);
    hasher.update(membership_digest.0);
    hasher.update(schedule_digest.0);
    hasher.update(case_set_digest.0);
    hash_scorecard(&mut hasher, overall_scorecard);

    let mut quality = data_quality.iter().collect::<Vec<_>>();
    quality.sort_by(|a, b| a.input.cmp(&b.input));
    for item in quality {
        hash_str(&mut hasher, item.input.as_str());
        hasher.update(item.expected_records.to_be_bytes());
        hasher.update(item.missing_records.to_be_bytes());
        hasher.update(item.conflicting_records.to_be_bytes());
        hasher.update(item.stale_records.to_be_bytes());
        hasher.update(item.maximum_observed_ingest_delay_ms.to_be_bytes());
        hasher.update(item.evidence_digest.0);
    }

    let mut slice_items = slices.iter().collect::<Vec<_>>();
    slice_items.sort_by(|a, b| a.slice.cmp(&b.slice));
    for item in slice_items {
        hash_str(&mut hasher, item.slice.as_str());
        hasher.update(item.criterion_digest.0);
        hasher.update(item.cases.to_be_bytes());
        hasher.update([u8::from(item.shadow_gate_passed)]);
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

fn strong_report_digest(report: &StrongHospitalityReport) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:strong-hospitality-report:v1");
    hasher.update(report.registration_digest.0);
    hasher.update(report.campaign_digest.0);
    hasher.update(report.campaign_integrity.evidence_digest.0);
    hasher.update(report.actual_membership.evidence_digest.0);
    hasher.update(report.time_schedule_digest.0);
    hasher.update(report.overall_case_set_digest.0);
    hash_scorecard(&mut hasher, &report.overall_scorecard);
    for slice in &report.slices {
        hasher.update(slice.evidence_digest.0);
    }
    hasher.update(report.field_evidence.evidence_digest.0);
    for limitation in &report.unresolved_limitations {
        hash_str(&mut hasher, limitation.as_str());
    }
    match &report.decision {
        StrongQualificationDecision::OverallShadowFailed(decision) => {
            hasher.update([1]);
            hash_shadow_decision(&mut hasher, decision);
        }
        StrongQualificationDecision::Field(decision) => {
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
    use mycelix_business_import_diagnostics::{
        ExtractionCampaignManifest, diagnose_delimited_import,
    };
    use mycelix_business_pilot_hospitality::{
        DAY_MS, HOUR_MS, HospitalityForecastPilotConfig, HospitalityPilotPolicy, sales_input_ref,
        standard_daypart_slices_v1,
    };
    use mycelix_business_shadow::{ForecastTarget, ForecastValue, ShadowForecast};
    use mycelix_business_time_evidence::UtcOffsetPeriod;

    use super::*;

    const EVAL_START: u64 = 1_788_739_200_000;
    const EVAL_END: u64 = 1_789_344_000_000;
    const BREAKFAST: u64 = 1_788_760_800_000;
    const LUNCH: u64 = 1_788_775_200_000;
    const EVENING: u64 = 1_788_796_800_000;
    const WEEKEND: u64 = 1_789_192_800_000;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:strong-field-test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:strong-field-test:v1"),
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
            ("breakfast", BREAKFAST + HOUR_MS + 1, 10),
            ("lunch", LUNCH + HOUR_MS + 1, 20),
            ("evening", EVENING + HOUR_MS + 1, 30),
            ("weekend", WEEKEND + HOUR_MS + 1, 40),
        ];
        let mut csv = String::from("event_id,occurred_at,location,quantity\n");
        for (name, observed, quantity) in rows {
            csv.push_str(&format!("event:{name},{observed},a,{quantity}\n"));
        }
        CampaignSourceFile {
            bytes: csv.into_bytes(),
            ingested_at_unix_ms: EVAL_END - 1,
        }
    }

    fn campaign(adapter: &DelimitedIngressAdapter, file: &CampaignSourceFile) -> ExtractionCampaignManifest {
        let manifest = diagnose_delimited_import(
            adapter,
            file.bytes.as_slice(),
            file.ingested_at_unix_ms,
        )
        .unwrap();
        ExtractionCampaignManifest::from_files(vec![manifest]).unwrap()
    }

    fn registration(
        connector: mycelix_business_ingress::IngressQualificationBinding,
    ) -> HospitalityForecastPilotRegistration {
        HospitalityForecastPilotRegistration::build(HospitalityForecastPilotConfig {
            plan_id: id("pilot:strong-field:v1"),
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

    fn actuals(adapter: &DelimitedIngressAdapter, file: &CampaignSourceFile) -> Vec<mycelix_business_shadow::MetricObservation> {
        adapter
            .parse_batch(file.bytes.as_slice(), file.ingested_at_unix_ms)
            .unwrap()
            .records
            .into_iter()
            .flat_map(|record| record.normalized.into_iter().map(|item| item.observation))
            .collect()
    }

    fn cases(
        registration: &HospitalityForecastPilotRegistration,
        adapter: &DelimitedIngressAdapter,
        file: &CampaignSourceFile,
    ) -> Vec<ForecastCase> {
        let starts = [BREAKFAST, LUNCH, EVENING, WEEKEND];
        actuals(adapter, file)
            .into_iter()
            .zip(starts)
            .enumerate()
            .map(|(index, (actual, start))| {
                let unit = actual.value.unit.clone();
                let target = ForecastTarget {
                    metric: actual.metric.clone(),
                    scope: registration.field_plan.scope.clone(),
                    unit: unit.clone(),
                    scale: actual.value.scale,
                    window_start_unix_ms: start,
                    window_end_unix_ms: start + HOUR_MS,
                };
                let forecast_value = |point: i128| ForecastValue {
                    point: ScaledValue {
                        mantissa: point,
                        scale: actual.value.scale,
                        unit: unit.clone(),
                    },
                    lower: ScaledValue {
                        mantissa: point.saturating_sub(1),
                        scale: actual.value.scale,
                        unit: unit.clone(),
                    },
                    upper: ScaledValue {
                        mantissa: point.saturating_add(1),
                        scale: actual.value.scale,
                        unit: unit.clone(),
                    },
                };
                ForecastCase {
                    candidate: ShadowForecast {
                        forecast: ForecastRef(id(&format!("forecast:candidate:{index}"))),
                        model_lineage: registration.protocol.candidate_model_lineage.clone(),
                        target: target.clone(),
                        issued_at_unix_ms: start - HOUR_MS,
                        disposition: ForecastDisposition::Predicted(forecast_value(
                            actual.value.mantissa,
                        )),
                    },
                    baseline: ShadowForecast {
                        forecast: ForecastRef(id(&format!("forecast:baseline:{index}"))),
                        model_lineage: registration.protocol.baseline_model_lineage.clone(),
                        target,
                        issued_at_unix_ms: start - HOUR_MS,
                        disposition: ForecastDisposition::Predicted(forecast_value(
                            actual.value.mantissa + 5,
                        )),
                    },
                    actual,
                }
            })
            .collect()
    }

    fn quality(campaign: &ExtractionCampaignManifest) -> Vec<DataQualityEvidence> {
        vec![DataQualityEvidence {
            input: sales_input_ref(),
            expected_records: campaign.discovered_rows,
            missing_records: campaign.rejected_rows,
            conflicting_records: 0,
            stale_records: 0,
            maximum_observed_ingest_delay_ms: campaign.maximum_observed_ingest_delay_ms,
            evidence_digest: Digest32::repeat(9),
        }]
    }

    #[test]
    fn strong_field_requires_all_independent_proofs() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let schedule = schedule(&registration);
        let cases = cases(&registration, &adapter, &file);
        let report = evaluate_strong_hospitality_field(
            &registration,
            &schedule,
            &adapter,
            campaign.clone(),
            std::slice::from_ref(&file),
            cases,
            quality(&campaign),
            BTreeSet::new(),
        )
        .unwrap();
        assert!(STRONG_HOSPITALITY_FIELD_IS_READ_ONLY);
        assert_eq!(
            report.decision,
            StrongQualificationDecision::Field(FieldQualificationDecision::PassShadowFieldGate)
        );
        assert!(report.campaign_integrity.validate_against(&campaign).is_ok());
        assert!(report.validate_digest());
        assert!(report
            .unresolved_limitations
            .contains(&upstream_export_completeness_unverified_ref()));
        assert!(report
            .unresolved_limitations
            .contains(&time_rule_source_authority_unverified_ref()));
        assert!(!report
            .unresolved_limitations
            .contains(&fixed_offset_clock_limitation_ref()));
        assert!(!report
            .unresolved_limitations
            .contains(&forecast_actual_membership_limitation_ref()));
    }

    #[test]
    fn substituted_actual_fails_exact_membership() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let schedule = schedule(&registration);
        let mut cases = cases(&registration, &adapter, &file);
        cases[0].actual.value.mantissa += 1;
        assert!(matches!(
            evaluate_strong_hospitality_field(
                &registration,
                &schedule,
                &adapter,
                campaign.clone(),
                std::slice::from_ref(&file),
                cases,
                quality(&campaign),
                BTreeSet::new(),
            ),
            Err(StrongFieldError::Membership(_))
        ));
    }

    #[test]
    fn target_crossing_offset_transition_fails_closed() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let cases = cases(&registration, &adapter, &file);
        let first_start = cases[0].candidate.target.window_start_unix_ms;
        let transition = first_start + HOUR_MS / 2;
        let schedule = LocalTimeSchedule::build(
            registration.timezone.clone(),
            id("time-rules:test-transition:v1"),
            Digest32::repeat(8),
            registration.protocol.registered_at_unix_ms,
            EVAL_START,
            EVAL_END,
            vec![
                UtcOffsetPeriod {
                    start_unix_ms: EVAL_START,
                    end_unix_ms: transition,
                    utc_offset_minutes: 120,
                },
                UtcOffsetPeriod {
                    start_unix_ms: transition,
                    end_unix_ms: EVAL_END,
                    utc_offset_minutes: 60,
                },
            ],
        )
        .unwrap();
        assert!(matches!(
            evaluate_strong_hospitality_field(
                &registration,
                &schedule,
                &adapter,
                campaign.clone(),
                std::slice::from_ref(&file),
                cases,
                quality(&campaign),
                BTreeSet::new(),
            ),
            Err(StrongFieldError::TargetCrossesOffsetTransition)
        ));
    }

    #[test]
    fn time_rules_must_be_frozen_no_later_than_pilot_registration() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let late_schedule = LocalTimeSchedule::build(
            registration.timezone.clone(),
            id("time-rules:late:v1"),
            Digest32::repeat(8),
            registration.protocol.registered_at_unix_ms + 1,
            EVAL_START,
            EVAL_END,
            vec![UtcOffsetPeriod {
                start_unix_ms: EVAL_START,
                end_unix_ms: EVAL_END,
                utc_offset_minutes: 120,
            }],
        )
        .unwrap();
        assert!(matches!(
            evaluate_strong_hospitality_field(
                &registration,
                &late_schedule,
                &adapter,
                campaign.clone(),
                std::slice::from_ref(&file),
                cases(&registration, &adapter, &file),
                quality(&campaign),
                BTreeSet::new(),
            ),
            Err(StrongFieldError::TimeScheduleRegisteredAfterPilot)
        ));
    }

    #[test]
    fn retired_weaker_limitations_cannot_be_reintroduced() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let registration = registration(campaign.connector.clone());
        let schedule = schedule(&registration);
        let mut limitations = BTreeSet::new();
        limitations.insert(fixed_offset_clock_limitation_ref());
        assert!(matches!(
            evaluate_strong_hospitality_field(
                &registration,
                &schedule,
                &adapter,
                campaign.clone(),
                std::slice::from_ref(&file),
                cases(&registration, &adapter, &file),
                quality(&campaign),
                limitations,
            ),
            Err(StrongFieldError::RetiredLimitationPresent { .. })
        ));
    }
}
