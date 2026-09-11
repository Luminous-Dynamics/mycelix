// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Protocol-bound, derived qualification evidence for read-only hospitality pilots.
//!
//! This layer does not accept caller-supplied slice pass/fail booleans. It validates exact
//! forecast cases against the preregistered hospitality protocol, derives deterministic local-time
//! slice membership from a preregistered fixed UTC offset, scores each slice, and only then builds
//! the generic field-evidence representation consumed by the lower-level qualification gate.

use std::collections::BTreeSet;

use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_field_qualification::{
    DataQualityEvidence, FieldQualificationDecision, FieldQualificationEvidence, SliceEvidence,
};
use mycelix_business_import_diagnostics::{
    ExtractionBoundHospitalityEvidence, ExtractionBoundPilotError, ExtractionCampaignManifest,
    evaluate_extraction_bound_hospitality_pilot, upstream_export_completeness_unverified_ref,
};
use mycelix_business_pilot_hospitality::{
    DAY_MS, HospitalityForecastPilotRegistration, HospitalityPilotEvidence,
    HospitalitySliceDefinition, HospitalitySliceKind,
};
use mycelix_business_shadow::{
    ForecastCase, ForecastDisposition, ForecastScorecard, ScaledValue, ShadowForecast,
    ShadowPromotionDecision, evaluate_forecast_gate,
};
use sha2::{Digest, Sha256};

pub const DERIVED_EVIDENCE_IS_READ_ONLY: bool = true;
const MINUTES_PER_DAY: i64 = 24 * 60;
const MILLIS_PER_MINUTE: i128 = 60_000;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    let bytes: [u8; 32] = hasher.finalize().into();
    Digest32(bytes)
}

pub fn fixed_offset_clock_limitation_ref() -> ReferenceId {
    ReferenceId::new("limitation:fixed-offset-clock-rules:v1")
        .expect("static limitation id is canonical")
}

pub fn forecast_actual_campaign_membership_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:forecast-actual-campaign-membership-unverified:v1")
        .expect("static limitation id is canonical")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FixedOffsetSlicePlan {
    pub registration_digest: Digest32,
    pub timezone: ReferenceId,
    pub utc_offset_minutes: i16,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub slice_bindings: Vec<(ReferenceId, Digest32)>,
    pub plan_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClockPlanError {
    RegistrationInvalid,
    InvalidOffset,
    NotPreregistered,
    TimezoneMismatch,
    SliceSetMismatch,
    DigestMismatch,
}

impl FixedOffsetSlicePlan {
    pub fn build(
        registration: &HospitalityForecastPilotRegistration,
        registered_at_unix_ms: u64,
        utc_offset_minutes: i16,
    ) -> Result<Self, ClockPlanError> {
        registration
            .validate()
            .map_err(|_| ClockPlanError::RegistrationInvalid)?;
        if !(-14 * 60..=14 * 60).contains(&i32::from(utc_offset_minutes)) {
            return Err(ClockPlanError::InvalidOffset);
        }
        if registered_at_unix_ms == 0
            || registered_at_unix_ms >= registration.protocol.evaluation_start_unix_ms
        {
            return Err(ClockPlanError::NotPreregistered);
        }
        let slice_bindings = registration
            .slices
            .iter()
            .map(|slice| (slice.slice.clone(), slice.criterion_digest))
            .collect::<Vec<_>>();
        let mut plan = Self {
            registration_digest: registration.registration_digest,
            timezone: registration.timezone.clone(),
            utc_offset_minutes,
            registered_at_unix_ms,
            evaluation_start_unix_ms: registration.protocol.evaluation_start_unix_ms,
            evaluation_end_unix_ms: registration.protocol.evaluation_end_unix_ms,
            slice_bindings,
            plan_digest: Digest32([0; 32]),
        };
        plan.plan_digest = compute_clock_plan_digest(&plan);
        plan.validate_against(registration)?;
        Ok(plan)
    }

    pub fn validate_against(
        &self,
        registration: &HospitalityForecastPilotRegistration,
    ) -> Result<(), ClockPlanError> {
        registration
            .validate()
            .map_err(|_| ClockPlanError::RegistrationInvalid)?;
        if !(-14 * 60..=14 * 60).contains(&i32::from(self.utc_offset_minutes)) {
            return Err(ClockPlanError::InvalidOffset);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.evaluation_start_unix_ms
        {
            return Err(ClockPlanError::NotPreregistered);
        }
        if self.registration_digest != registration.registration_digest
            || self.evaluation_start_unix_ms != registration.protocol.evaluation_start_unix_ms
            || self.evaluation_end_unix_ms != registration.protocol.evaluation_end_unix_ms
        {
            return Err(ClockPlanError::RegistrationInvalid);
        }
        if self.timezone != registration.timezone {
            return Err(ClockPlanError::TimezoneMismatch);
        }
        let expected = registration
            .slices
            .iter()
            .map(|slice| (slice.slice.clone(), slice.criterion_digest))
            .collect::<Vec<_>>();
        if self.slice_bindings != expected {
            return Err(ClockPlanError::SliceSetMismatch);
        }
        if zero_digest(&self.plan_digest) || self.plan_digest != compute_clock_plan_digest(self) {
            return Err(ClockPlanError::DigestMismatch);
        }
        Ok(())
    }
}

fn compute_clock_plan_digest(plan: &FixedOffsetSlicePlan) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:hospitality-fixed-offset-clock:v1");
    hasher.update(plan.registration_digest.0);
    hash_str(&mut hasher, plan.timezone.as_str());
    hasher.update(plan.utc_offset_minutes.to_be_bytes());
    hasher.update(plan.registered_at_unix_ms.to_be_bytes());
    hasher.update(plan.evaluation_start_unix_ms.to_be_bytes());
    hasher.update(plan.evaluation_end_unix_ms.to_be_bytes());
    for (slice, digest) in &plan.slice_bindings {
        hash_str(&mut hasher, slice.as_str());
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedSliceReport {
    pub slice: ReferenceId,
    pub criterion_digest: Digest32,
    pub case_set_digest: Digest32,
    pub scorecard: ForecastScorecard,
    pub shadow_gate_passed: bool,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DerivedQualificationDecision {
    OverallShadowFailed(ShadowPromotionDecision),
    Field(FieldQualificationDecision),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedHospitalityReport {
    pub decision: DerivedQualificationDecision,
    pub overall_scorecard: ForecastScorecard,
    pub overall_case_set_digest: Digest32,
    pub slices: Vec<DerivedSliceReport>,
    pub clock_plan_digest: Digest32,
    pub field_evidence_digest: Digest32,
}

#[derive(Debug)]
pub enum DerivedEvidenceError {
    RegistrationInvalid,
    Clock(ClockPlanError),
    CampaignInvalid,
    EmptyCaseSet,
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
    ScoringFailed,
    ProtocolGateFailed,
    ClockArithmetic,
    TooManyCases,
    Extraction(ExtractionBoundPilotError),
}

pub fn derive_and_evaluate_hospitality_pilot(
    registration: &HospitalityForecastPilotRegistration,
    clock: &FixedOffsetSlicePlan,
    campaign: ExtractionCampaignManifest,
    cases: Vec<ForecastCase>,
    data_quality: Vec<DataQualityEvidence>,
    mut known_limitations: BTreeSet<ReferenceId>,
) -> Result<DerivedHospitalityReport, DerivedEvidenceError> {
    registration
        .validate()
        .map_err(|_| DerivedEvidenceError::RegistrationInvalid)?;
    clock
        .validate_against(registration)
        .map_err(DerivedEvidenceError::Clock)?;
    campaign
        .validate()
        .map_err(|_| DerivedEvidenceError::CampaignInvalid)?;
    if cases.is_empty() {
        return Err(DerivedEvidenceError::EmptyCaseSet);
    }
    validate_cases(registration, &cases)?;

    let overall_scorecard = ForecastScorecard::score(&cases)
        .map_err(|_| DerivedEvidenceError::ScoringFailed)?;
    let overall_case_set_digest = case_set_digest(&cases);
    let overall_decision = evaluate_forecast_gate(&registration.protocol, &overall_scorecard)
        .map_err(|_| DerivedEvidenceError::ProtocolGateFailed)?;

    let mut slice_reports = Vec::with_capacity(registration.slices.len());
    let mut slice_evidence = Vec::with_capacity(registration.slices.len());
    for slice in &registration.slices {
        let selected = cases
            .iter()
            .filter(|case| case_belongs_to_slice(case, slice, clock).unwrap_or(false))
            .cloned()
            .collect::<Vec<_>>();
        let scorecard = ForecastScorecard::score(&selected)
            .map_err(|_| DerivedEvidenceError::ScoringFailed)?;
        let selected_digest = case_set_digest(&selected);
        let passed = scorecard.cases >= slice.minimum_cases
            && scorecard.abstention_bps() <= registration.protocol.maximum_abstention_bps
            && (!registration.protocol.require_candidate_not_worse_than_baseline
                || scorecard.candidate_not_worse_than_baseline());
        let evidence_digest = slice_evidence_digest(
            clock.plan_digest,
            slice,
            selected_digest,
            &scorecard,
            passed,
        );
        slice_reports.push(DerivedSliceReport {
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

    known_limitations.insert(upstream_export_completeness_unverified_ref());
    known_limitations.insert(fixed_offset_clock_limitation_ref());
    known_limitations.insert(forecast_actual_campaign_membership_unverified_ref());
    let field_evidence_digest = field_evidence_digest(
        registration.registration_digest,
        campaign.campaign_digest,
        clock.plan_digest,
        overall_case_set_digest,
        &overall_scorecard,
        &data_quality,
        &slice_evidence,
        &known_limitations,
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
        known_limitations,
        evidence_digest: field_evidence_digest,
    };

    let decision = if overall_decision != ShadowPromotionDecision::PassShadowGate {
        DerivedQualificationDecision::OverallShadowFailed(overall_decision)
    } else {
        let pilot_evidence = HospitalityPilotEvidence {
            registration_digest: registration.registration_digest,
            ingress_connector: registration.ingress_connector.clone(),
            field_evidence,
        };
        let extraction_evidence = ExtractionBoundHospitalityEvidence {
            campaign,
            pilot_evidence,
        };
        let field_decision = evaluate_extraction_bound_hospitality_pilot(
            registration,
            &extraction_evidence,
        )
        .map_err(DerivedEvidenceError::Extraction)?;
        DerivedQualificationDecision::Field(field_decision)
    };

    Ok(DerivedHospitalityReport {
        decision,
        overall_scorecard,
        overall_case_set_digest,
        slices: slice_reports,
        clock_plan_digest: clock.plan_digest,
        field_evidence_digest,
    })
}

fn validate_cases(
    registration: &HospitalityForecastPilotRegistration,
    cases: &[ForecastCase],
) -> Result<(), DerivedEvidenceError> {
    if cases.len() > u32::MAX as usize {
        return Err(DerivedEvidenceError::TooManyCases);
    }
    let mut candidate_ids = BTreeSet::new();
    let mut baseline_ids = BTreeSet::new();
    let mut actual_ids = BTreeSet::new();
    let mut source_events = BTreeSet::new();
    let mut targets = BTreeSet::new();

    for (index, case) in cases.iter().enumerate() {
        case.validate()
            .map_err(|_| DerivedEvidenceError::InvalidCase { index })?;
        if case.candidate.model_lineage != registration.protocol.candidate_model_lineage {
            return Err(DerivedEvidenceError::CandidateLineageMismatch { index });
        }
        if case.baseline.model_lineage != registration.protocol.baseline_model_lineage {
            return Err(DerivedEvidenceError::BaselineLineageMismatch { index });
        }
        let target = &case.candidate.target;
        if target.window_start_unix_ms < registration.protocol.evaluation_start_unix_ms
            || target.window_end_unix_ms > registration.protocol.evaluation_end_unix_ms
        {
            return Err(DerivedEvidenceError::CaseOutsideEvaluationWindow { index });
        }
        if case.candidate.issued_at_unix_ms < registration.protocol.registered_at_unix_ms
            || case.baseline.issued_at_unix_ms < registration.protocol.registered_at_unix_ms
        {
            return Err(DerivedEvidenceError::ForecastPredatesProtocol { index });
        }
        if target.scope != registration.field_plan.scope {
            return Err(DerivedEvidenceError::ScopeMismatch { index });
        }
        if case.actual.source_system != registration.ingress_connector.source_system {
            return Err(DerivedEvidenceError::ActualSourceMismatch { index });
        }
        if case.actual.mapping_digest != registration.ingress_connector.mapping_digest {
            return Err(DerivedEvidenceError::ActualMappingMismatch { index });
        }
        if !candidate_ids.insert(case.candidate.forecast.as_ref_id().clone()) {
            return Err(DerivedEvidenceError::DuplicateCandidateForecast);
        }
        if !baseline_ids.insert(case.baseline.forecast.as_ref_id().clone()) {
            return Err(DerivedEvidenceError::DuplicateBaselineForecast);
        }
        if !actual_ids.insert(case.actual.observation.as_ref_id().clone()) {
            return Err(DerivedEvidenceError::DuplicateActualObservation);
        }
        if !source_events.insert(case.actual.source_event_id.clone()) {
            return Err(DerivedEvidenceError::DuplicateSourceEvent);
        }
        let target_identity = (
            target.metric.clone(),
            target.scope.clone(),
            target.window_start_unix_ms,
            target.window_end_unix_ms,
        );
        if !targets.insert(target_identity) {
            return Err(DerivedEvidenceError::DuplicateTargetWindow);
        }
    }
    Ok(())
}

fn case_belongs_to_slice(
    case: &ForecastCase,
    slice: &HospitalitySliceDefinition,
    clock: &FixedOffsetSlicePlan,
) -> Result<bool, DerivedEvidenceError> {
    let target = &case.candidate.target;
    let (start_day, start_minute, start_weekday) =
        local_parts(target.window_start_unix_ms, clock.utc_offset_minutes)?;
    let end_inclusive = target
        .window_end_unix_ms
        .checked_sub(1)
        .ok_or(DerivedEvidenceError::ClockArithmetic)?;
    let (end_day, end_minute, end_weekday) =
        local_parts(end_inclusive, clock.utc_offset_minutes)?;
    match slice.kind {
        HospitalitySliceKind::Weekend => Ok(start_weekday >= 5 && end_weekday >= 5),
        _ => {
            let (Some(start), Some(end)) = (slice.start_local_minute, slice.end_local_minute)
            else {
                return Ok(false);
            };
            Ok(start_day == end_day
                && start_minute >= start
                && end_minute < end)
        }
    }
}

fn local_parts(
    unix_ms: u64,
    utc_offset_minutes: i16,
) -> Result<(i128, u16, u8), DerivedEvidenceError> {
    let shifted = i128::from(unix_ms)
        .checked_add(i128::from(utc_offset_minutes) * MILLIS_PER_MINUTE)
        .ok_or(DerivedEvidenceError::ClockArithmetic)?;
    if shifted < 0 {
        return Err(DerivedEvidenceError::ClockArithmetic);
    }
    let day_ms = i128::from(DAY_MS);
    let local_day = shifted.div_euclid(day_ms);
    let within_day_ms = shifted.rem_euclid(day_ms);
    let minute = u16::try_from(within_day_ms / MILLIS_PER_MINUTE)
        .map_err(|_| DerivedEvidenceError::ClockArithmetic)?;
    if i64::from(minute) >= MINUTES_PER_DAY {
        return Err(DerivedEvidenceError::ClockArithmetic);
    }
    // 1970-01-01 was Thursday. With Monday=0, Thursday=3.
    let weekday = u8::try_from((local_day + 3).rem_euclid(7))
        .map_err(|_| DerivedEvidenceError::ClockArithmetic)?;
    Ok((local_day, minute, weekday))
}

fn compute_case_digest(case: &ForecastCase) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:hospitality-forecast-case:v1");
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
    hash_str(&mut hasher, "mycelix:hospitality-forecast-case-set:v1");
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
    clock_plan_digest: Digest32,
    slice: &HospitalitySliceDefinition,
    case_set_digest: Digest32,
    scorecard: &ForecastScorecard,
    passed: bool,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:hospitality-derived-slice-evidence:v1");
    hasher.update(clock_plan_digest.0);
    hash_str(&mut hasher, slice.slice.as_str());
    hasher.update(slice.criterion_digest.0);
    hasher.update(case_set_digest.0);
    hash_scorecard(&mut hasher, scorecard);
    hasher.update([u8::from(passed)]);
    finish_digest(hasher)
}

fn field_evidence_digest(
    registration_digest: Digest32,
    campaign_digest: Digest32,
    clock_digest: Digest32,
    case_set_digest: Digest32,
    overall_scorecard: &ForecastScorecard,
    data_quality: &[DataQualityEvidence],
    slices: &[SliceEvidence],
    limitations: &BTreeSet<ReferenceId>,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:hospitality-derived-field-evidence:v1");
    hasher.update(registration_digest.0);
    hasher.update(campaign_digest.0);
    hasher.update(clock_digest.0);
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

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, DelimitedIngressAdapter, OutputMapping, ScopeMapping,
        TimestampEncoding, ValueMapping,
    };
    use mycelix_business_core::{Digest32, ForecastRef, ObservationRef, ReferenceId, ScopeRef};
    use mycelix_business_field_qualification::DataQualityEvidence;
    use mycelix_business_import_diagnostics::{
        ExtractionCampaignManifest, diagnose_delimited_import,
    };
    use mycelix_business_pilot_hospitality::{
        HospitalityForecastPilotConfig, HospitalityPilotPolicy, HOUR_MS, standard_daypart_slices_v1,
        sales_input_ref,
    };
    use mycelix_business_shadow::{
        ForecastDisposition, ForecastTarget, ForecastValue, MetricObservation, ScaledValue,
        ShadowForecast,
    };

    const EVAL_START: u64 = 1_788_739_200_000;
    const EVAL_END: u64 = 1_789_344_000_000;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn connector_and_campaign() -> (mycelix_business_ingress::IngressQualificationBinding, ExtractionCampaignManifest) {
        let adapter = DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:test:v1"),
            delimiter: b',',
            expected_headers: vec!["event_id".into(), "occurred_at".into(), "location".into(), "quantity".into()],
            source_event_id_column: "event_id".into(),
            observed_at_column: "occurred_at".into(),
            timestamp_encoding: TimestampEncoding::UnixMilliseconds,
            scope: ScopeMapping::Column { column: "location".into(), prefix: "scope:location:".into() },
            outputs: vec![OutputMapping {
                input: sales_input_ref(),
                metric: id("metric:hospitality:item-demand"),
                value: ValueMapping::Column { column: "quantity".into(), decimal: DecimalPolicy::default() },
                unit: id("unit:count"),
                scale: 0,
            }],
            maximum_batch_records: 100,
        }).unwrap();
        let csv = b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\nevent:2,1100,a,2\nevent:3,1200,a,3\nevent:4,1300,a,4\n";
        let manifest = diagnose_delimited_import(&adapter, csv.as_slice(), 2_000).unwrap();
        let connector = manifest.connector.clone();
        let campaign = ExtractionCampaignManifest::from_files(vec![manifest]).unwrap();
        (connector, campaign)
    }

    fn registration(connector: mycelix_business_ingress::IngressQualificationBinding) -> HospitalityForecastPilotRegistration {
        HospitalityForecastPilotRegistration::build(HospitalityForecastPilotConfig {
            plan_id: id("pilot:derived-test:v1"),
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
            slices: Some(standard_daypart_slices_v1(id("timezone:iana:Africa/Johannesburg"), 1)),
        }).unwrap()
    }

    fn case(
        registration: &HospitalityForecastPilotRegistration,
        name: &str,
        start: u64,
        actual_value: i128,
        candidate_value: i128,
    ) -> ForecastCase {
        let unit = id("unit:count");
        let target = ForecastTarget {
            metric: id("metric:hospitality:item-demand"),
            scope: registration.field_plan.scope.clone(),
            unit: unit.clone(),
            scale: 0,
            window_start_unix_ms: start,
            window_end_unix_ms: start + HOUR_MS,
        };
        let value = |point: i128| ForecastValue {
            point: ScaledValue { mantissa: point, scale: 0, unit: unit.clone() },
            lower: ScaledValue { mantissa: point.saturating_sub(1), scale: 0, unit: unit.clone() },
            upper: ScaledValue { mantissa: point.saturating_add(1), scale: 0, unit: unit.clone() },
        };
        ForecastCase {
            candidate: ShadowForecast {
                forecast: ForecastRef(id(&format!("forecast:candidate:{name}"))),
                model_lineage: registration.protocol.candidate_model_lineage.clone(),
                target: target.clone(),
                issued_at_unix_ms: start - HOUR_MS,
                disposition: ForecastDisposition::Predicted(value(candidate_value)),
            },
            baseline: ShadowForecast {
                forecast: ForecastRef(id(&format!("forecast:baseline:{name}"))),
                model_lineage: registration.protocol.baseline_model_lineage.clone(),
                target,
                issued_at_unix_ms: start - HOUR_MS,
                disposition: ForecastDisposition::Predicted(value(actual_value + 5)),
            },
            actual: MetricObservation {
                observation: ObservationRef(id(&format!("observation:{name}"))),
                source_system: registration.ingress_connector.source_system.clone(),
                source_event_id: id(&format!("actual-event:{name}")),
                source_payload_digest: Digest32::repeat(name.as_bytes()[0]),
                mapping_digest: registration.ingress_connector.mapping_digest,
                metric: id("metric:hospitality:item-demand"),
                scope: registration.field_plan.scope.clone(),
                value: ScaledValue { mantissa: actual_value, scale: 0, unit },
                observed_at_unix_ms: start + HOUR_MS + 1,
            },
        }
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
    fn derives_daypart_and_weekend_gates_from_cases() {
        let (connector, campaign) = connector_and_campaign();
        let registration = registration(connector);
        let clock = FixedOffsetSlicePlan::build(&registration, EVAL_START - DAY_MS, 120).unwrap();
        let cases = vec![
            case(&registration, "breakfast", 1_788_760_800_000, 10, 10),
            case(&registration, "lunch", 1_788_775_200_000, 20, 20),
            case(&registration, "evening", 1_788_796_800_000, 30, 30),
            case(&registration, "weekend", 1_789_192_800_000, 40, 40),
        ];
        let report = derive_and_evaluate_hospitality_pilot(
            &registration,
            &clock,
            campaign.clone(),
            cases,
            quality(&campaign),
            BTreeSet::new(),
        ).unwrap();
        assert!(DERIVED_EVIDENCE_IS_READ_ONLY);
        assert_eq!(report.decision, DerivedQualificationDecision::Field(FieldQualificationDecision::PassShadowFieldGate));
        assert!(report.slices.iter().all(|slice| slice.shadow_gate_passed));
        let weekend = report.slices.iter().find(|slice| slice.slice.as_str().contains("weekend")).unwrap();
        assert_eq!(weekend.scorecard.cases, 1);
    }

    #[test]
    fn wrong_candidate_lineage_fails_structurally() {
        let (connector, campaign) = connector_and_campaign();
        let registration = registration(connector);
        let clock = FixedOffsetSlicePlan::build(&registration, EVAL_START - DAY_MS, 120).unwrap();
        let mut cases = vec![case(&registration, "bad-lineage", 1_788_760_800_000, 10, 10)];
        cases[0].candidate.model_lineage = id("model:wrong");
        assert!(matches!(
            derive_and_evaluate_hospitality_pilot(&registration, &clock, campaign.clone(), cases, quality(&campaign), BTreeSet::new()),
            Err(DerivedEvidenceError::CandidateLineageMismatch { .. })
        ));
    }

    #[test]
    fn duplicate_target_cannot_gain_extra_weight() {
        let (connector, campaign) = connector_and_campaign();
        let registration = registration(connector);
        let clock = FixedOffsetSlicePlan::build(&registration, EVAL_START - DAY_MS, 120).unwrap();
        let a = case(&registration, "a", 1_788_760_800_000, 10, 10);
        let mut b = case(&registration, "b", 1_788_760_800_000, 10, 10);
        b.actual.source_event_id = id("actual-event:b-unique");
        assert!(matches!(
            derive_and_evaluate_hospitality_pilot(&registration, &clock, campaign.clone(), vec![a, b], quality(&campaign), BTreeSet::new()),
            Err(DerivedEvidenceError::DuplicateTargetWindow)
        ));
    }

    #[test]
    fn weak_breakfast_slice_is_derived_as_failure() {
        let (connector, campaign) = connector_and_campaign();
        let registration = registration(connector);
        let clock = FixedOffsetSlicePlan::build(&registration, EVAL_START - DAY_MS, 120).unwrap();
        let cases = vec![
            case(&registration, "breakfast-bad", 1_788_760_800_000, 10, 100),
            case(&registration, "lunch-good", 1_788_775_200_000, 20, 20),
        ];
        let report = derive_and_evaluate_hospitality_pilot(
            &registration,
            &clock,
            campaign.clone(),
            cases,
            quality(&campaign),
            BTreeSet::new(),
        ).unwrap();
        let breakfast = report.slices.iter().find(|slice| slice.slice.as_str().contains("breakfast")).unwrap();
        assert!(!breakfast.shadow_gate_passed);
        assert!(matches!(
            report.decision,
            DerivedQualificationDecision::Field(FieldQualificationDecision::SliceShadowGateFailed { .. })
                | DerivedQualificationDecision::OverallShadowFailed(_)
        ));
    }

    #[test]
    fn clock_rules_must_be_preregistered() {
        let (connector, _) = connector_and_campaign();
        let registration = registration(connector);
        assert_eq!(
            FixedOffsetSlicePlan::build(&registration, EVAL_START, 120),
            Err(ClockPlanError::NotPreregistered)
        );
    }
}
