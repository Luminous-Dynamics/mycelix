// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Preregistered, read-only hospitality pilot registration.
//!
//! This crate composes the generic shadow, field-qualification, ingress, and hospitality-profile
//! contracts. It introduces no execution path and grants no authority.

use mycelix_business_core::{CapabilityRef, Digest32, ReferenceId, ScopeRef};
use mycelix_business_field_qualification::{
    ConnectorBinding, DataQualityThreshold, EvaluationSlice, FieldQualificationPlan,
};
use mycelix_business_ingress::IngressQualificationBinding;
use mycelix_business_profile_hospitality::{
    DemandForecastProtocolConfig, demand_forecast_capability, demand_forecast_protocol,
    food_service_profile_ref, seasonal_naive_baseline_ref,
};
use mycelix_business_shadow::ForecastQualificationProtocol;
use sha2::{Digest, Sha256};

pub const HOSPITALITY_PILOT_IS_READ_ONLY: bool = true;
pub const DAY_MS: u64 = 86_400_000;
pub const HOUR_MS: u64 = 3_600_000;

fn id(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("static pilot semantic identifier is canonical")
}

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

pub fn sales_input_ref() -> ReferenceId {
    id("input:hospitality:sales-transactions:v1")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HospitalityPilotPolicy {
    pub minimum_preregistration_lead_ms: u64,
    pub minimum_evaluation_duration_ms: u64,
    pub minimum_total_forecast_cases: u32,
    pub minimum_cases_per_slice: u32,
    pub maximum_abstention_bps: u16,
    pub maximum_missing_bps: u16,
    pub maximum_conflicting_bps: u16,
    pub maximum_stale_bps: u16,
    pub maximum_ingest_delay_ms: u64,
}

impl HospitalityPilotPolicy {
    /// Conservative starting point for a manual/daily export shadow pilot.
    ///
    /// These are pilot defaults, not universal hospitality law. A different preregistered policy
    /// may be supplied before evidence collection starts.
    pub const fn conservative_manual_export_v1() -> Self {
        Self {
            minimum_preregistration_lead_ms: 24 * HOUR_MS,
            minimum_evaluation_duration_ms: 28 * DAY_MS,
            minimum_total_forecast_cases: 56,
            minimum_cases_per_slice: 10,
            maximum_abstention_bps: 1_000, // 10%
            maximum_missing_bps: 100,      // 1%
            maximum_conflicting_bps: 0,
            maximum_stale_bps: 200,        // 2%
            maximum_ingest_delay_ms: 36 * HOUR_MS,
        }
    }

    pub fn validate(&self) -> Result<(), PilotError> {
        if self.minimum_preregistration_lead_ms == 0
            || self.minimum_evaluation_duration_ms == 0
            || self.minimum_total_forecast_cases == 0
            || self.minimum_cases_per_slice == 0
            || self.maximum_ingest_delay_ms == 0
        {
            return Err(PilotError::InvalidPolicy);
        }
        if self.maximum_abstention_bps > 10_000
            || self.maximum_missing_bps > 10_000
            || self.maximum_conflicting_bps > 10_000
            || self.maximum_stale_bps > 10_000
        {
            return Err(PilotError::InvalidPolicy);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum HospitalitySliceKind {
    Breakfast,
    Lunch,
    Evening,
    Weekend,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HospitalitySliceDefinition {
    pub kind: HospitalitySliceKind,
    pub slice: ReferenceId,
    pub timezone: ReferenceId,
    /// Minutes since local midnight. `None` for non-daypart criteria such as Weekend.
    pub start_local_minute: Option<u16>,
    pub end_local_minute: Option<u16>,
    pub minimum_cases: u32,
    pub criterion_digest: Digest32,
}

impl HospitalitySliceDefinition {
    fn criterion_digest_for(
        kind: HospitalitySliceKind,
        timezone: &ReferenceId,
        start_local_minute: Option<u16>,
        end_local_minute: Option<u16>,
    ) -> Digest32 {
        let mut hasher = Sha256::new();
        hash_str(&mut hasher, "mycelix:hospitality-pilot-slice:v1");
        hash_str(&mut hasher, timezone.as_str());
        hasher.update([match kind {
            HospitalitySliceKind::Breakfast => 1,
            HospitalitySliceKind::Lunch => 2,
            HospitalitySliceKind::Evening => 3,
            HospitalitySliceKind::Weekend => 4,
        }]);
        match (start_local_minute, end_local_minute) {
            (Some(start), Some(end)) => {
                hasher.update([1]);
                hasher.update(start.to_be_bytes());
                hasher.update(end.to_be_bytes());
            }
            _ => hasher.update([0]),
        }
        finish_digest(hasher)
    }

    pub fn validate(&self) -> Result<(), PilotError> {
        if zero_digest(&self.criterion_digest) || self.minimum_cases == 0 {
            return Err(PilotError::InvalidSlice(self.slice.clone()));
        }
        match self.kind {
            HospitalitySliceKind::Weekend => {
                if self.start_local_minute.is_some() || self.end_local_minute.is_some() {
                    return Err(PilotError::InvalidSlice(self.slice.clone()));
                }
            }
            _ => {
                let (Some(start), Some(end)) = (self.start_local_minute, self.end_local_minute)
                else {
                    return Err(PilotError::InvalidSlice(self.slice.clone()));
                };
                if start >= end || end > 24 * 60 {
                    return Err(PilotError::InvalidSlice(self.slice.clone()));
                }
            }
        }
        let expected = Self::criterion_digest_for(
            self.kind,
            &self.timezone,
            self.start_local_minute,
            self.end_local_minute,
        );
        if self.criterion_digest != expected {
            return Err(PilotError::SliceDigestMismatch(self.slice.clone()));
        }
        Ok(())
    }
}

pub fn standard_daypart_slices_v1(
    timezone: ReferenceId,
    minimum_cases: u32,
) -> Vec<HospitalitySliceDefinition> {
    let daypart = |kind, name: &str, start, end| HospitalitySliceDefinition {
        kind,
        slice: id(name),
        timezone: timezone.clone(),
        start_local_minute: Some(start),
        end_local_minute: Some(end),
        minimum_cases,
        criterion_digest: HospitalitySliceDefinition::criterion_digest_for(
            kind,
            &timezone,
            Some(start),
            Some(end),
        ),
    };
    vec![
        daypart(
            HospitalitySliceKind::Breakfast,
            "slice:hospitality:breakfast:0500-1100:v1",
            5 * 60,
            11 * 60,
        ),
        daypart(
            HospitalitySliceKind::Lunch,
            "slice:hospitality:lunch:1100-1500:v1",
            11 * 60,
            15 * 60,
        ),
        daypart(
            HospitalitySliceKind::Evening,
            "slice:hospitality:evening:1500-2300:v1",
            15 * 60,
            23 * 60,
        ),
        HospitalitySliceDefinition {
            kind: HospitalitySliceKind::Weekend,
            slice: id("slice:hospitality:weekend:v1"),
            timezone: timezone.clone(),
            start_local_minute: None,
            end_local_minute: None,
            minimum_cases,
            criterion_digest: HospitalitySliceDefinition::criterion_digest_for(
                HospitalitySliceKind::Weekend,
                &timezone,
                None,
                None,
            ),
        },
    ]
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HospitalityForecastPilotConfig {
    pub plan_id: ReferenceId,
    pub scope: ScopeRef,
    pub timezone: ReferenceId,
    pub candidate_model_lineage: ReferenceId,
    pub connector: IngressQualificationBinding,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub policy: HospitalityPilotPolicy,
    pub slices: Option<Vec<HospitalitySliceDefinition>>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HospitalityForecastPilotRegistration {
    pub protocol: ForecastQualificationProtocol,
    pub field_plan: FieldQualificationPlan,
    /// Full ingress connector identity. This preserves source-schema binding that the generic
    /// field-plan connector projection intentionally does not own.
    pub ingress_connector: IngressQualificationBinding,
    pub timezone: ReferenceId,
    pub policy: HospitalityPilotPolicy,
    pub slices: Vec<HospitalitySliceDefinition>,
    pub registration_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PilotError {
    InvalidPolicy,
    InvalidWindow,
    InsufficientPreregistrationLead,
    EvaluationWindowTooShort,
    ZeroConnectorDigest,
    ZeroSourceSchemaDigest,
    InvalidSlice(ReferenceId),
    SliceDigestMismatch(ReferenceId),
    DuplicateSlice(ReferenceId),
    ProtocolInvalid,
    FieldPlanInvalid,
    ProfileMismatch,
    CapabilityMismatch,
    ScopeMismatch,
    ProtocolDigestMismatch,
    EvaluationWindowMismatch,
    ConnectorMismatch,
    DataQualityPolicyMismatch,
    SliceSetMismatch,
    RegistrationDigestMismatch,
}

impl HospitalityForecastPilotRegistration {
    pub fn build(config: HospitalityForecastPilotConfig) -> Result<Self, PilotError> {
        config.policy.validate()?;
        validate_window(&config)?;
        if zero_digest(&config.connector.adapter_digest)
            || zero_digest(&config.connector.mapping_digest)
        {
            return Err(PilotError::ZeroConnectorDigest);
        }
        if zero_digest(&config.connector.source_schema_digest) {
            return Err(PilotError::ZeroSourceSchemaDigest);
        }

        let slices = match config.slices.clone() {
            Some(slices) => slices,
            None => standard_daypart_slices_v1(
                config.timezone.clone(),
                config.policy.minimum_cases_per_slice,
            ),
        };
        validate_slices(&slices)?;

        let protocol_digest = protocol_digest(&config, &slices);
        let protocol = demand_forecast_protocol(DemandForecastProtocolConfig {
            candidate_model_lineage: config.candidate_model_lineage.clone(),
            registered_at_unix_ms: config.registered_at_unix_ms,
            evaluation_start_unix_ms: config.evaluation_start_unix_ms,
            evaluation_end_unix_ms: config.evaluation_end_unix_ms,
            minimum_cases: config.policy.minimum_total_forecast_cases,
            maximum_abstention_bps: config.policy.maximum_abstention_bps,
            require_candidate_not_worse_than_baseline: true,
            protocol_digest,
        });

        let field_connector = ConnectorBinding {
            source_system: config.connector.source_system.clone(),
            adapter_semantic_id: config.connector.adapter_semantic_id.clone(),
            adapter_digest: config.connector.adapter_digest,
            mapping_digest: config.connector.mapping_digest,
        };
        let data_quality = vec![DataQualityThreshold {
            input: sales_input_ref(),
            maximum_missing_bps: config.policy.maximum_missing_bps,
            maximum_conflicting_bps: config.policy.maximum_conflicting_bps,
            maximum_stale_bps: config.policy.maximum_stale_bps,
            maximum_ingest_delay_ms: config.policy.maximum_ingest_delay_ms,
        }];
        let field_slices = slices
            .iter()
            .map(|slice| EvaluationSlice {
                slice: slice.slice.clone(),
                criterion_digest: slice.criterion_digest,
                minimum_cases: slice.minimum_cases,
            })
            .collect::<Vec<_>>();

        let registration_digest = registration_digest(
            &config,
            protocol_digest,
            &data_quality,
            &slices,
        );
        let field_plan = FieldQualificationPlan {
            plan_id: config.plan_id,
            profile: food_service_profile_ref(),
            capability: demand_forecast_capability(),
            scope: config.scope,
            shadow_protocol_digest: protocol_digest,
            connectors: vec![field_connector],
            data_quality,
            slices: field_slices,
            registered_at_unix_ms: config.registered_at_unix_ms,
            evaluation_start_unix_ms: config.evaluation_start_unix_ms,
            evaluation_end_unix_ms: config.evaluation_end_unix_ms,
            // The field-plan digest is the stronger registration digest, which includes the
            // source-schema digest even though ConnectorBinding is a narrower projection.
            plan_digest: registration_digest,
        };

        let registration = Self {
            protocol,
            field_plan,
            ingress_connector: config.connector,
            timezone: config.timezone,
            policy: config.policy,
            slices,
            registration_digest,
        };
        registration.validate()?;
        Ok(registration)
    }

    pub fn validate(&self) -> Result<(), PilotError> {
        self.policy.validate()?;
        self.protocol.validate().map_err(|_| PilotError::ProtocolInvalid)?;
        self.field_plan
            .validate()
            .map_err(|_| PilotError::FieldPlanInvalid)?;
        validate_slices(&self.slices)?;

        if self.protocol.profile != food_service_profile_ref()
            || self.field_plan.profile != food_service_profile_ref()
        {
            return Err(PilotError::ProfileMismatch);
        }
        if self.protocol.capability != demand_forecast_capability()
            || self.field_plan.capability != demand_forecast_capability()
        {
            return Err(PilotError::CapabilityMismatch);
        }
        if self.protocol.protocol_digest != self.field_plan.shadow_protocol_digest {
            return Err(PilotError::ProtocolDigestMismatch);
        }
        if self.protocol.registered_at_unix_ms != self.field_plan.registered_at_unix_ms
            || self.protocol.evaluation_start_unix_ms != self.field_plan.evaluation_start_unix_ms
            || self.protocol.evaluation_end_unix_ms != self.field_plan.evaluation_end_unix_ms
        {
            return Err(PilotError::EvaluationWindowMismatch);
        }
        if self.field_plan.connectors.len() != 1 {
            return Err(PilotError::ConnectorMismatch);
        }
        let field_connector = &self.field_plan.connectors[0];
        if field_connector.source_system != self.ingress_connector.source_system
            || field_connector.adapter_semantic_id != self.ingress_connector.adapter_semantic_id
            || field_connector.adapter_digest != self.ingress_connector.adapter_digest
            || field_connector.mapping_digest != self.ingress_connector.mapping_digest
            || zero_digest(&self.ingress_connector.source_schema_digest)
        {
            return Err(PilotError::ConnectorMismatch);
        }
        if self.field_plan.data_quality.len() != 1 {
            return Err(PilotError::DataQualityPolicyMismatch);
        }
        let quality = &self.field_plan.data_quality[0];
        if quality.input != sales_input_ref()
            || quality.maximum_missing_bps != self.policy.maximum_missing_bps
            || quality.maximum_conflicting_bps != self.policy.maximum_conflicting_bps
            || quality.maximum_stale_bps != self.policy.maximum_stale_bps
            || quality.maximum_ingest_delay_ms != self.policy.maximum_ingest_delay_ms
        {
            return Err(PilotError::DataQualityPolicyMismatch);
        }
        let field_slices = self
            .field_plan
            .slices
            .iter()
            .map(|slice| (slice.slice.clone(), (slice.criterion_digest, slice.minimum_cases)))
            .collect::<std::collections::BTreeMap<_, _>>();
        let expected_slices = self
            .slices
            .iter()
            .map(|slice| (slice.slice.clone(), (slice.criterion_digest, slice.minimum_cases)))
            .collect::<std::collections::BTreeMap<_, _>>();
        if field_slices != expected_slices {
            return Err(PilotError::SliceSetMismatch);
        }

        let config = HospitalityForecastPilotConfig {
            plan_id: self.field_plan.plan_id.clone(),
            scope: self.field_plan.scope.clone(),
            timezone: self.timezone.clone(),
            candidate_model_lineage: self.protocol.candidate_model_lineage.clone(),
            connector: self.ingress_connector.clone(),
            registered_at_unix_ms: self.field_plan.registered_at_unix_ms,
            evaluation_start_unix_ms: self.field_plan.evaluation_start_unix_ms,
            evaluation_end_unix_ms: self.field_plan.evaluation_end_unix_ms,
            policy: self.policy.clone(),
            slices: Some(self.slices.clone()),
        };
        validate_window(&config)?;
        let expected_protocol_digest = protocol_digest(&config, &self.slices);
        if expected_protocol_digest != self.protocol.protocol_digest {
            return Err(PilotError::ProtocolDigestMismatch);
        }
        let expected_registration_digest = registration_digest(
            &config,
            self.protocol.protocol_digest,
            &self.field_plan.data_quality,
            &self.slices,
        );
        if self.registration_digest != expected_registration_digest
            || self.field_plan.plan_digest != expected_registration_digest
        {
            return Err(PilotError::RegistrationDigestMismatch);
        }
        Ok(())
    }

    pub fn scope(&self) -> &ScopeRef {
        &self.field_plan.scope
    }

    pub fn capability(&self) -> &CapabilityRef {
        &self.field_plan.capability
    }
}

fn validate_window(config: &HospitalityForecastPilotConfig) -> Result<(), PilotError> {
    if config.registered_at_unix_ms == 0
        || config.evaluation_start_unix_ms == 0
        || config.evaluation_start_unix_ms >= config.evaluation_end_unix_ms
        || config.registered_at_unix_ms >= config.evaluation_start_unix_ms
    {
        return Err(PilotError::InvalidWindow);
    }
    let lead = config
        .evaluation_start_unix_ms
        .saturating_sub(config.registered_at_unix_ms);
    if lead < config.policy.minimum_preregistration_lead_ms {
        return Err(PilotError::InsufficientPreregistrationLead);
    }
    let duration = config
        .evaluation_end_unix_ms
        .saturating_sub(config.evaluation_start_unix_ms);
    if duration < config.policy.minimum_evaluation_duration_ms {
        return Err(PilotError::EvaluationWindowTooShort);
    }
    Ok(())
}

fn validate_slices(slices: &[HospitalitySliceDefinition]) -> Result<(), PilotError> {
    if slices.is_empty() {
        return Err(PilotError::SliceSetMismatch);
    }
    let mut seen = std::collections::BTreeSet::new();
    for slice in slices {
        slice.validate()?;
        if !seen.insert(slice.slice.clone()) {
            return Err(PilotError::DuplicateSlice(slice.slice.clone()));
        }
    }
    Ok(())
}

fn protocol_digest(
    config: &HospitalityForecastPilotConfig,
    slices: &[HospitalitySliceDefinition],
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:hospitality-demand-shadow-protocol:v1");
    hash_str(&mut hasher, food_service_profile_ref().0.as_str());
    hash_str(&mut hasher, demand_forecast_capability().0.as_str());
    hash_str(&mut hasher, seasonal_naive_baseline_ref().as_str());
    hash_str(&mut hasher, config.candidate_model_lineage.as_str());
    hash_str(&mut hasher, config.timezone.as_str());
    hasher.update(config.registered_at_unix_ms.to_be_bytes());
    hasher.update(config.evaluation_start_unix_ms.to_be_bytes());
    hasher.update(config.evaluation_end_unix_ms.to_be_bytes());
    hasher.update(config.policy.minimum_total_forecast_cases.to_be_bytes());
    hasher.update(config.policy.maximum_abstention_bps.to_be_bytes());
    for slice in slices {
        hash_str(&mut hasher, slice.slice.as_str());
        hasher.update(slice.criterion_digest.0);
        hasher.update(slice.minimum_cases.to_be_bytes());
    }
    finish_digest(hasher)
}

fn registration_digest(
    config: &HospitalityForecastPilotConfig,
    protocol_digest: Digest32,
    quality: &[DataQualityThreshold],
    slices: &[HospitalitySliceDefinition],
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:hospitality-field-registration:v1");
    hash_str(&mut hasher, config.plan_id.as_str());
    hash_str(&mut hasher, config.scope.0.as_str());
    hasher.update(protocol_digest.0);
    hash_str(&mut hasher, config.connector.source_system.as_str());
    hash_str(&mut hasher, config.connector.adapter_semantic_id.as_str());
    hasher.update(config.connector.adapter_digest.0);
    hasher.update(config.connector.mapping_digest.0);
    hasher.update(config.connector.source_schema_digest.0);
    for threshold in quality {
        hash_str(&mut hasher, threshold.input.as_str());
        hasher.update(threshold.maximum_missing_bps.to_be_bytes());
        hasher.update(threshold.maximum_conflicting_bps.to_be_bytes());
        hasher.update(threshold.maximum_stale_bps.to_be_bytes());
        hasher.update(threshold.maximum_ingest_delay_ms.to_be_bytes());
    }
    for slice in slices {
        hash_str(&mut hasher, slice.slice.as_str());
        hasher.update(slice.criterion_digest.0);
        hasher.update(slice.minimum_cases.to_be_bytes());
    }
    hasher.update(config.registered_at_unix_ms.to_be_bytes());
    hasher.update(config.evaluation_start_unix_ms.to_be_bytes());
    hasher.update(config.evaluation_end_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn connector(schema_byte: u8) -> IngressQualificationBinding {
        IngressQualificationBinding {
            source_system: id("source:restaurant-pos-export"),
            adapter_semantic_id: id("adapter:delimited:hospitality-sales:v1"),
            adapter_digest: Digest32::repeat(1),
            mapping_digest: Digest32::repeat(2),
            source_schema_digest: Digest32::repeat(schema_byte),
        }
    }

    fn config() -> HospitalityForecastPilotConfig {
        HospitalityForecastPilotConfig {
            plan_id: id("pilot:hospitality:location-a:demand:v1"),
            scope: ScopeRef(id("scope:location:a")),
            timezone: id("timezone:iana:Africa/Johannesburg"),
            candidate_model_lineage: id("model:symthaea:hospitality-demand:v1"),
            connector: connector(3),
            registered_at_unix_ms: 10 * DAY_MS,
            evaluation_start_unix_ms: 12 * DAY_MS,
            evaluation_end_unix_ms: 40 * DAY_MS,
            policy: HospitalityPilotPolicy::conservative_manual_export_v1(),
            slices: None,
        }
    }

    #[test]
    fn builds_preregistered_read_only_forecast_pilot() {
        let registration = HospitalityForecastPilotRegistration::build(config()).unwrap();
        assert!(HOSPITALITY_PILOT_IS_READ_ONLY);
        assert_eq!(registration.validate(), Ok(()));
        assert_eq!(registration.protocol.baseline_model_lineage, seasonal_naive_baseline_ref());
        assert_eq!(registration.protocol.require_candidate_not_worse_than_baseline, true);
        assert_eq!(registration.slices.len(), 4);
        assert_eq!(registration.field_plan.plan_digest, registration.registration_digest);
    }

    #[test]
    fn source_schema_identity_is_part_of_registration_digest() {
        let first = HospitalityForecastPilotRegistration::build(config()).unwrap();
        let mut changed = config();
        changed.connector = connector(4);
        let second = HospitalityForecastPilotRegistration::build(changed).unwrap();
        assert_ne!(first.registration_digest, second.registration_digest);
    }

    #[test]
    fn schema_substitution_after_registration_fails_closed() {
        let mut registration = HospitalityForecastPilotRegistration::build(config()).unwrap();
        registration.ingress_connector.source_schema_digest = Digest32::repeat(9);
        assert_eq!(
            registration.validate(),
            Err(PilotError::RegistrationDigestMismatch)
        );
    }

    #[test]
    fn evaluation_cannot_start_without_preregistration_lead() {
        let mut value = config();
        value.registered_at_unix_ms = value.evaluation_start_unix_ms - HOUR_MS;
        assert_eq!(
            HospitalityForecastPilotRegistration::build(value),
            Err(PilotError::InsufficientPreregistrationLead)
        );
    }

    #[test]
    fn short_holdout_is_rejected() {
        let mut value = config();
        value.evaluation_end_unix_ms = value.evaluation_start_unix_ms + 7 * DAY_MS;
        assert_eq!(
            HospitalityForecastPilotRegistration::build(value),
            Err(PilotError::EvaluationWindowTooShort)
        );
    }

    #[test]
    fn timezone_changes_slice_and_registration_evidence() {
        let first = HospitalityForecastPilotRegistration::build(config()).unwrap();
        let mut changed = config();
        changed.timezone = id("timezone:iana:Europe/London");
        let second = HospitalityForecastPilotRegistration::build(changed).unwrap();
        assert_ne!(
            first.slices[0].criterion_digest,
            second.slices[0].criterion_digest
        );
        assert_ne!(first.registration_digest, second.registration_digest);
    }

    #[test]
    fn conservative_policy_requires_zero_conflicting_records() {
        let registration = HospitalityForecastPilotRegistration::build(config()).unwrap();
        assert_eq!(registration.field_plan.data_quality[0].maximum_conflicting_bps, 0);
        assert_eq!(registration.field_plan.data_quality[0].maximum_missing_bps, 100);
    }
}
