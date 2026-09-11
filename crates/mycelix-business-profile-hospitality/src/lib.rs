// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Brand-agnostic hospitality shadow profile for the Mycelix Business Fabric.
//!
//! This crate is a preset and qualification contract. It owns no restaurant business state
//! and cannot express autonomous execution.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    BusinessDescriptor, CapabilityRef, Digest32, ProfileRef, ReferenceId, RoleRef,
};
use mycelix_business_shadow::{
    ForecastQualificationProtocol, RecommendationQualificationProtocol, ShadowCapability,
    ShadowMode,
};

fn id(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("static hospitality semantic identifiers are canonical")
}

fn capability(value: &str) -> CapabilityRef {
    CapabilityRef(id(value))
}

fn role(value: &str) -> RoleRef {
    RoleRef(id(value))
}

pub fn food_service_profile_ref() -> ProfileRef {
    ProfileRef(id("profile:hospitality:food-service:v1"))
}

pub fn demand_forecast_capability() -> CapabilityRef {
    capability("forecast:hospitality:demand:v1")
}

pub fn prep_recommendation_capability() -> CapabilityRef {
    capability("recommend:hospitality:prep:v1")
}

pub fn replenishment_recommendation_capability() -> CapabilityRef {
    capability("recommend:hospitality:replenishment:v1")
}

pub fn waste_review_capability() -> CapabilityRef {
    capability("recommend:hospitality:waste-review:v1")
}

pub fn demand_forecast_lane_ref() -> ReferenceId {
    id("lane:hospitality:demand-forecast:v1")
}

pub fn prep_recommendation_lane_ref() -> ReferenceId {
    id("lane:hospitality:prep-recommendation:v1")
}

pub fn replenishment_recommendation_lane_ref() -> ReferenceId {
    id("lane:hospitality:replenishment-recommendation:v1")
}

pub fn waste_review_lane_ref() -> ReferenceId {
    id("lane:hospitality:waste-review:v1")
}

pub fn seasonal_naive_baseline_ref() -> ReferenceId {
    id("baseline:hospitality:seasonal-naive:same-weekday-daypart:v1")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputRequirement {
    pub input: ReferenceId,
    pub description: &'static str,
    pub privacy_minimal: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualificationLane {
    pub lane: ReferenceId,
    pub capability: CapabilityRef,
    pub mode: ShadowMode,
    pub required_inputs: BTreeSet<ReferenceId>,
    pub baseline: Option<ReferenceId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HospitalityShadowProfile {
    pub descriptor: BusinessDescriptor,
    pub shadow_capabilities: Vec<ShadowCapability>,
    pub inputs: BTreeMap<ReferenceId, InputRequirement>,
    pub lanes: BTreeMap<ReferenceId, QualificationLane>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HospitalityProfileError {
    ProfileIdentityMismatch,
    DuplicateShadowCapability,
    DescriptorCapabilityMismatch,
    LaneCapabilityMissing { lane: ReferenceId },
    LaneModeMismatch { lane: ReferenceId },
    MissingInputDefinition { lane: ReferenceId, input: ReferenceId },
    DemandLaneMissingBaseline,
    RecommendationLaneHasBaseline { lane: ReferenceId },
}

impl HospitalityShadowProfile {
    pub fn validate(&self) -> Result<(), HospitalityProfileError> {
        if self.descriptor.profile != food_service_profile_ref() {
            return Err(HospitalityProfileError::ProfileIdentityMismatch);
        }

        let mut shadow_by_capability = BTreeMap::new();
        for value in &self.shadow_capabilities {
            if shadow_by_capability
                .insert(value.capability.clone(), value.mode)
                .is_some()
            {
                return Err(HospitalityProfileError::DuplicateShadowCapability);
            }
        }

        let declared = shadow_by_capability.keys().cloned().collect::<BTreeSet<_>>();
        if declared != self.descriptor.capabilities {
            return Err(HospitalityProfileError::DescriptorCapabilityMismatch);
        }

        for (lane_ref, lane) in &self.lanes {
            let Some(mode) = shadow_by_capability.get(&lane.capability) else {
                return Err(HospitalityProfileError::LaneCapabilityMissing {
                    lane: lane_ref.clone(),
                });
            };
            if mode != &lane.mode {
                return Err(HospitalityProfileError::LaneModeMismatch {
                    lane: lane_ref.clone(),
                });
            }
            for input in &lane.required_inputs {
                if !self.inputs.contains_key(input) {
                    return Err(HospitalityProfileError::MissingInputDefinition {
                        lane: lane_ref.clone(),
                        input: input.clone(),
                    });
                }
            }
            if lane.lane == demand_forecast_lane_ref() && lane.baseline.is_none() {
                return Err(HospitalityProfileError::DemandLaneMissingBaseline);
            }
            if lane.mode == ShadowMode::Recommend && lane.baseline.is_some() {
                return Err(HospitalityProfileError::RecommendationLaneHasBaseline {
                    lane: lane_ref.clone(),
                });
            }
        }

        Ok(())
    }

    pub fn lane(&self, lane: &ReferenceId) -> Option<&QualificationLane> {
        self.lanes.get(lane)
    }
}

pub fn food_service_shadow_profile() -> HospitalityShadowProfile {
    let sales = id("input:hospitality:sales-transactions:v1");
    let inventory = id("input:hospitality:inventory-observations:v1");
    let recipes = id("input:hospitality:menu-recipe-mapping:v1");
    let supplier_lead = id("input:hospitality:supplier-lead-time:v1");
    let waste = id("input:hospitality:waste-observations:v1");
    let calendar = id("input:hospitality:operating-calendar:v1");
    let capacity = id("input:hospitality:prep-capacity:v1");
    let demand_forecast = id("input:hospitality:qualified-demand-forecast:v1");

    let mut inputs = BTreeMap::new();
    for requirement in [
        InputRequirement {
            input: sales.clone(),
            description: "normalized aggregate sales/item demand observations",
            privacy_minimal: true,
        },
        InputRequirement {
            input: inventory.clone(),
            description: "current and historical inventory observations",
            privacy_minimal: true,
        },
        InputRequirement {
            input: recipes.clone(),
            description: "menu-to-resource transformation mapping",
            privacy_minimal: true,
        },
        InputRequirement {
            input: supplier_lead.clone(),
            description: "supplier lead-time observations or qualified estimates",
            privacy_minimal: true,
        },
        InputRequirement {
            input: waste.clone(),
            description: "waste/spoilage observations",
            privacy_minimal: true,
        },
        InputRequirement {
            input: calendar.clone(),
            description: "operating-day/daypart/calendar context",
            privacy_minimal: true,
        },
        InputRequirement {
            input: capacity.clone(),
            description: "prep/kitchen capacity observations without employee-sensitive detail",
            privacy_minimal: true,
        },
        InputRequirement {
            input: demand_forecast.clone(),
            description: "forecast output already qualified for this shadow evaluation lineage",
            privacy_minimal: true,
        },
    ] {
        inputs.insert(requirement.input.clone(), requirement);
    }

    let capabilities = vec![
        ShadowCapability {
            capability: capability("observe:hospitality:sales:v1"),
            mode: ShadowMode::Observe,
        },
        ShadowCapability {
            capability: capability("observe:hospitality:inventory:v1"),
            mode: ShadowMode::Observe,
        },
        ShadowCapability {
            capability: capability("observe:hospitality:waste:v1"),
            mode: ShadowMode::Observe,
        },
        ShadowCapability {
            capability: demand_forecast_capability(),
            mode: ShadowMode::Forecast,
        },
        ShadowCapability {
            capability: prep_recommendation_capability(),
            mode: ShadowMode::Recommend,
        },
        ShadowCapability {
            capability: replenishment_recommendation_capability(),
            mode: ShadowMode::Recommend,
        },
        ShadowCapability {
            capability: waste_review_capability(),
            mode: ShadowMode::Recommend,
        },
    ];

    let descriptor = BusinessDescriptor {
        profile: food_service_profile_ref(),
        roles: BTreeSet::from([
            role("role:hospitality:operator:v1"),
            role("role:hospitality:manager:v1"),
            role("role:hospitality:inventory-steward:v1"),
        ]),
        capabilities: capabilities
            .iter()
            .map(|value| value.capability.clone())
            .collect(),
    };

    let lanes = BTreeMap::from([
        (
            demand_forecast_lane_ref(),
            QualificationLane {
                lane: demand_forecast_lane_ref(),
                capability: demand_forecast_capability(),
                mode: ShadowMode::Forecast,
                required_inputs: BTreeSet::from([sales.clone(), calendar]),
                baseline: Some(seasonal_naive_baseline_ref()),
            },
        ),
        (
            prep_recommendation_lane_ref(),
            QualificationLane {
                lane: prep_recommendation_lane_ref(),
                capability: prep_recommendation_capability(),
                mode: ShadowMode::Recommend,
                required_inputs: BTreeSet::from([
                    demand_forecast.clone(),
                    inventory.clone(),
                    recipes.clone(),
                    capacity,
                ]),
                baseline: None,
            },
        ),
        (
            replenishment_recommendation_lane_ref(),
            QualificationLane {
                lane: replenishment_recommendation_lane_ref(),
                capability: replenishment_recommendation_capability(),
                mode: ShadowMode::Recommend,
                required_inputs: BTreeSet::from([
                    demand_forecast,
                    inventory.clone(),
                    recipes,
                    supplier_lead,
                ]),
                baseline: None,
            },
        ),
        (
            waste_review_lane_ref(),
            QualificationLane {
                lane: waste_review_lane_ref(),
                capability: waste_review_capability(),
                mode: ShadowMode::Recommend,
                required_inputs: BTreeSet::from([waste, sales, inventory]),
                baseline: None,
            },
        ),
    ]);

    HospitalityShadowProfile {
        descriptor,
        shadow_capabilities: capabilities,
        inputs,
        lanes,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DemandForecastProtocolConfig {
    pub candidate_model_lineage: ReferenceId,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub minimum_cases: u32,
    pub maximum_abstention_bps: u16,
    pub require_candidate_not_worse_than_baseline: bool,
    pub protocol_digest: Digest32,
}

pub fn demand_forecast_protocol(
    config: DemandForecastProtocolConfig,
) -> ForecastQualificationProtocol {
    ForecastQualificationProtocol {
        protocol_id: id("protocol:hospitality:demand-forecast:shadow:v1"),
        profile: food_service_profile_ref(),
        capability: demand_forecast_capability(),
        candidate_model_lineage: config.candidate_model_lineage,
        baseline_model_lineage: seasonal_naive_baseline_ref(),
        registered_at_unix_ms: config.registered_at_unix_ms,
        evaluation_start_unix_ms: config.evaluation_start_unix_ms,
        evaluation_end_unix_ms: config.evaluation_end_unix_ms,
        minimum_cases: config.minimum_cases,
        maximum_abstention_bps: config.maximum_abstention_bps,
        require_candidate_not_worse_than_baseline: config.require_candidate_not_worse_than_baseline,
        protocol_digest: config.protocol_digest,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecommendationProtocolConfig {
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub minimum_reviews: u32,
    pub minimum_useful_or_modified_bps: u16,
    pub maximum_unsafe_reviews: u32,
    pub protocol_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecommendationProtocolBuildError {
    UnknownLane,
    LaneIsNotRecommendation,
}

pub fn recommendation_protocol_for_lane(
    profile: &HospitalityShadowProfile,
    lane_ref: &ReferenceId,
    config: RecommendationProtocolConfig,
) -> Result<RecommendationQualificationProtocol, RecommendationProtocolBuildError> {
    let Some(lane) = profile.lane(lane_ref) else {
        return Err(RecommendationProtocolBuildError::UnknownLane);
    };
    if lane.mode != ShadowMode::Recommend {
        return Err(RecommendationProtocolBuildError::LaneIsNotRecommendation);
    }

    Ok(RecommendationQualificationProtocol {
        protocol_id: id(&format!("protocol:{}:shadow-review:v1", lane_ref.as_str())),
        profile: food_service_profile_ref(),
        capability: lane.capability.clone(),
        registered_at_unix_ms: config.registered_at_unix_ms,
        evaluation_start_unix_ms: config.evaluation_start_unix_ms,
        evaluation_end_unix_ms: config.evaluation_end_unix_ms,
        minimum_reviews: config.minimum_reviews,
        minimum_useful_or_modified_bps: config.minimum_useful_or_modified_bps,
        maximum_unsafe_reviews: config.maximum_unsafe_reviews,
        protocol_digest: config.protocol_digest,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn food_service_profile_is_self_consistent_and_read_only() {
        let profile = food_service_shadow_profile();
        assert_eq!(profile.validate(), Ok(()));
        assert!(profile
            .shadow_capabilities
            .iter()
            .all(|value| value.mode <= ShadowMode::Recommend));
    }

    #[test]
    fn demand_lane_pins_explicit_baseline_and_profile() {
        let protocol = demand_forecast_protocol(DemandForecastProtocolConfig {
            candidate_model_lineage: id("model:symthaea:demand:v1"),
            registered_at_unix_ms: 1_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 4_000,
            minimum_cases: 20,
            maximum_abstention_bps: 1_000,
            require_candidate_not_worse_than_baseline: true,
            protocol_digest: Digest32::repeat(7),
        });
        assert_eq!(protocol.profile, food_service_profile_ref());
        assert_eq!(protocol.capability, demand_forecast_capability());
        assert_eq!(protocol.baseline_model_lineage, seasonal_naive_baseline_ref());
        assert_eq!(protocol.validate(), Ok(()));
    }

    #[test]
    fn forecast_lane_cannot_be_used_as_recommendation_protocol() {
        let profile = food_service_shadow_profile();
        let result = recommendation_protocol_for_lane(
            &profile,
            &demand_forecast_lane_ref(),
            RecommendationProtocolConfig {
                registered_at_unix_ms: 1_000,
                evaluation_start_unix_ms: 2_000,
                evaluation_end_unix_ms: 4_000,
                minimum_reviews: 10,
                minimum_useful_or_modified_bps: 7_000,
                maximum_unsafe_reviews: 0,
                protocol_digest: Digest32::repeat(8),
            },
        );
        assert_eq!(
            result,
            Err(RecommendationProtocolBuildError::LaneIsNotRecommendation)
        );
    }

    #[test]
    fn replenishment_protocol_is_still_review_only() {
        let profile = food_service_shadow_profile();
        let protocol = recommendation_protocol_for_lane(
            &profile,
            &replenishment_recommendation_lane_ref(),
            RecommendationProtocolConfig {
                registered_at_unix_ms: 1_000,
                evaluation_start_unix_ms: 2_000,
                evaluation_end_unix_ms: 4_000,
                minimum_reviews: 10,
                minimum_useful_or_modified_bps: 7_000,
                maximum_unsafe_reviews: 0,
                protocol_digest: Digest32::repeat(9),
            },
        )
        .unwrap();
        assert_eq!(protocol.capability, replenishment_recommendation_capability());
        assert_eq!(
            profile
                .lane(&replenishment_recommendation_lane_ref())
                .unwrap()
                .mode,
            ShadowMode::Recommend
        );
    }
}
