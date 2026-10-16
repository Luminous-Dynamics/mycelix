//! Fail-closed admission for traffic/model combinations in the research kernel.
//!
//! The registry intentionally admits exactly one synthetic fixture profile. Adding
//! any new pair requires an explicit contract/test update; a real-world traffic
//! archetype cannot inherit toy-kernel support from its coordinates alone.

use serde::Deserialize;
use std::collections::BTreeSet;
use std::sync::OnceLock;

const REGISTRY_SCHEMA: &str = "airspace-model-applicability-v1";
const REGISTRY_STATUS: &str = "research_only_registry";
const REGISTRY_TEXT: &str = include_str!("../fixtures/model_applicability.json");

pub(crate) const SYNTHETIC_TRAFFIC_PROFILE_ID: &str = "synthetic_fixture_kinematics_v1";
pub(crate) const CONSTANT_VELOCITY_TOY_MODEL_ID: &str = "constant_velocity_toy_v1";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelApplicabilityRegistryV1 {
    schema: String,
    status: String,
    default_on_unmatched: String,
    authority_ceiling: String,
    supported_pairs: Vec<ModelApplicabilityPairV1>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelApplicabilityPairV1 {
    traffic_profile_id: String,
    model_profile_id: String,
    kernel_id: String,
    applicability_domain: String,
    model_status: String,
    operational_claims: bool,
}

fn registry() -> Result<&'static ModelApplicabilityRegistryV1, String> {
    static REGISTRY: OnceLock<Result<ModelApplicabilityRegistryV1, String>> = OnceLock::new();
    match REGISTRY.get_or_init(|| {
        let value: ModelApplicabilityRegistryV1 = serde_json::from_str(REGISTRY_TEXT)
            .map_err(|error| format!("model-applicability registry is invalid JSON/schema: {error}"))?;
        if value.schema != REGISTRY_SCHEMA
            || value.status != REGISTRY_STATUS
            || value.default_on_unmatched != "unresolved"
            || value.authority_ceiling
                != "synthetic_fixture_admission_only_not_operational_model_support"
        {
            return Err("model-applicability registry header is unsupported".to_owned());
        }
        // This first implementation deliberately recognizes exactly one pair.
        // A future model must expand the schema and independent tests explicitly;
        // adding an entry alone must never silently widen runtime admission.
        if value.supported_pairs.len() != 1 {
            return Err("model-applicability registry must contain exactly one explicitly qualified pair".to_owned());
        }
        let pair = &value.supported_pairs[0];
        let required = (
            SYNTHETIC_TRAFFIC_PROFILE_ID,
            CONSTANT_VELOCITY_TOY_MODEL_ID,
            "synthetic_local_cartesian_constant_velocity",
            "synthetic_fixtures_only",
            "toy_not_validated",
        );
        let observed = (
            pair.traffic_profile_id.as_str(),
            pair.model_profile_id.as_str(),
            pair.kernel_id.as_str(),
            pair.applicability_domain.as_str(),
            pair.model_status.as_str(),
        );
        if observed != required || pair.operational_claims {
            return Err("model-applicability registry attempted to widen the toy kernel authority or applicability domain".to_owned());
        }
        let unique: BTreeSet<(&str, &str)> = value.supported_pairs.iter()
            .map(|item| (item.traffic_profile_id.as_str(), item.model_profile_id.as_str()))
            .collect();
        if unique.len() != value.supported_pairs.len() {
            return Err("model-applicability registry contains duplicate traffic/model pairs".to_owned());
        }
        Ok(value)
    }) {
        Ok(value) => Ok(value),
        Err(error) => Err(error.clone()),
    }
}

/// Return success only for the single research-only synthetic pair.
///
/// The reason is deliberately suitable for an unresolved report. It is never a
/// declaration of operational support or a claim that the toy model is validated.
pub(crate) fn validate_model_profile(
    traffic_profile_id: &str,
    model_profile_id: &str,
) -> Result<(), String> {
    if traffic_profile_id.trim().is_empty() {
        return Err("traffic_profile_id is missing; model applicability is unresolved".to_owned());
    }
    if model_profile_id.trim().is_empty() {
        return Err("model_profile_id is missing; model applicability is unresolved".to_owned());
    }
    let value = registry()?;
    let admitted = value.supported_pairs.iter().any(|pair| {
        pair.traffic_profile_id == traffic_profile_id
            && pair.model_profile_id == model_profile_id
            && pair.applicability_domain == "synthetic_fixtures_only"
            && pair.model_status == "toy_not_validated"
            && !pair.operational_claims
    });
    if admitted {
        Ok(())
    } else {
        Err(format!(
            "traffic profile {traffic_profile_id:?} with model profile {model_profile_id:?} has no admissible registry entry"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_admits_only_the_declared_synthetic_pair() {
        assert!(validate_model_profile(
            SYNTHETIC_TRAFFIC_PROFILE_ID,
            CONSTANT_VELOCITY_TOY_MODEL_ID
        ).is_ok());
        assert!(validate_model_profile("", CONSTANT_VELOCITY_TOY_MODEL_ID).is_err());
        assert!(validate_model_profile(SYNTHETIC_TRAFFIC_PROFILE_ID, "").is_err());
        assert!(validate_model_profile(
            SYNTHETIC_TRAFFIC_PROFILE_ID,
            "some_future_model"
        ).is_err());
    }

    #[test]
    fn every_real_world_traffic_archetype_is_rejected_by_the_toy_kernel() {
        let catalogue: serde_json::Value = serde_json::from_str(include_str!(
            "../fixtures/traffic_diversity.json"
        )).expect("traffic-diversity catalogue must parse");
        let archetypes = catalogue["archetypes"]
            .as_array()
            .expect("catalogue must contain archetypes");
        assert_eq!(archetypes.len(), 15);
        for archetype in archetypes {
            let profile_id = archetype["profile_id"]
                .as_str()
                .expect("each archetype must have an ID");
            assert!(
                validate_model_profile(profile_id, CONSTANT_VELOCITY_TOY_MODEL_ID).is_err(),
                "real-world archetype must not enter the toy kernel: {profile_id}"
            );
        }
        assert!(
            validate_model_profile("unknown_unregistered_traffic", CONSTANT_VELOCITY_TOY_MODEL_ID).is_err()
        );
    }
}
