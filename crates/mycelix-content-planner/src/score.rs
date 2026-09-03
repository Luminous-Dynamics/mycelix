use std::collections::BTreeMap;

use mycelix_content_core::PlacementPreferencesV1;
use mycelix_content_policy::PolicyEligibleCandidateV1;
use mycelix_content_state::ActionRefV1;

use crate::{
    CandidateScoreV1, CandidateSoftEvidenceV1, PENALTY_PPM_MAX, PlannerNormalizationProfileV1,
    SoftEvidenceStateV1, SoftMetricKindV1,
};

pub(crate) enum ResolvedSoftEvidenceV1 {
    One(CandidateSoftEvidenceV1),
    Conflicting,
}

pub(crate) fn normalize_soft_evidence(
    mut evidence: Vec<CandidateSoftEvidenceV1>,
) -> BTreeMap<ActionRefV1, ResolvedSoftEvidenceV1> {
    evidence.sort();
    let mut grouped: BTreeMap<ActionRefV1, Vec<CandidateSoftEvidenceV1>> = BTreeMap::new();
    for item in evidence {
        grouped.entry(item.availability_action).or_default().push(item);
    }

    grouped
        .into_iter()
        .map(|(action, mut values)| {
            values.dedup();
            let resolved = if values.len() == 1 {
                ResolvedSoftEvidenceV1::One(values.pop().expect("one evidence value"))
            } else {
                ResolvedSoftEvidenceV1::Conflicting
            };
            (action, resolved)
        })
        .collect()
}

fn ratio_ppm(value: u64, ceiling: u64) -> u32 {
    let bounded = value.min(ceiling);
    let scaled = (u128::from(bounded) * u128::from(PENALTY_PPM_MAX)) / u128::from(ceiling);
    u32::try_from(scaled).expect("bounded ppm ratio fits u32")
}

fn latency_penalty_ppm(
    latency_ms: u32,
    target_latency_ms: Option<u32>,
    ceiling_ms: u32,
) -> u32 {
    match target_latency_ms {
        Some(target) if latency_ms <= target => 0,
        Some(target) if ceiling_ms <= target => PENALTY_PPM_MAX,
        Some(target) => ratio_ppm(
            u64::from(latency_ms.saturating_sub(target)),
            u64::from(ceiling_ms - target),
        ),
        None => ratio_ppm(u64::from(latency_ms), u64::from(ceiling_ms)),
    }
}

fn evidence_state<'a>(
    candidate: &PolicyEligibleCandidateV1,
    evidence: Option<&'a ResolvedSoftEvidenceV1>,
    evaluated_at_unix_ms: u64,
) -> (SoftEvidenceStateV1, Option<&'a CandidateSoftEvidenceV1>) {
    let Some(evidence) = evidence else {
        return (SoftEvidenceStateV1::Missing, None);
    };
    let ResolvedSoftEvidenceV1::One(evidence) = evidence else {
        return (SoftEvidenceStateV1::Conflicting, None);
    };

    if evidence.advertisement_action != candidate.candidate.advertisement_action
        || evidence.provider != candidate.candidate.provider
    {
        return (SoftEvidenceStateV1::IdentityMismatch, None);
    }
    if evidence.observed_at_unix_ms >= evidence.valid_until_unix_ms {
        return (SoftEvidenceStateV1::InvalidWindow, None);
    }
    if evidence.observed_at_unix_ms > evaluated_at_unix_ms {
        return (SoftEvidenceStateV1::Future, None);
    }
    if evaluated_at_unix_ms >= evidence.valid_until_unix_ms {
        return (SoftEvidenceStateV1::Stale, None);
    }

    (SoftEvidenceStateV1::Fresh, Some(evidence))
}

fn missing_if_weighted(
    missing: &mut Vec<SoftMetricKindV1>,
    kind: SoftMetricKindV1,
    weight: u16,
    present: bool,
) {
    if weight > 0 && !present {
        missing.push(kind);
    }
}

pub(crate) fn score_candidate_v1(
    candidate: &PolicyEligibleCandidateV1,
    evidence: Option<&ResolvedSoftEvidenceV1>,
    evaluated_at_unix_ms: u64,
    preferences: PlacementPreferencesV1,
    profile: PlannerNormalizationProfileV1,
) -> CandidateScoreV1 {
    let (state, usable) = evidence_state(candidate, evidence, evaluated_at_unix_ms);

    let cost = usable.and_then(|value| value.cost_microunits_per_gib);
    let latency = usable.and_then(|value| value.latency_ms);
    let energy = usable.and_then(|value| value.energy_millijoules_per_gib);
    let locality = usable.and_then(|value| value.locality_distance_km);

    let cost_penalty_ppm = cost
        .map(|value| ratio_ppm(value, profile.cost_ceiling_microunits_per_gib))
        .unwrap_or(PENALTY_PPM_MAX);
    let latency_penalty_ppm = latency
        .map(|value| {
            latency_penalty_ppm(
                value,
                preferences.target_latency_ms,
                profile.latency_ceiling_ms,
            )
        })
        .unwrap_or(PENALTY_PPM_MAX);
    let energy_penalty_ppm = energy
        .map(|value| ratio_ppm(value, profile.energy_ceiling_millijoules_per_gib))
        .unwrap_or(PENALTY_PPM_MAX);
    let locality_penalty_ppm = locality
        .map(|value| ratio_ppm(u64::from(value), u64::from(profile.locality_ceiling_km)))
        .unwrap_or(PENALTY_PPM_MAX);

    let mut missing_weighted_metrics = Vec::new();
    missing_if_weighted(
        &mut missing_weighted_metrics,
        SoftMetricKindV1::Cost,
        preferences.cost_weight,
        cost.is_some(),
    );
    missing_if_weighted(
        &mut missing_weighted_metrics,
        SoftMetricKindV1::Latency,
        preferences.latency_weight,
        latency.is_some(),
    );
    missing_if_weighted(
        &mut missing_weighted_metrics,
        SoftMetricKindV1::Energy,
        preferences.energy_weight,
        energy.is_some(),
    );
    missing_if_weighted(
        &mut missing_weighted_metrics,
        SoftMetricKindV1::Locality,
        preferences.locality_weight,
        locality.is_some(),
    );

    let weighted_penalty = u64::from(preferences.cost_weight) * u64::from(cost_penalty_ppm)
        + u64::from(preferences.latency_weight) * u64::from(latency_penalty_ppm)
        + u64::from(preferences.energy_weight) * u64::from(energy_penalty_ppm)
        + u64::from(preferences.locality_weight) * u64::from(locality_penalty_ppm);

    CandidateScoreV1 {
        availability_action: candidate.candidate.availability_action,
        advertisement_action: candidate.candidate.advertisement_action,
        provider: candidate.candidate.provider,
        weighted_penalty,
        cost_penalty_ppm,
        latency_penalty_ppm,
        energy_penalty_ppm,
        locality_penalty_ppm,
        evidence_state: state,
        missing_weighted_metrics,
    }
}
