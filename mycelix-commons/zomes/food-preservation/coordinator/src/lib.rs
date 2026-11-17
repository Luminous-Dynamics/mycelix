// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Food Preservation Coordinator Zome
//! Business logic for preservation batches, methods, and storage management.

use food_preservation_integrity::*;
use hdk::prelude::*;
use mycelix_bridge_common::{civic_requirement_basic, civic_requirement_proposal};
use mycelix_zome_helpers as _;
use mycelix_zome_helpers::records_from_links;
use std::collections::BTreeSet;

fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    let anchor = Anchor(anchor_str.to_string());
    hash_entry(&EntryTypes::Anchor(anchor))
}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ContainerLifecycleSummary {
    pub container_hash: ActionHash,
    /// Derived from validated Filled events. This is never stored as a counter.
    pub derived_fill_cycle_count: u64,
    pub latest_event_hash: Option<ActionHash>,
    pub latest_sequence_number: Option<u64>,
    pub latest_condition: Option<ContainerCondition>,
}

fn container_history_records(container_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(container_hash, LinkTypes::ContainerToCycleEvents)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

/// Load and verify the complete linked history. A duplicated sequence,
/// missing link, fork or broken parent pointer returns an error instead of
/// guessing which branch is authoritative.
fn validate_container_event_chain(
    container_hash: &ActionHash,
    mut events: Vec<(ActionHash, ContainerCycleEvent)>,
) -> Result<Vec<(ActionHash, ContainerCycleEvent)>, String> {
    events.sort_by_key(|(_, event)| event.sequence_number);
    let mut expected_parent: Option<ActionHash> = None;
    let mut seen_sequences = BTreeSet::new();
    for (index, (action_hash, event)) in events.iter().enumerate() {
        if &event.container_hash != container_hash {
            return Err("Container history contains an event for another container".into());
        }
        if !seen_sequences.insert(event.sequence_number) {
            return Err(
                "Container lifecycle conflict: duplicate sequence number; reconciliation required"
                    .into(),
            );
        }
        if event.sequence_number != index as u64 {
            return Err(
                "Container lifecycle conflict: sequence gap; reconciliation required".into(),
            );
        }
        if event.previous_event_hash != expected_parent {
            return Err("Container lifecycle conflict: event parent does not match the unique preceding event".into());
        }
        expected_parent = Some(action_hash.clone());
    }
    Ok(events)
}

fn verified_container_history(
    container_hash: ActionHash,
) -> ExternResult<Vec<(ActionHash, ContainerCycleEvent)>> {
    let records = container_history_records(container_hash.clone())?;
    let mut events = Vec::with_capacity(records.len());
    for record in records {
        let action_hash = record.action_address().clone();
        let event: ContainerCycleEvent = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Container history link points to a non-event record".into()
            )))?;
        events.push((action_hash, event));
    }
    validate_container_event_chain(&container_hash, events)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))
}

fn derived_fill_cycle_count(events: &[(ActionHash, ContainerCycleEvent)]) -> u64 {
    events
        .iter()
        .filter(|(_, event)| event.event_kind == ContainerEventKind::Filled)
        .count() as u64
}


/// Versioned impact factors supplied by a source/region/year-specific LCA factor set.
/// The factor set is explicit input; this module contains no hard-coded emissions factors.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContainerTransportImpactFactorV1 {
    pub mode: ContainerTransportMode,
    pub mg_co2e_per_tonne_km: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContainerLifecycleImpactFactorsV1 {
    pub schema_version: u32,
    pub factor_set_ref: String,
    pub geography: String,
    pub reference_year: u32,
    pub electricity_mg_co2e_per_wh: u64,
    pub water_mg_co2e_per_litre: u64,
    pub detergent_mg_co2e_per_gram: u64,
    pub end_of_life_mg_co2e_per_container: u64,
    pub transport_factors: Vec<ContainerTransportImpactFactorV1>,
    pub evidence: ContainerEvidenceRef,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContainerImpactErrorCodeV1 {
    UnsupportedFactorSchema,
    MissingFactorProvenance,
    DuplicateTransportFactor,
    MissingEmbodiedImpact,
    MissingAcceptedServings,
    MissingSanitationObservation,
    MissingTransportObservation,
    MissingTransportFactor,
    ArithmeticOverflow,
    InvalidHistory,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContainerImpactErrorV1 {
    pub code: ContainerImpactErrorCodeV1,
    pub details: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContainerLifecycleImpactReportV1 {
    pub factor_set_ref: String,
    pub geography: String,
    pub reference_year: u32,
    pub fill_events: u64,
    pub accepted_servings: u64,
    pub empty_container_mass_g: u64,
    pub embodied_packaging_mg_co2e: u64,
    pub washing_electricity_mg_co2e: u64,
    pub washing_water_mg_co2e: u64,
    pub detergent_mg_co2e: u64,
    pub logistics_mg_co2e: u64,
    /// Projected end-of-life impact from the supplied factor set.
    pub end_of_life_mg_co2e: u64,
    pub total_packaging_mg_co2e: u64,
    pub wash_water_ml: u64,
    pub wash_energy_wh: u64,
    pub detergent_mass_mg: u64,
    pub transport_distance_m: u64,
    /// Exact fixed-point ratios: numerator / denominator.
    pub mg_co2e_per_accepted_serving: (u64, u64),
    pub wash_water_ml_per_accepted_serving: (u64, u64),
    pub wash_energy_wh_per_accepted_serving: (u64, u64),
}

/// Conservative integer ceiling division: partial units never understate impact.
fn impact_ceil_div(numerator: u128, denominator: u128) -> Result<u128, ContainerImpactErrorV1> {
    if denominator == 0 {
        return Err(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
            details: vec!["impact factor denominator cannot be zero".into()],
        });
    }
    let Some(adjusted) = numerator.checked_add(denominator - 1) else {
        return Err(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
            details: vec!["impact ceiling division overflow".into()],
        });
    };
    Ok(adjusted / denominator)
}

fn impact_mul(values: &[u128]) -> Result<u128, ContainerImpactErrorV1> {
    values.iter().try_fold(1_u128, |acc, value| {
        acc.checked_mul(*value).ok_or(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
            details: vec!["impact multiplication overflow".into()],
        })
    })
}

fn impact_sum(target: &mut u128, value: u128) -> Result<(), ContainerImpactErrorV1> {
    *target = target.checked_add(value).ok_or(ContainerImpactErrorV1 {
        code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
        details: vec!["impact aggregation overflow".into()],
    })?;
    Ok(())
}

fn impact_u64(value: u128, field: &str) -> Result<u64, ContainerImpactErrorV1> {
    u64::try_from(value).map_err(|_| ContainerImpactErrorV1 {
        code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
        details: vec![format!("{field} exceeds u64")],
    })
}

/// Validate an external factor set. A valid factor set is not independently
/// audited merely because it is structurally valid; evidence class is retained.
pub fn validate_container_impact_factors(
    factors: &ContainerLifecycleImpactFactorsV1,
) -> Result<(), ContainerImpactErrorV1> {
    if factors.schema_version != 1 {
        return Err(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::UnsupportedFactorSchema,
            details: vec![format!("unsupported impact factor schema {}", factors.schema_version)],
        });
    }
    if factors.factor_set_ref.trim().is_empty()
        || factors.geography.trim().is_empty()
        || factors.reference_year == 0
        || factors.evidence.source_ref.trim().is_empty()
        || factors.evidence.method_ref.as_deref().is_some_and(|value| value.trim().is_empty())
        || factors.evidence.calibration_ref.as_deref().is_some_and(|value| value.trim().is_empty())
    {
        return Err(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::MissingFactorProvenance,
            details: vec!["factor set, geography, year and evidence references must be populated".into()],
        });
    }
    if factors.transport_factors.iter().enumerate().any(|(index, factor)| {
        factors.transport_factors[..index].iter().any(|prior| prior.mode == factor.mode)
    }) {
        return Err(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::DuplicateTransportFactor,
            details: vec!["transport modes must have one factor each".into()],
        });
    }
    Ok(())
}

/// Compute packaging-only cradle-to-grave impact for the supplied functional
/// unit (accepted servings). Required measured/declared observations must be
/// complete; unknown values cause an error rather than silently becoming zero.
/// The report preserves exact numerator/denominator ratios and uses ceiling
/// rounding for unit conversions so integer rounding cannot understate impact.
pub fn calculate_container_lifecycle_impact(
    container: &ReusableContainer,
    container_hash: &ActionHash,
    events: &[(ActionHash, ContainerCycleEvent)],
    factors: &ContainerLifecycleImpactFactorsV1,
) -> Result<ContainerLifecycleImpactReportV1, ContainerImpactErrorV1> {
    validate_container_impact_factors(factors)?;
    let checked = validate_container_event_chain(container_hash, events.to_vec()).map_err(|reason| {
        ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::InvalidHistory,
            details: vec![reason],
        }
    })?;

    let embodied = container.embodied_impact_mg_co2e.ok_or(ContainerImpactErrorV1 {
        code: ContainerImpactErrorCodeV1::MissingEmbodiedImpact,
        details: vec!["container requires an embodied-impact value and source reference".into()],
    })?;

    let mut accepted_servings = 0_u128;
    let mut fill_events = 0_u128;
    let mut wash_water_ml = 0_u128;
    let mut wash_energy_wh = 0_u128;
    let mut detergent_mass_mg = 0_u128;
    let mut electricity_impact = 0_u128;
    let mut water_impact = 0_u128;
    let mut detergent_impact = 0_u128;
    let mut logistics_impact = 0_u128;
    let mut transport_distance_m = 0_u128;

    for (_, event) in &checked {
        if event.event_kind == ContainerEventKind::Filled {
            let servings = event.accepted_servings.ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingAcceptedServings,
                details: vec!["a Filled event is missing accepted_servings".into()],
            })?;
            accepted_servings = accepted_servings.checked_add(u128::from(servings)).ok_or(
                ContainerImpactErrorV1 {
                    code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
                    details: vec!["accepted-serving count overflow".into()],
                },
            )?;
            impact_sum(&mut fill_events, 1)?;
        }

        if matches!(&event.event_kind, ContainerEventKind::Washed | ContainerEventKind::Sanitized) {
            let observation = event.sanitation.as_ref().ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingSanitationObservation,
                details: vec![format!("{:?} event is missing sanitation observations", event.event_kind)],
            })?;
            let water_ml = observation.wash_water_ml.ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingSanitationObservation,
                details: vec!["wash_water_ml is unknown".into()],
            })?;
            let energy_wh = observation.wash_energy_wh.ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingSanitationObservation,
                details: vec!["wash_energy_wh is unknown".into()],
            })?;
            let detergent_mg = observation.detergent_mass_mg.ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingSanitationObservation,
                details: vec!["detergent_mass_mg is unknown".into()],
            })?;
            impact_sum(&mut wash_water_ml, u128::from(water_ml))?;
            impact_sum(&mut wash_energy_wh, u128::from(energy_wh))?;
            impact_sum(&mut detergent_mass_mg, u128::from(detergent_mg))?;
            impact_sum(
                &mut electricity_impact,
                impact_mul(&[u128::from(energy_wh), u128::from(factors.electricity_mg_co2e_per_wh)])?,
            )?;
            impact_sum(
                &mut water_impact,
                impact_ceil_div(
                    impact_mul(&[u128::from(water_ml), u128::from(factors.water_mg_co2e_per_litre)])?,
                    1_000,
                )?,
            )?;
            impact_sum(
                &mut detergent_impact,
                impact_ceil_div(
                    impact_mul(&[u128::from(detergent_mg), u128::from(factors.detergent_mg_co2e_per_gram)])?,
                    1_000,
                )?,
            )?;
        }

        if matches!(&event.event_kind, ContainerEventKind::Dispatched | ContainerEventKind::Returned) {
            let distance = event.transport_distance_m.ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingTransportObservation,
                details: vec![format!("{:?} event is missing transport_distance_m", event.event_kind)],
            })?;
            let mode = event.transport_mode.as_ref().ok_or(ContainerImpactErrorV1 {
                code: ContainerImpactErrorCodeV1::MissingTransportObservation,
                details: vec![format!("{:?} event is missing transport_mode", event.event_kind)],
            })?;
            let factor = factors.transport_factors.iter().find(|candidate| &candidate.mode == mode)
                .ok_or(ContainerImpactErrorV1 {
                    code: ContainerImpactErrorCodeV1::MissingTransportFactor,
                    details: vec![format!("no impact factor supplied for transport mode {:?}", mode)],
                })?;
            impact_sum(&mut transport_distance_m, u128::from(distance))?;
            // g * m / 1e9 = tonne-kilometres; factor is mg CO2e / tonne-km.
            impact_sum(
                &mut logistics_impact,
                impact_ceil_div(
                    impact_mul(&[
                        u128::from(event.transported_mass_g.ok_or(ContainerImpactErrorV1 {
                            code: ContainerImpactErrorCodeV1::MissingTransportObservation,
                            details: vec!["transported_mass_g is unknown".into()],
                        })?),
                        u128::from(distance),
                        u128::from(factor.mg_co2e_per_tonne_km),
                    ])?,
                    1_000_000_000,
                )?,
            )?;
        }
    }

    if accepted_servings == 0 || fill_events == 0 {
        return Err(ContainerImpactErrorV1 {
            code: ContainerImpactErrorCodeV1::MissingAcceptedServings,
            details: vec!["history contains no accepted servings from Filled events".into()],
        });
    }

    let eol = u128::from(factors.end_of_life_mg_co2e_per_container);
    let total = [
        u128::from(embodied),
        electricity_impact,
        water_impact,
        detergent_impact,
        logistics_impact,
        eol,
    ]
    .into_iter()
    .try_fold(0_u128, |acc, value| acc.checked_add(value).ok_or(ContainerImpactErrorV1 {
        code: ContainerImpactErrorCodeV1::ArithmeticOverflow,
        details: vec!["total lifecycle impact overflow".into()],
    }))?;

    let servings = accepted_servings;
    Ok(ContainerLifecycleImpactReportV1 {
        factor_set_ref: factors.factor_set_ref.clone(),
        geography: factors.geography.clone(),
        reference_year: factors.reference_year,
        fill_events: impact_u64(fill_events, "fill events")?,
        accepted_servings: impact_u64(accepted_servings, "accepted servings")?,
        empty_container_mass_g: container.empty_mass_g,
        embodied_packaging_mg_co2e: embodied,
        washing_electricity_mg_co2e: impact_u64(electricity_impact, "washing electricity impact")?,
        washing_water_mg_co2e: impact_u64(water_impact, "washing water impact")?,
        detergent_mg_co2e: impact_u64(detergent_impact, "detergent impact")?,
        logistics_mg_co2e: impact_u64(logistics_impact, "logistics impact")?,
        end_of_life_mg_co2e: factors.end_of_life_mg_co2e_per_container,
        total_packaging_mg_co2e: impact_u64(total, "total packaging impact")?,
        wash_water_ml: impact_u64(wash_water_ml, "wash water")?,
        wash_energy_wh: impact_u64(wash_energy_wh, "wash energy")?,
        detergent_mass_mg: impact_u64(detergent_mass_mg, "detergent mass")?,
        transport_distance_m: impact_u64(transport_distance_m, "transport distance")?,
        mg_co2e_per_accepted_serving: (impact_u64(total, "total packaging impact")?, impact_u64(servings, "accepted servings")?),
        wash_water_ml_per_accepted_serving: (impact_u64(wash_water_ml, "wash water")?, impact_u64(servings, "accepted servings")?),
        wash_energy_wh_per_accepted_serving: (impact_u64(wash_energy_wh, "wash energy")?, impact_u64(servings, "accepted servings")?),
    })
}


/// Register a reusable container in the existing food-preservation registry.
#[hdk_extern]
pub fn register_reusable_container(container: ReusableContainer) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "register_reusable_container",
    )?;
    let agent = agent_info()?.agent_initial_pubkey;
    if container.steward != agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the declared container steward may register the container".into()
        )));
    }
    for record in get_all_reusable_containers(())? {
        let existing: ReusableContainer = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Container registry link points to an invalid entry".into()
            )))?;
        if existing.id == container.id {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Reusable container ID is already registered".into()
            )));
        }
    }
    let action_hash = create_entry(&EntryTypes::ReusableContainer(container))?;
    create_entry(&EntryTypes::Anchor(Anchor(
        "all_reusable_containers".into(),
    )))?;
    create_link(
        anchor_hash("all_reusable_containers")?,
        action_hash.clone(),
        LinkTypes::AllReusableContainers,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created reusable container".into()
    )))
}

#[hdk_extern]
pub fn get_reusable_container(container_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(container_hash, GetOptions::default())
}

#[hdk_extern]
pub fn get_all_reusable_containers(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash("all_reusable_containers")?,
            LinkTypes::AllReusableContainers,
        )?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

/// Append one lifecycle event. The event must extend the unique current head;
/// a fork or incomplete history blocks mutation until it is reconciled.
#[hdk_extern]
pub fn record_container_cycle_event(event: ContainerCycleEvent) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "record_container_cycle_event",
    )?;
    let container_record = get(event.container_hash.clone(), GetOptions::default())?.ok_or(
        wasm_error!(WasmErrorInner::Guest("Container not found".into())),
    )?;
    let container: ReusableContainer = container_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Container reference is not a ReusableContainer entry".into()
        )))?;
    let agent = agent_info()?.agent_initial_pubkey;
    if container.steward != agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the container steward may append lifecycle events".into()
        )));
    }

    let history = verified_container_history(event.container_hash.clone())?;
    let expected_sequence = history.len() as u64;
    let expected_parent = history.last().map(|(hash, _)| hash.clone());
    if event.sequence_number != expected_sequence || event.previous_event_hash != expected_parent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Container event does not extend the unique current head; reload and reconcile before retrying".into()
        )));
    }

    if event.event_kind == ContainerEventKind::Filled {
        if !container.compatible_product_refs.is_empty()
            && event.product_ref.as_deref().is_none_or(|product_ref| {
                !container
                    .compatible_product_refs
                    .iter()
                    .any(|allowed| allowed == product_ref)
            })
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Product is not listed as compatible with this container".into()
            )));
        }
        let fills = derived_fill_cycle_count(&history);
        if container
            .manufacturer_reuse_cycle_limit
            .is_some_and(|limit| fills >= u64::from(limit))
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Manufacturer reuse-cycle limit reached; container cannot be filled again".into()
            )));
        }
    }

    let action_hash = create_entry(&EntryTypes::ContainerCycleEvent(event.clone()))?;
    create_link(
        event.container_hash.clone(),
        action_hash.clone(),
        LinkTypes::ContainerToCycleEvents,
        event.sequence_number.to_be_bytes().to_vec().into(),
    )?;
    if let Some(previous_hash) = event.previous_event_hash {
        create_link(
            previous_hash,
            action_hash.clone(),
            LinkTypes::ContainerEventToNext,
            (),
        )?;
    }
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created container lifecycle event".into()
    )))
}

#[hdk_extern]
pub fn get_container_cycle_history(container_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let history = verified_container_history(container_hash)?;
    history
        .into_iter()
        .map(|(action_hash, _)| {
            get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
                "Container lifecycle event disappeared during history retrieval".into()
            )))
        })
        .collect()
}

#[hdk_extern]
pub fn get_container_lifecycle_summary(
    container_hash: ActionHash,
) -> ExternResult<ContainerLifecycleSummary> {
    let history = verified_container_history(container_hash.clone())?;
    let latest = history.last();
    Ok(ContainerLifecycleSummary {
        container_hash,
        derived_fill_cycle_count: derived_fill_cycle_count(&history),
        latest_event_hash: latest.map(|(hash, _)| hash.clone()),
        latest_sequence_number: latest.map(|(_, event)| event.sequence_number),
        latest_condition: latest.map(|(_, event)| event.condition.clone()),
    })
}


// ============================================================================
// BATCH MANAGEMENT
// ============================================================================

#[hdk_extern]
pub fn start_batch(batch: PreservationBatch) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "start_batch",
    )?;
    let agent = agent_info()?.agent_initial_pubkey;
    let action_hash = create_entry(&EntryTypes::PreservationBatch(batch.clone()))?;

    create_entry(&EntryTypes::Anchor(Anchor("all_batches".to_string())))?;
    create_link(
        anchor_hash("all_batches")?,
        action_hash.clone(),
        LinkTypes::AllBatches,
        (),
    )?;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToBatch, ())?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created batch".into()
    )))
}

#[hdk_extern]
pub fn complete_batch(batch_hash: ActionHash) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_proposal(),
        "complete_batch",
    )?;
    let record = get(batch_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Batch not found".into())))?;
    let mut batch: PreservationBatch = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid batch entry".into()
        )))?;

    batch.status = BatchStatus::Completed;
    let new_hash = update_entry(
        record.action_address().clone(),
        &EntryTypes::PreservationBatch(batch),
    )?;
    get(new_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated batch".into()
    )))
}

#[hdk_extern]
pub fn get_batch(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

#[hdk_extern]
pub fn get_agent_batches(_: ()) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToBatch)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

// ============================================================================
// METHOD REGISTRY
// ============================================================================

#[hdk_extern]
pub fn register_method(method: PreservationMethod) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "register_method",
    )?;
    let action_hash = create_entry(&EntryTypes::PreservationMethod(method))?;

    create_entry(&EntryTypes::Anchor(Anchor("all_methods".to_string())))?;
    create_link(
        anchor_hash("all_methods")?,
        action_hash.clone(),
        LinkTypes::AllMethods,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created method".into()
    )))
}

#[hdk_extern]
pub fn get_all_methods(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("all_methods")?, LinkTypes::AllMethods)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

// ============================================================================
// STORAGE MANAGEMENT
// ============================================================================

#[hdk_extern]
pub fn register_storage(storage: StorageUnit) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "register_storage",
    )?;
    let action_hash = create_entry(&EntryTypes::StorageUnit(storage))?;

    create_entry(&EntryTypes::Anchor(Anchor("all_storage".to_string())))?;
    create_link(
        anchor_hash("all_storage")?,
        action_hash.clone(),
        LinkTypes::AllStorage,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created storage".into()
    )))
}

#[hdk_extern]
pub fn get_storage_inventory(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("all_storage")?, LinkTypes::AllStorage)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    #[derive(Deserialize)]
    struct ContainerLifecycleFixtureBundle {
        container: ContainerFixtureIdentity,
        events: Vec<ContainerEventFixture>,
    }

    #[derive(Deserialize)]
    struct ContainerFixtureIdentity {
        id: String,
        material: ContainerMaterial,
        capacity_ml: u64,
        manufacturer_reuse_cycle_limit: Option<u32>,
    }

    #[derive(Deserialize)]
    struct ContainerEventFixture {
        container_hash: String,
        previous_event_hash: Option<String>,
        sequence_number: u64,
        event_kind: ContainerEventKind,
        occurred_at: u64,
        product_ref: Option<String>,
        product_lot_ref: Option<String>,
        location_ref: Option<String>,
        condition: ContainerCondition,
        sanitation: Option<ContainerSanitationObservation>,
        evidence: ContainerEvidenceRef,
    }

    #[test]
    fn synthetic_container_lifecycle_fixture_has_one_verified_linear_history() {
        let bundle: ContainerLifecycleFixtureBundle = serde_json::from_str(include_str!(
            "../../integrity/schemas/fixtures/container-lifecycle-v1.json"
        ))
        .expect("container lifecycle fixture bundle must deserialize");
        assert_eq!(bundle.container.id, "fixture:container-za-001");
        assert_eq!(bundle.container.capacity_ml, 1_000);
        assert_eq!(bundle.container.manufacturer_reuse_cycle_limit, Some(500));
        assert_eq!(
            bundle.container.material,
            ContainerMaterial::BorosilicateGlass
        );

        let container_hash = sample_container_hash();
        let mut records = Vec::new();
        let mut prior_hash: Option<ActionHash> = None;
        for (index, fixture) in bundle.events.into_iter().enumerate() {
            assert_eq!(fixture.container_hash, "uhCEkSyntheticContainerHash");
            assert_eq!(fixture.sequence_number, index as u64);
            assert_eq!(
                fixture.previous_event_hash,
                index
                    .checked_sub(1)
                    .map(|previous| format!("uhCEkSyntheticEventHash{previous}"))
            );
            let action_hash = ActionHash::from_raw_36(vec![(index as u8).wrapping_add(20); 36]);
            let event = ContainerCycleEvent {
                container_hash: container_hash.clone(),
                previous_event_hash: prior_hash.clone(),
                sequence_number: fixture.sequence_number,
                event_kind: fixture.event_kind,
                occurred_at: fixture.occurred_at,
                product_ref: fixture.product_ref,
                product_lot_ref: fixture.product_lot_ref,
                preservation_batch_hash: None,
                location_ref: fixture.location_ref,
                condition: fixture.condition,
                sanitation: fixture.sanitation,
                evidence: fixture.evidence,
            };
            records.push((action_hash.clone(), event));
            prior_hash = Some(action_hash);
        }

        let checked = validate_container_event_chain(&container_hash, records).unwrap();
        assert_eq!(checked.len(), 12);
        assert_eq!(derived_fill_cycle_count(&checked), 1);
    }


    fn sample_container_hash() -> ActionHash {
        ActionHash::from_raw_36(vec![7u8; 36])
    }

    fn sample_cycle_event(
        sequence_number: u64,
        previous_event_hash: Option<ActionHash>,
        event_kind: ContainerEventKind,
    ) -> (ActionHash, ContainerCycleEvent) {
        let container_hash = sample_container_hash();
        let hash = ActionHash::from_raw_36(vec![(sequence_number as u8).wrapping_add(20); 36]);
        let is_filled = event_kind == ContainerEventKind::Filled;
        let event = ContainerCycleEvent {
            container_hash,
            previous_event_hash,
            sequence_number,
            event_kind,
            occurred_at: 1_700_000_000 + sequence_number,
            product_ref: is_filled.then(|| "product:fixture".into()),
            product_lot_ref: is_filled.then(|| "lot:fixture-001".into()),
            accepted_servings: is_filled.then_some(4),
            filled_mass_g: is_filled.then_some(1_000),
            filled_volume_ml: None,
            transport_distance_m: None,
            transport_mode: None,
            transported_mass_g: None,
            end_of_life_route_ref: None,
            preservation_batch_hash: None,
            location_ref: None,
            condition: ContainerCondition::Unknown,
            sanitation: matches!(
                &event_kind,
                ContainerEventKind::Washed | ContainerEventKind::Sanitized
            )
            .then(|| ContainerSanitationObservation {
                protocol_ref: "protocol:fixture".into(),
                wash_water_temp_millic: Some(60_000),
                sanitizer_ref: Some("sanitizer:fixture".into()),
                sanitizer_concentration_mg_l: Some(100),
                exposure_time_ms: Some(30_000),
                wash_water_ml: Some(2_000),
                wash_energy_wh: Some(30),
                detergent_mass_mg: Some(10_000),
                observation_ref: "observation:fixture".into(),
                evidence: ContainerEvidenceRef {
                    source_ref: "fixture:sanitation".into(),
                    method_ref: None,
                    calibration_ref: None,
                    evidence_class: ContainerEvidenceClass::Declared,
                },
            }),
            evidence: ContainerEvidenceRef {
                source_ref: "fixture:event".into(),
                method_ref: None,
                calibration_ref: None,
                evidence_class: ContainerEvidenceClass::Declared,
            },
        };
        (hash, event)
    }


    #[test]
    fn container_history_accepts_one_linear_chain() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h1, e1) = sample_cycle_event(1, Some(h0.clone()), ContainerEventKind::Sanitized);
        let (h2, e2) = sample_cycle_event(2, Some(h1.clone()), ContainerEventKind::Inspected);
        let checked = validate_container_event_chain(
            &sample_container_hash(),
            vec![(h2, e2), (h0, e0), (h1, e1)],
        )
        .unwrap();
        assert_eq!(checked.len(), 3);
        assert_eq!(checked[2].1.sequence_number, 2);
    }

    #[test]
    fn container_history_rejects_duplicate_sequence_fork() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h1a, e1a) = sample_cycle_event(1, Some(h0.clone()), ContainerEventKind::Sanitized);
        let (h1b, e1b) = sample_cycle_event(1, Some(h0), ContainerEventKind::Sanitized);
        let error = validate_container_event_chain(
            &sample_container_hash(),
            vec![
                (h1a, e1a),
                (h1b, e1b),
                (ActionHash::from_raw_36(vec![1; 36]), e0),
            ],
        )
        .unwrap_err();
        assert!(error.contains("duplicate sequence"));
    }

    #[test]
    fn container_history_rejects_gaps_and_wrong_parent() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h2, e2) = sample_cycle_event(2, Some(h0.clone()), ContainerEventKind::Inspected);
        assert!(
            validate_container_event_chain(
                &sample_container_hash(),
                vec![(h0.clone(), e0.clone()), (h2, e2),]
            )
            .unwrap_err()
            .contains("sequence gap")
        );

        let (wrong_parent_hash, wrong_parent_event) = sample_cycle_event(
            1,
            Some(ActionHash::from_raw_36(vec![99; 36])),
            ContainerEventKind::Sanitized,
        );
        assert!(
            validate_container_event_chain(
                &sample_container_hash(),
                vec![(h0, e0), (wrong_parent_hash, wrong_parent_event),]
            )
            .unwrap_err()
            .contains("parent")
        );
    }

    fn synthetic_lca_container() -> ReusableContainer {
        ReusableContainer {
            id: "fixture:lca-container".into(),
            material: ContainerMaterial::BorosilicateGlass,
            capacity_ml: 2_000,
            empty_mass_g: 600,
            temperature_min_millic: Some(0),
            temperature_max_millic: Some(100_000),
            closure_type: "screw-top".into(),
            food_contact_qualification_ref: "fixture:food-contact".into(),
            cleaning_protocol_ref: "fixture:cleaning".into(),
            manufacturer_reuse_cycle_limit: None,
            compatible_product_refs: vec!["product:fixture".into()],
            registered_at: 1_700_000_000,
            steward: fake_agent(),
            embodied_impact_mg_co2e: Some(25_000),
            embodied_impact_source_ref: Some("fixture:embodied-impact".into()),
            evidence: ContainerEvidenceRef {
                source_ref: "fixture:container".into(),
                method_ref: Some("method:container-lca".into()),
                calibration_ref: None,
                evidence_class: ContainerEvidenceClass::Declared,
            },
        }
    }

    fn synthetic_lca_factors() -> ContainerLifecycleImpactFactorsV1 {
        ContainerLifecycleImpactFactorsV1 {
            schema_version: 1,
            factor_set_ref: "fixture:za-impact-factors-2026".into(),
            geography: "ZA-fixture-only".into(),
            reference_year: 2026,
            electricity_mg_co2e_per_wh: 100,
            water_mg_co2e_per_litre: 10,
            detergent_mg_co2e_per_gram: 1_000,
            end_of_life_mg_co2e_per_container: 100,
            transport_factors: vec![ContainerTransportImpactFactorV1 {
                mode: ContainerTransportMode::Bicycle,
                mg_co2e_per_tonne_km: 100_000,
            }],
            evidence: ContainerEvidenceRef {
                source_ref: "fixture:impact-factor-source".into(),
                method_ref: Some("method:synthetic-factor-set".into()),
                calibration_ref: None,
                evidence_class: ContainerEvidenceClass::Simulated,
            },
        }
    }

    fn synthetic_lca_history() -> Vec<(ActionHash, ContainerCycleEvent)> {
        let kinds = [
            ContainerEventKind::Washed,
            ContainerEventKind::Sanitized,
            ContainerEventKind::Inspected,
            ContainerEventKind::Filled,
            ContainerEventKind::Sealed,
            ContainerEventKind::Dispatched,
            ContainerEventKind::Received,
            ContainerEventKind::Emptied,
            ContainerEventKind::Returned,
            ContainerEventKind::Washed,
            ContainerEventKind::Sanitized,
            ContainerEventKind::Inspected,
        ];
        let mut history = Vec::new();
        let mut previous: Option<ActionHash> = None;
        for (index, kind) in kinds.into_iter().enumerate() {
            let is_dispatched = kind == ContainerEventKind::Dispatched;
            let is_returned = kind == ContainerEventKind::Returned;
            let (hash, mut event) =
                sample_cycle_event(index as u64, previous.clone(), kind);
            if is_dispatched {
                event.transport_distance_m = Some(10_000);
                event.transport_mode = Some(ContainerTransportMode::Bicycle);
                event.transported_mass_g = Some(1_600);
            }
            if is_returned {
                event.transport_distance_m = Some(10_000);
                event.transport_mode = Some(ContainerTransportMode::Bicycle);
                event.transported_mass_g = Some(600);
            }
            previous = Some(hash.clone());
            history.push((hash, event));
        }
        history
    }

    #[test]
    fn packaging_lifecycle_impact_is_reproducible_per_accepted_serving() {
        let container = synthetic_lca_container();
        let container_hash = sample_container_hash();
        let history = synthetic_lca_history();
        let factors = synthetic_lca_factors();
        let report = calculate_container_lifecycle_impact(
            &container,
            &container_hash,
            &history,
            &factors,
        )
        .unwrap();

        assert_eq!(report.fill_events, 1);
        assert_eq!(report.accepted_servings, 4);
        assert_eq!(report.embodied_packaging_mg_co2e, 25_000);
        assert_eq!(report.washing_electricity_mg_co2e, 12_000);
        assert_eq!(report.washing_water_mg_co2e, 80);
        assert_eq!(report.detergent_mg_co2e, 40_000);
        assert_eq!(report.logistics_mg_co2e, 2_200);
        assert_eq!(report.end_of_life_mg_co2e, 100);
        assert_eq!(report.total_packaging_mg_co2e, 79_380);
        assert_eq!(report.mg_co2e_per_accepted_serving, (79_380, 4));
        assert_eq!(report.wash_water_ml_per_accepted_serving, (8_000, 4));
        assert_eq!(report.wash_energy_wh_per_accepted_serving, (120, 4));
    }

    #[test]
    fn packaging_lifecycle_impact_refuses_unknown_sanitation_inputs() {
        let container = synthetic_lca_container();
        let container_hash = sample_container_hash();
        let mut history = synthetic_lca_history();
        history[0]
            .1
            .sanitation
            .as_mut()
            .expect("wash fixture has an observation")
            .wash_energy_wh = None;
        let error = calculate_container_lifecycle_impact(
            &container,
            &container_hash,
            &history,
            &synthetic_lca_factors(),
        )
        .unwrap_err();
        assert_eq!(
            error.code,
            ContainerImpactErrorCodeV1::MissingSanitationObservation
        );
    }

    #[test]
    fn reuse_count_is_derived_only_from_fill_events() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h1, e1) = sample_cycle_event(1, Some(h0.clone()), ContainerEventKind::Filled);
        let (h2, e2) = sample_cycle_event(2, Some(h1.clone()), ContainerEventKind::Emptied);
        let (h3, e3) = sample_cycle_event(3, Some(h2.clone()), ContainerEventKind::Filled);
        let history = validate_container_event_chain(
            &sample_container_hash(),
            vec![(h0, e0), (h1, e1), (h2, e2), (h3, e3)],
        )
        .unwrap();
        assert_eq!(derived_fill_cycle_count(&history), 2);
    }


    // ========================================================================
    // BatchStatus enum serde tests
    // ========================================================================

    #[test]
    fn batch_status_all_variants_serde() {
        let variants = vec![
            BatchStatus::InProgress,
            BatchStatus::Completed,
            BatchStatus::Failed,
            BatchStatus::Consumed,
        ];
        for variant in variants {
            let json = serde_json::to_string(&variant).unwrap();
            let decoded: BatchStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded, variant);
        }
    }

    // ========================================================================
    // SkillLevel enum serde tests
    // ========================================================================

    #[test]
    fn skill_level_all_variants_serde() {
        let variants = vec![
            SkillLevel::Beginner,
            SkillLevel::Intermediate,
            SkillLevel::Advanced,
        ];
        for variant in variants {
            let json = serde_json::to_string(&variant).unwrap();
            let decoded: SkillLevel = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded, variant);
        }
    }

    // ========================================================================
    // StorageType enum serde tests
    // ========================================================================

    #[test]
    fn storage_type_all_variants_serde() {
        let variants = vec![
            StorageType::RootCellar,
            StorageType::Cellar,
            StorageType::Freezer,
            StorageType::Dehydrator,
            StorageType::Fermenter,
            StorageType::Pantry,
            StorageType::Composter,
        ];
        for variant in variants {
            let json = serde_json::to_string(&variant).unwrap();
            let decoded: StorageType = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded, variant);
        }
    }

    // ========================================================================
    // PreservationBatch serde roundtrip tests
    // ========================================================================

    #[test]
    fn preservation_batch_serde_roundtrip() {
        let batch = PreservationBatch {
            id: "batch-1".to_string(),
            source_crop_hash: None,
            method: "Lacto-fermentation".to_string(),
            quantity_kg: 5.0,
            started_at: 1700000000,
            expected_ready: 1701000000,
            status: BatchStatus::InProgress,
            notes: Some("First fermentation attempt".to_string()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, "batch-1");
        assert_eq!(decoded.method, "Lacto-fermentation");
        assert_eq!(decoded.quantity_kg, 5.0);
        assert_eq!(decoded.status, BatchStatus::InProgress);
        assert_eq!(
            decoded.notes,
            Some("First fermentation attempt".to_string())
        );
    }

    #[test]
    fn preservation_batch_none_fields_roundtrip() {
        let batch = PreservationBatch {
            id: "batch-2".to_string(),
            source_crop_hash: None,
            method: "Drying".to_string(),
            quantity_kg: 2.0,
            started_at: 1700000000,
            expected_ready: 1700500000,
            status: BatchStatus::Completed,
            notes: None,
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.source_crop_hash, None);
        assert_eq!(decoded.notes, None);
        assert_eq!(decoded.status, BatchStatus::Completed);
    }

    // ========================================================================
    // PreservationMethod serde roundtrip tests
    // ========================================================================

    #[test]
    fn preservation_method_serde_roundtrip() {
        let method = PreservationMethod {
            name: "Water Bath Canning".to_string(),
            description: "High-acid food preservation using boiling water".to_string(),
            shelf_life_days: 365,
            equipment_needed: vec!["Canning pot".to_string(), "Jars".to_string()],
            skill_level: SkillLevel::Intermediate,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.name, "Water Bath Canning");
        assert_eq!(decoded.shelf_life_days, 365);
        assert_eq!(decoded.skill_level, SkillLevel::Intermediate);
        assert_eq!(decoded.equipment_needed.len(), 2);
    }

    // ========================================================================
    // StorageUnit serde roundtrip tests
    // ========================================================================

    #[test]
    fn storage_unit_serde_roundtrip() {
        let unit = StorageUnit {
            id: "store-1".to_string(),
            name: "Community Root Cellar".to_string(),
            capacity_kg: 500.0,
            storage_type: StorageType::RootCellar,
            steward: fake_agent(),
        };
        let json = serde_json::to_string(&unit).unwrap();
        let decoded: StorageUnit = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, "store-1");
        assert_eq!(decoded.name, "Community Root Cellar");
        assert_eq!(decoded.capacity_kg, 500.0);
        assert_eq!(decoded.storage_type, StorageType::RootCellar);
    }

    // ========================================================================
    // Clone/equality tests
    // ========================================================================

    #[test]
    fn preservation_batch_clone_equals_original() {
        let batch = PreservationBatch {
            id: "batch-clone".to_string(),
            source_crop_hash: Some(ActionHash::from_raw_36(vec![1u8; 36])),
            method: "Smoking".to_string(),
            quantity_kg: 3.5,
            started_at: 1700000000,
            expected_ready: 1700500000,
            status: BatchStatus::InProgress,
            notes: Some("Hickory chips".to_string()),
            allergen_flags: vec![],
        };
        assert_eq!(batch, batch.clone());
    }

    #[test]
    fn preservation_batch_ne_different_status() {
        let a = PreservationBatch {
            id: "batch-ne".to_string(),
            source_crop_hash: None,
            method: "Drying".to_string(),
            quantity_kg: 1.0,
            started_at: 0,
            expected_ready: 1,
            status: BatchStatus::InProgress,
            notes: None,
            allergen_flags: vec![],
        };
        let mut b = a.clone();
        b.status = BatchStatus::Failed;
        assert_ne!(a, b);
    }

    #[test]
    fn preservation_method_clone_equals_original() {
        let method = PreservationMethod {
            name: "Canning".to_string(),
            description: "Seal in jars".to_string(),
            shelf_life_days: 730,
            equipment_needed: vec!["Jars".to_string(), "Lids".to_string()],
            skill_level: SkillLevel::Advanced,
        };
        assert_eq!(method, method.clone());
    }

    #[test]
    fn storage_unit_clone_equals_original() {
        let unit = StorageUnit {
            id: "s-clone".to_string(),
            name: "Pantry".to_string(),
            capacity_kg: 50.0,
            storage_type: StorageType::Pantry,
            steward: fake_agent(),
        };
        assert_eq!(unit, unit.clone());
    }

    // ========================================================================
    // Edge case tests
    // ========================================================================

    #[test]
    fn preservation_batch_with_source_crop_hash() {
        let batch = PreservationBatch {
            id: "batch-crop".to_string(),
            source_crop_hash: Some(ActionHash::from_raw_36(vec![99u8; 36])),
            method: "Pickling".to_string(),
            quantity_kg: 10.0,
            started_at: 1700000000,
            expected_ready: 1702000000,
            status: BatchStatus::Completed,
            notes: Some("Pickled cucumbers".to_string()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert!(decoded.source_crop_hash.is_some());
        assert_eq!(decoded.status, BatchStatus::Completed);
    }

    #[test]
    fn preservation_batch_zero_quantity_serde() {
        let batch = PreservationBatch {
            id: "batch-zero".to_string(),
            source_crop_hash: None,
            method: "Freeze-drying".to_string(),
            quantity_kg: 0.0,
            started_at: 0,
            expected_ready: 0,
            status: BatchStatus::Failed,
            notes: None,
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.quantity_kg, 0.0);
        assert_eq!(decoded.status, BatchStatus::Failed);
    }

    #[test]
    fn preservation_batch_consumed_status_serde() {
        let batch = PreservationBatch {
            id: "batch-consumed".to_string(),
            source_crop_hash: None,
            method: "Fermenting".to_string(),
            quantity_kg: 2.5,
            started_at: 1700000000,
            expected_ready: 1700500000,
            status: BatchStatus::Consumed,
            notes: Some("Kimchi consumed by community".to_string()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.status, BatchStatus::Consumed);
    }

    #[test]
    fn preservation_method_empty_equipment_serde() {
        let method = PreservationMethod {
            name: "Sun drying".to_string(),
            description: "Dry in direct sunlight".to_string(),
            shelf_life_days: 90,
            equipment_needed: vec![],
            skill_level: SkillLevel::Beginner,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert!(decoded.equipment_needed.is_empty());
    }

    #[test]
    fn preservation_method_u32_max_shelf_life() {
        let method = PreservationMethod {
            name: "Ancient method".to_string(),
            description: "Lasts forever".to_string(),
            shelf_life_days: u32::MAX,
            equipment_needed: vec!["Magic jar".to_string()],
            skill_level: SkillLevel::Advanced,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.shelf_life_days, u32::MAX);
    }

    #[test]
    fn storage_unit_all_types_serde() {
        for st in [
            StorageType::RootCellar,
            StorageType::Cellar,
            StorageType::Freezer,
            StorageType::Dehydrator,
            StorageType::Fermenter,
            StorageType::Pantry,
            StorageType::Composter,
        ] {
            let unit = StorageUnit {
                id: "s".to_string(),
                name: "T".to_string(),
                capacity_kg: 1.0,
                storage_type: st.clone(),
                steward: fake_agent(),
            };
            let json = serde_json::to_string(&unit).unwrap();
            let decoded: StorageUnit = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded.storage_type, st);
        }
    }

    #[test]
    fn preservation_method_unicode_name_serde() {
        let method = PreservationMethod {
            name: "\u{767D}\u{83DC}\u{306E}\u{6F2C}\u{7269}".to_string(),
            description: "Japanese pickling".to_string(),
            shelf_life_days: 30,
            equipment_needed: vec!["\u{6F2C}\u{7269}\u{77F3}".to_string()],
            skill_level: SkillLevel::Intermediate,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.name, "\u{767D}\u{83DC}\u{306E}\u{6F2C}\u{7269}");
    }

    #[test]
    fn storage_unit_large_capacity_serde() {
        let unit = StorageUnit {
            id: "s-big".to_string(),
            name: "Warehouse".to_string(),
            capacity_kg: f64::MAX,
            storage_type: StorageType::Freezer,
            steward: fake_agent(),
        };
        let json = serde_json::to_string(&unit).unwrap();
        let decoded: StorageUnit = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.capacity_kg, f64::MAX);
    }

    // ========================================================================
    // Additional boundary and edge case tests
    // ========================================================================

    #[test]
    fn preservation_method_many_equipment_items_serde() {
        let equipment: Vec<String> = (0..100).map(|i| format!("Equipment item #{}", i)).collect();
        let method = PreservationMethod {
            name: "Complex method".to_string(),
            description: "Requires many tools and supplies".to_string(),
            shelf_life_days: 180,
            equipment_needed: equipment.clone(),
            skill_level: SkillLevel::Advanced,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.equipment_needed.len(), 100);
        assert_eq!(decoded.equipment_needed[0], "Equipment item #0");
        assert_eq!(decoded.equipment_needed[99], "Equipment item #99");
    }

    #[test]
    fn preservation_batch_ne_different_id() {
        let a = PreservationBatch {
            id: "batch-a".to_string(),
            source_crop_hash: None,
            method: "Drying".to_string(),
            quantity_kg: 1.0,
            started_at: 0,
            expected_ready: 1,
            status: BatchStatus::InProgress,
            notes: None,
            allergen_flags: vec![],
        };
        let mut b = a.clone();
        b.id = "batch-b".to_string();
        assert_ne!(a, b);
    }

    #[test]
    fn storage_unit_ne_different_type() {
        let a = StorageUnit {
            id: "s-1".to_string(),
            name: "Unit A".to_string(),
            capacity_kg: 100.0,
            storage_type: StorageType::Freezer,
            steward: fake_agent(),
        };
        let mut b = a.clone();
        b.storage_type = StorageType::Pantry;
        assert_ne!(a, b);
    }

    #[test]
    fn preservation_batch_long_notes_serde() {
        let long_notes = "A".repeat(10_000);
        let batch = PreservationBatch {
            id: "batch-long".to_string(),
            source_crop_hash: None,
            method: "Salt curing".to_string(),
            quantity_kg: 25.0,
            started_at: 1700000000,
            expected_ready: 1702000000,
            status: BatchStatus::InProgress,
            notes: Some(long_notes.clone()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.notes.unwrap().len(), 10_000);
    }
}
