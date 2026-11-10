// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Versioned, privacy-aware Battery Passport lifecycle projection.
//!
//! This module models the regulatory passport as a projection of the
//! append-only physical-battery lifecycle. It does not establish regulatory
//! compliance, safety, conformity assessment, or authorization.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const DIGEST_LENGTH: usize = 64;
const PROJECTION_POLICY_VERSION: &str = "battery-passport-access-policy/0.1";

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn valid_digest(value: &str) -> bool {
    value.len() == DIGEST_LENGTH
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryCategory {
    ElectricVehicle,
    LightMeansOfTransport,
    IndustrialOverTwoKwh,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PassportIssueKind {
    InitialMarketPlacement,
    Remanufacture,
    Repurpose,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryPassportStatus {
    Active,
    Superseded,
    ClosedByRecycling,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryLifecycleEventKind {
    Manufactured,
    PlacedOnMarket,
    Inspection,
    RoutineService,
    SimpleRepair,
    Remanufactured,
    Repurposed,
    ConfigurationChanged,
    ResponsibilityTransferred,
    Quarantined,
    EndOfLife,
    Recycled,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryEvidenceReference {
    pub reference: String,
    pub digest: String,
    pub evidence_class: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryPassportRecord {
    pub passport_id: String,
    pub physical_battery_id: String,
    pub model_id: String,
    pub predecessor_passport_id: Option<String>,
    pub issue_kind: PassportIssueKind,
    pub issued_at_ms: u64,
    pub issuing_operator_ref: String,
    pub registry_version: String,
    pub configuration_digest: String,
    pub issuing_event_digest: String,
    pub status: BatteryPassportStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryLifecycleEvent {
    pub event_id: String,
    pub sequence: u64,
    pub kind: BatteryLifecycleEventKind,
    pub physical_battery_id: String,
    /// Passport in force before this event. Initial market placement names
    /// the newly issued passport; manufacture may occur before a passport.
    pub passport_id: Option<String>,
    /// Set only for remanufacture/repurpose events that issue a successor.
    pub successor_passport_id: Option<String>,
    pub resulting_configuration_digest: String,
    pub actor_ref: String,
    pub responsible_operator_after: Option<String>,
    pub occurred_at_ms: u64,
    pub recorded_at_ms: u64,
    pub previous_event_digest: Option<String>,
    pub evidence: Vec<BatteryEvidenceReference>,
    pub revalidation_obligation_refs: Vec<String>,
    pub event_digest: String,
}

#[derive(Debug, Clone, Serialize)]
struct EventDigestPayload<'a> {
    event_id: &'a str,
    sequence: u64,
    kind: BatteryLifecycleEventKind,
    physical_battery_id: &'a str,
    passport_id: &'a Option<String>,
    successor_passport_id: &'a Option<String>,
    resulting_configuration_digest: &'a str,
    actor_ref: &'a str,
    responsible_operator_after: &'a Option<String>,
    occurred_at_ms: u64,
    recorded_at_ms: u64,
    previous_event_digest: &'a Option<String>,
    evidence: &'a [BatteryEvidenceReference],
    revalidation_obligation_refs: &'a [String],
}

impl BatteryLifecycleEvent {
    pub fn calculate_digest(&self) -> Result<String, String> {
        let payload = EventDigestPayload {
            event_id: &self.event_id,
            sequence: self.sequence,
            kind: self.kind,
            physical_battery_id: &self.physical_battery_id,
            passport_id: &self.passport_id,
            successor_passport_id: &self.successor_passport_id,
            resulting_configuration_digest: &self.resulting_configuration_digest,
            actor_ref: &self.actor_ref,
            responsible_operator_after: &self.responsible_operator_after,
            occurred_at_ms: self.occurred_at_ms,
            recorded_at_ms: self.recorded_at_ms,
            previous_event_digest: &self.previous_event_digest,
            evidence: &self.evidence,
            revalidation_obligation_refs: &self.revalidation_obligation_refs,
        };
        let bytes = serde_json::to_vec(&payload)
            .map_err(|error| format!("event canonicalization failed: {error}"))?;
        Ok(sha256_hex(&bytes))
    }

    pub fn seal(&mut self) -> Result<(), String> {
        self.event_digest = self.calculate_digest()?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryPassportLineage {
    pub physical_battery_id: String,
    pub model_id: String,
    pub category: BatteryCategory,
    pub events: Vec<BatteryLifecycleEvent>,
    pub passports: Vec<BatteryPassportRecord>,
}

fn issue_event_matches(
    record: &BatteryPassportRecord,
    event: &BatteryLifecycleEvent,
) -> bool {
    if record.issuing_event_digest != event.event_digest
        || record.issued_at_ms != event.occurred_at_ms
        || record.issuing_operator_ref != event.actor_ref
        || record.physical_battery_id != event.physical_battery_id
        || record.configuration_digest != event.resulting_configuration_digest
    {
        return false;
    }

    match record.issue_kind {
        PassportIssueKind::InitialMarketPlacement => {
            event.kind == BatteryLifecycleEventKind::PlacedOnMarket
                && record.predecessor_passport_id.is_none()
                && event.passport_id.as_deref() == Some(record.passport_id.as_str())
                && event.successor_passport_id.is_none()
        }
        PassportIssueKind::Remanufacture => {
            event.kind == BatteryLifecycleEventKind::Remanufactured
                && record.predecessor_passport_id.as_ref() == event.passport_id.as_ref()
                && event.successor_passport_id.as_deref() == Some(record.passport_id.as_str())
        }
        PassportIssueKind::Repurpose => {
            event.kind == BatteryLifecycleEventKind::Repurposed
                && record.predecessor_passport_id.as_ref() == event.passport_id.as_ref()
                && event.successor_passport_id.as_deref() == Some(record.passport_id.as_str())
        }
    }
}

/// Validate physical identity, event-chain integrity, passport succession,
/// configuration transitions and terminal recycling semantics.
pub fn validate_battery_passport_lineage(
    lineage: &BatteryPassportLineage,
) -> Result<(), String> {
    if !nonempty(&lineage.physical_battery_id) || !nonempty(&lineage.model_id) {
        return Err("physical battery identity and model identity are required".to_owned());
    }
    if lineage.events.is_empty() || lineage.passports.is_empty() {
        return Err("battery lineage requires events and at least one passport".to_owned());
    }

    let mut records = BTreeMap::new();
    for record in &lineage.passports {
        if !nonempty(&record.passport_id)
            || !nonempty(&record.model_id)
            || !nonempty(&record.issuing_operator_ref)
            || !nonempty(&record.registry_version)
            || record.physical_battery_id != lineage.physical_battery_id
            || record.model_id != lineage.model_id
            || !valid_digest(&record.configuration_digest)
            || !valid_digest(&record.issuing_event_digest)
            || record.issued_at_ms == 0
        {
            return Err(format!("passport record {} has invalid identity or provenance", record.passport_id));
        }
        if records.insert(record.passport_id.as_str(), record).is_some() {
            return Err(format!("duplicate passport identity {}", record.passport_id));
        }
    }

    let mut event_ids = BTreeSet::new();
    let mut used_issue_events = BTreeSet::new();
    let mut previous_digest: Option<String> = None;
    let mut previous_recorded_at_ms = 0u64;
    let mut current_passport_id: Option<String> = None;
    let mut current_configuration_digest: Option<String> = None;
    let mut expected_status: BTreeMap<String, BatteryPassportStatus> = BTreeMap::new();
    let mut recycled = false;

    for (index, event) in lineage.events.iter().enumerate() {
        if event.sequence != index as u64 {
            return Err("event sequence must be contiguous and zero-based".to_owned());
        }
        if !nonempty(&event.event_id)
            || !event_ids.insert(event.event_id.clone())
            || event.physical_battery_id != lineage.physical_battery_id
            || !nonempty(&event.actor_ref)
            || !valid_digest(&event.resulting_configuration_digest)
            || event.occurred_at_ms == 0
            || event.recorded_at_ms < event.occurred_at_ms
            || event.recorded_at_ms < previous_recorded_at_ms
            || event.evidence.is_empty()
        {
            return Err(format!("event {} has invalid identity, time or configuration", event.event_id));
        }
        if event.previous_event_digest != previous_digest {
            return Err(format!("event {} does not extend the preceding event digest", event.event_id));
        }
        if event.event_digest != event.calculate_digest()? || !valid_digest(&event.event_digest) {
            return Err(format!("event {} digest does not match its canonical payload", event.event_id));
        }
        for evidence in &event.evidence {
            if !nonempty(&evidence.reference)
                || !nonempty(&evidence.evidence_class)
                || !valid_digest(&evidence.digest)
            {
                return Err(format!("event {} contains invalid evidence reference", event.event_id));
            }
        }
        if recycled {
            return Err("no lifecycle events may follow the recycling closure event".to_owned());
        }

        let configuration_changed = current_configuration_digest
            .as_ref()
            .is_some_and(|current| current != &event.resulting_configuration_digest);
        if configuration_changed
            && !matches!(
                event.kind,
                BatteryLifecycleEventKind::SimpleRepair
                    | BatteryLifecycleEventKind::Remanufactured
                    | BatteryLifecycleEventKind::Repurposed
                    | BatteryLifecycleEventKind::ConfigurationChanged
            )
        {
            return Err(format!(
                "event {} changes configuration without an eligible change event",
                event.event_id
            ));
        }
        if configuration_changed && event.revalidation_obligation_refs.is_empty() {
            return Err(format!(
                "event {} changes configuration without revalidation obligations",
                event.event_id
            ));
        }
        if event.revalidation_obligation_refs.iter().any(|reference| !nonempty(reference)) {
            return Err(format!("event {} has an empty revalidation obligation reference", event.event_id));
        }

        match event.kind {
            BatteryLifecycleEventKind::Manufactured => {
                if current_passport_id.is_some()
                    || event.passport_id.is_some()
                    || event.successor_passport_id.is_some()
                {
                    return Err("manufacture event must precede initial passport issuance".to_owned());
                }
            }
            BatteryLifecycleEventKind::PlacedOnMarket => {
                if current_passport_id.is_some() || event.successor_passport_id.is_some() {
                    return Err("initial market placement can issue only the first passport".to_owned());
                }
                let passport_id = event.passport_id.as_ref()
                    .ok_or_else(|| "market placement must identify the initial passport".to_owned())?;
                let record = records.get(passport_id.as_str())
                    .ok_or_else(|| "initial passport record is missing".to_owned())?;
                if record.issue_kind != PassportIssueKind::InitialMarketPlacement
                    || record.predecessor_passport_id.is_some()
                    || !issue_event_matches(record, event)
                {
                    return Err("initial passport must be bound to its market-placement event".to_owned());
                }
                if !used_issue_events.insert(record.passport_id.clone()) {
                    return Err("passport record is issued more than once".to_owned());
                }
                current_passport_id = Some(passport_id.clone());
                current_configuration_digest = Some(event.resulting_configuration_digest.clone());
                expected_status.insert(passport_id.clone(), BatteryPassportStatus::Active);
            }
            BatteryLifecycleEventKind::Remanufactured
            | BatteryLifecycleEventKind::Repurposed => {
                let predecessor = current_passport_id.as_ref()
                    .ok_or_else(|| "successor passport event requires an active predecessor".to_owned())?;
                if event.passport_id.as_ref() != Some(predecessor) {
                    return Err("remanufacture/repurpose event must name the current predecessor passport".to_owned());
                }
                if current_configuration_digest.as_deref()
                    == Some(event.resulting_configuration_digest.as_str())
                {
                    return Err("remanufacture/repurpose must produce a new engineering configuration".to_owned());
                }
                let successor = event.successor_passport_id.as_ref()
                    .filter(|value| nonempty(value))
                    .ok_or_else(|| "remanufacture/repurpose must name the new passport".to_owned())?;
                if successor == predecessor {
                    return Err("successor passport must have a new identity".to_owned());
                }
                let record = records.get(successor.as_str())
                    .ok_or_else(|| "successor passport record is missing".to_owned())?;
                if !issue_event_matches(record, event) {
                    return Err("successor passport must link to the triggering remanufacture/repurpose event".to_owned());
                }
                if !used_issue_events.insert(record.passport_id.clone()) {
                    return Err("successor passport record is issued more than once".to_owned());
                }
                expected_status.insert(predecessor.clone(), BatteryPassportStatus::Superseded);
                expected_status.insert(successor.clone(), BatteryPassportStatus::Active);
                current_passport_id = Some(successor.clone());
                current_configuration_digest = Some(event.resulting_configuration_digest.clone());
            }
            BatteryLifecycleEventKind::Inspection
            | BatteryLifecycleEventKind::RoutineService
            | BatteryLifecycleEventKind::SimpleRepair
            | BatteryLifecycleEventKind::ConfigurationChanged
            | BatteryLifecycleEventKind::ResponsibilityTransferred
            | BatteryLifecycleEventKind::Quarantined
            | BatteryLifecycleEventKind::EndOfLife
            | BatteryLifecycleEventKind::Recycled => {
                let current = current_passport_id.as_ref()
                    .ok_or_else(|| "lifecycle event requires an issued passport".to_owned())?;
                if event.passport_id.as_ref() != Some(current)
                    || event.successor_passport_id.is_some()
                {
                    return Err(format!(
                        "event {} must retain the current passport identity",
                        event.event_id
                    ));
                }
                if let Some(current_config) = &current_configuration_digest {
                    if !configuration_changed
                        && event.resulting_configuration_digest != *current_config
                    {
                        return Err("event configuration continuity check failed".to_owned());
                    }
                }
                if event.kind == BatteryLifecycleEventKind::Recycled {
                    recycled = true;
                    expected_status.insert(current.clone(), BatteryPassportStatus::ClosedByRecycling);
                } else if configuration_changed {
                    current_configuration_digest = Some(event.resulting_configuration_digest.clone());
                }
            }
        }

        if event.kind == BatteryLifecycleEventKind::PlacedOnMarket
            && event.responsible_operator_after.as_ref().is_none_or(|value| !nonempty(value))
        {
            return Err("market placement must establish the responsible economic operator".to_owned());
        }
        if event.kind == BatteryLifecycleEventKind::ResponsibilityTransferred
            && event.responsible_operator_after.as_ref().is_none_or(|value| !nonempty(value))
        {
            return Err("responsibility transfer must identify the succeeding operator".to_owned());
        }
        if matches!(
            event.kind,
            BatteryLifecycleEventKind::Remanufactured | BatteryLifecycleEventKind::Repurposed
        ) && event.revalidation_obligation_refs.is_empty()
        {
            return Err("remanufacture/repurpose requires revalidation obligations".to_owned());
        }

        if !matches!(
            event.kind,
            BatteryLifecycleEventKind::PlacedOnMarket
                | BatteryLifecycleEventKind::ResponsibilityTransferred
        ) && event.responsible_operator_after.is_some()
        {
            return Err("responsible operator may change only at placement or an explicit transfer event".to_owned());
        }
        previous_recorded_at_ms = event.recorded_at_ms;
        previous_digest = Some(event.event_digest.clone());
    }

    if used_issue_events.len() != records.len() {
        return Err("every passport record must be issued by exactly one lifecycle event".to_owned());
    }
    if current_passport_id.is_none() {
        return Err("battery lineage has no passport in force".to_owned());
    }
    for record in &lineage.passports {
        let expected = expected_status.get(&record.passport_id)
            .ok_or_else(|| format!("passport {} has no lifecycle status derivation", record.passport_id))?;
        if record.status != *expected {
            return Err(format!("passport {} status contradicts lifecycle lineage", record.passport_id));
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PassportAudience {
    GeneralPublic,
    NotifiedBody,
    MarketSurveillanceAuthority,
    EuropeanCommission,
    LegitimateInterestHolder,
    ResponsibleOperator,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PassportAccessClass {
    Public,
    AuthorityRestricted,
    LegitimateInterest,
    OperatorConfidential,
}

impl PassportAccessClass {
    pub fn allows(self, audience: PassportAudience) -> bool {
        if audience == PassportAudience::ResponsibleOperator {
            return true;
        }
        match self {
            Self::Public => true,
            Self::AuthorityRestricted => matches!(
                audience,
                PassportAudience::NotifiedBody
                    | PassportAudience::MarketSurveillanceAuthority
                    | PassportAudience::EuropeanCommission
            ),
            Self::LegitimateInterest => matches!(
                audience,
                PassportAudience::NotifiedBody
                    | PassportAudience::MarketSurveillanceAuthority
                    | PassportAudience::EuropeanCommission
                    | PassportAudience::LegitimateInterestHolder
            ),
            Self::OperatorConfidential => audience == PassportAudience::ResponsibleOperator,
        }
    }

    /// An instance may be more restrictive than its registry default, never
    /// more permissive. Access classes are checked as sets, not as a scalar rank.
    fn is_no_broader_than(self, default: Self) -> bool {
        let audiences = [
            PassportAudience::GeneralPublic,
            PassportAudience::NotifiedBody,
            PassportAudience::MarketSurveillanceAuthority,
            PassportAudience::EuropeanCommission,
            PassportAudience::LegitimateInterestHolder,
            PassportAudience::ResponsibleOperator,
        ];
        audiences
            .iter()
            .all(|audience| !self.allows(*audience) || default.allows(*audience))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataPointApplicability {
    Applicable,
    NotApplicable { reason_ref: String },
    Unknown { reason_ref: String },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryPassportDataPointDefinition {
    pub point_id: String,
    pub legal_source_ref: String,
    pub applicable_categories: Vec<BatteryCategory>,
    pub default_access: PassportAccessClass,
    pub dynamic: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryPassportRegistry {
    pub version: String,
    pub source_ref: String,
    pub source_digest: String,
    pub published_at_ms: u64,
    pub definitions: Vec<BatteryPassportDataPointDefinition>,
}

pub fn validate_battery_passport_registry(
    registry: &BatteryPassportRegistry,
) -> Result<(), String> {
    if !nonempty(&registry.version)
        || !nonempty(&registry.source_ref)
        || !valid_digest(&registry.source_digest)
        || registry.published_at_ms == 0
        || registry.definitions.is_empty()
    {
        return Err("registry identity, source digest, publication time and definitions are required".to_owned());
    }
    let mut ids = BTreeSet::new();
    for definition in &registry.definitions {
        if !nonempty(&definition.point_id)
            || !ids.insert(definition.point_id.clone())
            || !nonempty(&definition.legal_source_ref)
            || definition.applicable_categories.is_empty()
        {
            return Err("registry contains an invalid or duplicate data-point definition".to_owned());
        }
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryPassportDataPoint {
    pub point_id: String,
    pub applicability: DataPointApplicability,
    pub value: Option<String>,
    pub source_record_digest: Option<String>,
    pub access: PassportAccessClass,
    pub updated_at_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BatteryPassportProjection {
    pub passport_id: String,
    pub audience: PassportAudience,
    pub registry_version: String,
    pub policy_version: String,
    pub visible_data_points: Vec<BatteryPassportDataPoint>,
    pub projection_digest: String,
}

#[derive(Debug, Clone, Serialize)]
struct ProjectionDigestPayload<'a> {
    passport_id: &'a str,
    audience: PassportAudience,
    registry_version: &'a str,
    registry_digest: &'a str,
    policy_version: &'a str,
    visible_data_points: &'a [BatteryPassportDataPoint],
}

/// Build an audience-filtered projection. Values are included only when the
/// point is applicable, has source evidence, and the audience is authorized.
/// Unknown/not-applicable points remain explicit but carry no value.
pub fn build_battery_passport_projection(
    passport_id: &str,
    category: BatteryCategory,
    audience: PassportAudience,
    registry: &BatteryPassportRegistry,
    data_points: &[BatteryPassportDataPoint],
) -> Result<BatteryPassportProjection, String> {
    if !nonempty(passport_id) {
        return Err("passport identity is required".to_owned());
    }
    validate_battery_passport_registry(registry)?;
    let definitions: BTreeMap<&str, &BatteryPassportDataPointDefinition> = registry
        .definitions
        .iter()
        .map(|definition| (definition.point_id.as_str(), definition))
        .collect();
    let mut seen = BTreeSet::new();
    let mut visible = Vec::new();

    for point in data_points {
        if !nonempty(&point.point_id) || !seen.insert(point.point_id.clone()) {
            return Err("projection data points contain an empty or duplicate identity".to_owned());
        }
        let definition = definitions
            .get(point.point_id.as_str())
            .ok_or_else(|| format!("data point {} is absent from registry {}", point.point_id, registry.version))?;
        if !definition.applicable_categories.contains(&category) {
            return Err(format!(
                "data point {} is not declared for battery category {:?}",
                point.point_id, category
            ));
        }
        if !point.access.is_no_broader_than(definition.default_access) {
            return Err(format!(
                "data point {} attempts to broaden the registry access class",
                point.point_id
            ));
        }
        if point.updated_at_ms == 0 {
            return Err(format!("data point {} has no update timestamp", point.point_id));
        }

        match &point.applicability {
            DataPointApplicability::Applicable => {
                if point.value.as_ref().is_none_or(|value| !nonempty(value))
                    || point.source_record_digest.as_deref().is_none_or(|digest| !valid_digest(digest))
                {
                    return Err(format!(
                        "applicable data point {} requires a value and source-record digest",
                        point.point_id
                    ));
                }
            }
            DataPointApplicability::NotApplicable { reason_ref }
            | DataPointApplicability::Unknown { reason_ref } => {
                if !nonempty(reason_ref)
                    || point.value.is_some()
                    || point.source_record_digest.is_some()
                {
                    return Err(format!(
                        "non-applicable/unknown data point {} must carry a reason and no value",
                        point.point_id
                    ));
                }
            }
        }

        if point.access.allows(audience) {
            visible.push(point.clone());
        }
    }
    visible.sort_by(|left, right| left.point_id.cmp(&right.point_id));

    let payload = ProjectionDigestPayload {
        passport_id,
        audience,
        registry_version: &registry.version,
        registry_digest: &registry.source_digest,
        policy_version: PROJECTION_POLICY_VERSION,
        visible_data_points: &visible,
    };
    let bytes = serde_json::to_vec(&payload)
        .map_err(|error| format!("projection canonicalization failed: {error}"))?;
    Ok(BatteryPassportProjection {
        passport_id: passport_id.to_owned(),
        audience,
        registry_version: registry.version.clone(),
        policy_version: PROJECTION_POLICY_VERSION.to_owned(),
        visible_data_points: visible,
        projection_digest: sha256_hex(&bytes),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(seed: u8) -> String {
        format!("{:02x}", seed).repeat(32)
    }

    fn evidence(seed: u8) -> BatteryEvidenceReference {
        BatteryEvidenceReference {
            reference: format!("evidence://{seed}"),
            digest: digest(seed),
            evidence_class: "synthetic_fixture".to_owned(),
        }
    }

    fn event(
        sequence: u64,
        kind: BatteryLifecycleEventKind,
        passport_id: Option<&str>,
        successor_passport_id: Option<&str>,
        config_digest: &str,
        previous: Option<&str>,
    ) -> BatteryLifecycleEvent {
        let mut value = BatteryLifecycleEvent {
            event_id: format!("event-{sequence}"),
            sequence,
            kind,
            physical_battery_id: "battery-001".to_owned(),
            passport_id: passport_id.map(str::to_owned),
            successor_passport_id: successor_passport_id.map(str::to_owned),
            resulting_configuration_digest: config_digest.to_owned(),
            actor_ref: "did:example:operator".to_owned(),
            responsible_operator_after: None,
            occurred_at_ms: 1_000 + sequence,
            recorded_at_ms: 1_000 + sequence,
            previous_event_digest: previous.map(str::to_owned),
            evidence: vec![evidence(sequence as u8 + 1)],
            revalidation_obligation_refs: Vec::new(),
            event_digest: String::new(),
        };
        value.seal().unwrap();
        value
    }

    fn make_registry() -> BatteryPassportRegistry {
        BatteryPassportRegistry {
            version: "commission-guidance-2026-08-21".to_owned(),
            source_ref: "https://single-market-economy.ec.europa.eu/news/guidance-support-preparations-digital-batteries-passport-2026-08-21_en".to_owned(),
            source_digest: digest(90),
            published_at_ms: 1,
            definitions: vec![
                BatteryPassportDataPointDefinition {
                    point_id: "battery_model".to_owned(),
                    legal_source_ref: "Regulation 2023/1542 Annex XIII".to_owned(),
                    applicable_categories: vec![BatteryCategory::ElectricVehicle],
                    default_access: PassportAccessClass::Public,
                    dynamic: false,
                },
                BatteryPassportDataPointDefinition {
                    point_id: "cell_operating_history".to_owned(),
                    legal_source_ref: "Regulation 2023/1542 Annex XIII".to_owned(),
                    applicable_categories: vec![BatteryCategory::ElectricVehicle],
                    default_access: PassportAccessClass::LegitimateInterest,
                    dynamic: true,
                },
                BatteryPassportDataPointDefinition {
                    point_id: "operator_confidential_process".to_owned(),
                    legal_source_ref: "contractual source".to_owned(),
                    applicable_categories: vec![BatteryCategory::ElectricVehicle],
                    default_access: PassportAccessClass::OperatorConfidential,
                    dynamic: true,
                },
            ],
        }
    }

    fn make_lineage() -> BatteryPassportLineage {
        let mut placed = event(
            1,
            BatteryLifecycleEventKind::PlacedOnMarket,
            Some("passport-1"),
            None,
            &digest(1),
            Some(&digest(0)),
        );
        placed.actor_ref = "operator-a".to_owned();
        placed.responsible_operator_after = Some("operator-a".to_owned());
        placed.seal().unwrap();
        let mut repaired = event(
            2,
            BatteryLifecycleEventKind::SimpleRepair,
            Some("passport-1"),
            None,
            &digest(1),
            Some(&placed.event_digest),
        );
        repaired.revalidation_obligation_refs = vec!["revalidation://inspection-01".to_owned()];
        repaired.seal().unwrap();
        let mut remanufactured = event(
            3,
            BatteryLifecycleEventKind::Remanufactured,
            Some("passport-1"),
            Some("passport-2"),
            &digest(2),
            Some(&repaired.event_digest),
        );
        remanufactured.actor_ref = "operator-b".to_owned();
        remanufactured.revalidation_obligation_refs = vec!["revalidation://pack-test-02".to_owned()];
        remanufactured.seal().unwrap();

        let record_one = BatteryPassportRecord {
            passport_id: "passport-1".to_owned(),
            physical_battery_id: "battery-001".to_owned(),
            model_id: "model-001".to_owned(),
            predecessor_passport_id: None,
            issue_kind: PassportIssueKind::InitialMarketPlacement,
            issued_at_ms: placed.occurred_at_ms,
            issuing_operator_ref: "operator-a".to_owned(),
            registry_version: "commission-guidance-2026-08-21".to_owned(),
            configuration_digest: digest(1),
            issuing_event_digest: placed.event_digest.clone(),
            status: BatteryPassportStatus::Superseded,
        };
        let record_two = BatteryPassportRecord {
            passport_id: "passport-2".to_owned(),
            physical_battery_id: "battery-001".to_owned(),
            model_id: "model-001".to_owned(),
            predecessor_passport_id: Some("passport-1".to_owned()),
            issue_kind: PassportIssueKind::Remanufacture,
            issued_at_ms: remanufactured.occurred_at_ms,
            issuing_operator_ref: "operator-b".to_owned(),
            registry_version: "commission-guidance-2026-08-21".to_owned(),
            configuration_digest: digest(2),
            issuing_event_digest: remanufactured.event_digest.clone(),
            status: BatteryPassportStatus::Active,
        };

        // The first lifecycle event is manufacture and precedes passport issuance.
        let manufactured = event(
            0,
            BatteryLifecycleEventKind::Manufactured,
            None,
            None,
            &digest(1),
            None,
        );
        let mut placed = placed;
        placed.sequence = 1;
        placed.previous_event_digest = Some(manufactured.event_digest.clone());
        placed.seal().unwrap();
        let mut repaired = repaired;
        repaired.sequence = 2;
        repaired.previous_event_digest = Some(placed.event_digest.clone());
        repaired.seal().unwrap();
        let mut remanufactured = remanufactured;
        remanufactured.sequence = 3;
        remanufactured.previous_event_digest = Some(repaired.event_digest.clone());
        remanufactured.seal().unwrap();

        let mut record_one = record_one;
        record_one.issuing_event_digest = placed.event_digest.clone();
        let mut record_two = record_two;
        record_two.issuing_event_digest = remanufactured.event_digest.clone();

        BatteryPassportLineage {
            physical_battery_id: "battery-001".to_owned(),
            model_id: "model-001".to_owned(),
            category: BatteryCategory::ElectricVehicle,
            events: vec![manufactured, placed, repaired, remanufactured],
            passports: vec![record_one, record_two],
        }
    }

    #[test]
    fn initial_placement_simple_repair_and_remanufacture_form_linked_lineage() {
        let lineage = make_lineage();
        assert!(validate_battery_passport_lineage(&lineage).is_ok());
        assert_eq!(lineage.passports.len(), 2);
        assert_eq!(
            lineage.passports[1].predecessor_passport_id.as_deref(),
            Some("passport-1")
        );
    }

    #[test]
    fn simple_repair_does_not_issue_a_new_passport() {
        let lineage = make_lineage();
        let repair = &lineage.events[2];
        assert_eq!(repair.kind, BatteryLifecycleEventKind::SimpleRepair);
        assert_eq!(repair.passport_id.as_deref(), Some("passport-1"));
        assert_eq!(repair.successor_passport_id, None);
    }

    #[test]
    fn repurpose_requires_a_new_passport_with_prior_passport_link() {
        let mut lineage = make_lineage();
        let event = &mut lineage.events[3];
        event.kind = BatteryLifecycleEventKind::Repurposed;
        event.seal().unwrap();
        let record = &mut lineage.passports[1];
        record.issue_kind = PassportIssueKind::Repurpose;
        record.issuing_event_digest = event.event_digest.clone();
        assert!(validate_battery_passport_lineage(&lineage).is_ok());
    }

    #[test]
    fn simple_repair_cannot_issue_a_successor_passport() {
        let mut lineage = make_lineage();
        let repair = &mut lineage.events[2];
        repair.successor_passport_id = Some("passport-forged".to_owned());
        repair.seal().unwrap();
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn remanufacture_requires_a_changed_configuration_digest() {
        let mut lineage = make_lineage();
        lineage.events[3].resulting_configuration_digest = digest(1);
        lineage.events[3].seal().unwrap();
        lineage.passports[1].configuration_digest = digest(1);
        lineage.passports[1].issuing_event_digest = lineage.events[3].event_digest.clone();
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn missing_successor_passport_record_is_rejected() {
        let mut lineage = make_lineage();
        lineage.passports.pop();
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn successor_without_predecessor_link_is_rejected() {
        let mut lineage = make_lineage();
        lineage.passports[1].predecessor_passport_id = None;
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn configuration_change_without_revalidation_is_rejected() {
        let mut lineage = make_lineage();
        let repair = &mut lineage.events[2];
        repair.resulting_configuration_digest = digest(9);
        repair.revalidation_obligation_refs.clear();
        repair.seal().unwrap();
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn tampered_event_payload_or_predecessor_digest_is_rejected() {
        let mut lineage = make_lineage();
        lineage.events[2].actor_ref = "did:example:forged".to_owned();
        assert!(validate_battery_passport_lineage(&lineage).is_err());

        let mut lineage = make_lineage();
        lineage.events[2].previous_event_digest = Some(digest(77));
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn recycling_closes_passport_projection_but_keeps_event_history() {
        let mut lineage = make_lineage();
        let prior = lineage.events.last().unwrap().event_digest.clone();
        let mut recycled = event(
            4,
            BatteryLifecycleEventKind::Recycled,
            Some("passport-2"),
            None,
            &digest(2),
            Some(&prior),
        );
        recycled.seal().unwrap();
        lineage.events.push(recycled);
        lineage.passports[1].status = BatteryPassportStatus::ClosedByRecycling;
        assert!(validate_battery_passport_lineage(&lineage).is_ok());
        assert_eq!(lineage.events.len(), 5);
        assert_eq!(lineage.passports.len(), 2);
    }

    #[test]
    fn lifecycle_events_after_recycling_are_rejected() {
        let mut lineage = make_lineage();
        let prior = lineage.events.last().unwrap().event_digest.clone();
        let mut recycled = event(
            4,
            BatteryLifecycleEventKind::Recycled,
            Some("passport-2"),
            None,
            &digest(2),
            Some(&prior),
        );
        recycled.seal().unwrap();
        let mut inspection = event(
            5,
            BatteryLifecycleEventKind::Inspection,
            Some("passport-2"),
            None,
            &digest(2),
            Some(&recycled.event_digest),
        );
        inspection.seal().unwrap();
        lineage.events.extend([recycled, inspection]);
        lineage.passports[1].status = BatteryPassportStatus::ClosedByRecycling;
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn public_projection_excludes_legitimate_interest_and_operator_confidential_values() {
        let registry = make_registry();
        let points = vec![
            BatteryPassportDataPoint {
                point_id: "battery_model".to_owned(),
                applicability: DataPointApplicability::Applicable,
                value: Some("EV battery model X".to_owned()),
                source_record_digest: Some(digest(1)),
                access: PassportAccessClass::Public,
                updated_at_ms: 1,
            },
            BatteryPassportDataPoint {
                point_id: "cell_operating_history".to_owned(),
                applicability: DataPointApplicability::Applicable,
                value: Some("private telemetry".to_owned()),
                source_record_digest: Some(digest(2)),
                access: PassportAccessClass::LegitimateInterest,
                updated_at_ms: 2,
            },
            BatteryPassportDataPoint {
                point_id: "operator_confidential_process".to_owned(),
                applicability: DataPointApplicability::Applicable,
                value: Some("proprietary process".to_owned()),
                source_record_digest: Some(digest(3)),
                access: PassportAccessClass::OperatorConfidential,
                updated_at_ms: 3,
            },
        ];
        let projection = build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).unwrap();
        assert_eq!(projection.visible_data_points.len(), 1);
        assert_eq!(projection.visible_data_points[0].point_id, "battery_model");
    }

    #[test]
    fn registry_access_cannot_be_broadened_by_a_data_point() {
        let registry = make_registry();
        let mut points = vec![BatteryPassportDataPoint {
            point_id: "cell_operating_history".to_owned(),
            applicability: DataPointApplicability::Applicable,
            value: Some("secret".to_owned()),
            source_record_digest: Some(digest(1)),
            access: PassportAccessClass::Public,
            updated_at_ms: 1,
        }];
        assert!(build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).is_err());
        points[0].access = PassportAccessClass::OperatorConfidential;
        assert!(build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).is_ok());
    }

    #[test]
    fn unknown_data_point_and_fabricated_unknown_value_are_rejected() {
        let registry = make_registry();
        let mut points = vec![BatteryPassportDataPoint {
            point_id: "not-in-registry".to_owned(),
            applicability: DataPointApplicability::Unknown {
                reason_ref: "reason://not-applicable-yet".to_owned(),
            },
            value: None,
            source_record_digest: None,
            access: PassportAccessClass::Public,
            updated_at_ms: 1,
        }];
        assert!(build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).is_err());
        points[0].point_id = "battery_model".to_owned();
        points[0].value = Some("invented".to_owned());
        assert!(build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).is_err());
    }

    #[test]
    fn projection_receipt_is_stable_under_data_point_order() {
        let registry = make_registry();
        let mut points = vec![
            BatteryPassportDataPoint {
                point_id: "battery_model".to_owned(),
                applicability: DataPointApplicability::Applicable,
                value: Some("model".to_owned()),
                source_record_digest: Some(digest(1)),
                access: PassportAccessClass::Public,
                updated_at_ms: 1,
            },
            BatteryPassportDataPoint {
                point_id: "cell_operating_history".to_owned(),
                applicability: DataPointApplicability::Unknown {
                    reason_ref: "reason://no-current-data".to_owned(),
                },
                value: None,
                source_record_digest: None,
                access: PassportAccessClass::LegitimateInterest,
                updated_at_ms: 2,
            },
        ];
        let first = build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).unwrap();
        points.reverse();
        let second = build_battery_passport_projection(
            "passport-1",
            BatteryCategory::ElectricVehicle,
            PassportAudience::GeneralPublic,
            &registry,
            &points,
        ).unwrap();
        assert_eq!(first.projection_digest, second.projection_digest);
        assert_eq!(first, second);
    }

    #[test]
    fn serde_roundtrip_preserves_registry_version_and_visibility() {
        let registry = make_registry();
        let json = serde_json::to_string(&registry).unwrap();
        let restored: BatteryPassportRegistry = serde_json::from_str(&json).unwrap();
        assert_eq!(restored, registry);
        assert_eq!(restored.version, "commission-guidance-2026-08-21");
    }

    #[test]
    fn responsible_operator_can_access_restricted_passport_data() {
        assert!(PassportAccessClass::Public.allows(PassportAudience::ResponsibleOperator));
        assert!(PassportAccessClass::AuthorityRestricted.allows(PassportAudience::ResponsibleOperator));
        assert!(PassportAccessClass::LegitimateInterest.allows(PassportAudience::ResponsibleOperator));
        assert!(PassportAccessClass::OperatorConfidential.allows(PassportAudience::ResponsibleOperator));
    }

    #[test]
    fn event_record_time_cannot_move_backwards() {
        let mut lineage = make_lineage();
        lineage.events[3].recorded_at_ms = lineage.events[2].recorded_at_ms - 1;
        lineage.events[3].seal().unwrap();
        assert!(validate_battery_passport_lineage(&lineage).is_err());
    }

    #[test]
    fn registry_rejects_duplicate_data_point_ids() {
        let mut registry = make_registry();
        registry.definitions.push(registry.definitions[0].clone());
        assert!(validate_battery_passport_registry(&registry).is_err());
    }
}
