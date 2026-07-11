// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Purity Integrity Zome
//! Water quality monitoring, contamination alerts, and remediation tracking

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ============================================================================
// QUALITY READINGS
// ============================================================================

/// A water quality reading from a source
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct QualityReading {
    /// Water source this reading is for
    pub source_hash: ActionHash,
    /// Agent who took the sample
    pub sampler: AgentPubKey,
    /// When the sample was taken
    pub timestamp: Timestamp,
    /// Water temperature in Celsius
    pub temperature_celsius: Option<f32>,
    /// Turbidity in NTU (Nephelometric Turbidity Units)
    pub turbidity_ntu: Option<f32>,
    /// pH level (0-14)
    pub ph: Option<f32>,
    /// Total dissolved solids in parts per million
    pub tds_ppm: Option<f32>,
    /// Dissolved oxygen in mg/L
    pub dissolved_oxygen_mg_l: Option<f32>,
    /// Nitrate concentration in mg/L
    pub nitrates_mg_l: Option<f32>,
    /// Arsenic concentration in micrograms per liter
    pub arsenic_ug_l: Option<f32>,
    /// Lead concentration in micrograms per liter
    pub lead_ug_l: Option<f32>,
    /// Total coliform count in colony-forming units
    pub total_coliform_cfu: Option<u32>,
    /// E. coli count in colony-forming units
    pub e_coli_cfu: Option<u32>,
    /// Free chlorine residual in mg/L
    pub chlorine_mg_l: Option<f32>,
    /// Calculated potability score (0.0-1.0)
    pub potability_score: f32,
    /// Whether this reading meets WHO drinking water standards
    pub meets_who_standards: bool,
    /// Whether this reading meets EPA drinking water standards
    pub meets_epa_standards: bool,
}

// ============================================================================
// CONTAMINATION ALERTS
// ============================================================================

/// Severity of a contamination alert
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AlertSeverity {
    /// Informational, no immediate danger
    Advisory,
    /// Caution, take precautions
    Warning,
    /// Immediate danger, do not consume
    Emergency,
}

/// Type of contamination
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AlertType {
    /// Chemical contamination (heavy metals, pesticides)
    Chemical,
    /// Biological contamination (bacteria, viruses)
    Biological,
    /// Physical contamination (turbidity, sediment)
    Physical,
    /// Radiological contamination
    Radiological,
}

/// A contamination alert for a water source
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ContaminationAlert {
    /// Affected water source
    pub source_hash: ActionHash,
    /// How serious the contamination is
    pub severity: AlertSeverity,
    /// Name of the contaminant detected
    pub contaminant: String,
    /// Measured value of the contaminant
    pub measured_value: f32,
    /// Safe threshold value
    pub threshold_value: f32,
    /// Type of contamination
    pub alert_type: AlertType,
    /// Agent who reported the alert
    pub reported_by: AgentPubKey,
    /// When the alert was raised
    pub reported_at: Timestamp,
    /// When the alert was resolved (if resolved)
    pub resolved_at: Option<Timestamp>,
    /// Link to remediation action if taken
    pub remediation_hash: Option<ActionHash>,
}

// ============================================================================
// REMEDIATION
// ============================================================================

/// A remediation action taken to address contamination
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Remediation {
    /// The contamination alert this addresses
    pub alert_hash: ActionHash,
    /// Description of remediation method used
    pub method: String,
    /// When remediation started
    pub started_at: Timestamp,
    /// When remediation completed (if completed)
    pub completed_at: Option<Timestamp>,
    /// Agent who verified remediation effectiveness
    pub verified_by: Option<AgentPubKey>,
    /// Post-treatment quality reading
    pub post_treatment_reading: Option<ActionHash>,
    /// Estimated cost in smallest currency unit
    pub cost_estimate: Option<u64>,
    /// Additional notes
    pub notes: String,
}

// ============================================================================
// ENTRY & LINK TYPE REGISTRATION
// ============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    QualityReading(QualityReading),
    ContaminationAlert(ContaminationAlert),
    Remediation(Remediation),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Source to its quality readings
    SourceToReading,
    /// Sampler to their readings
    SamplerToReading,
    /// All active alerts anchor
    ActiveAlerts,
    /// Source to its alerts
    SourceToAlert,
    /// Alert to its remediation
    AlertToRemediation,
    /// All readings anchor (for global queries)
    AllReadings,
}

// ============================================================================
// VALIDATION
// ============================================================================

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::QualityReading(reading) => validate_create_reading(action, reading),
                EntryTypes::ContaminationAlert(alert) => validate_create_alert(action, alert),
                EntryTypes::Remediation(remediation) => {
                    validate_create_remediation(action, remediation)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ContaminationAlert(alert) => {
                    validate_update_alert(action, alert, original_action_hash)
                }
                EntryTypes::Remediation(remediation) => {
                    validate_update_remediation(action, remediation, original_action_hash)
                }
                _ => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::SourceToReading => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SamplerToReading => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ActiveAlerts => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SourceToAlert => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AlertToRemediation => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllReadings => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_reading(
    action: Create,
    reading: QualityReading,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the reading to its committer -- submit_reading already derives
    // `sampler` from agent_info() coordinator-side with zero user input, so
    // this never rejects a legitimate reading; it's the real DHT-level
    // enforcement a modified coordinator could otherwise bypass (P0
    // author-binding gap).
    if reading.sampler != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Quality reading sampler must be the committing agent (forgery)".to_string(),
        ));
    }

    // Potability score must be in [0, 1]
    if reading.potability_score < 0.0 || reading.potability_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Potability score must be between 0.0 and 1.0".into(),
        ));
    }

    // pH must be in [0, 14] if provided
    if let Some(ph) = reading.ph {
        if ph < 0.0 || ph > 14.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "pH must be between 0 and 14".into(),
            ));
        }
    }

    // Temperature sanity check
    if let Some(temp) = reading.temperature_celsius {
        if temp < -50.0 || temp > 100.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Temperature must be between -50 and 100 Celsius".into(),
            ));
        }
    }

    // Turbidity must be non-negative
    if let Some(turb) = reading.turbidity_ntu {
        if turb < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Turbidity cannot be negative".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_alert(
    action: Create,
    alert: ContaminationAlert,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the alert to its committer -- raise_alert already derives
    // `reported_by` from agent_info() coordinator-side with zero user
    // input, same rationale as validate_create_reading above.
    if alert.reported_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Contamination alert reported_by must be the committing agent (forgery)".to_string(),
        ));
    }

    if alert.contaminant.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Contaminant name cannot be empty".into(),
        ));
    }
    if alert.measured_value < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Measured value cannot be negative".into(),
        ));
    }
    if alert.threshold_value < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Threshold value cannot be negative".into(),
        ));
    }
    // Alert should only be raised if measured exceeds threshold
    if alert.measured_value <= alert.threshold_value {
        return Ok(ValidateCallbackResult::Invalid(
            "Measured value must exceed threshold to raise an alert".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_alert(
    _action: Update,
    _alert: ContaminationAlert,
    _original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: resolve_alert has no restriction on who
    // may resolve a contamination alert today (not gated to reported_by or
    // any water-authority role) -- there's no established identity model to
    // bind against. Same class of gap as mycelix-property's dispute
    // resolution paths (see memory/mycelix_attribution_author_binding_jul8.md).
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_remediation(
    _action: Create,
    remediation: Remediation,
) -> ExternResult<ValidateCallbackResult> {
    // No author-binding possible here: Remediation has no "started_by"
    // identity field at all -- only `verified_by`, which is set later by
    // complete_remediation (bound in validate_update_remediation below).
    if remediation.method.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Remediation method cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_remediation(
    action: Update,
    remediation: Remediation,
    _original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    // Bind verified_by to its committer -- complete_remediation already
    // derives it from agent_info() coordinator-side with zero user input.
    // Only enforced when set (the only coordinator path that populates
    // verified_by is complete_remediation; no other update path exists for
    // this entry type today).
    if let Some(verifier) = &remediation.verified_by {
        if *verifier != action.author {
            return Ok(ValidateCallbackResult::Invalid(
                "Remediation verified_by must be the committing agent (forgery)".to_string(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_reading(sampler: AgentPubKey) -> QualityReading {
        QualityReading {
            source_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            sampler,
            timestamp: Timestamp::from_micros(0),
            temperature_celsius: Some(20.0),
            turbidity_ntu: Some(1.0),
            ph: Some(7.0),
            tds_ppm: None,
            dissolved_oxygen_mg_l: None,
            nitrates_mg_l: None,
            arsenic_ug_l: None,
            lead_ug_l: None,
            total_coliform_cfu: None,
            e_coli_cfu: None,
            chlorine_mg_l: None,
            potability_score: 0.9,
            meets_who_standards: true,
            meets_epa_standards: true,
        }
    }

    #[test]
    fn test_create_reading_valid() {
        let reading = valid_reading(test_action().author);
        let result = validate_create_reading(test_action(), reading).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_reading_sampler_forgery_rejected() {
        let reading = valid_reading(other_agent());
        let result = validate_create_reading(test_action(), reading).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_alert(reported_by: AgentPubKey) -> ContaminationAlert {
        ContaminationAlert {
            source_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            severity: AlertSeverity::Warning,
            contaminant: "Lead".to_string(),
            measured_value: 15.0,
            threshold_value: 10.0,
            alert_type: AlertType::Chemical,
            reported_by,
            reported_at: Timestamp::from_micros(0),
            resolved_at: None,
            remediation_hash: None,
        }
    }

    #[test]
    fn test_create_alert_valid() {
        let alert = valid_alert(test_action().author);
        let result = validate_create_alert(test_action(), alert).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_alert_reported_by_forgery_rejected() {
        let alert = valid_alert(other_agent());
        let result = validate_create_alert(test_action(), alert).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn test_update_action() -> Update {
        Update {
            author: test_action().author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn valid_remediation(verified_by: Option<AgentPubKey>) -> Remediation {
        Remediation {
            alert_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            method: "Activated carbon filtration".to_string(),
            started_at: Timestamp::from_micros(0),
            completed_at: None,
            verified_by,
            post_treatment_reading: None,
            cost_estimate: None,
            notes: String::new(),
        }
    }

    #[test]
    fn test_update_remediation_no_verifier_valid() {
        let remediation = valid_remediation(None);
        let result = validate_update_remediation(
            test_update_action(),
            remediation,
            ActionHash::from_raw_36(vec![0u8; 36]),
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_update_remediation_verifier_valid() {
        let remediation = valid_remediation(Some(test_update_action().author));
        let result = validate_update_remediation(
            test_update_action(),
            remediation,
            ActionHash::from_raw_36(vec![0u8; 36]),
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_update_remediation_verifier_forgery_rejected() {
        let remediation = valid_remediation(Some(other_agent()));
        let result = validate_update_remediation(
            test_update_action(),
            remediation,
            ActionHash::from_raw_36(vec![0u8; 36]),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
