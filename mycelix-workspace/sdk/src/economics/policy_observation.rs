// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Measurement-only economic observations and exact input snapshots.
//!
//! This module records economic signals without deciding what they mean for
//! policy. Observations retain their native units, time period, source,
//! methodology, evidence, uncertainty, and conflict-set identity.
//!
//! No composite score is defined here. A policy engine may interpret these
//! observations later under an explicit policy profile.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Measurement signal represented by an economic observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicObservationSignal {
    PriceLevel,
    PriceGrowth,
    LaborCapacity,
    LaborParticipation,
    ProductiveCapacityUtilization,
    EcologicalCapacity,
    EcologicalPressure,
    ExternalConstraint,
    ImportConstraint,
    ForeignCurrencyConstraint,
}

/// One measurement-only economic observation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicObservation {
    /// Stable observation identifier.
    pub observation_id: String,
    /// Entity, geography, sector, or resource to which the observation refers.
    pub subject_ref: String,
    /// Measurement signal type.
    pub signal: EconomicObservationSignal,
    /// Native unit supplied by the source.
    pub native_unit: String,
    /// Canonical decimal lexical value; no exponent notation.
    pub value: String,
    /// Inclusive reference-period start.
    pub reference_period_start: u64,
    /// Inclusive reference-period end.
    pub reference_period_end: u64,
    /// Source/methodology identity.
    pub methodology_ref: String,
    /// Publishing/source identity.
    pub source_ref: String,
    /// Evidence supporting this observation.
    pub evidence_refs: Vec<String>,
    /// Confidence in the observation, in basis points.
    pub confidence_bps: u16,
    /// Quality metadata, in basis points.
    pub quality_bps: u16,
    /// Time at which the observation was recorded/published.
    pub observed_at: u64,
    /// Optional explicit predecessor observation.
    pub supersedes_observation_ref: Option<String>,
    /// Optional shared identity for conflicting observations.
    pub conflict_set_ref: Option<String>,
}

impl EconomicObservation {
    /// Validate measurement structure without making a policy judgment.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("observation ID", self.observation_id.as_str()),
            ("subject reference", self.subject_ref.as_str()),
            ("native unit", self.native_unit.as_str()),
            ("value", self.value.as_str()),
            ("methodology reference", self.methodology_ref.as_str()),
            ("source reference", self.source_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Economic observation {name} cannot be empty"));
            }
        }

        if !is_canonical_decimal(&self.value) {
            return Err(
                "Economic observation value must be a canonical decimal without exponent notation"
                    .into(),
            );
        }

        if self.reference_period_end < self.reference_period_start {
            return Err(
                "Economic observation reference period cannot end before it starts".into(),
            );
        }

        if self.observed_at < self.reference_period_end {
            return Err(
                "Economic observation observed-at timestamp cannot precede reference-period end"
                    .into(),
            );
        }

        if self.confidence_bps > 10_000 {
            return Err("Economic observation confidence exceeds 10,000 bps".into());
        }
        if self.quality_bps > 10_000 {
            return Err("Economic observation quality exceeds 10,000 bps".into());
        }

        if self.evidence_refs.is_empty() {
            return Err("Economic observation requires evidence references".into());
        }

        validate_unique_nonempty_refs("evidence", &self.evidence_refs)?;

        if let Some(reference) = &self.supersedes_observation_ref {
            if reference.trim().is_empty() {
                return Err("Superseded observation reference cannot be empty".into());
            }
            if reference == &self.observation_id {
                return Err("Economic observation cannot supersede itself".into());
            }
        }

        if let Some(reference) = &self.conflict_set_ref {
            if reference.trim().is_empty() {
                return Err("Conflict-set reference cannot be empty".into());
            }
        }

        Ok(())
    }

    /// Return a deterministic content identity.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut evidence_refs = self.evidence_refs.clone();
        evidence_refs.sort();

        let payload = serde_json::json!({
            "version": 1,
            "observation_id": self.observation_id,
            "subject_ref": self.subject_ref,
            "signal": self.signal,
            "native_unit": self.native_unit,
            "value": self.value,
            "reference_period_start": self.reference_period_start,
            "reference_period_end": self.reference_period_end,
            "methodology_ref": self.methodology_ref,
            "source_ref": self.source_ref,
            "evidence_refs": evidence_refs,
            "confidence_bps": self.confidence_bps,
            "quality_bps": self.quality_bps,
            "observed_at": self.observed_at,
            "supersedes_observation_ref": self.supersedes_observation_ref,
            "conflict_set_ref": self.conflict_set_ref,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic observation canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-OBSERVATION-V1\\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

/// Exact content identity for one observation in a snapshot.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicObservationBinding {
    pub observation_ref: String,
    pub observation_fingerprint: String,
}

impl EconomicObservationBinding {
    pub fn validate(&self) -> Result<(), String> {
        if self.observation_ref.trim().is_empty() {
            return Err("Economic observation reference cannot be empty".into());
        }
        if !is_sha256_hex(&self.observation_fingerprint) {
            return Err(
                "Economic observation fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        Ok(())
    }
}

/// Immutable identity of the exact observations consumed by a model.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicObservationSnapshot {
    pub snapshot_id: String,
    pub observation_bindings: Vec<EconomicObservationBinding>,
    pub captured_at: u64,
}

impl EconomicObservationSnapshot {
    pub fn validate(&self) -> Result<(), String> {
        if self.snapshot_id.trim().is_empty() {
            return Err("Economic observation snapshot ID cannot be empty".into());
        }
        if self.observation_bindings.is_empty() {
            return Err("Economic observation snapshot requires observations".into());
        }

        let mut refs = BTreeSet::new();
        for binding in &self.observation_bindings {
            binding.validate()?;
            if !refs.insert(&binding.observation_ref) {
                return Err(format!(
                    "Duplicate economic observation in snapshot: {}",
                    binding.observation_ref
                ));
            }
        }
        Ok(())
    }

    pub fn validate_against(
        &self,
        observations: &std::collections::BTreeMap<String, EconomicObservation>,
    ) -> Result<(), String> {
        self.validate()?;

        for binding in &self.observation_bindings {
            let observation = observations.get(&binding.observation_ref).ok_or_else(|| {
                format!(
                    "Economic observation is not available for snapshot: {}",
                    binding.observation_ref
                )
            })?;
            let fingerprint = observation.fingerprint()?;
            if fingerprint != binding.observation_fingerprint {
                return Err(format!(
                    "Economic observation fingerprint does not match snapshot binding: {}",
                    binding.observation_ref
                ));
            }
        }

        Ok(())
    }

    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut bindings = self.observation_bindings.clone();
        bindings.sort_by_key(|binding| {
            (
                binding.observation_ref.clone(),
                binding.observation_fingerprint.clone(),
            )
        });

        let payload = serde_json::json!({
            "version": 1,
            "snapshot_id": self.snapshot_id,
            "observation_bindings": bindings,
            "captured_at": self.captured_at,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic observation snapshot canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-OBSERVATION-SNAPSHOT-V1\\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.as_bytes().iter().all(u8::is_ascii_hexdigit)
}

fn is_canonical_decimal(value: &str) -> bool {
    if value.starts_with('+') || value.contains('e') || value.contains('E') {
        return false;
    }

    let mut digits_before = 0usize;
    let mut digits_after = 0usize;
    let mut seen_dot = false;

    for (index, byte) in value.bytes().enumerate() {
        if index == 0 && byte == b'-' {
            continue;
        }
        match byte {
            b'0'..=b'9' if !seen_dot => digits_before += 1,
            b'0'..=b'9' if seen_dot => digits_after += 1,
            b'.' if !seen_dot => seen_dot = true,
            _ => return false,
        }
    }

    digits_before > 0 && (!seen_dot || digits_after > 0)
}

fn validate_unique_nonempty_refs(name: &str, refs: &[String]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        if reference.trim().is_empty() {
            return Err(format!("Economic observation {name} references cannot be empty"));
        }
        if !seen.insert(reference) {
            return Err(format!(
                "Duplicate economic observation {name} reference: {reference}"
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation() -> EconomicObservation {
        EconomicObservation {
            observation_id: "observation:price:1".into(),
            subject_ref: "economy:za".into(),
            signal: EconomicObservationSignal::PriceGrowth,
            native_unit: "percent".into(),
            value: "4.25".into(),
            reference_period_start: 1_000,
            reference_period_end: 2_000,
            methodology_ref: "method:cpi:v1".into(),
            source_ref: "source:statistics".into(),
            evidence_refs: vec!["evidence:cpi:1".into()],
            confidence_bps: 9_200,
            quality_bps: 9_000,
            observed_at: 2_100,
            supersedes_observation_ref: None,
            conflict_set_ref: None,
        }
    }

    #[test]
    fn validates_measurement_only_observation() {
        assert!(observation().validate().is_ok());
        assert_eq!(observation().fingerprint().unwrap().len(), 64);
    }

    #[test]
    fn decimal_validation_is_explicit() {
        for valid in ["0", "1", "-1", "4.25", "0.0"] {
            assert!(is_canonical_decimal(valid));
        }
        for invalid in ["", "+1", "1.", ".5", "1e2", "1E2", "nan"] {
            assert!(!is_canonical_decimal(invalid));
        }
    }

    #[test]
    fn reference_period_precedes_or_matches_observation_time() {
        let mut value = observation();
        value.observed_at = 1_999;
        assert!(value.validate().is_err());
    }

    #[test]
    fn conflicts_are_preserved_by_reference() {
        let mut left = observation();
        let mut right = observation();
        left.observation_id = "observation:source-a".into();
        right.observation_id = "observation:source-b".into();
        left.source_ref = "source:a".into();
        right.source_ref = "source:b".into();
        left.conflict_set_ref = Some("conflict:price:2026-01".into());
        right.conflict_set_ref = Some("conflict:price:2026-01".into());

        assert!(left.validate().is_ok());
        assert!(right.validate().is_ok());
        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn observation_reference_order_does_not_change_identity() {
        let left = observation();
        let mut right = observation();
        right.evidence_refs.reverse();
        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn snapshot_binds_exact_observation_content() {
        let value = observation();
        let fingerprint = value.fingerprint().unwrap();
        let snapshot = EconomicObservationSnapshot {
            snapshot_id: "snapshot:1".into(),
            observation_bindings: vec![EconomicObservationBinding {
                observation_ref: value.observation_id.clone(),
                observation_fingerprint: fingerprint,
            }],
            captured_at: 2_200,
        };

        let mut observations = std::collections::BTreeMap::new();
        observations.insert(value.observation_id.clone(), value.clone());
        assert!(snapshot.validate_against(&observations).is_ok());

        let mut changed = value;
        changed.value = "6.25".into();
        observations.insert(changed.observation_id.clone(), changed);
        assert!(snapshot.validate_against(&observations).is_err());
    }

    #[test]
    fn snapshot_order_does_not_change_identity() {
        let value_a = observation();
        let mut value_b = observation();
        value_b.observation_id = "observation:capacity:1".into();
        value_b.signal = EconomicObservationSignal::ProductiveCapacityUtilization;
        value_b.value = "0.84".into();

        let binding_a = EconomicObservationBinding {
            observation_ref: value_a.observation_id.clone(),
            observation_fingerprint: value_a.fingerprint().unwrap(),
        };
        let binding_b = EconomicObservationBinding {
            observation_ref: value_b.observation_id.clone(),
            observation_fingerprint: value_b.fingerprint().unwrap(),
        };

        let left = EconomicObservationSnapshot {
            snapshot_id: "snapshot:1".into(),
            observation_bindings: vec![binding_a.clone(), binding_b.clone()],
            captured_at: 2_200,
        };
        let right = EconomicObservationSnapshot {
            snapshot_id: "snapshot:1".into(),
            observation_bindings: vec![binding_b, binding_a],
            captured_at: 2_200,
        };

        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }
}
