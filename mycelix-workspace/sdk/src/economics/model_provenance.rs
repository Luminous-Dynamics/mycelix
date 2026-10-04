// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Structured provenance for economic reasoning models.
//!
//! Model provenance describes what generated an analysis. It does not certify
//! correctness, independence, safety, authorship, or legal authority.
//! Unknown provenance remains unknown; callers must not infer independence from
//! absent metadata.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicModelProvenance {
    pub model_ref: String,
    pub model_version: String,
    pub model_family_ref: Option<String>,
    pub provider_ref: Option<String>,
    pub implementation_fingerprint: Option<String>,
    pub deployment_ref: Option<String>,
    pub training_data_refs: Vec<String>,
    pub evaluation_data_refs: Vec<String>,
    pub declared_at: u64,
}

impl EconomicModelProvenance {
    pub fn validate(&self) -> Result<(), String> {
        if self.model_ref.trim().is_empty() { return Err("Economic model reference cannot be empty".into()); }
        if self.model_version.trim().is_empty() { return Err("Economic model version cannot be empty".into()); }
        for (name, value) in [("model family", self.model_family_ref.as_deref()), ("provider", self.provider_ref.as_deref()), ("deployment", self.deployment_ref.as_deref())] {
            if let Some(value) = value {
                if value.trim().is_empty() { return Err(format!("Economic model {name} reference cannot be empty")); }
            }
        }
        if let Some(fingerprint) = &self.implementation_fingerprint {
            if !is_sha256_hex(fingerprint) { return Err("Economic model implementation fingerprint must be a 64-character hexadecimal SHA-256".into()); }
        }
        validate_unique_nonempty_refs("training-data", &self.training_data_refs)?;
        validate_unique_nonempty_refs("evaluation-data", &self.evaluation_data_refs)?;
        Ok(())
    }

    pub fn has_comparable_identity(&self) -> bool { self.implementation_fingerprint.is_some() }

    pub fn comparison_key(&self) -> Result<Option<String>, String> {
        self.validate()?;
        let implementation = match &self.implementation_fingerprint { Some(value) => value, None => return Ok(None) };
        let payload = serde_json::json!({
            "version": 1,
            "model_ref": self.model_ref,
            "model_version": self.model_version,
            "model_family_ref": self.model_family_ref,
            "provider_ref": self.provider_ref,
            "implementation_fingerprint": implementation,
        });
        let canonical = serde_json::to_vec(&payload).map_err(|error| format!("Economic model provenance canonicalization failed: {error}"))?;
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-MODEL-PROVENANCE-KEY-V1\0");
        hasher.update(canonical);
        Ok(Some(hex::encode(hasher.finalize())))
    }

    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;
        let mut training_data_refs = self.training_data_refs.clone();
        let mut evaluation_data_refs = self.evaluation_data_refs.clone();
        training_data_refs.sort();
        evaluation_data_refs.sort();
        let payload = serde_json::json!({
            "version": 1,
            "model_ref": self.model_ref,
            "model_version": self.model_version,
            "model_family_ref": self.model_family_ref,
            "provider_ref": self.provider_ref,
            "implementation_fingerprint": self.implementation_fingerprint,
            "deployment_ref": self.deployment_ref,
            "training_data_refs": training_data_refs,
            "evaluation_data_refs": evaluation_data_refs,
            "declared_at": self.declared_at,
        });
        let canonical = serde_json::to_vec(&payload).map_err(|error| format!("Economic model provenance canonicalization failed: {error}"))?;
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-MODEL-PROVENANCE-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

fn is_sha256_hex(value: &str) -> bool { value.len() == 64 && value.as_bytes().iter().all(u8::is_ascii_hexdigit) }

fn validate_unique_nonempty_refs(name: &str, refs: &[String]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        if reference.trim().is_empty() { return Err(format!("Economic model {name} references cannot be empty")); }
        if !seen.insert(reference) { return Err(format!("Duplicate economic model {name} reference: {reference}")); }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn provenance() -> EconomicModelProvenance { EconomicModelProvenance {
        model_ref: "model:symthaea".into(),
        model_version: "2026.10".into(),
        model_family_ref: Some("family:symthaea-econ".into()),
        provider_ref: Some("provider:luminous-dynamics".into()),
        implementation_fingerprint: Some("a".repeat(64)),
        deployment_ref: Some("deployment:local-01".into()),
        training_data_refs: vec!["dataset:macro-2025".into()],
        evaluation_data_refs: vec!["dataset:macro-holdout-2026".into()],
        declared_at: 1_000,
    }}
    #[test] fn validates_structured_provenance() { let value=provenance(); assert!(value.validate().is_ok()); assert!(value.has_comparable_identity()); assert_eq!(value.fingerprint().unwrap().len(),64); assert_eq!(value.comparison_key().unwrap().unwrap().len(),64); }
    #[test] fn deployment_does_not_create_new_comparison_identity() { let left=provenance(); let mut right=provenance(); right.deployment_ref=Some("deployment:other".into()); assert_eq!(left.comparison_key().unwrap(),right.comparison_key().unwrap()); }
    #[test] fn missing_implementation_fingerprint_does_not_fabricate_independence() { let mut value=provenance(); value.implementation_fingerprint=None; assert!(!value.has_comparable_identity()); assert!(value.comparison_key().unwrap().is_none()); }
    #[test] fn implementation_change_changes_content_identity() { let left=provenance(); let mut right=left.clone(); right.implementation_fingerprint=Some("b".repeat(64)); assert_ne!(left.fingerprint().unwrap(),right.fingerprint().unwrap()); }
    #[test] fn rejects_malformed_implementation_fingerprint() { let mut value=provenance(); value.implementation_fingerprint=Some("bad".into()); assert!(value.validate().is_err()); }
    #[test] fn rejects_duplicate_dataset_refs() { let mut value=provenance(); value.training_data_refs.push("dataset:macro-2025".into()); assert!(value.validate().is_err()); }
}
