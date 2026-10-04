// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Jurisdiction Conformance Packs
//!
//! A conformance pack binds an Economic OS policy profile to the concrete
//! operations, authorities, standards, and declared interoperability limits
//! supported by one jurisdiction or economic regime.
//!
//! Packs are intentionally additive: a country, monetary union, municipality,
//! cooperative, or mutual-credit network can publish a pack without forking
//! the Economic OS kernel.

use super::policy_profile::{EconomicOsOperation, EconomicPolicyProfile};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

/// Degree to which a jurisdiction adapter preserves semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InteroperabilityGuarantee {
    /// All modeled semantics required by the mapping are preserved.
    Lossless,
    /// Some semantics cannot be represented; the loss is explicitly declared.
    LossyDeclared,
    /// Only a human-readable representation is supported.
    HumanReadableOnly,
    /// The adapter does not support the representation.
    Unsupported,
}

/// Declares one concrete capability of a jurisdiction implementation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicOsCapability {
    /// Operation implemented by the pack.
    pub operation: EconomicOsOperation,
    /// Implementation/adapter reference.
    pub implementation_ref: String,
    /// Authority reference required for this operation, when applicable.
    pub authority_ref: Option<String>,
    /// External standards or rails this capability maps to.
    pub interoperability_refs: Vec<String>,
    /// Semantic preservation guarantee for this capability.
    pub guarantee: InteroperabilityGuarantee,
    /// Deterministic test-vector references used to establish conformance.
    pub conformance_test_refs: Vec<String>,
}

impl EconomicOsCapability {
    /// Validate one capability declaration.
    pub fn validate(&self) -> Result<(), String> {
        if self.implementation_ref.trim().is_empty() {
            return Err("Economic OS implementation reference cannot be empty".into());
        }

        if let Some(authority) = &self.authority_ref {
            if authority.trim().is_empty() {
                return Err("Economic OS capability authority reference cannot be empty".into());
            }
        }

        for (name, refs) in [
            ("interoperability", &self.interoperability_refs),
            ("conformance test", &self.conformance_test_refs),
        ] {
            let mut seen = BTreeSet::new();
            for reference in refs {
                if reference.trim().is_empty() {
                    return Err(format!("Economic OS {name} references cannot be empty"));
                }
                if !seen.insert(reference) {
                    return Err(format!("Duplicate Economic OS {name} reference: {reference}"));
                }
            }
        }

        Ok(())
    }
}

/// Complete, versioned conformance declaration for one policy profile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicJurisdictionPack {
    /// Stable pack identifier.
    pub pack_id: String,
    /// Exact economic policy profile represented by this pack.
    pub profile: EconomicPolicyProfile,
    /// Economic OS kernel compatibility version.
    pub kernel_version: String,
    /// Operations intentionally implemented by the pack.
    pub supported_operations: Vec<EconomicOsOperation>,
    /// Concrete capability/adapters for supported operations.
    pub capabilities: Vec<EconomicOsCapability>,
    /// Evidence supporting this conformance declaration.
    pub conformance_evidence_refs: Vec<String>,
    /// Declared semantic losses keyed by external representation.
    pub loss_declarations: BTreeMap<String, String>,
    /// Optional predecessor pack when the jurisdiction profile is superseded.
    pub supersedes_pack_ref: Option<String>,
    /// Pack declaration timestamp.
    pub declared_at: u64,
}

impl EconomicJurisdictionPack {
    /// Validate the pack and its correspondence with the embedded profile.
    pub fn validate(&self) -> Result<(), String> {
        if self.pack_id.trim().is_empty() {
            return Err("Economic OS pack ID cannot be empty".into());
        }
        if self.kernel_version.trim().is_empty() {
            return Err("Economic OS kernel version cannot be empty".into());
        }
        self.profile.validate()?;

        if self.conformance_evidence_refs.is_empty() {
            return Err("Economic OS pack requires conformance evidence".into());
        }

        let mut supported = BTreeSet::new();
        for operation in &self.supported_operations {
            if !supported.insert(*operation) {
                return Err(format!(
                    "Duplicate supported Economic OS operation: {operation:?}"
                ));
            }
        }

        let mut capability_operations = BTreeSet::new();
        for capability in &self.capabilities {
            capability.validate()?;
            if !supported.contains(&capability.operation) {
                return Err(format!(
                    "Capability {:?} is not listed in supported operations",
                    capability.operation
                ));
            }
            if !capability_operations.insert(capability.operation) {
                return Err(format!(
                    "Multiple capabilities declared for Economic OS operation: {:?}",
                    capability.operation
                ));
            }

            if let Some(authority_ref) = &capability.authority_ref {
                if !self.profile.authority_refs.contains(authority_ref) {
                    return Err(format!(
                        "Capability authority {authority_ref} is absent from policy profile"
                    ));
                }
            }
        }

        if capability_operations != supported {
            return Err(
                "Every supported Economic OS operation must have exactly one capability"
                    .into(),
            );
        }

        for (representation, declaration) in &self.loss_declarations {
            if representation.trim().is_empty() || declaration.trim().is_empty() {
                return Err("Economic OS loss declarations cannot contain empty keys or values".into());
            }
        }

        if let Some(previous) = &self.supersedes_pack_ref {
            if previous.trim().is_empty() {
                return Err("Superseded Economic OS pack reference cannot be empty".into());
            }
            if previous == &self.pack_id {
                return Err("Economic OS pack cannot supersede itself".into());
            }
        }

        for evidence in &self.conformance_evidence_refs {
            if evidence.trim().is_empty() {
                return Err("Economic OS conformance evidence references cannot be empty".into());
            }
        }

        Ok(())
    }

    /// Determine whether the pack explicitly supports an operation.
    pub fn supports(&self, operation: EconomicOsOperation) -> bool {
        self.supported_operations.contains(&operation)
    }

    /// Return the exact profile content identity represented by this pack.
    pub fn profile_fingerprint(&self) -> Result<String, String> {
        self.profile.fingerprint()
    }

    /// Return a deterministic content identity for the entire pack.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut supported_operations = self.supported_operations.clone();
        supported_operations.sort();

        let mut conformance_evidence_refs = self.conformance_evidence_refs.clone();
        conformance_evidence_refs.sort();

        let payload = serde_json::json!({
            "version": 1,
            "pack_id": self.pack_id,
            "profile": self.profile,
            "kernel_version": self.kernel_version,
            "supported_operations": supported_operations,
            "capabilities": self.capabilities,
            "conformance_evidence_refs": conformance_evidence_refs,
            "loss_declarations": self.loss_declarations,
            "supersedes_pack_ref": self.supersedes_pack_ref,
            "declared_at": self.declared_at,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic OS pack canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-OS-JURISDICTION-PACK-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> EconomicPolicyProfile {
        EconomicPolicyProfile {
            profile_id: "profile:za:reference:v1".into(),
            jurisdiction_ref: "jurisdiction:ZA".into(),
            regime_ref: "regime:national-fiat".into(),
            policy_version: "1.0.0".into(),
            currency_refs: vec!["ZAR".into()],
            authority_refs: vec!["authority:treasury".into(), "authority:central-bank".into()],
            policy_rule_refs: vec!["rule:fiscal-v1".into()],
            interoperability_profile_refs: vec![
                "standard:SNA-2025".into(),
                "standard:BPM7".into(),
                "standard:SDMX-3.1".into(),
                "standard:SEEA".into(),
                "standard:ISO-20022".into(),
            ],
            effective_from: 1_000,
            effective_until: None,
            evidence_refs: vec!["evidence:profile".into()],
            supersedes_profile_ref: None,
            declared_at: 1_000,
        }
    }

    fn pack() -> EconomicJurisdictionPack {
        let operations = vec![
            EconomicOsOperation::Observe,
            EconomicOsOperation::Authorize,
            EconomicOsOperation::Commit,
            EconomicOsOperation::Settle,
            EconomicOsOperation::Reconcile,
            EconomicOsOperation::Finalize,
            EconomicOsOperation::Publish,
        ];

        EconomicJurisdictionPack {
            pack_id: "pack:za:v1".into(),
            profile: profile(),
            kernel_version: "economic-os-v1".into(),
            supported_operations: operations.clone(),
            capabilities: operations
                .iter()
                .map(|operation| EconomicOsCapability {
                    operation: *operation,
                    implementation_ref: format!("adapter:{operation:?}"),
                    authority_ref: Some("authority:treasury".into()),
                    interoperability_refs: vec!["standard:SDMX-3.1".into()],
                    guarantee: InteroperabilityGuarantee::Lossless,
                    conformance_test_refs: vec![format!("test:{operation:?}:v1")],
                })
                .collect(),
            conformance_evidence_refs: vec!["evidence:conformance".into()],
            loss_declarations: BTreeMap::new(),
            supersedes_pack_ref: None,
            declared_at: 1_100,
        }
    }

    #[test]
    fn validates_complete_pack() {
        assert!(pack().validate().is_ok());
        assert!(pack().supports(EconomicOsOperation::Settle));
    }

    #[test]
    fn rejects_capability_for_unsupported_operation() {
        let mut value = pack();
        value.capabilities.pop();
        assert!(value.validate().is_err());
    }

    #[test]
    fn rejects_authority_not_declared_by_profile() {
        let mut value = pack();
        value.capabilities[0].authority_ref = Some("authority:unknown".into());
        assert!(value.validate().is_err());
    }

    #[test]
    fn loss_is_explicit() {
        let mut value = pack();
        value.loss_declarations.insert(
            "legacy:report".into(),
            "causation_refs are not representable".into(),
        );
        assert!(value.validate().is_ok());
    }

    #[test]
    fn pack_fingerprint_changes_when_capability_changes() {
        let left = pack().fingerprint().unwrap();
        let mut changed = pack();
        changed.capabilities[0].implementation_ref = "adapter:changed".into();
        let right = changed.fingerprint().unwrap();
        assert_ne!(left, right);
    }
}
