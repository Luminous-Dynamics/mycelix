// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact material-flow accounting for Mycelix Circularity.
//!
//! This crate proves only a bounded conservation/accounting proposition.
//! It does not prove physical measurements, recovery yield, recycled content,
//! environmental benefit, compliance, or process execution.
//!
//! Identity is deliberately split to avoid a cryptographic cycle:
//!
//! ```text
//! MaterialFlowAccount -> account_id
//! CircularityProfile   -> account_ref -> profile_id
//! ERG Event            -> profile_ref -> event_id
//! MaterialFlowBinding  -> account_ref + event_ref -> binding_id
//! ```

use mycelix_circularity::ChainOfCustodyStrategyV1;
use mycelix_economic_reality_graph::ExactErgSubjectRefV1;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const ACCOUNT_DOMAIN: &str = "mycelix-material-flow-accounting::account-v1";
const BINDING_DOMAIN: &str = "mycelix-material-flow-accounting::event-binding-v1";
const MAX_TOKEN_BYTES: usize = 512;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum MaterialFlowError {
    #[error("{field} must be a canonical non-empty token")]
    NonCanonical { field: &'static str },
    #[error("material allocation quantity must be positive")]
    ZeroQuantity,
    #[error("material-flow account requires at least one attributable input")]
    MissingInput,
    #[error("material-flow account requires at least one output/loss/residual allocation")]
    MissingOutput,
    #[error("allocation requires at least one exact evidence reference")]
    MissingEvidence,
    #[error("duplicate allocation id: {0}")]
    DuplicateAllocationId(String),
    #[error("same exact source subject allocated more than once without split-lineage child identities: {0}")]
    DoubleCountedSource(String),
    #[error("same exact material/output subject allocated to mutually exclusive roles: {0}")]
    DoubleCountedOutput(String),
    #[error("duplicate evidence reference in allocation {0}")]
    DuplicateEvidence(String),
    #[error("invalid external reference: {0}")]
    InvalidExternalRef(String),
    #[error("checked mass arithmetic overflow")]
    ArithmeticOverflow,
    #[error("material-flow account is not closed: sources={sources_mg} mg, uses={uses_mg} mg")]
    ConservationMismatch { sources_mg: u64, uses_mg: u64 },
}

fn canonical(field: &'static str, value: &str) -> Result<(), MaterialFlowError> {
    if value.is_empty()
        || value.len() > MAX_TOKEN_BYTES
        || value.trim() != value
        || value.chars().any(char::is_control)
    {
        return Err(MaterialFlowError::NonCanonical { field });
    }
    Ok(())
}

fn hash_field(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct MassMg(pub u64);

impl MassMg {
    pub fn checked_add(self, rhs: Self) -> Result<Self, MaterialFlowError> {
        self.0
            .checked_add(rhs.0)
            .map(Self)
            .ok_or(MaterialFlowError::ArithmeticOverflow)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum MassBasisV1 {
    AsReceived,
    DryMatterEquivalent,
    OtherExactProfile(ExactErgSubjectRefV1),
}

impl MassBasisV1 {
    fn validate(&self) -> Result<(), MaterialFlowError> {
        if let Self::OtherExactProfile(reference) = self {
            reference
                .validate()
                .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?;
        }
        Ok(())
    }

    fn identity_key(&self) -> Result<String, MaterialFlowError> {
        match self {
            Self::AsReceived => Ok("as-received".into()),
            Self::DryMatterEquivalent => Ok("dry-matter-equivalent".into()),
            Self::OtherExactProfile(reference) => Ok(format!(
                "other:{}",
                reference
                    .ref_id()
                    .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?
            )),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum MaterialAllocationRoleV1 {
    AttributableInput,
    SeparatelyEvidencedAddition,
    RetainedComponentOutput,
    RecoveredMaterialOutput,
    RecycledMaterialOutput,
    DowncycledOutput,
    ProcessLoss,
    UnresolvedResidual,
    DisposedOutput,
}

impl MaterialAllocationRoleV1 {
    fn tag(self) -> &'static str {
        match self {
            Self::AttributableInput => "attributable-input",
            Self::SeparatelyEvidencedAddition => "separately-evidenced-addition",
            Self::RetainedComponentOutput => "retained-component-output",
            Self::RecoveredMaterialOutput => "recovered-material-output",
            Self::RecycledMaterialOutput => "recycled-material-output",
            Self::DowncycledOutput => "downcycled-output",
            Self::ProcessLoss => "process-loss",
            Self::UnresolvedResidual => "unresolved-residual",
            Self::DisposedOutput => "disposed-output",
        }
    }

    fn is_source(self) -> bool {
        matches!(
            self,
            Self::AttributableInput | Self::SeparatelyEvidencedAddition
        )
    }

    fn is_output_or_residual(self) -> bool {
        !self.is_source()
    }

    fn is_exclusive_physical_output(self) -> bool {
        matches!(
            self,
            Self::RetainedComponentOutput
                | Self::RecoveredMaterialOutput
                | Self::RecycledMaterialOutput
                | Self::DowncycledOutput
                | Self::DisposedOutput
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaterialAllocationV1 {
    pub allocation_id: String,
    pub role: MaterialAllocationRoleV1,
    pub material_or_resource_ref: ExactErgSubjectRefV1,
    pub mass: MassMg,
    #[serde(default)]
    pub evidence_refs: Vec<ExactErgSubjectRefV1>,
}

impl MaterialAllocationV1 {
    pub fn validate(&self) -> Result<(), MaterialFlowError> {
        canonical("allocation.allocation_id", &self.allocation_id)?;
        if self.mass.0 == 0 {
            return Err(MaterialFlowError::ZeroQuantity);
        }
        self.material_or_resource_ref
            .validate()
            .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?;
        if self.evidence_refs.is_empty() {
            return Err(MaterialFlowError::MissingEvidence);
        }

        let mut seen = BTreeSet::new();
        for reference in &self.evidence_refs {
            reference
                .validate()
                .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?;
            let id = reference
                .ref_id()
                .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?;
            if !seen.insert(id) {
                return Err(MaterialFlowError::DuplicateEvidence(
                    self.allocation_id.clone(),
                ));
            }
        }
        Ok(())
    }

    fn resource_id(&self) -> Result<String, MaterialFlowError> {
        self.material_or_resource_ref
            .ref_id()
            .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))
    }

    fn identity_key(&self) -> Result<String, MaterialFlowError> {
        self.validate()?;
        let material_id = self.resource_id()?;
        let mut evidence = self
            .evidence_refs
            .iter()
            .map(|reference| {
                reference
                    .ref_id()
                    .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        evidence.sort();
        Ok(format!(
            "{}|{}|{}|{}|{}",
            self.allocation_id,
            self.role.tag(),
            material_id,
            self.mass.0,
            evidence.join(",")
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MaterialFlowAuthorityCeilingV1 {
    ConservationAndAttributionOnly,
}

/// Event-independent material accounting subject.
///
/// It is intentionally event-independent so a Circularity event profile may reference this
/// account before the event ID exists. `MaterialFlowEventBindingV1` connects the finalized event
/// afterward without creating a digest cycle.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaterialFlowAccountV1 {
    pub semantic_version: String,
    pub mass_basis: MassBasisV1,
    pub chain_of_custody: ChainOfCustodyStrategyV1,
    pub allocations: Vec<MaterialAllocationV1>,
    pub authority_ceiling: MaterialFlowAuthorityCeilingV1,
    pub display_label: Option<String>,
}

impl MaterialFlowAccountV1 {
    pub fn validate(&self) -> Result<(), MaterialFlowError> {
        canonical("account.semantic_version", &self.semantic_version)?;
        self.mass_basis.validate()?;
        if let Some(label) = &self.display_label {
            canonical("account.display_label", label)?;
        }

        let mut allocation_ids = BTreeSet::new();
        let mut source_subjects = BTreeSet::new();
        let mut exclusive_outputs = BTreeSet::new();
        let mut sources = MassMg(0);
        let mut uses = MassMg(0);
        let mut has_input = false;
        let mut has_output = false;

        for allocation in &self.allocations {
            allocation.validate()?;
            if !allocation_ids.insert(allocation.allocation_id.clone()) {
                return Err(MaterialFlowError::DuplicateAllocationId(
                    allocation.allocation_id.clone(),
                ));
            }

            let resource_id = allocation.resource_id()?;

            if allocation.role == MaterialAllocationRoleV1::AttributableInput {
                has_input = true;
            }
            if allocation.role.is_output_or_residual() {
                has_output = true;
            }

            if allocation.role.is_source() && !source_subjects.insert(resource_id.clone()) {
                return Err(MaterialFlowError::DoubleCountedSource(resource_id));
            }

            if allocation.role.is_exclusive_physical_output()
                && !exclusive_outputs.insert(resource_id.clone())
            {
                return Err(MaterialFlowError::DoubleCountedOutput(resource_id));
            }

            if allocation.role.is_source() {
                sources = sources.checked_add(allocation.mass)?;
            } else {
                uses = uses.checked_add(allocation.mass)?;
            }
        }

        if !has_input {
            return Err(MaterialFlowError::MissingInput);
        }
        if !has_output {
            return Err(MaterialFlowError::MissingOutput);
        }
        if sources != uses {
            return Err(MaterialFlowError::ConservationMismatch {
                sources_mg: sources.0,
                uses_mg: uses.0,
            });
        }
        Ok(())
    }

    pub fn account_id(&self) -> Result<String, MaterialFlowError> {
        self.validate()?;
        let mut allocations = self
            .allocations
            .iter()
            .map(MaterialAllocationV1::identity_key)
            .collect::<Result<Vec<_>, _>>()?;
        allocations.sort();

        let custody = match self.chain_of_custody {
            ChainOfCustodyStrategyV1::IdentityPreserved => "identity-preserved",
            ChainOfCustodyStrategyV1::Segregated => "segregated",
            ChainOfCustodyStrategyV1::ControlledBlending => "controlled-blending",
            ChainOfCustodyStrategyV1::MassBalance => "mass-balance",
            ChainOfCustodyStrategyV1::BookAndClaim => "book-and-claim",
        };

        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, ACCOUNT_DOMAIN);
        hash_field(&mut hasher, &self.semantic_version);
        hash_field(&mut hasher, &self.mass_basis.identity_key()?);
        hash_field(&mut hasher, custody);
        for allocation in allocations {
            hash_field(&mut hasher, &allocation);
        }
        hash_field(&mut hasher, "authority:conservation-and-attribution-only");
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn exact_ref(&self) -> Result<ExactErgSubjectRefV1, MaterialFlowError> {
        let id = self.account_id()?;
        Ok(ExactErgSubjectRefV1 {
            namespace: "mycelix.material-flow.account".into(),
            subject_id: id.clone(),
            semantic_version: self.semantic_version.clone(),
            content_blake3: id,
        })
    }

    pub fn claims_recycled_content(&self) -> bool {
        false
    }

    pub fn claims_recovery_yield(&self) -> bool {
        false
    }

    pub fn claims_environmental_benefit(&self) -> bool {
        false
    }
}

/// Post-event binding that connects one already-identified account to one already-identified ERG
/// lifecycle event. Keeping this separate breaks the profile/account/event hash cycle.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaterialFlowEventBindingV1 {
    pub account_ref: ExactErgSubjectRefV1,
    pub lifecycle_event_ref: ExactErgSubjectRefV1,
}

impl MaterialFlowEventBindingV1 {
    pub fn validate(&self) -> Result<(), MaterialFlowError> {
        self.account_ref
            .validate()
            .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?;
        self.lifecycle_event_ref
            .validate()
            .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?;
        Ok(())
    }

    pub fn binding_id(&self) -> Result<String, MaterialFlowError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, BINDING_DOMAIN);
        hash_field(
            &mut hasher,
            &self
                .account_ref
                .ref_id()
                .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?,
        );
        hash_field(
            &mut hasher,
            &self
                .lifecycle_event_ref
                .ref_id()
                .map_err(|err| MaterialFlowError::InvalidExternalRef(err.to_string()))?,
        );
        Ok(hasher.finalize().to_hex().to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn exact(namespace: &str, id: &str, fill: char) -> ExactErgSubjectRefV1 {
        ExactErgSubjectRefV1 {
            namespace: namespace.into(),
            subject_id: id.into(),
            semantic_version: "1".into(),
            content_blake3: std::iter::repeat_n(fill, 64).collect(),
        }
    }

    fn alloc(
        id: &str,
        role: MaterialAllocationRoleV1,
        resource: &str,
        mass: u64,
        fill: char,
    ) -> MaterialAllocationV1 {
        MaterialAllocationV1 {
            allocation_id: id.into(),
            role,
            material_or_resource_ref: exact("mycelix.erg.resource", resource, fill),
            mass: MassMg(mass),
            evidence_refs: vec![exact("field.mass-evidence", id, fill)],
        }
    }

    fn account() -> MaterialFlowAccountV1 {
        MaterialFlowAccountV1 {
            semantic_version: "1".into(),
            mass_basis: MassBasisV1::AsReceived,
            chain_of_custody: ChainOfCustodyStrategyV1::MassBalance,
            allocations: vec![
                alloc(
                    "input",
                    MaterialAllocationRoleV1::AttributableInput,
                    "returned-product",
                    1_000,
                    'b',
                ),
                alloc(
                    "recovered",
                    MaterialAllocationRoleV1::RecoveredMaterialOutput,
                    "recovered-metal",
                    700,
                    'c',
                ),
                alloc(
                    "loss",
                    MaterialAllocationRoleV1::ProcessLoss,
                    "process-loss-account",
                    100,
                    'd',
                ),
                alloc(
                    "residual",
                    MaterialAllocationRoleV1::UnresolvedResidual,
                    "unresolved-residual-account",
                    200,
                    'e',
                ),
            ],
            authority_ceiling: MaterialFlowAuthorityCeilingV1::ConservationAndAttributionOnly,
            display_label: Some("Synthetic recovery account".into()),
        }
    }

    #[test]
    fn exact_closed_account_validates_without_claim_upgrades() {
        let value = account();
        assert!(value.validate().is_ok());
        assert!(!value.claims_recycled_content());
        assert!(!value.claims_recovery_yield());
        assert!(!value.claims_environmental_benefit());
    }

    #[test]
    fn same_source_subject_cannot_be_counted_twice_by_renaming_allocation() {
        let mut value = account();
        value.allocations[0].mass = MassMg(500);
        value.allocations.push(MaterialAllocationV1 {
            allocation_id: "renamed-second-input-allocation".into(),
            role: MaterialAllocationRoleV1::SeparatelyEvidencedAddition,
            material_or_resource_ref: value.allocations[0].material_or_resource_ref.clone(),
            mass: MassMg(500),
            evidence_refs: vec![exact("field.mass-evidence", "second-reading", '9')],
        });
        assert!(matches!(
            value.validate(),
            Err(MaterialFlowError::DoubleCountedSource(_))
        ));
    }

    #[test]
    fn split_child_subjects_may_be_accounted_separately() {
        let mut value = account();
        value.allocations[0].mass = MassMg(500);
        value.allocations.push(alloc(
            "split-child-input",
            MaterialAllocationRoleV1::SeparatelyEvidencedAddition,
            "returned-product-split-child",
            500,
            '9',
        ));
        assert!(value.validate().is_ok());
    }

    #[test]
    fn same_output_subject_cannot_be_harvested_and_recovered_twice() {
        let mut value = account();
        value.allocations[3] = alloc(
            "harvested",
            MaterialAllocationRoleV1::RetainedComponentOutput,
            "recovered-metal",
            200,
            'f',
        );
        assert!(matches!(
            value.validate(),
            Err(MaterialFlowError::DoubleCountedOutput(_))
        ));
    }

    #[test]
    fn unexplained_mass_gap_rejects_until_residual_is_explicit() {
        let mut value = account();
        value.allocations.pop();
        assert_eq!(
            value.validate(),
            Err(MaterialFlowError::ConservationMismatch {
                sources_mg: 1_000,
                uses_mg: 800,
            })
        );
    }

    #[test]
    fn allocation_order_and_display_label_do_not_change_identity() {
        let a = account();
        let mut b = a.clone();
        b.allocations.reverse();
        b.display_label = Some("Renamed UI label".into());
        assert_eq!(a.account_id().unwrap(), b.account_id().unwrap());
    }

    #[test]
    fn arithmetic_overflow_fails_closed() {
        let value = MaterialFlowAccountV1 {
            semantic_version: "1".into(),
            mass_basis: MassBasisV1::AsReceived,
            chain_of_custody: ChainOfCustodyStrategyV1::Segregated,
            allocations: vec![
                alloc(
                    "input-a",
                    MaterialAllocationRoleV1::AttributableInput,
                    "input-a",
                    u64::MAX,
                    '4',
                ),
                alloc(
                    "input-b",
                    MaterialAllocationRoleV1::SeparatelyEvidencedAddition,
                    "input-b",
                    1,
                    '5',
                ),
                alloc(
                    "residual",
                    MaterialAllocationRoleV1::UnresolvedResidual,
                    "residual",
                    1,
                    '6',
                ),
            ],
            authority_ceiling: MaterialFlowAuthorityCeilingV1::ConservationAndAttributionOnly,
            display_label: None,
        };
        assert_eq!(value.validate(), Err(MaterialFlowError::ArithmeticOverflow));
    }

    #[test]
    fn account_identity_is_event_independent_to_avoid_digest_cycle() {
        let value = account();
        let account_ref = value.exact_ref().unwrap();
        let binding_a = MaterialFlowEventBindingV1 {
            account_ref: account_ref.clone(),
            lifecycle_event_ref: exact("mycelix.erg.event", "recovery-a", '7'),
        };
        let binding_b = MaterialFlowEventBindingV1 {
            account_ref,
            lifecycle_event_ref: exact("mycelix.erg.event", "recovery-b", '8'),
        };
        assert_ne!(binding_a.binding_id().unwrap(), binding_b.binding_id().unwrap());
        assert_eq!(value.account_id().unwrap(), account().account_id().unwrap());
    }

    #[test]
    fn serde_round_trip_preserves_account_and_binding_identity() {
        let value = account();
        let encoded = serde_json::to_string(&value).unwrap();
        let decoded: MaterialFlowAccountV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(value.account_id().unwrap(), decoded.account_id().unwrap());

        let binding = MaterialFlowEventBindingV1 {
            account_ref: value.exact_ref().unwrap(),
            lifecycle_event_ref: exact("mycelix.erg.event", "recovery-event", '9'),
        };
        let encoded_binding = serde_json::to_string(&binding).unwrap();
        let decoded_binding: MaterialFlowEventBindingV1 =
            serde_json::from_str(&encoded_binding).unwrap();
        assert_eq!(binding.binding_id().unwrap(), decoded_binding.binding_id().unwrap());
    }
}
