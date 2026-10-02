// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Reproducible evidence binding for economic simulations.
//!
//! The evidence layer is deliberately separate from economic state. It binds
//! model identity, parameter identity, seed, the initial state, the chained
//! timestep history, and terminal observations without turning any of those
//! metadata fields into behavioral rules.

use serde::{Deserialize, Serialize};

use super::observables::EconomicObservables;
use super::period_ledger::EconomicPeriodLedger;
use super::transition::{EconomicChainReceipt, EconomicStepError};

/// Immutable identity for one reproducible simulation configuration.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEvidenceManifest {
    pub model_version: String,
    pub parameter_hash: String,
    pub seed: u64,
    pub initial_state_hash: String,
}

impl EconomicEvidenceManifest {
    pub fn new(
        model_version: impl Into<String>,
        parameter_hash: impl Into<String>,
        seed: u64,
        initial_state_hash: impl Into<String>,
    ) -> Result<Self, EconomicStepError> {
        let manifest = Self {
            model_version: model_version.into(),
            parameter_hash: parameter_hash.into(),
            seed,
            initial_state_hash: initial_state_hash.into(),
        };
        if manifest.model_version.is_empty()
            || manifest.parameter_hash.is_empty()
            || manifest.initial_state_hash.is_empty()
        {
            return Err(EconomicStepError::Serialization(
                "evidence manifest identity fields must be non-empty".into(),
            ));
        }
        Ok(manifest)
    }

    pub fn hash(&self) -> Result<String, EconomicStepError> {
        let bytes = serde_json::to_vec(self)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        Ok(blake3::hash(&bytes).to_hex().to_string())
    }
}

/// Terminal evidence capsule for a completed economic simulation run.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEvidenceCapsule {
    pub manifest: EconomicEvidenceManifest,
    pub manifest_hash: String,
    pub final_chain_hash: String,
    pub observations_hash: String,
    pub evidence_hash: String,
}

impl EconomicEvidenceCapsule {
    /// Bind a manifest, final receipt-chain node, and terminal observations.
    pub fn seal(
        manifest: EconomicEvidenceManifest,
        final_receipt: &EconomicChainReceipt,
        observations: &EconomicObservables,
    ) -> Result<Self, EconomicStepError> {
        let manifest_hash = manifest.hash()?;
        let observations_hash = hash_observations(observations)?;
        let binding = (
            &manifest_hash,
            &final_receipt.chain_hash,
            &observations_hash,
        );
        let bytes = serde_json::to_vec(&binding)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let evidence_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok(Self {
            manifest,
            manifest_hash,
            final_chain_hash: final_receipt.chain_hash.clone(),
            observations_hash,
            evidence_hash,
        })
    }
}

fn hash_observations(
    observations: &EconomicObservables,
) -> Result<String, EconomicStepError> {
    let bytes = serde_json::to_vec(observations)
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{ActorBalanceSheet, CreditCreation, EconomicState};
    use crate::economics::transition::{apply_step, EconomicTransition};

    fn fixture() -> (
        EconomicEvidenceManifest,
        EconomicChainReceipt,
        EconomicObservables,
    ) {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        let state = EconomicState::new(vec![bank, ActorBalanceSheet::new("household")]);
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 100).unwrap(),
        )];
        let (post, step) = apply_step(&state, 1, &transitions, None).unwrap();
        let chain = EconomicChainReceipt::link(None, step).unwrap();
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        let observations = EconomicObservables::from_state_and_ledger(&post, &ledger);
        let manifest = EconomicEvidenceManifest::new(
            "economics-v1",
            "params-abc",
            42,
            crate::economics::transition::state_hash(&state).unwrap(),
        ).unwrap();
        (manifest, chain, observations)
    }

    #[test]
    fn evidence_capsule_is_deterministic() {
        let (manifest, chain, observations) = fixture();
        let a = EconomicEvidenceCapsule::seal(manifest.clone(), &chain, &observations).unwrap();
        let b = EconomicEvidenceCapsule::seal(manifest, &chain, &observations).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.manifest_hash, a.manifest.hash().unwrap());
    }

    #[test]
    fn evidence_capsule_changes_when_observations_change() {
        let (manifest, chain, mut observations) = fixture();
        let a = EconomicEvidenceCapsule::seal(manifest.clone(), &chain, &observations).unwrap();
        observations.credit_created += 1;
        let b = EconomicEvidenceCapsule::seal(manifest, &chain, &observations).unwrap();
        assert_ne!(a.evidence_hash, b.evidence_hash);
        assert_ne!(a.observations_hash, b.observations_hash);
    }
}
