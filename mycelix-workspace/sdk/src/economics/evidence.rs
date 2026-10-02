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
use std::collections::BTreeMap;

use super::accounting_closure::EconomicAccountingClosure;
use super::actor_observables::ActorEconomicObservables;
use super::observables::EconomicObservables;
use super::sector_observables::SectorEconomicObservables;
use super::sector_flow::EconomicSector;
use super::period_ledger::EconomicPeriodLedger;
use super::transition::{EconomicChainReceipt, EconomicStepError};

/// Immutable identity for one reproducible simulation configuration.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicEvidenceManifest {
    pub model_version: String,
    pub parameter_hash: String,
    pub seed: u64,
    pub initial_state_hash: String,
    /// Optional exact source revision used to produce the evidence.
    ///
    /// When omitted, serialization remains compatible with manifests created
    /// before source-revision provenance was introduced.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_revision: Option<String>,
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
            source_revision: None,
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

    /// Attach an exact source revision (for example, a Git commit SHA)
    /// to the manifest before sealing evidence.
    pub fn with_source_revision(
        mut self,
        source_revision: impl Into<String>,
    ) -> Result<Self, EconomicStepError> {
        let source_revision = source_revision.into();
        if source_revision.is_empty() {
            return Err(EconomicStepError::Serialization(
                "source revision must be non-empty".into(),
            ));
        }
        self.source_revision = Some(source_revision);
        Ok(self)
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
    pub actor_observations_hash: Option<String>,
    #[serde(default)]
    pub sector_observations_hash: Option<String>,
    #[serde(default)]
    pub sector_financial_flow_hash: Option<String>,
    #[serde(default)]
    pub accounting_closure_hash: Option<String>,
    pub evidence_hash: String,
}

impl EconomicEvidenceCapsule {
    /// Bind a manifest, final receipt-chain node, and terminal observations.
    pub fn seal(
        manifest: EconomicEvidenceManifest,
        final_receipt: &EconomicChainReceipt,
        observations: &EconomicObservables,
    ) -> Result<Self, EconomicStepError> {
        Self::seal_with_actor_observations(manifest, final_receipt, observations, None)
    }

    /// Seal aggregate observations together with an optional deterministic
    /// actor-observation map.
    pub fn seal_with_actor_observations(
        manifest: EconomicEvidenceManifest,
        final_receipt: &EconomicChainReceipt,
        observations: &EconomicObservables,
        actor_observations: Option<&BTreeMap<String, ActorEconomicObservables>>,
    ) -> Result<Self, EconomicStepError> {
        let manifest_hash = manifest.hash()?;
        if final_receipt.genesis_state_hash != manifest.initial_state_hash {
            return Err(EconomicStepError::Serialization(
                "final evidence chain does not descend from the manifest initial state".into(),
            ));
        }
        let observations_hash = hash_observations(observations)?;
        let actor_observations_hash = actor_observations
            .map(|values| {
                if values.iter().any(|(actor, observation)| {
                    actor != &observation.actor
                        || observation
                            .cash
                            .checked_add(observation.deposits)
                            != Some(observation.liquidity)
                        || !observation.liquidity_stock_flow_reconciliation_holds()
                        || !observation.net_working_capital_stock_flow_reconciliation_holds()
                        || !observation.working_capital_component_reconciliation_holds()
                        || !observation.liquidity_flow_reconciliation_holds()
                }) {
                    return Err(EconomicStepError::Serialization(
                        "actor observations fail structural liquidity reconciliation".into(),
                    ));
                }
                hash_actor_observations(values)
            })
            .transpose()?;
        let binding = (
            &manifest_hash,
            &final_receipt.chain_hash,
            &observations_hash,
            &actor_observations_hash,
        );
        let bytes = serde_json::to_vec(&binding)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let evidence_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok(Self {
            manifest,
            manifest_hash,
            final_chain_hash: final_receipt.chain_hash.clone(),
            observations_hash,
            actor_observations_hash,
            sector_observations_hash: None,
            sector_financial_flow_hash: None,
            accounting_closure_hash: None,
            evidence_hash,
        })
    }

    /// Seal aggregate and actor observations together with an optional
    /// deterministic sector-observation map. Existing actor-only sealing keeps
    /// its original evidence-hash binding; sector-aware sealing binds the
    /// additional projection into the evidence hash.
    pub fn seal_with_actor_and_sector_observations(
        manifest: EconomicEvidenceManifest,
        final_receipt: &EconomicChainReceipt,
        observations: &EconomicObservables,
        actor_observations: Option<&BTreeMap<String, ActorEconomicObservables>>,
        sector_observations: Option<&BTreeMap<EconomicSector, SectorEconomicObservables>>,
    ) -> Result<Self, EconomicStepError> {
        let manifest_hash = manifest.hash()?;
        if final_receipt.genesis_state_hash != manifest.initial_state_hash {
            return Err(EconomicStepError::Serialization(
                "final evidence chain does not descend from the manifest initial state".into(),
            ));
        }

        let observations_hash = hash_observations(observations)?;
        let actor_observations_hash = actor_observations
            .map(|values| {
                if values.iter().any(|(actor, observation)| {
                    actor != &observation.actor
                        || observation
                            .cash
                            .checked_add(observation.deposits)
                            != Some(observation.liquidity)
                        || !observation.liquidity_stock_flow_reconciliation_holds()
                        || !observation.net_working_capital_stock_flow_reconciliation_holds()
                        || !observation.working_capital_component_reconciliation_holds()
                        || !observation.liquidity_flow_reconciliation_holds()
                }) {
                    return Err(EconomicStepError::Serialization(
                        "actor observations fail structural liquidity reconciliation".into(),
                    ));
                }
                hash_actor_observations(values)
            })
            .transpose()?;
        let sector_observations_hash = sector_observations
            .map(|values| {
                if values.iter().any(|(sector, observation)| {
                    *sector != observation.sector
                        || !observation.liquidity_flow_reconciliation_holds()
                        || !observation.liquidity_stock_flow_reconciliation_holds()
                        || !observation.net_working_capital_stock_flow_reconciliation_holds()
                        || !observation.working_capital_component_reconciliation_holds()
                }) {
                    return Err(EconomicStepError::Serialization(
                        "sector observations fail structural liquidity reconciliation".into(),
                    ));
                }
                hash_sector_observations(values)
            })
            .transpose()?;

        let binding = (
            &manifest_hash,
            &final_receipt.chain_hash,
            &observations_hash,
            &actor_observations_hash,
            &sector_observations_hash,
        );
        let bytes = serde_json::to_vec(&binding)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let evidence_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok(Self {
            manifest,
            manifest_hash,
            final_chain_hash: final_receipt.chain_hash.clone(),
            observations_hash,
            actor_observations_hash,
            sector_observations_hash,
            sector_financial_flow_hash: None,
            accounting_closure_hash: None,
            evidence_hash,
        })
    }

    /// Full evidence seal including the sector financial-claim projection.
    ///
    /// This is additive to the existing sealing methods: actor-only and
    /// actor+sector sealing retain their historical evidence-hash semantics.
    pub fn seal_with_actor_sector_financial_observations(
        manifest: EconomicEvidenceManifest,
        final_receipt: &EconomicChainReceipt,
        observations: &EconomicObservables,
        actor_observations: Option<&BTreeMap<String, ActorEconomicObservables>>,
        sector_observations: Option<&BTreeMap<EconomicSector, SectorEconomicObservables>>,
        sector_financial_flows: Option<&crate::economics::sector_financial_flow::SectorFinancialFlowMatrix>,
    ) -> Result<Self, EconomicStepError> {
        let manifest_hash = manifest.hash()?;
        if final_receipt.genesis_state_hash != manifest.initial_state_hash {
            return Err(EconomicStepError::Serialization(
                "final evidence chain does not descend from the manifest initial state".into(),
            ));
        }

        let observations_hash = hash_observations(observations)?;
        let actor_observations_hash = actor_observations
            .map(|values| {
                if values.iter().any(|(actor, observation)| {
                    actor != &observation.actor
                        || observation
                            .cash
                            .checked_add(observation.deposits)
                            != Some(observation.liquidity)
                        || !observation.liquidity_stock_flow_reconciliation_holds()
                        || !observation.net_working_capital_stock_flow_reconciliation_holds()
                        || !observation.working_capital_component_reconciliation_holds()
                        || !observation.liquidity_flow_reconciliation_holds()
                }) {
                    return Err(EconomicStepError::Serialization(
                        "actor observations fail structural liquidity reconciliation".into(),
                    ));
                }
                hash_actor_observations(values)
            })
            .transpose()?;
        let sector_observations_hash = sector_observations
            .map(|values| {
                if values.iter().any(|(sector, observation)| {
                    *sector != observation.sector
                        || !observation.liquidity_flow_reconciliation_holds()
                        || !observation.liquidity_stock_flow_reconciliation_holds()
                        || !observation.net_working_capital_stock_flow_reconciliation_holds()
                        || !observation.working_capital_component_reconciliation_holds()
                }) {
                    return Err(EconomicStepError::Serialization(
                        "sector observations fail structural liquidity reconciliation".into(),
                    ));
                }
                hash_sector_observations(values)
            })
            .transpose()?;
        let sector_financial_flow_hash = sector_financial_flows
            .map(|matrix| {
                if !matrix.clears() {
                    return Err(EconomicStepError::Serialization(
                        "sector financial-flow matrix does not clear".into(),
                    ));
                }
                hash_sector_financial_flows(matrix)
            })
            .transpose()?;

        let binding = (
            &manifest_hash,
            &final_receipt.chain_hash,
            &observations_hash,
            &actor_observations_hash,
            &sector_observations_hash,
            &sector_financial_flow_hash,
        );
        let bytes = serde_json::to_vec(&binding)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let evidence_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok(Self {
            manifest,
            manifest_hash,
            final_chain_hash: final_receipt.chain_hash.clone(),
            observations_hash,
            actor_observations_hash,
            sector_observations_hash,
            sector_financial_flow_hash,
            accounting_closure_hash: None,
            evidence_hash,
        })
    }

    /// Bind a self-verifying cross-layer accounting closure into the evidence capsule.
    pub fn seal_with_accounting_closure(
        manifest: EconomicEvidenceManifest,
        final_receipt: &EconomicChainReceipt,
        observations: &EconomicObservables,
        closure: &EconomicAccountingClosure,
    ) -> Result<Self, EconomicStepError> {
        closure
            .verify()
            .map_err(EconomicStepError::Serialization)?;

        if final_receipt.genesis_state_hash != manifest.initial_state_hash {
            return Err(EconomicStepError::Serialization(
                "final evidence chain does not descend from the manifest initial state".into(),
            ));
        }
        if closure.pre_state_hash != final_receipt.step.pre_state_hash
            || closure.post_state_hash != final_receipt.step.post_state_hash
            || closure.transition_hash != final_receipt.step.transition_hash
            || closure.transition_count != final_receipt.step.transition_count
        {
            return Err(EconomicStepError::Serialization(
                "accounting closure does not match the final evidence step".into(),
            ));
        }

        let manifest_hash = manifest.hash()?;
        let observations_hash = hash_observations(observations)?;
        if observations_hash != closure.aggregate_observations_hash {
            return Err(EconomicStepError::Serialization(
                "aggregate observations do not match accounting closure".into(),
            ));
        }
        let binding = (
            &manifest_hash,
            &final_receipt.chain_hash,
            &observations_hash,
            &closure.closure_hash,
        );
        let bytes = serde_json::to_vec(&binding)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let evidence_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok(Self {
            manifest,
            manifest_hash,
            final_chain_hash: final_receipt.chain_hash.clone(),
            observations_hash,
            actor_observations_hash: Some(closure.actor_observations_hash.clone()),
            sector_observations_hash: Some(closure.sector_observations_hash.clone()),
            sector_financial_flow_hash: Some(closure.sector_financial_flow_hash.clone()),
            accounting_closure_hash: Some(closure.closure_hash.clone()),
            evidence_hash,
        })
    }
}

fn hash_actor_observations(
    observations: &BTreeMap<String, ActorEconomicObservables>,
) -> Result<String, EconomicStepError> {
    let bytes = serde_json::to_vec(observations)
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

fn hash_sector_observations(
    observations: &BTreeMap<EconomicSector, SectorEconomicObservables>,
) -> Result<String, EconomicStepError> {
    let bytes = serde_json::to_vec(observations)
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

fn hash_sector_financial_flows(
    matrix: &crate::economics::sector_financial_flow::SectorFinancialFlowMatrix,
) -> Result<String, EconomicStepError> {
    let bytes = serde_json::to_vec(matrix)
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
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

    fn fixture_state() -> EconomicState {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        EconomicState::new(vec![bank, ActorBalanceSheet::new("household")])
    }

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
    fn evidence_capsule_accepts_legacy_without_sector_hash() {
        let (manifest, chain, observations) = fixture();
        let capsule = EconomicEvidenceCapsule::seal(manifest, &chain, &observations).unwrap();
        let mut value = serde_json::to_value(&capsule).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .remove("sector_observations_hash");
        value
            .as_object_mut()
            .unwrap()
            .remove("sector_financial_flow_hash");
        value
            .as_object_mut()
            .unwrap()
            .remove("accounting_closure_hash");
        let decoded: EconomicEvidenceCapsule = serde_json::from_value(value).unwrap();
        assert_eq!(decoded.sector_observations_hash, None);
    }

    #[test]
    fn source_revision_is_bound_without_changing_legacy_manifest_hashes() {
        let base = EconomicEvidenceManifest::new(
            "economics-v1",
            "params-abc",
            42,
            "genesis",
        )
        .unwrap();
        let revised = base.clone().with_source_revision(
            "0123456789abcdef0123456789abcdef01234567",
        ).unwrap();

        assert_eq!(
            serde_json::to_value(&base).unwrap()
                .as_object().unwrap()
                .get("source_revision"),
            None
        );
        assert_ne!(base.hash().unwrap(), revised.hash().unwrap());
        assert!(base.clone().with_source_revision("").is_err());
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
    fn evidence_capsule_rejects_wrong_genesis() {
        let (mut manifest, chain, observations) = fixture();
        manifest.initial_state_hash = "wrong-genesis".into();
        assert!(EconomicEvidenceCapsule::seal(manifest, &chain, &observations).is_err());
    }

    #[test]
    fn evidence_capsule_binds_actor_observations_when_supplied() {
        let (manifest, chain, observations) = fixture();
        let actors_a = ActorEconomicObservables::from_state_and_transitions(
            &fixture_state(),
            &[],
        ).unwrap();
        let mut actors_b = actors_a.clone();
        actors_b.get_mut("household").unwrap().deposits += 1;

        let a = EconomicEvidenceCapsule::seal_with_actor_observations(
            manifest.clone(),
            &chain,
            &observations,
            Some(&actors_a),
        ).unwrap();
        let b = EconomicEvidenceCapsule::seal_with_actor_observations(
            manifest,
            &chain,
            &observations,
            Some(&actors_b),
        ).unwrap();

        assert_ne!(a.actor_observations_hash, b.actor_observations_hash);
        assert_ne!(a.evidence_hash, b.evidence_hash);
    }

    #[test]
    fn evidence_capsule_rejects_tampered_actor_liquidity() {
        let (manifest, chain, observations) = fixture();
        let mut actors =
            ActorEconomicObservables::from_state_and_transitions(&fixture_state(), &[])
                .unwrap();
        actors.get_mut("household").unwrap().liquidity = 1;

        assert!(EconomicEvidenceCapsule::seal_with_actor_observations(
            manifest,
            &chain,
            &observations,
            Some(&actors),
        )
        .is_err());
    }

    #[test]
    fn evidence_capsule_rejects_tampered_actor_working_capital() {
        let (manifest, chain, observations) = fixture();
        let mut actors =
            ActorEconomicObservables::from_state_and_transitions(&fixture_state(), &[])
                .unwrap();
        actors.get_mut("household").unwrap().net_working_capital_change = 1;

        assert!(EconomicEvidenceCapsule::seal_with_actor_observations(
            manifest,
            &chain,
            &observations,
            Some(&actors),
        )
        .is_err());
    }

    #[test]
    fn evidence_capsule_rejects_tampered_actor_working_capital_components() {
        let (manifest, chain, observations) = fixture();
        let mut actors =
            ActorEconomicObservables::from_state_and_transitions(&fixture_state(), &[])
                .unwrap();
        actors
            .get_mut("household")
            .unwrap()
            .inventory_carrying_value_change = 1;

        assert!(EconomicEvidenceCapsule::seal_with_actor_observations(
            manifest,
            &chain,
            &observations,
            Some(&actors),
        )
        .is_err());
    }

    #[test]
    fn evidence_capsule_binds_sector_observations_when_supplied() {
        let (manifest, chain, observations) = fixture();
        let actors = ActorEconomicObservables::from_state_and_transitions(
            &fixture_state(),
            &[],
        ).unwrap();
        let assignments = vec![
            crate::economics::sector_balance::SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            crate::economics::sector_balance::SectorAssignment {
                actor: "household".into(),
                sector: EconomicSector::Household,
            },
        ];
        let sectors = SectorEconomicObservables::from_actor_observations(&actors, &assignments)
            .unwrap();
        let mut changed = sectors.clone();
        changed.get_mut(&EconomicSector::Household).unwrap().depreciation += 1;

        let a = EconomicEvidenceCapsule::seal_with_actor_and_sector_observations(
            manifest.clone(),
            &chain,
            &observations,
            Some(&actors),
            Some(&sectors),
        ).unwrap();
        let b = EconomicEvidenceCapsule::seal_with_actor_and_sector_observations(
            manifest,
            &chain,
            &observations,
            Some(&actors),
            Some(&changed),
        ).unwrap();

        assert_ne!(a.sector_observations_hash, b.sector_observations_hash);
        assert_ne!(a.evidence_hash, b.evidence_hash);
    }

    #[test]
    fn evidence_capsule_binds_sector_financial_flows() {
        let (manifest, chain, observations) = fixture();
        let mut flows = crate::economics::sector_financial_flow::SectorFinancialFlowMatrix::default();
        flows.push(
            crate::economics::sector_financial_flow::SectorFinancialFlow::new(
                EconomicSector::Bank,
                EconomicSector::Household,
                crate::economics::sector_financial_flow::FinancialFlowCategory::LoanCreation,
                100,
            )
            .unwrap(),
        );

        let a = EconomicEvidenceCapsule::seal_with_actor_sector_financial_observations(
            manifest.clone(),
            &chain,
            &observations,
            None,
            None,
            Some(&flows),
        )
        .unwrap();

        let mut changed = flows.clone();
        changed.flows[0].amount = 99;

        let b = EconomicEvidenceCapsule::seal_with_actor_sector_financial_observations(
            manifest,
            &chain,
            &observations,
            None,
            None,
            Some(&changed),
        )
        .unwrap();

        assert_ne!(a.sector_financial_flow_hash, b.sector_financial_flow_hash);
        assert_ne!(a.evidence_hash, b.evidence_hash);
    }

    #[test]
    fn evidence_capsule_binds_accounting_closure() {
        let mut bank = crate::economics::stock_flow::ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        let mut firm = crate::economics::stock_flow::ActorBalanceSheet::new("firm");
        firm.real.inventories = 10;
        firm.inventory_carrying_value = 30;
        let state = crate::economics::stock_flow::EconomicState::new(vec![
            bank,
            firm,
            crate::economics::stock_flow::ActorBalanceSheet::new("household"),
        ]);
        let assignments = vec![
            crate::economics::sector_balance::SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            crate::economics::sector_balance::SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
            crate::economics::sector_balance::SectorAssignment {
                actor: "household".into(),
                sector: EconomicSector::Household,
            },
        ];
        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "household", 100).unwrap(),
            ),
            EconomicTransition::TradeCreditSale(
                crate::economics::stock_flow::TradeCreditSale::new(
                    "firm", "household", 1, 30,
                )
                .unwrap(),
            ),
            EconomicTransition::TradeCreditSettlement(
                crate::economics::stock_flow::TradeCreditSettlement::new(
                    "firm", "household", 30,
                )
                .unwrap(),
            ),
        ];
        let (post, step) = apply_step(&state, 1, &transitions, None).unwrap();
        let chain = EconomicChainReceipt::link(None, step).unwrap();
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        let observations = EconomicObservables::from_state_and_ledger(&post, &ledger);
        let closure = EconomicAccountingClosure::validate_and_seal(
            &state, &post, &assignments, &transitions,
        )
        .unwrap();

        let capsule = EconomicEvidenceCapsule::seal_with_accounting_closure(
            EconomicEvidenceManifest::new(
                "economics-v1",
                "params-closure",
                7,
                crate::economics::transition::state_hash(&state).unwrap(),
            )
            .unwrap(),
            &chain,
            &observations,
            &closure,
        )
        .unwrap();

        assert_eq!(
            capsule.accounting_closure_hash.as_deref(),
            Some(closure.closure_hash.as_str())
        );
        assert_eq!(
            capsule.actor_observations_hash.as_deref(),
            Some(closure.actor_observations_hash.as_str())
        );
    }

    #[test]
    fn evidence_capsule_rejects_aggregate_observation_not_bound_to_closure() {
        let mut bank = crate::economics::stock_flow::ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        let state = crate::economics::stock_flow::EconomicState::new(vec![
            bank,
            crate::economics::stock_flow::ActorBalanceSheet::new("household"),
        ]);
        let assignments = vec![
            crate::economics::sector_balance::SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            crate::economics::sector_balance::SectorAssignment {
                actor: "household".into(),
                sector: EconomicSector::Household,
            },
        ];
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 100).unwrap(),
        )];
        let (post, step) = apply_step(&state, 1, &transitions, None).unwrap();
        let chain = EconomicChainReceipt::link(None, step).unwrap();
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        let mut observations = EconomicObservables::from_state_and_ledger(&post, &ledger);
        let closure = EconomicAccountingClosure::validate_and_seal(
            &state, &post, &assignments, &transitions,
        )
        .unwrap();

        observations.credit_created = 0;
        assert!(EconomicEvidenceCapsule::seal_with_accounting_closure(
            EconomicEvidenceManifest::new(
                "economics-v1",
                "params-closure",
                8,
                crate::economics::transition::state_hash(&state).unwrap(),
            )
            .unwrap(),
            &chain,
            &observations,
            &closure,
        )
        .is_err());
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
