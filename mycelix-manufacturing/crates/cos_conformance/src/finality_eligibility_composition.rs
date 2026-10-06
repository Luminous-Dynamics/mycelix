//! D6N/D6O finality-eligibility composition reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6N classifies what each external observation says about an effect.
//! D6O classifies whether that observation remains eligible under observer
//! lifecycle continuity. D6P joins those two evidence layers without making
//! lifecycle evidence itself an authority primitive.
//!
//! Current-finality witness counting is therefore:
//!
//!     D6N CorroboratingIndependent
//!       + exact D6O EligibleCurrent
//!       = current-finality-eligible witness
//!
//! Historical, blocked, archived, dependent, or contested lifecycle evidence
//! does not satisfy a current independent-observer threshold. Historical D6N
//! observations remain preserved even when later lifecycle changes prevent
//! current reuse.

use crate::contestable_finality::{
    verify_observation_set_assessment_provenance, verify_observation_set_provenance, ExternalObservedEvidenceV1,
    ExternalObservationSetV1, FinalityQualificationProfileV1, ObservationClassificationV1,
    ObservationSetAssessmentV1, ObservationSetDispositionV1, CONTESTABLE_FINALITY_CLAIM_CEILING,
};
use crate::observer_lifecycle::{
    verify_eligibility_receipt_provenance,
    EvidenceEligibilityDispositionV1, EvidenceEligibilityReceiptV1,
    ObserverEvidenceProvenanceV1, ObserverLifecycleLedgerV1, ObserverLifecycleProfileV1,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use crate::substitution_continuity::{ProviderRouteV1, SemanticEffectV1};
use std::collections::{BTreeMap, BTreeSet};

pub const FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING: &str =
    "D6N/D6O finality-eligibility composition reference semantics only; no semantic authority or actuation claim.";
pub const D6P_WITNESS_COMMITMENT_DOMAIN: &[u8] = b"MYCELIX-INTEGRAL-D6P-WITNESS-V1\0";
pub const D6P_RECEIPT_COMMITMENT_DOMAIN: &[u8] = b"MYCELIX-INTEGRAL-D6P-RECEIPT-V1\0";
pub const D6P_COMPOSITION_COMMITMENT_DOMAIN: &[u8] = b"MYCELIX-INTEGRAL-D6P-COMPOSITION-V1\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinalityEligibilityDispositionV1 {
    EligibleCurrent,
    InsufficientEligibleWitnesses,
    Contested,
    BlockedBinding,
    BlockedProfile,
    BlockedCurrentness,
    BlockedLifecycle,
    BlockedDependency,
    BlockedContinuity,
    BlockedArchive,
    BlockedAuthorization,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinalityCompositionRecordDispositionV1 {
    Recorded,
    BlockedDuplicate,
    Conflict,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityWitnessEligibilityV1 {
    pub observation_id: String,
    pub observer_id: String,
    pub observer_generation_id: Option<String>,
    pub d6n_observation_set_id: String,
    pub d6n_observation_set_commitment: String,
    /// Exact D6N assessment-item identity carried through the D6N/D6O join.
    /// This prevents a witness from silently retaining only set-level identity
    /// while its per-observation evidence/assessment object is substituted.
    pub d6n_assessment_item_commitment: String,
    pub d6n_classification: ObservationClassificationV1,
    pub d6o_eligibility_id: Option<String>,
    pub d6o_disposition: Option<EvidenceEligibilityDispositionV1>,
    pub d6o_dependency_snapshot_id: Option<String>,
    pub observation_frontier_root: String,
    pub current_frontier_root: String,
    pub lifecycle_profile_id: String,
    pub witness_commitment: String,
    pub claim_ceiling: String,
}

impl FinalityWitnessEligibilityV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.observation_id.is_empty()
            && !self.observer_id.is_empty()
            && !self.d6n_observation_set_id.is_empty()
            && !self.d6n_observation_set_commitment.is_empty()
            && !self.d6n_assessment_item_commitment.is_empty()
            && !self.observation_frontier_root.is_empty()
            && !self.current_frontier_root.is_empty()
            && !self.lifecycle_profile_id.is_empty()
            && !self.witness_commitment.is_empty()
            && self.claim_ceiling == FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING
    }

    /// Recompute the witness commitment from every semantic witness field.
    /// This binds the witness identity to the complete D6N/D6O join rather
    /// than a selected subset of fields. It is an integrity binding only;
    /// provenance and authority are established by the qualified D6P path.
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.witness_commitment.clear();
        let payload = serde_json::to_vec(&unsigned)
            .expect("D6P witness reference model must be serializable");
        let mut input = Vec::with_capacity(D6P_WITNESS_COMMITMENT_DOMAIN.len() + payload.len());
        input.extend_from_slice(D6P_WITNESS_COMMITMENT_DOMAIN);
        input.extend_from_slice(&payload);
        let digest = Sha256::digest(&input);
        digest.iter().map(|byte| format!("{byte:02x}")).collect()
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.witness_commitment == self.recomputed_commitment()
    }

    pub fn counts_as_current_independent_witness(&self) -> bool {
        matches!(
            (
                self.d6n_classification,
                self.d6o_disposition,
                &self.observer_generation_id,
                &self.d6o_eligibility_id,
            ),
            (
                ObservationClassificationV1::CorroboratingIndependent,
                Some(EvidenceEligibilityDispositionV1::EligibleCurrent),
                Some(_),
                Some(_)
            )
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityEligibilityCompositionV1 {
    pub composition_id: String,
    pub effect_id: String,
    pub effect_lineage_id: String,
    pub lifecycle_generation_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_operation_id: String,
    pub provider_profile_root: String,
    pub semantic_environment_root: String,
    pub observation_set_id: String,
    pub observation_set_commitment: String,
    pub d6n_assessment_commitment: String,
    pub lifecycle_profile_id: String,
    pub current_frontier_root: String,
    pub eligible_independent_count: u32,
    pub required_independent_observations: u32,
    pub preserved_contradictory_count: u32,
    pub witnesses: Vec<FinalityWitnessEligibilityV1>,
    pub disposition: FinalityEligibilityDispositionV1,
    pub qualification_transition_id: Option<String>,
    pub composition_commitment: String,
    pub claim_ceiling: String,
}

impl FinalityEligibilityCompositionV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.composition_id.is_empty()
            && !self.effect_id.is_empty()
            && !self.effect_lineage_id.is_empty()
            && !self.lifecycle_generation_id.is_empty()
            && !self.route_id.is_empty()
            && !self.provider_id.is_empty()
            && !self.provider_operation_id.is_empty()
            && !self.provider_profile_root.is_empty()
            && !self.semantic_environment_root.is_empty()
            && !self.observation_set_id.is_empty()
            && !self.observation_set_commitment.is_empty()
            && !self.d6n_assessment_commitment.is_empty()
            && !self.lifecycle_profile_id.is_empty()
            && !self.current_frontier_root.is_empty()
            && !self.composition_commitment.is_empty()
            && self.claim_ceiling == FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING
    }
    
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.composition_commitment.clear();
        let bytes = serde_json::to_vec(&unsigned)
            .expect("D6P composition reference model must be serializable");
        let mut input = Vec::with_capacity(D6P_COMPOSITION_COMMITMENT_DOMAIN.len() + bytes.len());
        input.extend_from_slice(D6P_COMPOSITION_COMMITMENT_DOMAIN);
        input.extend_from_slice(&bytes);
        let digest = Sha256::digest(&input);
        digest.iter().map(|byte| format!("{byte:02x}")).collect()
    }

    /// Validate the internal semantic relationships of a committed D6P
    /// composition without reconstructing D6N/D6O authority.
    ///
    /// This is deliberately narrower than authoritative reconstruction:
    /// callers can use it to reject a self-consistent-but-incoherent
    /// composition, while `compose_finality_eligibility` remains the source
    /// of truth for provenance.
    pub fn semantically_valid(&self) -> bool {
        if !self.commitment_matches()
            || self.required_independent_observations == 0
            || self.witnesses.is_empty()
            || self.composition_id != format!("composition:{}", self.observation_set_id)
        {
            return false;
        }

        let mut observation_ids = BTreeSet::new();
        let mut eligible_observer_ids = BTreeSet::new();
        let mut derived_contradictory_count = 0u32;

        for witness in &self.witnesses {
            if !witness.structurally_valid()
                || witness.d6n_observation_set_id != self.observation_set_id
                || witness.d6n_observation_set_commitment != self.observation_set_commitment
                || witness.observation_frontier_root != self.current_frontier_root
                || witness.current_frontier_root != self.current_frontier_root
                || witness.lifecycle_profile_id != self.lifecycle_profile_id
                || !observation_ids.insert(witness.observation_id.clone())
            {
                return false;
            }

            if !witness.commitment_matches() {
                return false;
            }

            let d6o_fields_present = witness.d6o_eligibility_id.is_some()
                || witness.d6o_disposition.is_some()
                || witness.d6o_dependency_snapshot_id.is_some()
                || witness.observer_generation_id.is_some();
            let d6o_fields_complete = witness.d6o_eligibility_id.is_some()
                && witness.d6o_disposition.is_some()
                && witness.d6o_dependency_snapshot_id.is_some()
                && witness.observer_generation_id.is_some();
            if d6o_fields_present != d6o_fields_complete
                || witness
                    .d6o_eligibility_id
                    .as_deref()
                    .is_some_and(str::is_empty)
                || witness
                    .d6o_dependency_snapshot_id
                    .as_deref()
                    .is_some_and(str::is_empty)
                || witness
                    .observer_generation_id
                    .as_deref()
                    .is_some_and(str::is_empty)
            {
                return false;
            }

            if witness.counts_as_current_independent_witness() {
                eligible_observer_ids.insert(witness.observer_id.clone());
            }

            if matches!(
                witness.d6n_classification,
                ObservationClassificationV1::ContradictoryIndependent
                    | ObservationClassificationV1::ContradictoryDependent
            ) {
                derived_contradictory_count += 1;
            }
        }

        if self.eligible_independent_count != eligible_observer_ids.len() as u32
            || self.preserved_contradictory_count != derived_contradictory_count
        {
            return false;
        }

        match self.disposition {
            FinalityEligibilityDispositionV1::EligibleCurrent => {
                self.preserved_contradictory_count == 0
                    && self.eligible_independent_count >= self.required_independent_observations
                    && self
                        .qualification_transition_id
                        .as_deref()
                        .is_some_and(|id| !id.is_empty())
            }
            FinalityEligibilityDispositionV1::Contested => {
                self.preserved_contradictory_count > 0
                    && self.qualification_transition_id.is_none()
            }
            FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses => {
                self.preserved_contradictory_count == 0
                    && self.eligible_independent_count < self.required_independent_observations
                    && self.qualification_transition_id.is_none()
            }
            FinalityEligibilityDispositionV1::BlockedBinding
            | FinalityEligibilityDispositionV1::BlockedProfile
            | FinalityEligibilityDispositionV1::BlockedCurrentness
            | FinalityEligibilityDispositionV1::BlockedLifecycle
            | FinalityEligibilityDispositionV1::BlockedDependency
            | FinalityEligibilityDispositionV1::BlockedContinuity
            | FinalityEligibilityDispositionV1::BlockedArchive
            | FinalityEligibilityDispositionV1::BlockedAuthorization => false,
        }
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.composition_commitment == self.recomputed_commitment()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentFinalityEligibilityReceiptV1 {
    pub receipt_id: String,
    pub effect_id: String,
    pub effect_lineage_id: String,
    pub lifecycle_generation_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_operation_id: String,
    pub provider_profile_root: String,
    pub semantic_environment_root: String,
    pub observation_set_id: String,
    pub observation_set_commitment: String,
    pub d6n_assessment_commitment: String,
    /// Candidate-independent D6P composition identity that this receipt projects.
    /// This links the qualified receipt to the exact committed composition without
    /// requiring downstream D6S consumers to reconstruct D6N/D6O semantics.
    pub composition_commitment: String,
    pub witness_eligibility_ids: BTreeSet<String>,
    pub observer_generation_ids: BTreeSet<String>,
    pub current_frontier_root: String,
    pub lifecycle_profile_id: String,
    pub eligible_independent_count: u32,
    pub preserved_contradictory_count: u32,
    pub disposition: FinalityEligibilityDispositionV1,
    pub qualification_transition_id: String,
    pub receipt_commitment: String,
    pub claim_ceiling: String,
}

impl CurrentFinalityEligibilityReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.receipt_id.is_empty()
            && !self.effect_id.is_empty()
            && !self.effect_lineage_id.is_empty()
            && !self.lifecycle_generation_id.is_empty()
            && !self.route_id.is_empty()
            && !self.provider_id.is_empty()
            && !self.provider_operation_id.is_empty()
            && !self.provider_profile_root.is_empty()
            && !self.semantic_environment_root.is_empty()
            && !self.observation_set_id.is_empty()
            && !self.observation_set_commitment.is_empty()
            && !self.d6n_assessment_commitment.is_empty()
            && !self.composition_commitment.is_empty()
            && !self.witness_eligibility_ids.is_empty()
            && !self.observer_generation_ids.is_empty()
            && !self.current_frontier_root.is_empty()
            && !self.lifecycle_profile_id.is_empty()
            && !self.qualification_transition_id.is_empty()
            && !self.receipt_commitment.is_empty()
            && self.claim_ceiling == FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING
    }

    /// Recompute the D6P receipt commitment from every semantic receipt field.
    ///
    /// This is an integrity binding only; it does not confer finality,
    /// authority, truth, or actuation rights.
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.receipt_commitment.clear();
        let bytes = serde_json::to_vec(&unsigned)
            .expect("D6P receipt reference model must be serializable");
        let mut input =
            Vec::with_capacity(D6P_RECEIPT_COMMITMENT_DOMAIN.len() + bytes.len());
        input.extend_from_slice(D6P_RECEIPT_COMMITMENT_DOMAIN);
        input.extend_from_slice(&bytes);
        let digest = Sha256::digest(&input);
        digest.iter().map(|byte| format!("{byte:02x}")).collect()
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.receipt_commitment == self.recomputed_commitment()
    }

    /// Validate the receipt's internal coherence without asserting provenance
    /// from the authoritative D6P source inputs.
    pub fn semantically_valid(&self) -> bool {
        if !self.commitment_matches()
            || self.eligible_independent_count == 0
            || self.witness_eligibility_ids.is_empty()
            || self.observer_generation_ids.is_empty()
            || self.witness_eligibility_ids.iter().any(String::is_empty)
            || self.observer_generation_ids.iter().any(String::is_empty)
        {
            return false;
        }

        match self.disposition {
            FinalityEligibilityDispositionV1::EligibleCurrent => {
                self.preserved_contradictory_count == 0
                    && self.eligible_independent_count
                        == self.witness_eligibility_ids.len() as u32
                    && self.witness_eligibility_ids.len()
                        == self.observer_generation_ids.len()
                    && !self.qualification_transition_id.is_empty()
            }
            _ => false,
        }
    }
}

/// Verify that a D6P receipt is an exact projection of the authoritative
/// composition it claims to represent. Commitment equality alone is insufficient:
/// both objects may be attacker-generated and internally self-consistent.
pub fn verify_current_receipt_provenance_from_composition(
receipt: &CurrentFinalityEligibilityReceiptV1,
composition: &FinalityEligibilityCompositionV1,
) -> bool {
receipt.commitment_matches()
    && receipt.semantically_valid()
    && composition.semantically_valid()
    && composition.commitment_matches()
    && receipt.composition_commitment == composition.composition_commitment
    && receipt.effect_id == composition.effect_id
    && receipt.effect_lineage_id == composition.effect_lineage_id
    && receipt.lifecycle_generation_id == composition.lifecycle_generation_id
    && receipt.route_id == composition.route_id
    && receipt.provider_id == composition.provider_id
    && receipt.provider_operation_id == composition.provider_operation_id
    && receipt.provider_profile_root == composition.provider_profile_root
    && receipt.semantic_environment_root == composition.semantic_environment_root
    && receipt.observation_set_id == composition.observation_set_id
    && receipt.observation_set_commitment == composition.observation_set_commitment
    && receipt.d6n_assessment_commitment == composition.d6n_assessment_commitment
    && receipt.witness_eligibility_ids
        == composition.witnesses.iter().filter_map(|w| w.d6o_eligibility_id.clone()).collect()
    && receipt.observer_generation_ids
        == composition.witnesses.iter().filter_map(|w| w.observer_generation_id.clone()).collect()
    && receipt.current_frontier_root == composition.current_frontier_root
    && receipt.lifecycle_profile_id == composition.lifecycle_profile_id
    && receipt.eligible_independent_count == composition.eligible_independent_count
    && receipt.preserved_contradictory_count == composition.preserved_contradictory_count
    && receipt.disposition == composition.disposition
    && receipt.qualification_transition_id
        == composition.qualification_transition_id.clone().unwrap_or_default()
}


#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityEligibilityLedgerV1 {
    /// Single-view witness registry keyed by observation ID. This registry
    /// deliberately rejects a second witness version for the same observation;
    /// historical frontier versions are preserved inside immutable D6P
    /// compositions rather than inferred from this convenience view.
    pub witnesses: BTreeMap<String, FinalityWitnessEligibilityV1>,
    /// Immutable composition versions keyed by canonical composition commitment.
    /// The semantic composition id may recur across frontier versions.
    pub compositions: BTreeMap<String, FinalityEligibilityCompositionV1>,
    pub receipts: BTreeMap<String, CurrentFinalityEligibilityReceiptV1>,
    /// Most recently recorded terminal receipt for each effect. This is only a
    /// convenience index; because frontier roots are opaque identifiers, arrival
    /// order cannot establish which frontier is authoritative.
    pub terminal_receipt_by_effect: BTreeMap<String, String>,
    /// Exact terminal receipt selected by effect and frontier. This is the
    /// strict selection index and is independent of arrival order.
    pub terminal_receipt_by_effect_and_frontier: BTreeMap<(String, String), String>,
}

impl FinalityEligibilityLedgerV1 {
    pub fn record_witness(
        &mut self,
        witness: FinalityWitnessEligibilityV1,
    ) -> FinalityCompositionRecordDispositionV1 {
        if !witness.commitment_matches() {
            return FinalityCompositionRecordDispositionV1::InsufficientEvidence;
        }
        match self.witnesses.get(&witness.observation_id) {
            Some(existing) if existing == &witness => {
                FinalityCompositionRecordDispositionV1::BlockedDuplicate
            }
            Some(_) => FinalityCompositionRecordDispositionV1::Conflict,
            None => {
                self.witnesses
                    .insert(witness.observation_id.clone(), witness);
                FinalityCompositionRecordDispositionV1::Recorded
            }
        }
    }

    /// Return a witness only when the stored witness is bound to the exact
    /// frontier requested by the caller. This is a selection/binding check,
    /// not an authority proof: the caller must still establish that the
    /// expected frontier is authoritative for its trust domain.
    pub fn witness_at_frontier(
        &self,
        observation_id: &str,
        expected_frontier_root: &str,
    ) -> Option<&FinalityWitnessEligibilityV1> {
        if observation_id.is_empty() || expected_frontier_root.is_empty() {
            return None;
        }
        let witness = self.witnesses.get(observation_id)?;
        (witness.current_frontier_root == expected_frontier_root
            && witness.observation_frontier_root == expected_frontier_root
            && witness.commitment_matches())
            .then_some(witness)
    }

    pub fn record_composition(
        &mut self,
        composition: FinalityEligibilityCompositionV1,
    ) -> FinalityCompositionRecordDispositionV1 {
        if !composition.semantically_valid() {
            return FinalityCompositionRecordDispositionV1::InsufficientEvidence;
        }
        // The semantic composition id identifies the observation-set lineage,
        // not a single point-in-time state. Store immutable versions by their
        // content commitment so later frontiers remain representable without
        // overwriting history.
        if let Some(existing) = self
            .compositions
            .values()
            .find(|existing| existing.composition_commitment == composition.composition_commitment)
        {
            return if existing == &composition {
                FinalityCompositionRecordDispositionV1::BlockedDuplicate
            } else {
                FinalityCompositionRecordDispositionV1::Conflict
            };
        }
        if self.compositions.values().any(|existing| {
            existing.composition_id == composition.composition_id
                && existing.current_frontier_root == composition.current_frontier_root
        }) {
            return FinalityCompositionRecordDispositionV1::Conflict;
        }
        self.compositions
            .insert(composition.composition_commitment.clone(), composition);
        FinalityCompositionRecordDispositionV1::Recorded
    }

    /// Return a composition only when its canonical commitment and expected
    /// frontier agree. This is a selection/binding check, not an authority
    /// proof: callers must still establish that the expected frontier is
    /// authoritative for their trust domain.
    pub fn composition_at_frontier(
        &self,
        composition_commitment: &str,
        expected_frontier_root: &str,
    ) -> Option<&FinalityEligibilityCompositionV1> {
        if composition_commitment.is_empty() || expected_frontier_root.is_empty() {
            return None;
        }
        let composition = self.compositions.get(composition_commitment)?;
        (composition.current_frontier_root == expected_frontier_root
            && composition.composition_commitment == composition_commitment
            && composition.semantically_valid())
            .then_some(composition)
    }

    /// Return the terminal receipt selected for an effect only when its
    /// indexed receipt, composition, and frontier all agree. The caller still
    /// supplies the expected frontier; the ledger never self-declares that
    /// its latest index is authoritative.
    pub fn terminal_receipt_at_frontier(
        &self,
        effect_id: &str,
        expected_frontier_root: &str,
    ) -> Option<&CurrentFinalityEligibilityReceiptV1> {
        if effect_id.is_empty() || expected_frontier_root.is_empty() {
            return None;
        }
        let receipt_id = self
            .terminal_receipt_by_effect_and_frontier
            .get(&(effect_id.to_owned(), expected_frontier_root.to_owned()))?;
        let receipt = self.receipts.get(receipt_id)?;
        if receipt.effect_id != effect_id
            || receipt.current_frontier_root != expected_frontier_root
            || !receipt.commitment_matches()
            || !matches!(
                receipt.disposition,
                FinalityEligibilityDispositionV1::EligibleCurrent
            )
        {
            return None;
        }
        let composition = self.compositions.get(&receipt.composition_commitment)?;
        if composition.current_frontier_root != expected_frontier_root
            || !current_receipt_matches_composition(receipt, composition)
            || !composition.semantically_valid()
        {
            return None;
        }
        Some(receipt)
    }

    pub fn record_receipt(
        &mut self,
        receipt: CurrentFinalityEligibilityReceiptV1,
    ) -> FinalityCompositionRecordDispositionV1 {
        if !receipt.structurally_valid() || !receipt.commitment_matches() {
            return FinalityCompositionRecordDispositionV1::InsufficientEvidence;
        }
        let Some(composition) = self.compositions.values().find(|composition| {
            composition.composition_commitment == receipt.composition_commitment
        }) else {
            return FinalityCompositionRecordDispositionV1::InsufficientEvidence;
        };
        if !current_receipt_matches_composition(&receipt, composition) {
            return FinalityCompositionRecordDispositionV1::Conflict;
        }
        let terminal = matches!(
            receipt.disposition,
            FinalityEligibilityDispositionV1::EligibleCurrent
        );
        if terminal {
            let frontier_key = (
                receipt.effect_id.clone(),
                receipt.current_frontier_root.clone(),
            );
            if let Some(existing_id) = self
                .terminal_receipt_by_effect_and_frontier
                .get(&frontier_key)
            {
                if existing_id != &receipt.receipt_id {
                    return FinalityCompositionRecordDispositionV1::Conflict;
                }
            }
        }
        match self.receipts.get(&receipt.receipt_id) {
            Some(existing) if existing == &receipt => {
                FinalityCompositionRecordDispositionV1::BlockedDuplicate
            }
            Some(_) => FinalityCompositionRecordDispositionV1::Conflict,
            None => {
                if terminal {
                    self.terminal_receipt_by_effect
                        .insert(receipt.effect_id.clone(), receipt.receipt_id.clone());
                    self.terminal_receipt_by_effect_and_frontier.insert(
                        (
                            receipt.effect_id.clone(),
                            receipt.current_frontier_root.clone(),
                        ),
                        receipt.receipt_id.clone(),
                    );
                }
                self.receipts.insert(receipt.receipt_id.clone(), receipt);
                FinalityCompositionRecordDispositionV1::Recorded
            }
        }
    }
}

fn d6n_assessment_is_exact(
    set: &ExternalObservationSetV1,
    assessment: &ObservationSetAssessmentV1,
    required_independent_observations: u32,
) -> bool {
    if !assessment.structurally_valid()
        || assessment.set_id != set.set_id
        || assessment.claim_ceiling != CONTESTABLE_FINALITY_CLAIM_CEILING
        || assessment.assessment_commitment != format!("assessment:{}", set.set_commitment)
        || assessment.assessments.len() != set.observation_ids.len()
    {
        return false;
    }

    let mut observation_ids = BTreeSet::new();
    let mut independent_count = 0u32;
    let mut contradictory_independent_count = 0u32;
    let mut dependent_count = 0u32;

    for item in &assessment.assessments {
        if !item.structurally_valid()
            || !set.observation_ids.contains(&item.observation_id)
            || !observation_ids.insert(item.observation_id.clone())
            || item.assessment_commitment
                != format!(
                    "{}:{}:{}",
                    item.observation_id, item.evidence_root, item.custody_root
                )
        {
            return false;
        }

        // D6P consumes the D6N assessment as a qualified semantic
        // boundary. An Independent classification must agree with the
        // independence declaration that produced it; otherwise a
        // self-consistent assessment can relabel dependent evidence as
        // independent without changing its commitment shape.
        if matches!(
            item.classification,
            ObservationClassificationV1::CorroboratingIndependent
                | ObservationClassificationV1::ContradictoryIndependent
        ) && item.independence
            != crate::contestable_finality::ObservationIndependenceV1::DeclaredIndependent
        {
            return false;
        }

        if matches!(
            item.classification,
            ObservationClassificationV1::CorroboratingIndependent
        ) {
            independent_count += 1;
        }
        if matches!(
            item.classification,
            ObservationClassificationV1::ContradictoryIndependent
        ) {
            contradictory_independent_count += 1;
        }
        if matches!(
            item.classification,
            ObservationClassificationV1::CorroboratingDependent
                | ObservationClassificationV1::ContradictoryDependent
        ) {
            dependent_count += 1;
        }
    }

    if observation_ids != set.observation_ids
        || assessment.independent_count != independent_count
        || assessment.contradictory_independent_count != contradictory_independent_count
        || assessment.dependent_count != dependent_count
    {
        return false;
    }

    let expected_disposition = if contradictory_independent_count > 0 {
        ObservationSetDispositionV1::Contested
    } else if independent_count >= required_independent_observations {
        ObservationSetDispositionV1::QualifiedEvidence
    } else {
        ObservationSetDispositionV1::InsufficientEvidence
    };

    assessment.disposition == expected_disposition
}

fn observation_matches_set(
    observation: &ExternalObservedEvidenceV1,
    set: &ExternalObservationSetV1,
) -> bool {
    let o = &observation.observation;
    o.commitment_matches()
        && o.effect_id == set.effect_id
        && o.effect_lineage_id == set.effect_lineage_id
        && o.lifecycle_generation_id == set.lifecycle_generation_id
        && o.route_id == set.route_id
        && o.provider_id == set.provider_id
        && o.provider_operation_id == set.provider_operation_id
        && o.provider_profile_root == set.provider_profile_root
        && o.semantic_environment_root == set.semantic_environment_root
        && o.observed_frontier_root == set.observation_frontier_root
}

fn receipt_binding_failure(
    receipt: &EvidenceEligibilityReceiptV1,
    evidence: &ExternalObservedEvidenceV1,
    assessment: &crate::contestable_finality::ObservationAssessmentV1,
    set: &ExternalObservationSetV1,
    lifecycle_profile_id: &str,
    current_frontier_root: &str,
) -> Option<FinalityEligibilityDispositionV1> {
    if !receipt.commitment_matches() {
        return Some(FinalityEligibilityDispositionV1::BlockedBinding);
    }
    if receipt.observation_id != evidence.observation.observation_id
        || receipt.observer_id != evidence.observer_id
        || receipt.classification != assessment.classification
    {
        return Some(FinalityEligibilityDispositionV1::BlockedBinding);
    }
    if receipt.observation_profile_id != evidence.observer.observation_profile_id
        || receipt.semantic_environment_root != set.semantic_environment_root
        || receipt.qualification_profile_id != lifecycle_profile_id
    {
        return Some(FinalityEligibilityDispositionV1::BlockedProfile);
    }
    if receipt.observation_frontier_root != set.observation_frontier_root
        || receipt.current_frontier_root != current_frontier_root
    {
        return Some(FinalityEligibilityDispositionV1::BlockedCurrentness);
    }
    if receipt.current_generation_id.as_deref() != Some(receipt.observer_generation_id.as_str()) {
        return Some(FinalityEligibilityDispositionV1::BlockedContinuity);
    }
    if receipt.provenance != ObserverEvidenceProvenanceV1::Live {
        return Some(FinalityEligibilityDispositionV1::BlockedArchive);
    }
    if receipt.disposition == EvidenceEligibilityDispositionV1::EligibleCurrent {
        None
    } else {
        Some(map_receipt_failure(
            receipt,
            current_frontier_root,
            lifecycle_profile_id,
        ))
    }
}

fn witness_from(
    assessment: &crate::contestable_finality::ObservationAssessmentV1,
    evidence: &ExternalObservedEvidenceV1,
    receipt: Option<&EvidenceEligibilityReceiptV1>,
    set: &ExternalObservationSetV1,
    lifecycle_profile_id: &str,
    current_frontier_root: &str,
) -> FinalityWitnessEligibilityV1 {
    let mut witness = FinalityWitnessEligibilityV1 {
        observation_id: assessment.observation_id.clone(),
        observer_id: assessment.observer_id.clone(),
        observer_generation_id: receipt.map(|r| r.observer_generation_id.clone()),
        d6n_observation_set_id: set.set_id.clone(),
        d6n_observation_set_commitment: set.set_commitment.clone(),
        d6n_assessment_item_commitment: assessment.assessment_commitment.clone(),
        d6n_classification: assessment.classification,
        d6o_eligibility_id: receipt.map(|r| r.eligibility_id.clone()),
        d6o_disposition: receipt.map(|r| r.disposition),
        d6o_dependency_snapshot_id: receipt.map(|r| r.dependency_snapshot_id.clone()),
        observation_frontier_root: set.observation_frontier_root.clone(),
        current_frontier_root: current_frontier_root.to_owned(),
        lifecycle_profile_id: lifecycle_profile_id.to_owned(),
        witness_commitment: String::new(),
        claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.to_owned(),
    };
    let mut witness = witness;
    witness.witness_commitment = witness.recomputed_commitment();
    witness
}

/// Verify that a D6P witness is the exact join of one D6N assessment item,
/// one D6N evidence object, and (when present) one D6O eligibility receipt.
///
/// The individual objects may each be internally valid while still referring to
/// different revisions of the same observation identifier. This predicate makes
/// the join itself an explicit semantic boundary rather than relying on shared
/// IDs alone.
pub fn verify_witness_join_binding(
    witness: &FinalityWitnessEligibilityV1,
    assessment: &crate::contestable_finality::ObservationAssessmentV1,
    evidence: &ExternalObservedEvidenceV1,
    receipt: Option<&EvidenceEligibilityReceiptV1>,
    set: &ExternalObservationSetV1,
    lifecycle_profile_id: &str,
    current_frontier_root: &str,
) -> bool {
    if !witness.commitment_matches()
        || !assessment.commitment_matches()
        || !evidence.structurally_valid()
        || !evidence.observation.commitment_matches()
        || !set.structurally_valid()
        || lifecycle_profile_id.is_empty()
        || current_frontier_root.is_empty()
    {
        return false;
    }

    let observation = &evidence.observation;

    if witness.observation_id != assessment.observation_id
        || witness.observation_id != observation.observation_id
        || witness.observer_id != assessment.observer_id
        || witness.observer_id != evidence.observer.observer_id
        || witness.d6n_observation_set_id != set.set_id
        || witness.d6n_observation_set_commitment != set.set_commitment
        || witness.d6n_assessment_item_commitment != assessment.assessment_commitment
        || witness.d6n_classification != assessment.classification
        || assessment.observation_commitment != observation.observation_commitment
        || assessment.evidence_root != evidence.observer.evidence_root
        || assessment.custody_root != evidence.observer.custody_root
        || !observation_matches_set(evidence, set)
        || witness.observation_frontier_root != set.observation_frontier_root
        || witness.current_frontier_root != current_frontier_root
        || witness.lifecycle_profile_id != lifecycle_profile_id
    {
        return false;
    }

    match receipt {
        Some(receipt) => {
            receipt.commitment_matches()
                && receipt.qualification_profile_id == lifecycle_profile_id
                && witness.observer_generation_id.as_deref() == Some(receipt.observer_generation_id.as_str())
                && witness.d6o_eligibility_id.as_deref() == Some(receipt.eligibility_id.as_str())
                && witness.d6o_disposition == Some(receipt.disposition)
                && witness.d6o_dependency_snapshot_id.as_deref()
                    == Some(receipt.dependency_snapshot_id.as_str())
                && receipt.observation_id == observation.observation_id
                && receipt.observer_id == evidence.observer.observer_id
                && receipt.classification == assessment.classification
                && receipt.observation_profile_id == evidence.observer.observation_profile_id
                && receipt.semantic_environment_root == set.semantic_environment_root
                && receipt.observation_frontier_root == set.observation_frontier_root
                && receipt.current_frontier_root == current_frontier_root
        }
        None => {
            witness.observer_generation_id.is_none()
                && witness.d6o_eligibility_id.is_none()
                && witness.d6o_disposition.is_none()
                && witness.d6o_dependency_snapshot_id.is_none()
        }
    }
}

fn map_receipt_failure(
    receipt: &EvidenceEligibilityReceiptV1,
    current_frontier_root: &str,
    lifecycle_profile_id: &str,
) -> FinalityEligibilityDispositionV1 {
    if receipt.current_frontier_root != current_frontier_root {
        FinalityEligibilityDispositionV1::BlockedCurrentness
    } else if receipt.qualification_profile_id != lifecycle_profile_id {
        FinalityEligibilityDispositionV1::BlockedProfile
    } else {
        match receipt.disposition {
            EvidenceEligibilityDispositionV1::BlockedLifecycle => {
                FinalityEligibilityDispositionV1::BlockedLifecycle
            }
            EvidenceEligibilityDispositionV1::BlockedDependency => {
                FinalityEligibilityDispositionV1::BlockedDependency
            }
            EvidenceEligibilityDispositionV1::BlockedContinuity => {
                FinalityEligibilityDispositionV1::BlockedContinuity
            }
            EvidenceEligibilityDispositionV1::BlockedCurrentness => {
                FinalityEligibilityDispositionV1::BlockedCurrentness
            }
            EvidenceEligibilityDispositionV1::BlockedProfile => {
                FinalityEligibilityDispositionV1::BlockedProfile
            }
            EvidenceEligibilityDispositionV1::BlockedArchive => {
                FinalityEligibilityDispositionV1::BlockedArchive
            }
            EvidenceEligibilityDispositionV1::BlockedAuthorization => {
                FinalityEligibilityDispositionV1::BlockedAuthorization
            }
            EvidenceEligibilityDispositionV1::Contested => {
                FinalityEligibilityDispositionV1::Contested
            }
            EvidenceEligibilityDispositionV1::InsufficientEvidence
            | EvidenceEligibilityDispositionV1::HistoricalOnly
            | EvidenceEligibilityDispositionV1::EligibleCurrent => {
                FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
            }
        }
    }
}

pub fn compose_finality_eligibility(
    set: &ExternalObservationSetV1,
    assessment: &ObservationSetAssessmentV1,
    evidence: &[ExternalObservedEvidenceV1],
    eligibility_receipts: &[EvidenceEligibilityReceiptV1],
    lifecycle_profile_id: &str,
    current_frontier_root: &str,
    required_independent_observations: u32,
) -> FinalityEligibilityCompositionV1 {
    let empty = |disposition| FinalityEligibilityCompositionV1 {
        composition_id: format!("composition:{}", set.set_id),
        effect_id: set.effect_id.clone(),
        effect_lineage_id: set.effect_lineage_id.clone(),
        lifecycle_generation_id: set.lifecycle_generation_id.clone(),
        route_id: set.route_id.clone(),
        provider_id: set.provider_id.clone(),
        provider_operation_id: set.provider_operation_id.clone(),
        provider_profile_root: set.provider_profile_root.clone(),
        semantic_environment_root: set.semantic_environment_root.clone(),
        observation_set_id: set.set_id.clone(),
        observation_set_commitment: set.set_commitment.clone(),
        d6n_assessment_commitment: assessment.assessment_commitment.clone(),
        lifecycle_profile_id: lifecycle_profile_id.to_owned(),
        current_frontier_root: current_frontier_root.to_owned(),
        eligible_independent_count: 0,
        required_independent_observations,
        preserved_contradictory_count: assessment
            .assessments
            .iter()
            .filter(|a| {
                matches!(
                    a.classification,
                    ObservationClassificationV1::ContradictoryIndependent
                        | ObservationClassificationV1::ContradictoryDependent
                )
            })
            .count() as u32,
        witnesses: Vec::new(),
        disposition,
        qualification_transition_id: None,
        composition_commitment: "blocked".to_owned(),
        claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.to_owned(),
    };

    if !set.structurally_valid()
        || !d6n_assessment_is_exact(
            set,
            assessment,
            required_independent_observations,
        )
        || lifecycle_profile_id.is_empty()
        || current_frontier_root.is_empty()
        || required_independent_observations == 0
    {
        return empty(FinalityEligibilityDispositionV1::BlockedBinding);
    }

    if set.observation_frontier_root != current_frontier_root {
        return empty(FinalityEligibilityDispositionV1::BlockedCurrentness);
    }

    let mut evidence_by_id = BTreeMap::new();
    for item in evidence {
        if item.structurally_valid() {
            let id = item.observation.observation_id.clone();
            if evidence_by_id.contains_key(&id) {
                return empty(FinalityEligibilityDispositionV1::BlockedBinding);
            }
            evidence_by_id.insert(id, item);
        }
    }

    let mut receipt_by_id = BTreeMap::new();
    for receipt in eligibility_receipts {
        if receipt.structurally_valid() {
            let id = receipt.observation_id.clone();
            if receipt_by_id.contains_key(&id) {
                return empty(FinalityEligibilityDispositionV1::BlockedBinding);
            }
            receipt_by_id.insert(id, receipt);
        }
    }

    if assessment.assessments.len() != set.observation_ids.len()
        || assessment
            .assessments
            .iter()
            .any(|item| !set.observation_ids.contains(&item.observation_id))
    {
        return empty(FinalityEligibilityDispositionV1::BlockedBinding);
    }

    let mut witnesses = Vec::with_capacity(assessment.assessments.len());
    let mut eligible_observer_ids = BTreeSet::new();
    let mut preserved_contradictory_count = 0u32;
    let mut blocking_dispositions = Vec::new();

    for item in &assessment.assessments {
        let Some(observation) = evidence_by_id.get(&item.observation_id) else {
            return empty(FinalityEligibilityDispositionV1::BlockedBinding);
        };

        if !observation_matches_set(observation, set)
            || item.observer_id != observation.observer.observer_id
            || item.evidence_root != observation.observer.evidence_root
            || item.custody_root != observation.observer.custody_root
            || item.assessment_commitment.is_empty()
        {
            return empty(FinalityEligibilityDispositionV1::BlockedBinding);
        }

        if matches!(
            item.classification,
            ObservationClassificationV1::ContradictoryIndependent
                | ObservationClassificationV1::ContradictoryDependent
        ) {
            preserved_contradictory_count += 1;
        }

        let receipt = receipt_by_id.get(&item.observation_id).copied();
        let witness = witness_from(
            item,
            observation,
            receipt,
            set,
            lifecycle_profile_id,
            current_frontier_root,
        );

        if matches!(
            item.classification,
            ObservationClassificationV1::CorroboratingIndependent
        ) {
            if let Some(receipt) = receipt {
                if let Some(failure) = receipt_binding_failure(
                    receipt,
                    observation,
                    item,
                    set,
                    lifecycle_profile_id,
                    current_frontier_root,
                ) {
                    blocking_dispositions.push(failure);
                } else {
                    eligible_observer_ids.insert(item.observer_id.clone());
                }
            }
        }

        witnesses.push(witness);
    }

    let mut distinct_blocking_dispositions = Vec::new();
    for disposition in blocking_dispositions {
        if !distinct_blocking_dispositions.contains(&disposition) {
            distinct_blocking_dispositions.push(disposition);
        }
    }

    let disposition = if preserved_contradictory_count > 0 {
        FinalityEligibilityDispositionV1::Contested
    } else if eligible_observer_ids.len() as u32 >= required_independent_observations {
        FinalityEligibilityDispositionV1::EligibleCurrent
    } else if distinct_blocking_dispositions.len() == 1 {
        distinct_blocking_dispositions[0]
    } else {
        FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
    };

    let mut composition = FinalityEligibilityCompositionV1 {
        composition_id: format!("composition:{}", set.set_id),
        effect_id: set.effect_id.clone(),
        effect_lineage_id: set.effect_lineage_id.clone(),
        lifecycle_generation_id: set.lifecycle_generation_id.clone(),
        route_id: set.route_id.clone(),
        provider_id: set.provider_id.clone(),
        provider_operation_id: set.provider_operation_id.clone(),
        provider_profile_root: set.provider_profile_root.clone(),
        semantic_environment_root: set.semantic_environment_root.clone(),
        observation_set_id: set.set_id.clone(),
        observation_set_commitment: set.set_commitment.clone(),
        d6n_assessment_commitment: assessment.assessment_commitment.clone(),
        lifecycle_profile_id: lifecycle_profile_id.to_owned(),
        current_frontier_root: current_frontier_root.to_owned(),
        eligible_independent_count: eligible_observer_ids.len() as u32,
        required_independent_observations,
        preserved_contradictory_count,
        witnesses,
        disposition,
        qualification_transition_id: if matches!(
            disposition,
            FinalityEligibilityDispositionV1::EligibleCurrent
        ) {
            Some("qualified-finality-eligibility-transition".to_owned())
        } else {
            None
        },
        composition_commitment: String::new(),
        claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.to_owned(),
    };
    composition.composition_commitment = composition.recomputed_commitment();
    composition
}

/// Verify that a D6P current-finality receipt is a faithful projection of
/// the authoritative D6P composition that produced it.
///
/// This closes the gap between receipt self-integrity and provenance: a receipt
/// may be cryptographically self-consistent while still describing a different
/// composition unless its semantic fields and witness sets are reconstructed
/// from the D6P composition.
pub fn current_receipt_matches_composition(
    receipt: &CurrentFinalityEligibilityReceiptV1,
    composition: &FinalityEligibilityCompositionV1,
) -> bool {
    if !receipt.commitment_matches() || !composition.semantically_valid() {
        return false;
    }

    if receipt.effect_id != composition.effect_id
        || receipt.effect_lineage_id != composition.effect_lineage_id
        || receipt.lifecycle_generation_id != composition.lifecycle_generation_id
        || receipt.route_id != composition.route_id
        || receipt.provider_id != composition.provider_id
        || receipt.provider_operation_id != composition.provider_operation_id
        || receipt.provider_profile_root != composition.provider_profile_root
        || receipt.semantic_environment_root != composition.semantic_environment_root
        || receipt.observation_set_id != composition.observation_set_id
        || receipt.observation_set_commitment != composition.observation_set_commitment
        || receipt.d6n_assessment_commitment != composition.d6n_assessment_commitment
        || receipt.composition_commitment != composition.composition_commitment
        || receipt.current_frontier_root != composition.current_frontier_root
        || receipt.lifecycle_profile_id != composition.lifecycle_profile_id
        || receipt.eligible_independent_count != composition.eligible_independent_count
        || receipt.preserved_contradictory_count != composition.preserved_contradictory_count
        || receipt.disposition != composition.disposition
        || receipt.claim_ceiling != composition.claim_ceiling
    {
        return false;
    }

    let expected_witness_eligibility_ids = composition
        .witnesses
        .iter()
        .filter_map(|witness| witness.d6o_eligibility_id.clone())
        .collect::<BTreeSet<_>>();
    let expected_observer_generation_ids = composition
        .witnesses
        .iter()
        .filter_map(|witness| witness.observer_generation_id.clone())
        .collect::<BTreeSet<_>>();

    if receipt.witness_eligibility_ids != expected_witness_eligibility_ids
        || receipt.observer_generation_ids != expected_observer_generation_ids
    {
        return false;
    }

    composition.qualification_transition_id.as_deref()
        == Some(receipt.qualification_transition_id.as_str())
}

/// Reconstruct the authoritative D6P composition from its source inputs and
/// require the supplied receipt to be an exact projection of that result.
/// Compose D6P only after independently reconstructing every supplied
/// current-eligibility D6O receipt from the authoritative lifecycle ledger.
///
/// The legacy compose function intentionally remains a ReferenceModelOnly
/// convenience API. This boundary is the qualified path: a receipt can
/// contribute to current-finality composition only after D6O provenance has
/// been checked against authoritative generation, dependency, profile,
/// transition, and current-continuity state.
///
/// Supplied receipts that fail authoritative D6O reconstruction are fail-closed
/// rather than being allowed to contribute merely because their own commitment
/// is internally consistent.
pub fn compose_finality_eligibility_from_authoritative_d6n_d6o(
    effect: &SemanticEffectV1,
    route: &ProviderRouteV1,
    profile: &FinalityQualificationProfileV1,
    expected_finality_profile_commitment: &str,
    set: &ExternalObservationSetV1,
    assessment: &ObservationSetAssessmentV1,
    evidence: &[ExternalObservedEvidenceV1],
    eligibility_receipts: &[EvidenceEligibilityReceiptV1],
    lifecycle_profile: &ObserverLifecycleProfileV1,
    expected_lifecycle_profile_commitment: &str,
    d6o_ledger: &ObserverLifecycleLedgerV1,
    current_frontier_root: &str,
    live_generation_id: &str,
    required_independent_observations: u32,
) -> Option<FinalityEligibilityCompositionV1> {
    if profile.required_independent_observations != required_independent_observations
        || profile.profile_commitment != expected_finality_profile_commitment
        || !lifecycle_profile.strict_commitment_matches()
        || lifecycle_profile.profile_commitment != expected_lifecycle_profile_commitment
    {
        return None;
    }

    if !verify_observation_set_provenance(
        set,
        effect,
        route,
        profile,
        evidence,
        current_frontier_root,
        live_generation_id,
    ) || !verify_observation_set_assessment_provenance(
        assessment, effect, route, profile, set, evidence,
        current_frontier_root, live_generation_id,
    ) {
        return None;
    }
    compose_finality_eligibility_from_authoritative_d6o(
        set, assessment, evidence, eligibility_receipts,
        lifecycle_profile, d6o_ledger, current_frontier_root,
        required_independent_observations,
    ).into()
}

/// Internal authoritative helper. External consumers must enter through
/// the D6N+D6O boundary so the D6N qualification profile, observation set,
/// and assessment are reconstructed before this helper can contribute evidence.
fn compose_finality_eligibility_from_authoritative_d6o(
    set: &ExternalObservationSetV1,
    assessment: &ObservationSetAssessmentV1,
    evidence: &[ExternalObservedEvidenceV1],
    eligibility_receipts: &[EvidenceEligibilityReceiptV1],
    lifecycle_profile: &ObserverLifecycleProfileV1,
    d6o_ledger: &ObserverLifecycleLedgerV1,
    current_frontier_root: &str,
    required_independent_observations: u32,
) -> Option<FinalityEligibilityCompositionV1> {
    let mut evidence_by_id = BTreeMap::new();
    for item in evidence {
        if !item.structurally_valid() {
            continue;
        }
        let id = item.observation.observation_id.clone();
        if evidence_by_id.contains_key(&id) {
            return None;
        }
        evidence_by_id.insert(id, item);
    }

    let mut receipt_by_id = BTreeMap::new();
    for receipt in eligibility_receipts {
        if !receipt.structurally_valid() {
            continue;
        }
        let id = receipt.observation_id.clone();
        if receipt_by_id.contains_key(&id) {
            return None;
        }
        receipt_by_id.insert(id, receipt);
    }

    let mut authoritative_receipts = Vec::new();
    for receipt in receipt_by_id.values().copied() {
        let Some(assessment_item) = assessment
            .assessments
            .iter()
            .find(|item| item.observation_id == receipt.observation_id)
        else {
            continue;
        };

        if !matches!(
            assessment_item.classification,
            ObservationClassificationV1::CorroboratingIndependent
        ) {
            continue;
        }

        let Some(observation) = evidence_by_id.get(&receipt.observation_id) else {
            return None;
        };
        let Some(generation) = d6o_ledger
            .generations
            .get(&receipt.observer_generation_id)
        else {
            return None;
        };
        let Some(snapshot) = d6o_ledger
            .dependency_snapshots
            .get(&receipt.dependency_snapshot_id)
        else {
            return None;
        };

        if receipt.current_frontier_root != current_frontier_root
            || receipt.classification != assessment_item.classification
        {
            return None;
        }

        if d6o_ledger
            .eligibility_receipts
            .get(&receipt.eligibility_id)
            != Some(receipt)
            || receipt.qualification_profile_id != lifecycle_profile.profile_id
            || !verify_eligibility_receipt_provenance(
                receipt,
                observation,
                generation,
                snapshot,
                lifecycle_profile,
                d6o_ledger,
            )
        {
            return None;
        }

        authoritative_receipts.push(receipt.clone());
    }

    let composition = compose_finality_eligibility(
        set,
        assessment,
        evidence_by_id.values().map(|item| (*item).clone()).collect::<Vec<_>>().as_slice(),
        &authoritative_receipts,
        lifecycle_profile.profile_id.as_str(),
        current_frontier_root,
        required_independent_observations,
    );

    for witness in &composition.witnesses {
        let Some(assessment_item) = assessment
            .assessments
            .iter()
            .find(|item| item.observation_id == witness.observation_id)
        else {
            return None;
        };
        let Some(observation) = evidence_by_id.get(&witness.observation_id) else {
            return None;
        };
        let receipt = authoritative_receipts
            .iter()
            .find(|receipt| receipt.observation_id == witness.observation_id);

        if !verify_witness_join_binding(
            witness,
            assessment_item,
            observation,
            receipt,
            set,
            lifecycle_profile.profile_id.as_str(),
            current_frontier_root,
        ) {
            return None;
        }
    }

    Some(composition)
}

pub fn verify_current_receipt_provenance(
    receipt: &CurrentFinalityEligibilityReceiptV1,
    set: &ExternalObservationSetV1,
    assessment: &ObservationSetAssessmentV1,
    evidence: &[ExternalObservedEvidenceV1],
    eligibility_receipts: &[EvidenceEligibilityReceiptV1],
    lifecycle_profile_id: &str,
    current_frontier_root: &str,
    required_independent_observations: u32,
) -> bool {
    let composition = compose_finality_eligibility(
        set,
        assessment,
        evidence,
        eligibility_receipts,
        lifecycle_profile_id,
        current_frontier_root,
        required_independent_observations,
    );
    current_receipt_matches_composition(receipt, &composition)
}

pub fn current_finality_receipt_is_non_authorizing(
    receipt: &CurrentFinalityEligibilityReceiptV1,
) -> bool {
    let _ = receipt;
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn committed_receipt() -> CurrentFinalityEligibilityReceiptV1 {
        let mut receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "environment-1".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-commitment-1".into(),
            d6n_assessment_commitment: "assessment-1".into(),
            composition_commitment: "composition-1".into(),
            witness_eligibility_ids: ["witness-1".into()].into_iter().collect(),
            observer_generation_ids: ["generation-1".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "lifecycle-1".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(),
            receipt_commitment: String::new(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        receipt.receipt_commitment = receipt.recomputed_commitment();
        receipt
    }

    fn matching_composition() -> FinalityEligibilityCompositionV1 {
        let mut composition = FinalityEligibilityCompositionV1 {
            composition_id: "composition:set-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "environment-1".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-commitment-1".into(),
            d6n_assessment_commitment: "assessment-1".into(),
            lifecycle_profile_id: "lifecycle-1".into(),
            current_frontier_root: "frontier-1".into(),
            eligible_independent_count: 1,
            required_independent_observations: 1,
            preserved_contradictory_count: 0,
            witnesses: vec![FinalityWitnessEligibilityV1 {
                observation_id: "observation-1".into(),
                observer_id: "observer-1".into(),
                observer_generation_id: Some("generation-1".into()),
                d6n_observation_set_id: "set-1".into(),
                d6n_observation_set_commitment: "set-commitment-1".into(),
                d6n_assessment_item_commitment: "assessment-item-1".into(),
                d6n_classification: ObservationClassificationV1::CorroboratingIndependent,
                d6o_eligibility_id: Some("witness-1".into()),
                d6o_disposition: Some(EvidenceEligibilityDispositionV1::EligibleCurrent),
                d6o_dependency_snapshot_id: Some("dependency-1".into()),
                observation_frontier_root: "frontier-1".into(),
                current_frontier_root: "frontier-1".into(),
                lifecycle_profile_id: "lifecycle-1".into(),
                witness_commitment: String::new(),
                claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
            }],
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: Some("transition-1".into()),
            composition_commitment: "composition-commitment-1".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        for witness in &mut composition.witnesses {
            witness.witness_commitment = witness.recomputed_commitment();
        }
        composition.composition_commitment = composition.recomputed_commitment();
        composition
    }

    #[test]
    fn duplicate_evidence_identity_is_rejected_without_last_write_wins() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let mut substituted = e.clone();
        substituted.observation.observed_frontier_root = "frontier-substituted".into();

        let result = compose_finality_eligibility_from_authoritative_d6o(
            &s,
            &a,
            &[e, substituted],
            &[r],
            &ObserverLifecycleProfileV1 {
                profile_id: "life-profile-1".into(),
                semantic_environment_root: "env-1".into(),
                observation_profile_id: "obs-profile-1".into(),
                allowed_roles: BTreeSet::new(),
                current_frontier_required: true,
                historical_evidence_allowed: true,
                profile_commitment: "life-profile-commitment".into(),
                claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
            },
            &ObserverLifecycleLedgerV1::default(),
            "frontier-1",
            1,
        );

        assert!(result.is_none(), "conflicting same-ID evidence must not be resolved by arrival order");
    }

    #[test]
    fn duplicate_identical_evidence_identity_is_rejected() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let result = compose_finality_eligibility_from_authoritative_d6o(
            &s,
            &a,
            &[e.clone(), e],
            &[r],
            &ObserverLifecycleProfileV1 {
                profile_id: "life-profile-1".into(),
                semantic_environment_root: "env-1".into(),
                observation_profile_id: "obs-profile-1".into(),
                allowed_roles: BTreeSet::new(),
                current_frontier_required: true,
                historical_evidence_allowed: true,
                profile_commitment: "life-profile-commitment".into(),
                claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
            },
            &ObserverLifecycleLedgerV1::default(),
            "frontier-1",
            1,
        );

        assert!(result.is_none(), "identical duplicate evidence identities must not be collapsed");
    }

    #[test]
    fn duplicate_identical_eligibility_receipt_identity_is_rejected() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let result = compose_finality_eligibility_from_authoritative_d6o(
            &s,
            &a,
            &[],
            &[r.clone(), r],
            &ObserverLifecycleProfileV1 {
                profile_id: "life-profile-1".into(),
                semantic_environment_root: "env-1".into(),
                observation_profile_id: "obs-profile-1".into(),
                allowed_roles: BTreeSet::new(),
                current_frontier_required: true,
                historical_evidence_allowed: true,
                profile_commitment: "life-profile-commitment".into(),
                claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
            },
            &ObserverLifecycleLedgerV1::default(),
            "frontier-1",
            1,
        );

        assert!(result.is_none(), "identical duplicate eligibility identities must not be collapsed");
    }

    #[test]
    fn generic_composition_rejects_identical_duplicate_evidence_identity() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let result = compose_finality_eligibility(
            &s,
            &a,
            &[e.clone(), e],
            &[r],
            "life-profile-1",
            "frontier-1",
            1,
        );

        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::BlockedBinding
        );
    }

    #[test]
    fn witness_join_binding_rejects_cross_object_assessment_substitution() {
        let composition = matching_composition();
        let witness = &composition.witnesses[0];
        let g = generation("observer-A");
        let evidence = observation("observation-1", &g, ExternalObservedStateV1::Applied);
        let set = set(&["observation-1"]);
        let mut assessment = d6n_assessment(
            &set,
            &[(
                "observation-1".into(),
                "observer-A".into(),
                witness.d6n_classification,
            )],
        )
        .assessments
        .remove(0);
        let (_, receipt) = ledger_and_receipt(&g, &evidence);

        assert!(verify_witness_join_binding(
            witness,
            &assessment,
            &evidence,
            Some(&receipt),
            &set,
            "lifecycle-1",
            "frontier-1",
        ));

        assessment.evidence_root = "evidence-substituted".into();
        assessment.assessment_commitment = "assessment-item-substituted".into();

        assert!(!verify_witness_join_binding(
            witness,
            &assessment,
            &evidence,
            Some(&receipt),
            &set,
            "lifecycle-1",
            "frontier-1",
        ));

        let mut forged_receipt = receipt.clone();
        forged_receipt.dependency_snapshot_id = "snapshot-substituted".into();
        assessment.assessment_commitment = witness.d6n_assessment_item_commitment.clone();
        assert!(!verify_witness_join_binding(
            witness,
            &assessment,
            &evidence,
            Some(&forged_receipt),
            &set,
            "lifecycle-1",
            "frontier-1",
        ));
    }

    #[test]
    fn ledger_witness_registry_rejects_cross_frontier_replacement() {
        let composition = matching_composition();
        let mut witness = composition.witnesses[0].clone();
        let mut replay = witness.clone();
        replay.current_frontier_root = "frontier-replayed".into();
        replay.observation_frontier_root = "frontier-replayed".into();
        replay.witness_commitment = format!(
            "witness:{}:{}:{}:{}",
            replay.observation_id,
            replay.d6n_observation_set_commitment,
            replay.d6o_eligibility_id.as_deref().unwrap_or("missing"),
            replay.current_frontier_root,
        );

        witness.witness_commitment = witness.recomputed_commitment();
        let mut ledger = FinalityEligibilityLedgerV1::default();
        assert_eq!(
            ledger.record_witness(witness.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_witness(replay),
            FinalityCompositionRecordDispositionV1::Conflict
        );
        assert_eq!(
            ledger
                .witness_at_frontier(&witness.observation_id, &witness.current_frontier_root)
                .map(|stored| &stored.witness_commitment),
            Some(&witness.witness_commitment)
        );
        assert!(ledger
            .witness_at_frontier(&witness.observation_id, "frontier-replayed")
            .is_none());
    }

    #[test]
    fn ledger_witness_selection_requires_expected_frontier() {
        let composition = matching_composition();
        let witness = composition.witnesses[0].clone();
        let mut ledger = FinalityEligibilityLedgerV1::default();

        assert_eq!(
            ledger.record_witness(witness.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger
                .witness_at_frontier(&witness.observation_id, &witness.current_frontier_root)
                .map(|stored| &stored.witness_commitment),
            Some(&witness.witness_commitment)
        );
        assert!(ledger
            .witness_at_frontier(&witness.observation_id, "frontier-replayed")
            .is_none());
        assert!(ledger
            .witness_at_frontier(&witness.observation_id, "")
            .is_none());
    }

    #[test]
    fn witness_commitment_binds_all_semantic_fields() {
        let mut witness = matching_composition().witnesses[0].clone();
        assert!(witness.commitment_matches());

        witness.observer_id = "observer-substituted".into();
        assert!(!witness.commitment_matches());

        witness = matching_composition().witnesses[0].clone();
        witness.d6o_dependency_snapshot_id = Some("snapshot-substituted".into());
        assert!(!witness.commitment_matches());
    }

    #[test]
    fn witness_commitment_binds_current_frontier() {
        let composition = matching_composition();
        let witness = &composition.witnesses[0];
        assert!(witness.commitment_matches());

        let mut replay = witness.clone();
        replay.current_frontier_root = "frontier-replayed".into();
        assert!(!replay.commitment_matches());
        assert!({
            let mut candidate = composition.clone();
            candidate.witnesses[0] = replay;
            candidate.composition_commitment = candidate.recomputed_commitment();
            !candidate.semantically_valid()
        });
    }

    #[test]
    fn receipt_provenance_requires_authoritative_composition_binding() {
        let composition = matching_composition();
        let mut receipt = committed_receipt();
        receipt.composition_commitment = composition.composition_commitment.clone();
        receipt.receipt_commitment = receipt.recomputed_commitment();
        assert!(verify_current_receipt_provenance_from_composition(&receipt, &composition));

        receipt.eligible_independent_count += 1;
        receipt.receipt_commitment = receipt.recomputed_commitment();
        assert!(!verify_current_receipt_provenance_from_composition(&receipt, &composition));
    }

    #[test]
    fn ledger_rejects_noncanonical_composition_id() {
        let mut ledger = FinalityEligibilityLedgerV1::default();
        let mut composition = matching_composition();
        composition.composition_id = "composition:attacker-shadow".into();
        composition.composition_commitment = composition.recomputed_commitment();

        assert!(!composition.semantically_valid());
        assert_eq!(
            ledger.record_composition(composition),
            FinalityCompositionRecordDispositionV1::InsufficientEvidence
        );
    }

    #[test]
    fn ledger_rejects_self_consistent_but_semantically_incoherent_composition() {
        let mut ledger = FinalityEligibilityLedgerV1::default();
        let mut composition = matching_composition();
        composition.disposition = FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses;
        composition.composition_commitment = composition.recomputed_commitment();
        assert!(!composition.semantically_valid());
        assert_eq!(
            ledger.record_composition(composition),
            FinalityCompositionRecordDispositionV1::InsufficientEvidence
        );
    }

    #[test]
    fn ledger_requires_receipt_to_project_a_recorded_composition() {
        let mut ledger = FinalityEligibilityLedgerV1::default();
        let composition = matching_composition();
        assert_eq!(
            ledger.record_composition(composition.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );

        let mut receipt = committed_receipt();
        receipt.composition_commitment = composition.composition_commitment.clone();
        receipt.receipt_commitment = receipt.recomputed_commitment();
        assert_eq!(
            ledger.record_receipt(receipt),
            FinalityCompositionRecordDispositionV1::Recorded
        );
    }

    #[test]
    fn composition_commitment_binds_semantic_provenance() {
        let composition = matching_composition();
        assert!(composition.commitment_matches());

        let mut mutated = composition.clone();
        mutated.effect_lineage_id = "lineage-attacker".into();
        assert!(!mutated.commitment_matches());

        let mut mutated = composition.clone();
        mutated.witnesses[0].d6o_eligibility_id = Some("witness-attacker".into());
        assert!(!mutated.commitment_matches());

        let mut mutated = composition;
        mutated.disposition = FinalityEligibilityDispositionV1::Contested;
        assert!(!mutated.commitment_matches());
    }

    #[test]
    fn current_receipt_provenance_is_reconstructed_from_composition() {
        let receipt = committed_receipt();
        let composition = matching_composition();
        assert!(current_receipt_matches_composition(&receipt, &composition));

        let mut substituted = composition.clone();
        substituted.provider_operation_id = "operation-attacker".into();
        assert!(!current_receipt_matches_composition(&receipt, &substituted));

        let mut substituted = composition.clone();
        substituted.witnesses[0].d6o_eligibility_id = Some("witness-attacker".into());
        assert!(!current_receipt_matches_composition(&receipt, &substituted));

        let mut substituted = composition;
        substituted.current_frontier_root = "frontier-attacker".into();
        assert!(!current_receipt_matches_composition(&receipt, &substituted));
    }

    #[test]
    fn current_receipt_commitment_binds_every_semantic_field() {
        let receipt = committed_receipt();
        assert!(receipt.commitment_matches());

        let mut mutated = receipt.clone();
        mutated.provider_operation_id = "operation-attacker".into();
        assert!(!mutated.commitment_matches());

        let mut mutated = receipt.clone();
        mutated.current_frontier_root = "frontier-attacker".into();
        assert!(!mutated.commitment_matches());

        let mut mutated = receipt.clone();
        mutated.disposition = FinalityEligibilityDispositionV1::Contested;
        assert!(!mutated.commitment_matches());
    }

    #[test]
    fn d6p_commitment_domains_end_with_a_nul_separator() {
        assert_eq!(D6P_RECEIPT_COMMITMENT_DOMAIN.last(), Some(&0));
        assert_eq!(D6P_COMPOSITION_COMMITMENT_DOMAIN.last(), Some(&0));
        assert_ne!(D6P_RECEIPT_COMMITMENT_DOMAIN, D6P_COMPOSITION_COMMITMENT_DOMAIN);
    }

    #[test]
    fn current_receipt_commitment_is_domain_separated_and_canonical_length() {
        let receipt = committed_receipt();
        assert_eq!(receipt.receipt_commitment.len(), 64);
        assert!(receipt.receipt_commitment.bytes().all(|b| {
            b.is_ascii_digit() || (b'a'..=b'f').contains(&b)
        }));

        let mut mutated = receipt.clone();
        mutated.receipt_commitment = String::from("receipt-1");
        assert!(!mutated.commitment_matches());
    }
    use crate::contestable_finality::{
        ExternalObserverProfileV1, ExternalObserverRoleV1, ObservationAssessmentV1, ObservationSetDispositionV1,
    };
    use crate::effect_finality::{
        ExternalEffectObservationV1, ExternalObservedStateV1, ExternalObservationSourceV1,
    };
    use crate::observer_lifecycle::{
        EvidenceDependencySnapshotV1, ObserverGenerationV1, ObserverLifecycleLedgerV1,
        ObserverLifecycleProfileV1, ObserverStatusV1,
    };

    fn lifecycle_profile() -> ObserverLifecycleProfileV1 {
        let allowed_roles = [ExternalObserverRoleV1::IndependentObserver]
            .into_iter()
            .collect::<BTreeSet<_>>();
        let mut profile = ObserverLifecycleProfileV1 {
            profile_id: "life-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_profile_id: "obs-profile-1".into(),
            allowed_roles,
            current_frontier_required: true,
            historical_evidence_allowed: true,
            profile_commitment: String::new(),
            claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        profile.profile_commitment = profile.recomputed_commitment();
        profile
    }

    fn generation(id: &str) -> ObserverGenerationV1 {
        ObserverGenerationV1 {
            generation_id: id.into(),
            observer_id: id.into(),
            generation_sequence: 1,
            predecessor_generation_id: None,
            role: ExternalObserverRoleV1::IndependentObserver,
            observation_method: "independent-state-read".into(),
            provider_relationship: "external".into(),
            evidence_root: format!("evidence-{id}"),
            custody_root: format!("custody-{id}"),
            upstream_observer_ids: BTreeSet::new(),
            upstream_evidence_roots: BTreeSet::new(),
            semantic_environment_root: "env-1".into(),
            observation_profile_id: "obs-profile-1".into(),
            created_frontier_root: "frontier-1".into(),
            created_frontier_sequence: 1,
            initial_status: ObserverStatusV1::Active,
            generation_commitment: format!("generation:{id}"),
            claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    fn observation(
        id: &str,
        observer: &ObserverGenerationV1,
        state: ExternalObservedStateV1,
    ) -> ExternalObservedEvidenceV1 {
        let mut observation = ExternalEffectObservationV1 {
            observation_id: id.into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "effect-generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            provider_outcome_id: format!("outcome-{id}"),
            request_commitment: "request-1".into(),
            idempotency_key: "idem-1".into(),
            semantic_environment_root: "env-1".into(),
            observed_frontier_root: "frontier-1".into(),
            observed_state: state,
            source: ExternalObservationSourceV1::IndependentObserver,
            evidence_root: observer.evidence_root.clone(),
            observation_commitment: String::new(),
            claim_ceiling: crate::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
        };
        observation.observation_commitment = observation.recomputed_commitment();
        ExternalObservedEvidenceV1 {
            observation,
            observer_id: observer.observer_id.clone(),
            observer: ExternalObserverProfileV1 {
                observer_id: observer.observer_id.clone(),
                role: observer.role,
                observation_method: observer.observation_method.clone(),
                provider_relationship: observer.provider_relationship.clone(),
                evidence_root: observer.evidence_root.clone(),
                custody_root: observer.custody_root.clone(),
                upstream_observer_ids: observer.upstream_observer_ids.clone(),
                upstream_evidence_roots: observer.upstream_evidence_roots.clone(),
                semantic_environment_root: observer.semantic_environment_root.clone(),
                observation_profile_id: observer.observation_profile_id.clone(),
                independence: crate::contestable_finality::ObservationIndependenceV1::DeclaredIndependent,
                independence_commitment: format!("independence:{}", observer.observer_id),
                claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
            },
        }
    }

    fn set(ids: &[&str]) -> ExternalObservationSetV1 {
        let mut set = ExternalObservationSetV1 {
            set_id: "set-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "effect-generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_frontier_root: "frontier-1".into(),
            qualification_profile_id: "finality-profile-1".into(),
            observation_ids: ids.iter().map(|id| (*id).to_owned()).collect(),
            target_state: crate::effect_finality::ExternalFinalityStateV1::Applied,
            set_commitment: "set-commitment".into(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        };
        set.set_commitment = set.recomputed_commitment();
        set
    }

    fn d6n_assessment(
        set: &ExternalObservationSetV1,
        classifications: &[(String, String, ObservationClassificationV1)],
    ) -> ObservationSetAssessmentV1 {
        let assessments = classifications
            .iter()
            .map(|(observation_id, observer_id, classification)| {
                let observer = generation(observer_id);
                let evidence = observation(
                    observation_id,
                    &observer,
                    ExternalObservedStateV1::Applied,
                );
                let mut assessment = ObservationAssessmentV1 {
                    observation_id: observation_id.clone(),
                    observer_id: observer_id.clone(),
                    independence: crate::contestable_finality::ObservationIndependenceV1::DeclaredIndependent,
                    classification: *classification,
                    evidence_root: format!("evidence-{observer_id}"),
                    custody_root: format!("custody-{observer_id}"),
                    observation_commitment: evidence.observation.observation_commitment.clone(),
                    assessment_commitment: String::new(),
                    claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
                };
                assessment.assessment_commitment = assessment.recomputed_commitment();
                assessment
            })
            .collect();

        ObservationSetAssessmentV1 {
            set_id: set.set_id.clone(),
            disposition: if classifications.iter().any(|(_, _, classification)| {
                matches!(
                    classification,
                    ObservationClassificationV1::ContradictoryIndependent
                        | ObservationClassificationV1::ContradictoryDependent
                )
            }) {
                ObservationSetDispositionV1::Contested
            } else {
                ObservationSetDispositionV1::QualifiedEvidence
            },
            independent_count: classifications
                .iter()
                .filter(|(_, _, classification)| {
                    matches!(classification, ObservationClassificationV1::CorroboratingIndependent)
                })
                .count() as u32,
            contradictory_independent_count: classifications
                .iter()
                .filter(|(_, _, classification)| {
                    matches!(classification, ObservationClassificationV1::ContradictoryIndependent)
                })
                .count() as u32,
            dependent_count: classifications
                .iter()
                .filter(|(_, _, classification)| {
                    matches!(
                        classification,
                        ObservationClassificationV1::CorroboratingDependent
                            | ObservationClassificationV1::ContradictoryDependent
                    )
                })
                .count() as u32,
            assessments,
            assessment_commitment: format!("assessment:{}", set.set_commitment),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn snapshot(generation: &ObserverGenerationV1) -> EvidenceDependencySnapshotV1 {
        EvidenceDependencySnapshotV1 {
            snapshot_id: format!("snapshot-{}", generation.generation_id),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            evidence_root: generation.evidence_root.clone(),
            custody_root: generation.custody_root.clone(),
            upstream_observer_ids: BTreeSet::new(),
            upstream_evidence_roots: BTreeSet::new(),
            independence: crate::contestable_finality::ObservationIndependenceV1::DeclaredIndependent,
            effective_frontier_root: "frontier-1".into(),
            effective_frontier_sequence: 1,
            predecessor_snapshot_id: None,
            snapshot_commitment: format!("snapshot:{}", generation.generation_id),
            claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    fn ledger_and_receipt(
        generation: &ObserverGenerationV1,
        observation: &ExternalObservedEvidenceV1,
    ) -> (ObserverLifecycleLedgerV1, EvidenceEligibilityReceiptV1) {
        let snapshot = snapshot(generation);
        let mut ledger = ObserverLifecycleLedgerV1::default();
        ledger.record_generation(generation.clone());
        ledger.record_dependency_snapshot(snapshot.clone());

        let receipt = EvidenceEligibilityReceiptV1 {
            eligibility_id: format!("eligibility-{}", observation.observation.observation_id),
            observation_id: observation.observation.observation_id.clone(),
            observer_id: observation.observer.observer_id.clone(),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            dependency_snapshot_id: snapshot.snapshot_id.clone(),
            observation_frontier_root: "frontier-1".into(),
            observation_frontier_sequence: 1,
            current_frontier_root: "frontier-1".into(),
            current_frontier_sequence: 1,
            current_generation_id: Some(generation.generation_id.clone()),
            qualification_profile_id: lifecycle_profile().profile_id,
            provenance: ObserverEvidenceProvenanceV1::Live,
            classification: ObservationClassificationV1::CorroboratingIndependent,
            disposition: EvidenceEligibilityDispositionV1::EligibleCurrent,
            lifecycle_transition_ids: BTreeSet::new(),
            eligibility_commitment: String::new(),
            claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        let mut receipt = receipt;
        receipt.eligibility_commitment = receipt.recomputed_commitment();
        assert_eq!(
            ledger.record_eligibility_receipt(receipt.clone()),
            crate::observer_lifecycle::LifecycleRecordDispositionV1::Recorded
        );
        (ledger, receipt)
    }

    #[test]
    fn authoritative_d6n_d6o_rejects_threshold_parameter_substitution() {
        let generation = generation("observer-A");
        let evidence = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let (ledger, receipt) = ledger_and_receipt(&generation, &evidence);
        let set = set(&["obs-1"]);
        let assessment = d6n_assessment(
            &set,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let mut finality_profile = FinalityQualificationProfileV1 {
            profile_id: "finality-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            allowed_observation_sources: [ExternalObservationSourceV1::IndependentObserver]
                .into_iter()
                .collect(),
            required_independent_observations: 2,
            current_frontier_required: true,
            provider_reports_may_satisfy_independence: false,
            allow_explicit_conflict_resolution: false,
            profile_commitment: String::new(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        };
        finality_profile.profile_commitment = finality_profile.recomputed_commitment();
        assert!(finality_profile.structurally_valid());

        let rejected = compose_finality_eligibility_from_authoritative_d6n_d6o(
            &SemanticEffectV1 {
                effect_id: "effect-1".into(),
                lineage_id: "lineage-1".into(),
                generation_id: "effect-generation-1".into(),
                request_commitment: "request-1".into(),
                semantic_environment_root: "env-1".into(),
                effect_class: "class".into(),
                resource_id: "resource".into(),
                tenant_id: "tenant".into(),
                amount: "1".into(),
                unit: "unit".into(),
                authority_claim_id: "authority".into(),
                consent_claim_id: "consent".into(),
                idempotency_key: "idem".into(),
            },
            &ProviderRouteV1 {
                route_id: "route-1".into(),
                effect_id: "effect-1".into(),
                substitution_profile_id: "profile".into(),
                provider_id: "provider-1".into(),
                provider_profile_root: "provider-profile-1".into(),
                provider_operation_id: "operation-1".into(),
                route_generation: 1,
                lifecycle_generation_id: "effect-generation-1".into(),
                request_commitment: "request-1".into(),
                semantic_environment_root: "env-1".into(),
                effect_class: "class".into(),
                resource_id: "resource".into(),
                tenant_id: "tenant".into(),
                amount: "1".into(),
                unit: "unit".into(),
                authority_claim_id: "authority".into(),
                consent_claim_id: "consent".into(),
                idempotency_key: "idem".into(),
                route_frontier_root: "frontier-1".into(),
            },
            &finality_profile,
            &finality_profile.profile_commitment,
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&receipt),
            &lifecycle_profile(),
            &lifecycle_profile().profile_commitment,
            &ledger,
            "frontier-1",
            "effect-generation-1",
            1,
        );

        assert!(
            rejected.is_none(),
            "authoritative D6P reconstruction must not accept a caller-supplied threshold lower than the qualification profile"
        );
    }

    #[test]
    fn authoritative_d6o_boundary_rejects_unregistered_self_consistent_receipt() {
        let generation = generation("observer-A");
        let evidence = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let (mut ledger, receipt) = ledger_and_receipt(&generation, &evidence);
        ledger.eligibility_receipts.clear();

        let set = set(&["obs-1"]);
        let assessment = d6n_assessment(
            &set,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );

        assert!(
            receipt.commitment_matches(),
            "the adversarial D6O receipt must remain internally self-consistent"
        );
        let rejected = compose_finality_eligibility_from_authoritative_d6o(
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&receipt),
            &lifecycle_profile(),
            &ledger,
            "frontier-1",
            1,
        );
        assert!(
            rejected.is_none(),
            "authoritative D6O admission must require the exact receipt registered by the lifecycle ledger"
        );
    }

    #[test]
    fn authoritative_d6o_boundary_rejects_forged_but_self_consistent_receipt() {
        let generation = generation("observer-A");
        let evidence = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let (ledger, receipt) = ledger_and_receipt(&generation, &evidence);
        let set = set(&["obs-1"]);
        let assessment = d6n_assessment(
            &set,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let valid = compose_finality_eligibility_from_authoritative_d6o(
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&receipt),
            &lifecycle_profile(),
            &ledger,
            "frontier-1",
            1,
        );
        let valid = valid.expect("authoritative D6N/D6O reconstruction");
        assert_eq!(
            valid.disposition,
            FinalityEligibilityDispositionV1::EligibleCurrent
        );

        let mut forged = receipt;
        forged.current_frontier_root = "frontier-attacker".into();
        forged.eligibility_commitment = forged.recomputed_commitment();
        assert!(forged.commitment_matches());

        let rejected = compose_finality_eligibility_from_authoritative_d6o(
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&forged),
            &lifecycle_profile(),
            &ledger,
            "frontier-1",
            1,
        );
        assert!(
            rejected.is_none(),
            "authoritative D6O reconstruction must fail closed"
        );
    }

    #[test]
    fn witness_join_rejects_self_consistent_d6n_to_d6m_substitution() {
        let composition = matching_composition();
        let witness = &composition.witnesses[0];
        let g = generation("observer-A");
        let evidence = observation("observation-1", &g, ExternalObservedStateV1::Applied);
        let set = set(&["observation-1"]);
        let (_, receipt) = ledger_and_receipt(&g, &evidence);
        let mut assessment = d6n_assessment(
            &set,
            &[(
                "observation-1".into(),
                "observer-A".into(),
                witness.d6n_classification,
            )],
        )
        .assessments
        .remove(0);

        let mut substituted = evidence.clone();
        substituted.observation.request_commitment = "forged-request".into();
        substituted.observation.observation_commitment =
            substituted.observation.recomputed_commitment();

        assessment.observation_commitment = substituted.observation.observation_commitment;
        assessment.assessment_commitment = assessment.recomputed_commitment();

        assert!(assessment.commitment_matches());
        assert!(!verify_witness_join_binding(
            witness,
            &assessment,
            &evidence,
            Some(&receipt),
            &set,
            "lifecycle-1",
            "frontier-1",
        ));
    }

    #[test]
    fn eligible_independent_witness_counts() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&g1, &e1);
        let (_, r2) = ledger_and_receipt(&g2, &e2);
        let result = compose_finality_eligibility(
            &s, &a, &[e1, e2], &[r1, r2], "life-profile-1", "frontier-1", 2
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::EligibleCurrent);
        assert_eq!(result.eligible_independent_count, 2);
    }

    fn receipt_for_composition(
        composition: &FinalityEligibilityCompositionV1,
    ) -> CurrentFinalityEligibilityReceiptV1 {
        let mut receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-authoritative-1".into(),
            effect_id: composition.effect_id.clone(),
            effect_lineage_id: composition.effect_lineage_id.clone(),
            lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
            route_id: composition.route_id.clone(),
            provider_id: composition.provider_id.clone(),
            provider_operation_id: composition.provider_operation_id.clone(),
            provider_profile_root: composition.provider_profile_root.clone(),
            semantic_environment_root: composition.semantic_environment_root.clone(),
            observation_set_id: composition.observation_set_id.clone(),
            observation_set_commitment: composition.observation_set_commitment.clone(),
            d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
            composition_commitment: composition.composition_commitment.clone(),
            witness_eligibility_ids: composition
                .witnesses
                .iter()
                .filter_map(|witness| witness.d6o_eligibility_id.clone())
                .collect(),
            observer_generation_ids: composition
                .witnesses
                .iter()
                .filter_map(|witness| witness.observer_generation_id.clone())
                .collect(),
            current_frontier_root: composition.current_frontier_root.clone(),
            lifecycle_profile_id: composition.lifecycle_profile_id.clone(),
            eligible_independent_count: composition.eligible_independent_count,
            preserved_contradictory_count: composition.preserved_contradictory_count,
            disposition: composition.disposition,
            qualification_transition_id: composition
                .qualification_transition_id
                .clone()
                .unwrap_or_default(),
            receipt_commitment: String::new(),
            claim_ceiling: composition.claim_ceiling.clone(),
        };
        receipt.receipt_commitment = receipt.recomputed_commitment();
        receipt
    }

    #[test]
    fn d6p_rejects_assessment_with_duplicate_or_missing_observation_ids() {
        let set = set(&["obs-1", "obs-2"]);
        let mut assessment = d6n_assessment(
            &set,
            &[
                (
                    "obs-1".into(),
                    "observer-A".into(),
                    ObservationClassificationV1::CorroboratingIndependent,
                ),
                (
                    "obs-2".into(),
                    "observer-B".into(),
                    ObservationClassificationV1::CorroboratingIndependent,
                ),
            ],
        );

        assessment.assessments[1] = assessment.assessments[0].clone();
        assert_eq!(assessment.assessments.len(), set.observation_ids.len());

        let generation = generation("observer-A");
        let evidence = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let (_, eligibility) = ledger_and_receipt(&generation, &evidence);

        let composition = compose_finality_eligibility(
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        );

        assert_eq!(
            composition.disposition,
            FinalityEligibilityDispositionV1::BlockedBinding
        );
        assert_eq!(composition.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_rejects_independent_assessment_without_independent_declaration() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let mut a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );

        a.assessments[0].independence =
            crate::contestable_finality::ObservationIndependenceV1::DeclaredDependent;

        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s,
            &a,
            &[e],
            &[r],
            "life-profile-1",
            "frontier-1",
            1,
        );

        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::BlockedBinding
        );
        assert_eq!(result.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_rejects_self_consistent_assessment_count_substitution() {
        let set = set(&["obs-1"]);
        let mut assessment = d6n_assessment(
            &set,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        assessment.independent_count = 0;

        let generation = generation("observer-A");
        let evidence = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let (_, eligibility) = ledger_and_receipt(&generation, &evidence);

        let composition = compose_finality_eligibility(
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        );

        assert_eq!(
            composition.disposition,
            FinalityEligibilityDispositionV1::BlockedBinding
        );
        assert_eq!(composition.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_receipt_rejects_composition_identity_substitution() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);
        let composition = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        let receipt = receipt_for_composition(&composition);

        assert!(current_receipt_matches_composition(&receipt, &composition));

        let mut substituted = composition.clone();
        substituted.witnesses[0].observer_id = "observer-substituted".into();
        substituted.composition_commitment = substituted.recomputed_commitment();

        assert!(substituted.commitment_matches());
        assert!(substituted.semantically_valid());
        assert!(!current_receipt_matches_composition(&receipt, &substituted));
    }

    #[test]
    fn d6p_rejects_self_consistent_assessment_disposition_substitution() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let mut a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        assert_eq!(a.disposition, ObservationSetDispositionV1::QualifiedEvidence);

        a.disposition = ObservationSetDispositionV1::InsufficientEvidence;
        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::BlockedBinding
        );
        assert_eq!(result.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_rejects_assessment_root_substitution_against_observer_evidence() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let mut a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        a.assessments[0].evidence_root = "substituted-evidence-root".into();
        a.assessments[0].assessment_commitment = format!(
            "{}:{}:{}",
            a.assessments[0].observation_id,
            a.assessments[0].evidence_root,
            a.assessments[0].custody_root
        );

        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::BlockedBinding
        );
        assert_eq!(result.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_authoritative_provenance_rejects_source_substitution() {
        let generation = generation("observer-A");
        let evidence = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let set = set(&["obs-1"]);
        let assessment = d6n_assessment(
            &set,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, eligibility) = ledger_and_receipt(&generation, &evidence);
        let composition = compose_finality_eligibility(
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        );
        let receipt = receipt_for_composition(&composition);

        assert!(verify_current_receipt_provenance(
            &receipt,
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        ));

        let mut substituted_set = set.clone();
        substituted_set.provider_operation_id = "operation-attacker".into();
        assert!(!verify_current_receipt_provenance(
            &receipt,
            &substituted_set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        ));

        let substituted_assessment = d6n_assessment(
            &set,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::ContradictoryIndependent,
            )],
        );
        assert!(!verify_current_receipt_provenance(
            &receipt,
            &set,
            &substituted_assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        ));

        let mut substituted_evidence = evidence.clone();
        substituted_evidence.observer_id = "observer-attacker".into();
        assert!(!verify_current_receipt_provenance(
            &receipt,
            &set,
            &assessment,
            std::slice::from_ref(&substituted_evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        ));

        let mut substituted_eligibility = eligibility.clone();
        substituted_eligibility.dependency_snapshot_id = "snapshot-attacker".into();
        assert!(!verify_current_receipt_provenance(
            &receipt,
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&substituted_eligibility),
            "life-profile-1",
            "frontier-1",
            1,
        ));

        assert!(!verify_current_receipt_provenance(
            &receipt,
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "lifecycle-attacker",
            "frontier-1",
            1,
        ));

        assert!(!verify_current_receipt_provenance(
            &receipt,
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-attacker",
            1,
        ));

        assert!(!verify_current_receipt_provenance(
            &receipt,
            &set,
            &assessment,
            std::slice::from_ref(&evidence),
            std::slice::from_ref(&eligibility),
            "life-profile-1",
            "frontier-1",
            2,
        ));
    }

    #[test]
    fn d6p_composition_is_invariant_to_evidence_and_receipt_order() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&g1, &e1);
        let (_, r2) = ledger_and_receipt(&g2, &e2);

        let forward = compose_finality_eligibility(
            &s, &a, &[e1.clone(), e2.clone()], &[r1.clone(), r2.clone()], "life-profile-1", "frontier-1", 2
        );
        let reversed = compose_finality_eligibility(
            &s, &a, &[e2, e1], &[r2, r1], "life-profile-1", "frontier-1", 2
        );

        assert_eq!(forward, reversed);
        assert!(forward.commitment_matches());
    }

    #[test]
    fn d6p_composition_ignores_irrelevant_extra_evidence() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let irrelevant = observation("obs-irrelevant", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent)],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let baseline = compose_finality_eligibility(
            &s, &a, std::slice::from_ref(&e), std::slice::from_ref(&r), "life-profile-1", "frontier-1", 1
        );
        let with_irrelevant = compose_finality_eligibility(
            &s, &a, &[e, irrelevant], &[r], "life-profile-1", "frontier-1", 1
        );

        assert_eq!(baseline, with_irrelevant);
    }

    #[test]
    fn d6p_composition_is_invariant_to_identical_duplicate_evidence() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent)],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let baseline = compose_finality_eligibility(
            &s, &a, std::slice::from_ref(&e), std::slice::from_ref(&r), "life-profile-1", "frontier-1", 1
        );
        let duplicated = compose_finality_eligibility(
            &s, &a, &[e.clone(), e], &[r.clone(), r], "life-profile-1", "frontier-1", 1
        );

        assert_eq!(baseline, duplicated);
    }

    #[test]
    fn d6p_composition_rejects_conflicting_duplicate_evidence() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let mut conflicting = e.clone();
        conflicting.observation.provider_operation_id = "operation-conflict".into();
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent)],
        );
        let (_, r) = ledger_and_receipt(&g, &e);

        let result = compose_finality_eligibility(
            &s, &a, &[e, conflicting], &[r], "life-profile-1", "frontier-1", 1
        );

        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedBinding);
        assert_eq!(result.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_composition_rejects_conflicting_duplicate_receipt() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent)],
        );
        let (_, r) = ledger_and_receipt(&g, &e);
        let mut conflicting = r.clone();
        conflicting.observer_generation_id = "observer-A-conflict".into();

        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r, conflicting], "life-profile-1", "frontier-1", 1
        );

        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedBinding);
        assert_eq!(result.composition_commitment, "blocked");
    }

    #[test]
    fn d6p_semantic_validation_rejects_empty_witness_identity_members() {
        let mut receipt = committed_receipt();
        receipt.witness_eligibility_ids = [String::new()].into_iter().collect();
        receipt.observer_generation_ids = ["generation-1".into()].into_iter().collect();
        receipt.receipt_commitment = receipt.recomputed_commitment();

        assert!(receipt.commitment_matches());
        assert!(!receipt.semantically_valid());

        let mut receipt = committed_receipt();
        receipt.witness_eligibility_ids = ["witness-1".into()].into_iter().collect();
        receipt.observer_generation_ids = [String::new()].into_iter().collect();
        receipt.receipt_commitment = receipt.recomputed_commitment();

        assert!(receipt.commitment_matches());
        assert!(!receipt.semantically_valid());
    }

    #[test]
    fn d6p_semantic_validation_rejects_empty_witness_d6o_metadata() {
        let mut composition = matching_composition();
        composition.witnesses[0].d6o_eligibility_id = Some(String::new());
        composition.composition_commitment = composition.recomputed_commitment();

        assert!(composition.commitment_matches());
        assert!(!composition.semantically_valid());

        let mut composition = matching_composition();
        composition.witnesses[0].d6o_dependency_snapshot_id = Some(String::new());
        composition.composition_commitment = composition.recomputed_commitment();

        assert!(composition.commitment_matches());
        assert!(!composition.semantically_valid());

        let mut composition = matching_composition();
        composition.witnesses[0].observer_generation_id = Some(String::new());
        composition.composition_commitment = composition.recomputed_commitment();

        assert!(composition.commitment_matches());
        assert!(!composition.semantically_valid());
    }

    #[test]
    fn d6p_semantic_validation_rejects_self_consistent_witness_metadata_mutations() {
        let composition = matching_composition();
        assert!(composition.semantically_valid());

        let mut mutated = composition.clone();
        mutated.witnesses[0].witness_commitment = "witness:attacker".into();
        mutated.composition_commitment = mutated.recomputed_commitment();
        assert!(mutated.commitment_matches());
        assert!(!mutated.semantically_valid());

        let mut mutated = composition.clone();
        mutated.witnesses[0].d6o_eligibility_id = None;
        mutated.composition_commitment = mutated.recomputed_commitment();
        assert!(mutated.commitment_matches());
        assert!(!mutated.semantically_valid());
    }

    #[test]
    fn d6p_semantic_validation_rejects_self_consistent_count_mutations() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&g1, &e1);
        let (_, r2) = ledger_and_receipt(&g2, &e2);
        let baseline = compose_finality_eligibility(
            &s, &a, &[e1, e2], &[r1.clone(), r2.clone()], "life-profile-1", "frontier-1", 1
        );

        assert!(baseline.semantically_valid());

        let mut count_mutation = baseline.clone();
        count_mutation.eligible_independent_count += 1;
        count_mutation.composition_commitment = count_mutation.recomputed_commitment();
        assert!(count_mutation.commitment_matches());
        assert!(!count_mutation.semantically_valid());

        let mut disposition_mutation = baseline;
        disposition_mutation.disposition =
            FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses;
        disposition_mutation.qualification_transition_id = None;
        disposition_mutation.composition_commitment = disposition_mutation.recomputed_commitment();
        assert!(disposition_mutation.commitment_matches());
        assert!(!disposition_mutation.semantically_valid());

        let mut duplicate_witness = compose_finality_eligibility(
            &s, &a, &[observation("obs-1", &g1, ExternalObservedStateV1::Applied),
            observation("obs-2", &g2, ExternalObservedStateV1::Applied)],
            &[r1, r2], "life-profile-1", "frontier-1", 1
        );
        duplicate_witness.witnesses[1] = duplicate_witness.witnesses[0].clone();
        duplicate_witness.composition_commitment = duplicate_witness.recomputed_commitment();
        assert!(duplicate_witness.commitment_matches());
        assert!(!duplicate_witness.semantically_valid());
    }

    #[test]
    fn d6p_composition_identity_binds_the_required_observation_threshold() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&g1, &e1);
        let (_, r2) = ledger_and_receipt(&g2, &e2);

        let threshold_one = compose_finality_eligibility(
            &s, &a, &[e1.clone(), e2.clone()], &[r1.clone(), r2.clone()],
            "life-profile-1", "frontier-1", 1
        );
        let threshold_two = compose_finality_eligibility(
            &s, &a, &[e1, e2], &[r1, r2], "life-profile-1", "frontier-1", 3
        );

        assert_ne!(threshold_one.composition_commitment, threshold_two.composition_commitment);
        assert_eq!(threshold_one.disposition, FinalityEligibilityDispositionV1::EligibleCurrent);
        assert_eq!(threshold_two.disposition, FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses);
        assert_ne!(threshold_one.required_independent_observations, threshold_two.required_independent_observations);
    }

    #[test]
    fn historical_only_does_not_count() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::HistoricalOnly;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses);
        assert_eq!(result.eligible_independent_count, 0);
    }

    #[test]
    fn blocked_lifecycle_does_not_count() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::BlockedLifecycle;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedLifecycle);
    }

    #[test]
    fn blocked_dependency_does_not_count() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::BlockedDependency;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedDependency);
    }

    #[test]
    fn blocked_continuity_does_not_count() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::BlockedContinuity;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedContinuity);
    }

    #[test]
    fn archived_eligibility_does_not_count_current_finality() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.provenance = ObserverEvidenceProvenanceV1::Archived;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses);
    }

    #[test]
    fn observation_id_mismatch_blocks_join() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-2"]);
        let a = d6n_assessment(&s, &[(
            "obs-2".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedBinding);
    }

    #[test]
    fn generation_mismatch_does_not_count() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.observer_generation_id = "observer-A-g2".into();
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.eligible_independent_count, 0);
        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::BlockedContinuity
        );
    }

    #[test]
    fn d6n_assessment_commitment_mismatch_blocks() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1"]);
        s.set_commitment = "changed".into();
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let mut malformed = a.clone();
        malformed.assessment_commitment = "wrong".into();
        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s, &malformed, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedBinding);
    }

    #[test]
    fn environment_mismatch_blocks_join() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1"]);
        s.semantic_environment_root = "env-2".into();
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedBinding);
    }

    #[test]
    fn frontier_mismatch_blocks_current_composition() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1"]);
        s.observation_frontier_root = "frontier-old".into();
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-new", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedCurrentness);
    }

    #[test]
    fn lifecycle_profile_mismatch_blocks_join() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "other-profile", "frontier-1", 1
        );
        assert_eq!(result.eligible_independent_count, 0);
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedProfile);
    }

    #[test]
    fn two_eligible_independent_witnesses_satisfy_threshold() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&g1, &e1);
        let (_, r2) = ledger_and_receipt(&g2, &e2);
        let result = compose_finality_eligibility(
            &s, &a, &[e1, e2], &[r1, r2], "life-profile-1", "frontier-1", 2
        );
        assert_eq!(result.eligible_independent_count, 2);
    }

    #[test]
    fn many_ineligible_witnesses_cannot_substitute_for_eligible_witnesses() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let g3 = generation("observer-C");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let e3 = observation("obs-3", &g3, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2", "obs-3"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-3".into(), "observer-C".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, mut r1) = ledger_and_receipt(&g1, &e1);
        let (_, mut r2) = ledger_and_receipt(&g2, &e2);
        let (_, mut r3) = ledger_and_receipt(&g3, &e3);
        r1.disposition = EvidenceEligibilityDispositionV1::HistoricalOnly;
        r2.disposition = EvidenceEligibilityDispositionV1::BlockedDependency;
        r3.disposition = EvidenceEligibilityDispositionV1::BlockedContinuity;
        let result = compose_finality_eligibility(
            &s, &a, &[e1, e2, e3], &[r1, r2, r3], "life-profile-1", "frontier-1", 2
        );
        assert_eq!(result.eligible_independent_count, 0);
        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
        );
    }

    #[test]
    fn contradictory_d6n_observation_remains_preserved_after_lifecycle_change() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::NotApplied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::ContradictoryIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::HistoricalOnly;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::Contested);
        assert_eq!(result.preserved_contradictory_count, 1);
    }

    #[test]
    fn superseded_generation_cannot_count_under_same_observer_id() {
        let g1 = generation("observer-A");
        let e = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g1, &e);
        r.disposition = EvidenceEligibilityDispositionV1::BlockedContinuity;
        r.observer_generation_id = "observer-A-g1".into();
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.eligible_independent_count, 0);
    }

    #[test]
    fn later_dependency_change_removes_current_eligibility_without_rewriting_history() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::BlockedDependency;
        let result = compose_finality_eligibility(
            &s, &a, &[e.clone()], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.disposition, FinalityEligibilityDispositionV1::BlockedDependency);
        assert_eq!(e.observation.observation_id, "obs-1");
    }

    #[test]
    fn archive_mirror_cannot_count_as_current_independent_witness() {
        let mut g = generation("observer-A");
        g.role = ExternalObserverRoleV1::ArchiveMirror;
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::BlockedProfile;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(result.eligible_independent_count, 0);
    }

    #[test]
    fn duplicate_witness_identity_with_divergent_content_is_rejected() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let mut witness = FinalityWitnessEligibilityV1 {
            observation_id: "obs-1".into(),
            observer_id: "observer-A".into(),
            observer_generation_id: Some("observer-A".into()),
            d6n_observation_set_id: "set-1".into(),
            d6n_observation_set_commitment: "set-commitment".into(),
            d6n_assessment_item_commitment: "assessment-item".into(),
            d6n_classification: ObservationClassificationV1::CorroboratingIndependent,
            d6o_eligibility_id: Some("eligibility-1".into()),
            d6o_disposition: Some(EvidenceEligibilityDispositionV1::EligibleCurrent),
            d6o_dependency_snapshot_id: Some("snapshot-1".into()),
            observation_frontier_root: "frontier-1".into(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            witness_commitment: "witness-1".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        let mut ledger = FinalityEligibilityLedgerV1::default();
        assert_eq!(
            ledger.record_witness(witness.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        witness.observer_generation_id = Some("generation-substituted".into());
        witness.witness_commitment = witness.recomputed_commitment();
        assert_eq!(
            ledger.record_witness(witness),
            FinalityCompositionRecordDispositionV1::Conflict
        );
        let _ = e;
    }

    #[test]
    fn conflicting_terminal_composition_receipts_are_rejected() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let composition = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        let first = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-1".into(),
            effect_id: composition.effect_id.clone(),
            effect_lineage_id: composition.effect_lineage_id.clone(),
            lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
            route_id: composition.route_id.clone(),
            provider_id: composition.provider_id.clone(),
            provider_operation_id: composition.provider_operation_id.clone(),
            provider_profile_root: composition.provider_profile_root.clone(),
            semantic_environment_root: composition.semantic_environment_root.clone(),
            observation_set_id: composition.observation_set_id.clone(),
            observation_set_commitment: composition.observation_set_commitment.clone(),
            d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
            composition_commitment: composition.composition_commitment.clone(),
            witness_eligibility_ids: ["eligibility-obs-1".into()].into_iter().collect(),
            observer_generation_ids: ["observer-A".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(),
            receipt_commitment: "receipt-1-commitment".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        let second = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-2".into(),
            qualification_transition_id: "transition-2".into(),
            ..first.clone()
        };
        let mut ledger = FinalityEligibilityLedgerV1::default();
        assert_eq!(
            ledger.record_receipt(first),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(second),
            FinalityCompositionRecordDispositionV1::Conflict
        );
    }

    #[test]
    fn non_terminal_receipt_does_not_block_later_qualified_receipt() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let composition = compose_finality_eligibility(
            &s, &a, std::slice::from_ref(&e), std::slice::from_ref(&r),
            "life-profile-1", "frontier-1", 1
        );

        let non_terminal = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-insufficient".into(),
            effect_id: composition.effect_id.clone(),
            effect_lineage_id: composition.effect_lineage_id.clone(),
            lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
            route_id: composition.route_id.clone(),
            provider_id: composition.provider_id.clone(),
            provider_operation_id: composition.provider_operation_id.clone(),
            provider_profile_root: composition.provider_profile_root.clone(),
            semantic_environment_root: composition.semantic_environment_root.clone(),
            observation_set_id: composition.observation_set_id.clone(),
            observation_set_commitment: composition.observation_set_commitment.clone(),
            d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
            composition_commitment: composition.composition_commitment.clone(),
            witness_eligibility_ids: ["eligibility-obs-1".into()].into_iter().collect(),
            observer_generation_ids: ["observer-A".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            eligible_independent_count: 0,
            preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses,
            qualification_transition_id: "pending".into(),
            receipt_commitment: "pending-receipt".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };

        let terminal = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-qualified".into(),
            ..non_terminal.clone()
        };
        let terminal = CurrentFinalityEligibilityReceiptV1 {
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            eligible_independent_count: 1,
            qualification_transition_id: "qualified".into(),
            receipt_commitment: "qualified-receipt".into(),
            ..terminal
        };

        let mut ledger = FinalityEligibilityLedgerV1::default();
        assert_eq!(
            ledger.record_receipt(non_terminal),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(terminal),
            FinalityCompositionRecordDispositionV1::Recorded
        );
    }

    #[test]
    fn ledger_latest_index_is_not_authoritative_without_expected_frontier() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);
        let composition = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        let mut receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-frontier-1".into(),
            effect_id: composition.effect_id.clone(),
            effect_lineage_id: composition.effect_lineage_id.clone(),
            lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
            route_id: composition.route_id.clone(),
            provider_id: composition.provider_id.clone(),
            provider_operation_id: composition.provider_operation_id.clone(),
            provider_profile_root: composition.provider_profile_root.clone(),
            semantic_environment_root: composition.semantic_environment_root.clone(),
            observation_set_id: composition.observation_set_id.clone(),
            observation_set_commitment: composition.observation_set_commitment.clone(),
            d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
            composition_commitment: composition.composition_commitment.clone(),
            witness_eligibility_ids: composition
                .witnesses
                .iter()
                .filter_map(|w| w.d6o_eligibility_id.clone())
                .collect(),
            observer_generation_ids: composition
                .witnesses
                .iter()
                .filter_map(|w| w.observer_generation_id.clone())
                .collect(),
            current_frontier_root: composition.current_frontier_root.clone(),
            lifecycle_profile_id: composition.lifecycle_profile_id.clone(),
            eligible_independent_count: composition.eligible_independent_count,
            preserved_contradictory_count: composition.preserved_contradictory_count,
            disposition: composition.disposition,
            qualification_transition_id: composition
                .qualification_transition_id
                .clone()
                .unwrap_or_default(),
            receipt_commitment: String::new(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        receipt.receipt_commitment = receipt.recomputed_commitment();

        let mut ledger = FinalityEligibilityLedgerV1::default();
        assert_eq!(
            ledger.record_composition(composition.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(receipt.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert!(ledger
            .terminal_receipt_at_frontier("effect-1", "frontier-1")
            .is_some());
        assert!(ledger
            .terminal_receipt_at_frontier("effect-1", "frontier-2")
            .is_none());
        assert!(ledger
            .terminal_receipt_at_frontier("effect-1", "")
            .is_none());
        assert!(ledger
            .composition_at_frontier(&composition.composition_commitment, "frontier-1")
            .is_some());
        assert!(ledger
            .composition_at_frontier(&composition.composition_commitment, "frontier-2")
            .is_none());
    }

    #[test]
    fn ledger_preserves_terminal_receipts_across_frontier_versions() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);
        let first = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        let mut second = first.clone();
        second.current_frontier_root = "frontier-2".into();
        for witness in &mut second.witnesses {
            witness.current_frontier_root = "frontier-2".into();
        }
        second.qualification_transition_id = Some("transition-2".into());
        second.composition_commitment = second.recomputed_commitment();

        let make_receipt = |composition: &FinalityEligibilityCompositionV1, id: &str| {
            let mut receipt = CurrentFinalityEligibilityReceiptV1 {
                receipt_id: id.into(),
                effect_id: composition.effect_id.clone(),
                effect_lineage_id: composition.effect_lineage_id.clone(),
                lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
                route_id: composition.route_id.clone(),
                provider_id: composition.provider_id.clone(),
                provider_operation_id: composition.provider_operation_id.clone(),
                provider_profile_root: composition.provider_profile_root.clone(),
                semantic_environment_root: composition.semantic_environment_root.clone(),
                observation_set_id: composition.observation_set_id.clone(),
                observation_set_commitment: composition.observation_set_commitment.clone(),
                d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
                composition_commitment: composition.composition_commitment.clone(),
                witness_eligibility_ids: composition
                    .witnesses
                    .iter()
                    .filter_map(|w| w.d6o_eligibility_id.clone())
                    .collect(),
                observer_generation_ids: composition
                    .witnesses
                    .iter()
                    .filter_map(|w| w.observer_generation_id.clone())
                    .collect(),
                current_frontier_root: composition.current_frontier_root.clone(),
                lifecycle_profile_id: composition.lifecycle_profile_id.clone(),
                eligible_independent_count: composition.eligible_independent_count,
                preserved_contradictory_count: composition.preserved_contradictory_count,
                disposition: composition.disposition,
                qualification_transition_id: composition
                    .qualification_transition_id
                    .clone()
                    .unwrap_or_default(),
                receipt_commitment: String::new(),
                claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
            };
            receipt.receipt_commitment = receipt.recomputed_commitment();
            receipt
        };

        let first_receipt = make_receipt(&first, "receipt-frontier-1");
        let second_receipt = make_receipt(&second, "receipt-frontier-2");
        let mut ledger = FinalityEligibilityLedgerV1::default();

        assert_eq!(
            ledger.record_composition(first.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(first_receipt.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_composition(second.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(second_receipt.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );

        assert_eq!(ledger.compositions.len(), 2);
        assert_eq!(ledger.receipts.len(), 2);
        assert_eq!(
            ledger.terminal_receipt_by_effect.get(&first.effect_id),
            Some(&second_receipt.receipt_id)
        );
    }

    #[test]
    fn ledger_frontier_selection_is_arrival_order_independent() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);
        let first = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        let mut second = first.clone();
        second.current_frontier_root = "frontier-2".into();
        for witness in &mut second.witnesses {
            witness.current_frontier_root = "frontier-2".into();
        }
        second.qualification_transition_id = Some("transition-2".into());
        second.composition_commitment = second.recomputed_commitment();

        let make_receipt = |composition: &FinalityEligibilityCompositionV1, id: &str| {
            let mut receipt = CurrentFinalityEligibilityReceiptV1 {
                receipt_id: id.into(),
                effect_id: composition.effect_id.clone(),
                effect_lineage_id: composition.effect_lineage_id.clone(),
                lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
                route_id: composition.route_id.clone(),
                provider_id: composition.provider_id.clone(),
                provider_operation_id: composition.provider_operation_id.clone(),
                provider_profile_root: composition.provider_profile_root.clone(),
                semantic_environment_root: composition.semantic_environment_root.clone(),
                observation_set_id: composition.observation_set_id.clone(),
                observation_set_commitment: composition.observation_set_commitment.clone(),
                d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
                composition_commitment: composition.composition_commitment.clone(),
                witness_eligibility_ids: composition
                    .witnesses
                    .iter()
                    .filter_map(|w| w.d6o_eligibility_id.clone())
                    .collect(),
                observer_generation_ids: composition
                    .witnesses
                    .iter()
                    .filter_map(|w| w.observer_generation_id.clone())
                    .collect(),
                current_frontier_root: composition.current_frontier_root.clone(),
                lifecycle_profile_id: composition.lifecycle_profile_id.clone(),
                eligible_independent_count: composition.eligible_independent_count,
                preserved_contradictory_count: composition.preserved_contradictory_count,
                disposition: composition.disposition,
                qualification_transition_id: composition
                    .qualification_transition_id
                    .clone()
                    .unwrap_or_default(),
                receipt_commitment: String::new(),
                claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
            };
            receipt.receipt_commitment = receipt.recomputed_commitment();
            receipt
        };

        let first_receipt = make_receipt(&first, "receipt-frontier-1");
        let second_receipt = make_receipt(&second, "receipt-frontier-2");
        let mut ledger = FinalityEligibilityLedgerV1::default();

        assert_eq!(
            ledger.record_composition(second.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(second_receipt.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_composition(first.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(first_receipt.clone()),
            FinalityCompositionRecordDispositionV1::Recorded
        );

        assert_eq!(
            ledger
                .terminal_receipt_at_frontier(&first.effect_id, "frontier-1")
                .map(|receipt| &receipt.receipt_id),
            Some(&first_receipt.receipt_id)
        );
        assert_eq!(
            ledger
                .terminal_receipt_at_frontier(&first.effect_id, "frontier-2")
                .map(|receipt| &receipt.receipt_id),
            Some(&second_receipt.receipt_id)
        );
    }

    #[test]
    fn ledger_rejects_two_terminal_receipts_for_same_effect_and_frontier() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(
            &s,
            &[(
                "obs-1".into(),
                "observer-A".into(),
                ObservationClassificationV1::CorroboratingIndependent,
            )],
        );
        let (_, r) = ledger_and_receipt(&g, &e);
        let composition = compose_finality_eligibility(
            &s,
            &a,
            std::slice::from_ref(&e),
            std::slice::from_ref(&r),
            "life-profile-1",
            "frontier-1",
            1,
        );
        let mut make_receipt = |id: &str| {
            let mut receipt = CurrentFinalityEligibilityReceiptV1 {
                receipt_id: id.into(),
                effect_id: composition.effect_id.clone(),
                effect_lineage_id: composition.effect_lineage_id.clone(),
                lifecycle_generation_id: composition.lifecycle_generation_id.clone(),
                route_id: composition.route_id.clone(),
                provider_id: composition.provider_id.clone(),
                provider_operation_id: composition.provider_operation_id.clone(),
                provider_profile_root: composition.provider_profile_root.clone(),
                semantic_environment_root: composition.semantic_environment_root.clone(),
                observation_set_id: composition.observation_set_id.clone(),
                observation_set_commitment: composition.observation_set_commitment.clone(),
                d6n_assessment_commitment: composition.d6n_assessment_commitment.clone(),
                composition_commitment: composition.composition_commitment.clone(),
                witness_eligibility_ids: composition
                    .witnesses
                    .iter()
                    .filter_map(|w| w.d6o_eligibility_id.clone())
                    .collect(),
                observer_generation_ids: composition
                    .witnesses
                    .iter()
                    .filter_map(|w| w.observer_generation_id.clone())
                    .collect(),
                current_frontier_root: composition.current_frontier_root.clone(),
                lifecycle_profile_id: composition.lifecycle_profile_id.clone(),
                eligible_independent_count: composition.eligible_independent_count,
                preserved_contradictory_count: composition.preserved_contradictory_count,
                disposition: composition.disposition,
                qualification_transition_id: composition
                    .qualification_transition_id
                    .clone()
                    .unwrap_or_default(),
                receipt_commitment: String::new(),
                claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
            };
            receipt.receipt_commitment = receipt.recomputed_commitment();
            receipt
        };
        let first = make_receipt("receipt-1");
        let second = make_receipt("receipt-2");

        let mut ledger = FinalityEligibilityLedgerV1::default();
        assert_eq!(
            ledger.record_composition(composition),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(first),
            FinalityCompositionRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_receipt(second),
            FinalityCompositionRecordDispositionV1::Conflict
        );
    }

    fn out_of_order_d6n_d6o_delivery_converges() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, r) = ledger_and_receipt(&g, &e);
        let without_receipt = compose_finality_eligibility(
            &s, &a, &[e.clone()], &[], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(
            without_receipt.disposition,
            FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
        );
        let with_receipt = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(
            with_receipt.disposition,
            FinalityEligibilityDispositionV1::EligibleCurrent
        );
    }

    #[test]
    fn symthaea_composition_without_qualification_is_non_authoritative() {
        let g = generation("observer-A");
        let e = observation("obs-1", &g, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1"]);
        let a = d6n_assessment(&s, &[(
            "obs-1".into(),
            "observer-A".into(),
            ObservationClassificationV1::CorroboratingIndependent,
        )]);
        let (_, mut r) = ledger_and_receipt(&g, &e);
        r.disposition = EvidenceEligibilityDispositionV1::HistoricalOnly;
        let result = compose_finality_eligibility(
            &s, &a, &[e], &[r], "life-profile-1", "frontier-1", 1
        );
        assert_eq!(
            result.disposition,
            FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
        );
        assert!(result.qualification_transition_id.is_none());
    }

    #[test]
    fn composed_eligibility_cannot_authorize_actuation() {
        let receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment:set-commitment".into(),
            composition_commitment: "composition-test".into(),
            witness_eligibility_ids: ["eligibility-1".into()].into_iter().collect(),
            observer_generation_ids: ["generation-1".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(),
            receipt_commitment: "receipt-commitment".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        assert!(current_finality_receipt_is_non_authorizing(&receipt));
    }

    #[test]
    fn lifecycle_evidence_alone_cannot_establish_finality() {
        let mut witness = FinalityWitnessEligibilityV1 {
            observation_id: "obs-1".into(),
            observer_id: "observer-A".into(),
            observer_generation_id: Some("generation-1".into()),
            d6n_observation_set_id: "set-1".into(),
            d6n_observation_set_commitment: "set-commitment".into(),
            d6n_assessment_item_commitment: "assessment-item".into(),
            d6n_classification: ObservationClassificationV1::InsufficientEvidence,
            d6o_eligibility_id: Some("eligibility-1".into()),
            d6o_disposition: Some(EvidenceEligibilityDispositionV1::EligibleCurrent),
            d6o_dependency_snapshot_id: Some("snapshot-1".into()),
            observation_frontier_root: "frontier-1".into(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            witness_commitment: String::new(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        witness.witness_commitment = witness.recomputed_commitment();
        assert!(!witness.counts_as_current_independent_witness());
    }

    #[test]
    fn same_observer_cannot_inflate_independent_witness_threshold() {
        let generation = generation("observer-A");
        let e1 = observation("obs-1", &generation, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &generation, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&generation, &e1);
        let (_, r2) = ledger_and_receipt(&generation, &e2);

        let composition = compose_finality_eligibility(
            &s,
            &a,
            &[e1, e2],
            &[r1, r2],
            "life-profile-1",
            "frontier-1",
            2,
        );

        assert_eq!(composition.eligible_independent_count, 1);
        assert_eq!(
            composition.disposition,
            FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
        );
        assert!(composition.semantically_valid());
    }

    #[test]
    fn arrival_order_cannot_change_eligible_witness_count() {
        let g1 = generation("observer-A");
        let g2 = generation("observer-B");
        let e1 = observation("obs-1", &g1, ExternalObservedStateV1::Applied);
        let e2 = observation("obs-2", &g2, ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = d6n_assessment(
            &s,
            &[
                ("obs-1".into(), "observer-A".into(), ObservationClassificationV1::CorroboratingIndependent),
                ("obs-2".into(), "observer-B".into(), ObservationClassificationV1::CorroboratingIndependent),
            ],
        );
        let (_, r1) = ledger_and_receipt(&g1, &e1);
        let (_, r2) = ledger_and_receipt(&g2, &e2);
        let first = compose_finality_eligibility(
            &s, &a, &[e1.clone(), e2.clone()], &[r1.clone(), r2.clone()],
            "life-profile-1", "frontier-1", 2
        );
        let second = compose_finality_eligibility(
            &s, &a, &[e2, e1], &[r2, r1], "life-profile-1", "frontier-1", 2
        );
        assert_eq!(first.eligible_independent_count, second.eligible_independent_count);
        assert_eq!(first.disposition, second.disposition);
    }
}