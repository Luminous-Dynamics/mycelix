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
    ExternalObservedEvidenceV1, ExternalObservationSetV1, ObservationClassificationV1,
    ObservationSetAssessmentV1, CONTESTABLE_FINALITY_CLAIM_CEILING,
};
use crate::observer_lifecycle::{
    EvidenceEligibilityDispositionV1, EvidenceEligibilityReceiptV1,
    ObserverEvidenceProvenanceV1,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING: &str =
    "D6N/D6O finality-eligibility composition reference semantics only; no semantic authority or actuation claim.";
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
            && !self.observation_frontier_root.is_empty()
            && !self.current_frontier_root.is_empty()
            && !self.lifecycle_profile_id.is_empty()
            && !self.witness_commitment.is_empty()
            && self.claim_ceiling == FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING
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
        {
            return false;
        }

        let mut observation_ids = BTreeSet::new();
        let mut derived_eligible_count = 0u32;
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

            let expected_witness_commitment = format!(
                "witness:{}:{}:{}",
                witness.observation_id,
                self.observation_set_commitment,
                witness
                    .d6o_eligibility_id
                    .as_deref()
                    .unwrap_or("missing")
            );
            if witness.witness_commitment != expected_witness_commitment {
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
                derived_eligible_count += 1;
            }

            if matches!(
                witness.d6n_classification,
                ObservationClassificationV1::ContradictoryIndependent
                    | ObservationClassificationV1::ContradictoryDependent
            ) {
                derived_contradictory_count += 1;
            }
        }

        if self.eligible_independent_count != derived_eligible_count
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

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityEligibilityLedgerV1 {
    pub witnesses: BTreeMap<String, FinalityWitnessEligibilityV1>,
    pub compositions: BTreeMap<String, FinalityEligibilityCompositionV1>,
    pub receipts: BTreeMap<String, CurrentFinalityEligibilityReceiptV1>,
    pub terminal_receipt_by_effect: BTreeMap<String, String>,
}

impl FinalityEligibilityLedgerV1 {
    pub fn record_witness(
        &mut self,
        witness: FinalityWitnessEligibilityV1,
    ) -> FinalityCompositionRecordDispositionV1 {
        if !witness.structurally_valid() {
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

    pub fn record_composition(
        &mut self,
        composition: FinalityEligibilityCompositionV1,
    ) -> FinalityCompositionRecordDispositionV1 {
        if !composition.structurally_valid() {
            return FinalityCompositionRecordDispositionV1::InsufficientEvidence;
        }
        match self.compositions.get(&composition.composition_id) {
            Some(existing) if existing == &composition => {
                FinalityCompositionRecordDispositionV1::BlockedDuplicate
            }
            Some(_) => FinalityCompositionRecordDispositionV1::Conflict,
            None => {
                self.compositions
                    .insert(composition.composition_id.clone(), composition);
                FinalityCompositionRecordDispositionV1::Recorded
            }
        }
    }

    pub fn record_receipt(
        &mut self,
        receipt: CurrentFinalityEligibilityReceiptV1,
    ) -> FinalityCompositionRecordDispositionV1 {
        if !receipt.structurally_valid() {
            return FinalityCompositionRecordDispositionV1::InsufficientEvidence;
        }
        let terminal = matches!(
            receipt.disposition,
            FinalityEligibilityDispositionV1::EligibleCurrent
        );
        if terminal {
            if let Some(existing_id) = self.terminal_receipt_by_effect.get(&receipt.effect_id) {
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
    o.effect_id == set.effect_id
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
    if !receipt.structurally_valid() {
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
    FinalityWitnessEligibilityV1 {
        observation_id: assessment.observation_id.clone(),
        observer_id: assessment.observer_id.clone(),
        observer_generation_id: receipt.map(|r| r.observer_generation_id.clone()),
        d6n_observation_set_id: set.set_id.clone(),
        d6n_observation_set_commitment: set.set_commitment.clone(),
        d6n_classification: assessment.classification,
        d6o_eligibility_id: receipt.map(|r| r.eligibility_id.clone()),
        d6o_disposition: receipt.map(|r| r.disposition),
        d6o_dependency_snapshot_id: receipt.map(|r| r.dependency_snapshot_id.clone()),
        observation_frontier_root: set.observation_frontier_root.clone(),
        current_frontier_root: current_frontier_root.to_owned(),
        lifecycle_profile_id: lifecycle_profile_id.to_owned(),
        witness_commitment: format!(
            "witness:{}:{}:{}",
            assessment.observation_id,
            set.set_commitment,
            receipt.map(|r| r.eligibility_id.as_str()).unwrap_or("missing")
        ),
        claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.to_owned(),
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
            if let Some(existing) = evidence_by_id.insert(id.clone(), item) {
                if existing != item {
                    return empty(FinalityEligibilityDispositionV1::BlockedBinding);
                }
            }
        }
    }

    let mut receipt_by_id = BTreeMap::new();
    for receipt in eligibility_receipts {
        if receipt.structurally_valid() {
            let id = receipt.observation_id.clone();
            if let Some(existing) = receipt_by_id.insert(id.clone(), receipt) {
                if existing != receipt {
                    return empty(FinalityEligibilityDispositionV1::BlockedBinding);
                }
            }
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
    let mut eligible_count = 0u32;
    let mut preserved_contradictory_count = 0u32;
    let mut blocking_dispositions = Vec::new();

    for item in &assessment.assessments {
        let Some(observation) = evidence_by_id.get(&item.observation_id) else {
            return empty(FinalityEligibilityDispositionV1::BlockedBinding);
        };

        if !observation_matches_set(observation, set)
            || item.observer_id != observation.observer_id
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
                    eligible_count += 1;
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
    } else if eligible_count >= required_independent_observations {
        FinalityEligibilityDispositionV1::EligibleCurrent
    } else if distinct_blocking_dispositions.len() == 1 {
        distinct_blocking_dispositions[0]
    } else {
        FinalityEligibilityDispositionV1::InsufficientEligibleWitnesses
    };

    FinalityEligibilityCompositionV1 {
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
        eligible_independent_count: eligible_count,
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
    let mut composition = composition;
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
                d6n_classification: ObservationClassificationV1::CorroboratingIndependent,
                d6o_eligibility_id: Some("witness-1".into()),
                d6o_disposition: Some(EvidenceEligibilityDispositionV1::EligibleCurrent),
                d6o_dependency_snapshot_id: Some("dependency-1".into()),
                observation_frontier_root: "frontier-1".into(),
                current_frontier_root: "frontier-1".into(),
                lifecycle_profile_id: "lifecycle-1".into(),
                witness_commitment: "witness:observation-1:set-commitment-1:witness-1".into(),
                claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
            }],
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: Some("transition-1".into()),
            composition_commitment: "composition-commitment-1".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        composition.composition_commitment = composition.recomputed_commitment();
        composition
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
        ExternalObserverProfileV1, ExternalObservedStateV1, ExternalObservationSourceV1,
        ExternalObserverRoleV1, ObservationAssessmentV1, ObservationSetDispositionV1,
    };
    use crate::observer_lifecycle::{
        EvidenceDependencySnapshotV1, ObserverGenerationV1, ObserverLifecycleLedgerV1,
        ObserverLifecycleProfileV1, ObserverStatusV1,
    };

    fn lifecycle_profile() -> ObserverLifecycleProfileV1 {
        let allowed_roles = [ExternalObserverRoleV1::IndependentObserver]
            .into_iter()
            .collect::<BTreeSet<_>>();
        ObserverLifecycleProfileV1 {
            profile_id: "life-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_profile_id: "obs-profile-1".into(),
            allowed_roles,
            current_frontier_required: true,
            historical_evidence_allowed: true,
            profile_commitment: "life-profile-commitment".into(),
            claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
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
        ExternalObservedEvidenceV1 {
            observation: crate::contestable_finality::ExternalEffectObservationV1 {
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
                claim_ceiling: crate::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
            },
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
        ExternalObservationSetV1 {
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
        }
    }

    fn d6n_assessment(
        set: &ExternalObservationSetV1,
        classifications: &[(String, String, ObservationClassificationV1)],
    ) -> ObservationSetAssessmentV1 {
        let assessments = classifications
            .iter()
            .map(|(observation_id, observer_id, classification)| ObservationAssessmentV1 {
                observation_id: observation_id.clone(),
                observer_id: observer_id.clone(),
                independence: crate::contestable_finality::ObservationIndependenceV1::DeclaredIndependent,
                classification: *classification,
                evidence_root: format!("evidence-{observer_id}"),
                custody_root: format!("custody-{observer_id}"),
                assessment_commitment: format!("assessment-item:{observation_id}"),
                claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
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
            observer_id: observation.observer_id.clone(),
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
            eligibility_commitment: format!("eligibility:{}", observation.observation.observation_id),
            claim_ceiling: crate::observer_lifecycle::OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        (ledger, receipt)
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
        conflicting.provider_operation_id = "operation-conflict".into();

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
            &s, &a, &[e1, e2], &[r1, r2], "life-profile-1", "frontier-1", 1
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
        witness.witness_commitment = "different".into();
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
        let witness = FinalityWitnessEligibilityV1 {
            observation_id: "obs-1".into(),
            observer_id: "observer-A".into(),
            observer_generation_id: Some("generation-1".into()),
            d6n_observation_set_id: "set-1".into(),
            d6n_observation_set_commitment: "set-commitment".into(),
            d6n_classification: ObservationClassificationV1::InsufficientEvidence,
            d6o_eligibility_id: Some("eligibility-1".into()),
            d6o_disposition: Some(EvidenceEligibilityDispositionV1::EligibleCurrent),
            d6o_dependency_snapshot_id: Some("snapshot-1".into()),
            observation_frontier_root: "frontier-1".into(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            witness_commitment: "witness-1".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        assert!(!witness.counts_as_current_independent_witness());
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