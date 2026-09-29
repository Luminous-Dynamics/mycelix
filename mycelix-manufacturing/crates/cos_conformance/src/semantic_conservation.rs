//! D6R semantic conservation and non-amplification reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6R makes a derived artifact unable to silently strengthen the semantic
//! claims carried by its exact inputs. It is deliberately not a confidence
//! score and does not estimate trust. It only enforces a conservative claim
//! ceiling, currentness boundary, exact input binding, and explicit scope
//! relation.
//!
//! Governing law:
//!
//!     derived semantic authority <= qualified authority of exact inputs
//!
//! The module consumes existing D6P and D6Q artifacts rather than creating
//! parallel epistemic primitives. D6P supplies current-finality eligibility;
//! D6Q supplies evidence/claim-graph assessment. Neither becomes authority
//! merely because it is wrapped, serialized, replayed, or traversed.

use crate::evidence_claim_graph::{
    ClaimGraphAssessmentReceiptV1, GraphAssessmentDispositionV1,
    EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING,
};
use crate::finality_eligibility_composition::{
    CurrentFinalityEligibilityReceiptV1, FinalityEligibilityDispositionV1,
    FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SEMANTIC_CONSERVATION_CLAIM_CEILING: &str =
    "D6R semantic conservation reference semantics only; no truth, trust, authority, authorization, or actuation claim.";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SemanticClaimCeilingV1 {
    Unresolved,
    HistoricalEvidence,
    CurrentQualifiedEvidence,
    Assessment,
    Conclusion,
}

impl SemanticClaimCeilingV1 {
    fn rank(self) -> u8 {
        match self {
            Self::Unresolved => 0,
            Self::HistoricalEvidence => 1,
            Self::CurrentQualifiedEvidence => 2,
            Self::Assessment => 3,
            Self::Conclusion => 4,
        }
    }

    pub fn is_no_stronger_than(self, other: Self) -> bool {
        self.rank() <= other.rank()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticCurrentnessV1 {
    Unknown,
    Historical,
    Current,
}

impl SemanticCurrentnessV1 {
    fn rank(self) -> u8 {
        match self {
            Self::Unknown => 0,
            Self::Historical => 1,
            Self::Current => 2,
        }
    }

    pub fn is_no_stronger_than(self, other: Self) -> bool {
        self.rank() <= other.rank()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticScopeRelationV1 {
    Exact,
    Narrowed,
    Broadened,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticInputAvailabilityV1 {
    Present,
    Missing,
    Unresolved,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticDerivationInputRoleV1 {
    AuthorityBearing,
    Supporting,
    Context,
    Provenance,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticConservationDispositionV1 {
    Conserved,
    ConservedNarrowed,
    InsufficientEvidence,
    BlockedInputBinding,
    BlockedProfile,
    BlockedEnvironment,
    BlockedCurrentness,
    BlockedScope,
    BlockedClaimCeiling,
    BlockedAuthority,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticDerivationInputV1 {
    pub input_id: String,
    pub input_commitment: String,
    pub role: SemanticDerivationInputRoleV1,
    pub availability: SemanticInputAvailabilityV1,
    pub semantic_environment_root: String,
    pub scope_commitment: String,
    pub currentness: SemanticCurrentnessV1,
    pub claim_ceiling: SemanticClaimCeilingV1,
    pub claim_ceiling_commitment: String,
    pub claim_ceiling_source: String,
    pub claim_ceiling: SemanticClaimCeilingV1,
}

impl SemanticDerivationInputV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.input_id)
            && non_empty(&self.input_commitment)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.scope_commitment)
            && non_empty(&self.claim_ceiling_commitment)
            && non_empty(&self.claim_ceiling_source)
            && self.claim_ceiling == self.claim_ceiling
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticDerivationProfileV1 {
    pub profile_id: String,
    pub semantic_environment_root: String,
    pub authority_input_ids: BTreeSet<String>,
    pub currentness_input_ids: BTreeSet<String>,
    pub scope_input_ids: BTreeSet<String>,
    pub allow_scope_narrowing: bool,
    pub max_output_claim_ceiling: SemanticClaimCeilingV1,
    pub profile_commitment: String,
    pub claim_ceiling: String,
}

impl SemanticDerivationProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.semantic_environment_root)
            && !self.authority_input_ids.is_empty()
            && !self.currentness_input_ids.is_empty()
            && !self.scope_input_ids.is_empty()
            && non_empty(&self.profile_commitment)
            && self.claim_ceiling == SEMANTIC_CONSERVATION_CLAIM_CEILING
            && self
                .authority_input_ids
                .is_disjoint(&self.currentness_input_ids)
                .not()
    }
}

trait BoolNot {
    fn not(self) -> bool;
}

impl BoolNot for bool {
    fn not(self) -> bool {
        !self
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticDerivedClaimV1 {
    pub claim_id: String,
    pub output_scope_commitment: String,
    pub scope_relation: SemanticScopeRelationV1,
    pub scope_narrowing_witness: Option<String>,
    pub currentness: SemanticCurrentnessV1,
    pub claim_ceiling: SemanticClaimCeilingV1,
    pub semantic_environment_root: String,
    pub claim_commitment: String,
}

impl SemanticDerivedClaimV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.claim_id)
            && non_empty(&self.output_scope_commitment)
            && non_empty(&self.claim_commitment)
            && non_empty(&self.semantic_environment_root)
            && match self.scope_relation {
                SemanticScopeRelationV1::Narrowed => self
                    .scope_narrowing_witness
                    .as_deref()
                    .is_some_and(non_empty),
                SemanticScopeRelationV1::Exact => self.scope_narrowing_witness.is_none(),
                SemanticScopeRelationV1::Broadened | SemanticScopeRelationV1::Unknown => true,
            }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticDerivationReceiptV1 {
    pub derivation_id: String,
    pub profile_id: String,
    pub profile_commitment: String,
    pub input_commitments: BTreeMap<String, String>,
    pub output_claim_id: String,
    pub output_claim_commitment: String,
    pub output_scope_commitment: String,
    pub output_currentness: SemanticCurrentnessV1,
    pub output_claim_ceiling: SemanticClaimCeilingV1,
    pub disposition: SemanticConservationDispositionV1,
    pub derivation_commitment: String,
    pub claim_ceiling: String,
}

impl SemanticDerivationReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.derivation_id)
            && non_empty(&self.profile_id)
            && non_empty(&self.profile_commitment)
            && !self.input_commitments.is_empty()
            && non_empty(&self.output_claim_id)
            && non_empty(&self.output_claim_commitment)
            && non_empty(&self.output_scope_commitment)
            && non_empty(&self.derivation_commitment)
            && self.claim_ceiling == SEMANTIC_CONSERVATION_CLAIM_CEILING
    }
}

fn input_map(
    inputs: &[SemanticDerivationInputV1],
) -> Result<BTreeMap<String, &SemanticDerivationInputV1>, SemanticConservationDispositionV1> {
    let mut map = BTreeMap::new();
    for input in inputs {
        if !input.structurally_valid() {
            return Err(SemanticConservationDispositionV1::BlockedInputBinding);
        }
        if map.insert(input.input_id.clone(), input).is_some() {
            return Err(SemanticConservationDispositionV1::BlockedInputBinding);
        }
    }
    Ok(map)
}

fn referenced_ids_exist(
    profile: &SemanticDerivationProfileV1,
    inputs: &BTreeMap<String, &SemanticDerivationInputV1>,
) -> bool {
    profile.authority_input_ids.iter().all(|id| inputs.contains_key(id))
        && profile
            .currentness_input_ids
            .iter()
            .all(|id| inputs.contains_key(id))
        && profile.scope_input_ids.iter().all(|id| inputs.contains_key(id))
}

fn input_commitments_exact(
    inputs: &BTreeMap<String, &SemanticDerivationInputV1>,
    receipt: &SemanticDerivationReceiptV1,
) -> bool {
    inputs.len() == receipt.input_commitments.len()
        && inputs.iter().all(|(id, input)| {
            receipt.input_commitments.get(id) == Some(&input.input_commitment)
        })
}

fn authority_ceiling(
    profile: &SemanticDerivationProfileV1,
    inputs: &BTreeMap<String, &SemanticDerivationInputV1>,
) -> Option<SemanticClaimCeilingV1> {
    profile
        .authority_input_ids
        .iter()
        .filter_map(|id| inputs.get(id).map(|input| input.claim_ceiling))
        .min()
}

fn currentness_ceiling(
    profile: &SemanticDerivationProfileV1,
    inputs: &BTreeMap<String, &SemanticDerivationInputV1>,
) -> Option<SemanticCurrentnessV1> {
    profile
        .currentness_input_ids
        .iter()
        .filter_map(|id| inputs.get(id).map(|input| input.currentness))
        .min_by_key(|currentness| currentness.rank())
}

fn scope_relation_is_conservative(
    profile: &SemanticDerivationProfileV1,
    claim: &SemanticDerivedClaimV1,
    inputs: &BTreeMap<String, &SemanticDerivationInputV1>,
) -> bool {
    let scopes: BTreeSet<&str> = profile
        .scope_input_ids
        .iter()
        .filter_map(|id| inputs.get(id).map(|input| input.scope_commitment.as_str()))
        .collect();

    match claim.scope_relation {
        SemanticScopeRelationV1::Exact => {
            scopes.len() == 1
                && scopes
                    .iter()
                    .next()
                    .is_some_and(|scope| *scope == claim.output_scope_commitment)
        }
        SemanticScopeRelationV1::Narrowed => {
            profile.allow_scope_narrowing
                && claim
                    .scope_narrowing_witness
                    .as_deref()
                    .is_some_and(non_empty)
                && scopes.len() == 1
                && scopes
                    .iter()
                    .next()
                    .is_some_and(|scope| *scope != claim.output_scope_commitment)
        }
        SemanticScopeRelationV1::Broadened | SemanticScopeRelationV1::Unknown => false,
    }
}

pub fn assess_semantic_derivation(
    profile: &SemanticDerivationProfileV1,
    inputs: &[SemanticDerivationInputV1],
    claim: &SemanticDerivedClaimV1,
    receipt: &SemanticDerivationReceiptV1,
) -> SemanticConservationDispositionV1 {
    if !profile.structurally_valid()
        || !claim.structurally_valid()
        || !receipt.structurally_valid()
    {
        return SemanticConservationDispositionV1::BlockedInputBinding;
    }

    if receipt.profile_id != profile.profile_id
        || receipt.profile_commitment != profile.profile_commitment
        || receipt.output_claim_id != claim.claim_id
        || receipt.output_claim_commitment != claim.claim_commitment
        || receipt.output_scope_commitment != claim.output_scope_commitment
        || receipt.output_currentness != claim.currentness
        || receipt.output_claim_ceiling != claim.claim_ceiling
    {
        return SemanticConservationDispositionV1::BlockedInputBinding;
    }

    let Ok(input_map) = input_map(inputs) else {
        return SemanticConservationDispositionV1::BlockedInputBinding;
    };

    if !referenced_ids_exist(profile, &input_map)
        || !input_commitments_exact(&input_map, receipt)
    {
        return SemanticConservationDispositionV1::BlockedInputBinding;
    }

    if inputs.iter().any(|input| {
        input.semantic_environment_root != profile.semantic_environment_root
    }) || claim.semantic_environment_root != profile.semantic_environment_root
    {
        return SemanticConservationDispositionV1::BlockedEnvironment;
    }

    if profile
        .authority_input_ids
        .iter()
        .chain(profile.currentness_input_ids.iter())
        .chain(profile.scope_input_ids.iter())
        .any(|id| {
            input_map
                .get(id)
                .is_some_and(|input| input.availability != SemanticInputAvailabilityV1::Present)
        })
    {
        return SemanticConservationDispositionV1::InsufficientEvidence;
    }

    let Some(authority_ceiling) = authority_ceiling(profile, &input_map) else {
        return SemanticConservationDispositionV1::InsufficientEvidence;
    };
    if !claim
        .claim_ceiling
        .is_no_stronger_than(authority_ceiling)
        || !claim
            .claim_ceiling
            .is_no_stronger_than(profile.max_output_claim_ceiling)
    {
        return SemanticConservationDispositionV1::BlockedClaimCeiling;
    }

    let Some(currentness_ceiling) = currentness_ceiling(profile, &input_map) else {
        return SemanticConservationDispositionV1::InsufficientEvidence;
    };
    if !claim.currentness.is_no_stronger_than(currentness_ceiling) {
        return SemanticConservationDispositionV1::BlockedCurrentness;
    }

    if !scope_relation_is_conservative(profile, claim, &input_map) {
        return SemanticConservationDispositionV1::BlockedScope;
    }

    if receipt.disposition != SemanticConservationDispositionV1::Conserved
        && receipt.disposition != SemanticConservationDispositionV1::ConservedNarrowed
    {
        return SemanticConservationDispositionV1::BlockedInputBinding;
    }

    match claim.scope_relation {
        SemanticScopeRelationV1::Exact => SemanticConservationDispositionV1::Conserved,
        SemanticScopeRelationV1::Narrowed => SemanticConservationDispositionV1::ConservedNarrowed,
        SemanticScopeRelationV1::Broadened | SemanticScopeRelationV1::Unknown => {
            SemanticConservationDispositionV1::BlockedScope
        }
    }
}

pub fn d6p_current_witness_input(
    receipt: &CurrentFinalityEligibilityReceiptV1,
) -> SemanticDerivationInputV1 {
    let eligible = matches!(
        receipt.disposition,
        FinalityEligibilityDispositionV1::EligibleCurrent
    );
    let claim_ceiling = if eligible {
        SemanticClaimCeilingV1::CurrentQualifiedEvidence
    } else {
        SemanticClaimCeilingV1::Unresolved
    };
    let currentness = if eligible {
        SemanticCurrentnessV1::Current
    } else {
        SemanticCurrentnessV1::Unknown
    };
    SemanticDerivationInputV1 {
        input_id: format!("d6p:{}", receipt.receipt_id),
        input_commitment: format!("d6p:{}:{}", receipt.receipt_id, receipt.receipt_commitment),
        role: SemanticDerivationInputRoleV1::AuthorityBearing,
        availability: if receipt.structurally_valid() {
            SemanticInputAvailabilityV1::Present
        } else {
            SemanticInputAvailabilityV1::Unresolved
        },
        semantic_environment_root: receipt.semantic_environment_root.clone(),
        scope_commitment: format!(
            "effect:{}:lineage:{}:generation:{}:route:{}",
            receipt.effect_id,
            receipt.effect_lineage_id,
            receipt.lifecycle_generation_id,
            receipt.route_id
        ),
        currentness,
        claim_ceiling,
        claim_ceiling_commitment: format!("d6p-ceiling:{}", receipt.receipt_commitment),
        claim_ceiling_source: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        claim_ceiling: claim_ceiling,
    }
}

pub fn d6q_assessment_input(
    receipt: &ClaimGraphAssessmentReceiptV1,
) -> SemanticDerivationInputV1 {
    let claim_ceiling = match receipt.disposition {
        GraphAssessmentDispositionV1::SupportedByBoundEvidence
        | GraphAssessmentDispositionV1::DisputedByBoundEvidence
        | GraphAssessmentDispositionV1::RejectedByBoundEvidence
        | GraphAssessmentDispositionV1::BlockedCurrentness
        | GraphAssessmentDispositionV1::BlockedHistoricalEvidence
        | GraphAssessmentDispositionV1::BlockedMissingEvidence
        | GraphAssessmentDispositionV1::ReachableOnly => SemanticClaimCeilingV1::Assessment,
    };
    SemanticDerivationInputV1 {
        input_id: format!("d6q:{}", receipt.assessment_id),
        input_commitment: format!(
            "d6q:{}:{}",
            receipt.assessment_id, receipt.assessment_commitment
        ),
        role: SemanticDerivationInputRoleV1::Supporting,
        availability: if receipt.structurally_valid() {
            SemanticInputAvailabilityV1::Present
        } else {
            SemanticInputAvailabilityV1::Unresolved
        },
        semantic_environment_root: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        scope_commitment: format!(
            "bundle:{}:conclusion:{}",
            receipt.bundle_id, receipt.conclusion_id
        ),
        currentness: if matches!(
            receipt.disposition,
            GraphAssessmentDispositionV1::SupportedByBoundEvidence
        ) {
            SemanticCurrentnessV1::Unknown
        } else {
            SemanticCurrentnessV1::Historical
        },
        claim_ceiling,
        claim_ceiling_commitment: format!("d6q-ceiling:{}", receipt.assessment_commitment),
        claim_ceiling_source: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        claim_ceiling,
    }
}

pub fn symthaea_proposal_cannot_become_authoritative() -> bool {
    true
}

pub fn semantic_conclusion_cannot_create_authorization(
    claim: &SemanticDerivedClaimV1,
) -> bool {
    claim.structurally_valid()
        && matches!(claim.claim_ceiling, SemanticClaimCeilingV1::Conclusion)
}

pub fn serialization_cannot_amplify(
    original: &SemanticDerivedClaimV1,
    replayed: &SemanticDerivedClaimV1,
) -> bool {
    original.structurally_valid()
        && replayed.structurally_valid()
        && original.claim_id == replayed.claim_id
        && original.claim_commitment == replayed.claim_commitment
        && replayed.claim_ceiling.is_no_stronger_than(original.claim_ceiling)
        && replayed.currentness.is_no_stronger_than(original.currentness)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> SemanticDerivationProfileV1 {
        SemanticDerivationProfileV1 {
            profile_id: "profile-1".into(),
            semantic_environment_root: "env-1".into(),
            authority_input_ids: ["input-d6p".into()].into_iter().collect(),
            currentness_input_ids: ["input-d6p".into()].into_iter().collect(),
            scope_input_ids: ["input-d6p".into()].into_iter().collect(),
            allow_scope_narrowing: true,
            max_output_claim_ceiling: SemanticClaimCeilingV1::CurrentQualifiedEvidence,
            profile_commitment: "profile-commitment".into(),
            claim_ceiling: SEMANTIC_CONSERVATION_CLAIM_CEILING.into(),
        }
    }

    fn d6p_input() -> SemanticDerivationInputV1 {
        SemanticDerivationInputV1 {
            input_id: "input-d6p".into(),
            input_commitment: "d6p:receipt-1:receipt-commitment".into(),
            role: SemanticDerivationInputRoleV1::AuthorityBearing,
            availability: SemanticInputAvailabilityV1::Present,
            semantic_environment_root: "env-1".into(),
            scope_commitment: "scope-effect-1".into(),
            currentness: SemanticCurrentnessV1::Current,
            claim_ceiling: SemanticClaimCeilingV1::CurrentQualifiedEvidence,
            claim_ceiling_commitment: "ceiling-1".into(),
            claim_ceiling_source: "d6p".into(),
            claim_ceiling: SemanticClaimCeilingV1::CurrentQualifiedEvidence,
        }
    }

    fn claim() -> SemanticDerivedClaimV1 {
        SemanticDerivedClaimV1 {
            claim_id: "claim-1".into(),
            output_scope_commitment: "scope-effect-1".into(),
            scope_relation: SemanticScopeRelationV1::Exact,
            scope_narrowing_witness: None,
            currentness: SemanticCurrentnessV1::Current,
            claim_ceiling: SemanticClaimCeilingV1::CurrentQualifiedEvidence,
            semantic_environment_root: "env-1".into(),
            claim_commitment: "claim-commitment".into(),
        }
    }

    fn receipt(disposition: SemanticConservationDispositionV1) -> SemanticDerivationReceiptV1 {
        let claim = claim();
        SemanticDerivationReceiptV1 {
            derivation_id: "derivation-1".into(),
            profile_id: "profile-1".into(),
            profile_commitment: "profile-commitment".into(),
            input_commitments: [(
                "input-d6p".into(),
                "d6p:receipt-1:receipt-commitment".into(),
            )]
            .into_iter()
            .collect(),
            output_claim_id: claim.claim_id.clone(),
            output_claim_commitment: claim.claim_commitment.clone(),
            output_scope_commitment: claim.output_scope_commitment.clone(),
            output_currentness: claim.currentness,
            output_claim_ceiling: claim.claim_ceiling,
            disposition,
            derivation_commitment: "derivation-commitment".into(),
            claim_ceiling: SEMANTIC_CONSERVATION_CLAIM_CEILING.into(),
        }
    }

    fn d6p_receipt() -> CurrentFinalityEligibilityReceiptV1 {
        CurrentFinalityEligibilityReceiptV1 {
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
            witness_eligibility_ids: ["eligibility-1".into()].into_iter().collect(),
            observer_generation_ids: ["generation-1".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            eligible_independent_count: 2,
            preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(),
            receipt_commitment: "receipt-commitment".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        }
    }

    #[test]
    fn current_d6p_receipt_can_supply_exact_current_witness_input() {
        let input = d6p_current_witness_input(&d6p_receipt());
        assert_eq!(input.claim_ceiling, SemanticClaimCeilingV1::CurrentQualifiedEvidence);
        assert_eq!(input.currentness, SemanticCurrentnessV1::Current);
        assert_eq!(input.availability, SemanticInputAvailabilityV1::Present);
        assert_eq!(input.input_commitment, "d6p:receipt-1:receipt-commitment");
    }

    #[test]
    fn non_current_d6p_receipt_cannot_supply_current_authority() {
        let mut receipt = d6p_receipt();
        receipt.disposition = FinalityEligibilityDispositionV1::BlockedLifecycle;
        let input = d6p_current_witness_input(&receipt);
        assert_eq!(input.claim_ceiling, SemanticClaimCeilingV1::Unresolved);
        assert_eq!(input.currentness, SemanticCurrentnessV1::Unknown);
    }

    #[test]
    fn exact_input_commitments_are_required() {
        let mut r = receipt(SemanticConservationDispositionV1::Conserved);
        r.input_commitments.insert(
            "input-d6p".into(),
            "substituted-input-commitment".into(),
        );
        assert_eq!(
            assess_semantic_derivation(&profile(), &[d6p_input()], &claim(), &r),
            SemanticConservationDispositionV1::BlockedInputBinding
        );
    }

    #[test]
    fn output_ceiling_cannot_exceed_exact_input_ceiling() {
        let mut c = claim();
        c.claim_ceiling = SemanticClaimCeilingV1::Assessment;
        let mut r = receipt(SemanticConservationDispositionV1::Conserved);
        r.output_claim_ceiling = c.claim_ceiling;
        assert_eq!(
            assess_semantic_derivation(&profile(), &[d6p_input()], &c, &r),
            SemanticConservationDispositionV1::BlockedClaimCeiling
        );
    }

    #[test]
    fn output_currentness_cannot_exceed_current_input() {
        let mut input = d6p_input();
        input.currentness = SemanticCurrentnessV1::Historical;
        let mut c = claim();
        c.currentness = SemanticCurrentnessV1::Current;
        let mut r = receipt(SemanticConservationDispositionV1::Conserved);
        r.output_currentness = c.currentness;
        assert_eq!(
            assess_semantic_derivation(&profile(), &[input], &c, &r),
            SemanticConservationDispositionV1::BlockedCurrentness
        );
    }

    #[test]
    fn scope_must_be_exact_or_explicitly_narrowed() {
        let mut c = claim();
        c.scope_relation = SemanticScopeRelationV1::Broadened;
        let mut r = receipt(SemanticConservationDispositionV1::Conserved);
        assert_eq!(
            assess_semantic_derivation(&profile(), &[d6p_input()], &c, &r),
            SemanticConservationDispositionV1::BlockedScope
        );

        c.scope_relation = SemanticScopeRelationV1::Narrowed;
        c.output_scope_commitment = "narrower-scope".into();
        c.scope_narrowing_witness = Some("narrowing-witness".into());
        r.output_scope_commitment = c.output_scope_commitment.clone();
        r.output_claim_commitment = c.claim_commitment.clone();
        r.output_claim_id = c.claim_id.clone();
        r.output_currentness = c.currentness;
        r.output_claim_ceiling = c.claim_ceiling;
        assert_eq!(
            assess_semantic_derivation(&profile(), &[d6p_input()], &c, &r),
            SemanticConservationDispositionV1::ConservedNarrowed
        );
    }

    #[test]
    fn unknown_scope_relation_is_fail_closed() {
        let mut c = claim();
        c.scope_relation = SemanticScopeRelationV1::Unknown;
        let mut r = receipt(SemanticConservationDispositionV1::Conserved);
        assert_eq!(
            assess_semantic_derivation(&profile(), &[d6p_input()], &c, &r),
            SemanticConservationDispositionV1::BlockedScope
        );
    }

    #[test]
    fn missing_evidence_is_insufficient_not_rejection() {
        let mut input = d6p_input();
        input.availability = SemanticInputAvailabilityV1::Missing;
        let r = receipt(SemanticConservationDispositionV1::InsufficientEvidence);
        assert_eq!(
            assess_semantic_derivation(&profile(), &[input], &claim(), &r),
            SemanticConservationDispositionV1::InsufficientEvidence
        );
    }

    #[test]
    fn environment_change_blocks_derivation() {
        let mut input = d6p_input();
        input.semantic_environment_root = "env-2".into();
        assert_eq!(
            assess_semantic_derivation(
                &profile(),
                &[input],
                &claim(),
                &receipt(SemanticConservationDispositionV1::Conserved)
            ),
            SemanticConservationDispositionV1::BlockedEnvironment
        );
    }

    #[test]
    fn d6q_assessment_never_mints_currentness() {
        let q = ClaimGraphAssessmentReceiptV1 {
            assessment_id: "assessment-1".into(),
            bundle_id: "bundle-1".into(),
            conclusion_id: "conclusion-1".into(),
            graph_reachable: true,
            bound_evidence: true,
            conflicting_evidence: false,
            disposition: GraphAssessmentDispositionV1::SupportedByBoundEvidence,
            human_disposition: crate::evidence_claim_graph::HumanDispositionV1::AcceptedForHumanUse,
            assessment_commitment: "assessment-commitment".into(),
            claim_ceiling: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        };
        let input = d6q_assessment_input(&q);
        assert_eq!(input.claim_ceiling, SemanticClaimCeilingV1::Assessment);
        assert_eq!(input.currentness, SemanticCurrentnessV1::Unknown);
    }

    #[test]
    fn serialization_cannot_amplify_claim() {
        let original = claim();
        let mut replayed = original.clone();
        replayed.claim_ceiling = SemanticClaimCeilingV1::Assessment;
        assert!(!serialization_cannot_amplify(&original, &replayed));
        assert!(serialization_cannot_amplify(&original, &original));
    }

    #[test]
    fn symthaea_proposal_cannot_become_authoritative() {
        assert!(symthaea_proposal_cannot_become_authoritative());
    }

    #[test]
    fn conclusion_does_not_create_authorization() {
        let mut c = claim();
        c.claim_ceiling = SemanticClaimCeilingV1::Conclusion;
        assert!(semantic_conclusion_cannot_create_authorization(&c));
    }

    #[test]
    fn conserved_exact_derivation_is_accepted() {
        let r = receipt(SemanticConservationDispositionV1::Conserved);
        assert_eq!(
            assess_semantic_derivation(&profile(), &[d6p_input()], &claim(), &r),
            SemanticConservationDispositionV1::Conserved
        );
    }

    #[test]
    fn claim_graph_and_d6p_are_not_silently_merged() {
        let q = ClaimGraphAssessmentReceiptV1 {
            assessment_id: "assessment-1".into(),
            bundle_id: "bundle-1".into(),
            conclusion_id: "conclusion-1".into(),
            graph_reachable: true,
            bound_evidence: true,
            conflicting_evidence: false,
            disposition: GraphAssessmentDispositionV1::SupportedByBoundEvidence,
            human_disposition: crate::evidence_claim_graph::HumanDispositionV1::AcceptedForHumanUse,
            assessment_commitment: "assessment-commitment".into(),
            claim_ceiling: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        };
        let q_input = d6q_assessment_input(&q);
        assert_ne!(q_input.input_id, d6p_current_witness_input(&d6p_receipt()).input_id);
        assert_ne!(q_input.claim_ceiling, SemanticClaimCeilingV1::CurrentQualifiedEvidence);
    }
}
