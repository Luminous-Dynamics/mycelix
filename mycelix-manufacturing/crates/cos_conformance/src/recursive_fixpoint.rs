//! D6U — explicitly qualified recursive/fixpoint derivations.
//!
//! Status: ReferenceModelOnly.
//!
//! D6U does not make a cyclic DKG authoritative. It makes a recursive
//! semantic derivation explicit, bounded, replayable, and independently
//! committable. A fixpoint is a property of an exact trace under an exact
//! profile; convergence itself does not create truth, currentness, authority,
//! authorization, or actuation rights.

use crate::canonical_derivation_receipt::{canonical_sha256, D6S_CLAIM_CEILING};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const D6U_RULE_ID: &str = "recursive-fixpoint-v1";
pub const D6U_ITERATION_ORDER_RULE: &str = "ascending-index-v1";
pub const D6U_CONVERGENCE_RULE: &str = "state-equality-v1";
pub const D6U_CLAIM_CEILING: &str =
    "ReferenceModelOnly; recursive derivation integrity semantics only; no truth, authority, authorization, or actuation claim.";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecursiveResultStatusV1 {
    Converged,
    NonConvergentWithinBound,
    BlockedIterationBound,
    BlockedResourceBound,
    InsufficientEvidence,
    InvalidTrace,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecursiveDerivationProfileV1 {
    pub profile_id: String,
    pub version: String,
    pub rule_id: String,
    pub iteration_order_rule: String,
    pub convergence_rule: String,
    pub carrier_commitment: String,
    pub max_iterations: u64,
    pub max_trace_items: u64,
    pub claim_ceiling: String,
    pub profile_commitment: String,
}

impl RecursiveDerivationProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.profile_id.trim().is_empty()
            && !self.version.trim().is_empty()
            && self.rule_id == D6U_RULE_ID
            && self.iteration_order_rule == D6U_ITERATION_ORDER_RULE
            && self.convergence_rule == D6U_CONVERGENCE_RULE
            && !self.carrier_commitment.trim().is_empty()
            && self.max_iterations > 0
            && self.max_trace_items > 0
            && self.max_trace_items >= self.max_iterations
            && self.claim_ceiling == D6U_CLAIM_CEILING
            && !self.profile_commitment.trim().is_empty()
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.profile_commitment.clear();
        canonical_sha256("recursive-profile", &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.profile_commitment == self.recomputed_commitment()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecursiveSeedV1 {
    pub seed_id: String,
    pub projection_commitment: String,
    pub semantic_environment_commitment: String,
    pub input_commitments: BTreeSet<String>,
    pub initial_state_commitment: String,
    pub seed_commitment: String,
    pub claim_ceiling: String,
}

impl RecursiveSeedV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.seed_id.trim().is_empty()
            && !self.projection_commitment.trim().is_empty()
            && !self.semantic_environment_commitment.trim().is_empty()
            && !self.input_commitments.is_empty()
            && self.input_commitments.iter().all(|v| !v.trim().is_empty())
            && !self.initial_state_commitment.trim().is_empty()
            && !self.seed_commitment.trim().is_empty()
            && self.claim_ceiling == D6U_CLAIM_CEILING
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.seed_commitment.clear();
        canonical_sha256("recursive-seed", &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.seed_commitment == self.recomputed_commitment()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecursiveIterationV1 {
    pub iteration_index: u64,
    pub predecessor_state_commitment: String,
    pub state_commitment: String,
    pub delta_commitment: String,
    pub transition_commitment: String,
}

impl RecursiveIterationV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.predecessor_state_commitment.trim().is_empty()
            && !self.state_commitment.trim().is_empty()
            && !self.delta_commitment.trim().is_empty()
            && !self.transition_commitment.trim().is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecursiveDerivationTraceV1 {
    pub trace_id: String,
    pub projection_commitment: String,
    pub semantic_environment_commitment: String,
    pub profile_commitment: String,
    pub seed_commitment: String,
    pub initial_state_commitment: String,
    pub iterations: Vec<RecursiveIterationV1>,
    pub result_status: RecursiveResultStatusV1,
    pub unresolved_preserved: bool,
    pub trace_commitment: String,
    pub claim_ceiling: String,
}

impl RecursiveDerivationTraceV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.trace_id.trim().is_empty()
            && !self.projection_commitment.trim().is_empty()
            && !self.semantic_environment_commitment.trim().is_empty()
            && !self.profile_commitment.trim().is_empty()
            && !self.seed_commitment.trim().is_empty()
            && !self.initial_state_commitment.trim().is_empty()
            && !self.iterations.is_empty()
            && self.iterations.iter().all(RecursiveIterationV1::structurally_valid)
            && !self.trace_commitment.trim().is_empty()
            && self.claim_ceiling == D6U_CLAIM_CEILING
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.trace_commitment.clear();
        canonical_sha256("recursive-trace", &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.trace_commitment == self.recomputed_commitment()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecursiveDerivationReceiptV1 {
    pub receipt_id: String,
    pub projection_commitment: String,
    pub semantic_environment_commitment: String,
    pub profile_commitment: String,
    pub seed_commitment: String,
    pub trace_commitment: String,
    pub result_status: RecursiveResultStatusV1,
    pub final_state_commitment: String,
    pub iteration_count: u64,
    pub claim_ceiling: String,
    pub receipt_commitment: String,
}

impl RecursiveDerivationReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.receipt_id.trim().is_empty()
            && !self.projection_commitment.trim().is_empty()
            && !self.semantic_environment_commitment.trim().is_empty()
            && !self.profile_commitment.trim().is_empty()
            && !self.seed_commitment.trim().is_empty()
            && !self.trace_commitment.trim().is_empty()
            && !self.final_state_commitment.trim().is_empty()
            && self.iteration_count > 0
            && !self.receipt_commitment.trim().is_empty()
            && self.claim_ceiling == D6U_CLAIM_CEILING
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.receipt_commitment.clear();
        canonical_sha256("recursive-receipt", &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.receipt_commitment == self.recomputed_commitment()
    }
}

fn result_flags_are_consistent(
    status: RecursiveResultStatusV1,
    unresolved_preserved: bool,
) -> bool {
    match status {
        RecursiveResultStatusV1::Converged => !unresolved_preserved,
        RecursiveResultStatusV1::NonConvergentWithinBound
        | RecursiveResultStatusV1::BlockedIterationBound
        | RecursiveResultStatusV1::BlockedResourceBound
        | RecursiveResultStatusV1::InsufficientEvidence
        | RecursiveResultStatusV1::InvalidTrace => unresolved_preserved,
    }
}

fn trace_order_and_chain_are_valid(
    trace: &RecursiveDerivationTraceV1,
    seed: &RecursiveSeedV1,
    profile: &RecursiveDerivationProfileV1,
) -> bool {
    if trace.iterations.len() as u64 > profile.max_iterations
        || trace.iterations.len() as u64 > profile.max_trace_items
    {
        return false;
    }

    let mut predecessor = seed.initial_state_commitment.as_str();
    for (expected_index, iteration) in trace.iterations.iter().enumerate() {
        if iteration.iteration_index != expected_index as u64
            || iteration.predecessor_state_commitment != predecessor
        {
            return false;
        }
        predecessor = &iteration.state_commitment;
    }

    true
}

fn result_is_consistent_with_trace(
    trace: &RecursiveDerivationTraceV1,
    seed: &RecursiveSeedV1,
    profile: &RecursiveDerivationProfileV1,
) -> bool {
    if !result_flags_are_consistent(trace.result_status, trace.unresolved_preserved)
        || !trace_order_and_chain_are_valid(trace, seed, profile)
    {
        return false;
    }

    let Some(last) = trace.iterations.last() else {
        return false;
    };

    match trace.result_status {
        RecursiveResultStatusV1::Converged => {
            last.predecessor_state_commitment == last.state_commitment
        }
        RecursiveResultStatusV1::NonConvergentWithinBound
        | RecursiveResultStatusV1::BlockedIterationBound => {
            trace.iterations.len() as u64 == profile.max_iterations
                && last.predecessor_state_commitment != last.state_commitment
        }
        RecursiveResultStatusV1::BlockedResourceBound
        | RecursiveResultStatusV1::InsufficientEvidence
        | RecursiveResultStatusV1::InvalidTrace => true,
    }
}

pub fn build_recursive_trace(
    projection_commitment: String,
    semantic_environment_commitment: String,
    profile: &RecursiveDerivationProfileV1,
    seed: &RecursiveSeedV1,
    iterations: Vec<RecursiveIterationV1>,
    result_status: RecursiveResultStatusV1,
    unresolved_preserved: bool,
) -> Option<RecursiveDerivationTraceV1> {
    if !profile.structurally_valid()
        || !profile.commitment_matches()
        || !seed.structurally_valid()
        || !seed.commitment_matches()
        || projection_commitment.trim().is_empty()
        || semantic_environment_commitment.trim().is_empty()
        || seed.projection_commitment != projection_commitment
        || seed.semantic_environment_commitment != semantic_environment_commitment
    {
        return None;
    }

    let mut trace = RecursiveDerivationTraceV1 {
        trace_id: format!("trace:{}", seed.seed_id),
        projection_commitment,
        semantic_environment_commitment,
        profile_commitment: profile.profile_commitment.clone(),
        seed_commitment: seed.seed_commitment.clone(),
        initial_state_commitment: seed.initial_state_commitment.clone(),
        iterations,
        result_status,
        unresolved_preserved,
        trace_commitment: String::new(),
        claim_ceiling: D6U_CLAIM_CEILING.into(),
    };

    if !trace.structurally_valid()
        || !result_is_consistent_with_trace(&trace, seed, profile)
    {
        return None;
    }

    trace.trace_commitment = trace.recomputed_commitment();
    Some(trace)
}

pub fn build_recursive_receipt(
    receipt_id: String,
    projection_commitment: String,
    semantic_environment_commitment: String,
    profile: &RecursiveDerivationProfileV1,
    seed: &RecursiveSeedV1,
    trace: &RecursiveDerivationTraceV1,
) -> Option<RecursiveDerivationReceiptV1> {
    if !profile.structurally_valid()
        || !profile.commitment_matches()
        || !seed.structurally_valid()
        || !seed.commitment_matches()
        || !trace.structurally_valid()
        || !trace.commitment_matches()
        || trace.projection_commitment != projection_commitment
        || trace.semantic_environment_commitment != semantic_environment_commitment
        || trace.profile_commitment != profile.profile_commitment
        || trace.seed_commitment != seed.seed_commitment
        || !result_is_consistent_with_trace(trace, seed, profile)
    {
        return None;
    }

    let final_state_commitment = trace
        .iterations
        .last()
        .map(|iteration| iteration.state_commitment.clone())?;

    let mut receipt = RecursiveDerivationReceiptV1 {
        receipt_id,
        projection_commitment,
        semantic_environment_commitment,
        profile_commitment: profile.profile_commitment.clone(),
        seed_commitment: seed.seed_commitment.clone(),
        trace_commitment: trace.trace_commitment.clone(),
        result_status: trace.result_status,
        final_state_commitment,
        iteration_count: trace.iterations.len() as u64,
        claim_ceiling: D6U_CLAIM_CEILING.into(),
        receipt_commitment: String::new(),
    };

    if !receipt.structurally_valid() {
        return None;
    }

    receipt.receipt_commitment = receipt.recomputed_commitment();
    Some(receipt)
}

pub fn recursive_convergence_cannot_raise_claim(
    receipt: &RecursiveDerivationReceiptV1,
) -> bool {
    receipt.structurally_valid()
        && receipt.claim_ceiling == D6U_CLAIM_CEILING
        && matches!(receipt.result_status, RecursiveResultStatusV1::Converged)
}

pub fn trace_commitment_is_d6s_projection_input(
    trace: &RecursiveDerivationTraceV1,
    projection: &crate::canonical_derivation_receipt::QualifiedProjectionV1,
) -> bool {
    trace.structurally_valid()
        && trace.commitment_matches()
        && projection
            .recursive_derivation_trace_commitment
            .as_deref()
            .is_some_and(|commitment| commitment == trace.trace_commitment)
        && projection.semantic_environment_commitment == trace.semantic_environment_commitment
        && projection.derivation_profile_commitment == trace.profile_commitment
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> RecursiveDerivationProfileV1 {
        let mut profile = RecursiveDerivationProfileV1 {
            profile_id: "recursive-1".into(),
            version: "1".into(),
            rule_id: D6U_RULE_ID.into(),
            iteration_order_rule: D6U_ITERATION_ORDER_RULE.into(),
            convergence_rule: D6U_CONVERGENCE_RULE.into(),
            carrier_commitment: "carrier-1".into(),
            max_iterations: 3,
            max_trace_items: 3,
            claim_ceiling: D6U_CLAIM_CEILING.into(),
            profile_commitment: String::new(),
        };
        profile.profile_commitment = profile.recomputed_commitment();
        profile
    }

    fn seed() -> RecursiveSeedV1 {
        let mut seed = RecursiveSeedV1 {
            seed_id: "seed-1".into(),
            projection_commitment: "projection-1".into(),
            semantic_environment_commitment: "environment-1".into(),
            input_commitments: ["input-1".into(), "input-2".into()].into_iter().collect(),
            initial_state_commitment: "state-0".into(),
            seed_commitment: String::new(),
            claim_ceiling: D6U_CLAIM_CEILING.into(),
        };
        seed.seed_commitment = seed.recomputed_commitment();
        seed
    }

    fn iterations_converging() -> Vec<RecursiveIterationV1> {
        vec![
            RecursiveIterationV1 {
                iteration_index: 0,
                predecessor_state_commitment: "state-0".into(),
                state_commitment: "state-1".into(),
                delta_commitment: "delta-1".into(),
                transition_commitment: "transition-1".into(),
            },
            RecursiveIterationV1 {
                iteration_index: 1,
                predecessor_state_commitment: "state-1".into(),
                state_commitment: "state-2".into(),
                delta_commitment: "delta-2".into(),
                transition_commitment: "transition-2".into(),
            },
            RecursiveIterationV1 {
                iteration_index: 2,
                predecessor_state_commitment: "state-2".into(),
                state_commitment: "state-2".into(),
                delta_commitment: "delta-3".into(),
                transition_commitment: "transition-3".into(),
            },
        ]
    }

    fn converged_trace() -> RecursiveDerivationTraceV1 {
        build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            iterations_converging(),
            RecursiveResultStatusV1::Converged,
            false,
        )
        .unwrap()
    }

    #[test]
    fn recursive_profile_is_explicit_and_bounded() {
        assert!(profile().structurally_valid());
        assert!(profile().commitment_matches());
    }

    #[test]
    fn recursive_seed_binds_exact_inputs() {
        assert!(seed().structurally_valid());
        assert!(seed().commitment_matches());
    }

    #[test]
    fn convergence_requires_state_equality() {
        let mut iterations = iterations_converging();
        iterations.last_mut().unwrap().state_commitment = "state-3".into();
        assert!(build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            iterations,
            RecursiveResultStatusV1::Converged,
            false,
        ).is_none());
    }

    #[test]
    fn iteration_order_and_predecessor_chain_are_exact() {
        let mut iterations = iterations_converging();
        iterations.swap(0, 1);
        assert!(build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            iterations,
            RecursiveResultStatusV1::Converged,
            false,
        ).is_none());
    }

    #[test]
    fn bound_is_enforced() {
        let mut iterations = iterations_converging();
        iterations.push(RecursiveIterationV1 {
            iteration_index: 3,
            predecessor_state_commitment: "state-2".into(),
            state_commitment: "state-3".into(),
            delta_commitment: "delta-4".into(),
            transition_commitment: "transition-4".into(),
        });
        assert!(build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            iterations,
            RecursiveResultStatusV1::NonConvergentWithinBound,
            true,
        ).is_none());
    }

    #[test]
    fn non_convergence_requires_full_iteration_bound() {
        let iterations = vec![
            RecursiveIterationV1 {
                iteration_index: 0,
                predecessor_state_commitment: "state-0".into(),
                state_commitment: "state-1".into(),
                delta_commitment: "delta-1".into(),
                transition_commitment: "transition-1".into(),
            },
        ];
        assert!(build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            iterations,
            RecursiveResultStatusV1::NonConvergentWithinBound,
            true,
        ).is_none());
    }

    #[test]
    fn missing_seed_binding_is_blocked() {
        let mut seed = seed();
        seed.projection_commitment = "different".into();
        assert!(build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed,
            iterations_converging(),
            RecursiveResultStatusV1::Converged,
            false,
        ).is_none());
    }

    #[test]
    fn profile_mutation_changes_commitment() {
        let mut p = profile();
        let before = p.profile_commitment.clone();
        p.max_iterations = 4;
        assert_ne!(before, p.recomputed_commitment());
    }

    #[test]
    fn trace_mutation_changes_commitment() {
        let mut trace = converged_trace();
        let before = trace.trace_commitment.clone();
        trace.iterations[1].state_commitment = "changed".into();
        assert_ne!(before, trace.recomputed_commitment());
    }

    #[test]
    fn non_convergence_is_explicit_and_unresolved() {
        let iterations = vec![
            RecursiveIterationV1 {
                iteration_index: 0,
                predecessor_state_commitment: "state-0".into(),
                state_commitment: "state-1".into(),
                delta_commitment: "delta-1".into(),
                transition_commitment: "transition-1".into(),
            },
            RecursiveIterationV1 {
                iteration_index: 1,
                predecessor_state_commitment: "state-1".into(),
                state_commitment: "state-2".into(),
                delta_commitment: "delta-2".into(),
                transition_commitment: "transition-2".into(),
            },
            RecursiveIterationV1 {
                iteration_index: 2,
                predecessor_state_commitment: "state-2".into(),
                state_commitment: "state-3".into(),
                delta_commitment: "delta-3".into(),
                transition_commitment: "transition-3".into(),
            },
        ];
        let trace = build_recursive_trace(
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            iterations,
            RecursiveResultStatusV1::NonConvergentWithinBound,
            true,
        ).unwrap();
        assert!(trace.commitment_matches());
        assert!(trace.unresolved_preserved);
    }

    #[test]
    fn converged_receipt_does_not_authorize() {
        let trace = converged_trace();
        let receipt = build_recursive_receipt(
            "receipt-1".into(),
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            &trace,
        ).unwrap();
        assert!(recursive_convergence_cannot_raise_claim(&receipt));
        assert!(receipt.commitment_matches());
    }

    #[test]
    fn projection_trace_binding_is_exact() {
        let trace = converged_trace();
        let mut projection = crate::canonical_derivation_receipt::QualifiedProjectionV1 {
            projection_id: "projection-1".into(),
            projection_version: "1".into(),
            canonicalization_version: "D6S-CANON-1".into(),
            source_dkg_snapshot_commitment: "dkg-1".into(),
            nodes: Default::default(),
            edges: Default::default(),
            d6p_current_receipt_commitments: Default::default(),
            d6n_context_commitment: None,
            d6o_context_commitment: None,
            semantic_environment_commitment: "environment-1".into(),
            derivation_profile_commitment: "profile-1".into(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
            recursive_derivation_trace_commitment: Some(trace.trace_commitment.clone()),
        };
        assert!(trace_commitment_is_d6s_projection_input(&trace, &projection));
        projection.recursive_derivation_trace_commitment = Some("different".into());
        assert!(!trace_commitment_is_d6s_projection_input(&trace, &projection));
    }

    #[test]
    fn fixpoint_is_not_currentness() {
        let trace = converged_trace();
        let receipt = build_recursive_receipt(
            "receipt-1".into(),
            "projection-1".into(),
            "environment-1".into(),
            &profile(),
            &seed(),
            &trace,
        ).unwrap();
        assert_eq!(receipt.claim_ceiling, D6U_CLAIM_CEILING);
        assert!(!receipt.claim_ceiling.contains("Current"));
    }
}
