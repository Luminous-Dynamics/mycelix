//! COS -> ProductiveLoopV1 refinement contract.
//! The adapter proves only semantic correspondence; it never upgrades the
//! claim ceiling of either domain.

use crate::{Bindings, Decision, Evidence, Origin};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Domain {
    Manufacturing,
    H2Hydroponics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProductiveLoopObligation {
    PlannedInputVsConsumedInput,
    DeclaredWorkVsObservedWork,
    UsefulVsQualifiedOutput,
    SingleSuccessVsGeneralCapability,
    CapabilityVsAvailability,
    HistoricalFailurePreservation,
    ExternalDependencyVsLocalCapability,
    ProductiveClosureVsN2,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RefinementWitness {
    pub domain: Domain,
    pub obligation: ProductiveLoopObligation,
    pub source_evidence_id: &'static str,
    pub target_claim: &'static str,
    pub decision: Decision,
    pub claim_ceiling: &'static str,
}

pub fn refine(
    domain: Domain,
    obligation: ProductiveLoopObligation,
    bindings: &Bindings,
    evidence: &Evidence,
) -> RefinementWitness {
    let (decision, target_claim) = match obligation {
        ProductiveLoopObligation::PlannedInputVsConsumedInput =>
            (if bindings.plan_consumption { Decision::Accepted } else { Decision::Rejected },
             "planned input and observed consumption remain distinct"),
        ProductiveLoopObligation::DeclaredWorkVsObservedWork =>
            (if bindings.assignment_observed_work { Decision::Accepted } else { Decision::Rejected },
             "declared work and observed work require an explicit binding"),
        ProductiveLoopObligation::UsefulVsQualifiedOutput =>
            (if bindings.output_qualification { Decision::Accepted } else { Decision::Rejected },
             "useful output does not itself establish qualification"),
        ProductiveLoopObligation::SingleSuccessVsGeneralCapability =>
            (if bindings.general_capability_evidence { Decision::Accepted } else { Decision::Rejected },
             "one success does not establish general capability"),
        ProductiveLoopObligation::CapabilityVsAvailability =>
            (if bindings.requirement_availability { Decision::Accepted } else { Decision::Rejected },
             "capability and current availability remain distinct"),
        ProductiveLoopObligation::HistoricalFailurePreservation =>
            (if bindings.failure_history_preserved { Decision::Accepted } else { Decision::Rejected },
             "later success does not erase historical failure"),
        ProductiveLoopObligation::ExternalDependencyVsLocalCapability =>
            (if matches!(evidence.origin, Origin::Foreign(_)) && bindings.foreign_recognition {
                Decision::Accepted
             } else if matches!(evidence.origin, Origin::Local) {
                Decision::Rejected
             } else { Decision::Unknown },
             "external origin remains explicit through recognition"),
        ProductiveLoopObligation::ProductiveClosureVsN2 =>
            (Decision::Rejected,
             "productive-loop closure is not an N2 claim"),
    };

    let ceiling = match domain {
        Domain::Manufacturing =>
            "Manufacturing semantic refinement only; no production qualification or safety claim.",
        Domain::H2Hydroponics =>
            "H2 semantic refinement only; no crop efficacy, food safety, marketability, or food-independence claim.",
    };

    RefinementWitness {
        domain,
        obligation,
        source_evidence_id: evidence.id,
        target_claim,
        decision,
        claim_ceiling: ceiling,
    }
}
