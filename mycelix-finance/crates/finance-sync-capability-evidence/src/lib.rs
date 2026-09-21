#![forbid(unsafe_code)]

//! FIN-SYNC-003B0: structural rail-capability evidence.
//!
//! This crate binds bounded evidence to one exact FIN-SYNC-003A static
//! capability profile. It deliberately does not establish live currentness,
//! credential validity, plan admission, financial authority, dispatch,
//! settlement, or legal/commercial finality.

mod assemble;
mod canonical;
mod model;

pub use assemble::build_rail_capability_evidence_bundle_v1;
pub use model::{
    CapabilityEvidenceDimensionV1, ContradictedClaimSubjectV1, EvidenceBundleDispositionV1,
    EvidenceError, RailCapabilityEvidenceBundleInputV1, RailCapabilityEvidenceBundleV1,
    RailCapabilityEvidenceItemInputV1, SourceChronologyEvidenceV1,
    StructuralEvidenceClaimV1, StructuralEvidenceRecordV1, MAX_CONFLICT_IDENTITIES,
    MAX_CONTRADICTED_SUBJECTS, MAX_EVIDENCE_ITEMS, MAX_EVIDENCE_ITEMS_PER_DIMENSION,
};

#[cfg(test)]
mod tests;
