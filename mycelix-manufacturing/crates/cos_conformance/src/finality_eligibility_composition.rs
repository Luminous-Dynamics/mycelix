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
