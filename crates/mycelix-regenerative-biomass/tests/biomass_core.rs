// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Independent behavioral matrix for the first executable REGEN-011 biomass core.

use mycelix_core_types::{
    EnvironmentalObservation, EvidenceClass, ExternalEvidenceRef, GeoPoint, Measurement,
    SpatialExtent, TemporalExtent, Uncertainty,
};
use mycelix_regenerative_admission::{EvidenceAdmissionError, EvidenceCandidate};
use mycelix_regenerative_biomass::*;
use mycelix_regenerative_core::{BiomassLotId, RegenerativeSiteId};

fn r(value: &str) -> ExactRef {
    ExactRef::new(value).unwrap()
}

fn reason(value: &str) -> ReasonCode {
    ReasonCode::new(value).unwrap()
}

fn lot() -> BiomassLotId {
    BiomassLotId::new("lot-001").unwrap()
}

fn q(mg: u64, basis: MassBasis) -> BiomassQuantity {
    BiomassQuantity::new(MassMg::new(mg), basis)
}

fn source_evidence() -> ExternalEvidenceRef {
    ExternalEvidenceRef {
        source_system: "regen-011-fixture".into(),
        resource_id: "fixture/source/1".into(),
        content_digest: Some("sha256:fixture-source".into()),
        retrieved_at: Some(1_789_100_000),
        license: None,
    }
}

fn observation(id: &str, class: EvidenceClass) -> EnvironmentalObservation {
    EnvironmentalObservation::new(
        id,
        "biomass_mass",
        class,
        Some(Measurement::new(1.0, "kg").unwrap()),
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::instant(1_789_100_000),
        Uncertainty::Unspecified,
        vec![source_evidence()],
    )
    .unwrap()
}

fn as_received_binding() -> BiomassEvidenceBinding {
    BiomassEvidenceBinding::new(
        BiomassEvidenceRole::AsReceivedMass,
        "regen:011:mass:observed",
        "biomass_mass",
        Some(EvidenceClass::Observed),
    )
    .unwrap()
}

fn material_partition() -> BiomassMaterialPartition {
    BiomassMaterialPartition::new(
        q(1_000, MassBasis::AsReceived),
        q(200, MassBasis::AsReceived),
        q(700, MassBasis::AsReceived),
        q(100, MassBasis::AsReceived),
    )
    .unwrap()
}

fn rights() -> RightsAndCustodyRefs {
    RightsAndCustodyRefs::new(
        vec![r("custody/1")],
        vec![r("removal/1")],
        vec![r("processing/1")],
        vec![],
        vec![],
    )
    .unwrap()
}

fn eligible_feedstock(assessed_mg: u64, ecology_cap_mg: u64) -> EligibleFeedstock {
    EligibleFeedstock::new(
        r("assessment/1"),
        lot(),
        r("snapshot/1"),
        r("process/pyrolysis-profile-1"),
        r("evidence-snapshot/1"),
        r("material-partition/1"),
        material_partition(),
        EcologicalConstraintOutcome::Eligible(EcologicalEligibility::new(
            r("material-partition/1"),
            r("ecology/profile-1"),
            r("evidence-snapshot/1"),
            q(ecology_cap_mg, MassBasis::AsReceived),
        )),
        rights(),
        q(assessed_mg, MassBasis::AsReceived),
        vec![r("prerequisite/quality-profile-1")],
    )
    .unwrap()
}

fn reservation(reference: &str, mg: u64) -> ProcessInputReservation {
    ProcessInputReservation::new(
        r(reference),
        lot(),
        r("assessment/1"),
        r("snapshot/1"),
        r("process/pyrolysis-profile-1"),
        q(mg, MassBasis::AsReceived),
    )
    .unwrap()
}

#[test]
fn exact_u64_mass_arithmetic_is_checked() {
    assert_eq!(MassMg::new(2).checked_add(MassMg::new(3)).unwrap().get(), 5);
    assert_eq!(MassMg::new(5).checked_sub(MassMg::new(3)).unwrap().get(), 2);
    assert_eq!(MassMg::new(u64::MAX).checked_add(MassMg::new(1)), Err(BiomassError::MassOverflow));
    assert_eq!(MassMg::new(0).checked_sub(MassMg::new(1)), Err(BiomassError::MassUnderflow));
}

#[test]
fn incompatible_mass_bases_do_not_mix() {
    let as_received = q(100, MassBasis::AsReceived);
    let dry = q(100, MassBasis::DryMatterEquivalent);
    assert!(matches!(
        as_received.checked_add(dry),
        Err(BiomassError::MassBasisMismatch { .. })
    ));
}

#[test]
fn observed_pef_evidence_passes_through_shared_admission() {
    let binding = as_received_binding();
    let obs = observation("regen:011:mass:observed", EvidenceClass::Observed);
    let admitted = binding.admit(EvidenceCandidate::Raw(&obs)).unwrap();
    assert_eq!(admitted.id, "regen:011:mass:observed");
}

#[test]
fn bare_computed_pef_evidence_is_rejected_by_shared_admission() {
    let binding = BiomassEvidenceBinding::new(
        BiomassEvidenceRole::AsReceivedMass,
        "regen:011:mass:derived",
        "biomass_mass",
        Some(EvidenceClass::Derived),
    )
    .unwrap();
    let obs = observation("regen:011:mass:derived", EvidenceClass::Derived);
    assert!(matches!(
        binding.admit(EvidenceCandidate::Raw(&obs)),
        Err(BiomassError::EvidenceAdmission(
            EvidenceAdmissionError::ComputedEvidenceRequiresLineage { .. }
        ))
    ));
}

#[test]
fn evidence_substitution_fails_before_biomass_semantics() {
    let binding = as_received_binding();
    let obs = observation("regen:011:other", EvidenceClass::Observed);
    assert!(matches!(
        binding.admit(EvidenceCandidate::Raw(&obs)),
        Err(BiomassError::EvidenceAdmission(
            EvidenceAdmissionError::ObservationIdMismatch { .. }
        ))
    ));
}

#[test]
fn mass_assertion_requires_same_basis_mass_evidence_role() {
    let dry_binding = BiomassEvidenceBinding::new(
        BiomassEvidenceRole::DryMatterMass,
        "regen:011:dry",
        "biomass_mass",
        Some(EvidenceClass::Derived),
    )
    .unwrap();
    assert_eq!(
        BiomassMassAssertion::new(
            q(1_000, MassBasis::AsReceived),
            r("derivation/1"),
            vec![dry_binding],
        ),
        Err(BiomassError::MissingBasisMassEvidence(MassBasis::AsReceived))
    );
}

#[test]
fn duplicate_exact_evidence_binding_is_rejected() {
    let binding = as_received_binding();
    assert_eq!(
        BiomassMassAssertion::new(
            q(1_000, MassBasis::AsReceived),
            r("derivation/1"),
            vec![binding.clone(), binding],
        ),
        Err(BiomassError::DuplicateEvidenceBinding)
    );
}

#[test]
fn exact_mass_assertion_does_not_claim_implicit_pef_conversion() {
    let assertion = BiomassMassAssertion::new(
        q(1_000_000, MassBasis::AsReceived),
        r("normalization/profile-1/run-9"),
        vec![as_received_binding()],
    )
    .unwrap();
    assert_eq!(assertion.quantity().mass().get(), 1_000_000);
    assert_eq!(assertion.derivation_ref().as_str(), "normalization/profile-1/run-9");
}

#[test]
fn lot_identity_is_separate_from_state_snapshot() {
    let profile = BiomassLotProfile::new(
        lot(),
        RegenerativeSiteId::new("site-001").unwrap(),
        r("lot-formation/1"),
    );
    let assertion = BiomassMassAssertion::new(
        q(1_000, MassBasis::AsReceived),
        r("derivation/1"),
        vec![as_received_binding()],
    )
    .unwrap();
    let snapshot = BiomassStateSnapshot::new(lot(), r("snapshot/1"), assertion);
    assert_eq!(profile.lot_id(), snapshot.lot_id());
    assert_ne!(profile.formation_ref().as_str(), snapshot.snapshot_ref().as_str());
}

#[test]
fn material_partition_requires_explicit_residual_and_exact_closure() {
    let partition = material_partition();
    assert_eq!(partition.unresolved_residual().mass().get(), 100);
    assert!(matches!(
        BiomassMaterialPartition::new(
            q(1_000, MassBasis::AsReceived),
            q(200, MassBasis::AsReceived),
            q(700, MassBasis::AsReceived),
            q(99, MassBasis::AsReceived),
        ),
        Err(BiomassError::MaterialPartitionDoesNotClose { .. })
    ));
}

#[test]
fn material_partition_rejects_mixed_bases() {
    assert!(matches!(
        BiomassMaterialPartition::new(
            q(1_000, MassBasis::AsReceived),
            q(200, MassBasis::DryMatterEquivalent),
            q(700, MassBasis::AsReceived),
            q(100, MassBasis::AsReceived),
        ),
        Err(BiomassError::MassBasisMismatch { .. })
    ));
}

#[test]
fn unresolved_material_allocation_requires_reasons_and_carries_no_partition() {
    assert_eq!(
        MaterialAllocationOutcome::unresolved(vec![]),
        Err(BiomassError::EmptyReasonSet)
    );
    let unresolved = MaterialAllocationOutcome::unresolved(vec![reason("field-data-missing")]).unwrap();
    assert!(matches!(unresolved, MaterialAllocationOutcome::Unresolved { .. }));
}

#[test]
fn ecology_is_separate_from_mass_partition() {
    let partition = material_partition();
    let unresolved = EcologicalConstraintOutcome::unresolved(vec![reason("habitat-unresolved")]).unwrap();
    assert_eq!(partition.allocable().mass().get(), 700);
    assert!(matches!(unresolved, EcologicalConstraintOutcome::Unresolved { .. }));
}

#[test]
fn rights_resolution_requires_custody_removal_and_processing_references() {
    assert!(matches!(
        RightsAndCustodyRefs::new(vec![], vec![r("removal/1")], vec![r("processing/1")], vec![], vec![]),
        Err(BiomassError::EmptyReferenceSet("custody_refs"))
    ));
    assert!(matches!(
        RightsAndCustodyRefs::new(vec![r("custody/1")], vec![], vec![r("processing/1")], vec![], vec![]),
        Err(BiomassError::EmptyReferenceSet("removal_refs"))
    ));
}

#[test]
fn duplicate_rights_reference_within_one_role_is_rejected() {
    assert!(matches!(
        RightsAndCustodyRefs::new(
            vec![r("custody/1"), r("custody/1")],
            vec![r("removal/1")],
            vec![r("processing/1")],
            vec![],
            vec![],
        ),
        Err(BiomassError::DuplicateReference { field: "custody_refs", .. })
    ));
}

#[test]
fn eligible_feedstock_requires_positive_mass_below_both_material_and_ecology_caps() {
    assert_eq!(eligible_feedstock(500, 600).assessed().mass().get(), 500);
    assert!(matches!(
        EligibleFeedstock::new(
            r("assessment/1"),
            lot(),
            r("snapshot/1"),
            r("process/pyrolysis-profile-1"),
            r("evidence-snapshot/1"),
            r("material-partition/1"),
            material_partition(),
            EcologicalConstraintOutcome::Eligible(EcologicalEligibility::new(
                r("material-partition/1"),
                r("ecology/profile-1"),
                r("evidence-snapshot/1"),
                q(600, MassBasis::AsReceived),
            )),
            rights(),
            q(701, MassBasis::AsReceived),
            vec![],
        ),
        Err(BiomassError::EligibleMassExceedsMaterialAllocation)
    ));
    assert!(matches!(
        EligibleFeedstock::new(
            r("assessment/1"),
            lot(),
            r("snapshot/1"),
            r("process/pyrolysis-profile-1"),
            r("evidence-snapshot/1"),
            r("material-partition/1"),
            material_partition(),
            EcologicalConstraintOutcome::Eligible(EcologicalEligibility::new(
                r("material-partition/1"),
                r("ecology/profile-1"),
                r("evidence-snapshot/1"),
                q(400, MassBasis::AsReceived),
            )),
            rights(),
            q(500, MassBasis::AsReceived),
            vec![],
        ),
        Err(BiomassError::EligibleMassExceedsEcologyCap)
    ));
}

#[test]
fn unresolved_or_ineligible_ecology_cannot_create_eligible_feedstock() {
    for ecology in [
        EcologicalConstraintOutcome::unresolved(vec![reason("ecology-unresolved")]).unwrap(),
        EcologicalConstraintOutcome::ineligible(vec![reason("habitat-gate-failed")]).unwrap(),
    ] {
        assert!(matches!(
            EligibleFeedstock::new(
                r("assessment/1"),
                lot(),
                r("snapshot/1"),
                r("process/pyrolysis-profile-1"),
                r("evidence-snapshot/1"),
                r("material-partition/1"),
                material_partition(),
                ecology,
                rights(),
                q(500, MassBasis::AsReceived),
                vec![],
            ),
            Err(BiomassError::EcologyNotEligible)
        ));
    }
}

#[test]
fn ecology_must_bind_the_exact_material_partition_reference() {
    assert!(matches!(
        EligibleFeedstock::new(
            r("assessment/1"),
            lot(),
            r("snapshot/1"),
            r("process/pyrolysis-profile-1"),
            r("evidence-snapshot/1"),
            r("material-partition/1"),
            material_partition(),
            EcologicalConstraintOutcome::Eligible(EcologicalEligibility::new(
                r("material-partition/other"),
                r("ecology/profile-1"),
                r("evidence-snapshot/1"),
                q(600, MassBasis::AsReceived),
            )),
            rights(),
            q(500, MassBasis::AsReceived),
            vec![],
        ),
        Err(BiomassError::EcologyPartitionReferenceMismatch)
    ));
}

#[test]
fn only_eligible_feedstock_can_mint_reservation_capacity() {
    let eligible = FeedstockAssessment::Eligible(Box::new(eligible_feedstock(500, 600)));
    assert_eq!(eligible.reservation_capacity().unwrap().maximum().mass().get(), 500);

    let failure = FeedstockFailure::new(
        r("assessment/2"),
        lot(),
        r("snapshot/1"),
        r("process/pyrolysis-profile-1"),
        vec![reason("rights-unresolved")],
    )
    .unwrap();
    assert_eq!(
        FeedstockAssessment::Unresolved(failure).reservation_capacity(),
        Err(BiomassError::FeedstockNotEligible)
    );
}

#[test]
fn reservation_capacity_is_scoped_to_exact_assessment_snapshot_profile_and_basis() {
    let capacity = FeedstockAssessment::Eligible(Box::new(eligible_feedstock(500, 600)))
        .reservation_capacity()
        .unwrap();
    let wrong_snapshot = ProcessInputReservation::new(
        r("reservation/wrong-snapshot"),
        lot(),
        r("assessment/1"),
        r("snapshot/other"),
        r("process/pyrolysis-profile-1"),
        q(100, MassBasis::AsReceived),
    )
    .unwrap();
    assert_eq!(
        evaluate_reservations(&capacity, &[wrong_snapshot]),
        Err(BiomassError::ReservationScopeMismatch("state_snapshot_ref"))
    );
}

#[test]
fn reservation_arithmetic_allows_partial_and_exact_capacity() {
    let capacity = FeedstockAssessment::Eligible(Box::new(eligible_feedstock(500, 600)))
        .reservation_capacity()
        .unwrap();
    let summary = evaluate_reservations(
        &capacity,
        &[reservation("reservation/1", 200), reservation("reservation/2", 300)],
    )
    .unwrap();
    assert_eq!(summary.reserved().mass().get(), 500);
    assert_eq!(summary.remaining().mass().get(), 0);
}

#[test]
fn reservation_overbooking_is_rejected() {
    let capacity = FeedstockAssessment::Eligible(Box::new(eligible_feedstock(500, 600)))
        .reservation_capacity()
        .unwrap();
    assert_eq!(
        evaluate_reservations(
            &capacity,
            &[reservation("reservation/1", 300), reservation("reservation/2", 201)],
        ),
        Err(BiomassError::ReservationOverbooked)
    );
}

#[test]
fn duplicate_reservation_reference_is_rejected() {
    let capacity = FeedstockAssessment::Eligible(Box::new(eligible_feedstock(500, 600)))
        .reservation_capacity()
        .unwrap();
    assert!(matches!(
        evaluate_reservations(
            &capacity,
            &[reservation("reservation/1", 100), reservation("reservation/1", 100)],
        ),
        Err(BiomassError::DuplicateReservationReference(_))
    ));
}

#[test]
fn zero_reservation_is_rejected_before_accounting() {
    assert_eq!(
        ProcessInputReservation::new(
            r("reservation/zero"),
            lot(),
            r("assessment/1"),
            r("snapshot/1"),
            r("process/pyrolysis-profile-1"),
            q(0, MassBasis::AsReceived),
        ),
        Err(BiomassError::ZeroReservation)
    );
}

#[test]
fn shared_evidence_admission_does_not_manufacture_feedstock_eligibility() {
    let binding = as_received_binding();
    let obs = observation("regen:011:mass:observed", EvidenceClass::Observed);
    binding.admit(EvidenceCandidate::Raw(&obs)).unwrap();

    let failure = FeedstockFailure::new(
        r("assessment/unresolved"),
        lot(),
        r("snapshot/1"),
        r("process/pyrolysis-profile-1"),
        vec![reason("ecology-unresolved"), reason("rights-unresolved")],
    )
    .unwrap();
    let assessment = FeedstockAssessment::Unresolved(failure);
    assert_eq!(assessment.reservation_capacity(), Err(BiomassError::FeedstockNotEligible));
}

#[test]
fn ecology_must_bind_the_exact_evidence_snapshot() {
    assert!(matches!(
        EligibleFeedstock::new(
            r("assessment/snapshot-mismatch"),
            lot(),
            r("snapshot/1"),
            r("process/pyrolysis-profile-1"),
            r("evidence-snapshot/1"),
            r("material-partition/1"),
            material_partition(),
            EcologicalConstraintOutcome::Eligible(EcologicalEligibility::new(
                r("material-partition/1"),
                r("ecology/profile-1"),
                r("evidence-snapshot/other"),
                q(600, MassBasis::AsReceived),
            )),
            rights(),
            q(500, MassBasis::AsReceived),
            vec![],
        ),
        Err(BiomassError::EcologyEvidenceSnapshotMismatch)
    ));
}
