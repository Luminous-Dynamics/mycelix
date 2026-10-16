// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Evidence conformance for textile recovery and secondary-fiber claims.
//!
//! This module distinguishes an observed lifecycle event from a stronger claim
//! about the output material. Collection, sorting, reuse, downcycling and
//! fiber-to-fiber recycling are not interchangeable states. Evidence references
//! and Decision::Accepted values must be established by their owning validators;
//! this module checks the conformance contract, not source authenticity or
//! physical process performance.

use serde::{Deserialize, Serialize};

use crate::Decision;

/// Physical or commercial lifecycle outcome claimed for one textile event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TextileLifecycleEvent {
    Collected,
    Sorted,
    ReusedAsIs,
    RepairedAndReturnedToUse,
    Refurbished,
    Remanufactured,
    ComponentHarvested,
    FiberToFiberRecycled,
    Downcycled,
    MaterialRecovered,
    Disposed,
    UnknownDisposition,
}

/// Chain-of-custody method claimed for the input/output relationship.
///
/// These values declare a provenance method, not proof that a physical atom
/// retained item-level identity through blending or recycling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CustodyStrategy {
    IdentityPreserved,
    Segregated,
    ControlledBlending,
    MassBalance,
    BookAndClaim,
}

/// Evidence plane whose status prevents a higher-confidence fiber claim.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TextileEvidencePlane {
    EventObservation,
    SourceLineage,
    Composition,
    Contamination,
    SecondaryGrade,
    ProcessRouteQualification,
    FunctionalProfile,
    FunctionalAcceptance,
    CustodyStrategy,
    MassAttribution,
    OutputLotIdentity,
}

/// One explicit reason why a claim is not admitted at the requested level.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TextileClaimBlocker {
    pub plane: TextileEvidencePlane,
    pub decision: Decision,
    pub reason: String,
}

/// A textile lifecycle event and the evidence currently bound to its claims.
///
/// All mass quantities are integer milligrams. Output attribution is bounded
/// against attributable input plus separately evidenced added material. This
/// intentionally does not pretend that database IDs preserve atom identity.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TextileRecoveryRecord {
    pub record_id: String,
    pub event: TextileLifecycleEvent,
    pub source_lot_refs: Vec<String>,
    pub output_lot_ref: Option<String>,
    pub custody_strategy: Option<CustodyStrategy>,
    pub attributable_input_mg: u64,
    pub attributable_output_mg: u64,
    pub separately_evidenced_added_mg: u64,
    pub event_observation: Decision,
    pub source_lineage: Decision,
    pub composition: Decision,
    pub contamination: Decision,
    pub secondary_grade: Decision,
    pub process_route_qualification: Decision,
    pub functional_profile_ref: Option<String>,
    pub functional_acceptance: Decision,
}

/// Separate decisions for the event record and any fiber-to-fiber claim.
///
/// fiber_to_fiber_eligibility is None when that claim was not made. An
/// accepted event does not, by itself, admit a secondary material grade.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TextileRecoveryAssessment {
    pub record_decision: Decision,
    pub fiber_to_fiber_eligibility: Option<Decision>,
    pub blockers: Vec<TextileClaimBlocker>,
    pub custody_strategy: Option<CustodyStrategy>,
}

/// Evaluate an event record without upgrading weaker lifecycle states.
///
/// The record is rejected or unbound if its own event, input lineage, required
/// output identity, custody method, or material-attribution bound is invalid.
/// A fiber-to-fiber eligibility decision is calculated only for that exact
/// claimed outcome. Missing or conflicting grade evidence can therefore block
/// secondary-fiber admission without erasing the fact that a recovery event was
/// observed.
pub fn assess_textile_recovery(record: &TextileRecoveryRecord) -> TextileRecoveryAssessment {
    let mut blockers = Vec::new();

    let structural_problem = record.record_id.trim().is_empty()
        || record.source_lot_refs.is_empty()
        || record.source_lot_refs.iter().any(|value| value.trim().is_empty())
        || record
            .output_lot_ref
            .as_ref()
            .is_some_and(|value| value.trim().is_empty());

    if structural_problem {
        blockers.push(TextileClaimBlocker {
            plane: TextileEvidencePlane::SourceLineage,
            decision: Decision::Unbound,
            reason: "record ID and non-empty source-lot references are required".into(),
        });
    }

    if record.custody_strategy.is_none() {
        blockers.push(TextileClaimBlocker {
            plane: TextileEvidencePlane::CustodyStrategy,
            decision: Decision::Unbound,
            reason: "chain-of-custody strategy is not declared".into(),
        });
    }

    if record.event_observation != Decision::Accepted {
        blockers.push(TextileClaimBlocker {
            plane: TextileEvidencePlane::EventObservation,
            decision: record.event_observation.clone(),
            reason: "event observation is not currently admitted".into(),
        });
    }

    if record.source_lineage != Decision::Accepted {
        blockers.push(TextileClaimBlocker {
            plane: TextileEvidencePlane::SourceLineage,
            decision: record.source_lineage.clone(),
            reason: "source-lot lineage is not currently admitted".into(),
        });
    }

    if event_requires_output_lot(record.event) && record.output_lot_ref.is_none() {
        blockers.push(TextileClaimBlocker {
            plane: TextileEvidencePlane::OutputLotIdentity,
            decision: Decision::Unbound,
            reason: "this lifecycle outcome requires an output-lot identity".into(),
        });
    }

    if event_requires_material_attribution(record.event) {
        let upper_bound = record
            .attributable_input_mg
            .checked_add(record.separately_evidenced_added_mg);
        if upper_bound.is_none()
            || record.attributable_output_mg > upper_bound.unwrap_or_default()
        {
            blockers.push(TextileClaimBlocker {
                plane: TextileEvidencePlane::MassAttribution,
                decision: Decision::Rejected,
                reason:
                    "attributable output exceeds attributable input plus separately evidenced additions"
                        .into(),
            });
        }
    }

    let record_decision = decision_from_blockers(&blockers).unwrap_or(Decision::Accepted);

    let fiber_to_fiber_eligibility = if record.event == TextileLifecycleEvent::FiberToFiberRecycled
    {
        let mut fiber_blockers = blockers.clone();

        if record.custody_strategy == Some(CustodyStrategy::BookAndClaim) {
            fiber_blockers.push(TextileClaimBlocker {
                plane: TextileEvidencePlane::CustodyStrategy,
                decision: Decision::Rejected,
                reason:
                    "book-and-claim attribution does not establish a physical output-lot fiber-to-fiber pathway"
                        .into(),
            });
        }
        if record.custody_strategy == Some(CustodyStrategy::IdentityPreserved) {
            fiber_blockers.push(TextileClaimBlocker {
                plane: TextileEvidencePlane::CustodyStrategy,
                decision: Decision::Rejected,
                reason:
                    "fiber transformation creates a new output material subject; use a valid batch/segregated lineage method instead of item-identity continuity"
                        .into(),
            });
        }

        require_accepted(
            &mut fiber_blockers,
            TextileEvidencePlane::Composition,
            &record.composition,
            "input fiber composition is not currently admitted",
        );
        require_accepted(
            &mut fiber_blockers,
            TextileEvidencePlane::Contamination,
            &record.contamination,
            "contamination or impurity assessment is not currently admitted",
        );
        require_accepted(
            &mut fiber_blockers,
            TextileEvidencePlane::SecondaryGrade,
            &record.secondary_grade,
            "secondary-fiber grade is not currently admitted",
        );
        require_accepted(
            &mut fiber_blockers,
            TextileEvidencePlane::ProcessRouteQualification,
            &record.process_route_qualification,
            "the recycling process route is not currently qualified",
        );
        require_accepted(
            &mut fiber_blockers,
            TextileEvidencePlane::FunctionalAcceptance,
            &record.functional_acceptance,
            "the output has not passed its declared functional acceptance profile",
        );

        if record
            .functional_profile_ref
            .as_ref()
            .is_none_or(|value| value.trim().is_empty())
        {
            fiber_blockers.push(TextileClaimBlocker {
                plane: TextileEvidencePlane::FunctionalProfile,
                decision: Decision::Unbound,
                reason: "the intended output functional profile is missing".into(),
            });
        }

        let decision = decision_from_blockers(&fiber_blockers).unwrap_or(Decision::Accepted);
        blockers = fiber_blockers;
        Some(decision)
    } else {
        None
    };

    TextileRecoveryAssessment {
        record_decision,
        fiber_to_fiber_eligibility,
        blockers,
        custody_strategy: record.custody_strategy,
    }
}

fn event_requires_output_lot(event: TextileLifecycleEvent) -> bool {
    matches!(
        event,
        TextileLifecycleEvent::Sorted
            | TextileLifecycleEvent::ReusedAsIs
            | TextileLifecycleEvent::RepairedAndReturnedToUse
            | TextileLifecycleEvent::Refurbished
            | TextileLifecycleEvent::Remanufactured
            | TextileLifecycleEvent::ComponentHarvested
            | TextileLifecycleEvent::FiberToFiberRecycled
            | TextileLifecycleEvent::Downcycled
            | TextileLifecycleEvent::MaterialRecovered
    )
}

fn event_requires_material_attribution(event: TextileLifecycleEvent) -> bool {
    matches!(
        event,
        TextileLifecycleEvent::ComponentHarvested
            | TextileLifecycleEvent::FiberToFiberRecycled
            | TextileLifecycleEvent::Downcycled
            | TextileLifecycleEvent::MaterialRecovered
    )
}

fn require_accepted(
    blockers: &mut Vec<TextileClaimBlocker>,
    plane: TextileEvidencePlane,
    decision: &Decision,
    reason: &str,
) {
    if *decision != Decision::Accepted {
        blockers.push(TextileClaimBlocker {
            plane,
            decision: decision.clone(),
            reason: reason.into(),
        });
    }
}

/// Select a stable, conservative disposition while retaining every blocker.
///
/// Conflicts and historical invalidation take precedence over a mere absence
/// of evidence. The blockers vector remains the authoritative explanation
/// when multiple failures coexist.
fn decision_from_blockers(blockers: &[TextileClaimBlocker]) -> Option<Decision> {
    const PRECEDENCE: [Decision; 7] = [
        Decision::Superseded,
        Decision::Conflicting,
        Decision::Stale,
        Decision::Unauthorized,
        Decision::Rejected,
        Decision::Unbound,
        Decision::Unknown,
    ];

    PRECEDENCE.iter().find_map(|wanted| {
        blockers
            .iter()
            .any(|blocker| blocker.decision == *wanted)
            .then(|| wanted.clone())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_record(event: TextileLifecycleEvent) -> TextileRecoveryRecord {
        TextileRecoveryRecord {
            record_id: "REC-001".into(),
            event,
            source_lot_refs: vec!["LOT-IN-001".into()],
            output_lot_ref: Some("LOT-OUT-001".into()),
            custody_strategy: Some(CustodyStrategy::ControlledBlending),
            attributable_input_mg: 1_000_000,
            attributable_output_mg: 850_000,
            separately_evidenced_added_mg: 0,
            event_observation: Decision::Accepted,
            source_lineage: Decision::Accepted,
            composition: Decision::Accepted,
            contamination: Decision::Accepted,
            secondary_grade: Decision::Accepted,
            process_route_qualification: Decision::Accepted,
            functional_profile_ref: Some("TEXTILE-FUNCTION-PROFILE-01".into()),
            functional_acceptance: Decision::Accepted,
        }
    }

    #[test]
    fn collection_event_does_not_imply_fiber_to_fiber_recycling() {
        let mut record = valid_record(TextileLifecycleEvent::Collected);
        record.composition = Decision::Unknown;
        record.secondary_grade = Decision::Unknown;

        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Accepted);
        assert_eq!(assessment.fiber_to_fiber_eligibility, None);
    }

    #[test]
    fn qualified_physical_recycling_claim_can_be_admitted() {
        let record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Accepted);
        assert_eq!(assessment.fiber_to_fiber_eligibility, Some(Decision::Accepted));
        assert!(assessment.blockers.is_empty());
        assert_eq!(
            assessment.custody_strategy,
            Some(CustodyStrategy::ControlledBlending)
        );
    }

    #[test]
    fn unknown_secondary_grade_blocks_eligibility_but_not_observed_event() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.secondary_grade = Decision::Unknown;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Accepted);
        assert_eq!(assessment.fiber_to_fiber_eligibility, Some(Decision::Unknown));
        assert!(assessment.blockers.iter().any(|blocker| {
            blocker.plane == TextileEvidencePlane::SecondaryGrade
                && blocker.decision == Decision::Unknown
        }));
    }

    #[test]
    fn conflicting_composition_is_preserved_not_downgraded_to_unknown() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.composition = Decision::Conflicting;
        record.contamination = Decision::Unknown;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Conflicting)
        );
        assert!(assessment.blockers.iter().any(|blocker| {
            blocker.plane == TextileEvidencePlane::Composition
                && blocker.decision == Decision::Conflicting
        }));
        assert!(assessment.blockers.iter().any(|blocker| {
            blocker.plane == TextileEvidencePlane::Contamination
                && blocker.decision == Decision::Unknown
        }));
    }

    #[test]
    fn book_and_claim_does_not_establish_physical_fiber_to_fiber_output() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.custody_strategy = Some(CustodyStrategy::BookAndClaim);
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Accepted);
        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Rejected)
        );
        assert!(assessment.blockers.iter().any(|blocker| {
            blocker.plane == TextileEvidencePlane::CustodyStrategy
                && blocker.decision == Decision::Rejected
        }));
    }

    #[test]
    fn item_identity_cannot_be_claimed_across_fiber_transformation() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.custody_strategy = Some(CustodyStrategy::IdentityPreserved);
        let assessment = assess_textile_recovery(&record);

        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Rejected)
        );
        assert!(assessment.blockers.iter().any(|blocker| {
            blocker.plane == TextileEvidencePlane::CustodyStrategy
                && blocker.reason.contains("new output material subject")
        }));
    }

    #[test]
    fn attributable_output_cannot_exceed_input_plus_evidenced_additions() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.attributable_output_mg = 1_000_001;
        record.separately_evidenced_added_mg = 0;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Rejected);
        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Rejected)
        );
    }

    #[test]
    fn stale_process_qualification_does_not_admit_secondary_fiber() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.process_route_qualification = Decision::Stale;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Stale)
        );
    }

    #[test]
    fn missing_output_lot_keeps_recycling_claim_unbound() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.output_lot_ref = None;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Unbound);
        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Unbound)
        );
    }

    #[test]
    fn material_attribution_overflow_fails_closed() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.attributable_input_mg = u64::MAX;
        record.separately_evidenced_added_mg = 1;
        record.attributable_output_mg = u64::MAX;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Rejected);
        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Rejected)
        );
    }

    #[test]
    fn missing_custody_method_does_not_silently_pass() {
        let mut record = valid_record(TextileLifecycleEvent::FiberToFiberRecycled);
        record.custody_strategy = None;
        let assessment = assess_textile_recovery(&record);

        assert_eq!(assessment.record_decision, Decision::Unbound);
        assert_eq!(
            assessment.fiber_to_fiber_eligibility,
            Some(Decision::Unbound)
        );
    }
}
