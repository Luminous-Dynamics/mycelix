use cos_conformance::productive_loop::{refine, Domain, ProductiveLoopObligation};
use cos_conformance::{
    bind_assignment_to_observed_work, bind_general_capability, bind_plan_to_consumption,
    bind_requirement_to_availability, preserve_failure_history, qualify_output,
    recognize_foreign_evidence, Bindings, Decision, Evidence,
};

fn current() -> Evidence {
    Evidence::current_local("pl-ref-001", 100)
}

#[test]
fn manufacturing_and_h2_share_the_same_negative_boundaries() {
    let domains = [Domain::Manufacturing, Domain::H2Hydroponics];
    let obligations = [
        ProductiveLoopObligation::PlannedInputVsConsumedInput,
        ProductiveLoopObligation::DeclaredWorkVsObservedWork,
        ProductiveLoopObligation::UsefulVsQualifiedOutput,
        ProductiveLoopObligation::SingleSuccessVsGeneralCapability,
        ProductiveLoopObligation::CapabilityVsAvailability,
        ProductiveLoopObligation::HistoricalFailurePreservation,
        ProductiveLoopObligation::ProductiveClosureVsN2,
    ];

    for domain in domains {
        for obligation in obligations {
            let witness = refine(domain, obligation, &Bindings::default(), &current());
            assert_ne!(
                witness.decision,
                Decision::Accepted,
                "{domain:?}/{obligation:?} collapsed without evidence"
            );
        }
    }
}

#[test]
fn manufacturing_positive_refinement_requires_explicit_bindings() {
    let e = current();
    let mut b = Bindings::default();
    bind_plan_to_consumption(&mut b);
    bind_assignment_to_observed_work(&mut b);
    qualify_output(&mut b);
    bind_general_capability(&mut b);
    bind_requirement_to_availability(&mut b);
    preserve_failure_history(&mut b);

    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::PlannedInputVsConsumedInput, &b, &e).decision,
        Decision::Accepted
    );
    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::DeclaredWorkVsObservedWork, &b, &e).decision,
        Decision::Accepted
    );
    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::UsefulVsQualifiedOutput, &b, &e).decision,
        Decision::Accepted
    );
    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::SingleSuccessVsGeneralCapability, &b, &e).decision,
        Decision::Accepted
    );
    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::CapabilityVsAvailability, &b, &e).decision,
        Decision::Accepted
    );
    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::HistoricalFailurePreservation, &b, &e).decision,
        Decision::Accepted
    );
    assert_eq!(
        refine(Domain::Manufacturing, ProductiveLoopObligation::ProductiveClosureVsN2, &b, &e).decision,
        Decision::Rejected
    );
}

#[test]
fn h2_positive_refinement_keeps_h2_claim_ceiling() {
    let e = current();
    let mut b = Bindings::default();
    bind_plan_to_consumption(&mut b);
    qualify_output(&mut b);

    let witness = refine(
        Domain::H2Hydroponics,
        ProductiveLoopObligation::UsefulVsQualifiedOutput,
        &b,
        &e,
    );

    assert_eq!(witness.decision, Decision::Accepted);
    assert!(witness.claim_ceiling.contains("food safety"));
}

#[test]
fn external_dependency_stays_external_in_both_domains() {
    let e = Evidence::current_foreign("import-001", "node-b", 100);
    let mut b = Bindings::default();
    recognize_foreign_evidence(&mut b);

    for domain in [Domain::Manufacturing, Domain::H2Hydroponics] {
        let witness = refine(
            domain,
            ProductiveLoopObligation::ExternalDependencyVsLocalCapability,
            &b,
            &e,
        );
        assert_eq!(witness.decision, Decision::Accepted);
        assert_eq!(e.origin, cos_conformance::Origin::Foreign("node-b".into()));
    }
}
