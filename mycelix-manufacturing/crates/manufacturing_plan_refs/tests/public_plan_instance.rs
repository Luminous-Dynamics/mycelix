use mycelix_manufacturing_plan_refs::{
    PlanStepAssignmentV1, ProcessPlanInstanceV1, ProcessPlanSubjectRefV1, WorkScopeSubjectRefV1,
};
use mycelix_manufacturing_process_refs::{
    CapabilityProfileRefV1, DigestAlgorithmV1, DigestRefV1, EvidenceSubjectRefV1,
    ExternalSubjectRefV1, ProviderSubjectRefV1, ResourceSubjectRefV1, SiteSubjectRefV1,
};

fn digest(fill: char) -> DigestRefV1 {
    DigestRefV1 {
        algorithm: DigestAlgorithmV1::Blake3,
        hex: std::iter::repeat_n(fill, 64).collect(),
    }
}

fn subject(namespace: &str, id: &str, fill: char) -> ExternalSubjectRefV1 {
    ExternalSubjectRefV1 {
        namespace: namespace.into(),
        subject_id: id.into(),
        semantic_version: "1".into(),
        content_digest: digest(fill),
    }
}

fn assignment(node: &str) -> PlanStepAssignmentV1 {
    PlanStepAssignmentV1 {
        plan_node_id: node.into(),
        provider: ProviderSubjectRefV1(subject("mycelix.provider", "provider-1", 'a')),
        site: SiteSubjectRefV1(subject("mycelix.site", "site-1", 'b')),
        resource: ResourceSubjectRefV1(subject("eng-catalog.resource", "machine-1", 'c')),
        capability_profile: CapabilityProfileRefV1(subject(
            "symthaea.mfg-capability",
            "capability-1",
            'd',
        )),
        evidence_refs: vec![EvidenceSubjectRefV1(subject(
            "mycelix.evidence",
            "qualification-1",
            'e',
        ))],
        assignment_revision: "assignment-r1".into(),
        valid_from_unix_s: 100,
        valid_until_unix_s: 200,
        display_label: Some("Machine assignment".into()),
        scheduler_slot_ref: Some("scheduler:slot:a".into()),
    }
}

fn fixture() -> ProcessPlanInstanceV1 {
    ProcessPlanInstanceV1 {
        canonical_plan: ProcessPlanSubjectRefV1(subject(
            "symthaea.mfg-process-plan",
            "0b3feb49-plan-subject-example",
            'f',
        )),
        work_scope: WorkScopeSubjectRefV1(subject(
            "mycelix.work-order",
            "wo-001-batch-a",
            '1',
        )),
        instance_revision: "instance-r1".into(),
        assignments: vec![assignment("machine-coupon"), assignment("inspect-coupon")],
        display_label: Some("Synthetic coupon instance".into()),
    }
}

#[test]
fn public_plan_instance_validates_and_round_trips() {
    let instance = fixture();
    assert!(instance.validate().is_ok());
    let encoded = serde_json::to_string(&instance).unwrap();
    let decoded: ProcessPlanInstanceV1 = serde_json::from_str(&encoded).unwrap();
    assert_eq!(decoded, instance);
    assert_eq!(
        decoded.engineering_identity().unwrap(),
        instance.engineering_identity().unwrap()
    );
}

#[test]
fn schedule_and_display_changes_do_not_change_engineering_identity() {
    let a = fixture();
    let mut b = a.clone();
    b.display_label = Some("renamed".into());
    b.assignments[0].display_label = Some("renamed assignment".into());
    b.assignments[0].scheduler_slot_ref = Some("scheduler:slot:later".into());
    assert_eq!(
        a.engineering_identity().unwrap(),
        b.engineering_identity().unwrap()
    );
}

#[test]
fn assignment_order_does_not_change_engineering_identity() {
    let a = fixture();
    let mut b = a.clone();
    b.assignments.reverse();
    assert_eq!(
        a.engineering_identity().unwrap(),
        b.engineering_identity().unwrap()
    );
}

#[test]
fn resource_capability_or_evidence_changes_engineering_identity() {
    let a = fixture();

    let mut b = a.clone();
    b.assignments[0].resource = ResourceSubjectRefV1(subject(
        "eng-catalog.resource",
        "machine-2",
        '9',
    ));
    assert_ne!(
        a.engineering_identity().unwrap(),
        b.engineering_identity().unwrap()
    );

    let mut c = a.clone();
    c.assignments[0].capability_profile = CapabilityProfileRefV1(subject(
        "symthaea.mfg-capability",
        "capability-2",
        '8',
    ));
    assert_ne!(
        a.engineering_identity().unwrap(),
        c.engineering_identity().unwrap()
    );

    let mut d = a.clone();
    d.assignments[0].evidence_refs = vec![EvidenceSubjectRefV1(subject(
        "mycelix.evidence",
        "qualification-2",
        '7',
    ))];
    assert_ne!(
        a.engineering_identity().unwrap(),
        d.engineering_identity().unwrap()
    );
}

#[test]
fn duplicate_node_assignment_and_missing_evidence_fail_closed() {
    let mut duplicate = fixture();
    duplicate.assignments.push(assignment("machine-coupon"));
    assert!(duplicate.validate().is_err());

    let mut no_evidence = fixture();
    no_evidence.assignments[0].evidence_refs.clear();
    assert!(no_evidence.validate().is_err());
}

#[test]
fn malformed_nested_external_ref_fails_closed() {
    let mut instance = fixture();
    instance.assignments[0].resource.0.content_digest.hex = "BAD".into();
    assert!(instance.validate().is_err());
}
