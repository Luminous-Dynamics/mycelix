use mycelix_circularity::{
    ChainOfCustodyStrategyV1, CircularityAuthorityCeilingV1, CircularityDispositionV1,
    CircularityError, CircularityEventBindingV1, CircularityEventKindV1,
    CircularityEventProfileV1,
};
use mycelix_economic_reality_graph::{
    EconomicMaterialEventV1, EconomicResourceSubjectV1, ErgAuthorityCeilingV1,
    EventResourceEdgeV1, EventResourceRoleV1, ExactErgSubjectRefV1,
};

fn exact(namespace: &str, id: &str, fill: char) -> ExactErgSubjectRefV1 {
    ExactErgSubjectRefV1 {
        namespace: namespace.into(),
        subject_id: id.into(),
        semantic_version: "1".into(),
        content_blake3: std::iter::repeat_n(fill, 64).collect(),
    }
}

fn resource(profile_id: &str, state_id: &str, fill: char) -> EconomicResourceSubjectV1 {
    EconomicResourceSubjectV1 {
        semantic_version: "1".into(),
        resource_profile_ref: exact("mycelix.erg-profile", profile_id, fill),
        external_state_refs: vec![exact("symthaea.material-state", state_id, fill)],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some(state_id.into()),
    }
}

fn generic_event(
    profile_id: &str,
    input: ExactErgSubjectRefV1,
    output: ExactErgSubjectRefV1,
    process_ref: &str,
    evidence_ref: &str,
    fill: char,
) -> EconomicMaterialEventV1 {
    EconomicMaterialEventV1 {
        semantic_version: "1".into(),
        event_profile_ref: exact("mycelix.erg-event-profile", profile_id, fill),
        inputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: input,
        }],
        outputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: output,
        }],
        party_refs: vec![exact("mycelix.party", "synthetic-provider", 'a')],
        site_refs: vec![exact("mycelix.site", "synthetic-site", 'b')],
        effective_time_ref: exact("mycelix.time", profile_id, 'c'),
        process_or_agreement_refs: vec![exact("symthaea.mfg-plan", process_ref, 'd')],
        evidence_refs: vec![exact("field.evidence", evidence_ref, 'e')],
        revision_links: vec![],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some(profile_id.into()),
    }
}

fn repair_profile() -> CircularityEventProfileV1 {
    CircularityEventProfileV1 {
        semantic_version: "1".into(),
        event_kind: CircularityEventKindV1::Repair,
        disposition: CircularityDispositionV1::RepairedAndReturnedToUse,
        chain_of_custody: Some(ChainOfCustodyStrategyV1::IdentityPreserved),
        material_flow_account_ref: None,
        external_requirement_refs: vec![exact(
            "symthaea.lifecycle-design",
            "serviceable-product-profile",
            '1',
        )],
        authority_ceiling: CircularityAuthorityCeilingV1::LifecycleEventProfileAndReferencesOnly,
        display_label: Some("Repair lifecycle profile".into()),
    }
}

fn recovery_profile() -> CircularityEventProfileV1 {
    CircularityEventProfileV1 {
        semantic_version: "1".into(),
        event_kind: CircularityEventKindV1::RecoverMaterial,
        disposition: CircularityDispositionV1::MaterialRecovered,
        chain_of_custody: Some(ChainOfCustodyStrategyV1::MassBalance),
        material_flow_account_ref: Some(exact(
            "mycelix.material-flow",
            "synthetic-recovery-account",
            '2',
        )),
        external_requirement_refs: vec![
            exact(
                "symthaea.lifecycle-design",
                "serviceable-product-profile",
                '1',
            ),
            exact("symthaea.mfg-plan", "recovery-process-plan", '3'),
        ],
        authority_ceiling: CircularityAuthorityCeilingV1::LifecycleEventProfileAndReferencesOnly,
        display_label: Some("Material recovery profile".into()),
    }
}

fn circular_event(
    profile: &CircularityEventProfileV1,
    input: ExactErgSubjectRefV1,
    output: ExactErgSubjectRefV1,
    process_ref: &str,
    evidence_ref: &str,
) -> EconomicMaterialEventV1 {
    EconomicMaterialEventV1 {
        semantic_version: "1".into(),
        event_profile_ref: profile.exact_ref().unwrap(),
        inputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: input,
        }],
        outputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: output,
        }],
        party_refs: vec![exact("mycelix.party", "synthetic-circularity-provider", '4')],
        site_refs: vec![exact("mycelix.site", "synthetic-circularity-site", '5')],
        effective_time_ref: exact("mycelix.time", process_ref, '6'),
        process_or_agreement_refs: vec![exact("symthaea.mfg-plan", process_ref, '7')],
        evidence_refs: vec![exact("field.evidence", evidence_ref, '8')],
        revision_links: vec![],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some(process_ref.into()),
    }
}

struct SyntheticClosedLoop {
    primary: EconomicResourceSubjectV1,
    product_v1: EconomicResourceSubjectV1,
    manufacture_v1: EconomicMaterialEventV1,
    repaired_product: EconomicResourceSubjectV1,
    repair: CircularityEventBindingV1,
    recovered_material: EconomicResourceSubjectV1,
    recovery: CircularityEventBindingV1,
    secondary_feedstock: EconomicResourceSubjectV1,
    characterization: EconomicMaterialEventV1,
    product_v2: EconomicResourceSubjectV1,
    manufacture_v2: EconomicMaterialEventV1,
}

fn closed_loop() -> SyntheticClosedLoop {
    let primary = resource("feedstock", "primary-feedstock-state", '9');
    let product_v1 = resource("manufactured-article", "product-v1-as-built", 'a');
    let manufacture_v1 = generic_event(
        "manufacture",
        primary.exact_ref().unwrap(),
        product_v1.exact_ref().unwrap(),
        "manufacturing-plan-v1",
        "synthetic-manufacturing-receipt-v1",
        'b',
    );

    let repaired_product = resource("manufactured-article", "product-v1-repaired", 'c');
    let repair_profile = repair_profile();
    let repair = CircularityEventBindingV1 {
        event: circular_event(
            &repair_profile,
            product_v1.exact_ref().unwrap(),
            repaired_product.exact_ref().unwrap(),
            "repair-process-plan",
            "synthetic-post-repair-inspection",
        ),
        profile: repair_profile,
    };

    let recovered_material = resource("recovered-material", "recovered-material-unqualified", 'd');
    let recovery_profile = recovery_profile();
    let recovery = CircularityEventBindingV1 {
        event: circular_event(
            &recovery_profile,
            repaired_product.exact_ref().unwrap(),
            recovered_material.exact_ref().unwrap(),
            "recovery-process-plan",
            "synthetic-recovery-observation",
        ),
        profile: recovery_profile,
    };

    // Recovery does not become specification-grade secondary feedstock automatically. A separate
    // characterization/qualification event creates a new state/resource subject.
    let secondary_feedstock = resource(
        "secondary-feedstock",
        "secondary-feedstock-characterized",
        'e',
    );
    let characterization = generic_event(
        "material-characterization-and-qualification",
        recovered_material.exact_ref().unwrap(),
        secondary_feedstock.exact_ref().unwrap(),
        "secondary-feedstock-characterization-plan",
        "synthetic-characterization-receipt",
        'f',
    );

    let product_v2 = resource("manufactured-article", "product-v2-as-built", '1');
    let manufacture_v2 = generic_event(
        "manufacture",
        secondary_feedstock.exact_ref().unwrap(),
        product_v2.exact_ref().unwrap(),
        "manufacturing-plan-v2",
        "synthetic-manufacturing-receipt-v2",
        '2',
    );

    SyntheticClosedLoop {
        primary,
        product_v1,
        manufacture_v1,
        repaired_product,
        repair,
        recovered_material,
        recovery,
        secondary_feedstock,
        characterization,
        product_v2,
        manufacture_v2,
    }
}

#[test]
fn synthetic_closed_loop_preserves_distinct_lineage_and_claim_ceiling() {
    let loop_ = closed_loop();

    assert!(loop_.manufacture_v1.validate().is_ok());
    assert!(loop_.repair.validate().is_ok());
    assert!(loop_.recovery.validate().is_ok());
    assert!(loop_.characterization.validate().is_ok());
    assert!(loop_.manufacture_v2.validate().is_ok());

    let resource_ids = [
        loop_.primary.resource_id().unwrap().0,
        loop_.product_v1.resource_id().unwrap().0,
        loop_.repaired_product.resource_id().unwrap().0,
        loop_.recovered_material.resource_id().unwrap().0,
        loop_.secondary_feedstock.resource_id().unwrap().0,
        loop_.product_v2.resource_id().unwrap().0,
    ];
    let unique: std::collections::BTreeSet<_> = resource_ids.iter().collect();
    assert_eq!(unique.len(), resource_ids.len());

    // A complete semantic loop does not establish environmental benefit or compliance.
    assert!(!loop_.repair.claims_environmental_benefit());
    assert!(!loop_.repair.claims_regulatory_compliance());
    assert!(!loop_.recovery.claims_environmental_benefit());
    assert!(!loop_.recovery.claims_regulatory_compliance());
}

#[test]
fn recovered_material_is_not_secondary_feedstock_without_separate_state_transition() {
    let loop_ = closed_loop();
    assert_ne!(
        loop_.recovered_material.resource_id().unwrap(),
        loop_.secondary_feedstock.resource_id().unwrap()
    );
    assert_eq!(
        loop_.characterization.inputs[0].resource_ref,
        loop_.recovered_material.exact_ref().unwrap()
    );
    assert_eq!(
        loop_.characterization.outputs[0].resource_ref,
        loop_.secondary_feedstock.exact_ref().unwrap()
    );
}

#[test]
fn recovery_without_material_flow_account_fails_closed() {
    let mut profile = recovery_profile();
    profile.material_flow_account_ref = None;
    assert_eq!(
        profile.validate(),
        Err(CircularityError::MissingMaterialFlowAccount)
    );
}

#[test]
fn recovery_without_chain_of_custody_fails_closed() {
    let mut profile = recovery_profile();
    profile.chain_of_custody = None;
    assert_eq!(
        profile.validate(),
        Err(CircularityError::MissingChainOfCustody)
    );
}

#[test]
fn repair_event_cannot_be_relabelled_as_recovery_profile() {
    let loop_ = closed_loop();
    let wrong = CircularityEventBindingV1 {
        profile: recovery_profile(),
        event: loop_.repair.event.clone(),
    };
    assert_eq!(wrong.validate(), Err(CircularityError::EventProfileMismatch));
}

#[test]
fn later_lifecycle_events_do_not_rewrite_prior_event_identity() {
    let loop_ = closed_loop();
    let first_id_before = loop_.manufacture_v1.event_id().unwrap();
    let repair_id_before = loop_.repair.event.event_id().unwrap();

    let _later_ids = [
        loop_.recovery.event.event_id().unwrap(),
        loop_.characterization.event_id().unwrap(),
        loop_.manufacture_v2.event_id().unwrap(),
    ];

    assert_eq!(first_id_before, loop_.manufacture_v1.event_id().unwrap());
    assert_eq!(repair_id_before, loop_.repair.event.event_id().unwrap());
}

#[test]
fn display_labels_do_not_change_resource_or_event_identity() {
    let loop_ = closed_loop();
    let mut product = loop_.product_v1.clone();
    let product_id = product.resource_id().unwrap();
    product.display_label = Some("renamed display label".into());
    assert_eq!(product_id, product.resource_id().unwrap());

    let mut event = loop_.manufacture_v1.clone();
    let event_id = event.event_id().unwrap();
    event.display_label = Some("renamed event".into());
    assert_eq!(event_id, event.event_id().unwrap());
}

#[test]
fn serialization_preserves_closed_loop_subject_identities() {
    let loop_ = closed_loop();
    let encoded = serde_json::to_string(&loop_.recovery).unwrap();
    let decoded: CircularityEventBindingV1 = serde_json::from_str(&encoded).unwrap();
    assert_eq!(
        loop_.recovery.binding_id().unwrap(),
        decoded.binding_id().unwrap()
    );
}
