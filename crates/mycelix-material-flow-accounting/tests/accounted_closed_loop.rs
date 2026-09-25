use mycelix_circularity::{
    ChainOfCustodyStrategyV1, CircularityAuthorityCeilingV1, CircularityDispositionV1,
    CircularityEventBindingV1, CircularityEventKindV1, CircularityEventProfileV1,
};
use mycelix_economic_reality_graph::{
    EconomicMaterialEventV1, EconomicResourceSubjectV1, ErgAuthorityCeilingV1,
    EventResourceEdgeV1, EventResourceRoleV1, ExactErgSubjectRefV1,
};
use mycelix_material_flow_accounting::{
    MassBasisV1, MassMg, MaterialAllocationRoleV1, MaterialAllocationV1,
    MaterialFlowAccountV1, MaterialFlowAuthorityCeilingV1, MaterialFlowEventBindingV1,
};
use std::collections::BTreeSet;

fn exact(namespace: &str, id: &str, fill: char) -> ExactErgSubjectRefV1 {
    ExactErgSubjectRefV1 {
        namespace: namespace.into(),
        subject_id: id.into(),
        semantic_version: "1".into(),
        content_blake3: std::iter::repeat_n(fill, 64).collect(),
    }
}

fn resource(id: &str, fill: char) -> ExactErgSubjectRefV1 {
    EconomicResourceSubjectV1 {
        semantic_version: "1".into(),
        resource_profile_ref: exact("mycelix.erg-profile", "material-or-article", fill),
        external_state_refs: vec![exact("symthaea.material-state", id, fill)],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some(id.into()),
    }
    .exact_ref()
    .unwrap()
}

fn allocation(
    id: &str,
    role: MaterialAllocationRoleV1,
    resource_ref: ExactErgSubjectRefV1,
    mass_mg: u64,
    fill: char,
) -> MaterialAllocationV1 {
    MaterialAllocationV1 {
        allocation_id: id.into(),
        role,
        material_or_resource_ref: resource_ref,
        mass: MassMg(mass_mg),
        evidence_refs: vec![exact("field.mass-evidence", id, fill)],
    }
}

#[derive(Clone)]
struct ClosedLoopFixture {
    returned_product: ExactErgSubjectRefV1,
    recovered_material: ExactErgSubjectRefV1,
    secondary_feedstock: ExactErgSubjectRefV1,
    product_v2: ExactErgSubjectRefV1,
    account: MaterialFlowAccountV1,
    recovery_binding: CircularityEventBindingV1,
    material_flow_binding: MaterialFlowEventBindingV1,
    characterization_event: EconomicMaterialEventV1,
    second_manufacture_event: EconomicMaterialEventV1,
}

fn fixture() -> ClosedLoopFixture {
    let returned_product = resource("returned-product-v1", 'a');
    let recovered_material = resource("recovered-material-unqualified", 'b');
    let secondary_feedstock = resource("secondary-feedstock-qualified", 'c');
    let product_v2 = resource("product-v2", 'd');

    let account = MaterialFlowAccountV1 {
        semantic_version: "1".into(),
        mass_basis: MassBasisV1::AsReceived,
        chain_of_custody: ChainOfCustodyStrategyV1::MassBalance,
        allocations: vec![
            allocation(
                "returned-input",
                MaterialAllocationRoleV1::AttributableInput,
                returned_product.clone(),
                1_000,
                '1',
            ),
            allocation(
                "recovered-output",
                MaterialAllocationRoleV1::RecoveredMaterialOutput,
                recovered_material.clone(),
                700,
                '2',
            ),
            allocation(
                "process-loss",
                MaterialAllocationRoleV1::ProcessLoss,
                exact("mycelix.accounting", "recovery-process-loss", '3'),
                100,
                '3',
            ),
            allocation(
                "unresolved-residual",
                MaterialAllocationRoleV1::UnresolvedResidual,
                exact("mycelix.accounting", "recovery-unresolved-residual", '4'),
                200,
                '4',
            ),
        ],
        authority_ceiling: MaterialFlowAuthorityCeilingV1::ConservationAndAttributionOnly,
        display_label: Some("synthetic recovery mass account".into()),
    };
    account.validate().unwrap();

    let recovery_profile = CircularityEventProfileV1 {
        semantic_version: "1".into(),
        event_kind: CircularityEventKindV1::RecoverMaterial,
        disposition: CircularityDispositionV1::MaterialRecovered,
        chain_of_custody: Some(ChainOfCustodyStrategyV1::MassBalance),
        material_flow_account_ref: Some(account.exact_ref().unwrap()),
        external_requirement_refs: vec![
            exact("symthaea.lifecycle-design", "profile-v1", '5'),
            exact("symthaea.mfg-plan", "recovery-plan-v1", '6'),
        ],
        authority_ceiling: CircularityAuthorityCeilingV1::LifecycleEventProfileAndReferencesOnly,
        display_label: Some("synthetic material recovery".into()),
    };

    let recovery_event = EconomicMaterialEventV1 {
        semantic_version: "1".into(),
        event_profile_ref: recovery_profile.exact_ref().unwrap(),
        inputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: returned_product.clone(),
        }],
        outputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::RecoveredFrom,
            resource_ref: recovered_material.clone(),
        }],
        party_refs: vec![exact("mycelix.party", "synthetic-recycler", '7')],
        site_refs: vec![exact("mycelix.site", "synthetic-recovery-site", '8')],
        effective_time_ref: exact("mycelix.time", "t-recovery", '9'),
        process_or_agreement_refs: vec![exact("symthaea.mfg-plan", "recovery-plan-v1", '6')],
        evidence_refs: vec![exact("field.synthetic", "recovery-observation", 'a')],
        revision_links: vec![],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some("synthetic recovery event".into()),
    };

    let recovery_binding = CircularityEventBindingV1 {
        profile: recovery_profile,
        event: recovery_event,
    };
    recovery_binding.validate().unwrap();

    let material_flow_binding = MaterialFlowEventBindingV1 {
        account_ref: account.exact_ref().unwrap(),
        lifecycle_event_ref: recovery_binding.event.exact_ref().unwrap(),
    };
    material_flow_binding.validate().unwrap();

    let characterization_event = EconomicMaterialEventV1 {
        semantic_version: "1".into(),
        event_profile_ref: exact("mycelix.erg-profile", "material-characterization", 'b'),
        inputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: recovered_material.clone(),
        }],
        outputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Transformed,
            resource_ref: secondary_feedstock.clone(),
        }],
        party_refs: vec![exact("mycelix.party", "synthetic-lab", 'c')],
        site_refs: vec![exact("mycelix.site", "synthetic-lab-site", 'd')],
        effective_time_ref: exact("mycelix.time", "t-characterization", 'e'),
        process_or_agreement_refs: vec![exact(
            "symthaea.material-qualification",
            "secondary-feedstock-profile-v1",
            'f',
        )],
        evidence_refs: vec![exact(
            "field.synthetic",
            "secondary-feedstock-characterization",
            '1',
        )],
        revision_links: vec![],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some("synthetic secondary-feedstock qualification".into()),
    };

    let second_manufacture_event = EconomicMaterialEventV1 {
        semantic_version: "1".into(),
        event_profile_ref: exact("mycelix.erg-profile", "manufacture", '2'),
        inputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Consumed,
            resource_ref: secondary_feedstock.clone(),
        }],
        outputs: vec![EventResourceEdgeV1 {
            role: EventResourceRoleV1::Combined,
            resource_ref: product_v2.clone(),
        }],
        party_refs: vec![exact("mycelix.party", "synthetic-manufacturer", '3')],
        site_refs: vec![exact("mycelix.site", "synthetic-manufacturing-site", '4')],
        effective_time_ref: exact("mycelix.time", "t-manufacture-v2", '5'),
        process_or_agreement_refs: vec![exact("symthaea.mfg-plan", "plan-v2", '6')],
        evidence_refs: vec![exact("mycelix.synthetic", "execution-receipt-v2", '7')],
        revision_links: vec![],
        authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
        display_label: Some("synthetic second manufacture".into()),
    };

    ClosedLoopFixture {
        returned_product,
        recovered_material,
        secondary_feedstock,
        product_v2,
        account,
        recovery_binding,
        material_flow_binding,
        characterization_event,
        second_manufacture_event,
    }
}

fn ref_id(reference: &ExactErgSubjectRefV1) -> String {
    reference.ref_id().unwrap()
}

fn edge_ref_ids(edges: &[EventResourceEdgeV1]) -> BTreeSet<String> {
    edges.iter().map(|edge| ref_id(&edge.resource_ref)).collect()
}

fn validate_accounted_recovery(f: &ClosedLoopFixture) -> Result<(), String> {
    f.account.validate().map_err(|err| err.to_string())?;
    f.recovery_binding
        .validate()
        .map_err(|err| err.to_string())?;
    f.material_flow_binding
        .validate()
        .map_err(|err| err.to_string())?;

    if f.recovery_binding.profile.event_kind != CircularityEventKindV1::RecoverMaterial {
        return Err("fixture is not a RecoverMaterial event".into());
    }
    if f.recovery_binding.profile.material_flow_account_ref.as_ref()
        != Some(&f.account.exact_ref().map_err(|err| err.to_string())?)
    {
        return Err("Circularity profile does not bind the exact material-flow account".into());
    }
    if f.recovery_binding.profile.chain_of_custody != Some(f.account.chain_of_custody) {
        return Err("Circularity/account chain-of-custody mismatch".into());
    }

    if f.material_flow_binding.account_ref
        != f.account.exact_ref().map_err(|err| err.to_string())?
    {
        return Err("post-event binding references a different material-flow account".into());
    }
    if f.material_flow_binding.lifecycle_event_ref
        != f
            .recovery_binding
            .event
            .exact_ref()
            .map_err(|err| err.to_string())?
    {
        return Err("post-event binding references a different recovery event".into());
    }

    let event_inputs = edge_ref_ids(&f.recovery_binding.event.inputs);
    let event_outputs = edge_ref_ids(&f.recovery_binding.event.outputs);
    for allocation in &f.account.allocations {
        let id = ref_id(&allocation.material_or_resource_ref);
        match allocation.role {
            MaterialAllocationRoleV1::AttributableInput
            | MaterialAllocationRoleV1::SeparatelyEvidencedAddition => {
                if !event_inputs.contains(&id) {
                    return Err(format!("account source allocation absent from event inputs: {id}"));
                }
            }
            MaterialAllocationRoleV1::RetainedComponentOutput
            | MaterialAllocationRoleV1::RecoveredMaterialOutput
            | MaterialAllocationRoleV1::RecycledMaterialOutput
            | MaterialAllocationRoleV1::DowncycledOutput => {
                if !event_outputs.contains(&id) {
                    return Err(format!("account output allocation absent from event outputs: {id}"));
                }
            }
            MaterialAllocationRoleV1::ProcessLoss
            | MaterialAllocationRoleV1::UnresolvedResidual
            | MaterialAllocationRoleV1::DisposedOutput => {}
        }
    }
    Ok(())
}

fn validate_secondary_feedstock_gate(f: &ClosedLoopFixture) -> Result<(), String> {
    f.characterization_event
        .validate()
        .map_err(|err| err.to_string())?;
    f.second_manufacture_event
        .validate()
        .map_err(|err| err.to_string())?;

    if ref_id(&f.recovered_material) == ref_id(&f.secondary_feedstock) {
        return Err("recovered material was relabeled as qualified secondary feedstock".into());
    }

    let characterization_inputs = edge_ref_ids(&f.characterization_event.inputs);
    let characterization_outputs = edge_ref_ids(&f.characterization_event.outputs);
    let manufacture_inputs = edge_ref_ids(&f.second_manufacture_event.inputs);

    if !characterization_inputs.contains(&ref_id(&f.recovered_material)) {
        return Err("characterization does not consume the recovered material subject".into());
    }
    if !characterization_outputs.contains(&ref_id(&f.secondary_feedstock)) {
        return Err("characterization does not produce the secondary-feedstock subject".into());
    }
    if !manufacture_inputs.contains(&ref_id(&f.secondary_feedstock)) {
        return Err("second manufacture does not consume the qualified secondary feedstock".into());
    }
    if manufacture_inputs.contains(&ref_id(&f.recovered_material)) {
        return Err("unqualified recovered material bypassed characterization".into());
    }
    Ok(())
}

#[test]
fn synthetic_closed_loop_binds_real_conservation_without_claim_upgrade() {
    let f = fixture();
    assert!(validate_accounted_recovery(&f).is_ok());
    assert!(validate_secondary_feedstock_gate(&f).is_ok());
    assert_ne!(ref_id(&f.returned_product), ref_id(&f.recovered_material));
    assert_ne!(ref_id(&f.recovered_material), ref_id(&f.secondary_feedstock));
    assert_ne!(ref_id(&f.secondary_feedstock), ref_id(&f.product_v2));
    assert!(!f.account.claims_recovery_yield());
    assert!(!f.account.claims_recycled_content());
    assert!(!f.account.claims_environmental_benefit());
    assert!(!f.recovery_binding.claims_environmental_benefit());
    assert!(!f.recovery_binding.claims_regulatory_compliance());
}

#[test]
fn profile_account_custody_mismatch_rejects_composition() {
    let mut f = fixture();
    f.recovery_binding.profile.chain_of_custody = Some(ChainOfCustodyStrategyV1::Segregated);
    assert!(validate_accounted_recovery(&f).is_err());
}

#[test]
fn different_valid_account_ref_rejects_composition() {
    let mut f = fixture();
    let mut other = f.account.clone();
    other.display_label = Some("same semantics different label".into());
    assert_eq!(other.exact_ref().unwrap(), f.account.exact_ref().unwrap());
    other.allocations[1].mass = MassMg(650);
    other.allocations[3].mass = MassMg(250);
    assert!(other.validate().is_ok());
    f.recovery_binding.profile.material_flow_account_ref = Some(other.exact_ref().unwrap());
    assert!(validate_accounted_recovery(&f).is_err());
}

#[test]
fn account_recovered_output_missing_from_event_rejects() {
    let mut f = fixture();
    f.recovery_binding.event.outputs[0].resource_ref = resource("substituted-recovery-output", '8');
    f.material_flow_binding.lifecycle_event_ref = f.recovery_binding.event.exact_ref().unwrap();
    assert!(validate_accounted_recovery(&f).is_err());
}

#[test]
fn wrong_post_event_binding_rejects() {
    let mut f = fixture();
    f.material_flow_binding.lifecycle_event_ref = exact("mycelix.erg.event", "other-event", '9');
    assert!(validate_accounted_recovery(&f).is_err());
}

#[test]
fn recovered_material_cannot_bypass_characterization() {
    let mut f = fixture();
    f.second_manufacture_event.inputs[0].resource_ref = f.recovered_material.clone();
    assert!(validate_secondary_feedstock_gate(&f).is_err());
}

#[test]
fn unexplained_material_gap_still_fails_at_account_layer() {
    let mut f = fixture();
    f.account.allocations.retain(|allocation| {
        allocation.role != MaterialAllocationRoleV1::UnresolvedResidual
    });
    assert!(f.account.validate().is_err());
}
