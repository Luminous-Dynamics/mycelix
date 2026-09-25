use mycelix_manufacturing_process_refs::{
    CapabilityProfileRefV1, DigestAlgorithmV1, DigestRefV1, EvidenceSubjectRefV1,
    ExternalSubjectRefV1, ProviderSubjectRefV1, ResourceSubjectRefV1, SiteSubjectRefV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
struct ProcessPlanSubjectRefV1(ExternalSubjectRefV1);

impl ProcessPlanSubjectRefV1 {
    fn validate(&self) -> Result<(), String> {
        self.0.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
struct WorkScopeSubjectRefV1(ExternalSubjectRefV1);

impl WorkScopeSubjectRefV1 {
    fn validate(&self) -> Result<(), String> {
        self.0.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct PlanStepAssignmentV1 {
    plan_node_id: String,
    provider: ProviderSubjectRefV1,
    site: SiteSubjectRefV1,
    resource: ResourceSubjectRefV1,
    capability_profile: CapabilityProfileRefV1,
    evidence_refs: Vec<EvidenceSubjectRefV1>,
    assignment_revision: String,
    valid_from_unix_s: u64,
    valid_until_unix_s: u64,
    display_label: Option<String>,
    scheduler_slot_ref: Option<String>,
}

impl PlanStepAssignmentV1 {
    fn validate(&self) -> Result<(), String> {
        canonical("assignment.plan_node_id", &self.plan_node_id)?;
        canonical("assignment.revision", &self.assignment_revision)?;
        self.provider.validate()?;
        self.site.validate()?;
        self.resource.validate()?;
        self.capability_profile.validate()?;
        if self.evidence_refs.is_empty() {
            return Err("assignment requires capability/evidence refs".into());
        }
        let mut seen = BTreeSet::new();
        for evidence in &self.evidence_refs {
            evidence.validate()?;
            if !seen.insert(evidence.clone()) {
                return Err("duplicate assignment evidence ref".into());
            }
        }
        if self.valid_from_unix_s >= self.valid_until_unix_s {
            return Err("assignment validity window must be ordered".into());
        }
        if let Some(label) = &self.display_label {
            canonical("assignment.display_label", label)?;
        }
        if let Some(slot) = &self.scheduler_slot_ref {
            canonical("assignment.scheduler_slot_ref", slot)?;
        }
        Ok(())
    }

    fn engineering_identity(&self) -> AssignmentEngineeringIdentityV1 {
        AssignmentEngineeringIdentityV1 {
            plan_node_id: self.plan_node_id.clone(),
            provider: self.provider.clone(),
            site: self.site.clone(),
            resource: self.resource.clone(),
            capability_profile: self.capability_profile.clone(),
            evidence_refs: {
                let mut refs = self.evidence_refs.clone();
                refs.sort();
                refs
            },
            assignment_revision: self.assignment_revision.clone(),
            valid_from_unix_s: self.valid_from_unix_s,
            valid_until_unix_s: self.valid_until_unix_s,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct AssignmentEngineeringIdentityV1 {
    plan_node_id: String,
    provider: ProviderSubjectRefV1,
    site: SiteSubjectRefV1,
    resource: ResourceSubjectRefV1,
    capability_profile: CapabilityProfileRefV1,
    evidence_refs: Vec<EvidenceSubjectRefV1>,
    assignment_revision: String,
    valid_from_unix_s: u64,
    valid_until_unix_s: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct ProcessPlanInstanceV1 {
    canonical_plan: ProcessPlanSubjectRefV1,
    work_scope: WorkScopeSubjectRefV1,
    instance_revision: String,
    assignments: Vec<PlanStepAssignmentV1>,
    display_label: Option<String>,
}

impl ProcessPlanInstanceV1 {
    fn validate(&self) -> Result<(), String> {
        self.canonical_plan.validate()?;
        self.work_scope.validate()?;
        canonical("instance.revision", &self.instance_revision)?;
        if let Some(label) = &self.display_label {
            canonical("instance.display_label", label)?;
        }

        let mut assigned_nodes = BTreeSet::new();
        for assignment in &self.assignments {
            assignment.validate()?;
            if !assigned_nodes.insert(assignment.plan_node_id.as_str()) {
                return Err("duplicate resource assignment for canonical plan node".into());
            }
        }
        Ok(())
    }

    fn engineering_identity(&self) -> PlanInstanceEngineeringIdentityV1 {
        let mut assignments: Vec<_> = self
            .assignments
            .iter()
            .map(PlanStepAssignmentV1::engineering_identity)
            .collect();
        assignments.sort_by(|a, b| a.plan_node_id.cmp(&b.plan_node_id));
        PlanInstanceEngineeringIdentityV1 {
            canonical_plan: self.canonical_plan.clone(),
            work_scope: self.work_scope.clone(),
            instance_revision: self.instance_revision.clone(),
            assignments,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct PlanInstanceEngineeringIdentityV1 {
    canonical_plan: ProcessPlanSubjectRefV1,
    work_scope: WorkScopeSubjectRefV1,
    instance_revision: String,
    assignments: Vec<AssignmentEngineeringIdentityV1>,
}

fn canonical(field: &str, value: &str) -> Result<(), String> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(format!("{field} must be canonical"));
    }
    Ok(())
}

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
        scheduler_slot_ref: Some("scheduler:slot:2026-09-25T14:00".into()),
    }
}

fn fixture() -> ProcessPlanInstanceV1 {
    ProcessPlanInstanceV1 {
        canonical_plan: ProcessPlanSubjectRefV1(subject(
            "symthaea.mfg-process-plan",
            "plan-001",
            'f',
        )),
        work_scope: WorkScopeSubjectRefV1(subject(
            "mycelix.work-order",
            "wo-001-batch-a",
            '1',
        )),
        instance_revision: "instance-r1".into(),
        assignments: vec![assignment("rough-machine"), assignment("finish-machine")],
        display_label: Some("WO-001 production instance".into()),
    }
}

#[test]
fn valid_plan_instance_preserves_exact_cross_repo_refs() {
    let instance = fixture();
    assert!(instance.validate().is_ok());
    assert_eq!(instance.assignments.len(), 2);
}

#[test]
fn duplicate_assignment_for_same_plan_node_rejects() {
    let mut instance = fixture();
    instance.assignments.push(assignment("rough-machine"));
    assert!(instance.validate().is_err());
}

#[test]
fn malformed_nested_ref_fails_closed() {
    let mut instance = fixture();
    instance.assignments[0].resource.0.content_digest.hex = "BAD".into();
    assert!(instance.validate().is_err());
}

#[test]
fn assignment_requires_capability_evidence() {
    let mut instance = fixture();
    instance.assignments[0].evidence_refs.clear();
    assert!(instance.validate().is_err());
}

#[test]
fn scheduler_slot_and_display_labels_do_not_change_engineering_identity() {
    let a = fixture();
    let mut b = a.clone();
    b.display_label = Some("renamed UI instance".into());
    b.assignments[0].display_label = Some("renamed assignment".into());
    b.assignments[0].scheduler_slot_ref = Some("scheduler:slot:later".into());
    assert_eq!(a.engineering_identity(), b.engineering_identity());
}

#[test]
fn resource_or_capability_change_changes_engineering_identity() {
    let a = fixture();
    let mut b = a.clone();
    b.assignments[0].resource = ResourceSubjectRefV1(subject(
        "eng-catalog.resource",
        "machine-2",
        '9',
    ));
    assert_ne!(a.engineering_identity(), b.engineering_identity());

    let mut c = a.clone();
    c.assignments[0].capability_profile = CapabilityProfileRefV1(subject(
        "symthaea.mfg-capability",
        "capability-2",
        '8',
    ));
    assert_ne!(a.engineering_identity(), c.engineering_identity());
}

#[test]
fn assignment_order_does_not_change_engineering_identity() {
    let a = fixture();
    let mut b = a.clone();
    b.assignments.reverse();
    assert_eq!(a.engineering_identity(), b.engineering_identity());
}

#[test]
fn serde_round_trip_preserves_plan_instance_and_assignment_refs() {
    let instance = fixture();
    let encoded = serde_json::to_string(&instance).unwrap();
    let decoded: ProcessPlanInstanceV1 = serde_json::from_str(&encoded).unwrap();
    assert_eq!(decoded, instance);
    assert_eq!(decoded.engineering_identity(), instance.engineering_identity());
}
