// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Public, migration-safe production-plan instantiation and resource-assignment contracts.
//!
//! Core theorem:
//!
//! ```text
//! canonical process plan
//! != production plan instance
//! != resource assignment
//! != schedule
//! != execution receipt
//! != output acceptance
//! ```

use mycelix_manufacturing_process_refs::{
    CapabilityProfileRefV1, EvidenceSubjectRefV1, ExternalSubjectRefV1, ProviderSubjectRefV1,
    ResourceSubjectRefV1, SiteSubjectRefV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

fn canonical(field: &str, value: &str) -> Result<(), String> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(format!("{field} must be canonical"));
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ProcessPlanSubjectRefV1(pub ExternalSubjectRefV1);

impl ProcessPlanSubjectRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.0.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct WorkScopeSubjectRefV1(pub ExternalSubjectRefV1);

impl WorkScopeSubjectRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.0.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanStepAssignmentV1 {
    pub plan_node_id: String,
    pub provider: ProviderSubjectRefV1,
    pub site: SiteSubjectRefV1,
    pub resource: ResourceSubjectRefV1,
    pub capability_profile: CapabilityProfileRefV1,
    #[serde(default)]
    pub evidence_refs: Vec<EvidenceSubjectRefV1>,
    pub assignment_revision: String,
    pub valid_from_unix_s: u64,
    pub valid_until_unix_s: u64,
    /// Operational/UI metadata only; excluded from engineering identity.
    pub display_label: Option<String>,
    /// Scheduler placement only; excluded from engineering identity.
    pub scheduler_slot_ref: Option<String>,
}

impl PlanStepAssignmentV1 {
    pub fn validate(&self) -> Result<(), String> {
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

    pub fn engineering_identity(&self) -> Result<AssignmentEngineeringIdentityV1, String> {
        self.validate()?;
        let mut evidence_refs = self.evidence_refs.clone();
        evidence_refs.sort();
        Ok(AssignmentEngineeringIdentityV1 {
            plan_node_id: self.plan_node_id.clone(),
            provider: self.provider.clone(),
            site: self.site.clone(),
            resource: self.resource.clone(),
            capability_profile: self.capability_profile.clone(),
            evidence_refs,
            assignment_revision: self.assignment_revision.clone(),
            valid_from_unix_s: self.valid_from_unix_s,
            valid_until_unix_s: self.valid_until_unix_s,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct AssignmentEngineeringIdentityV1 {
    pub plan_node_id: String,
    pub provider: ProviderSubjectRefV1,
    pub site: SiteSubjectRefV1,
    pub resource: ResourceSubjectRefV1,
    pub capability_profile: CapabilityProfileRefV1,
    pub evidence_refs: Vec<EvidenceSubjectRefV1>,
    pub assignment_revision: String,
    pub valid_from_unix_s: u64,
    pub valid_until_unix_s: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProcessPlanInstanceV1 {
    pub canonical_plan: ProcessPlanSubjectRefV1,
    pub work_scope: WorkScopeSubjectRefV1,
    pub instance_revision: String,
    #[serde(default)]
    pub assignments: Vec<PlanStepAssignmentV1>,
    /// UI metadata only; excluded from engineering identity.
    pub display_label: Option<String>,
}

impl ProcessPlanInstanceV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.canonical_plan.validate()?;
        self.work_scope.validate()?;
        canonical("instance.revision", &self.instance_revision)?;
        if let Some(label) = &self.display_label {
            canonical("instance.display_label", label)?;
        }

        let mut assigned_nodes = BTreeSet::new();
        for assignment in &self.assignments {
            assignment.validate()?;
            if !assigned_nodes.insert(assignment.plan_node_id.clone()) {
                return Err("duplicate resource assignment for canonical plan node".into());
            }
        }
        Ok(())
    }

    pub fn engineering_identity(&self) -> Result<PlanInstanceEngineeringIdentityV1, String> {
        self.validate()?;
        let mut assignments = self
            .assignments
            .iter()
            .map(PlanStepAssignmentV1::engineering_identity)
            .collect::<Result<Vec<_>, _>>()?;
        assignments.sort_by(|a, b| a.plan_node_id.cmp(&b.plan_node_id));
        Ok(PlanInstanceEngineeringIdentityV1 {
            canonical_plan: self.canonical_plan.clone(),
            work_scope: self.work_scope.clone(),
            instance_revision: self.instance_revision.clone(),
            assignments,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanInstanceEngineeringIdentityV1 {
    pub canonical_plan: ProcessPlanSubjectRefV1,
    pub work_scope: WorkScopeSubjectRefV1,
    pub instance_revision: String,
    pub assignments: Vec<AssignmentEngineeringIdentityV1>,
}
