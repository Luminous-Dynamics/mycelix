// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Projects Integrity Zome
//!
//! Defines entry types and validation for climate projects.
//! Uses HDI 0.7.0-dev.1 with FlatOp validation pattern.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_link_author_match, did_for_author};

/// Anchor entry for creating deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

/// Type of climate project
#[hdk_entry_helper]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ProjectType {
    /// Tree planting and forest restoration
    Reforestation,
    /// Solar, wind, hydro, etc.
    RenewableEnergy,
    /// Capturing methane from landfills, farms, etc.
    MethaneCapture,
    /// Coastal and marine ecosystem restoration
    OceanRestoration,
    /// Direct air capture of CO2
    DirectAirCapture,
}

/// Status of a climate project
#[hdk_entry_helper]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ProjectStatus {
    /// Initial proposal submitted
    Proposed,
    /// Verified by third party
    Verified,
    /// Actively generating credits
    Active,
    /// Project has finished its term
    Completed,
}

/// Geographic location for a project
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Location {
    /// Country code (ISO 3166-1 alpha-2)
    pub country_code: String,
    /// Region/state/province
    pub region: Option<String>,
    /// Latitude in decimal degrees
    pub latitude: f64,
    /// Longitude in decimal degrees
    pub longitude: f64,
}

/// A climate project that generates carbon credits
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ClimateProject {
    /// Unique project identifier
    pub id: String,
    /// Human-readable project name
    pub name: String,
    /// Type of climate project
    pub project_type: ProjectType,
    /// Geographic location
    pub location: Location,
    /// Expected total credits over project lifetime
    pub expected_credits: f64,
    /// Project start date (Unix timestamp)
    pub start_date: i64,
    /// DID of the verifying organization
    pub verifier_did: Option<String>,
    /// Current project status
    pub status: ProjectStatus,
}

/// A milestone in a climate project's lifecycle
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ProjectMilestone {
    /// ID of the associated project
    pub project_id: String,
    /// Milestone title
    pub title: String,
    /// Detailed description
    pub description: String,
    /// Target date (Unix timestamp)
    pub target_date: i64,
    /// Actual completion date (Unix timestamp, if completed)
    pub completed_at: Option<i64>,
    /// Credits issued upon completion
    pub credits_issued: Option<f64>,
    /// DID of the verifier who approved this milestone
    pub verified_by: Option<String>,
}

/// Entry types for the projects zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    ClimateProject(ClimateProject),
    #[entry_type(visibility = "public")]
    ProjectMilestone(ProjectMilestone),
}

/// Link types for the projects zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all projects
    AnchorToProjects,
    /// Anchor to projects by type
    TypeToProjects,
    /// Anchor to projects by status
    StatusToProjects,
    /// Project to its milestones
    ProjectToMilestones,
    /// Project updates chain
    ProjectUpdates,
    /// Verifier to projects they verified
    VerifierToProjects,
}

/// Validate DIDs have proper format
fn validate_did(did: &str) -> ExternResult<ValidateCallbackResult> {
    if did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid("DID cannot be empty".to_string()));
    }
    if !did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:' prefix".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a Location
fn validate_location(location: &Location) -> ExternResult<ValidateCallbackResult> {
    if location.country_code.len() != 2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Country code must be 2 characters (ISO 3166-1 alpha-2)".to_string(),
        ));
    }

    if !location.latitude.is_finite() || !location.longitude.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude and longitude must be finite".to_string(),
        ));
    }

    if location.latitude < -90.0 || location.latitude > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".to_string(),
        ));
    }

    if location.longitude < -180.0 || location.longitude > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a ClimateProject entry
fn validate_climate_project(project: &ClimateProject) -> ExternResult<ValidateCallbackResult> {
    if project.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Project ID cannot be empty".to_string(),
        ));
    }

    if project.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Project name cannot be empty".to_string(),
        ));
    }

    let location_result = validate_location(&project.location)?;
    if let ValidateCallbackResult::Invalid(_) = location_result {
        return Ok(location_result);
    }

    if !project.expected_credits.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Expected credits must be finite".to_string(),
        ));
    }
    if project.expected_credits < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Expected credits cannot be negative".to_string(),
        ));
    }

    if let Some(ref verifier) = project.verifier_did {
        let verifier_result = validate_did(verifier)?;
        if let ValidateCallbackResult::Invalid(_) = verifier_result {
            return Ok(verifier_result);
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a ProjectMilestone entry
fn validate_milestone(milestone: &ProjectMilestone) -> ExternResult<ValidateCallbackResult> {
    if milestone.project_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Project ID cannot be empty".to_string(),
        ));
    }

    if milestone.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Milestone title cannot be empty".to_string(),
        ));
    }

    if let Some(credits) = milestone.credits_issued {
        if !credits.is_finite() {
            return Ok(ValidateCallbackResult::Invalid(
                "Credits issued must be finite".to_string(),
            ));
        }
        if credits < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Credits issued cannot be negative".to_string(),
            ));
        }
    }

    if let Some(ref verifier) = milestone.verified_by {
        let verifier_result = validate_did(verifier)?;
        if let ValidateCallbackResult::Invalid(_) = verifier_result {
            return Ok(verifier_result);
        }
    }

    if let Some(completed) = milestone.completed_at {
        if completed < milestone.target_date - 31536000 {
            return Ok(ValidateCallbackResult::Invalid(
                "Completion date seems unreasonably early".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Creation policy for a climate project.
fn validate_create_project(project: &ClimateProject) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_climate_project(project)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }

    if project.status != ProjectStatus::Proposed {
        return Ok(ValidateCallbackResult::Invalid(
            "Climate project must be created in Proposed status".into(),
        ));
    }
    if project.verifier_did.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposed climate project must not have a verifier".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Creation policy for a project milestone.
fn validate_create_milestone(milestone: &ProjectMilestone) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_milestone(milestone)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }

    if milestone.completed_at.is_some()
        || milestone.credits_issued.is_some()
        || milestone.verified_by.is_some()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Project milestone must be created incomplete and without credits or verifier".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Pure relationship policy for the ProjectToMilestones link.
fn validate_project_milestone_binding(
    project: &ClimateProject,
    milestone: &ProjectMilestone,
) -> ValidateCallbackResult {
    if project.id != milestone.project_id {
        return ValidateCallbackResult::Invalid(format!(
            "Milestone project_id {:?} does not match linked project id {:?}",
            milestone.project_id, project.id
        ));
    }
    ValidateCallbackResult::Valid
}

/// Resolve and validate both ends of a ProjectToMilestones link.
fn validate_project_to_milestone_link(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let project_action_hash = ActionHash::try_from(base_address).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "ProjectToMilestones base must be a ClimateProject action hash".into()
        ))
    })?;
    let milestone_action_hash = ActionHash::try_from(target_address).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "ProjectToMilestones target must be a ProjectMilestone action hash".into()
        ))
    })?;

    let project_record = must_get_valid_record(project_action_hash)?;
    let project: ClimateProject = project_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "ProjectToMilestones base is not a ClimateProject entry".into()
        )))?;

    let milestone_record = must_get_valid_record(milestone_action_hash)?;
    let milestone: ProjectMilestone = milestone_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "ProjectToMilestones target is not a ProjectMilestone entry".into()
        )))?;

    Ok(validate_project_milestone_binding(&project, &milestone))
}

/// Compute the sole valid anchor text for a project index link.
///
/// Indexes are derived views of validated project state. Their base anchor must
/// therefore be reproducible from the target record instead of trusted as
/// caller-supplied metadata.
fn expected_project_index_anchor(
    link_type: &LinkTypes,
    project: &ClimateProject,
) -> Result<String, String> {
    match link_type {
        LinkTypes::AnchorToProjects => Ok("all_projects".into()),
        LinkTypes::TypeToProjects => Ok(format!("type:{:?}", project.project_type)),
        LinkTypes::StatusToProjects => Ok(format!("status:{:?}", project.status)),
        LinkTypes::VerifierToProjects => project
            .verifier_did
            .as_ref()
            .map(|did| format!("verifier:{did}"))
            .ok_or_else(|| "VerifierToProjects target has no verifier DID".to_string()),
        _ => Err("link type is not a project index".into()),
    }
}

/// Validate a project index by resolving its target and recomputing the exact
/// anchor hash from target state.
fn validate_project_index_link(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    link_type: &LinkTypes,
) -> ExternResult<ValidateCallbackResult> {
    let project_action_hash = ActionHash::try_from(target_address).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "Project index target must be a ClimateProject action hash".into()
        ))
    })?;
    let project_record = must_get_valid_record(project_action_hash)?;
    let project: ClimateProject = project_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Project index target is not a ClimateProject entry".into()
        )))?;

    let anchor_text = match expected_project_index_anchor(link_type, &project) {
        Ok(anchor) => anchor,
        Err(reason) => return Ok(ValidateCallbackResult::Invalid(reason)),
    };
    let expected_entry_hash = hash_entry(&Anchor(anchor_text.clone()))?;
    let expected_base: AnyLinkableHash = expected_entry_hash.into();

    if base_address != expected_base {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Project index base does not match derived anchor {anchor_text:?}"
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Pure state-machine policy for climate-project lifecycle updates.
fn validate_project_transition(
    original: &ClimateProject,
    updated: &ClimateProject,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    if original.id != updated.id
        || original.name != updated.name
        || original.project_type != updated.project_type
        || original.location != updated.location
        || original.expected_credits != updated.expected_credits
        || original.start_date != updated.start_date
    {
        return ValidateCallbackResult::Invalid(
            "Climate project proposal fields are immutable after creation".into(),
        );
    }

    let author_did = did_for_author(author);
    match (original.status, updated.status) {
        (ProjectStatus::Proposed, ProjectStatus::Verified) => {
            if original.verifier_did.is_some() {
                return ValidateCallbackResult::Invalid(
                    "Proposed project must not already have a verifier".into(),
                );
            }
            match updated.verifier_did.as_deref() {
                Some(verifier) if verifier == author_did => ValidateCallbackResult::Valid,
                Some(_) => ValidateCallbackResult::Invalid(
                    "Project verifier must be the committing agent".into(),
                ),
                None => ValidateCallbackResult::Invalid(
                    "Verified project must record its verifier".into(),
                ),
            }
        }
        (ProjectStatus::Verified, ProjectStatus::Active)
        | (ProjectStatus::Active, ProjectStatus::Completed) => {
            if updated.verifier_did != original.verifier_did {
                return ValidateCallbackResult::Invalid(
                    "Project verifier cannot change after verification".into(),
                );
            }
            match original.verifier_did.as_deref() {
                Some(verifier) if verifier == author_did => ValidateCallbackResult::Valid,
                Some(_) => ValidateCallbackResult::Invalid(
                    "Only the recorded project verifier can advance the project lifecycle".into(),
                ),
                None => ValidateCallbackResult::Invalid(
                    "Verified/active project is missing its verifier".into(),
                ),
            }
        }
        _ => ValidateCallbackResult::Invalid(
            "Invalid climate project status transition".into(),
        ),
    }
}

fn validate_update_project(
    action: Update,
    updated: ClimateProject,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_climate_project(&updated)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: ClimateProject = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original ClimateProject entry not found".to_string()
        )))?;

    Ok(validate_project_transition(
        &original,
        &updated,
        &action.author,
    ))
}

/// Pure state-machine policy for milestone completion.
fn validate_milestone_transition(
    original: &ProjectMilestone,
    updated: &ProjectMilestone,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    if original.project_id != updated.project_id
        || original.title != updated.title
        || original.description != updated.description
        || original.target_date != updated.target_date
    {
        return ValidateCallbackResult::Invalid(
            "Project milestone definition is immutable after creation".into(),
        );
    }

    if original.completed_at.is_some()
        || original.credits_issued.is_some()
        || original.verified_by.is_some()
    {
        return ValidateCallbackResult::Invalid(
            "Completed milestone cannot be completed or reassigned again".into(),
        );
    }

    if updated.completed_at.is_none() {
        return ValidateCallbackResult::Invalid(
            "Milestone completion must set completed_at".into(),
        );
    }

    let expected_verifier = did_for_author(author);
    match updated.verified_by.as_deref() {
        Some(verifier) if verifier == expected_verifier => ValidateCallbackResult::Valid,
        Some(_) => ValidateCallbackResult::Invalid(
            "Milestone verifier must be the committing agent".into(),
        ),
        None => ValidateCallbackResult::Invalid(
            "Milestone completion must record its verifier".into(),
        ),
    }
}

fn validate_update_milestone(
    action: Update,
    updated: ProjectMilestone,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_milestone(&updated)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: ProjectMilestone = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original ProjectMilestone entry not found".to_string()
        )))?;

    Ok(validate_milestone_transition(
        &original,
        &updated,
        &action.author,
    ))
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ClimateProject(project) => validate_create_project(&project),
                EntryTypes::ProjectMilestone(milestone) => validate_create_milestone(&milestone),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Invalid(
                    "Project anchors cannot be updated".into(),
                )),
                EntryTypes::ClimateProject(project) => {
                    validate_update_project(action, project, original_action_hash)
                }
                EntryTypes::ProjectMilestone(milestone) => {
                    validate_update_milestone(action, milestone, original_action_hash)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            link_type,
            ..
        } => match link_type {
            LinkTypes::ProjectToMilestones => {
                validate_project_to_milestone_link(base_address, target_address)
            }
            LinkTypes::AnchorToProjects
            | LinkTypes::TypeToProjects
            | LinkTypes::StatusToProjects
            | LinkTypes::VerifierToProjects => {
                validate_project_index_link(base_address, target_address, &link_type)
            }
            LinkTypes::ProjectUpdates => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { link_type, action, .. } => match link_type {
            LinkTypes::ProjectToMilestones
            | LinkTypes::ProjectUpdates
            | LinkTypes::VerifierToProjects => Ok(ValidateCallbackResult::Invalid(
                "Project audit-history links cannot be deleted".to_string(),
            )),
            _ => {
                let original_action = must_get_action(action.link_add_address.clone())?;
                Ok(check_link_author_match(
                    original_action.action().author(),
                    &action.author,
                ))
            }
        },
        FlatOp::StoreRecord(_)
        | FlatOp::RegisterAgentActivity(_)
        | FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Climate projects and milestones are audit records and cannot be deleted".into(),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn valid_location() -> Location {
        Location {
            country_code: "ZA".into(),
            region: Some("Gauteng".into()),
            latitude: -26.2041,
            longitude: 28.0473,
        }
    }

    fn valid_project() -> ClimateProject {
        ClimateProject {
            id: "project:heat-resilience-1".into(),
            name: "Heat resilience pilot".into(),
            project_type: ProjectType::RenewableEnergy,
            location: valid_location(),
            expected_credits: 0.0,
            start_date: 1_788_825_600,
            verifier_did: None,
            status: ProjectStatus::Proposed,
        }
    }

    fn valid_milestone() -> ProjectMilestone {
        ProjectMilestone {
            project_id: "project:heat-resilience-1".into(),
            title: "Baseline complete".into(),
            description: "Baseline measurements collected".into(),
            target_date: 1_800_000_000,
            completed_at: None,
            credits_issued: None,
            verified_by: None,
        }
    }

    fn assert_valid(result: ExternResult<ValidateCallbackResult>) {
        assert!(matches!(
            result.expect("validator should execute"),
            ValidateCallbackResult::Valid
        ));
    }

    fn assert_invalid_contains(result: ExternResult<ValidateCallbackResult>, needle: &str) {
        match result.expect("validator should execute") {
            ValidateCallbackResult::Invalid(reason) => assert!(
                reason.contains(needle),
                "expected rejection containing {needle:?}, got {reason:?}"
            ),
            other => panic!("expected invalid result, got {other:?}"),
        }
    }

    fn assert_transition_invalid(result: ValidateCallbackResult, needle: &str) {
        match result {
            ValidateCallbackResult::Invalid(reason) => assert!(
                reason.contains(needle),
                "expected rejection containing {needle:?}, got {reason:?}"
            ),
            other => panic!("expected invalid result, got {other:?}"),
        }
    }

    #[test]
    fn production_location_validator_accepts_valid_coordinates() {
        assert_valid(validate_location(&valid_location()));
    }

    #[test]
    fn production_location_validator_rejects_out_of_range_coordinates() {
        let mut location = valid_location();
        location.latitude = 90.1;
        assert_invalid_contains(validate_location(&location), "Latitude");

        let mut location = valid_location();
        location.longitude = -180.1;
        assert_invalid_contains(validate_location(&location), "Longitude");
    }

    #[test]
    fn production_location_validator_rejects_non_finite_coordinates() {
        let mut location = valid_location();
        location.latitude = f64::NAN;
        assert_invalid_contains(validate_location(&location), "finite");
    }

    #[test]
    fn production_project_validator_accepts_valid_project() {
        assert_valid(validate_climate_project(&valid_project()));
    }

    #[test]
    fn production_project_validator_rejects_negative_expected_credits() {
        let mut project = valid_project();
        project.expected_credits = -0.01;
        assert_invalid_contains(validate_climate_project(&project), "Expected credits");
    }

    #[test]
    fn production_project_validator_rejects_non_finite_expected_credits() {
        let mut project = valid_project();
        project.expected_credits = f64::INFINITY;
        assert_invalid_contains(validate_climate_project(&project), "finite");
    }

    #[test]
    fn production_project_validator_rejects_invalid_verifier_identity() {
        let mut project = valid_project();
        project.verifier_did = Some("verifier-1".into());
        assert_invalid_contains(validate_climate_project(&project), "did:");
    }

    #[test]
    fn project_creation_requires_proposed_unverified_state() {
        assert_valid(validate_create_project(&valid_project()));

        let mut project = valid_project();
        project.status = ProjectStatus::Active;
        assert_invalid_contains(validate_create_project(&project), "Proposed");

        let mut project = valid_project();
        project.verifier_did = Some("did:example:forged".into());
        assert_invalid_contains(validate_create_project(&project), "must not have a verifier");
    }

    #[test]
    fn project_index_anchor_is_derived_from_target_state() {
        let project = valid_project();
        assert_eq!(
            expected_project_index_anchor(&LinkTypes::AnchorToProjects, &project).unwrap(),
            "all_projects"
        );
        assert_eq!(
            expected_project_index_anchor(&LinkTypes::TypeToProjects, &project).unwrap(),
            "type:RenewableEnergy"
        );
        assert_eq!(
            expected_project_index_anchor(&LinkTypes::StatusToProjects, &project).unwrap(),
            "status:Proposed"
        );
    }

    #[test]
    fn verifier_index_requires_and_uses_target_verifier() {
        let project = valid_project();
        assert!(expected_project_index_anchor(&LinkTypes::VerifierToProjects, &project).is_err());

        let mut verified = project;
        verified.status = ProjectStatus::Verified;
        verified.verifier_did = Some("did:example:verifier".into());
        assert_eq!(
            expected_project_index_anchor(&LinkTypes::VerifierToProjects, &verified).unwrap(),
            "verifier:did:example:verifier"
        );
    }

    #[test]
    fn production_milestone_validator_accepts_valid_milestone() {
        assert_valid(validate_milestone(&valid_milestone()));
    }

    #[test]
    fn milestone_creation_requires_incomplete_uncredited_state() {
        assert_valid(validate_create_milestone(&valid_milestone()));

        let mut milestone = valid_milestone();
        milestone.completed_at = Some(milestone.target_date);
        milestone.credits_issued = Some(10.0);
        milestone.verified_by = Some("did:example:forged".into());
        assert_invalid_contains(validate_create_milestone(&milestone), "must be created incomplete");
    }

    #[test]
    fn project_milestone_binding_requires_exact_logical_project_id() {
        let project = valid_project();
        let milestone = valid_milestone();
        assert!(matches!(
            validate_project_milestone_binding(&project, &milestone),
            ValidateCallbackResult::Valid
        ));

        let mut wrong = valid_milestone();
        wrong.project_id = "project:other".into();
        assert_transition_invalid(
            validate_project_milestone_binding(&project, &wrong),
            "does not match linked project id",
        );
    }

    #[test]
    fn production_milestone_validator_rejects_negative_credits() {
        let mut milestone = valid_milestone();
        milestone.credits_issued = Some(-1.0);
        assert_invalid_contains(validate_milestone(&milestone), "Credits issued");
    }

    #[test]
    fn production_milestone_validator_rejects_non_finite_credits() {
        let mut milestone = valid_milestone();
        milestone.credits_issued = Some(f64::NAN);
        assert_invalid_contains(validate_milestone(&milestone), "finite");
    }

    #[test]
    fn production_milestone_validator_rejects_unreasonably_early_completion() {
        let mut milestone = valid_milestone();
        milestone.completed_at = Some(milestone.target_date - 31_536_001);
        assert_invalid_contains(validate_milestone(&milestone), "unreasonably early");
    }

    #[test]
    fn proposed_project_can_be_verified_only_by_recorded_update_author() {
        let verifier = fake_agent(1);
        let original = valid_project();
        let mut updated = original.clone();
        updated.status = ProjectStatus::Verified;
        updated.verifier_did = Some(did_for_author(&verifier));
        assert!(matches!(
            validate_project_transition(&original, &updated, &verifier),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn project_verification_rejects_forged_verifier() {
        let verifier = fake_agent(1);
        let victim = fake_agent(2);
        let original = valid_project();
        let mut updated = original.clone();
        updated.status = ProjectStatus::Verified;
        updated.verifier_did = Some(did_for_author(&victim));
        assert_transition_invalid(
            validate_project_transition(&original, &updated, &verifier),
            "committing agent",
        );
    }

    #[test]
    fn project_verification_cannot_rewrite_proposal() {
        let verifier = fake_agent(1);
        let original = valid_project();
        let mut updated = original.clone();
        updated.status = ProjectStatus::Verified;
        updated.verifier_did = Some(did_for_author(&verifier));
        updated.expected_credits = 100.0;
        assert_transition_invalid(
            validate_project_transition(&original, &updated, &verifier),
            "immutable",
        );
    }

    #[test]
    fn project_cannot_skip_verification() {
        let actor = fake_agent(1);
        let original = valid_project();
        let mut updated = original.clone();
        updated.status = ProjectStatus::Active;
        assert_transition_invalid(
            validate_project_transition(&original, &updated, &actor),
            "Invalid climate project status transition",
        );
    }

    #[test]
    fn only_recorded_verifier_can_activate_project() {
        let verifier = fake_agent(1);
        let attacker = fake_agent(2);
        let mut original = valid_project();
        original.status = ProjectStatus::Verified;
        original.verifier_did = Some(did_for_author(&verifier));
        let mut updated = original.clone();
        updated.status = ProjectStatus::Active;

        assert!(matches!(
            validate_project_transition(&original, &updated, &verifier),
            ValidateCallbackResult::Valid
        ));
        assert_transition_invalid(
            validate_project_transition(&original, &updated, &attacker),
            "recorded project verifier",
        );
    }

    #[test]
    fn only_recorded_verifier_can_complete_project() {
        let verifier = fake_agent(1);
        let attacker = fake_agent(2);
        let mut original = valid_project();
        original.status = ProjectStatus::Active;
        original.verifier_did = Some(did_for_author(&verifier));
        let mut updated = original.clone();
        updated.status = ProjectStatus::Completed;

        assert!(matches!(
            validate_project_transition(&original, &updated, &verifier),
            ValidateCallbackResult::Valid
        ));
        assert_transition_invalid(
            validate_project_transition(&original, &updated, &attacker),
            "recorded project verifier",
        );
    }

    #[test]
    fn milestone_completion_binds_verifier_to_update_author() {
        let verifier = fake_agent(3);
        let original = valid_milestone();
        let mut updated = original.clone();
        updated.completed_at = Some(1_800_000_000);
        updated.credits_issued = Some(10.0);
        updated.verified_by = Some(did_for_author(&verifier));
        assert!(matches!(
            validate_milestone_transition(&original, &updated, &verifier),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn milestone_completion_rejects_forged_verifier() {
        let verifier = fake_agent(3);
        let victim = fake_agent(4);
        let original = valid_milestone();
        let mut updated = original.clone();
        updated.completed_at = Some(1_800_000_000);
        updated.verified_by = Some(did_for_author(&victim));
        assert_transition_invalid(
            validate_milestone_transition(&original, &updated, &verifier),
            "committing agent",
        );
    }

    #[test]
    fn milestone_completion_cannot_rewrite_definition() {
        let verifier = fake_agent(3);
        let original = valid_milestone();
        let mut updated = original.clone();
        updated.title = "Different milestone".into();
        updated.completed_at = Some(1_800_000_000);
        updated.verified_by = Some(did_for_author(&verifier));
        assert_transition_invalid(
            validate_milestone_transition(&original, &updated, &verifier),
            "immutable",
        );
    }

    #[test]
    fn completed_milestone_is_terminal() {
        let verifier = fake_agent(3);
        let mut original = valid_milestone();
        original.completed_at = Some(1_800_000_000);
        original.verified_by = Some(did_for_author(&verifier));
        let mut updated = original.clone();
        updated.credits_issued = Some(10.0);
        assert_transition_invalid(
            validate_milestone_transition(&original, &updated, &verifier),
            "cannot be completed",
        );
    }
}