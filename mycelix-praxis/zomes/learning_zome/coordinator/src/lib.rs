// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Learning Coordinator Zome
//!
//! Implements business logic for learning courses and progress tracking.
//! This zome is upgradeable - business logic can change without breaking data.

use hdk::prelude::HdkPathExt;
use hdk::prelude::*;
use learning_integrity::{Course, EntryTypes, LearnerProgress, LearningActivity, LinkTypes};
use mycelix_zome_helpers as _;
use praxis_core::{
    CourseSummary, GET_MY_PROGRESS_FN, LearningContractInfo, ProgressSnapshot, ProgressSyncInput,
    ProgressSyncReceipt, SYNC_PROGRESS_FN,
};
use std::collections::BTreeMap;

/// Create a new course
#[hdk_extern]
pub fn create_course(mut course: Course) -> ExternResult<ActionHash> {
    // 1. TRUST GATING (Vector 1: Resonant Authorship)
    let agent = agent_info()?.agent_initial_pubkey;
    let my_did = format!("did:mycelix:{}", agent);
    // Always the committing agent, never caller-supplied -- otherwise any
    // agent could claim another agent authored their course (P0
    // author-binding gap; integrity validation now enforces this too, see
    // learning_zome integrity's validate_course).
    course.creator = my_did.clone();
    let resonance_response = call(
        CallTargetCell::OtherRole("identity".into()),
        "identity_bridge",
        "get_reputation_score".into(),
        None,
        serde_json::json!({
            "did": my_did,
            "required_resonance": 0.7
        }),
    )?;
    let moral_resonance: f64 = match resonance_response {
        ZomeCallResponse::Ok(bytes) => bytes.decode().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Resonance check failed: {:?}",
                e
            )))
        })?,
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Resonance check call failed".into()
            )));
        }
    };

    if moral_resonance < 0.7 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Course Authorship Rejected: Moral resonance too low. Authors must maintain 0.7+ resonance.".into()
        )));
    }

    // 2. Basic tier gate (Legacy fallback)
    mycelix_bridge_common::gate_civic(
        "edunet_bridge",
        &mycelix_bridge_common::civic_requirement_constitutional(),
        "create_course",
    )?;

    let action_hash = create_entry(EntryTypes::Course(course))?;

    // Create a path anchor for listing all courses
    let path = Path::from("all_courses");
    let path_hash = ensure_path(path, LinkTypes::AllCourses)?;
    create_link(path_hash, action_hash.clone(), LinkTypes::AllCourses, ())?;

    Ok(action_hash)
}

/// Get a course by its action hash
#[hdk_extern]
pub fn get_course(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// List all courses using the path anchor
#[hdk_extern]
pub fn list_courses(_: ()) -> ExternResult<Vec<Record>> {
    // Get the path anchor for all courses
    let path = Path::from("all_courses");
    let path_hash = ensure_path(path, LinkTypes::AllCourses)?;

    // Get all links from the anchor
    let links = get_links(
        LinkQuery::try_new(path_hash, LinkTypes::AllCourses)?,
        GetStrategy::Local,
    )?;

    // Fetch each course record
    let mut courses = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!("Failed to convert link target to ActionHash"))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            courses.push(record);
        }
    }

    Ok(courses)
}

/// Describe the stable client contract implemented by this coordinator.
#[hdk_extern]
pub fn get_learning_contract(_: ()) -> ExternResult<LearningContractInfo> {
    let mut contract = LearningContractInfo::default();
    contract.capabilities.extend(
        [SYNC_PROGRESS_FN, GET_MY_PROGRESS_FN]
            .into_iter()
            .map(str::to_owned),
    );
    Ok(contract)
}

/// List courses through the shared, versioned client projection.
///
/// `list_courses` remains available for zome-internal and legacy callers that
/// need raw Holochain records. Browser clients should use this endpoint so the
/// integrity entry shape is not mistaken for a wire API.
#[hdk_extern]
pub fn list_course_summaries(_: ()) -> ExternResult<Vec<CourseSummary>> {
    list_courses(())?
        .into_iter()
        .map(|record| {
            let course = record
                .entry()
                .to_app_option::<Course>()
                .map_err(|error| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "Failed to decode linked Course entry: {error}"
                    )))
                })?
                .ok_or_else(|| {
                    wasm_error!(WasmErrorInner::Guest(
                        "AllCourses link resolved to a non-Course record".into()
                    ))
                })?;
            Ok(course_summary(course))
        })
        .collect()
}

fn course_summary(course: Course) -> CourseSummary {
    let domain = course
        .metadata
        .as_ref()
        .and_then(|metadata| metadata.get("domain"))
        .and_then(serde_json::Value::as_str)
        .map(str::to_owned)
        .or_else(|| course.tags.first().cloned())
        .unwrap_or_else(|| "General".to_string());

    CourseSummary {
        course_id: course.course_id,
        title: course.title,
        description: course.description,
        domain,
        updated_at: course.updated_at,
    }
}

/// Update an existing course
#[hdk_extern]
pub fn update_course(input: UpdateCourseInput) -> ExternResult<ActionHash> {
    let updated_action_hash = update_entry(
        input.original_action_hash,
        EntryTypes::Course(input.updated_course),
    )?;
    Ok(updated_action_hash)
}

/// Delete a course
#[hdk_extern]
pub fn delete_course(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Record learner progress
#[hdk_extern]
pub fn update_progress(mut progress: LearnerProgress) -> ExternResult<ActionHash> {
    // Always the committing agent, never caller-supplied -- same rationale
    // as create_course's identical fix above.
    let agent = agent_info()?.agent_initial_pubkey;
    progress.learner = format!("did:mycelix:{}", agent);
    progress.last_active = sys_time()?.as_micros();
    write_progress(agent, progress)
}

/// Publish progress using the shared client contract.
///
/// Identity and time are coordinator-derived. A browser cannot attribute an
/// entry to another learner or choose a timestamp that wins latest-value
/// reconciliation.
#[hdk_extern]
pub fn sync_progress(input: ProgressSyncInput) -> ExternResult<ProgressSyncReceipt> {
    let agent = agent_info()?.agent_initial_pubkey;
    let progress = LearnerProgress {
        course_id: input.course_id,
        learner: format!("did:mycelix:{}", agent),
        progress_percent: input.progress_percent,
        completed_items: input.completed_items,
        model_version: input.model_version,
        last_active: sys_time()?.as_micros(),
        metadata: input.metadata,
    };
    let snapshot = progress_snapshot(&progress);
    let action_hash = write_progress(agent, progress)?;

    Ok(ProgressSyncReceipt {
        action_hash: action_hash.to_string(),
        progress: snapshot,
    })
}

fn write_progress(learner: AgentPubKey, progress: LearnerProgress) -> ExternResult<ActionHash> {
    let action_hash = create_entry(EntryTypes::LearnerProgress(progress))?;
    create_link(
        learner,
        action_hash.clone(),
        LinkTypes::LearnerToProgress,
        (),
    )?;
    Ok(action_hash)
}

fn progress_snapshot(progress: &LearnerProgress) -> ProgressSnapshot {
    ProgressSnapshot {
        course_id: progress.course_id.clone(),
        progress_percent: progress.progress_percent,
        completed_items: progress.completed_items.clone(),
        model_version: progress.model_version.clone(),
        last_active: progress.last_active,
    }
}

/// Return the latest authored snapshot for each course or curriculum unit.
#[hdk_extern]
pub fn get_my_progress(_: ()) -> ExternResult<Vec<ProgressSnapshot>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let learner_did = format!("did:mycelix:{}", agent);
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::LearnerToProgress)?,
        GetStrategy::Network,
    )?;

    // BTreeMap makes the wire order deterministic. The action hash is a
    // deterministic tie-breaker if two entries share the same timestamp.
    let mut latest: BTreeMap<String, (i64, String, ProgressSnapshot)> = BTreeMap::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!("LearnerToProgress target was not an ActionHash"))?;
        let Some(record) = get(action_hash.clone(), GetOptions::default())? else {
            continue;
        };
        let progress = record
            .entry()
            .to_app_option::<LearnerProgress>()
            .map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode indexed LearnerProgress: {error}"
                )))
            })?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "LearnerToProgress link resolved to a non-progress record".into()
                ))
            })?;

        // Link validation binds the index to its author; this additional
        // check keeps an accidental self-authored link to another learner's
        // record out of the projection.
        if progress.learner != learner_did {
            continue;
        }

        let key = progress.course_id.0.clone();
        let candidate = (
            progress.last_active,
            action_hash.to_string(),
            progress_snapshot(&progress),
        );
        match latest.get(&key) {
            Some(current) if (current.0, &current.1) >= (candidate.0, &candidate.1) => {}
            _ => {
                latest.insert(key, candidate);
            }
        }
    }

    Ok(latest
        .into_values()
        .map(|(_, _, snapshot)| snapshot)
        .collect())
}

/// Get learner progress
#[hdk_extern]
pub fn get_progress(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// Record a learning activity (private entry for analytics)
#[hdk_extern]
pub fn record_activity(activity: LearningActivity) -> ExternResult<ActionHash> {
    let action_hash = create_entry(EntryTypes::LearningActivity(activity))?;
    Ok(action_hash)
}

/// Enroll in a course
#[hdk_extern]
pub fn enroll(course_action_hash: ActionHash) -> ExternResult<()> {
    let agent = agent_info()?.agent_initial_pubkey;

    // Create bidirectional links for enrollment
    // Course -> Agent
    create_link(
        course_action_hash.clone(),
        agent.clone(),
        LinkTypes::CourseToEnrolled,
        (),
    )?;

    // Agent -> Course
    create_link(agent, course_action_hash, LinkTypes::EnrolledCourses, ())?;

    Ok(())
}

/// Get all courses a learner is enrolled in
#[hdk_extern]
pub fn get_enrolled_courses(_: ()) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;

    // Get all links from agent to courses
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::EnrolledCourses)?,
        GetStrategy::Local,
    )?;

    // Fetch each course record
    let mut courses = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!("Failed to convert link target to ActionHash"))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            courses.push(record);
        }
    }

    Ok(courses)
}

/// Get all learners enrolled in a course
#[hdk_extern]
pub fn get_course_enrollments(course_action_hash: ActionHash) -> ExternResult<Vec<AgentPubKey>> {
    let links = get_links(
        LinkQuery::try_new(course_action_hash, LinkTypes::CourseToEnrolled)?,
        GetStrategy::Local,
    )?;

    let agents = links
        .into_iter()
        .filter_map(|link| AgentPubKey::try_from(link.target).ok())
        .collect();

    Ok(agents)
}

/// Input structure for updating a course
#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateCourseInput {
    pub original_action_hash: ActionHash,
    pub updated_course: Course,
}

fn ensure_path(path: Path, link_type: LinkTypes) -> ExternResult<EntryHash> {
    let typed = path.clone().typed(link_type)?;
    typed.ensure()?;
    typed.path_entry_hash()
}
