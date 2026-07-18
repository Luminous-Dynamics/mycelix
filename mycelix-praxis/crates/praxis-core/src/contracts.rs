// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Stable wire contracts shared by Praxis clients and coordinator zomes.
//!
//! Integrity entries are deliberately not exposed as UI response types. These
//! DTOs are the versioned boundary that both sides serialize, which keeps a
//! zome refactor from silently changing the browser contract.

use serde::{Deserialize, Serialize};

use crate::CourseId;

/// Current version of the learning coordinator's public wire contract.
pub const LEARNING_CONTRACT_VERSION: u16 = 1;

/// Installed app role containing the Praxis DNA.
pub const PRAXIS_ROLE_NAME: &str = "praxis";

/// Coordinator zome serving courses and learner progress.
pub const LEARNING_COORDINATOR_ZOME: &str = "learning_coordinator";

pub const GET_LEARNING_CONTRACT_FN: &str = "get_learning_contract";
pub const LIST_COURSE_SUMMARIES_FN: &str = "list_course_summaries";
pub const SYNC_PROGRESS_FN: &str = "sync_progress";
pub const GET_MY_PROGRESS_FN: &str = "get_my_progress";

/// Current version of the learner dashboard projection.
pub const DASHBOARD_CONTRACT_VERSION: u16 = 1;

/// Coordinator zome that composes read-only dashboard data across Praxis.
pub const INTEGRATION_COORDINATOR_ZOME: &str = "integration_coordinator";

pub const GET_DASHBOARD_SNAPSHOT_FN: &str = "get_dashboard_snapshot";

/// Current version of the browser-facing credential list projection.
pub const CREDENTIAL_CONTRACT_VERSION: u16 = 1;

pub const CREDENTIAL_COORDINATOR_ZOME: &str = "credential_coordinator";
pub const LIST_MY_CREDENTIAL_SUMMARIES_FN: &str = "list_my_credential_summaries";

/// Namespace used when the bundled curriculum treats a topic as an
/// addressable micro-course in the learning progress API.
pub const CURRICULUM_NODE_COURSE_PREFIX: &str = "curriculum-node:";

pub fn curriculum_node_course_id(node_id: &str) -> CourseId {
    CourseId(format!("{CURRICULUM_NODE_COURSE_PREFIX}{node_id}"))
}

/// Runtime-discoverable description of the learning API.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearningContractInfo {
    pub version: u16,
    pub role: String,
    pub coordinator_zome: String,
    pub capabilities: Vec<String>,
}

impl Default for LearningContractInfo {
    fn default() -> Self {
        Self {
            version: LEARNING_CONTRACT_VERSION,
            role: PRAXIS_ROLE_NAME.to_string(),
            coordinator_zome: LEARNING_COORDINATOR_ZOME.to_string(),
            capabilities: vec![LIST_COURSE_SUMMARIES_FN.to_string()],
        }
    }
}

/// Course projection returned to clients.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CourseSummary {
    pub course_id: CourseId,
    pub title: String,
    pub description: String,
    pub domain: String,
    pub updated_at: i64,
}

/// Caller-authenticated request to publish the current learner's progress.
///
/// There is intentionally no learner identity field: the coordinator derives
/// it from `agent_info()` and integrity validation binds it to the author.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProgressSyncInput {
    pub course_id: CourseId,
    pub progress_percent: f32,
    pub completed_items: Vec<String>,
    pub model_version: Option<String>,
    pub metadata: Option<serde_json::Value>,
}

/// Learner-owned progress projection returned to clients.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProgressSnapshot {
    pub course_id: CourseId,
    pub progress_percent: f32,
    pub completed_items: Vec<String>,
    pub model_version: Option<String>,
    pub last_active: i64,
}

/// Write acknowledgement containing both the source-chain address and the
/// canonical projection that can be read back through `get_my_progress`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProgressSyncReceipt {
    pub action_hash: String,
    pub progress: ProgressSnapshot,
}

/// Gamification values that can be displayed without exposing integrity
/// entries or requiring the browser to understand level calculations.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DashboardGamification {
    pub total_xp: u64,
    pub level: u32,
    pub xp_to_next_level: u64,
    pub level_progress_permille: u16,
    pub current_streak: u32,
    pub longest_streak: u32,
    pub streak_bonus_permille: u16,
    pub badges_earned: u32,
    pub freezes_remaining: u8,
}

/// Stable mastery projection. A display name is intentionally absent until
/// the referenced skill catalogue has a shared, authenticated lookup API.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DashboardSkillMastery {
    pub skill_id: String,
    pub mastery_permille: u16,
    pub total_attempts: u32,
    pub modified_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DashboardRecommendationKind {
    NextSkill,
    Review,
    Practice,
    Challenge,
    Course,
    Pod,
    Exploration,
}

/// Read-only recommendation projection. Fetching a dashboard must never
/// generate or persist recommendations as a side effect.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DashboardRecommendation {
    pub target_id: String,
    pub kind: DashboardRecommendationKind,
    pub explanation: String,
    pub rank: u32,
    pub expires_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DashboardActivityKind {
    SrsReview,
    SrsGraduated,
    LessonComplete,
    QuizPassed,
    QuizFailed,
    ProjectSubmit,
    PeerHelp,
    ContentCreated,
    BadgeEarned,
    SkillMastered,
    GoalAchieved,
    StreakMilestone,
    ChallengeComplete,
    DailyLogin,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DashboardActivity {
    pub kind: DashboardActivityKind,
    pub occurred_at: i64,
    pub xp_gained: u32,
}

/// A single, versioned browser-facing dashboard response.
///
/// Optional sections distinguish an unavailable coordinator from a healthy
/// coordinator that returned an empty collection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DashboardSnapshot {
    pub contract_version: u16,
    pub generated_at: i64,
    pub gamification: Option<DashboardGamification>,
    pub due_review_count: Option<u32>,
    pub skills: Option<Vec<DashboardSkillMastery>>,
    pub recommendations: Option<Vec<DashboardRecommendation>>,
    pub recent_activity: Option<Vec<DashboardActivity>>,
}

/// Stable credential projection returned to clients instead of a raw
/// Holochain `Record` or an integrity entry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CredentialSummary {
    pub credential_id: String,
    pub credential_type: Vec<String>,
    pub subject_id: String,
    pub course_id: CourseId,
    pub issuer: String,
    pub issuance_date: String,
    pub expiration_date: Option<String>,
    pub score: Option<f32>,
    pub score_band: String,
    pub proof_type: String,
    pub proof_created: String,
    pub verification_method: String,
    pub proof_purpose: String,
    pub proof_value: String,
    pub status_purpose: Option<String>,
    pub epistemic_empirical: Option<u8>,
    pub epistemic_normative: Option<u8>,
    pub epistemic_materiality: Option<u8>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CredentialList {
    pub contract_version: u16,
    pub credentials: Vec<CredentialSummary>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn learning_contract_advertises_the_shared_names() {
        let contract = LearningContractInfo::default();
        assert_eq!(contract.version, LEARNING_CONTRACT_VERSION);
        assert_eq!(contract.role, PRAXIS_ROLE_NAME);
        assert_eq!(contract.coordinator_zome, LEARNING_COORDINATOR_ZOME);
        assert_eq!(
            contract.capabilities,
            vec![LIST_COURSE_SUMMARIES_FN.to_string()]
        );
    }

    #[test]
    fn progress_contract_round_trips_json() {
        let input = ProgressSyncInput {
            course_id: CourseId("caps-math-10-functions".into()),
            progress_percent: 62.5,
            completed_items: vec!["lesson-1".into()],
            model_version: Some("bkt-v1".into()),
            metadata: Some(serde_json::json!({ "source": "local-browser" })),
        };

        let json = serde_json::to_string(&input).expect("serialize progress contract");
        let decoded: ProgressSyncInput =
            serde_json::from_str(&json).expect("deserialize progress contract");
        assert_eq!(decoded, input);
    }

    #[test]
    fn curriculum_node_ids_have_an_unambiguous_namespace() {
        assert_eq!(
            curriculum_node_course_id("CAPS.Mathematics.Gr12.P1.CALC"),
            CourseId("curriculum-node:CAPS.Mathematics.Gr12.P1.CALC".into())
        );
    }

    #[test]
    fn dashboard_contract_round_trips_partial_availability() {
        let snapshot = DashboardSnapshot {
            contract_version: DASHBOARD_CONTRACT_VERSION,
            generated_at: 42,
            gamification: None,
            due_review_count: Some(0),
            skills: Some(Vec::new()),
            recommendations: None,
            recent_activity: Some(Vec::new()),
        };

        let json = serde_json::to_string(&snapshot).expect("serialize dashboard contract");
        let decoded: DashboardSnapshot =
            serde_json::from_str(&json).expect("deserialize dashboard contract");
        assert_eq!(decoded, snapshot);
    }

    #[test]
    fn credential_contract_preserves_empty_success() {
        let list = CredentialList {
            contract_version: CREDENTIAL_CONTRACT_VERSION,
            credentials: Vec::new(),
        };
        let json = serde_json::to_string(&list).expect("serialize credential contract");
        let decoded: CredentialList =
            serde_json::from_str(&json).expect("deserialize credential contract");
        assert_eq!(decoded, list);
    }
}
