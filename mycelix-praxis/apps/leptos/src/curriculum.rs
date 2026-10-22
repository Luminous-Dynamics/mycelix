// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Curriculum data loader and reactive state provider.
//!
//! Embeds the unified curriculum graph JSON at compile time via `include_str!`.
//! Parses lazily on first access and provides reactive Leptos signals for
//! subject/grade filtering and progress tracking.

use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap};
use std::fmt;
use std::sync::OnceLock;

use crate::persistence;

// ============================================================
// Embedded curriculum data
// ============================================================

const UNIFIED_JSON: &str = include_str!("../../../examples/curriculum/unified_k_to_phd.json");

static UNIFIED_GRAPH: OnceLock<Result<CurriculumGraph, CurriculumLoadError>> = OnceLock::new();

/// A fatal error encountered while loading the embedded curriculum.
///
/// This is surfaced as a diagnostic page at startup instead of panicking and
/// leaving learners with a blank screen.
#[derive(Debug)]
pub struct CurriculumLoadError {
    source: String,
    message: String,
}

impl CurriculumLoadError {
    fn invalid_json(source: impl Into<String>, error: serde_json::Error) -> Self {
        Self {
            source: source.into(),
            message: error.to_string(),
        }
    }
}

impl fmt::Display for CurriculumLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.source, self.message)
    }
}

impl std::error::Error for CurriculumLoadError {}

/// Parse the embedded graph and return a diagnostic error on invalid content.
pub fn try_curriculum_graph() -> Result<&'static CurriculumGraph, &'static CurriculumLoadError> {
    UNIFIED_GRAPH.get_or_init(load_curriculum_graph).as_ref()
}

/// Get the parsed unified graph (lazily initialized and merged with seeds).
///
/// Application startup validates the graph before rendering routes. Code that
/// can run outside the application root should prefer [`try_curriculum_graph`].
pub fn curriculum_graph() -> &'static CurriculumGraph {
    try_curriculum_graph().expect("curriculum was not validated at application startup")
}

fn load_curriculum_graph() -> Result<CurriculumGraph, CurriculumLoadError> {
    let mut base_raw: RawCurriculumDocument = serde_json::from_str(UNIFIED_JSON)
        .map_err(|error| CurriculumLoadError::invalid_json("unified_k_to_phd.json", error))?;

    // Systematic merge of all vocational and professional seeds
    let seeds = vec![
        include_str!("../../../examples/curriculum/vocational/applied_resilience.json"),
        include_str!("../../../examples/curriculum/vocational/mycelial_humanities.json"),
        include_str!("../../../examples/curriculum/vocational/collective_flourishing.json"),
        include_str!("../../../examples/curriculum/vocational/epistemic_sovereignty.json"),
        include_str!("../../../examples/curriculum/vocational/type1_studies.json"),
        include_str!("../../../examples/curriculum/vocational/mycelial_soul.json"),
        include_str!("../../../examples/curriculum/vocational/bioregional_vitality.json"),
        include_str!("../../../examples/curriculum/vocational/robotics_platforms.json"),
        include_str!("../../../examples/curriculum/vocational/habitat_engineering.json"),
        include_str!("../../../examples/curriculum/vocational/warehouse_stewardship.json"),
        include_str!("../../../examples/curriculum/vocational/civilizational_synthesis.json"),
        include_str!("../../../examples/curriculum/vocational/foundational_sovereignty.json"),
        include_str!("../../../examples/curriculum/vocational/substrate_science.json"),
        include_str!("../../../examples/curriculum/vocational/academic_bridge.json"),
        include_str!("../../../examples/curriculum/vocational/sovereign_lineage.json"),
        include_str!("../../../examples/curriculum/vocational/sovereign_it_cyber.json"),
        include_str!("../../../examples/curriculum/vocational/planetary_repair.json"),
        include_str!("../../../examples/curriculum/vocational/cultural_resonance.json"),
        include_str!("../../../examples/curriculum/vocational/computational_cognition.json"),
        include_str!("../../../examples/curriculum/vocational/universal_patterns.json"),
        include_str!("../../../examples/curriculum/vocational/ecosystem_expansion.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs.json"),
        include_str!("../../../examples/curriculum/vocational/global_fruits.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_expanded.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_final.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_master.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_ultimate.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_supreme.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_zenith.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_apex.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_empyrean.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_pragmatic.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_utility.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_everyday.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_essentials.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_omnibus.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_resilient.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_elite.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_trades.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_comprehensive.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_catalyst.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_market.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_momentum.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_momentum_v2.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_revenue.json"),
        include_str!(
            "../../../examples/curriculum/vocational/industry_certs_frontline_trades.json"
        ),
        include_str!("../../../examples/curriculum/vocational/industry_certs_specialty.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_tactical.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_vanguard.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_summit.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_ascendant.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_legacy_bridge.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_nexus.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_apex_today.json"),
        include_str!(
            "../../../examples/curriculum/vocational/industry_certs_practical_prosperity.json"
        ),
        include_str!("../../../examples/curriculum/vocational/industry_certs_global_citizen.json"),
        include_str!("../../../examples/curriculum/vocational/industry_certs_horizon.json"),
        include_str!("../../../examples/curriculum/vocational/universal_wisdom.json"),
        include_str!("../../../examples/curriculum/vocational/living_tradition.json"),
        include_str!("../../../examples/curriculum/vocational/mk0_bootstrapper.json"),
    ];

    for (index, seed_json) in seeds.into_iter().enumerate() {
        let seed_raw: RawCurriculumDocument = serde_json::from_str(seed_json).map_err(|error| {
            CurriculumLoadError::invalid_json(
                format!("embedded vocational curriculum seed {}", index + 1),
                error,
            )
        })?;
        base_raw.nodes.extend(seed_raw.nodes);
        base_raw.edges.extend(seed_raw.edges);
    }

    Ok(CurriculumGraph::from_raw(base_raw))
}

// ============================================================
// Data types
// ============================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RawCurriculumDocument {
    metadata: RawMetadata,
    nodes: Vec<CurriculumNode>,
    edges: Vec<CurriculumEdge>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RawMetadata {
    title: String,
    total_standards: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CurriculumNode {
    pub id: String,
    pub title: String,
    pub description: String,
    pub node_type: String,
    pub difficulty: String,
    pub domain: String,
    pub subdomain: String,
    pub tags: Vec<String>,
    pub estimated_hours: u32,
    pub grade_levels: Vec<String>,
    pub bloom_level: String,
    pub subject_area: String,
    #[serde(default)]
    pub supplementary_resources: Vec<SupplementaryResource>,
    #[serde(default)]
    pub exam_weight: Option<ExamWeight>,
    #[serde(default)]
    pub academic_standards: Vec<AcademicStandard>,
    #[serde(default)]
    pub industry_mappings: Vec<IndustryMapping>,
    #[serde(default)]
    pub holonic_capstone: Option<HolonicCapstone>,
    #[serde(default)]
    pub alignment_vector: Option<AlignmentVector>,
    #[serde(default)]
    pub wisdom_bridge: Option<WisdomBridge>,
    #[serde(default)]
    pub economic_signals: Option<EconomicSignals>,
    #[serde(default)]
    pub biome_parameters: Option<BiomeParameters>,
    #[serde(default)]
    pub abstract_resources: Vec<ResourceProfile>,
    #[serde(default)]
    pub hdc_anchor: Option<HdcAnchor>,
    #[serde(default)]
    pub lineage_anchor: Option<LineageAnchor>,
    #[serde(default)]
    pub gaia_target: Option<GaiaSignal>,
    #[serde(default)]
    pub apoptosis: Option<ApoptosisState>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ApoptosisState {
    pub last_accessed: u64,
    pub usage_velocity: f32,
    pub semantic_weight: u16,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LineageAnchor {
    pub creator_did: String,
    pub mentor_dids: Vec<String>,
    pub generation: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GaiaSignal {
    pub metric_type: String,
    pub target_delta: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResourceProfile {
    pub class: String, // e.g. "DC_Storage_Cell"
    pub min_capacity_mah: Option<u32>,
    pub thermodynamic_threshold: Option<f32>,
    pub common_scraps: Vec<String>,
    #[serde(default)]
    pub embodied_energy_joules: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HdcAnchor {
    pub vector_hash: String, // SHA-256 of the 16k-dimensional vector
    pub semantic_tag: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EconomicSignals {
    pub average_starting_salary: u32,
    pub market_demand: String, // e.g. "Extreme", "High", "Steady"
    pub growth_rate_percent: u8,
    #[serde(default = "default_multiplier")]
    pub local_demand_multiplier: f32, // Controlled by Isibaya Governance
}

fn default_multiplier() -> f32 {
    1.0
}

/// Bioregional applicability metadata for a curriculum node -- which
/// USDA-style hardiness zones this technique/content is calibrated for
/// (see `location::BiomeContext::hardiness_zone`, also `u8`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct BiomeParameters {
    #[serde(default)]
    pub hardiness_zones: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WisdomBridge {
    pub pattern_name: String,
    pub tradition: String,
    pub description: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct AlignmentVector {
    pub ahimsa: u16,
    pub reversibility: u16,
    pub truthfulness: u16,
    pub sovereignty_respect: u16,
    pub reciprocity: u16,
    pub thermodynamic_efficiency: u16,
    pub accountability: u16,
    pub proportionality: u16,
    pub ecological_stewardship: u16,
    pub collective_flourishing: u16,
    pub knowledge_commoning: u16,
    pub mentorship: u16,
    pub aesthetic_harmony: u16,
    pub civic_participation: u16,
    pub innovation_for_good: u16,
    pub resilience: u16,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HolonicCapstone {
    /// Required nodes for this capstone
    pub required_node_ids: Vec<String>,
    /// Deliverable description
    pub deliverable: String,
    /// Minimum peer endorsements required
    pub min_peer_endorsements: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IndustryMapping {
    pub framework: String,
    pub version: String,
    pub code: String,
    pub label: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AcademicStandard {
    pub framework: String,
    pub code: String,
    pub description: String,
    pub grade_level: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExamWeight {
    pub paper: u8,
    pub marks: u16,
    pub total_paper_marks: u16,
    pub percentage: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupplementaryResource {
    pub title: String,
    pub url: String,
    pub source: serde_json::Value, // ResourceSource enum varies
    pub content_type: serde_json::Value,
    pub relevance_score: u8,
    #[serde(default)]
    pub aligned_standard: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CurriculumEdge {
    #[serde(alias = "source")]
    pub from: String,
    #[serde(alias = "target")]
    pub to: String,
    #[serde(alias = "relationship")]
    pub edge_type: String,
    #[serde(default)]
    pub strength_permille: u16,
    #[serde(default)]
    pub rationale: String,
}

/// Parsed and indexed curriculum graph.
pub struct CurriculumGraph {
    pub nodes: Vec<CurriculumNode>,
    pub edges: Vec<CurriculumEdge>,
    pub by_id: HashMap<String, usize>, // node id -> index
    pub by_subject_grade: HashMap<(String, String), Vec<usize>>, // (subject, grade) -> indices
    pub prerequisites: HashMap<String, Vec<String>>, // node id -> prerequisite node ids
    pub dependents: HashMap<String, Vec<String>>, // node id -> nodes that depend on it
    /// Cross-subject edges (AppliedIn/RelatedTo) indexed by node id
    pub cross_subject: HashMap<String, Vec<usize>>, // node id -> edge indices
}

impl CurriculumGraph {
    fn from_raw(raw: RawCurriculumDocument) -> Self {
        let mut by_id = HashMap::new();
        let mut by_subject_grade: HashMap<(String, String), Vec<usize>> = HashMap::new();
        let mut prerequisites: HashMap<String, Vec<String>> = HashMap::new();
        let mut dependents: HashMap<String, Vec<String>> = HashMap::new();

        // Normalize subject names (collapse spaces for consistency —
        // CAPS JSON has both "EnglishLanguageArts" and "English Language Arts")
        let mut nodes = raw.nodes;
        for node in &mut nodes {
            node.subject_area = node.subject_area.replace(' ', "");
        }

        for (i, node) in nodes.iter().enumerate() {
            by_id.insert(node.id.clone(), i);
            let grade = node.grade_levels.first().cloned().unwrap_or_default();
            by_subject_grade
                .entry((node.subject_area.clone(), grade))
                .or_default()
                .push(i);
        }

        let mut cross_subject: HashMap<String, Vec<usize>> = HashMap::new();

        for (edge_idx, edge) in raw.edges.iter().enumerate() {
            prerequisites
                .entry(edge.to.clone())
                .or_default()
                .push(edge.from.clone());
            dependents
                .entry(edge.from.clone())
                .or_default()
                .push(edge.to.clone());

            // Index cross-subject edges (AppliedIn/RelatedTo)
            if edge.edge_type == "AppliedIn" || edge.edge_type == "RelatedTo" {
                cross_subject
                    .entry(edge.from.clone())
                    .or_default()
                    .push(edge_idx);
                cross_subject
                    .entry(edge.to.clone())
                    .or_default()
                    .push(edge_idx);
            }
        }

        CurriculumGraph {
            nodes,
            edges: raw.edges,
            by_id,
            by_subject_grade,
            prerequisites,
            dependents,
            cross_subject,
        }
    }

    /// Get nodes for a subject + grade combination.
    pub fn nodes_for(&self, subject: &str, grade: &str) -> Vec<&CurriculumNode> {
        self.by_subject_grade
            .get(&(subject.to_string(), grade.to_string()))
            .map(|indices| indices.iter().map(|&i| &self.nodes[i]).collect())
            .unwrap_or_default()
    }

    /// Get prerequisite node IDs for a given node.
    pub fn prereqs_for(&self, node_id: &str) -> Vec<&str> {
        self.prerequisites
            .get(node_id)
            .map(|v| v.iter().map(|s| s.as_str()).collect())
            .unwrap_or_default()
    }

    /// Get a node by ID.
    pub fn node(&self, id: &str) -> Option<&CurriculumNode> {
        self.by_id.get(id).map(|&i| &self.nodes[i])
    }

    /// Get cross-subject connections for a node (AppliedIn/RelatedTo edges).
    /// Returns (neighbor_node, edge, is_outgoing) tuples.
    pub fn cross_subject_neighbors(
        &self,
        node_id: &str,
    ) -> Vec<(&CurriculumNode, &CurriculumEdge)> {
        self.cross_subject
            .get(node_id)
            .map(|edge_indices| {
                edge_indices
                    .iter()
                    .filter_map(|&ei| {
                        let edge = &self.edges[ei];
                        let neighbor_id = if edge.from == node_id {
                            &edge.to
                        } else {
                            &edge.from
                        };
                        let neighbor = self.node(neighbor_id)?;
                        Some((neighbor, edge))
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Find the highest-priority unmastered node whose prerequisites are all met.
    ///
    /// Sorts by `exam_weight.percentage` descending when available, then falls
    /// back to node order.  Returns the first node that is (a) not in
    /// `mastered_ids` and (b) has every prerequisite satisfied (i.e. all prereqs
    /// are in `mastered_ids`).  If no such node exists, returns the first
    /// unmastered node with no prerequisites at all.
    pub fn recommended_next(
        &self,
        progression_ready_ids: &std::collections::HashSet<String>,
    ) -> Option<&str> {
        // Build a list of (index, exam_weight_pct) sorted by weight desc
        let mut candidates: Vec<(usize, f32)> = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| !progression_ready_ids.contains(&n.id))
            .map(|(i, n)| {
                let weight = n.exam_weight.as_ref().map(|w| w.percentage).unwrap_or(0.0);
                (i, weight)
            })
            .collect();
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // First pass: highest-weight not-yet-ready node with progression-ready prerequisites
        for &(idx, _) in &candidates {
            let node = &self.nodes[idx];
            let prereqs = self.prereqs_for(&node.id);
            if prereqs.iter().all(|p| progression_ready_ids.contains(*p)) {
                return Some(&node.id);
            }
        }

        // Fallback: first not-yet-ready node with no prerequisites
        for &(idx, _) in &candidates {
            let node = &self.nodes[idx];
            if self.prereqs_for(&node.id).is_empty() {
                return Some(&node.id);
            }
        }

        None
    }

    /// Get all not-yet-ready nodes whose prerequisites are progression-ready (the Zone of Proximal Development).
    pub fn zpd_nodes(
        &self,
        progression_ready_ids: &std::collections::HashSet<String>,
    ) -> Vec<&CurriculumNode> {
        self.nodes
            .iter()
            .filter(|n| !progression_ready_ids.contains(&n.id))
            .filter(|n| {
                let prereqs = self.prereqs_for(&n.id);
                prereqs.is_empty() || prereqs.iter().all(|p| progression_ready_ids.contains(*p))
            })
            .collect()
    }

    /// All unique subjects.
    pub fn subjects(&self) -> Vec<&str> {
        let mut subjects: Vec<&str> = self.nodes.iter().map(|n| n.subject_area.as_str()).collect();
        subjects.sort();
        subjects.dedup();
        subjects
    }

    /// All unique grades.
    pub fn grades(&self) -> Vec<&str> {
        let mut grades: Vec<&str> = self
            .nodes
            .iter()
            .flat_map(|n| n.grade_levels.iter().map(|g| g.as_str()))
            .collect();
        grades.sort();
        grades.dedup();
        grades
    }
}

// ============================================================
// Progress tracking
// ============================================================

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ScoredPracticeReceipt {
    /// Opaque hash of node ID, prompt, answer, and normalized answer-choice set.
    /// Raw learner text or selected choice is not stored in this record.
    pub item_id: String,
    pub correct: bool,
    pub recorded_at_ms: f64,
    /// Monotonic within the node on one local profile; timestamps dominate
    /// across imports, with item_id providing a deterministic final tie-break.
    pub sequence: u64,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NodeProgress {
    /// Estimated mastery from scored answer choices only (0-1000).
    pub mastery_permille: u16,
    /// Answer choices submitted before feedback was revealed.
    pub attempts: u32,
    /// Correct answer choices submitted before feedback was revealed.
    pub correct: u32,
    pub status: ProgressStatus,
    pub last_reviewed: Option<f64>, // JS timestamp
    /// Self-reported confidence is retained separately and never feeds BKT.
    #[serde(default)]
    pub self_assessment: Option<SelfAssessment>,
    /// Legacy responses recorded after answer reveal; not valid mastery evidence.
    #[serde(default)]
    pub self_reported_attempts: u32,
    #[serde(default)]
    pub self_reported_correct: u32,
    /// Completed interactive activities; this is exposure/participation, not mastery.
    #[serde(default)]
    pub activity_completions: u32,
    /// Stable identifiers of practice items already counted by BKT.
    /// Replays of the same prompt must not inflate the posterior.
    #[serde(default)]
    pub scored_item_ids: Vec<String>,
    /// Private local evidence ledger for answer-first scored choices. It stores
    /// an opaque item hash, outcome and time—not the student's selected text.
    #[serde(default)]
    pub scored_practice_receipts: Vec<ScoredPracticeReceipt>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelfAssessment {
    FeelsUnderstood,
    NeedsMorePractice,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProgressStatus {
    #[default]
    NotStarted,
    Studying,
    /// A BKT estimate based on recent answered items, not durable mastery.
    ProvisionalMastery,
    /// Reserved for mastery verified by an independent, evidence-backed workflow.
    Mastered,
}

impl ProgressStatus {
    pub fn label(&self) -> &'static str {
        match self {
            ProgressStatus::NotStarted => "Not Started",
            ProgressStatus::Studying => "Studying",
            ProgressStatus::ProvisionalMastery => "Provisional Mastery",
            ProgressStatus::Mastered => "Verified Mastery",
        }
    }

    pub fn css_class(&self) -> &'static str {
        match self {
            ProgressStatus::NotStarted => "status-not-started",
            ProgressStatus::Studying => "status-studying",
            ProgressStatus::ProvisionalMastery => "status-provisional-mastery",
            ProgressStatus::Mastered => "status-mastered",
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProgressStore {
    /// Version of the local progress semantics/migration.
    #[serde(default)]
    pub schema_version: u32,
    pub nodes: HashMap<String, NodeProgress>,
    pub exam_date: Option<String>, // ISO date
    /// SRS flashcard state — keyed by card_id (e.g., "math:3")
    #[serde(default)]
    pub srs_cards: HashMap<String, SrsCardState>,
    /// BKT adaptive difficulty state — keyed by node_id
    #[serde(default)]
    pub bkt_states: HashMap<String, BktState>,
}

impl Default for ProgressStore {
    fn default() -> Self {
        Self {
            // Fresh in-memory stores use the current schema. Missing schema_version
            // in deserialized legacy JSON still defaults to zero via serde.
            schema_version: 2,
            nodes: HashMap::new(),
            exam_date: None,
            srs_cards: HashMap::new(),
            bkt_states: HashMap::new(),
        }
    }
}

/// SM-2 spaced repetition state for a single flashcard.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SrsCardState {
    pub ease_factor: f32,    // starts at 2.5
    pub interval_days: u32,  // current interval
    pub repetitions: u32,    // consecutive correct
    pub next_review_ms: f64, // JS timestamp of next review
}

impl Default for SrsCardState {
    fn default() -> Self {
        Self {
            ease_factor: 2.5,
            interval_days: 0,
            repetitions: 0,
            next_review_ms: 0.0,
        }
    }
}

impl SrsCardState {
    /// SM-2 update: quality 0-5 (0-1 = blackout, 2 = hard, 3-4 = good, 5 = easy)
    pub fn update(&mut self, quality: u8) {
        let q = quality as f32;
        if quality >= 3 {
            // Correct response
            self.interval_days = match self.repetitions {
                0 => 1,
                1 => 6,
                _ => (self.interval_days as f32 * self.ease_factor) as u32,
            };
            self.repetitions += 1;
        } else {
            // Incorrect — reset
            self.repetitions = 0;
            self.interval_days = 1;
        }
        // Update ease factor
        self.ease_factor += 0.1 - (5.0 - q) * (0.08 + (5.0 - q) * 0.02);
        if self.ease_factor < 1.3 {
            self.ease_factor = 1.3;
        }

        // Set next review time
        let now = js_sys::Date::now();
        self.next_review_ms = now + (self.interval_days as f64 * 24.0 * 60.0 * 60.0 * 1000.0);
    }

    /// Is this card due for review?
    pub fn is_due(&self) -> bool {
        js_sys::Date::now() >= self.next_review_ms
    }
}

impl ProgressStore {
    pub fn get(&self, node_id: &str) -> &NodeProgress {
        static DEFAULT: NodeProgress = NodeProgress {
            mastery_permille: 0,
            attempts: 0,
            correct: 0,
            status: ProgressStatus::NotStarted,
            last_reviewed: None,
            self_assessment: None,
            self_reported_attempts: 0,
            self_reported_correct: 0,
            activity_completions: 0,
            scored_item_ids: Vec::new(),
            scored_practice_receipts: Vec::new(),
        };
        self.nodes.get(node_id).unwrap_or(&DEFAULT)
    }

    /// Change the learner-facing progress state. A client-side request to mark
    /// a node Mastered is deliberately downgraded to a self-assessment; only a
    /// trusted evidence workflow may restore the verified Mastered state.
    pub fn set_status(&mut self, node_id: &str, status: ProgressStatus) {
        if status == ProgressStatus::Mastered {
            let bkt = self.bkt_states.get(node_id).cloned();
            let entry = self.nodes.entry(node_id.to_string()).or_default();
            if entry.status == ProgressStatus::Mastered {
                entry.self_reported_attempts = entry
                    .self_reported_attempts
                    .saturating_add(entry.attempts);
                entry.self_reported_correct = entry
                    .self_reported_correct
                    .saturating_add(entry.correct.min(entry.attempts));
                entry.attempts = 0;
                entry.correct = 0;
            }
            entry.self_assessment = Some(SelfAssessment::FeelsUnderstood);
            entry.self_reported_attempts = entry.self_reported_attempts.saturating_add(1);
            entry.self_reported_correct = entry.self_reported_correct.saturating_add(1);
            entry.mastery_permille = bkt
                .as_ref()
                .map_or(0, |state| (state.p_mastery * 1000.0) as u16);
            entry.status = if bkt.as_ref().is_some_and(BktState::is_likely_mastered) {
                ProgressStatus::ProvisionalMastery
            } else {
                ProgressStatus::Studying
            };
            return;
        }

        if status == ProgressStatus::ProvisionalMastery {
            let bkt = self.bkt_states.get(node_id).cloned();
            let entry = self.nodes.entry(node_id.to_string()).or_default();
            entry.mastery_permille = bkt
                .as_ref()
                .map_or(0, |state| (state.p_mastery * 1000.0) as u16);
            entry.status = if bkt.as_ref().is_some_and(BktState::is_likely_mastered) {
                ProgressStatus::ProvisionalMastery
            } else {
                ProgressStatus::Studying
            };
            entry.self_assessment = None;
            return;
        }

        let entry = self.nodes.entry(node_id.to_string()).or_default();
        entry.status = status;
        if matches!(status, ProgressStatus::NotStarted | ProgressStatus::Studying) {
            entry.self_assessment = None;
        }
    }

    /// Record a learner reflection separately from scored answers. This never
    /// changes the BKT posterior or correctness counts.
    pub fn record_self_assessment(&mut self, node_id: &str, assessment: SelfAssessment) {
        let bkt = self.bkt_states.get(node_id).cloned();
        let entry = self.nodes.entry(node_id.to_string()).or_default();
        if entry.status == ProgressStatus::Mastered {
            entry.self_reported_attempts = entry
                .self_reported_attempts
                .saturating_add(entry.attempts);
            entry.self_reported_correct = entry
                .self_reported_correct
                .saturating_add(entry.correct.min(entry.attempts));
            entry.attempts = 0;
            entry.correct = 0;
        }
        entry.self_assessment = Some(assessment);
        entry.self_reported_attempts = entry.self_reported_attempts.saturating_add(1);
        if assessment == SelfAssessment::FeelsUnderstood {
            entry.self_reported_correct = entry.self_reported_correct.saturating_add(1);
        }
        entry.last_reviewed = Some(js_sys::Date::now());
        if matches!(
            entry.status,
            ProgressStatus::NotStarted | ProgressStatus::Mastered
        ) {
            entry.status = if bkt.as_ref().is_some_and(BktState::is_likely_mastered) {
                ProgressStatus::ProvisionalMastery
            } else {
                ProgressStatus::Studying
            };
        }
        entry.mastery_permille = bkt
            .as_ref()
            .map_or(0, |state| (state.p_mastery * 1000.0) as u16);
    }

    /// One-time migration: previous Praxis versions recorded BKT responses only
    /// after showing the answer and allowed a manual Mastered toggle. Such data
    /// is preserved as self-report history, but cannot be treated as scored
    /// attempts, a BKT estimate, or verified mastery.
    pub fn migrate_legacy_progress(&mut self) -> bool {
        let mut changed = false;

        // Version 0 recorded a self-reported result only after showing the key.
        // Preserve those counters as reflection history, except where there is
        // already an item-level receipt from the answer-first scoring path.
        if self.schema_version < 1 {
            let node_ids = self
                .nodes
                .keys()
                .chain(self.bkt_states.keys())
                .cloned()
                .collect::<BTreeSet<_>>();
            for node_id in node_ids {
                let old_bkt = self.bkt_states.get(&node_id).cloned();
                let progress = self.nodes.entry(node_id.clone()).or_default();
                let receipt_count = progress.scored_practice_receipts.len() as u32;
                let receipt_correct = progress
                    .scored_practice_receipts
                    .iter()
                    .filter(|receipt| receipt.correct)
                    .count() as u32;
                let old_attempts = progress
                    .attempts
                    .max(old_bkt.as_ref().map_or(0, |bkt| bkt.attempts));
                let old_correct = progress
                    .correct
                    .max(old_bkt.as_ref().map_or(0, |bkt| bkt.correct))
                    .min(old_attempts);
                let unbound_attempts = old_attempts.saturating_sub(receipt_count);
                let unbound_correct = old_correct
                    .saturating_sub(receipt_correct)
                    .min(unbound_attempts);
                progress.self_reported_attempts = progress
                    .self_reported_attempts
                    .saturating_add(unbound_attempts);
                progress.self_reported_correct = progress
                    .self_reported_correct
                    .saturating_add(unbound_correct);
                if progress.status == ProgressStatus::Mastered {
                    progress.self_assessment = Some(SelfAssessment::FeelsUnderstood);
                }
                progress.attempts = receipt_count;
                progress.correct = receipt_correct;
                progress.scored_item_ids = progress
                    .scored_practice_receipts
                    .iter()
                    .map(|receipt| receipt.item_id.clone())
                    .collect();
                progress.mastery_permille = 0;
                progress.status = if progress.scored_practice_receipts.is_empty() {
                    if old_attempts > 0
                        || matches!(
                            progress.status,
                            ProgressStatus::Mastered | ProgressStatus::ProvisionalMastery
                        )
                    {
                        ProgressStatus::Studying
                    } else {
                        progress.status
                    }
                } else {
                    ProgressStatus::Studying
                };
                self.bkt_states.remove(&node_id);
            }
            self.schema_version = 1;
            changed = true;
        }

        // Version 1 can contain aggregate BKT numbers without enough item-level
        // evidence to deduplicate imports or audit which prompts produced them.
        // Move only that unbound aggregate to self-report history; receipt-backed
        // attempts remain scored and are reconstructed from the receipt ledger.
        if self.schema_version < 2 {
            let node_ids = self
                .nodes
                .keys()
                .chain(self.bkt_states.keys())
                .cloned()
                .collect::<BTreeSet<_>>();
            for node_id in node_ids {
                let old_bkt = self.bkt_states.get(&node_id).cloned();
                let progress = self.nodes.entry(node_id.clone()).or_default();
                let mut receipts = progress.scored_practice_receipts.clone();
                receipts.sort_by(|left, right| {
                    left.recorded_at_ms
                        .total_cmp(&right.recorded_at_ms)
                        .then_with(|| left.sequence.cmp(&right.sequence))
                        .then_with(|| left.item_id.cmp(&right.item_id))
                });
                let mut seen = BTreeSet::new();
                receipts.retain(|receipt| seen.insert(receipt.item_id.clone()));
                let receipt_count = receipts.len() as u32;
                let receipt_correct =
                    receipts.iter().filter(|receipt| receipt.correct).count() as u32;
                let old_attempts = progress
                    .attempts
                    .max(old_bkt.as_ref().map_or(0, |bkt| bkt.attempts));
                let old_correct = progress
                    .correct
                    .max(old_bkt.as_ref().map_or(0, |bkt| bkt.correct))
                    .min(old_attempts);
                let unbound_attempts = old_attempts.saturating_sub(receipt_count);
                let unbound_correct = old_correct
                    .saturating_sub(receipt_correct)
                    .min(unbound_attempts);
                progress.self_reported_attempts = progress
                    .self_reported_attempts
                    .saturating_add(unbound_attempts);
                progress.self_reported_correct = progress
                    .self_reported_correct
                    .saturating_add(unbound_correct);
                if progress.status == ProgressStatus::Mastered {
                    progress.self_assessment = Some(SelfAssessment::FeelsUnderstood);
                }
                progress.scored_practice_receipts = receipts;
                progress.scored_item_ids = progress
                    .scored_practice_receipts
                    .iter()
                    .map(|receipt| receipt.item_id.clone())
                    .collect();
                progress.attempts = receipt_count;
                progress.correct = receipt_correct;
                progress.mastery_permille = 0;
                if progress.scored_practice_receipts.is_empty()
                    && (old_attempts > 0
                        || old_correct > 0
                        || matches!(
                            progress.status,
                            ProgressStatus::Mastered | ProgressStatus::ProvisionalMastery
                        ))
                {
                    progress.status = ProgressStatus::Studying;
                } else if !progress.scored_practice_receipts.is_empty() {
                    progress.status = ProgressStatus::Studying;
                }
                self.bkt_states.remove(&node_id);
            }
            self.schema_version = 2;
            changed = true;
        }

        // No browser-local state can issue verified mastery. A stale Mastered
        // flag is a self-assessment; if it has item receipts, those receipts can
        // still produce a provisional navigation estimate after the next rebuild.
        let unverified_mastered_ids = self
            .nodes
            .iter()
            .filter(|(_, progress)| progress.status == ProgressStatus::Mastered)
            .map(|(node_id, _)| node_id.clone())
            .collect::<Vec<_>>();
        for node_id in unverified_mastered_ids {
            if let Some(progress) = self.nodes.get_mut(&node_id) {
                if progress.scored_practice_receipts.is_empty() {
                    progress.self_reported_attempts = progress
                        .self_reported_attempts
                        .saturating_add(progress.attempts);
                    progress.self_reported_correct = progress
                        .self_reported_correct
                        .saturating_add(progress.correct.min(progress.attempts));
                    progress.attempts = 0;
                    progress.correct = 0;
                    progress.mastery_permille = 0;
                    progress.scored_item_ids.clear();
                }
                progress.self_assessment = Some(SelfAssessment::FeelsUnderstood);
                progress.status = ProgressStatus::Studying;
            }
            self.bkt_states.remove(&node_id);
            changed = true;
        }

        // Scores, counts and provisional state are derived from item-level
        // receipts only. Unbound attempts and BKT snapshots cannot unlock the ZPD.
        let receipt_node_ids = self
            .nodes
            .iter()
            .filter(|(_, progress)| !progress.scored_practice_receipts.is_empty())
            .map(|(node_id, _)| node_id.clone())
            .collect::<Vec<_>>();
        for node_id in receipt_node_ids {
            changed |= self.rebuild_bkt_from_practice_receipts(&node_id);
        }

        // Remove BKT-only progress records that have no scored-item receipts.
        for node_id in self.bkt_states.keys().cloned().collect::<Vec<_>>() {
            if !self.nodes.contains_key(&node_id) {
                self.nodes.insert(node_id, NodeProgress::default());
                changed = true;
            }
        }

        let unbound_node_ids = self
            .nodes
            .iter()
            .filter(|(node_id, progress)| {
                progress.scored_practice_receipts.is_empty()
                    && (progress.attempts > 0
                        || progress.correct > 0
                        || progress.mastery_permille > 0
                        || !progress.scored_item_ids.is_empty()
                        || matches!(
                            progress.status,
                            ProgressStatus::ProvisionalMastery | ProgressStatus::Mastered
                        )
                        || self.bkt_states.contains_key(*node_id))
            })
            .map(|(node_id, _)| node_id.clone())
            .collect::<Vec<_>>();
        for node_id in unbound_node_ids {
            let old_bkt = self.bkt_states.remove(&node_id);
            if let Some(progress) = self.nodes.get_mut(&node_id) {
                progress.self_reported_attempts = progress
                    .self_reported_attempts
                    .saturating_add(progress.attempts.max(old_bkt.as_ref().map_or(0, |bkt| bkt.attempts)));
                progress.self_reported_correct = progress
                    .self_reported_correct
                    .saturating_add(progress.correct.max(old_bkt.as_ref().map_or(0, |bkt| bkt.correct)));
                let had_unbound_attempts = progress.attempts > 0
                    || old_bkt.as_ref().is_some_and(|bkt| bkt.attempts > 0);
                progress.attempts = 0;
                progress.correct = 0;
                progress.mastery_permille = 0;
                progress.scored_item_ids.clear();
                if had_unbound_attempts
                    || matches!(
                        progress.status,
                        ProgressStatus::ProvisionalMastery | ProgressStatus::Mastered
                    )
                {
                    progress.status = ProgressStatus::Studying;
                }
            }
            changed = true;
        }

        changed
    }

    pub fn mastered_count(&self) -> usize {
        self.nodes
            .values()
            .filter(|p| p.status == ProgressStatus::Mastered)
            .count()
    }

    pub fn studying_count(&self) -> usize {
        self.nodes
            .values()
            .filter(|p| p.status == ProgressStatus::Studying)
            .count()
    }

    /// Mastery permille for a subject (0-1000).
    pub fn subject_mastery(&self, graph: &CurriculumGraph, subject: &str) -> u16 {
        let nodes: Vec<&CurriculumNode> = graph
            .nodes
            .iter()
            .filter(|n| n.subject_area == subject)
            .collect();
        if nodes.is_empty() {
            return 0;
        }
        let total: u32 = nodes
            .iter()
            .map(|n| self.get(&n.id).mastery_permille as u32)
            .sum();
        (total / nodes.len() as u32) as u16
    }

    /// IDs whose prerequisites may be treated as provisionally satisfied for
    /// learning navigation. This deliberately includes a BKT provisional estimate,
    /// but must never be used for credentials, safety authorization or claims of
    /// verified/durable mastery.
    pub fn progression_ready_ids(&self) -> std::collections::HashSet<String> {
        self.nodes
            .iter()
            .filter(|(_, progress)| {
                matches!(
                    progress.status,
                    ProgressStatus::ProvisionalMastery | ProgressStatus::Mastered
                )
            })
            .map(|(id, _)| id.clone())
            .collect()
    }

    /// IDs with independently verified mastery receipts. Browser-only BKT and
    /// self-assessment cannot populate this set.
    pub fn mastered_ids(&self) -> std::collections::HashSet<String> {
        self.nodes
            .iter()
            .filter(|(_, p)| p.status == ProgressStatus::Mastered)
            .map(|(id, _)| id.clone())
            .collect()
    }

    pub fn provisional_mastery_count(&self) -> usize {
        self.nodes
            .values()
            .filter(|progress| progress.status == ProgressStatus::ProvisionalMastery)
            .count()
    }

    pub fn progression_ready_count(&self) -> usize {
        self.progression_ready_ids().len()
    }
}

// ============================================================
// Bayesian Knowledge Tracing (BKT) — adaptive difficulty engine
// ============================================================

/// BKT parameters for a topic — calibrated for educational assessment.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BktState {
    /// P(L) — probability the student has learned/mastered the skill
    pub p_mastery: f32,
    /// Total practice attempts on this topic
    pub attempts: u32,
    /// Correct responses
    pub correct: u32,
}

impl Default for BktState {
    fn default() -> Self {
        Self {
            p_mastery: 0.1, // P(L₀) — low prior, student hasn't demonstrated knowledge yet
            attempts: 0,
            correct: 0,
        }
    }
}

impl BktState {
    // BKT parameters (Corbett & Anderson, 1995)
    const P_TRANSIT: f32 = 0.1; // P(T) — probability of learning per opportunity
    const P_SLIP: f32 = 0.1; // P(S) — probability of slip (knows but answers wrong)
    const P_GUESS: f32 = 0.25; // P(G) — probability of guessing correctly

    /// Update P(mastery) after observing a response (correct or incorrect).
    pub fn update(&mut self, correct: bool) {
        self.attempts += 1;
        if correct {
            self.correct += 1;
        }

        // BKT posterior update
        let p_l = self.p_mastery;
        if correct {
            // P(L|correct) = P(L) * (1 - P(S)) / P(correct)
            let p_correct = p_l * (1.0 - Self::P_SLIP) + (1.0 - p_l) * Self::P_GUESS;
            let p_l_given_correct = p_l * (1.0 - Self::P_SLIP) / p_correct;
            // Apply transition: P(L_new) = P(L|obs) + (1 - P(L|obs)) * P(T)
            self.p_mastery = p_l_given_correct + (1.0 - p_l_given_correct) * Self::P_TRANSIT;
        } else {
            // P(L|incorrect) = P(L) * P(S) / P(incorrect)
            let p_incorrect = p_l * Self::P_SLIP + (1.0 - p_l) * (1.0 - Self::P_GUESS);
            let p_l_given_incorrect = p_l * Self::P_SLIP / p_incorrect;
            // Apply transition
            self.p_mastery = p_l_given_incorrect + (1.0 - p_l_given_incorrect) * Self::P_TRANSIT;
        }

        // Clamp to [0.01, 0.99]
        self.p_mastery = self.p_mastery.clamp(0.01, 0.99);
    }

    /// Recommended difficulty tier (0-3) based on current mastery estimate.
    /// 0 = gentle soil, 1 = steady ground, 2 = rocky terrain, 3 = mountain path
    pub fn recommended_difficulty(&self) -> u8 {
        match self.p_mastery {
            p if p < 0.3 => 0,  // Still learning — gentle problems
            p if p < 0.5 => 1,  // Getting it — steady challenge
            p if p < 0.75 => 2, // Strong — push with harder problems
            _ => 3,             // Near mastery — mountain paths only
        }
    }

    /// Has the student likely mastered this topic? (P(L) > 0.95)
    pub fn is_likely_mastered(&self) -> bool {
        self.p_mastery > 0.95
    }

    /// Terrain label for current difficulty recommendation.
    pub fn terrain_label(&self) -> &'static str {
        match self.recommended_difficulty() {
            0 => "Gentle soil",
            1 => "Steady ground",
            2 => "Rocky terrain",
            _ => "Mountain path",
        }
    }
}

impl ProgressStore {
    /// Get or create BKT state for a topic.
    pub fn bkt(&self, node_id: &str) -> BktState {
        self.bkt_states.get(node_id).cloned().unwrap_or_default()
    }

    /// Record a practice response and update BKT.
    pub fn record_response(&mut self, node_id: &str, correct: bool) {
        let bkt = self.bkt_states.entry(node_id.to_string()).or_default();
        bkt.update(correct);

        // Also update basic progress
        let progress = self.nodes.entry(node_id.to_string()).or_default();
        // A new scored observation supersedes a prior confidence-only reflection.
        progress.self_assessment = None;
        progress.attempts += 1;
        if correct {
            progress.correct += 1;
        }
        progress.mastery_permille = (bkt.p_mastery * 1000.0) as u16;

        // Browser-scored practice can only grant provisional mastery.
        // A correct-response streak never preserves or creates verified Mastered.
        if bkt.is_likely_mastered() {
            progress.status = ProgressStatus::ProvisionalMastery;
        } else if bkt.attempts > 0 {
            progress.status = ProgressStatus::Studying;
        }
    }

    /// Record a scored response only once per stable item identifier. This
    /// prevents replaying the same prompt from repeatedly inflating BKT. Returns
    /// false when that item was already scored for this node.
    pub fn record_response_for_item(
        &mut self,
        node_id: &str,
        item_id: &str,
        correct: bool,
    ) -> bool {
        let progress = self.nodes.entry(node_id.to_string()).or_default();
        if progress.scored_item_ids.iter().any(|seen| seen == item_id)
            || progress
                .scored_practice_receipts
                .iter()
                .any(|receipt| receipt.item_id == item_id)
        {
            return false;
        }

        progress.scored_item_ids.push(item_id.to_owned());
        progress.last_reviewed = Some(current_timestamp_ms());
        let sequence = progress
            .scored_practice_receipts
            .iter()
            .map(|receipt| receipt.sequence)
            .max()
            .unwrap_or(0)
            .saturating_add(1);
        progress.scored_practice_receipts.push(ScoredPracticeReceipt {
            item_id: item_id.to_owned(),
            correct,
            recorded_at_ms: js_sys::Date::now(),
            sequence,
        });
        self.rebuild_bkt_from_practice_receipts(node_id);
        true
    }

    /// Rebuild the BKT posterior deterministically from the merged, deduplicated
    /// response ledger. The ledger contains outcomes but no raw learner answers.
    pub fn rebuild_bkt_from_practice_receipts(&mut self, node_id: &str) -> bool {
        let mut receipts = self
            .nodes
            .get(node_id)
            .map(|progress| progress.scored_practice_receipts.clone())
            .unwrap_or_default();
        receipts.sort_by(|left, right| {
            left.recorded_at_ms
                .total_cmp(&right.recorded_at_ms)
                .then_with(|| left.sequence.cmp(&right.sequence))
                .then_with(|| left.item_id.cmp(&right.item_id))
        });

        // Keep only the first, time-ordered observation for each item ID even
        // if a corrupted/imported browser record contains duplicate receipts.
        let mut seen = BTreeSet::new();
        receipts.retain(|receipt| seen.insert(receipt.item_id.clone()));
        if receipts.is_empty() {
            return false;
        }

        let receipt_ids = receipts
            .iter()
            .map(|receipt| receipt.item_id.clone())
            .collect::<BTreeSet<_>>();
        let mut rebuilt = BktState::default();
        for receipt in &receipts {
            rebuilt.update(receipt.correct);
        }

        let old_bkt = self.bkt_states.get(node_id);
        let bkt_changed = old_bkt.is_none_or(|old| {
            old.p_mastery != rebuilt.p_mastery
                || old.attempts != rebuilt.attempts
                || old.correct != rebuilt.correct
        });
        let progress = self.nodes.entry(node_id.to_owned()).or_default();
        let new_mastery = (rebuilt.p_mastery * 1000.0) as u16;
        let new_status = if rebuilt.is_likely_mastered() {
            ProgressStatus::ProvisionalMastery
        } else {
            ProgressStatus::Studying
        };
        let progress_changed = progress.scored_practice_receipts != receipts
            || progress.attempts != rebuilt.attempts
            || progress.correct != rebuilt.correct
            || progress.mastery_permille != new_mastery
            || progress.status != new_status
            || progress.self_assessment.is_some()
            || progress.scored_item_ids.len() != receipt_ids.len()
            || progress
                .scored_item_ids
                .iter()
                .any(|item_id| !receipt_ids.contains(item_id))
            || progress.scored_practice_receipts.len() != receipts.len();

        self.bkt_states.insert(node_id.to_owned(), rebuilt.clone());
        progress.attempts = rebuilt.attempts;
        progress.correct = rebuilt.correct;
        progress.mastery_permille = new_mastery;
        progress.status = new_status;
        progress.self_assessment = None;
        progress.scored_item_ids = receipt_ids.into_iter().collect();
        progress.scored_practice_receipts = receipts;

        bkt_changed || progress_changed
    }

    /// Merge answer-first practice outcomes by opaque item ID, then rebuild
    /// the BKT state from the deduplicated evidence. Returns the number of new
    /// receipt records added. Safe to call repeatedly with the same profile.
    pub fn merge_scored_practice_evidence(&mut self, other: &Self) -> usize {
        let mut touched = BTreeSet::new();
        let mut added = 0usize;
        for (node_id, imported_node) in &other.nodes {
            if imported_node.scored_practice_receipts.is_empty() {
                continue;
            }
            let target = self.nodes.entry(node_id.clone()).or_default();
            for receipt in &imported_node.scored_practice_receipts {
                if target
                    .scored_practice_receipts
                    .iter()
                    .any(|existing| existing.item_id == receipt.item_id)
                {
                    continue;
                }
                target.scored_item_ids.push(receipt.item_id.clone());
                target.scored_practice_receipts.push(receipt.clone());
                added += 1;
            }
            touched.insert(node_id.clone());
        }

        for node_id in touched {
            self.rebuild_bkt_from_practice_receipts(&node_id);
        }
        added
    }

    /// Record completion of an interactive activity. Activity completion is a
    /// separate participation signal: it does not increase mastery, attempts,
    /// correctness, or the BKT posterior.
    pub fn record_activity_completion(&mut self, node_id: &str) {
        let entry = self.nodes.entry(node_id.to_string()).or_default();
        entry.activity_completions = entry.activity_completions.saturating_add(1);
        if entry.status == ProgressStatus::NotStarted {
            entry.status = ProgressStatus::Studying;
        }
    }

    /// Topics sorted by weakness (lowest P(mastery) first), filtered to those with attempts.
    pub fn weakest_topics(&self, limit: usize) -> Vec<(String, f32)> {
        let mut topics: Vec<(String, f32)> = self
            .bkt_states
            .iter()
            .filter(|(_, bkt)| bkt.attempts > 0)
            .map(|(id, bkt)| (id.clone(), bkt.p_mastery))
            .collect();
        topics.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        topics.truncate(limit);
        topics
    }
}

const PROGRESS_KEY: &str = "praxis_progress";

// ============================================================
// Leptos context provider
// ============================================================

/// Insert spaces before uppercase letters in camelCase subject names.
/// e.g. "EnglishLanguageArts" -> "English Language Arts"
pub fn display_subject(s: &str) -> String {
    let mut result = String::with_capacity(s.len() + 4);
    for (i, c) in s.chars().enumerate() {
        if i > 0 && c.is_uppercase() {
            result.push(' ');
        }
        result.push(c);
    }
    result
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Subject(pub String);

impl Subject {
    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// Get all unique subjects from the graph.
    pub fn all_from_graph(graph: &CurriculumGraph) -> Vec<Subject> {
        let mut subjects: Vec<String> = graph.subjects().iter().map(|s| s.to_string()).collect();
        subjects.sort();
        subjects.into_iter().map(Subject).collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Grade {
    Kindergarten,
    Gr1,
    Gr2,
    Gr3,
    Gr4,
    Gr5,
    Gr6,
    Gr7,
    Gr8,
    Gr9,
    Gr10,
    Gr11,
    Gr12,
    Undergraduate,
    Graduate,
    Doctoral,
    Adult,
}

impl Grade {
    pub fn as_str(&self) -> &'static str {
        match self {
            Grade::Kindergarten => "Kindergarten",
            Grade::Gr1 => "Grade1",
            Grade::Gr2 => "Grade2",
            Grade::Gr3 => "Grade3",
            Grade::Gr4 => "Grade4",
            Grade::Gr5 => "Grade5",
            Grade::Gr6 => "Grade6",
            Grade::Gr7 => "Grade7",
            Grade::Gr8 => "Grade8",
            Grade::Gr9 => "Grade9",
            Grade::Gr10 => "Grade10",
            Grade::Gr11 => "Grade11",
            Grade::Gr12 => "Grade12",
            Grade::Undergraduate => "Undergraduate",
            Grade::Graduate => "Graduate",
            Grade::Doctoral => "Doctoral",
            Grade::Adult => "Adult",
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Grade::Kindergarten => "K",
            Grade::Gr1 => "1",
            Grade::Gr2 => "2",
            Grade::Gr3 => "3",
            Grade::Gr4 => "4",
            Grade::Gr5 => "5",
            Grade::Gr6 => "6",
            Grade::Gr7 => "7",
            Grade::Gr8 => "8",
            Grade::Gr9 => "9",
            Grade::Gr10 => "10",
            Grade::Gr11 => "11",
            Grade::Gr12 => "12",
            Grade::Undergraduate => "Uni",
            Grade::Graduate => "Postgrad",
            Grade::Doctoral => "PhD",
            Grade::Adult => "Life",
        }
    }

    /// All grades including post-secondary.
    pub fn all() -> &'static [Grade] {
        &[
            Grade::Kindergarten,
            Grade::Gr1,
            Grade::Gr2,
            Grade::Gr3,
            Grade::Gr4,
            Grade::Gr5,
            Grade::Gr6,
            Grade::Gr7,
            Grade::Gr8,
            Grade::Gr9,
            Grade::Gr10,
            Grade::Gr11,
            Grade::Gr12,
            Grade::Undergraduate,
            Grade::Graduate,
            Grade::Doctoral,
            Grade::Adult,
        ]
    }

    /// K-12 grades only (including Kindergarten).
    pub fn k12() -> &'static [Grade] {
        &[
            Grade::Kindergarten,
            Grade::Gr1,
            Grade::Gr2,
            Grade::Gr3,
            Grade::Gr4,
            Grade::Gr5,
            Grade::Gr6,
            Grade::Gr7,
            Grade::Gr8,
            Grade::Gr9,
            Grade::Gr10,
            Grade::Gr11,
            Grade::Gr12,
        ]
    }

    /// Post-secondary grades.
    pub fn post_secondary() -> &'static [Grade] {
        &[
            Grade::Undergraduate,
            Grade::Graduate,
            Grade::Doctoral,
            Grade::Adult,
        ]
    }

    /// Convert from profile grade number to Grade enum.
    pub fn from_profile_grade(g: u8) -> Grade {
        match g {
            0 => Grade::Kindergarten,
            1 => Grade::Gr1,
            2 => Grade::Gr2,
            3 => Grade::Gr3,
            4 => Grade::Gr4,
            5 => Grade::Gr5,
            6 => Grade::Gr6,
            7 => Grade::Gr7,
            8 => Grade::Gr8,
            9 => Grade::Gr9,
            10 => Grade::Gr10,
            11 => Grade::Gr11,
            12 => Grade::Gr12,
            13 => Grade::Undergraduate,
            14 => Grade::Graduate,
            15 => Grade::Doctoral,
            16 => Grade::Adult,
            _ => Grade::Gr12,
        }
    }

    pub fn is_post_secondary(&self) -> bool {
        matches!(
            self,
            Grade::Undergraduate | Grade::Graduate | Grade::Doctoral | Grade::Adult
        )
    }
}

/// Load local progress and normalize records created by prior semantics.
pub fn load_progress_store() -> ProgressStore {
    let mut progress = persistence::load::<ProgressStore>(PROGRESS_KEY).unwrap_or_default();
    if progress.migrate_legacy_progress() {
        persistence::save(PROGRESS_KEY, &progress);
    }
    progress
}

/// Provide curriculum context. Call once at app root.
pub fn provide_curriculum_context() -> Result<(), &'static CurriculumLoadError> {
    // Validate before routes or graph consumers are rendered.
    let _ = try_curriculum_graph()?;

    let initial_progress = load_progress_store();

    // Default grade from student profile (so constellation shows relevant content)
    let profile_grade =
        persistence::load::<crate::student_profile::StudentProfile>("praxis_profile")
            .map(|p| Grade::from_profile_grade(p.grade))
            .unwrap_or(Grade::Gr12);
    let default_subject = match profile_grade {
        Grade::Adult => "Financial Literacy",
        Grade::Undergraduate => "Computer Science",
        _ => "Mathematics",
    };
    let (subject, set_subject) = signal(Subject(default_subject.to_string()));
    let (grade, set_grade) = signal(profile_grade);
    let (progress, set_progress) = signal(initial_progress);

    // Persist progress on change
    Effect::new(move |_| {
        let p = progress.get();
        persistence::save(PROGRESS_KEY, &p);
    });

    // Celebration signal — briefly true when a topic is mastered
    let (celebrating, set_celebrating) = signal(false);
    let (celebration_topic, set_celebration_topic) = signal(String::new());

    provide_context(subject);
    provide_context(set_subject);
    provide_context(grade);
    provide_context(set_grade);
    provide_context(progress);
    provide_context(set_progress);
    provide_context(celebrating);
    provide_context(set_celebrating);
    provide_context(celebration_topic);
    provide_context(set_celebration_topic);

    Ok(())
}

pub fn use_subject() -> ReadSignal<Subject> {
    expect_context::<ReadSignal<Subject>>()
}

pub fn use_set_subject() -> WriteSignal<Subject> {
    expect_context::<WriteSignal<Subject>>()
}

pub fn use_grade() -> ReadSignal<Grade> {
    expect_context::<ReadSignal<Grade>>()
}

pub fn use_set_grade() -> WriteSignal<Grade> {
    expect_context::<WriteSignal<Grade>>()
}

pub fn use_progress() -> ReadSignal<ProgressStore> {
    expect_context::<ReadSignal<ProgressStore>>()
}

pub fn use_set_progress() -> WriteSignal<ProgressStore> {
    expect_context::<WriteSignal<ProgressStore>>()
}

/// Update learner-facing progress. Self-assessment and BKT estimates do not
/// trigger a verified-mastery celebration; that needs a future trusted evidence path.
pub fn set_node_status(node_id: &str, status: ProgressStatus, _topic_title: &str) {
    let set_progress = use_set_progress();
    set_progress.update(|p| p.set_status(node_id, status));
}


#[cfg(test)]
mod evidence_first_progress_tests {
    use super::*;

    #[test]
    fn provisional_mastery_unlocks_learning_navigation_but_not_verified_mastery() {
        let mut progress = ProgressStore::default();
        progress.nodes.insert(
            "foundation".to_owned(),
            NodeProgress {
                status: ProgressStatus::ProvisionalMastery,
                mastery_permille: 960,
                ..NodeProgress::default()
            },
        );

        assert!(progress.progression_ready_ids().contains("foundation"));
        assert!(!progress.mastered_ids().contains("foundation"));
        assert_eq!(progress.provisional_mastery_count(), 1);
        assert_eq!(progress.progression_ready_count(), 1);
        assert_eq!(progress.mastered_count(), 0);

        let verified = NodeProgress {
            status: ProgressStatus::Mastered,
            ..NodeProgress::default()
        };
        progress.nodes.insert("verified".to_owned(), verified);
        assert!(progress.progression_ready_ids().contains("verified"));
        assert!(progress.mastered_ids().contains("verified"));
    }

    #[test]
    fn a_manual_mastered_toggle_becomes_self_assessment_not_verified_mastery() {
        let mut progress = ProgressStore::default();
        progress.set_status("math.addition", ProgressStatus::Mastered);

        let node = progress.get("math.addition");
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.self_assessment, Some(SelfAssessment::FeelsUnderstood));
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(progress.mastered_count(), 0);
        assert!(progress.mastered_ids().is_empty());
    }

    #[test]
    fn selecting_studying_clears_the_previous_confidence_reflection() {
        let mut progress = ProgressStore::default();
        progress.record_self_assessment("math.geometry", SelfAssessment::FeelsUnderstood);
        progress.set_status("math.geometry", ProgressStatus::Studying);
        assert_eq!(progress.get("math.geometry").self_assessment, None);
        assert_eq!(progress.get("math.geometry").status, ProgressStatus::Studying);
        assert_eq!(progress.get("math.geometry").self_reported_attempts, 1);
    }

    #[test]
    fn a_stale_mastered_value_cannot_keep_a_high_score_after_self_assessment() {
        let mut progress = ProgressStore::default();
        progress.nodes.insert(
            "math.stale".to_owned(),
            NodeProgress {
                mastery_permille: 1000,
                attempts: 8,
                correct: 8,
                status: ProgressStatus::Mastered,
                ..NodeProgress::default()
            },
        );

        progress.set_status("math.stale", ProgressStatus::Mastered);
        let node = progress.get("math.stale");
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(node.self_reported_attempts, 9);
        assert_eq!(node.self_reported_correct, 9);
        assert_eq!(node.self_assessment, Some(SelfAssessment::FeelsUnderstood));
        assert!(progress.mastered_ids().is_empty());

        progress.record_self_assessment("math.stale", SelfAssessment::NeedsMorePractice);
        let node = progress.get("math.stale");
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(node.self_assessment, Some(SelfAssessment::NeedsMorePractice));
    }

    #[test]
    fn self_assessment_does_not_update_scored_attempts_or_bkt() {
        let mut progress = ProgressStore::default();
        progress.record_self_assessment("math.fractions", SelfAssessment::FeelsUnderstood);

        let node = progress.get("math.fractions");
        assert_eq!(node.self_assessment, Some(SelfAssessment::FeelsUnderstood));
        assert_eq!(node.self_reported_attempts, 1);
        assert_eq!(node.self_reported_correct, 1);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(progress.bkt("math.fractions").attempts, 0);
        assert_eq!(node.status, ProgressStatus::Studying);
    }

    #[test]
    fn scored_practice_receipts_are_private_item_outcomes_and_rebuild_posterior() {
        let mut progress = ProgressStore::default();
        assert!(progress.record_response_for_item(
            "math.multiplication",
            "practice-item-a1",
            true
        ));
        assert!(progress.record_response_for_item(
            "math.multiplication",
            "practice-item-a2",
            false
        ));

        let receipts = &progress.get("math.multiplication").scored_practice_receipts;
        assert_eq!(receipts.len(), 2);
        assert!(receipts.iter().any(|receipt| receipt.correct));
        assert!(receipts.iter().any(|receipt| !receipt.correct));
        assert!(
            receipts.iter().all(|receipt| {
                receipt.item_id.starts_with("practice-item-")
                    && receipt.recorded_at_ms.is_finite()
            }),
            "the ledger stores opaque item IDs, outcomes and times only"
        );

        let mut rebuilt = ProgressStore::default();
        rebuilt.nodes.insert(
            "math.multiplication".to_owned(),
            progress.get("math.multiplication").clone()
        );
        rebuilt.bkt_states.insert(
            "math.multiplication".to_owned(),
            BktState {
                p_mastery: 0.99,
                attempts: 90,
                correct: 90,
            }
        );
        rebuilt.rebuild_bkt_from_practice_receipts("math.multiplication");
        assert_eq!(rebuilt.bkt("math.multiplication").attempts, 2);
        assert_eq!(rebuilt.bkt("math.multiplication").correct, 1);
        assert_eq!(rebuilt.get("math.multiplication").attempts, 2);
        assert_eq!(rebuilt.get("math.multiplication").correct, 1);
        assert_eq!(
            rebuilt.get("math.multiplication").mastery_permille,
            (rebuilt.bkt("math.multiplication").p_mastery * 1000.0) as u16
        );
    }

    #[test]
    fn repeating_the_same_scored_item_does_not_inflate_bkt() {
        let mut progress = ProgressStore::default();
        for _ in 0..4 {
            assert!(progress.record_response_for_item(
                "math.multiplication",
                "lesson-digest:practice-1",
                true
            ));
        }
        let attempts_before_repeat = progress.get("math.multiplication").attempts;
        let bkt_before_repeat = progress.bkt("math.multiplication").p_mastery;

        assert!(
            !progress.record_response_for_item(
                "math.multiplication",
                "lesson-digest:practice-1",
                true
            ),
            "a replayed item must not count as new evidence"
        );
        assert_eq!(
            progress.get("math.multiplication").attempts,
            attempts_before_repeat
        );
        assert_eq!(
            progress.bkt("math.multiplication").p_mastery,
            bkt_before_repeat
        );

        assert!(progress.record_response_for_item(
            "math.multiplication",
            "lesson-digest:practice-2",
            true
        ));
        assert_eq!(
            progress.get("math.multiplication").attempts,
            attempts_before_repeat + 1
        );
    }

    #[test]
    fn merging_the_same_scored_profile_twice_is_idempotent() {
        let mut imported = ProgressStore::default();
        assert!(imported.record_response_for_item("math.fractions", "item-a", true));
        assert!(imported.record_response_for_item("math.fractions", "item-b", false));

        let mut local = ProgressStore::default();
        assert_eq!(local.merge_scored_practice_evidence(&imported), 2);
        let first_attempts = local.bkt("math.fractions").attempts;
        let first_correct = local.bkt("math.fractions").correct;
        let first_posterior = local.bkt("math.fractions").p_mastery;
        assert_eq!(first_attempts, 2);
        assert_eq!(first_correct, 1);
        assert_eq!(local.get("math.fractions").scored_practice_receipts.len(), 2);

        assert_eq!(local.merge_scored_practice_evidence(&imported), 0);
        assert_eq!(local.bkt("math.fractions").attempts, first_attempts);
        assert_eq!(local.bkt("math.fractions").correct, first_correct);
        assert_eq!(local.bkt("math.fractions").p_mastery, first_posterior);
        assert_eq!(local.get("math.fractions").scored_practice_receipts.len(), 2);

        let mut other_device = ProgressStore::default();
        assert!(other_device.record_response_for_item("math.fractions", "item-c", true));
        assert_eq!(local.merge_scored_practice_evidence(&other_device), 1);
        assert_eq!(local.bkt("math.fractions").attempts, 3);
        assert_eq!(local.bkt("math.fractions").correct, 2);
        assert_eq!(local.get("math.fractions").scored_practice_receipts.len(), 3);
    }

    #[test]
    fn activity_completion_is_not_scored_as_mastery() {
        let mut progress = ProgressStore::default();
        progress.record_activity_completion("math.projectile");

        let node = progress.get("math.projectile");
        assert_eq!(node.activity_completions, 1);
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(progress.bkt("math.projectile").attempts, 0);
        assert_eq!(progress.mastered_count(), 0);

        progress.record_activity_completion("math.projectile");
        assert_eq!(progress.get("math.projectile").activity_completions, 2);
        assert_eq!(progress.get("math.projectile").mastery_permille, 0);
    }

    #[test]
    fn scored_responses_can_reach_provisional_but_not_verified_mastery() {
        let mut progress = ProgressStore::default();
        for _ in 0..5 {
            progress.record_response("math.multiplication", true);
        }

        assert_eq!(
            progress.get("math.multiplication").status,
            ProgressStatus::ProvisionalMastery
        );
        assert!(progress.get("math.multiplication").mastery_permille >= 950);
        assert_eq!(progress.mastered_count(), 0);
        assert!(progress.mastered_ids().is_empty());
    }

    #[test]
    fn legacy_post_reveal_feedback_is_migrated_as_unscored_self_report() {
        let mut progress = ProgressStore::default();
        progress.nodes.insert(
            "math.multiplication".to_owned(),
            NodeProgress {
                mastery_permille: 900,
                attempts: 4,
                correct: 4,
                status: ProgressStatus::Mastered,
                ..NodeProgress::default()
            },
        );
        progress.bkt_states.insert(
            "math.multiplication".to_owned(),
            BktState {
                p_mastery: 0.99,
                attempts: 4,
                correct: 4,
            },
        );

        assert!(progress.migrate_legacy_progress());
        let node = progress.get("math.multiplication");
        assert_eq!(progress.schema_version, 1);
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.self_assessment, Some(SelfAssessment::FeelsUnderstood));
        assert_eq!(node.self_reported_attempts, 4);
        assert_eq!(node.self_reported_correct, 4);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(progress.bkt("math.multiplication").attempts, 0);
        assert!(progress.mastered_ids().is_empty());

        assert!(
            !progress.migrate_legacy_progress(),
            "migration must not erase new scored responses on a later load"
        );
    }

    #[test]
    fn provisional_status_is_recomputed_from_bkt_on_load() {
        let mut unsupported = ProgressStore {
            schema_version: 1,
            ..ProgressStore::default()
        };
        unsupported.nodes.insert(
            "math.fractions".to_owned(),
            NodeProgress {
                status: ProgressStatus::ProvisionalMastery,
                mastery_permille: 990,
                ..NodeProgress::default()
            },
        );
        assert!(unsupported.migrate_legacy_progress());
        assert_eq!(
            unsupported.get("math.fractions").status,
            ProgressStatus::Studying
        );
        assert_eq!(unsupported.get("math.fractions").mastery_permille, 0);

        let mut supported = ProgressStore {
            schema_version: 1,
            ..ProgressStore::default()
        };
        supported.nodes.insert(
            "math.fractions".to_owned(),
            NodeProgress {
                status: ProgressStatus::ProvisionalMastery,
                mastery_permille: 800,
                ..NodeProgress::default()
            },
        );
        supported.bkt_states.insert(
            "math.fractions".to_owned(),
            BktState {
                p_mastery: 0.97,
                attempts: 8,
                correct: 8,
            },
        );
        assert!(supported.migrate_legacy_progress());
        assert_eq!(
            supported.get("math.fractions").status,
            ProgressStatus::ProvisionalMastery
        );
        assert_eq!(supported.get("math.fractions").mastery_permille, 970);
    }

    #[test]
    fn schema_v1_mastered_flags_are_still_scrubbed_without_a_trusted_receipt() {
        let mut progress = ProgressStore {
            schema_version: 1,
            ..ProgressStore::default()
        };
        progress.nodes.insert(
            "math.fractions".to_owned(),
            NodeProgress {
                mastery_permille: 1000,
                attempts: 5,
                correct: 5,
                status: ProgressStatus::Mastered,
                ..NodeProgress::default()
            },
        );
        progress.bkt_states.insert(
            "math.fractions".to_owned(),
            BktState {
                p_mastery: 0.99,
                attempts: 5,
                correct: 5,
            },
        );

        assert!(progress.migrate_legacy_progress());
        let node = progress.get("math.fractions");
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(node.self_reported_attempts, 5);
        assert_eq!(node.self_reported_correct, 5);
        assert_eq!(progress.bkt("math.fractions").attempts, 0);
        assert!(progress.mastered_ids().is_empty());
    }

    #[test]
    fn old_progress_json_remains_deserializable_and_is_migrated() {
        let old_json = serde_json::json!({
            "nodes": {
                "math.geometry": {
                    "mastery_permille": 990,
                    "attempts": 7,
                    "correct": 6,
                    "status": "Mastered",
                    "last_reviewed": null
                }
            },
            "exam_date": null,
            "bkt_states": {
                "math.geometry": {
                    "p_mastery": 0.98,
                    "attempts": 7,
                    "correct": 6
                }
            }
        });
        let mut progress: ProgressStore =
            serde_json::from_value(old_json).expect("old progress must remain readable");
        assert_eq!(progress.schema_version, 0);
        assert!(progress.migrate_legacy_progress());
        let node = progress.get("math.geometry");
        assert_eq!(progress.schema_version, 1);
        assert_eq!(node.status, ProgressStatus::Studying);
        assert_eq!(node.self_assessment, Some(SelfAssessment::FeelsUnderstood));
        assert_eq!(node.self_reported_attempts, 7);
        assert_eq!(node.self_reported_correct, 6);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.correct, 0);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(progress.bkt("math.geometry").attempts, 0);
    }

    #[test]
    fn legacy_bkt_entries_without_node_progress_are_not_trusted() {
        let mut progress = ProgressStore::default();
        progress.bkt_states.insert(
            "math.geometry".to_owned(),
            BktState {
                p_mastery: 0.98,
                attempts: 6,
                correct: 5,
            },
        );

        assert!(progress.migrate_legacy_progress());
        let node = progress.get("math.geometry");
        assert_eq!(node.self_reported_attempts, 6);
        assert_eq!(node.self_reported_correct, 5);
        assert_eq!(node.attempts, 0);
        assert_eq!(node.mastery_permille, 0);
        assert_eq!(progress.bkt("math.geometry").attempts, 0);
    }
}
