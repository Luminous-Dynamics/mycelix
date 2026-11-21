//! Synthetic-only structural and evidence auditor for root-channel inventories.
//!
//! This is not a flow solver, CT segmenter or root-origin classifier. It verifies
//! the typed inventory and refuses unsupported 2D-to-3D connectivity promotion.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic structural inventory audit only. No CT segmentation, root-origin inference, 3D reconstruction, hydraulic flow, solute transport, microbial function or field qualification is performed.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    observation_support: ObservationSupport,
    nodes: Vec<Node>,
    pathways: Vec<Pathway>,
    snapshots: Vec<Snapshot>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ObservationSupport {
    dimension: String,
    method: String,
    source_sha256: String,
    geometry_sha256: String,
    resolution_m: f64,
    crs: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Node {
    node_id: String,
    coordinates_m: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Pathway {
    pathway_id: String,
    origin_class: String,
    origin_evidence_status: String,
    connectivity_evidence_status: String,
    start_node_id: String,
    end_node_id: String,
    length_m: f64,
    mean_diameter_m: f64,
    tortuosity: f64,
    depth_min_m: f64,
    depth_max_m: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Snapshot {
    elapsed_days: f64,
    pathway_states: Vec<PathwayState>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PathwayState {
    pathway_id: String,
    root_occupancy_state: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    node_count: usize,
    pathway_count: usize,
    snapshot_count: usize,
    origin_class_counts: BTreeMap<String, usize>,
    root_origin_evidence_resolved_count: usize,
    root_origin_unresolved_count: usize,
    direct_3d_connectivity_count: usize,
    endpoint_graph_component_count: usize,
    warnings: Vec<String>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    id_integrity_pass: bool,
    endpoint_references_pass: bool,
    coordinate_support_pass: bool,
    temporal_state_coverage_pass: bool,
    root_origin_semantics_pass: bool,
    no_2d_to_3d_promotion_pass: bool,
}

fn is_root_origin(origin: &str) -> bool {
    matches!(origin, "live_root_channel" | "legacy_root_channel")
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "root-channel-inventory-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic schema/version".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.observation_support.method.trim().is_empty()
        || s.observation_support.crs.trim().is_empty()
    {
        return Err("scenario and observation-support metadata must be non-empty".into());
    }
    if !matches!(
        s.observation_support.dimension.as_str(),
        "planar_2d" | "volumetric_3d"
    ) {
        return Err("unsupported observation dimension".into());
    }
    if !valid_sha256(&s.observation_support.source_sha256)
        || !valid_sha256(&s.observation_support.geometry_sha256)
    {
        return Err(
            "source and geometry SHA-256 values must be lowercase hexadecimal digests".into(),
        );
    }
    if !s.observation_support.resolution_m.is_finite() || s.observation_support.resolution_m <= 0.0
    {
        return Err("observation resolution must be finite and positive".into());
    }
    if s.nodes.len() < 2 || s.pathways.is_empty() || s.snapshots.is_empty() {
        return Err("at least two nodes, one pathway and one snapshot are required".into());
    }

    let expected_dimensions = if s.observation_support.dimension == "planar_2d" {
        2
    } else {
        3
    };
    let mut node_ids = BTreeSet::new();
    for node in &s.nodes {
        if node.node_id.trim().is_empty() || !node_ids.insert(node.node_id.clone()) {
            return Err("node IDs must be non-empty and unique".into());
        }
        if node.coordinates_m.len() != expected_dimensions
            || node.coordinates_m.iter().any(|v| !v.is_finite())
        {
            return Err(format!(
                "node {} coordinates must have exactly {expected_dimensions} finite dimensions",
                node.node_id
            ));
        }
    }

    let mut pathway_ids = BTreeSet::new();
    for p in &s.pathways {
        if p.pathway_id.trim().is_empty() || !pathway_ids.insert(p.pathway_id.clone()) {
            return Err("pathway IDs must be non-empty and unique".into());
        }
        if !node_ids.contains(&p.start_node_id) || !node_ids.contains(&p.end_node_id) {
            return Err(format!(
                "pathway {} has an unresolved endpoint",
                p.pathway_id
            ));
        }
        if p.start_node_id == p.end_node_id {
            return Err(format!(
                "pathway {} has identical start and end nodes",
                p.pathway_id
            ));
        }
        if !matches!(
            p.origin_class.as_str(),
            "live_root_channel"
                | "legacy_root_channel"
                | "abiotic_crack"
                | "textural_finger"
                | "faunal_burrow"
                | "other"
                | "unknown"
        ) {
            return Err(format!(
                "pathway {} has an unsupported origin class",
                p.pathway_id
            ));
        }
        if !matches!(
            p.origin_evidence_status.as_str(),
            "directly_observed_root"
                | "historical_root_trace"
                | "morphology_only"
                | "unresolved"
                | "synthetic_truth"
                | "not_applicable"
        ) {
            return Err(format!(
                "pathway {} has an unsupported origin evidence status",
                p.pathway_id
            ));
        }
        if !matches!(
            p.connectivity_evidence_status.as_str(),
            "observed_3d"
                | "projected_2d_only"
                | "model_inferred"
                | "not_assessed"
                | "synthetic_truth_3d"
        ) {
            return Err(format!(
                "pathway {} has an unsupported connectivity evidence status",
                p.pathway_id
            ));
        }
        if s.observation_support.dimension == "planar_2d"
            && matches!(
                p.connectivity_evidence_status.as_str(),
                "observed_3d" | "synthetic_truth_3d"
            )
        {
            return Err(format!(
                "pathway {} promotes a planar 2D observation to 3D connectivity",
                p.pathway_id
            ));
        }
        if !is_root_origin(&p.origin_class) && p.origin_evidence_status != "not_applicable" {
            return Err(format!(
                "non-root pathway {} must use origin_evidence_status=not_applicable",
                p.pathway_id
            ));
        }
        if p.origin_class == "live_root_channel"
            && p.origin_evidence_status == "historical_root_trace"
        {
            return Err(format!(
                "live root pathway {} cannot use historical-only origin evidence",
                p.pathway_id
            ));
        }
        for (name, value) in [
            ("length_m", p.length_m),
            ("mean_diameter_m", p.mean_diameter_m),
            ("tortuosity", p.tortuosity),
            ("depth_min_m", p.depth_min_m),
            ("depth_max_m", p.depth_max_m),
        ] {
            if !value.is_finite() {
                return Err(format!("pathway {} has non-finite {name}", p.pathway_id));
            }
        }
        if p.length_m <= 0.0
            || p.mean_diameter_m <= 0.0
            || p.tortuosity < 1.0
            || p.depth_min_m < 0.0
            || p.depth_max_m < p.depth_min_m
        {
            return Err(format!(
                "pathway {} has physically inadmissible geometry",
                p.pathway_id
            ));
        }
    }

    let mut previous_time = None;
    for (index, snapshot) in s.snapshots.iter().enumerate() {
        if !snapshot.elapsed_days.is_finite() || snapshot.elapsed_days < 0.0 {
            return Err(format!("snapshot {index} has invalid elapsed_days"));
        }
        if previous_time.is_some_and(|t| snapshot.elapsed_days <= t) {
            return Err("snapshot times must be strictly increasing".into());
        }
        previous_time = Some(snapshot.elapsed_days);
        let mut state_ids = BTreeSet::new();
        for state in &snapshot.pathway_states {
            if !pathway_ids.contains(&state.pathway_id)
                || !state_ids.insert(state.pathway_id.clone())
            {
                return Err(format!(
                    "snapshot {index} has unknown or duplicate pathway state"
                ));
            }
            if !matches!(
                state.root_occupancy_state.as_str(),
                "occupied"
                    | "partially_occupied"
                    | "unoccupied_legacy"
                    | "not_applicable"
                    | "not_observed"
            ) {
                return Err(format!(
                    "snapshot {index} has unsupported root occupancy state"
                ));
            }
            let pathway = s
                .pathways
                .iter()
                .find(|p| p.pathway_id == state.pathway_id)
                .ok_or_else(|| "validated pathway disappeared".to_string())?;
            if !is_root_origin(&pathway.origin_class)
                && state.root_occupancy_state != "not_applicable"
            {
                return Err(format!(
                    "non-root pathway {} cannot have a root occupancy state",
                    state.pathway_id
                ));
            }
        }
        if state_ids != pathway_ids {
            return Err(format!(
                "snapshot {index} must provide exactly one state for every pathway"
            ));
        }
    }
    Ok(())
}

fn endpoint_component_count(s: &Scenario) -> usize {
    let mut parent: BTreeMap<String, String> = s
        .nodes
        .iter()
        .map(|n| (n.node_id.clone(), n.node_id.clone()))
        .collect();

    fn find(parent: &mut BTreeMap<String, String>, x: &str) -> String {
        let next = parent.get(x).expect("known node").clone();
        if next == x {
            next
        } else {
            let root = find(parent, &next);
            parent.insert(x.to_string(), root.clone());
            root
        }
    }

    for p in &s.pathways {
        let a = find(&mut parent, &p.start_node_id);
        let b = find(&mut parent, &p.end_node_id);
        if a != b {
            parent.insert(a, b);
        }
    }
    let roots: BTreeSet<String> = s
        .nodes
        .iter()
        .map(|n| find(&mut parent, &n.node_id))
        .collect();
    roots.len()
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let mut counts = BTreeMap::new();
    let mut root_resolved = 0usize;
    let mut root_unresolved = 0usize;
    let mut direct_3d = 0usize;
    let mut warnings = Vec::new();

    for p in &s.pathways {
        *counts.entry(p.origin_class.clone()).or_insert(0usize) += 1;
        if is_root_origin(&p.origin_class) {
            if matches!(
                p.origin_evidence_status.as_str(),
                "directly_observed_root" | "historical_root_trace" | "synthetic_truth"
            ) {
                root_resolved += 1;
            } else {
                root_unresolved += 1;
            }
        }
        if s.observation_support.dimension == "volumetric_3d"
            && matches!(
                p.connectivity_evidence_status.as_str(),
                "observed_3d" | "synthetic_truth_3d"
            )
        {
            direct_3d += 1;
        }
    }

    if s.observation_support.dimension == "planar_2d" {
        warnings.push("2D projection cannot establish 3D pathway connectivity".into());
    }
    if root_unresolved > 0 {
        warnings.push("some root-origin pathways remain morphology-only or unresolved".into());
    }
    if s.pathways
        .iter()
        .any(|p| p.connectivity_evidence_status == "model_inferred")
    {
        warnings.push("model-inferred connectivity is not direct observation".into());
    }
    warnings.push("endpoint graph components are topology of supplied IDs, not proof of hydraulic connectivity or transport".into());

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        node_count: s.nodes.len(),
        pathway_count: s.pathways.len(),
        snapshot_count: s.snapshots.len(),
        origin_class_counts: counts,
        root_origin_evidence_resolved_count: root_resolved,
        root_origin_unresolved_count: root_unresolved,
        direct_3d_connectivity_count: direct_3d,
        endpoint_graph_component_count: endpoint_component_count(s),
        warnings,
        controls: Controls {
            id_integrity_pass: true,
            endpoint_references_pass: true,
            coordinate_support_pass: true,
            temporal_state_coverage_pass: true,
            root_origin_semantics_pass: true,
            no_2d_to_3d_promotion_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: root-channel-inventory <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("root-channel inventory audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-root-channel-inventory.json"
        ))
        .expect("synthetic fixture parses")
    }

    #[test]
    fn synthetic_inventory_validates_with_visible_support_warning() {
        let s = scenario();
        validate(&s).expect("valid synthetic inventory");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.pathway_count, 4);
        assert_eq!(report.snapshot_count, 3);
        assert_eq!(report.root_origin_evidence_resolved_count, 2);
        assert_eq!(report.direct_3d_connectivity_count, 0);
        assert!(report.warnings.iter().any(|w| w.contains("2D projection")));
        assert_eq!(report.endpoint_graph_component_count, 1);
    }

    #[test]
    fn duplicate_pathway_id_is_rejected() {
        let mut s = scenario();
        s.pathways.push(s.pathways[0].clone());
        assert!(validate(&s).is_err());
    }

    #[test]
    fn missing_endpoint_is_rejected() {
        let mut s = scenario();
        s.pathways[0].end_node_id = "missing-node".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn planar_observation_cannot_claim_3d_connectivity() {
        let mut s = scenario();
        s.pathways[0].connectivity_evidence_status = "observed_3d".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn coordinate_dimension_must_match_observation_support() {
        let mut s = scenario();
        s.nodes[0].coordinates_m.push(0.0);
        assert!(validate(&s).is_err());
    }

    #[test]
    fn snapshot_times_must_increase_strictly() {
        let mut s = scenario();
        s.snapshots[2].elapsed_days = s.snapshots[1].elapsed_days;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn non_root_pathway_cannot_claim_root_occupancy() {
        let mut s = scenario();
        s.snapshots[0].pathway_states[2].root_occupancy_state = "occupied".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn morphology_only_root_origin_remains_unresolved() {
        let mut s = scenario();
        s.pathways[0].origin_evidence_status = "morphology_only".into();
        validate(&s).expect("morphology-only is valid but unresolved");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.root_origin_evidence_resolved_count, 1);
        assert_eq!(report.root_origin_unresolved_count, 1);
    }

    #[test]
    fn planar_synthetic_fixture_does_not_claim_direct_connectivity() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert_eq!(report.direct_3d_connectivity_count, 0);
        assert!(report.controls.no_2d_to_3d_promotion_pass);
    }
}
