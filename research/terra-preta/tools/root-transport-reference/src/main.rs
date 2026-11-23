use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    fs::OpenOptions,
    io::{self, Write},
    path::Path,
};

const VERSION: &str = "root-transport-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const EPS: f64 = 1e-10;
const BOUNDARY: &str = "This deterministic network reference is a synthetic known-truth test operator. It is not a Richards, HYDRUS DualPerm, MACRO, Stokes, lattice-Boltzmann or pore-network solver; it does not infer real root origin, CT connectivity, hydraulic conductivity, field tracer transport or agronomic outcomes.";

#[derive(Debug, Deserialize)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    truth_support_dimension: String,
    projection_plane: String,
    head_drop_m: f64,
    nodes: Vec<Node>,
    pathways: Vec<Pathway>,
    snapshots: Vec<Snapshot>,
    tracer: TracerContract,
}

#[derive(Debug, Deserialize)]
struct Node {
    node_id: String,
    coordinates_m: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct Pathway {
    pathway_id: String,
    origin_class: String,
    expression: String,
    source_node_id: String,
    outlet_node_id: String,
    recovery_fraction: f64,
    segments: Vec<Segment>,
}

#[derive(Debug, Deserialize)]
struct Segment {
    edge_id: String,
    from_node_id: String,
    to_node_id: String,
    length_m: f64,
    baseline_conductance_m3_per_hour_per_m_head: f64,
    travel_time_hours: f64,
}

#[derive(Debug, Deserialize)]
struct Snapshot {
    snapshot_id: String,
    time_days: f64,
    edge_states: Vec<EdgeState>,
}

#[derive(Debug, Deserialize)]
struct EdgeState {
    edge_id: String,
    root_occupancy: String,
    hydraulic_state: String,
    conductance_scale: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct TracerContract {
    injected_mass_mg: f64,
    sample_interval_hours: f64,
    collection_duration_hours: f64,
    detection_limit_mg: f64,
    mass_balance_tolerance_mg: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    pathway_count: usize,
    snapshot_count: usize,
    projection_plane: String,
    projection_alias_group_count: usize,
    projection_alias_groups: Vec<ProjectionAliasGroup>,
    snapshot_results: Vec<SnapshotResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct ProjectionAliasGroup {
    projected_coordinate_m: [f64; 2],
    node_ids: Vec<String>,
}

#[derive(Debug, Serialize)]
struct SnapshotResult {
    snapshot_id: String,
    time_days: f64,
    flow_allocation_status: String,
    total_network_conductance_m3_per_hour_per_m_head: Option<f64>,
    total_flow_m3_per_hour: Option<f64>,
    injected_tracer_mass_mg: f64,
    potential_recovered_tracer_mass_mg: Option<f64>,
    unrecovered_or_retained_tracer_mass_mg: Option<f64>,
    mass_balance_residual_mg: Option<f64>,
    expected_recovered_mass_weighted_transit_time_hours: Option<f64>,
    pathways: Vec<PathwayResult>,
}

#[derive(Debug, Serialize)]
struct PathwayResult {
    pathway_id: String,
    origin_class: String,
    expression: String,
    geometric_path_continuity: bool,
    hydraulic_state: String,
    effective_conductance_m3_per_hour_per_m_head: Option<f64>,
    flow_fraction: Option<f64>,
    flow_m3_per_hour: Option<f64>,
    expected_transit_time_hours: Option<f64>,
    potential_recovered_tracer_mass_mg: Option<f64>,
    tracer_observation_status: String,
    observed_arrival_time_hours: Option<f64>,
    observed_tracer_mass_mg: Option<f64>,
}

#[derive(Debug, Serialize)]
struct Controls {
    three_d_truth_support_required: bool,
    projection_aliases_do_not_certify_3d_connectivity: bool,
    geometric_connectivity_separate_from_hydraulic_activation: bool,
    unassessed_activation_withholds_complete_flow_allocation: bool,
    root_history_separate_from_static_origin: bool,
    tracer_mass_balance_explicit: bool,
    no_empirical_solver_or_field_claim: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn projection_key(plane: &str, xyz: [f64; 3]) -> (u64, u64) {
    let (a, b) = match plane {
        "xy" => (xyz[0], xyz[1]),
        "xz" => (xyz[0], xyz[2]),
        "yz" => (xyz[1], xyz[2]),
        _ => unreachable!("projection plane validated"),
    };
    let a = if a == 0.0 { 0.0 } else { a };
    let b = if b == 0.0 { 0.0 } else { b };
    (a.to_bits(), b.to_bits())
}

fn projected_coordinate(plane: &str, xyz: [f64; 3]) -> [f64; 2] {
    match plane {
        "xy" => [xyz[0], xyz[1]],
        "xz" => [xyz[0], xyz[2]],
        "yz" => [xyz[1], xyz[2]],
        _ => unreachable!("projection plane validated"),
    }
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "root-transport-v0.1" || s.fixture_status != FIXTURE_STATUS {
        return Err("unsupported schema version or non-synthetic fixture status".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.truth_support_dimension != "volumetric_3d"
        || !matches!(s.projection_plane.as_str(), "xy" | "xz" | "yz")
        || s.nodes.len() < 2
        || s.pathways.is_empty()
        || s.snapshots.is_empty()
    {
        return Err("scenario requires a 3D truth volume, valid projection, nodes, pathways and snapshots".into());
    }
    finite("head_drop_m", s.head_drop_m)?;
    if s.head_drop_m <= 0.0 {
        return Err("head_drop_m must be positive".into());
    }

    let mut nodes = BTreeMap::new();
    for node in &s.nodes {
        if node.node_id.trim().is_empty() || nodes.contains_key(&node.node_id) {
            return Err("node IDs must be non-empty and unique".into());
        }
        for coordinate in node.coordinates_m {
            finite("node coordinate", coordinate)?;
        }
        nodes.insert(node.node_id.clone(), node.coordinates_m);
    }

    let mut edge_ids = BTreeSet::new();
    let mut pathway_ids = BTreeSet::new();
    for path in &s.pathways {
        if path.pathway_id.trim().is_empty()
            || !pathway_ids.insert(path.pathway_id.clone())
            || path.segments.is_empty()
            || !nodes.contains_key(&path.source_node_id)
            || !nodes.contains_key(&path.outlet_node_id)
            || !(0.0..=1.0).contains(&path.recovery_fraction)
        {
            return Err("pathway identity, endpoints, segment set or recovery fraction invalid".into());
        }
        if !matches!(
            path.origin_class.as_str(),
            "live_root_channel" | "legacy_root_channel" | "abiotic_crack" | "textural_finger" | "faunal_burrow"
        ) {
            return Err(format!("pathway {} has unsupported origin class", path.pathway_id));
        }
        let root_origin = matches!(path.origin_class.as_str(), "live_root_channel" | "legacy_root_channel");
        let valid_expression = if root_origin {
            matches!(
                path.expression.as_str(),
                "shallow_root_network_dominated" | "vertical_root_controlled" | "interface_guided" | "legacy_root_biopore_dominated"
            )
        } else {
            path.expression == "not_applicable"
        };
        if !valid_expression {
            return Err(format!("pathway {} has incompatible origin/expression", path.pathway_id));
        }

        let mut previous_to: Option<&str> = None;
        for segment in &path.segments {
            if segment.edge_id.trim().is_empty()
                || !edge_ids.insert(segment.edge_id.clone())
                || !nodes.contains_key(&segment.from_node_id)
                || !nodes.contains_key(&segment.to_node_id)
            {
                return Err(format!("pathway {} has empty/duplicate edge or unknown node", path.pathway_id));
            }
            if let Some(previous) = previous_to {
                if previous != segment.from_node_id {
                    return Err(format!("pathway {} has a discontinuous ordered edge chain", path.pathway_id));
                }
            } else if segment.from_node_id != path.source_node_id {
                return Err(format!("pathway {} does not start at its declared source node", path.pathway_id));
            }
            finite("segment length", segment.length_m)?;
            finite("baseline conductance", segment.baseline_conductance_m3_per_hour_per_m_head)?;
            finite("segment travel time", segment.travel_time_hours)?;
            if segment.length_m <= 0.0
                || segment.baseline_conductance_m3_per_hour_per_m_head <= 0.0
                || segment.travel_time_hours <= 0.0
            {
                return Err("segment length, conductance and travel time must be positive".into());
            }
            let chord = distance(
                *nodes.get(&segment.from_node_id).expect("node validated"),
                *nodes.get(&segment.to_node_id).expect("node validated"),
            );
            if segment.length_m + EPS < chord {
                return Err(format!("segment {} length is shorter than its 3D endpoint chord", segment.edge_id));
            }
            previous_to = Some(&segment.to_node_id);
        }
        if previous_to != Some(path.outlet_node_id.as_str()) {
            return Err(format!("pathway {} does not end at its declared outlet node", path.pathway_id));
        }
    }

    if s.tracer.injected_mass_mg <= 0.0
        || s.tracer.sample_interval_hours <= 0.0
        || s.tracer.collection_duration_hours <= 0.0
        || s.tracer.detection_limit_mg < 0.0
        || s.tracer.mass_balance_tolerance_mg <= 0.0
    {
        return Err("tracer mass, sampling interval, collection window and tolerance must be valid".into());
    }
    for (name, value) in [
        ("injected_mass_mg", s.tracer.injected_mass_mg),
        ("sample_interval_hours", s.tracer.sample_interval_hours),
        ("collection_duration_hours", s.tracer.collection_duration_hours),
        ("detection_limit_mg", s.tracer.detection_limit_mg),
        ("mass_balance_tolerance_mg", s.tracer.mass_balance_tolerance_mg),
    ] {
        finite(name, value)?;
    }

    let mut snapshot_ids = BTreeSet::new();
    let mut previous_time = None;
    for snapshot in &s.snapshots {
        if snapshot.snapshot_id.trim().is_empty() || !snapshot_ids.insert(snapshot.snapshot_id.clone()) {
            return Err("snapshot IDs must be non-empty and unique".into());
        }
        finite("snapshot time", snapshot.time_days)?;
        if snapshot.time_days < 0.0 || previous_time.is_some_and(|time| snapshot.time_days <= time) {
            return Err("snapshot times must be non-negative and strictly increasing".into());
        }
        previous_time = Some(snapshot.time_days);

        let mut states = BTreeSet::new();
        for state in &snapshot.edge_states {
            if !edge_ids.contains(&state.edge_id) || !states.insert(state.edge_id.clone()) {
                return Err("snapshot has unknown or duplicate edge state".into());
            }
            let is_root_edge = s.pathways.iter().any(|path| {
                matches!(path.origin_class.as_str(), "live_root_channel" | "legacy_root_channel")
                    && path.segments.iter().any(|segment| segment.edge_id == state.edge_id)
            });
            if is_root_edge {
                if !matches!(state.root_occupancy.as_str(), "occupied" | "partially_occupied" | "unoccupied_legacy" | "not_observed") {
                    return Err(format!("root edge {} requires an explicit root occupancy state", state.edge_id));
                }
            } else if state.root_occupancy != "not_applicable" {
                return Err(format!("non-root edge {} must use not_applicable root occupancy", state.edge_id));
            }
            match state.hydraulic_state.as_str() {
                "active" => {
                    let scale = state.conductance_scale.ok_or("active edge requires conductance_scale")?;
                    finite("conductance_scale", scale)?;
                    if scale <= 0.0 || scale > 1.0 {
                        return Err("active conductance_scale must be in (0,1]".into());
                    }
                }
                "inactive" => {
                    let scale = state.conductance_scale.ok_or("inactive edge requires conductance_scale")?;
                    finite("conductance_scale", scale)?;
                    if scale.abs() > EPS {
                        return Err("inactive edge conductance_scale must be zero".into());
                    }
                }
                "unassessed" => {
                    if state.conductance_scale.is_some() {
                        return Err("unassessed edge must not supply conductance_scale".into());
                    }
                }
                _ => return Err("hydraulic_state must be active, inactive or unassessed".into()),
            }
        }
        if states != edge_ids {
            return Err("every snapshot must provide exactly one state for every declared edge".into());
        }
    }
    Ok(())
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PathState {
    Active,
    Inactive,
    Unassessed,
}

fn path_state(path: &Pathway, states: &BTreeMap<&str, &EdgeState>) -> PathState {
    let mut saw_unassessed = false;
    for segment in &path.segments {
        match states.get(segment.edge_id.as_str()).expect("validated edge state").hydraulic_state.as_str() {
            "inactive" => return PathState::Inactive,
            "unassessed" => saw_unassessed = true,
            "active" => {}
            _ => unreachable!("hydraulic state validated"),
        }
    }
    if saw_unassessed { PathState::Unassessed } else { PathState::Active }
}

fn projection_aliases(s: &Scenario) -> Vec<ProjectionAliasGroup> {
    let mut grouped: BTreeMap<(u64, u64), Vec<&Node>> = BTreeMap::new();
    for node in &s.nodes {
        grouped.entry(projection_key(&s.projection_plane, node.coordinates_m)).or_default().push(node);
    }
    grouped.into_iter().filter_map(|(_, nodes)| {
        let first = nodes.first()?;
        let has_distinct_3d = nodes.iter().any(|node| {
            (0..3).any(|i| (node.coordinates_m[i] - first.coordinates_m[i]).abs() > EPS)
        });
        if nodes.len() > 1 && has_distinct_3d {
            Some(ProjectionAliasGroup {
                projected_coordinate_m: projected_coordinate(&s.projection_plane, first.coordinates_m),
                node_ids: nodes.iter().map(|node| node.node_id.clone()).collect(),
            })
        } else {
            None
        }
    }).collect()
}

fn build_report(s: &Scenario, input_sha256: String) -> Result<Report, String> {
    let aliases = projection_aliases(s);
    let mut snapshot_results = Vec::with_capacity(s.snapshots.len());

    for snapshot in &s.snapshots {
        let states: BTreeMap<&str, &EdgeState> = snapshot.edge_states.iter()
            .map(|state| (state.edge_id.as_str(), state)).collect();
        let mut path_states = Vec::with_capacity(s.pathways.len());
        let mut conductances = Vec::with_capacity(s.pathways.len());
        let mut travel_times = Vec::with_capacity(s.pathways.len());

        for path in &s.pathways {
            let state = path_state(path, &states);
            let conductance = match state {
                PathState::Inactive => Some(0.0),
                PathState::Unassessed => None,
                PathState::Active => {
                    let mut total_resistance = 0.0;
                    for segment in &path.segments {
                        let scale = states.get(segment.edge_id.as_str())
                            .expect("validated state").conductance_scale.expect("active state scale");
                        let segment_conductance = segment.baseline_conductance_m3_per_hour_per_m_head * scale;
                        total_resistance += 1.0 / segment_conductance;
                    }
                    Some(1.0 / total_resistance)
                }
            };
            let travel_time = path.segments.iter().map(|segment| segment.travel_time_hours).sum::<f64>();
            path_states.push(state);
            conductances.push(conductance);
            travel_times.push(travel_time);
        }

        let allocation_complete = !path_states.iter().any(|state| *state == PathState::Unassessed);
        let total_conductance = if allocation_complete {
            Some(conductances.iter().map(|value| value.unwrap_or(0.0)).sum::<f64>())
        } else {
            None
        };
        let total_flow = total_conductance.map(|conductance| conductance * s.head_drop_m);
        let mut potential_recovered_total = 0.0;
        let mut weighted_transit_numerator = 0.0;
        let mut path_results = Vec::with_capacity(s.pathways.len());

        for (index, path) in s.pathways.iter().enumerate() {
            let state = path_states[index];
            let effective = conductances[index];
            let conductance_total = total_conductance.unwrap_or(0.0);
            let flow_fraction = if allocation_complete && conductance_total > EPS {
                Some(effective.unwrap_or(0.0) / conductance_total)
            } else if allocation_complete {
                Some(0.0)
            } else {
                None
            };
            let flow = match (allocation_complete, flow_fraction) {
                (true, Some(fraction)) => Some(total_flow.unwrap_or(0.0) * fraction),
                _ => None,
            };
            let expected_transit = match state {
                PathState::Active if allocation_complete && flow.unwrap_or(0.0) > EPS => Some(travel_times[index]),
                _ => None,
            };
            let potential_recovered = match (allocation_complete, flow_fraction) {
                (true, Some(fraction)) => {
                    let recovered = s.tracer.injected_mass_mg * fraction * path.recovery_fraction;
                    potential_recovered_total += recovered;
                    if let Some(time) = expected_transit {
                        weighted_transit_numerator += recovered * time;
                    }
                    Some(recovered)
                }
                _ => None,
            };
            let (observation_status, observed_arrival, observed_mass) =
                match (allocation_complete, state, expected_transit, potential_recovered) {
                    (false, _, _, _) => ("abstained_unassessed_hydraulic_state", None, None),
                    (true, PathState::Inactive, _, _) => ("no_hydraulic_flow", None, Some(0.0)),
                    (true, PathState::Unassessed, _, _) => ("abstained_unassessed_hydraulic_state", None, None),
                    (true, PathState::Active, Some(time), Some(_mass)) if time > s.tracer.collection_duration_hours => {
                        ("outside_collection_window", None, Some(0.0))
                    }
                    (true, PathState::Active, Some(time), Some(mass)) if mass < s.tracer.detection_limit_mg => {
                        ("below_detection_limit", None, Some(0.0))
                    }
                    (true, PathState::Active, Some(time), Some(mass)) => {
                        let measured_time = (time / s.tracer.sample_interval_hours).round() * s.tracer.sample_interval_hours;
                        ("observed", Some(measured_time), Some(mass))
                    }
                    _ => ("no_hydraulic_flow", None, Some(0.0)),
                };

            path_results.push(PathwayResult {
                pathway_id: path.pathway_id.clone(),
                origin_class: path.origin_class.clone(),
                expression: path.expression.clone(),
                geometric_path_continuity: true,
                hydraulic_state: match state {
                    PathState::Active => "active".into(),
                    PathState::Inactive => "inactive".into(),
                    PathState::Unassessed => "unassessed".into(),
                },
                effective_conductance_m3_per_hour_per_m_head: effective,
                flow_fraction,
                flow_m3_per_hour: flow,
                expected_transit_time_hours: expected_transit,
                potential_recovered_tracer_mass_mg: potential_recovered,
                tracer_observation_status: observation_status.into(),
                observed_arrival_time_hours: observed_arrival,
                observed_tracer_mass_mg: observed_mass,
            });
        }

        let (potential_recovered, unrecovered, residual, weighted_time) = if allocation_complete {
            let unrecovered_mass = (s.tracer.injected_mass_mg - potential_recovered_total).max(0.0);
            let residual = s.tracer.injected_mass_mg - potential_recovered_total - unrecovered_mass;
            let weighted = (potential_recovered_total > EPS)
                .then(|| weighted_transit_numerator / potential_recovered_total);
            (Some(potential_recovered_total), Some(unrecovered_mass), Some(residual), weighted)
        } else {
            (None, None, None, None)
        };

        if let Some(residual) = residual {
            if residual.abs() > s.tracer.mass_balance_tolerance_mg {
                return Err("tracer mass-balance residual exceeds declared tolerance".into());
            }
        }

        snapshot_results.push(SnapshotResult {
            snapshot_id: snapshot.snapshot_id.clone(),
            time_days: snapshot.time_days,
            flow_allocation_status: if allocation_complete { "complete".into() } else { "withheld_unassessed_state".into() },
            total_network_conductance_m3_per_hour_per_m_head: total_conductance,
            total_flow_m3_per_hour: total_flow,
            injected_tracer_mass_mg: s.tracer.injected_mass_mg,
            potential_recovered_tracer_mass_mg: potential_recovered,
            unrecovered_or_retained_tracer_mass_mg: unrecovered,
            mass_balance_residual_mg: residual,
            expected_recovered_mass_weighted_transit_time_hours: weighted_time,
            pathways: path_results,
        });
    }

    Ok(Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        pathway_count: s.pathways.len(),
        snapshot_count: s.snapshots.len(),
        projection_plane: s.projection_plane.clone(),
        projection_alias_group_count: aliases.len(),
        projection_alias_groups: aliases,
        snapshot_results,
        controls: Controls {
            three_d_truth_support_required: true,
            projection_aliases_do_not_certify_3d_connectivity: true,
            geometric_connectivity_separate_from_hydraulic_activation: true,
            unassessed_activation_withholds_complete_flow_allocation: true,
            root_history_separate_from_static_origin: true,
            tracer_mass_balance_explicit: true,
            no_empirical_solver_or_field_claim: true,
        },
        interpretation_boundary: BOUNDARY,
    })
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest).map_err(io::Error::other)?;
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: root-transport-reference <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("root transport reference failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-root-transport.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn root_and_crack_paths_share_projection_but_not_3d_geometry() {
        let s = scenario();
        validate(&s).expect("fixture valid");
        let groups = projection_aliases(&s);
        assert!(!groups.is_empty());
        assert!(groups.iter().any(|group| group.node_ids.len() >= 2));
        assert_eq!(s.truth_support_dimension, "volumetric_3d");
    }

    #[test]
    fn geometrically_connected_inactive_path_carries_no_flow_or_tracer() {
        let report = build_report(&scenario(), "0".repeat(64)).expect("report builds");
        let inactive = report.snapshot_results[0].pathways.iter()
            .find(|path| path.pathway_id == "inactive-finger").unwrap();
        assert!(inactive.geometric_path_continuity);
        assert_eq!(inactive.hydraulic_state, "inactive");
        assert_eq!(inactive.flow_m3_per_hour, Some(0.0));
        assert_eq!(inactive.potential_recovered_tracer_mass_mg, Some(0.0));
    }

    #[test]
    fn root_turnover_changes_flow_allocation_without_changing_origin_label() {
        let report = build_report(&scenario(), "0".repeat(64)).expect("report builds");
        let first = report.snapshot_results[0].pathways.iter()
            .find(|path| path.pathway_id == "root-path").unwrap();
        let last = report.snapshot_results[2].pathways.iter()
            .find(|path| path.pathway_id == "root-path").unwrap();
        let first_fraction = first.flow_fraction.unwrap();
        let last_fraction = last.flow_fraction.unwrap();
        assert!(last_fraction < first_fraction);
        assert_eq!(last.origin_class, "live_root_channel");
        assert_eq!(last.hydraulic_state, "active");
    }

    #[test]
    fn tracer_mass_closes_and_path_transit_is_reported_separately() {
        let report = build_report(&scenario(), "0".repeat(64)).expect("report builds");
        for snapshot in &report.snapshot_results {
            assert!(snapshot.mass_balance_residual_mg.unwrap().abs() <= 1e-9);
            assert!(snapshot.potential_recovered_tracer_mass_mg.unwrap()
                + snapshot.unrecovered_or_retained_tracer_mass_mg.unwrap()
                <= snapshot.injected_tracer_mass_mg + 1e-9);
        }
    }

    #[test]
    fn unassessed_edge_withholds_all_flow_allocation() {
        let mut s = scenario();
        let state = s.snapshots[0].edge_states.iter_mut()
            .find(|state| state.edge_id == "root-2").unwrap();
        state.hydraulic_state = "unassessed".into();
        state.conductance_scale = None;
        validate(&s).expect("unassessed is a valid unknown state");
        let report = build_report(&s, "0".repeat(64)).expect("report builds");
        assert_eq!(report.snapshot_results[0].flow_allocation_status, "withheld_unassessed_state");
        assert!(report.snapshot_results[0].total_flow_m3_per_hour.is_none());
        assert!(report.snapshot_results[0].potential_recovered_tracer_mass_mg.is_none());
    }

    #[test]
    fn edge_chain_discontinuity_is_rejected() {
        let mut s = scenario();
        s.pathways[0].segments[1].from_node_id = "crack-middle".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn root_occupancy_transition_must_be_explicit() {
        let mut s = scenario();
        s.snapshots[2].edge_states.iter_mut()
            .find(|state| state.edge_id == "root-1").unwrap()
            .root_occupancy = "not_applicable".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn invalid_tracer_recovery_contract_is_rejected() {
        let mut s = scenario();
        s.pathways[0].recovery_fraction = 1.2;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn two_dimensional_only_truth_support_is_rejected() {
        let mut s = scenario();
        s.truth_support_dimension = "planar_2d".into();
        assert!(validate(&s).is_err());
    }
}
