use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    env, fs,
    fs::OpenOptions,
    io::{self, Write},
    path::Path,
};

const VERSION: &str = "segmentation-ensemble-audit-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const CLAIM_CEILING: &str = "This deterministic sensitivity audit compares registered graph variants across declared segmentation percentiles. Inclusion/connectivity fractions describe only the supplied variant set; they are not calibrated probabilities, posterior confidence or hydraulic activation. The tool does not segment CT images, resolve ambiguous split/merge correspondence, solve flow or validate field behavior.";

#[derive(Debug, Deserialize)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    image: ImageDescriptor,
    node_catalog: Vec<NodeDefinition>,
    edge_catalog: Vec<EdgeDefinition>,
    source_node_ids: Vec<String>,
    sink_node_ids: Vec<String>,
    variants: Vec<Variant>,
}

#[derive(Debug, Deserialize)]
struct ImageDescriptor {
    artifact_uri: String,
    sha256: String,
    dimensions_voxels: [u32; 3],
    voxel_spacing_um: [f64; 3],
    phase_name: String,
    segmentation_method: String,
    segmentation_version: String,
    preprocessing_id: String,
    network_extractor: String,
    network_extractor_version: String,
}

#[derive(Debug, Deserialize)]
struct NodeDefinition {
    node_id: String,
    coordinates_um: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct EdgeDefinition {
    edge_id: String,
    from_node_id: String,
    to_node_id: String,
}

#[derive(Debug, Deserialize)]
struct Variant {
    variant_id: String,
    alpha_percent: u8,
    perturbation_label: String,
    segmentation_mask_sha256: String,
    network_artifact_sha256: String,
    extractor_settings_sha256: String,
    present_node_ids: Vec<String>,
    present_edge_ids: Vec<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    image_sha256: String,
    variant_count: usize,
    reference_variant_id: String,
    node_catalog_count: usize,
    edge_catalog_count: usize,
    source_count: usize,
    sink_count: usize,
    node_presence: Vec<PresenceSummary>,
    edge_presence: Vec<PresenceSummary>,
    variant_results: Vec<VariantResult>,
    terminal_pair_results: Vec<TerminalPairResult>,
    pairwise_edge_jaccard: JaccardSummary,
    interpretation_boundary: String,
}

#[derive(Debug, Serialize)]
struct PresenceSummary {
    feature_id: String,
    present_variant_count: usize,
    total_variant_count: usize,
    sensitivity_frequency: f64,
}

#[derive(Debug, Serialize)]
struct VariantResult {
    variant_id: String,
    alpha_percent: u8,
    perturbation_label: String,
    present_node_count: usize,
    present_edge_count: usize,
    connected_component_count: usize,
    cycle_rank: usize,
    connected_terminal_pair_count: usize,
    node_count_relative_change_from_reference: Option<f64>,
    edge_count_relative_change_from_reference: Option<f64>,
    component_count_relative_change_from_reference: Option<f64>,
    cycle_rank_relative_change_from_reference: Option<f64>,
}

#[derive(Debug, Serialize)]
struct TerminalPairResult {
    source_node_id: String,
    sink_node_id: String,
    connected_variant_count: usize,
    total_variant_count: usize,
    sensitivity_frequency: f64,
    stability_class: String,
}

#[derive(Debug, Serialize)]
struct JaccardSummary {
    pair_count: usize,
    minimum: f64,
    maximum: f64,
    mean: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PairConnectivity {
    Connected,
    Disconnected,
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn unique_nonempty<'a>(values: &'a [String], label: &str) -> Result<BTreeSet<&'a str>, String> {
    let mut result = BTreeSet::new();
    for value in values {
        if value.trim().is_empty() || !result.insert(value.as_str()) {
            return Err(format!("{label} values must be non-empty and unique"));
        }
    }
    Ok(result)
}

fn validate(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != "segmentation-ensemble-v0.1"
        || scenario.fixture_status != FIXTURE_STATUS
        || scenario.scenario_id.trim().is_empty()
    {
        return Err("unsupported schema/fixture status or empty scenario identity".into());
    }

    let image = &scenario.image;
    if image.artifact_uri.trim().is_empty()
        || !valid_sha256(&image.sha256)
        || image.phase_name.trim().is_empty()
        || image.segmentation_method.trim().is_empty()
        || image.segmentation_version.trim().is_empty()
        || image.preprocessing_id.trim().is_empty()
        || image.network_extractor.trim().is_empty()
        || image.network_extractor_version.trim().is_empty()
    {
        return Err(
            "image descriptor requires immutable image and method/extractor identity".into(),
        );
    }
    if image.dimensions_voxels.contains(&0) {
        return Err("image dimensions must be positive".into());
    }
    for (axis, spacing) in ["x", "y", "z"].into_iter().zip(image.voxel_spacing_um) {
        finite(&format!("voxel_spacing_{axis}_um"), spacing)?;
        if spacing <= 0.0 {
            return Err("voxel spacing must be positive".into());
        }
    }

    if scenario.node_catalog.len() < 2 || scenario.edge_catalog.is_empty() {
        return Err("canonical catalog requires at least two nodes and one edge".into());
    }
    let mut nodes = BTreeMap::new();
    for node in &scenario.node_catalog {
        if node.node_id.trim().is_empty() || nodes.contains_key(node.node_id.as_str()) {
            return Err("canonical node IDs must be non-empty and unique".into());
        }
        for coordinate in node.coordinates_um {
            finite("node coordinate", coordinate)?;
        }
        nodes.insert(node.node_id.as_str(), node);
    }

    let mut edges = BTreeMap::new();
    for edge in &scenario.edge_catalog {
        if edge.edge_id.trim().is_empty()
            || edges.contains_key(edge.edge_id.as_str())
            || edge.from_node_id == edge.to_node_id
            || !nodes.contains_key(edge.from_node_id.as_str())
            || !nodes.contains_key(edge.to_node_id.as_str())
        {
            return Err("canonical edges require unique IDs, distinct known endpoints".into());
        }
        edges.insert(edge.edge_id.as_str(), edge);
    }

    let sources = unique_nonempty(&scenario.source_node_ids, "source")?;
    let sinks = unique_nonempty(&scenario.sink_node_ids, "sink")?;
    if sources.is_empty()
        || sinks.is_empty()
        || sources.iter().any(|id| !nodes.contains_key(*id))
        || sinks.iter().any(|id| !nodes.contains_key(id))
        || sources.iter().any(|id| sinks.contains(*id))
    {
        return Err("source/sink sets must be non-empty, disjoint and use catalog nodes".into());
    }

    if !(2..=25).contains(&scenario.variants.len()) {
        return Err("variant count must be between 2 and 25".into());
    }
    let mut variant_ids = BTreeSet::new();
    let mut alpha_values = BTreeSet::new();
    let mut has_reference = false;
    for variant in &scenario.variants {
        if variant.variant_id.trim().is_empty()
            || !variant_ids.insert(variant.variant_id.as_str())
            || variant.perturbation_label.trim().is_empty()
            || !(1..=99).contains(&variant.alpha_percent)
            || !alpha_values.insert(variant.alpha_percent)
            || !valid_sha256(&variant.segmentation_mask_sha256)
            || !valid_sha256(&variant.network_artifact_sha256)
            || !valid_sha256(&variant.extractor_settings_sha256)
        {
            return Err(
                "variants require unique IDs/percentiles and valid artifact digests".into(),
            );
        }
        if variant.alpha_percent == 50 {
            has_reference = true;
        }

        let present_nodes = unique_nonempty(&variant.present_node_ids, "present node")?;
        let present_edges = unique_nonempty(&variant.present_edge_ids, "present edge")?;
        if !sources.iter().all(|id| present_nodes.contains(*id))
            || !sinks.iter().all(|id| present_nodes.contains(id))
            || present_nodes.iter().any(|id| !nodes.contains_key(id))
            || present_edges.iter().any(|id| !edges.contains_key(id))
        {
            return Err(format!(
                "variant {} has a missing terminal or unknown catalog feature",
                variant.variant_id
            ));
        }
        for edge_id in &present_edges {
            let edge = edges.get(edge_id).expect("edge catalog membership checked");
            if !present_nodes.contains(edge.from_node_id.as_str())
                || !present_nodes.contains(edge.to_node_id.as_str())
            {
                return Err(format!(
                    "variant {} includes edge {} without both endpoint nodes",
                    variant.variant_id, edge_id
                ));
            }
        }
    }
    if !has_reference {
        return Err("percentile ensemble requires exactly one 50% reference variant".into());
    }
    Ok(())
}

fn adjacency(
    present_nodes: &BTreeSet<String>,
    present_edges: &BTreeSet<String>,
    edge_catalog: &BTreeMap<&str, &EdgeDefinition>,
) -> BTreeMap<String, Vec<String>> {
    let mut graph: BTreeMap<String, Vec<String>> = present_nodes
        .iter()
        .map(|node| (node.clone(), Vec::new()))
        .collect();
    for edge_id in present_edges {
        let edge = edge_catalog
            .get(edge_id.as_str())
            .expect("validated edge exists");
        graph
            .get_mut(&edge.from_node_id)
            .expect("validated endpoint exists")
            .push(edge.to_node_id.clone());
        graph
            .get_mut(&edge.to_node_id)
            .expect("validated endpoint exists")
            .push(edge.from_node_id.clone());
    }
    for neighbours in graph.values_mut() {
        neighbours.sort();
    }
    graph
}

fn connected(graph: &BTreeMap<String, Vec<String>>, source: &str, sink: &str) -> bool {
    let mut seen = BTreeSet::new();
    let mut queue = VecDeque::from([source.to_string()]);
    seen.insert(source.to_string());
    while let Some(node) = queue.pop_front() {
        if node == sink {
            return true;
        }
        if let Some(neighbours) = graph.get(&node) {
            for neighbour in neighbours {
                if seen.insert(neighbour.clone()) {
                    queue.push_back(neighbour.clone());
                }
            }
        }
    }
    false
}

fn component_count(graph: &BTreeMap<String, Vec<String>>) -> usize {
    let mut seen = BTreeSet::new();
    let mut components = 0;
    for start in graph.keys() {
        if seen.contains(start) {
            continue;
        }
        components += 1;
        let mut queue = VecDeque::from([start.clone()]);
        seen.insert(start.clone());
        while let Some(node) = queue.pop_front() {
            if let Some(neighbours) = graph.get(&node) {
                for neighbour in neighbours {
                    if seen.insert(neighbour.clone()) {
                        queue.push_back(neighbour.clone());
                    }
                }
            }
        }
    }
    components
}

fn relative_change(value: usize, reference: usize) -> Option<f64> {
    if reference == 0 {
        None
    } else {
        Some((value as f64 - reference as f64) / reference as f64)
    }
}

fn build_report(scenario: &Scenario, input_sha256: String) -> Report {
    let node_catalog: BTreeMap<&str, &NodeDefinition> = scenario
        .node_catalog
        .iter()
        .map(|node| (node.node_id.as_str(), node))
        .collect();
    let edge_catalog: BTreeMap<&str, &EdgeDefinition> = scenario
        .edge_catalog
        .iter()
        .map(|edge| (edge.edge_id.as_str(), edge))
        .collect();

    let mut variants: Vec<&Variant> = scenario.variants.iter().collect();
    variants.sort_by(|left, right| left.alpha_percent.cmp(&right.alpha_percent));
    let reference = variants
        .iter()
        .find(|variant| variant.alpha_percent == 50)
        .expect("validated reference percentile");
    let reference_node_count = reference.present_node_ids.len();
    let reference_edge_count = reference.present_edge_ids.len();
    let (reference_component_count, reference_cycle_rank) =
        topology_metrics(reference, &edge_catalog);

    let mut variant_graphs = BTreeMap::new();
    let mut variant_results = Vec::new();
    let mut edge_incidence: BTreeMap<String, usize> = edge_catalog
        .keys()
        .map(|edge_id| (edge_id.to_string(), 0))
        .collect();
    let mut node_incidence: BTreeMap<String, usize> = node_catalog
        .keys()
        .map(|node_id| (node_id.to_string(), 0))
        .collect();

    for variant in &variants {
        let present_nodes: BTreeSet<String> = variant.present_node_ids.iter().cloned().collect();
        let present_edges: BTreeSet<String> = variant.present_edge_ids.iter().cloned().collect();
        for node in &present_nodes {
            *node_incidence.get_mut(node).expect("validated node") += 1;
        }
        for edge in &present_edges {
            *edge_incidence.get_mut(edge).expect("validated edge") += 1;
        }
        let graph = adjacency(&present_nodes, &present_edges, &edge_catalog);
        let components = component_count(&graph);
        let cycle_rank = present_edges.len() + components - present_nodes.len();
        let mut connected_terminal_pair_count = 0;
        for source in &scenario.source_node_ids {
            for sink in &scenario.sink_node_ids {
                if connected(&graph, source, sink) {
                    connected_terminal_pair_count += 1;
                }
            }
        }
        variant_graphs.insert(variant.variant_id.clone(), present_edges.clone());
        variant_results.push(VariantResult {
            variant_id: variant.variant_id.clone(),
            alpha_percent: variant.alpha_percent,
            perturbation_label: variant.perturbation_label.clone(),
            present_node_count: present_nodes.len(),
            present_edge_count: present_edges.len(),
            connected_component_count: components,
            cycle_rank,
            connected_terminal_pair_count,
            node_count_relative_change_from_reference: relative_change(
                present_nodes.len(),
                reference_node_count,
            ),
            edge_count_relative_change_from_reference: relative_change(
                present_edges.len(),
                reference_edge_count,
            ),
            component_count_relative_change_from_reference: relative_change(
                components,
                reference_component_count,
            ),
            cycle_rank_relative_change_from_reference: relative_change(
                cycle_rank,
                reference_cycle_rank,
            ),
        });
    }

    let mut node_presence: Vec<PresenceSummary> = node_incidence
        .into_iter()
        .map(|(feature_id, count)| PresenceSummary {
            feature_id,
            present_variant_count: count,
            total_variant_count: variants.len(),
            sensitivity_frequency: count as f64 / variants.len() as f64,
        })
        .collect();
    node_presence.sort_by(|left, right| left.feature_id.cmp(&right.feature_id));
    let mut edge_presence: Vec<PresenceSummary> = edge_incidence
        .into_iter()
        .map(|(feature_id, count)| PresenceSummary {
            feature_id,
            present_variant_count: count,
            total_variant_count: variants.len(),
            sensitivity_frequency: count as f64 / variants.len() as f64,
        })
        .collect();
    edge_presence.sort_by(|left, right| left.feature_id.cmp(&right.feature_id));
    variant_results.sort_by(|left, right| left.alpha_percent.cmp(&right.alpha_percent));

    let mut terminal_pair_results = Vec::new();
    for source in &scenario.source_node_ids {
        for sink in &scenario.sink_node_ids {
            let mut connected_count = 0;
            for variant in &variants {
                let nodes: BTreeSet<String> = variant.present_node_ids.iter().cloned().collect();
                let edges: BTreeSet<String> = variant.present_edge_ids.iter().cloned().collect();
                let graph = adjacency(&nodes, &edges, &edge_catalog);
                if connected(&graph, source, sink) {
                    connected_count += 1;
                }
            }
            let stability_class = if connected_count == variants.len() {
                "stable_connected"
            } else if connected_count == 0 {
                "stable_disconnected"
            } else {
                "segmentation_sensitive"
            };
            terminal_pair_results.push(TerminalPairResult {
                source_node_id: source.clone(),
                sink_node_id: sink.clone(),
                connected_variant_count: connected_count,
                total_variant_count: variants.len(),
                sensitivity_frequency: connected_count as f64 / variants.len() as f64,
                stability_class: stability_class.into(),
            });
        }
    }
    terminal_pair_results.sort_by(|left, right| {
        (&left.source_node_id, &left.sink_node_id)
            .cmp(&(&right.source_node_id, &right.sink_node_id))
    });

    let mut jaccard_values = Vec::new();
    for left in 0..variants.len() {
        for right in (left + 1)..variants.len() {
            let left_edges = variant_graphs
                .get(&variants[left].variant_id)
                .expect("variant graph exists");
            let right_edges = variant_graphs
                .get(&variants[right].variant_id)
                .expect("variant graph exists");
            let intersection = left_edges.intersection(right_edges).count();
            let union = left_edges.union(right_edges).count();
            jaccard_values.push(if union == 0 {
                1.0
            } else {
                intersection as f64 / union as f64
            });
        }
    }
    let jaccard_minimum = jaccard_values.iter().copied().fold(f64::INFINITY, f64::min);
    let jaccard_maximum = jaccard_values
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let jaccard_mean = jaccard_values.iter().sum::<f64>() / jaccard_values.len() as f64;

    Report {
        audit_version: VERSION.into(),
        fixture_status: FIXTURE_STATUS.into(),
        scenario_id: scenario.scenario_id.clone(),
        input_sha256,
        image_sha256: scenario.image.sha256.clone(),
        variant_count: variants.len(),
        reference_variant_id: reference.variant_id.clone(),
        node_catalog_count: node_catalog.len(),
        edge_catalog_count: edge_catalog.len(),
        source_count: scenario.source_node_ids.len(),
        sink_count: scenario.sink_node_ids.len(),
        node_presence,
        edge_presence,
        variant_results,
        terminal_pair_results,
        pairwise_edge_jaccard: JaccardSummary {
            pair_count: jaccard_values.len(),
            minimum: jaccard_minimum,
            maximum: jaccard_maximum,
            mean: jaccard_mean,
        },
        interpretation_boundary: CLAIM_CEILING.into(),
    }
}

fn topology_metrics(
    variant: &Variant,
    edge_catalog: &BTreeMap<&str, &EdgeDefinition>,
) -> (usize, usize) {
    let present_nodes: BTreeSet<String> = variant.present_node_ids.iter().cloned().collect();
    let present_edges: BTreeSet<String> = variant.present_edge_ids.iter().cloned().collect();
    let graph = adjacency(&present_nodes, &present_edges, edge_catalog);
    let components = component_count(&graph);
    let cycle_rank = present_edges.len() + components - present_nodes.len();
    (components, cycle_rank)
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let input_bytes = fs::read(input_path)?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input_bytes));
    let scenario: Scenario = serde_json::from_slice(&input_bytes)?;
    validate(&scenario).map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?;
    let report = build_report(&scenario, input_sha256);
    let output = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    file.write_all(&output)?;
    file.write_all(b"\n")?;
    file.sync_all()?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: segmentation-ensemble-audit <input.json> <new-output.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("segmentation-ensemble-audit: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-segmentation-ensemble.json"
        ))
        .expect("synthetic fixture parses")
    }

    fn report() -> Report {
        let scenario = scenario();
        validate(&scenario).expect("fixture validates");
        build_report(&scenario, "0".repeat(64))
    }

    #[test]
    fn reports_segmentation_sensitive_source_sink_pair() {
        let report = report();
        assert_eq!(report.variant_count, 5);
        assert_eq!(
            report.terminal_pair_results[0].stability_class,
            "segmentation_sensitive"
        );
        assert_eq!(report.terminal_pair_results[0].connected_variant_count, 4);
    }

    #[test]
    fn reports_reference_and_variant_topology() {
        let report = report();
        assert_eq!(report.reference_variant_id, "alpha-50");
        let reference = report
            .variant_results
            .iter()
            .find(|variant| variant.alpha_percent == 50)
            .unwrap();
        assert_eq!(reference.present_edge_count, 6);
        assert_eq!(reference.cycle_rank, 1);
        let strict = report
            .variant_results
            .iter()
            .find(|variant| variant.alpha_percent == 90)
            .unwrap();
        assert_eq!(strict.connected_component_count, 2);
        assert_eq!(strict.cycle_rank, 0);
    }

    #[test]
    fn reports_edge_sensitivity_frequency_not_probability() {
        let report = report();
        let upper_b = report
            .edge_presence
            .iter()
            .find(|e| e.feature_id == "upper-b")
            .unwrap();
        assert_eq!(upper_b.present_variant_count, 3);
        assert!((upper_b.sensitivity_frequency - 0.6).abs() < 1e-12);
    }

    #[test]
    fn pairwise_jaccard_detects_identical_and_different_graphs() {
        let report = report();
        assert_eq!(report.pairwise_edge_jaccard.pair_count, 10);
        assert!(report.pairwise_edge_jaccard.maximum == 1.0);
        assert!(report.pairwise_edge_jaccard.minimum < 1.0);
    }

    #[test]
    fn rejects_missing_reference_percentile() {
        let mut scenario = scenario();
        scenario
            .variants
            .retain(|variant| variant.alpha_percent != 50);
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_duplicate_percentiles() {
        let mut scenario = scenario();
        scenario.variants[0].alpha_percent = 50;
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_malformed_image_digest() {
        let mut scenario = scenario();
        scenario.image.sha256 = "bad".into();
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_variant_digest_mismatch() {
        let mut scenario = scenario();
        scenario.variants[0].network_artifact_sha256 = "not-a-digest".into();
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_missing_source_or_sink_nodes() {
        let mut scenario = scenario();
        scenario.variants[0].present_node_ids.remove(0);
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_edge_with_missing_endpoint() {
        let mut scenario = scenario();
        scenario.variants[0]
            .present_node_ids
            .retain(|node| node != "upper");
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_self_loop_in_canonical_edge_catalog() {
        let mut scenario = scenario();
        scenario.edge_catalog[0].to_node_id = scenario.edge_catalog[0].from_node_id.clone();
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_unknown_canonical_edge_endpoint() {
        let mut scenario = scenario();
        scenario.edge_catalog[0].from_node_id = "unknown-node".into();
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_unknown_edge_reference() {
        let mut scenario = scenario();
        scenario.variants[0]
            .present_edge_ids
            .push("unknown-edge".into());
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_duplicate_variant_identity() {
        let mut scenario = scenario();
        scenario.variants[1].variant_id = scenario.variants[0].variant_id.clone();
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_ambiguous_duplicate_node_and_edge_ids() {
        let mut scenario = scenario();
        let duplicate_edge_id = scenario.variants[0].present_edge_ids[0].clone();
        scenario.variants[0]
            .present_edge_ids
            .push(duplicate_edge_id);
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn relative_change_is_undefined_for_zero_reference() {
        assert_eq!(relative_change(1, 0), None);
        assert_eq!(relative_change(0, 0), None);
        assert_eq!(relative_change(2, 1), Some(1.0));
    }

    #[test]
    fn stable_connected_when_all_variants_connect() {
        let mut scenario = scenario();
        let all_edges: Vec<String> = scenario
            .edge_catalog
            .iter()
            .map(|edge| edge.edge_id.clone())
            .collect();
        for variant in &mut scenario.variants {
            variant.present_edge_ids = all_edges.clone();
        }
        let report = build_report(&scenario, "0".repeat(64));
        assert_eq!(
            report.terminal_pair_results[0].stability_class,
            "stable_connected"
        );
    }

    #[test]
    fn stable_disconnected_when_no_variant_connects() {
        let mut scenario = scenario();
        for variant in &mut scenario.variants {
            variant
                .present_edge_ids
                .retain(|edge| edge != "upper-b" && edge != "lower-b");
        }
        let report = build_report(&scenario, "0".repeat(64));
        assert_eq!(
            report.terminal_pair_results[0].stability_class,
            "stable_disconnected"
        );
    }
}
