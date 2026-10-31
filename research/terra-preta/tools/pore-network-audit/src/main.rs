use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    cmp::{Ordering, Reverse},
    collections::{BinaryHeap, HashMap, HashSet},
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const AUDIT_VERSION: &str = "0.1.0";
const GEOMETRY_TOLERANCE: f64 = 1.0e-9;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    site_id: String,
    sample_id: String,
    horizon_id: String,
    image: ImageMetadata,
    segmentation: SegmentationMetadata,
    network: NetworkMetadata,
    threshold_sweep_um: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ImageMetadata {
    dimensions_voxels_xyz: [usize; 3],
    voxel_spacing_um_xyz: [f64; 3],
    /// v0.1 currently supports z as the vertical axis; importers must reorder axes explicitly.
    vertical_axis: String,
    source_image_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SegmentationMetadata {
    method: String,
    threshold_description: String,
    software_name: String,
    software_version: String,
    output_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NetworkMetadata {
    extractor: String,
    extractor_version: String,
    parameter_summary: String,
    source_network_sha256: String,
    pores: Vec<Pore>,
    throats: Vec<Throat>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Pore {
    pore_id: String,
    centroid_voxels_xyz: [f64; 3],
    volume_um3: f64,
    inscribed_radius_um: f64,
    touches_top_boundary: bool,
    touches_bottom_boundary: bool,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Throat {
    throat_id: String,
    pore_a: String,
    pore_b: String,
    total_length_um: f64,
    direct_length_um: f64,
    inscribed_radius_um: f64,
}

#[derive(Clone, Copy, Debug)]
struct QueueState {
    score: f64,
    node: usize,
}

impl PartialEq for QueueState {
    fn eq(&self, other: &Self) -> bool {
        self.score.to_bits() == other.score.to_bits() && self.node == other.node
    }
}

impl Eq for QueueState {}

impl PartialOrd for QueueState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueueState {
    fn cmp(&self, other: &Self) -> Ordering {
        self.score
            .total_cmp(&other.score)
            .then_with(|| self.node.cmp(&other.node))
    }
}

#[derive(Clone, Debug)]
struct IndexedThroat {
    a: usize,
    b: usize,
    length_um: f64,
    radius_um: f64,
}

#[derive(Serialize)]
struct ThresholdResult {
    throat_radius_threshold_um: f64,
    connected_component_count: usize,
    top_bottom_percolating_component_count: usize,
    top_bottom_percolates: bool,
    percolating_pore_volume_fraction: f64,
    largest_component_pore_volume_fraction: f64,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_image_sha256: String,
    segmentation_output_sha256: String,
    source_network_sha256: String,
    study_id: String,
    site_id: String,
    sample_id: String,
    horizon_id: String,
    image_dimensions_voxels_xyz: [usize; 3],
    voxel_spacing_um_xyz: [f64; 3],
    vertical_axis: String,
    sample_volume_um3: f64,
    total_extracted_pore_volume_um3: f64,
    extracted_pore_volume_fraction: f64,
    pore_count: usize,
    throat_count: usize,
    connected_component_count_at_zero_threshold: usize,
    largest_component_pore_volume_fraction: f64,
    top_bottom_percolating_component_count_at_zero_threshold: usize,
    top_bottom_percolates_at_zero_threshold: bool,
    percolating_pore_volume_fraction_at_zero_threshold: f64,
    direct_spanning_pore_count: usize,
    widest_top_bottom_path_bottleneck_radius_um: Option<f64>,
    shortest_top_bottom_path_length_um: Option<f64>,
    straight_sample_height_um: f64,
    shortest_path_tortuosity: Option<f64>,
    threshold_analysis: Vec<ThresholdResult>,
    segmentation_method: String,
    segmentation_threshold_description: String,
    segmentation_software: String,
    segmentation_software_version: String,
    network_extractor: String,
    network_extractor_version: String,
    network_parameter_summary: String,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn valid_sha256(value: &str, label: &str) -> Result<(), String> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!("{label} must contain exactly 64 hexadecimal characters"));
    }
    Ok(())
}

fn finite_positive(value: f64, label: &str) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{label} must be finite and greater than zero"));
    }
    Ok(())
}

fn validate(input: &Input) -> Result<(Vec<IndexedThroat>, HashMap<String, usize>, f64, f64), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }

    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.sample_id, "sample_id"),
        (&input.horizon_id, "horizon_id"),
        (&input.segmentation.method, "segmentation.method"),
        (&input.segmentation.threshold_description, "segmentation.threshold_description"),
        (&input.segmentation.software_name, "segmentation.software_name"),
        (&input.segmentation.software_version, "segmentation.software_version"),
        (&input.network.extractor, "network.extractor"),
        (&input.network.extractor_version, "network.extractor_version"),
        (&input.network.parameter_summary, "network.parameter_summary"),
    ] {
        nonblank(value, label)?;
    }

    valid_sha256(&input.image.source_image_sha256, "image.source_image_sha256")?;
    if input.image.vertical_axis != "z" {
        return Err("v0.1 requires image.vertical_axis = \"z\"; reorder dimensions, spacing and pore coordinates before import".to_owned());
    }
    valid_sha256(&input.segmentation.output_sha256, "segmentation.output_sha256")?;
    valid_sha256(&input.network.source_network_sha256, "network.source_network_sha256")?;

    if input.image.dimensions_voxels_xyz.iter().any(|dimension| *dimension < 2) {
        return Err("all image dimensions must be at least two voxels".to_owned());
    }
    for (axis, spacing) in input.image.voxel_spacing_um_xyz.iter().enumerate() {
        finite_positive(*spacing, &format!("voxel_spacing_um_xyz[{axis}]"))?;
    }

    let [nx, ny, nz] = input.image.dimensions_voxels_xyz;
    let [sx, sy, sz] = input.image.voxel_spacing_um_xyz;
    let sample_volume_um3 = (nx as f64) * (ny as f64) * (nz as f64) * sx * sy * sz;
    let straight_height_um = (nz.saturating_sub(1) as f64) * sz;
    if !sample_volume_um3.is_finite() || sample_volume_um3 <= 0.0 || !straight_height_um.is_finite() || straight_height_um <= 0.0 {
        return Err("image dimensions/spacing produce invalid sample geometry".to_owned());
    }

    if input.network.pores.is_empty() {
        return Err("network.pores must not be empty".to_owned());
    }
    if input.threshold_sweep_um.is_empty() {
        return Err("threshold_sweep_um must contain at least one threshold".to_owned());
    }

    let mut pore_index = HashMap::new();
    let mut pore_volume_total_um3 = 0.0;
    let mut top_count = 0usize;
    let mut bottom_count = 0usize;

    for (index, pore) in input.network.pores.iter().enumerate() {
        nonblank(&pore.pore_id, &format!("pores[{index}].pore_id"))?;
        if pore_index.insert(pore.pore_id.clone(), index).is_some() {
            return Err(format!("duplicate pore_id {:?}", pore.pore_id));
        }
        for axis in 0..3 {
            let coordinate = pore.centroid_voxels_xyz[axis];
            let upper = (input.image.dimensions_voxels_xyz[axis] - 1) as f64;
            if !coordinate.is_finite() || coordinate < 0.0 || coordinate > upper {
                return Err(format!("pores[{index}].centroid_voxels_xyz[{axis}] lies outside the image volume"));
            }
        }
        finite_positive(pore.volume_um3, &format!("pores[{index}].volume_um3"))?;
        finite_positive(pore.inscribed_radius_um, &format!("pores[{index}].inscribed_radius_um"))?;
        pore_volume_total_um3 += pore.volume_um3;
        if !pore_volume_total_um3.is_finite() {
            return Err("extracted pore volume total overflowed".to_owned());
        }
        if pore.touches_top_boundary {
            top_count += 1;
        }
        if pore.touches_bottom_boundary {
            bottom_count += 1;
        }
    }

    if pore_volume_total_um3 > sample_volume_um3 * (1.0 + GEOMETRY_TOLERANCE) {
        return Err("sum of extracted pore volumes exceeds the imaged sample volume".to_owned());
    }
    if top_count == 0 || bottom_count == 0 {
        return Err("network must contain at least one top-boundary and one bottom-boundary pore".to_owned());
    }

    let mut throat_ids = HashSet::new();
    let mut connections = HashSet::new();
    let mut indexed = Vec::with_capacity(input.network.throats.len());

    for (index, throat) in input.network.throats.iter().enumerate() {
        nonblank(&throat.throat_id, &format!("throats[{index}].throat_id"))?;
        if !throat_ids.insert(throat.throat_id.as_str()) {
            return Err(format!("duplicate throat_id {:?}", throat.throat_id));
        }
        let Some(&a) = pore_index.get(&throat.pore_a) else {
            return Err(format!("throats[{index}] references unknown pore_a {:?}", throat.pore_a));
        };
        let Some(&b) = pore_index.get(&throat.pore_b) else {
            return Err(format!("throats[{index}] references unknown pore_b {:?}", throat.pore_b));
        };
        if a == b {
            return Err(format!("throats[{index}] must connect two distinct pores"));
        }
        let pair = if a < b { (a, b) } else { (b, a) };
        if !connections.insert(pair) {
            return Err(format!("duplicate undirected pore connection between {:?} and {:?}", throat.pore_a, throat.pore_b));
        }

        finite_positive(throat.total_length_um, &format!("throats[{index}].total_length_um"))?;
        finite_positive(throat.direct_length_um, &format!("throats[{index}].direct_length_um"))?;
        finite_positive(throat.inscribed_radius_um, &format!("throats[{index}].inscribed_radius_um"))?;
        if throat.total_length_um + GEOMETRY_TOLERANCE < throat.direct_length_um {
            return Err(format!("throats[{index}].total_length_um is shorter than direct_length_um"));
        }
        indexed.push(IndexedThroat {
            a,
            b,
            length_um: throat.total_length_um,
            radius_um: throat.inscribed_radius_um,
        });
    }

    let mut thresholds = HashSet::new();
    for (index, threshold) in input.threshold_sweep_um.iter().enumerate() {
        if !threshold.is_finite() || *threshold < 0.0 {
            return Err(format!("threshold_sweep_um[{index}] must be finite and non-negative"));
        }
        if !thresholds.insert(threshold.to_bits()) {
            return Err(format!("duplicate throat-radius threshold {:?}", threshold));
        }
    }

    Ok((indexed, pore_index, sample_volume_um3, pore_volume_total_um3))
}

fn adjacency(
    pore_count: usize,
    throats: &[IndexedThroat],
    minimum_radius_um: f64,
) -> Vec<Vec<(usize, usize)>> {
    let mut graph = vec![Vec::new(); pore_count];
    for (edge_index, throat) in throats.iter().enumerate() {
        if throat.radius_um >= minimum_radius_um {
            graph[throat.a].push((throat.b, edge_index));
            graph[throat.b].push((throat.a, edge_index));
        }
    }
    graph
}

fn component_metrics(input: &Input, graph: &[Vec<(usize, usize)>], total_pore_volume_um3: f64) -> ThresholdResult {
    let pore_count = input.network.pores.len();
    let mut visited = vec![false; pore_count];
    let mut components = 0usize;
    let mut percolating_components = 0usize;
    let mut percolating_volume = 0.0;
    let mut largest_component_volume = 0.0;

    for start in 0..pore_count {
        if visited[start] {
            continue;
        }
        components += 1;
        let mut stack = vec![start];
        visited[start] = true;
        let mut volume = 0.0;
        let mut has_top = false;
        let mut has_bottom = false;

        while let Some(node) = stack.pop() {
            let pore = &input.network.pores[node];
            volume += pore.volume_um3;
            has_top |= pore.touches_top_boundary;
            has_bottom |= pore.touches_bottom_boundary;
            for &(neighbor, _) in &graph[node] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }

        largest_component_volume = largest_component_volume.max(volume);
        if has_top && has_bottom {
            percolating_components += 1;
            percolating_volume += volume;
        }
    }

    ThresholdResult {
        throat_radius_threshold_um: 0.0,
        connected_component_count: components,
        top_bottom_percolating_component_count: percolating_components,
        top_bottom_percolates: percolating_components > 0,
        percolating_pore_volume_fraction: percolating_volume / total_pore_volume_um3,
        largest_component_pore_volume_fraction: largest_component_volume / total_pore_volume_um3,
    }
}

fn widest_path_bottleneck(input: &Input, graph: &[Vec<(usize, usize)>], throats: &[IndexedThroat]) -> Option<f64> {
    // A single pore spanning both faces has no finite throat bottleneck; its
    // presence makes the maximum-minimum-throat-radius metric undefined.
    if input.network.pores.iter().any(|pore| pore.touches_top_boundary && pore.touches_bottom_boundary) {
        return None;
    }
    let n = input.network.pores.len();
    let mut best = vec![0.0f64; n];
    let mut heap = BinaryHeap::new();

    for (index, pore) in input.network.pores.iter().enumerate() {
        if pore.touches_top_boundary {
            best[index] = f64::INFINITY;
            heap.push(QueueState { score: f64::INFINITY, node: index });
        }
    }

    while let Some(state) = heap.pop() {
        if state.score < best[state.node] {
            continue;
        }
        if input.network.pores[state.node].touches_bottom_boundary {
            return Some(state.score);
        }

        for &(neighbor, edge_index) in &graph[state.node] {
            let candidate = state.score.min(throats[edge_index].radius_um);
            if candidate > best[neighbor] {
                best[neighbor] = candidate;
                heap.push(QueueState { score: candidate, node: neighbor });
            }
        }
    }
    None
}

fn shortest_top_bottom_path(input: &Input, graph: &[Vec<(usize, usize)>], throats: &[IndexedThroat]) -> Option<f64> {
    let n = input.network.pores.len();
    let mut distance = vec![f64::INFINITY; n];
    let mut heap = BinaryHeap::new();
    let spacing_z = input.image.voxel_spacing_um_xyz[2];

    for (index, pore) in input.network.pores.iter().enumerate() {
        if pore.touches_top_boundary {
            let start_offset = pore.centroid_voxels_xyz[2] * spacing_z;
            if start_offset < distance[index] {
                distance[index] = start_offset;
                heap.push(Reverse(QueueState { score: start_offset, node: index }));
            }
        }
    }

    let mut best_boundary_path = f64::INFINITY;
    while let Some(Reverse(state)) = heap.pop() {
        if state.score > distance[state.node] {
            continue;
        }
        if state.score >= best_boundary_path {
            break;
        }
        let pore = &input.network.pores[state.node];
        if pore.touches_bottom_boundary {
            let bottom_offset = ((input.image.dimensions_voxels_xyz[2] - 1) as f64 - pore.centroid_voxels_xyz[2]) * spacing_z;
            best_boundary_path = best_boundary_path.min(state.score + bottom_offset);
            continue;
        }
        for &(neighbor, edge_index) in &graph[state.node] {
            let candidate = state.score + throats[edge_index].length_um;
            if candidate < distance[neighbor] {
                distance[neighbor] = candidate;
                heap.push(Reverse(QueueState { score: candidate, node: neighbor }));
            }
        }
    }
    best_boundary_path.is_finite().then_some(best_boundary_path)
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (throats, _pore_index, sample_volume_um3, total_pore_volume_um3) = validate(input)?;
    let pore_count = input.network.pores.len();
    let all_graph = adjacency(pore_count, &throats, 0.0);
    let baseline_components = component_metrics(input, &all_graph, total_pore_volume_um3);
    let widest_path = widest_path_bottleneck(input, &all_graph, &throats);
    let shortest_path = shortest_top_bottom_path(input, &all_graph, &throats);
    let straight_height_um = (input.image.dimensions_voxels_xyz[2] - 1) as f64
        * input.image.voxel_spacing_um_xyz[2];
    let tortuosity = shortest_path.map(|length| length / straight_height_um);
    let direct_spanning_pore_count = input.network.pores.iter()
        .filter(|pore| pore.touches_top_boundary && pore.touches_bottom_boundary)
        .count();

    let mut thresholds = input.threshold_sweep_um.clone();
    thresholds.sort_by(f64::total_cmp);
    let threshold_analysis = thresholds
        .into_iter()
        .map(|threshold| {
            let graph = adjacency(pore_count, &throats, threshold);
            let mut result = component_metrics(input, &graph, total_pore_volume_um3);
            result.throat_radius_threshold_um = threshold;
            result
        })
        .collect();

    let mut warnings = vec![
        "This audit evaluates an already extracted pore/throat graph; it does not segment raw CT images, solve flow, or establish that geometrically connected pores are water-filled during an event.".to_owned(),
        "Network extraction and segmentation can change pore/throat counts and connectivity. Preserve raw-image and segmentation digests, software versions, thresholds and extraction parameters.".to_owned(),
        "Top/bottom percolation is a graph-topology statement for the supplied segmentation, not a measured hydraulic conductivity or proof of field-scale preferential flow.".to_owned(),
        "The shortest-path tortuosity uses extracted total throat lengths plus centroid-to-boundary offsets along the vertical axis; it is not a flow-weighted tortuosity.".to_owned(),
        "The widest-path bottleneck is the maximum possible minimum throat inscribed radius across top-to-bottom graph paths; it is not a capillary-entry or hydraulic conductance calculation.".to_owned(),
    ];
    if let Some(value) = tortuosity {
        if value + GEOMETRY_TOLERANCE < 1.0 {
            warnings.push("computed shortest path tortuosity is below one; inspect axis conventions, segmentation geometry and throat-length definitions".to_owned());
        }
    } else {
        warnings.push("no top-to-bottom path exists in the unfiltered graph".to_owned());
    }
    if direct_spanning_pore_count > 0 {
        warnings.push("one or more single pore regions touch both top and bottom faces; geometric percolation is recorded, but no finite throat bottleneck radius is defined for that direct-pore path".to_owned());
    }

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        source_image_sha256: input.image.source_image_sha256.clone(),
        segmentation_output_sha256: input.segmentation.output_sha256.clone(),
        source_network_sha256: input.network.source_network_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        sample_id: input.sample_id.clone(),
        horizon_id: input.horizon_id.clone(),
        image_dimensions_voxels_xyz: input.image.dimensions_voxels_xyz,
        voxel_spacing_um_xyz: input.image.voxel_spacing_um_xyz,
        vertical_axis: input.image.vertical_axis.clone(),
        sample_volume_um3,
        total_extracted_pore_volume_um3,
        extracted_pore_volume_fraction: total_pore_volume_um3 / sample_volume_um3,
        pore_count: input.network.pores.len(),
        throat_count: input.network.throats.len(),
        connected_component_count_at_zero_threshold: baseline_components.connected_component_count,
        largest_component_pore_volume_fraction: baseline_components.largest_component_pore_volume_fraction,
        top_bottom_percolating_component_count_at_zero_threshold: baseline_components.top_bottom_percolating_component_count,
        top_bottom_percolates_at_zero_threshold: baseline_components.top_bottom_percolates,
        percolating_pore_volume_fraction_at_zero_threshold: baseline_components.percolating_pore_volume_fraction,
        direct_spanning_pore_count,
        widest_top_bottom_path_bottleneck_radius_um: widest_path,
        shortest_top_bottom_path_length_um: shortest_path,
        straight_sample_height_um: straight_height_um,
        shortest_path_tortuosity: tortuosity,
        threshold_analysis,
        segmentation_method: input.segmentation.method.clone(),
        segmentation_threshold_description: input.segmentation.threshold_description.clone(),
        segmentation_software: input.segmentation.software_name.clone(),
        segmentation_software_version: input.segmentation.software_version.clone(),
        network_extractor: input.network.extractor.clone(),
        network_extractor_version: input.network.extractor_version.clone(),
        network_parameter_summary: input.network.parameter_summary.clone(),
        interpretation_boundary: "geometric graph connectivity and radius-weighted percolation diagnostics; not a hydraulic simulation, dynamic wetting state, measured flux or field-scale preferential-flow ground truth",
        warnings,
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: pore-network-audit <network-input.json> <new-report.json>",
        ).into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("refusing overwrite or cannot create report: {error}"),
            )
        })?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "pore network audited: {} pores; {} throats; top-bottom percolates={}; bottleneck={:?} um; input_sha256={}",
        report.pore_count,
        report.throat_count,
        report.top_bottom_percolates_at_zero_threshold,
        report.widest_top_bottom_path_bottleneck_radius_um,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("pore network audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pore(id: &str, z: f64, volume: f64, top: bool, bottom: bool) -> Pore {
        Pore {
            pore_id: id.to_owned(),
            centroid_voxels_xyz: [5.0, 5.0, z],
            volume_um3: volume,
            inscribed_radius_um: 10.0,
            touches_top_boundary: top,
            touches_bottom_boundary: bottom,
        }
    }

    fn throat(id: &str, a: &str, b: &str, length: f64, radius: f64) -> Throat {
        Throat {
            throat_id: id.to_owned(),
            pore_a: a.to_owned(),
            pore_b: b.to_owned(),
            total_length_um: length,
            direct_length_um: length,
            inscribed_radius_um: radius,
        }
    }

    fn base(pores: Vec<Pore>, throats: Vec<Throat>, thresholds: Vec<f64>) -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            sample_id: "CORE-01".to_owned(),
            horizon_id: "0-10cm".to_owned(),
            image: ImageMetadata {
                dimensions_voxels_xyz: [10, 10, 100],
                voxel_spacing_um_xyz: [100.0, 100.0, 100.0],
                vertical_axis: "z".to_owned(),
                source_image_sha256: "a".repeat(64),
            },
            segmentation: SegmentationMetadata {
                method: "synthetic labelled volume".to_owned(),
                threshold_description: "known truth".to_owned(),
                software_name: "synthetic".to_owned(),
                software_version: "0.1".to_owned(),
                output_sha256: "b".repeat(64),
            },
            network: NetworkMetadata {
                extractor: "synthetic graph".to_owned(),
                extractor_version: "0.1".to_owned(),
                parameter_summary: "explicit test network".to_owned(),
                source_network_sha256: "c".repeat(64),
                pores,
                throats,
            },
            threshold_sweep_um: thresholds,
        }
    }

    #[test]
    fn identifies_connected_top_bottom_path_and_bottleneck() {
        let input = base(
            vec![
                pore("top", 0.0, 100.0, true, false),
                pore("mid", 50.0, 100.0, false, false),
                pore("bottom", 99.0, 100.0, false, true),
            ],
            vec![
                throat("t1", "top", "mid", 5000.0, 80.0),
                throat("t2", "mid", "bottom", 4900.0, 50.0),
            ],
            vec![0.0, 40.0, 60.0],
        );
        let report = evaluate(&input, "d".repeat(64)).unwrap();
        assert!(report.top_bottom_percolates_at_zero_threshold);
        assert_eq!(report.widest_top_bottom_path_bottleneck_radius_um, Some(50.0));
        assert_eq!(report.shortest_top_bottom_path_length_um, Some(9900.0));
        assert_eq!(report.shortest_path_tortuosity, Some(1.0));
        assert_eq!(report.threshold_analysis[0].top_bottom_percolates, true);
        assert_eq!(report.threshold_analysis[1].top_bottom_percolates, true);
        assert_eq!(report.threshold_analysis[2].top_bottom_percolates, false);
    }

    #[test]
    fn does_not_certify_disconnected_top_and_bottom_components() {
        let input = base(
            vec![
                pore("top", 0.0, 100.0, true, false),
                pore("middle", 50.0, 100.0, false, false),
                pore("bottom", 99.0, 100.0, false, true),
            ],
            vec![throat("t1", "top", "middle", 5000.0, 50.0)],
            vec![0.0, 60.0],
        );
        let report = evaluate(&input, "e".repeat(64)).unwrap();
        assert!(!report.top_bottom_percolates_at_zero_threshold);
        assert_eq!(report.widest_top_bottom_path_bottleneck_radius_um, None);
        assert_eq!(report.shortest_top_bottom_path_length_um, None);
        assert_eq!(report.percolating_pore_volume_fraction_at_zero_threshold, 0.0);
    }

    #[test]
    fn widest_path_chooses_larger_bottleneck_not_shortest_graph_path() {
        let input = base(
            vec![
                pore("top", 0.0, 100.0, true, false),
                pore("wide-mid", 40.0, 100.0, false, false),
                pore("narrow-mid", 60.0, 100.0, false, false),
                pore("bottom", 99.0, 100.0, false, true),
            ],
            vec![
                throat("wide-a", "top", "wide-mid", 4000.0, 30.0),
                throat("wide-b", "wide-mid", "bottom", 5900.0, 30.0),
                throat("narrow-a", "top", "narrow-mid", 2000.0, 60.0),
                throat("narrow-b", "narrow-mid", "bottom", 3900.0, 20.0),
            ],
            vec![0.0, 25.0, 35.0],
        );
        let report = evaluate(&input, "f".repeat(64)).unwrap();
        assert_eq!(report.widest_top_bottom_path_bottleneck_radius_um, Some(30.0));
        assert_eq!(report.shortest_top_bottom_path_length_um, Some(5900.0));
    }

    #[test]
    fn reports_pore_volume_percolation_fraction_separately_from_total_porosity() {
        let input = base(
            vec![
                pore("top", 0.0, 50_000_000.0, true, false),
                pore("bottom", 99.0, 50_000_000.0, false, true),
                pore("dead-end", 40.0, 100_000_000.0, false, false),
            ],
            vec![throat("path", "top", "bottom", 9900.0, 40.0)],
            vec![0.0],
        );
        let report = evaluate(&input, "1".repeat(64)).unwrap();
        assert!((report.extracted_pore_volume_fraction - 0.02).abs() < 1.0e-12);
        assert!((report.percolating_pore_volume_fraction_at_zero_threshold - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn rejects_undeclared_or_unsupported_vertical_axis() {
        let mut input = base(
            vec![pore("top", 0.0, 100.0, true, false), pore("bottom", 99.0, 100.0, false, true)],
            vec![throat("path", "top", "bottom", 9900.0, 20.0)],
            vec![0.0],
        );
        input.image.vertical_axis = "y".to_owned();
        assert!(validate(&input).unwrap_err().contains("vertical_axis"));
    }

    #[test]
    fn single_pore_spanning_both_boundaries_does_not_emit_infinite_bottleneck() {
        let input = base(
            vec![pore("spanning", 49.5, 100.0, true, true)],
            vec![],
            vec![0.0],
        );
        let report = evaluate(&input, "9".repeat(64)).unwrap();
        assert!(report.top_bottom_percolates_at_zero_threshold);
        assert_eq!(report.widest_top_bottom_path_bottleneck_radius_um, None);
        assert_eq!(report.shortest_top_bottom_path_length_um, Some(9900.0));
        assert_eq!(report.direct_spanning_pore_count, 1);
    }

    #[test]
    fn rejects_unknown_pore_and_duplicate_connection() {
        let mut input = base(
            vec![pore("top", 0.0, 100.0, true, false), pore("bottom", 99.0, 100.0, false, true)],
            vec![throat("t1", "top", "missing", 9900.0, 20.0)],
            vec![0.0],
        );
        assert!(validate(&input).unwrap_err().contains("unknown pore_a"));
        input.network.throats = vec![
            throat("t1", "top", "bottom", 9900.0, 20.0),
            throat("t2", "bottom", "top", 9900.0, 20.0),
        ];
        assert!(validate(&input).unwrap_err().contains("duplicate undirected pore connection"));
    }

    #[test]
    fn rejects_invalid_digests_and_pore_volume_overflow() {
        let mut input = base(
            vec![pore("top", 0.0, 100.0, true, false), pore("bottom", 99.0, 100.0, false, true)],
            vec![throat("t1", "top", "bottom", 9900.0, 20.0)],
            vec![0.0],
        );
        input.image.source_image_sha256 = "bad".to_owned();
        assert!(validate(&input).unwrap_err().contains("source_image_sha256"));
        input.image.source_image_sha256 = "a".repeat(64);
        input.network.pores[1].volume_um3 = 1.0e12;
        assert!(validate(&input).unwrap_err().contains("exceeds the imaged sample volume"));
    }
}
