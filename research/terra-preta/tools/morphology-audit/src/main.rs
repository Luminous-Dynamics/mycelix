use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::HashMap,
    env,
    error::Error,
    fs::{self, OpenOptions},
    io::{self, Write},
    path::Path,
};

const ORIENTATION_BIN_DEGREES: f64 = 10.0;
const ORIENTATION_BIN_COUNT: usize = 18;

#[derive(Clone, Debug)]
struct Columns {
    area: usize,
    perimeter: usize,
    min_feret: usize,
    feret: usize,
    angle: usize,
}

#[derive(Clone, Debug)]
struct Pore {
    area_mm2: f64,
    perimeter_mm: f64,
    min_feret_mm: f64,
    feret_mm: f64,
    feret_angle_degrees: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Distribution {
    count: usize,
    min: f64,
    max: f64,
    mean: f64,
    median: f64,
    p90: f64,
    p95: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ColumnMapping {
    area: String,
    perimeter: String,
    min_feret: String,
    feret: String,
    feret_angle: String,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    tool_version: &'static str,
    input_sha256: String,
    horizon_label: String,
    image_resolution_um_per_pixel: f64,
    cross_section_area_mm2: Option<f64>,
    minimum_object_area_mm2: f64,
    input_object_count: usize,
    excluded_below_minimum_area_count: usize,
    detected_object_count: usize,
    summed_projected_pore_area_mm2: f64,
    projected_pore_area_fraction: Option<f64>,
    area_mm2: Distribution,
    perimeter_mm: Distribution,
    min_feret_mm: Distribution,
    feret_mm: Distribution,
    feret_angle_degrees: Distribution,
    circularity_4pi_area_over_perimeter_squared: Distribution,
    feret_aspect_ratio: Distribution,
    circularity_above_one_count: usize,
    orientation_histogram_10deg_bins: [usize; ORIENTATION_BIN_COUNT],
    orientation_nematic_order_x: f64,
    orientation_nematic_order_y: f64,
    orientation_nematic_strength: f64,
    dominant_orientation_degrees: Option<f64>,
    column_mapping: ColumnMapping,
    dimensionality_boundary: &'static str,
    warnings: Vec<String>,
}

fn quantile(sorted: &[f64], probability: f64) -> f64 {
    if sorted.len() == 1 {
        return sorted[0];
    }
    let position = probability * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction
}

fn distribution(values: &[f64]) -> Result<Distribution, String> {
    if values.is_empty() {
        return Err("no valid macropore rows were found".to_owned());
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let sum: f64 = sorted.iter().sum();
    Ok(Distribution {
        count: sorted.len(),
        min: sorted[0],
        max: sorted[sorted.len() - 1],
        mean: sum / sorted.len() as f64,
        median: quantile(&sorted, 0.50),
        p90: quantile(&sorted, 0.90),
        p95: quantile(&sorted, 0.95),
    })
}

fn column_index(headers: &csv::StringRecord, name: &str) -> Result<usize, String> {
    headers
        .iter()
        .position(|header| header == name)
        .ok_or_else(|| format!("required ImageJ column {name:?} was not found"))
}

fn number(record: &csv::StringRecord, index: usize, name: &str, row: usize) -> Result<f64, String> {
    let raw = record
        .get(index)
        .ok_or_else(|| format!("row {row}: missing field for column {name:?}"))?;
    let value: f64 = raw
        .trim()
        .parse()
        .map_err(|error| format!("row {row}: cannot parse {name:?} value {raw:?}: {error}"))?;
    if !value.is_finite() || value < 0.0 {
        return Err(format!("row {row}: {name:?} must be finite and non-negative"));
    }
    Ok(value)
}

fn parse_pores<R: io::Read>(
    reader: R,
    mapping: &ColumnMapping,
) -> Result<Vec<Pore>, String> {
    let mut csv_reader = csv::ReaderBuilder::new()
        .flexible(false)
        .trim(csv::Trim::Headers)
        .from_reader(reader);
    let headers = csv_reader
        .headers()
        .map_err(|error| format!("CSV header parse failed: {error}"))?
        .clone();

    let columns = Columns {
        area: column_index(&headers, &mapping.area)?,
        perimeter: column_index(&headers, &mapping.perimeter)?,
        min_feret: column_index(&headers, &mapping.min_feret)?,
        feret: column_index(&headers, &mapping.feret)?,
        angle: column_index(&headers, &mapping.feret_angle)?,
    };

    let mut pores = Vec::new();
    for (offset, result) in csv_reader.records().enumerate() {
        let row_number = offset + 2;
        let record = result.map_err(|error| format!("row {row_number}: CSV parse failed: {error}"))?;
        if record.iter().all(|value| value.trim().is_empty()) {
            continue;
        }
        let pore = Pore {
            area_mm2: number(&record, columns.area, &mapping.area, row_number)?,
            perimeter_mm: number(&record, columns.perimeter, &mapping.perimeter, row_number)?,
            min_feret_mm: number(&record, columns.min_feret, &mapping.min_feret, row_number)?,
            feret_mm: number(&record, columns.feret, &mapping.feret, row_number)?,
            feret_angle_degrees: number(&record, columns.angle, &mapping.feret_angle, row_number)?,
        };
        if pore.area_mm2 <= 0.0 {
            return Err(format!("row {row_number}: area must be greater than zero"));
        }
        if pore.perimeter_mm <= 0.0 || pore.min_feret_mm <= 0.0 || pore.feret_mm <= 0.0 {
            return Err(format!("row {row_number}: perimeter and Feret diameters must be greater than zero"));
        }
        if pore.min_feret_mm > pore.feret_mm {
            return Err(format!("row {row_number}: MinFeret cannot exceed Feret"));
        }
        if pore.feret_angle_degrees >= 180.0 {
            return Err(format!("row {row_number}: FeretAngle must be in [0, 180) degrees"));
        }
        pores.push(pore);
    }
    if pores.is_empty() {
        return Err("no valid macropore objects found after the header".to_owned());
    }
    Ok(pores)
}

fn summarize(
    pores: &[Pore],
    input_sha256: String,
    horizon_label: String,
    image_resolution_um_per_pixel: f64,
    cross_section_area_mm2: Option<f64>,
    minimum_object_area_mm2: f64,
    column_mapping: ColumnMapping,
) -> Result<Report, String> {
    if horizon_label.trim().is_empty() {
        return Err("horizon_label must not be blank".to_owned());
    }
    if !image_resolution_um_per_pixel.is_finite() || image_resolution_um_per_pixel <= 0.0 {
        return Err("image resolution must be finite and positive".to_owned());
    }
    if let Some(area) = cross_section_area_mm2 {
        if !area.is_finite() || area <= 0.0 {
            return Err("cross-section area must be finite and positive".to_owned());
        }
    }
    if !minimum_object_area_mm2.is_finite() || minimum_object_area_mm2 < 0.0 {
        return Err("minimum object area must be finite and non-negative".to_owned());
    }

    let input_object_count = pores.len();
    let retained: Vec<_> = pores
        .iter()
        .filter(|pore| pore.area_mm2 >= minimum_object_area_mm2)
        .collect();
    if retained.is_empty() {
        return Err("no macropore objects remain after the minimum-area filter".to_owned());
    }

    let areas: Vec<_> = retained.iter().map(|pore| pore.area_mm2).collect();
    let perimeters: Vec<_> = retained.iter().map(|pore| pore.perimeter_mm).collect();
    let min_ferets: Vec<_> = retained.iter().map(|pore| pore.min_feret_mm).collect();
    let ferets: Vec<_> = retained.iter().map(|pore| pore.feret_mm).collect();
    let angles: Vec<_> = retained.iter().map(|pore| pore.feret_angle_degrees).collect();
    let circularities: Vec<_> = retained
        .iter()
        .map(|pore| 4.0 * std::f64::consts::PI * pore.area_mm2
            / (pore.perimeter_mm * pore.perimeter_mm))
        .collect();
    let aspect_ratios: Vec<_> = retained
        .iter()
        .map(|pore| pore.feret_mm / pore.min_feret_mm)
        .collect();
    let circularity_above_one_count = circularities.iter().filter(|&&value| value > 1.0).count();
    let summed_projected_pore_area_mm2: f64 = areas.iter().sum();

    let mut orientation_histogram = [0usize; ORIENTATION_BIN_COUNT];
    let mut orientation_nematic_order_x = 0.0;
    let mut orientation_nematic_order_y = 0.0;
    for angle in &angles {
        let index = (*angle / ORIENTATION_BIN_DEGREES).floor() as usize;
        orientation_histogram[index.min(ORIENTATION_BIN_COUNT - 1)] += 1;
        let radians = angle.to_radians() * 2.0;
        orientation_nematic_order_x += radians.cos();
        orientation_nematic_order_y += radians.sin();
    }
    orientation_nematic_order_x /= angles.len() as f64;
    orientation_nematic_order_y /= angles.len() as f64;
    let orientation_nematic_strength = orientation_nematic_order_x.hypot(orientation_nematic_order_y);
    let dominant_orientation_degrees = if orientation_nematic_strength > 1.0e-12 {
        let degrees = 0.5 * orientation_nematic_order_y.atan2(orientation_nematic_order_x).to_degrees();
        Some(degrees.rem_euclid(180.0))
    } else {
        None
    };

    let projected_pore_area_fraction = cross_section_area_mm2
        .map(|section_area| summed_projected_pore_area_mm2 / section_area);

    if let Some(fraction) = projected_pore_area_fraction {
        if fraction > 1.0 {
            return Err("summed projected pore area exceeds declared cross-section area; check units, ROI selection and duplicate objects".to_owned());
        }
    }

    Ok(Report {
        tool_version: "0.1.0",
        input_sha256,
        horizon_label,
        image_resolution_um_per_pixel,
        cross_section_area_mm2,
        minimum_object_area_mm2,
        input_object_count,
        excluded_below_minimum_area_count: input_object_count - retained.len(),
        detected_object_count: retained.len(),
        summed_projected_pore_area_mm2,
        projected_pore_area_fraction,
        area_mm2: distribution(&areas)?,
        perimeter_mm: distribution(&perimeters)?,
        min_feret_mm: distribution(&min_ferets)?,
        feret_mm: distribution(&ferets)?,
        feret_angle_degrees: distribution(&angles)?,
        circularity_4pi_area_over_perimeter_squared: distribution(&circularities)?,
        feret_aspect_ratio: distribution(&aspect_ratios)?,
        circularity_above_one_count,
        orientation_histogram_10deg_bins: orientation_histogram,
        orientation_nematic_order_x,
        orientation_nematic_order_y,
        orientation_nematic_strength,
        dominant_orientation_degrees,
        column_mapping,
        dimensionality_boundary: "2D segmented cross-section morphology only; these measurements do not establish 3D pore connectivity, hydraulic conductance or preferential-flow truth",
        warnings: vec![
            "Input rows are already segmented ImageJ objects. This tool does not segment raw images or select a pore-size detection threshold.".to_owned(),
            "Feret geometry and image orientation depend on image calibration, section orientation, thresholding and ROI selection; preserve the raw image and ImageJ settings.".to_owned(),
            "A 2D section cannot establish 3D pore connectivity, tortuosity or vertical continuity. Use orthogonal sections, CT or independent transport measurements for those claims.".to_owned(),
            "Projected pore-area fraction is not total 3D porosity and is meaningful only when the declared cross-section area matches the exact analyzed ROI.".to_owned(),
            "The reported 180-micrometre pixel pitch, if used, must be confirmed for the exact source image and exported object table.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 15 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: morphology-audit <input.csv> <new-report.json> --horizon LABEL --resolution-um VALUE --area-col NAME --perimeter-col NAME --min-feret-col NAME --feret-col NAME --angle-col NAME [--cross-section-area-mm2 VALUE] [--min-area-mm2 VALUE]",
        ).into());
    }
    let input_path = &args[1];
    let output_path = &args[2];
    let mut options = HashMap::new();
    let mut index = 3;
    while index < args.len() {
        if index + 1 >= args.len() {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "every option requires a value").into());
        }
        let key = args[index].clone();
        let value = args[index + 1].clone();
        if options.insert(key.clone(), value).is_some() {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("duplicate option {key}")).into());
        }
        index += 2;
    }

    let required = |key: &str| -> Result<String, io::Error> {
        options
            .get(key)
            .cloned()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, format!("missing required option {key}")))
    };
    let horizon = required("--horizon")?;
    let resolution: f64 = required("--resolution-um")?.parse()?;
    let mapping = ColumnMapping {
        area: required("--area-col")?,
        perimeter: required("--perimeter-col")?,
        min_feret: required("--min-feret-col")?,
        feret: required("--feret-col")?,
        feret_angle: required("--angle-col")?,
    };
    let cross_section_area = options
        .get("--cross-section-area-mm2")
        .map(|value| value.parse::<f64>())
        .transpose()?;
    let minimum_object_area = options
        .get("--min-area-mm2")
        .map(|value| value.parse::<f64>())
        .transpose()?
        .unwrap_or(0.0);

    for key in options.keys() {
        if ![
            "--horizon",
            "--resolution-um",
            "--area-col",
            "--perimeter-col",
            "--min-feret-col",
            "--feret-col",
            "--angle-col",
            "--cross-section-area-mm2",
            "--min-area-mm2",
        ].contains(&key.as_str()) {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("unknown option {key}")).into());
        }
    }

    let bytes = fs::read(input_path)?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let pores = parse_pores(bytes.as_slice(), &mapping).map_err(|error| {
        io::Error::new(io::ErrorKind::InvalidData, error)
    })?;
    let report = summarize(
        &pores,
        input_sha256,
        horizon,
        resolution,
        cross_section_area,
        minimum_object_area,
        mapping,
    ).map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let serialized = serde_json::to_vec_pretty(&report)?;

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(Path::new(output_path))
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report {output_path}: {error}")))?;
    output.write_all(&serialized)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "morphology audit complete: {} 2D pore objects; projected pore area {:.6} mm2; input SHA-256 {}",
        report.detected_object_count,
        report.summed_projected_pore_area_mm2,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("morphology audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn mapping() -> ColumnMapping {
        ColumnMapping {
            area: "Area".to_owned(),
            perimeter: "Perim.".to_owned(),
            min_feret: "MinFeret".to_owned(),
            feret: "Feret".to_owned(),
            feret_angle: "FeretAngle".to_owned(),
        }
    }

    const CSV: &str = "Area,Perim.,MinFeret,Feret,FeretAngle\n2.0,5.0,1.0,2.0,5.0\n4.0,8.0,1.5,3.0,95.0\n";

    #[test]
    fn parses_and_summarizes_imagej_geometry() {
        let pores = parse_pores(Cursor::new(CSV), &mapping()).unwrap();
        assert_eq!(pores.len(), 2);
        assert_eq!(pores[0].area_mm2, 2.0);
        let report = summarize(
            &pores,
            "a".repeat(64),
            "Ap1".to_owned(),
            180.0,
            Some(100.0),
            0.0,
            mapping(),
        ).unwrap();
        assert_eq!(report.detected_object_count, 2);
        assert_eq!(report.summed_projected_pore_area_mm2, 6.0);
        assert_eq!(report.projected_pore_area_fraction, Some(0.06));
        assert_eq!(report.area_mm2.median, 3.0);
        assert_eq!(report.orientation_histogram_10deg_bins[0], 1);
        assert_eq!(report.orientation_histogram_10deg_bins[9], 1);
    }

    #[test]
    fn area_filter_excludes_objects_below_published_threshold_without_losing_them_silently() {
        let pores = parse_pores(Cursor::new(CSV), &mapping()).unwrap();
        let report = summarize(
            &pores,
            "d".repeat(64),
            "Ap1".to_owned(),
            180.0,
            Some(100.0),
            std::f64::consts::PI,
            mapping(),
        ).unwrap();
        assert_eq!(report.input_object_count, 2);
        assert_eq!(report.excluded_below_minimum_area_count, 1);
        assert_eq!(report.detected_object_count, 1);
        assert_eq!(report.summed_projected_pore_area_mm2, 4.0);
    }

    #[test]
    fn computes_shape_descriptors_and_nematic_orientation_without_directing_flow_claims() {
        let pores = parse_pores(Cursor::new(CSV), &mapping()).unwrap();
        let report = summarize(
            &pores,
            "e".repeat(64),
            "Ap1".to_owned(),
            180.0,
            None,
            0.0,
            mapping(),
        ).unwrap();
        assert_eq!(report.feret_aspect_ratio.mean, 2.0);
        assert_eq!(report.orientation_histogram_10deg_bins[0], 1);
        assert_eq!(report.orientation_histogram_10deg_bins[9], 1);
        assert!(report.orientation_nematic_strength < 1.0e-10);
        assert_eq!(report.dominant_orientation_degrees, None);
        assert!(report.dimensionality_boundary.contains("2D"));
    }

    #[test]
    fn rejects_negative_minimum_area_filter() {
        let pores = parse_pores(Cursor::new(CSV), &mapping()).unwrap();
        assert!(summarize(
            &pores, "f".repeat(64), "Ap1".to_owned(), 180.0, None, -1.0, mapping()
        ).unwrap_err().contains("minimum object area"));
    }

    #[test]
    fn fails_when_requested_header_is_missing() {
        let error = parse_pores(
            Cursor::new("Area,Perim.,MinFeret,Feret\n2,5,1,2\n"),
            &mapping(),
        ).unwrap_err();
        assert!(error.contains("FeretAngle"));
    }

    #[test]
    fn rejects_nonfinite_or_negative_geometry() {
        let error = parse_pores(
            Cursor::new("Area,Perim.,MinFeret,Feret,FeretAngle\nNaN,5,1,2,10\n"),
            &mapping(),
        ).unwrap_err();
        assert!(error.contains("finite and non-negative"));
    }

    #[test]
    fn rejects_inconsistent_feret_order() {
        let error = parse_pores(
            Cursor::new("Area,Perim.,MinFeret,Feret,FeretAngle\n2,5,3,2,10\n"),
            &mapping(),
        ).unwrap_err();
        assert!(error.contains("MinFeret cannot exceed Feret"));
    }

    #[test]
    fn rejects_orientation_outside_half_circle() {
        let error = parse_pores(
            Cursor::new("Area,Perim.,MinFeret,Feret,FeretAngle\n2,5,1,2,180\n"),
            &mapping(),
        ).unwrap_err();
        assert!(error.contains("FeretAngle must be in [0, 180)"));
    }

    #[test]
    fn rejects_projected_area_larger_than_declared_section() {
        let pores = parse_pores(Cursor::new(CSV), &mapping()).unwrap();
        let error = summarize(
            &pores,
            "b".repeat(64),
            "Ap1".to_owned(),
            180.0,
            Some(5.0),
            0.0,
            mapping(),
        ).unwrap_err();
        assert!(error.contains("exceeds declared cross-section area"));
    }

    #[test]
    fn calculates_interpolated_quantiles() {
        let summary = distribution(&[1.0, 2.0, 3.0, 4.0]).unwrap();
        assert_eq!(summary.median, 2.5);
        assert!((summary.p90 - 3.7).abs() < 1e-12);
        assert!((summary.p95 - 3.85).abs() < 1e-12);
    }

    #[test]
    fn records_hash_and_does_not_call_2d_area_3d_porosity() {
        let pores = parse_pores(Cursor::new(CSV), &mapping()).unwrap();
        let report = summarize(
            &pores,
            "c".repeat(64),
            "Bt".to_owned(),
            180.0,
            None,
            0.0,
            mapping(),
        ).unwrap();
        assert_eq!(report.input_sha256, "c".repeat(64));
        assert!(report.dimensionality_boundary.contains("2D"));
        assert_eq!(report.projected_pore_area_fraction, None);
    }
}
