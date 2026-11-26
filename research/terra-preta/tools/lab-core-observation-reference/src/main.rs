use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "terra-lab-core-observation-reference-v0.1";
const PI: f64 = std::f64::consts::PI;

#[derive(Clone, Debug, Deserialize, Serialize)]
struct SpecimenGeometry {
    diameter_m: f64,
    length_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "protocol", rename_all = "snake_case")]
enum LabObservation {
    ConstantHead {
        record_id: String,
        physical_sample_group_id: String,
        original_method_label: String,
        specimen_preparation: String,
        specimen: SpecimenGeometry,
        delta_head_m: f64,
        flow_rate_m3_s: f64,
    },
    FallingHead {
        record_id: String,
        physical_sample_group_id: String,
        original_method_label: String,
        specimen_preparation: String,
        specimen: SpecimenGeometry,
        standpipe_diameter_m: f64,
        elapsed_time_s: f64,
        initial_head_m: f64,
        final_head_m: f64,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    source_description: String,
    observations: Vec<LabObservation>,
}

#[derive(Clone, Debug, Serialize)]
struct ObservationSensitivity {
    parameter_id: String,
    observed_quantity_id: String,
    observed_value: f64,
    derivative_per_unit_log_parameter: f64,
    derivative_unit: String,
    log_elasticity: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ObservationReceipt {
    record_id: String,
    physical_sample_group_id: String,
    protocol: String,
    original_method_label: String,
    specimen_preparation: String,
    support_domain: String,
    specimen_diameter_m: f64,
    specimen_length_m: f64,
    specimen_area_m2: f64,
    estimated_k_m_s: f64,
    forward_reconstruction_error: f64,
    forward_reconstruction_relative_error: f64,
    local_sensitivity: ObservationSensitivity,
    observation_operator: String,
    assumptions: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    input_sha256: String,
    observation_count: usize,
    observations: Vec<ObservationReceipt>,
    interpretation_boundary: String,
}

fn require_positive(value: f64, name: &str) -> Result<(), String> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be finite and positive"))
    }
}

fn specimen_area(geometry: &SpecimenGeometry) -> Result<f64, String> {
    require_positive(geometry.diameter_m, "specimen diameter")?;
    require_positive(geometry.length_m, "specimen length")?;
    let area = PI * geometry.diameter_m.powi(2) / 4.0;
    require_positive(area, "specimen cross-sectional area")?;
    Ok(area)
}

fn constant_head_k_from_measurement(
    geometry: &SpecimenGeometry,
    delta_head_m: f64,
    flow_rate_m3_s: f64,
) -> Result<f64, String> {
    let area = specimen_area(geometry)?;
    require_positive(delta_head_m, "constant-head difference")?;
    require_positive(flow_rate_m3_s, "constant-head flow rate")?;
    let k = flow_rate_m3_s * geometry.length_m / (area * delta_head_m);
    require_positive(k, "constant-head hydraulic conductivity")?;
    Ok(k)
}

fn predict_constant_head_flow(
    hydraulic_conductivity_m_s: f64,
    geometry: &SpecimenGeometry,
    delta_head_m: f64,
) -> Result<f64, String> {
    require_positive(hydraulic_conductivity_m_s, "hydraulic conductivity")?;
    let area = specimen_area(geometry)?;
    require_positive(delta_head_m, "constant-head difference")?;
    let flow = hydraulic_conductivity_m_s * area * delta_head_m / geometry.length_m;
    require_positive(flow, "predicted constant-head flow")?;
    Ok(flow)
}

fn falling_head_k_from_measurement(
    geometry: &SpecimenGeometry,
    standpipe_diameter_m: f64,
    elapsed_time_s: f64,
    initial_head_m: f64,
    final_head_m: f64,
) -> Result<f64, String> {
    let specimen_area = specimen_area(geometry)?;
    require_positive(standpipe_diameter_m, "standpipe diameter")?;
    require_positive(elapsed_time_s, "falling-head elapsed time")?;
    require_positive(initial_head_m, "initial falling head")?;
    require_positive(final_head_m, "final falling head")?;
    if initial_head_m <= final_head_m {
        return Err("falling-head initial head must exceed final head".into());
    }

    let standpipe_area = PI * standpipe_diameter_m.powi(2) / 4.0;
    require_positive(standpipe_area, "standpipe area")?;
    let log_ratio = ((initial_head_m - final_head_m) / final_head_m).ln_1p();
    require_positive(log_ratio, "falling-head logarithmic head ratio")?;
    let k = standpipe_area * geometry.length_m * log_ratio / (specimen_area * elapsed_time_s);
    require_positive(k, "falling-head hydraulic conductivity")?;
    Ok(k)
}

fn predict_falling_head_final(
    hydraulic_conductivity_m_s: f64,
    geometry: &SpecimenGeometry,
    standpipe_diameter_m: f64,
    elapsed_time_s: f64,
    initial_head_m: f64,
) -> Result<f64, String> {
    require_positive(hydraulic_conductivity_m_s, "hydraulic conductivity")?;
    let specimen_area = specimen_area(geometry)?;
    require_positive(standpipe_diameter_m, "standpipe diameter")?;
    require_positive(elapsed_time_s, "falling-head elapsed time")?;
    require_positive(initial_head_m, "initial falling head")?;
    let standpipe_area = PI * standpipe_diameter_m.powi(2) / 4.0;
    require_positive(standpipe_area, "standpipe area")?;

    let exponent = -hydraulic_conductivity_m_s * specimen_area * elapsed_time_s
        / (standpipe_area * geometry.length_m);
    let final_head = initial_head_m * exponent.exp();
    require_positive(final_head, "predicted final falling head")?;
    Ok(final_head)
}

fn analyse_observation(observation: &LabObservation) -> Result<ObservationReceipt, String> {
    match observation {
        LabObservation::ConstantHead {
            record_id,
            physical_sample_group_id,
            original_method_label,
            specimen_preparation,
            specimen,
            delta_head_m,
            flow_rate_m3_s,
        } => {
            if record_id.trim().is_empty()
                || physical_sample_group_id.trim().is_empty()
                || original_method_label.trim().is_empty()
                || specimen_preparation.trim().is_empty()
            {
                return Err("constant-head observation requires record, sample, method and preparation metadata".into());
            }
            let area = specimen_area(specimen)?;
            let k = constant_head_k_from_measurement(specimen, *delta_head_m, *flow_rate_m3_s)?;
            let predicted_flow = predict_constant_head_flow(k, specimen, *delta_head_m)?;
            let absolute_error = (predicted_flow - flow_rate_m3_s).abs();
            let relative_error = absolute_error / flow_rate_m3_s.abs().max(f64::MIN_POSITIVE);
            if !relative_error.is_finite() || relative_error > 1.0e-10 {
                return Err("constant-head forward/inverse reconstruction did not close".into());
            }
            Ok(ObservationReceipt {
                record_id: record_id.clone(),
                physical_sample_group_id: physical_sample_group_id.clone(),
                protocol: "constant_head".into(),
                original_method_label: original_method_label.clone(),
                specimen_preparation: specimen_preparation.clone(),
                support_domain: "saturated_laboratory_core".into(),
                specimen_diameter_m: specimen.diameter_m,
                specimen_length_m: specimen.length_m,
                specimen_area_m2: area,
                estimated_k_m_s: k,
                forward_reconstruction_error: absolute_error,
                forward_reconstruction_relative_error: relative_error,
                local_sensitivity: ObservationSensitivity {
                    parameter_id: "log_hydraulic_conductivity".into(),
                    observed_quantity_id: "flow_rate_m3_s".into(),
                    observed_value: predicted_flow,
                    // Q(K) is linear in K, so dQ / d(log K) = Q.
                    derivative_per_unit_log_parameter: predicted_flow,
                    derivative_unit: "m^3 s^-1 per unit log(K [m s^-1])".into(),
                    log_elasticity: 1.0,
                },
                observation_operator: "Q = K * A * delta_h / L; K = Q * L / (A * delta_h)".into(),
                assumptions: vec![
                    "steady saturated one-dimensional Darcy flow".into(),
                    "homogeneous specimen with known cross-sectional area and length".into(),
                    "negligible sidewall leakage, entrance losses and apparatus storage".into(),
                    "constant applied hydraulic head difference during the measured interval"
                        .into(),
                ],
            })
        }
        LabObservation::FallingHead {
            record_id,
            physical_sample_group_id,
            original_method_label,
            specimen_preparation,
            specimen,
            standpipe_diameter_m,
            elapsed_time_s,
            initial_head_m,
            final_head_m,
        } => {
            if record_id.trim().is_empty()
                || physical_sample_group_id.trim().is_empty()
                || original_method_label.trim().is_empty()
                || specimen_preparation.trim().is_empty()
            {
                return Err("falling-head observation requires record, sample, method and preparation metadata".into());
            }
            let area = specimen_area(specimen)?;
            let k = falling_head_k_from_measurement(
                specimen,
                *standpipe_diameter_m,
                *elapsed_time_s,
                *initial_head_m,
                *final_head_m,
            )?;
            let predicted_final = predict_falling_head_final(
                k,
                specimen,
                *standpipe_diameter_m,
                *elapsed_time_s,
                *initial_head_m,
            )?;
            let absolute_error = (predicted_final - final_head_m).abs();
            let relative_error = absolute_error / final_head_m.abs().max(f64::MIN_POSITIVE);
            if !relative_error.is_finite() || relative_error > 1.0e-10 {
                return Err("falling-head forward/inverse reconstruction did not close".into());
            }
            Ok(ObservationReceipt {
                record_id: record_id.clone(),
                physical_sample_group_id: physical_sample_group_id.clone(),
                protocol: "falling_head".into(),
                original_method_label: original_method_label.clone(),
                specimen_preparation: specimen_preparation.clone(),
                support_domain: "saturated_laboratory_core".into(),
                specimen_diameter_m: specimen.diameter_m,
                specimen_length_m: specimen.length_m,
                specimen_area_m2: area,
                estimated_k_m_s: k,
                forward_reconstruction_error: absolute_error,
                forward_reconstruction_relative_error: relative_error,
                local_sensitivity: {
                    let standpipe_area = PI * standpipe_diameter_m.powi(2) / 4.0;
                    let exponent = -k * area * elapsed_time_s
                        / (standpipe_area * specimen.length_m);
                    ObservationSensitivity {
                        parameter_id: "log_hydraulic_conductivity".into(),
                        observed_quantity_id: "final_head_m".into(),
                        observed_value: predicted_final,
                        // h1 = h0 * exp(-K*A*t/(a*L)); d(h1)/d(log K) = h1 * exponent.
                        derivative_per_unit_log_parameter: predicted_final * exponent,
                        derivative_unit: "m per unit log(K [m s^-1])".into(),
                        log_elasticity: exponent,
                    }
                },
                observation_operator: "K = (a * L / (A * t)) * ln(h0 / h1); h1 = h0 * exp(-K * A * t / (a * L))".into(),
                assumptions: vec![
                    "saturated one-dimensional Darcy flow".into(),
                    "constant specimen and standpipe cross-sectional areas".into(),
                    "negligible sidewall leakage, entrance losses and tubing storage".into(),
                    "standpipe and specimen are hydraulically connected with the same evolving head difference".into(),
                ],
            })
        }
    }
}

fn analyse(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != "synthetic_only_not_empirical_evidence"
        || fixture.source_description.trim().is_empty()
    {
        return Err(
            "fixture version, synthetic-only status or source description is invalid".into(),
        );
    }
    if fixture.observations.is_empty() {
        return Err("fixture must contain at least one laboratory observation".into());
    }

    let mut record_ids = std::collections::BTreeSet::new();
    let mut receipts = Vec::with_capacity(fixture.observations.len());
    for observation in &fixture.observations {
        let record_id = match observation {
            LabObservation::ConstantHead { record_id, .. } => record_id,
            LabObservation::FallingHead { record_id, .. } => record_id,
        };
        if !record_ids.insert(record_id.as_str()) {
            return Err(format!("duplicate observation record ID {record_id}"));
        }
        receipts.push(analyse_observation(observation)?);
    }

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        input_sha256,
        observation_count: receipts.len(),
        observations: receipts,
        interpretation_boundary: "Synthetic reference only. Constant-head and falling-head operators assume saturated one-dimensional Darcy flow with idealized apparatus boundaries. They are not applicable to tension infiltrometers, Guelph/Aardvark permeameters, unsaturated transient redistribution, anisotropic flow, sidewall leakage, or heterogeneous cores without separate verified observation operators. Derived K is a protocol-conditioned estimate, not a universal raw label.".into(),
    })
}

fn run() -> Result<(), String> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(format!(
            "usage: {} INPUT.json OUTPUT.json",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-lab-core-observation-reference")
        ));
    }
    let bytes = fs::read(&arguments[1]).map_err(|error| format!("cannot read input: {error}"))?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let fixture: Fixture =
        serde_json::from_slice(&bytes).map_err(|error| format!("invalid fixture: {error}"))?;
    let report = analyse(&fixture, digest)?;
    let encoded = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot encode output: {error}"))?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&arguments[2])
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    output
        .write_all(&encoded)
        .map_err(|error| format!("cannot write output: {error}"))?;
    output
        .write_all(b"\n")
        .map_err(|error| format!("cannot finish output: {error}"))?;
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geometry() -> SpecimenGeometry {
        SpecimenGeometry {
            diameter_m: 0.05,
            length_m: 0.10,
        }
    }

    #[test]
    fn constant_head_inverse_matches_forward_darcy_operator() {
        let specimen = geometry();
        let expected_k = 1.0e-4;
        let delta_head = 0.5;
        let flow = predict_constant_head_flow(expected_k, &specimen, delta_head).unwrap();
        let recovered = constant_head_k_from_measurement(&specimen, delta_head, flow).unwrap();
        assert!((recovered / expected_k - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn falling_head_inverse_matches_forward_exponential_operator() {
        let specimen = geometry();
        let standpipe_diameter = 0.005;
        let elapsed = 600.0;
        let initial = 1.0;
        for expected_k in [1.0e-8, 1.0e-6, 1.0e-4] {
            let final_head = predict_falling_head_final(
                expected_k,
                &specimen,
                standpipe_diameter,
                elapsed,
                initial,
            )
            .unwrap();
            let recovered = falling_head_k_from_measurement(
                &specimen,
                standpipe_diameter,
                elapsed,
                initial,
                final_head,
            )
            .unwrap();
            assert!((recovered / expected_k - 1.0).abs() < 1.0e-11);
        }
    }

    #[test]
    fn falling_head_log_ratio_is_stable_for_near_equal_heads() {
        let specimen = geometry();
        let h0 = 1.0;
        let h1 = 1.0 - 1.0e-10;
        let k = falling_head_k_from_measurement(&specimen, 0.005, 100.0, h0, h1).unwrap();
        assert!(k.is_finite() && k > 0.0);
    }

    #[test]
    fn specimen_geometry_changes_observation_for_same_latent_conductivity() {
        let small = SpecimenGeometry {
            diameter_m: 0.025,
            length_m: 0.10,
        };
        let large = SpecimenGeometry {
            diameter_m: 0.05,
            length_m: 0.10,
        };
        let small_q = predict_constant_head_flow(1.0e-5, &small, 0.5).unwrap();
        let large_q = predict_constant_head_flow(1.0e-5, &large, 0.5).unwrap();
        assert!((large_q / small_q - 4.0).abs() < 1.0e-12);
    }

    #[test]
    fn equivalent_si_inputs_produce_identical_k() {
        let specimen_m = geometry();
        let flow_m3_s = 1.0e-6;
        let dh_m = 0.5;
        let k_m = constant_head_k_from_measurement(&specimen_m, dh_m, flow_m3_s).unwrap();

        let specimen_mm = SpecimenGeometry {
            diameter_m: 50.0e-3,
            length_m: 100.0e-3,
        };
        let k_mm_inputs =
            constant_head_k_from_measurement(&specimen_mm, 500.0e-3, flow_m3_s).unwrap();
        assert!((k_m / k_mm_inputs - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn method_label_and_specimen_preparation_are_required() {
        let mut observation = LabObservation::ConstantHead {
            record_id: "core-2".into(),
            physical_sample_group_id: "sample-2".into(),
            original_method_label: "constant head permeameter".into(),
            specimen_preparation: "repacked".into(),
            specimen: geometry(),
            delta_head_m: 0.5,
            flow_rate_m3_s: 1.0e-6,
        };
        assert!(analyse_observation(&observation).is_ok());
        if let LabObservation::ConstantHead {
            original_method_label,
            ..
        } = &mut observation
        {
            original_method_label.clear();
        }
        assert!(analyse_observation(&observation).is_err());
    }

    #[test]
    fn falling_head_receipt_keeps_source_method_and_preparation() {
        let observation = LabObservation::FallingHead {
            record_id: "core-3".into(),
            physical_sample_group_id: "sample-3".into(),
            original_method_label: "falling head permeameter".into(),
            specimen_preparation: "repacked".into(),
            specimen: geometry(),
            standpipe_diameter_m: 0.005,
            elapsed_time_s: 600.0,
            initial_head_m: 1.0,
            final_head_m: 0.5,
        };
        let receipt = analyse_observation(&observation).unwrap();
        assert_eq!(receipt.original_method_label, "falling head permeameter");
        assert_eq!(receipt.specimen_preparation, "repacked");
    }

    #[test]
    fn zero_and_negative_geometry_or_head_fail_closed() {
        let invalid = SpecimenGeometry {
            diameter_m: 0.0,
            length_m: 0.1,
        };
        assert!(specimen_area(&invalid).is_err());
        assert!(predict_constant_head_flow(1.0e-5, &geometry(), 0.0).is_err());
        assert!(falling_head_k_from_measurement(&geometry(), 0.005, 10.0, 0.5, 1.0).is_err());
    }

    #[test]
    fn nonfinite_measurements_fail_closed() {
        assert!(constant_head_k_from_measurement(&geometry(), 0.5, f64::NAN).is_err());
        assert!(
            falling_head_k_from_measurement(&geometry(), 0.005, f64::INFINITY, 1.0, 0.5).is_err()
        );
    }

    #[test]
    fn registry_receipt_keeps_protocol_and_support_explicit() {
        let observation = LabObservation::ConstantHead {
            record_id: "core-1".into(),
            physical_sample_group_id: "sample-1".into(),
            original_method_label: "constant head permeameter".into(),
            specimen_preparation: "undisturbed".into(),
            specimen: geometry(),
            delta_head_m: 0.5,
            flow_rate_m3_s: 1.0e-6,
        };
        let receipt = analyse_observation(&observation).unwrap();
        assert_eq!(receipt.protocol, "constant_head");
        assert_eq!(receipt.support_domain, "saturated_laboratory_core");
        assert_eq!(receipt.original_method_label, "constant head permeameter");
        assert_eq!(receipt.specimen_preparation, "undisturbed");
        assert!(receipt.observation_operator.contains("Q = K * A"));
        assert!(
            receipt
                .assumptions
                .iter()
                .any(|item| item.contains("one-dimensional"))
        );
    }
    #[test]
    fn constant_head_observation_jacobian_matches_log_parameter_finite_difference() {
        let specimen = geometry();
        let k = 1.0e-5;
        let delta_head = 0.4;
        let flow = predict_constant_head_flow(k, &specimen, delta_head).unwrap();
        let epsilon: f64 = 1.0e-6;
        let plus = predict_constant_head_flow(k * epsilon.exp(), &specimen, delta_head).unwrap();
        let minus =
            predict_constant_head_flow(k * (-epsilon).exp(), &specimen, delta_head).unwrap();
        let finite_difference = (plus - minus) / (2.0 * epsilon);
        assert!((finite_difference / flow - 1.0).abs() < 1.0e-8);

        let observation = LabObservation::ConstantHead {
            record_id: "jacobian-constant".into(),
            physical_sample_group_id: "sample-jacobian".into(),
            original_method_label: "constant head".into(),
            specimen_preparation: "undisturbed".into(),
            specimen: specimen.clone(),
            delta_head_m: delta_head,
            flow_rate_m3_s: flow,
        };
        let receipt = analyse_observation(&observation).unwrap();
        assert_eq!(
            receipt.local_sensitivity.parameter_id,
            "log_hydraulic_conductivity"
        );
        assert_eq!(
            receipt.local_sensitivity.observed_quantity_id,
            "flow_rate_m3_s"
        );
        assert!(
            (receipt.local_sensitivity.derivative_per_unit_log_parameter / flow - 1.0).abs()
                < 1.0e-12
        );
        assert!((receipt.local_sensitivity.log_elasticity - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn falling_head_observation_jacobian_matches_log_parameter_finite_difference() {
        let specimen = geometry();
        let standpipe_diameter = 0.005;
        let elapsed = 600.0;
        let initial = 1.0;
        for k in [1.0e-8, 1.0e-6, 1.0e-4] {
            let final_head =
                predict_falling_head_final(k, &specimen, standpipe_diameter, elapsed, initial)
                    .unwrap();
            let standpipe_area = PI * standpipe_diameter.powi(2) / 4.0;
            let exponent = -k * specimen_area(&specimen).unwrap() * elapsed
                / (standpipe_area * specimen.length_m);
            let analytic = final_head * exponent;
            let epsilon: f64 = 1.0e-6;
            let plus = predict_falling_head_final(
                k * epsilon.exp(),
                &specimen,
                standpipe_diameter,
                elapsed,
                initial,
            )
            .unwrap();
            let minus = predict_falling_head_final(
                k * (-epsilon).exp(),
                &specimen,
                standpipe_diameter,
                elapsed,
                initial,
            )
            .unwrap();
            let finite_difference = (plus - minus) / (2.0 * epsilon);
            assert!((finite_difference / analytic - 1.0).abs() < 1.0e-7);

            let observation = LabObservation::FallingHead {
                record_id: format!("jacobian-falling-{k:e}"),
                physical_sample_group_id: "sample-jacobian".into(),
                original_method_label: "falling head".into(),
                specimen_preparation: "undisturbed".into(),
                specimen: specimen.clone(),
                standpipe_diameter_m: standpipe_diameter,
                elapsed_time_s: elapsed,
                initial_head_m: initial,
                final_head_m: final_head,
            };
            let receipt = analyse_observation(&observation).unwrap();
            assert_eq!(
                receipt.local_sensitivity.observed_quantity_id,
                "final_head_m"
            );
            assert!(
                (receipt.local_sensitivity.derivative_per_unit_log_parameter / analytic - 1.0)
                    .abs()
                    < 1.0e-12
            );
            assert!((receipt.local_sensitivity.log_elasticity - exponent).abs() < 1.0e-12);
        }
    }
}
