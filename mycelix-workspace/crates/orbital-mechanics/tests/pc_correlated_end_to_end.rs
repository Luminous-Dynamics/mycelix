// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! End-to-end differential tests for correlated encounter-plane covariance.
//!
//! The oracle values come from an independent Gauss-Legendre polar integral of
//! a bivariate Gaussian over a circular hard-body region. The fixture's 3-D
//! states and covariances are synthetic and exist only to exercise the public
//! checked path: state validation -> encounter-plane projection -> Pc kernel.

use chrono::{DateTime, Utc};
use nalgebra::{Matrix2, Matrix6};
use orbital_mechanics::conjunction::{ConjunctionAnalyzer, PcMethod};
use orbital_mechanics::covariance::{CovarianceFrame, CovarianceMatrix, CovarianceSource};
use orbital_mechanics::state::{DataSource, OrbitalState, StateVector};
use serde::Deserialize;

const FIXTURE: &str = include_str!("fixtures/pc_correlated_gaussian_disk_v1.json");
const RELATIVE_TOLERANCE: f64 = 0.002;
const REFERENCE_RADIAL_ORDERS: &[usize] = &[60, 100, 160, 220];

#[derive(Debug, Deserialize)]
struct Fixture {
    schema_version: String,
    reference: ReferenceMetadata,
    comparison: ComparisonMetadata,
    cases: Vec<ReferenceCase>,
    analytic_control: ReferenceCase,
}

#[derive(Debug, Deserialize)]
struct ReferenceMetadata {
    integrator: String,
    radial_orders: Vec<usize>,
    angular_nodes_rule: String,
}

#[derive(Debug, Deserialize)]
struct ComparisonMetadata {
    relative_tolerance: f64,
    production_method: String,
}

#[derive(Debug, Deserialize)]
struct ReferenceCase {
    case_id: String,
    miss_encounter_plane_km: [f64; 2],
    covariance_encounter_plane_km2: [[f64; 2]; 2],
    hard_body_radius_km: f64,
    expected_pc_gaussian_small_disk: f64,
    expected_pc_integral_reference: f64,
}

fn covariance_for_projected_case(projected: [[f64; 2]; 2]) -> CovarianceMatrix {
    // The production basis for relative velocity +Z is u1=+Y, u2=-X.
    // Therefore, to produce projected [[a,b],[b,c]], the source-frame XY
    // covariance must be [[c,-b],[-b,a]]. Split that relative covariance
    // evenly between the two independent synthetic object covariances.
    let mut matrix = Matrix6::zeros();
    matrix[(0, 0)] = projected[1][1] / 2.0;
    matrix[(1, 1)] = projected[0][0] / 2.0;
    matrix[(0, 1)] = -projected[0][1] / 2.0;
    matrix[(1, 0)] = -projected[1][0] / 2.0;

    // Positive variances outside the tested encounter plane keep the full
    // 6x6 state covariance positive definite without affecting the 2-D oracle.
    matrix[(2, 2)] = 0.25;
    matrix[(3, 3)] = 1e-6;
    matrix[(4, 4)] = 1e-6;
    matrix[(5, 5)] = 1e-6;

    let mut covariance = CovarianceMatrix::from_matrix(matrix);
    covariance.frame = CovarianceFrame::TEME;

    // The source enum has no Synthetic variant. This marker is only a structural
    // test stand-in required by try_assess; it does not describe real TLE data or
    // claim the fixture covariance is calibrated or orbit-determination-derived.
    covariance.source = CovarianceSource::Estimated { tle_age_hours: 0.0 };
    covariance
}

fn synthetic_state(
    norad_id: u32,
    epoch: DateTime<Utc>,
    miss_encounter_plane_km: [f64; 2],
    primary: bool,
    covariance: CovarianceMatrix,
) -> OrbitalState {
    // With relative velocity along +Z, the checked projection basis is (+Y,-X).
    // Encode the fixture's encounter-plane miss [mx,my] as the 3-D relative
    // position [-my,mx,0], which projects back to [mx,my].
    let (x_offset, y_offset) = if primary {
        (0.0, 0.0)
    } else {
        (-miss_encounter_plane_km[1], miss_encounter_plane_km[0])
    };

    // Both synthetic objects have 7.5 km/s speed; their relative velocity is
    // +7.5 km/s along Z and is orthogonal to the relative position at TCA.
    let transverse_velocity = (7.5_f64.powi(2) - 3.75_f64.powi(2)).sqrt();
    let vz = if primary { -3.75 } else { 3.75 };
    OrbitalState::new(
        norad_id,
        epoch,
        StateVector::new(
            7000.0 + x_offset,
            y_offset,
            0.0,
            0.0,
            transverse_velocity,
            vz,
        ),
        DataSource::OperatorEphemeris,
    )
    .with_covariance(covariance)
}

#[test]
fn checked_assessment_matches_correlated_gaussian_disk_reference() {
    let fixture: Fixture =
        serde_json::from_str(FIXTURE).expect("correlated Pc reference fixture must be valid JSON");

    assert_eq!(fixture.schema_version, "1.0.0");
    assert_eq!(
        fixture.reference.integrator,
        "tensor-product Gauss-Legendre quadrature in polar coordinates"
    );
    assert_eq!(
        fixture.reference.radial_orders.as_slice(),
        REFERENCE_RADIAL_ORDERS
    );
    assert_eq!(
        fixture.reference.angular_nodes_rule,
        "angular_nodes = 2 * radial_order + 24"
    );
    assert_eq!(fixture.comparison.relative_tolerance, RELATIVE_TOLERANCE);
    assert_eq!(
        fixture.comparison.production_method,
        "GaussianSmallDiskApprox2D"
    );
    assert_eq!(fixture.cases.len(), 4);
    assert_eq!(
        fixture.analytic_control.case_id,
        "centered-isotropic-closed-form"
    );
    assert_eq!(fixture.analytic_control.miss_encounter_plane_km, [0.0, 0.0]);
    assert_eq!(
        fixture.analytic_control.covariance_encounter_plane_km2,
        [[0.04, 0.0], [0.0, 0.04]]
    );
    let analytic_radius = fixture.analytic_control.hard_body_radius_km;
    let analytic_sigma_km = 0.2_f64;
    let analytic_closed_form_pc =
        1.0 - (-analytic_radius.powi(2) / (2.0 * analytic_sigma_km.powi(2))).exp();
    let analytic_reference_error = (fixture.analytic_control.expected_pc_integral_reference
        - analytic_closed_form_pc)
        .abs()
        / analytic_closed_form_pc;
    assert!(
        analytic_reference_error <= 1e-12,
        "centered-isotropic reference must match 1-exp(-R^2/(2*sigma^2)); relative error={analytic_reference_error:.3e}"
    );
    assert_eq!(
        fixture.analytic_control.expected_pc_gaussian_small_disk,
        0.00125
    );

    let epoch = DateTime::parse_from_rfc3339("2026-01-01T00:00:00Z")
        .expect("fixed test epoch")
        .with_timezone(&Utc);

    for case in fixture.cases.iter().chain(std::iter::once(&fixture.analytic_control)) {
        assert!(!case.case_id.trim().is_empty());
        assert!(
            case.miss_encounter_plane_km.iter().all(|v| v.is_finite()),
            "{}: miss vector must be finite",
            case.case_id
        );
        assert!(case.hard_body_radius_km.is_finite() && case.hard_body_radius_km > 0.0);
        assert!(
            case.expected_pc_integral_reference.is_finite()
                && case.expected_pc_integral_reference > 0.0
                && case.expected_pc_integral_reference <= 1.0,
            "{}: integral reference must be a finite probability",
            case.case_id
        );
        assert!(
            case.expected_pc_gaussian_small_disk.is_finite()
                && case.expected_pc_gaussian_small_disk > 0.0
                && case.expected_pc_gaussian_small_disk <= 1.0,
            "{}: small-disk reference must be a finite probability",
            case.case_id
        );

        let projected = Matrix2::new(
            case.covariance_encounter_plane_km2[0][0],
            case.covariance_encounter_plane_km2[0][1],
            case.covariance_encounter_plane_km2[1][0],
            case.covariance_encounter_plane_km2[1][1],
        );
        assert!(
            projected.iter().all(|v| v.is_finite()) && projected.cholesky().is_some(),
            "{}: correlated encounter covariance must be finite and positive definite",
            case.case_id
        );
        assert_eq!(
            case.covariance_encounter_plane_km2[0][1],
            case.covariance_encounter_plane_km2[1][0],
            "{}: encounter covariance must be symmetric",
            case.case_id
        );
        if case.case_id == fixture.analytic_control.case_id {
            assert_eq!(
                case.covariance_encounter_plane_km2[0][1],
                0.0,
                "analytic control intentionally uses an isotropic diagonal covariance"
            );
        } else {
            assert_ne!(
                case.covariance_encounter_plane_km2[0][1],
                0.0,
                "{}: correlated fixtures must keep a non-zero covariance cross term",
                case.case_id
            );
        }

        let covariance_primary = covariance_for_projected_case(case.covariance_encounter_plane_km2);
        let covariance_secondary =
            covariance_for_projected_case(case.covariance_encounter_plane_km2);
        let primary = synthetic_state(
            25544,
            epoch.clone(),
            case.miss_encounter_plane_km,
            true,
            covariance_primary,
        );
        let secondary = synthetic_state(
            12345,
            epoch.clone(),
            case.miss_encounter_plane_km,
            false,
            covariance_secondary,
        );

        let result = ConjunctionAnalyzer::new()
            .with_hbr(case.hard_body_radius_km * 1000.0)
            .try_assess(&primary, &secondary)
            .unwrap_or_else(|error| {
                panic!(
                    "{}: checked assessment rejected fixture: {error}",
                    case.case_id
                )
            });

        assert_eq!(
            result.collision_probability.method,
            PcMethod::GaussianSmallDiskApprox2D,
            "{}: production must retain its truthful method identity",
            case.case_id
        );
        assert!(
            result.collision_probability.has_covariance,
            "{}: result must report covariance use",
            case.case_id
        );

        let actual_pc = result.collision_probability.pc;
        let implementation_error = (actual_pc - case.expected_pc_gaussian_small_disk).abs()
            / case.expected_pc_gaussian_small_disk;
        assert!(
            implementation_error <= 1e-9,
            "{}: end-to-end path differs from its pinned local-density value; actual={actual_pc:.16e}, expected={:.16e}, relative error={implementation_error:.4e}",
            case.case_id,
            case.expected_pc_gaussian_small_disk
        );

        let integral_relative_error = (actual_pc - case.expected_pc_integral_reference).abs()
            / case.expected_pc_integral_reference;
        assert!(
            integral_relative_error <= fixture.comparison.relative_tolerance,
            "{}: production Pc differs from independent Gaussian-disk integral by {:.5}%, tolerance is {:.3}%; actual={:.16e}, reference={:.16e}",
            case.case_id,
            integral_relative_error * 100.0,
            fixture.comparison.relative_tolerance * 100.0,
            actual_pc,
            case.expected_pc_integral_reference
        );
    }
}
