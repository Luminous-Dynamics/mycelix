//! Structural integrity checks for the pinned NASA CARA Foster reference fixture.
//!
//! These checks protect the fixture and its provenance. They do not execute
//! NASA's MATLAB implementation and do not establish Mycelix algorithm parity.

use serde_json::Value;
use std::collections::HashSet;

const FIXTURE: &str = include_str!("fixtures/cara_pc2d_foster_reference_v1.json");

fn finite_vector(value: &Value, expected_len: usize) -> bool {
    value.as_array().is_some_and(|items| {
        items.len() == expected_len
            && items
                .iter()
                .all(|item| item.as_f64().is_some_and(f64::is_finite))
    })
}

fn finite_square_matrix(value: &Value, dimension: usize) -> bool {
    let Some(rows) = value.as_array() else {
        return false;
    };
    if rows.len() != dimension
        || !rows.iter().all(|row| finite_vector(row, dimension))
    {
        return false;
    }

    // Check relative symmetry without letting large entries hide defects in a
    // small pair. The fixture's upstream covariance includes roundoff-level
    // asymmetry (about 1e-12), so this is a data-integrity check, not repair.
    for i in 0..dimension {
        for j in (i + 1)..dimension {
            let upper = rows[i][j].as_f64().expect("shape was checked");
            let lower = rows[j][i].as_f64().expect("shape was checked");
            let scale = upper.abs().max(lower.abs());
            if (upper - lower).abs() > 1e-10 * scale {
                return false;
            }
        }
    }
    true
}

#[test]
fn cara_fixture_source_and_comparison_status_are_explicit() {
    let fixture: Value =
        serde_json::from_str(FIXTURE).expect("CARA reference fixture must remain valid JSON");

    assert_eq!(fixture["schema_version"], "1.0.0");
    assert_eq!(fixture["record_type"], "orbital-pc-reference-fixtures");
    assert_eq!(
        fixture["status"],
        "source-pinned-upstream-regression-expectations"
    );
    assert_eq!(
        fixture["verification_boundary"]["mycelix_comparison_status"],
        "implemented_not_yet_ci_verified",
        "fixture metadata must not overstate exact-head CI verification"
    );

    assert_eq!(
        fixture["source"]["repository"],
        "https://github.com/nasa/CARA_Analysis_Tools"
    );
    assert_eq!(
        fixture["source"]["commit"],
        "ce0f00722c56808755ee0adbe47a4e3d56439484"
    );
    assert_eq!(
        fixture["source"]["algorithm"]["blob"],
        "84dbc44bf6276c1cf66343a680ac7e3b87a5f528"
    );
    assert_eq!(
        fixture["source"]["upstream_test"]["blob"],
        "dea6063ce5abdd7ad112f1ed55a15b4a24acc81b"
    );
}

#[test]
fn cara_fixture_inputs_and_expected_results_are_well_formed() {
    let fixture: Value = serde_json::from_str(FIXTURE).expect("fixture JSON");
    let input = &fixture["encounter_input"];

    for key in ["r1_km", "r2_km", "v1_km_per_s", "v2_km_per_s"] {
        assert!(finite_vector(&input[key], 3), "{key} must be a finite 3-vector");
    }
    for key in ["cov1_km2", "cov2_km2"] {
        assert!(
            finite_square_matrix(&input[key], 3),
            "{key} must be a finite, symmetric 3x3 matrix"
        );
    }

    let hbr = input["hbr_km"].as_f64().expect("hbr_km must be numeric");
    assert!(hbr.is_finite() && hbr > 0.0);
    assert_eq!(fixture["conventions"]["epoch"], Value::Null, "do not invent an epoch");

    let cases = fixture["cases"].as_array().expect("cases must be an array");
    assert_eq!(cases.len(), 3);

    let expected = [
        ("circle", 0.0000270601573490125),
        ("square", 0.0000344534649703584),
        ("squareEquArea", 0.0000270601573490111),
    ];
    let mut ids = HashSet::new();

    for (case, (region, expected_pc)) in cases.iter().zip(expected) {
        let id = case["case_id"].as_str().expect("case_id must be a string");
        assert!(ids.insert(id), "case IDs must be unique");
        assert_eq!(case["hard_body_region"], region);
        assert_eq!(case["expected_source"], "upstream_test_assertion");
        assert_eq!(case["upstream_relative_tolerance"], 0.001);

        let actual_pc = case["expected_pc"]
            .as_f64()
            .expect("expected_pc must be numeric");
        assert!(actual_pc.is_finite() && (0.0..=1.0).contains(&actual_pc));
        assert!(
            (actual_pc - expected_pc).abs() <= expected_pc.abs() * 1e-14,
            "expected_pc must remain the pinned upstream assertion for {region}"
        );
    }
}
type Vec3 = [f64; 3];
type Mat3 = [[f64; 3]; 3];

fn parse_vec3(value: &Value) -> Vec3 {
    let values = value.as_array().expect("vector must be array");
    assert_eq!(values.len(), 3);
    [
        values[0].as_f64().expect("numeric vector entry"),
        values[1].as_f64().expect("numeric vector entry"),
        values[2].as_f64().expect("numeric vector entry"),
    ]
}

fn parse_mat3(value: &Value) -> Mat3 {
    let rows = value.as_array().expect("matrix must be array");
    assert_eq!(rows.len(), 3);
    [
        parse_vec3(&rows[0]),
        parse_vec3(&rows[1]),
        parse_vec3(&rows[2]),
    ]
}

fn sub3(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot3(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm3(a: Vec3) -> f64 {
    dot3(a, a).sqrt()
}

fn unit3(a: Vec3) -> Vec3 {
    let norm = norm3(a);
    assert!(norm.is_finite() && norm > 0.0, "degenerate encounter basis");
    [a[0] / norm, a[1] / norm, a[2] / norm]
}

fn combine_covariances(a: Mat3, b: Mat3) -> Mat3 {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
    result
}

fn project_covariance(row_a: Vec3, covariance: Mat3, row_b: Vec3) -> f64 {
    let mut result = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            result += row_a[i] * covariance[i][j] * row_b[j];
        }
    }
    result
}

// Composite Simpson quadrature used as an independent executable check of
// the pinned Foster integral. It deliberately does not call NASA MATLAB code.
fn simpson<F>(f: F, a: f64, b: f64, intervals: usize) -> f64
where
    F: Fn(f64) -> f64,
{
    assert!(intervals > 0 && intervals % 2 == 0);
    let step = (b - a) / intervals as f64;
    let mut total = f(a) + f(b);
    for index in 1..intervals {
        let weight = if index % 2 == 0 { 2.0 } else { 4.0 };
        total += weight * f(a + step * index as f64);
    }
    total * step / 3.0
}

fn foster_density(x: f64, z: f64, covariance: [f64; 4]) -> f64 {
    let [c00, c01, c10, c11] = covariance;
    let determinant = c00 * c11 - c01 * c10;
    assert!(
        determinant.is_finite() && determinant > 0.0,
        "projected covariance must be positive definite"
    );
    let inv00 = c11 / determinant;
    let inv01 = -c01 / determinant;
    let inv10 = -c10 / determinant;
    let inv11 = c00 / determinant;
    let exponent = -0.5
        * (inv00 * x * x + (inv01 + inv10) * z * x + inv11 * z * z);
    exponent.exp() / (2.0 * std::f64::consts::PI * determinant.sqrt())
}

fn integrate_foster_case(
    region: &str,
    x0: f64,
    hbr: f64,
    covariance: [f64; 4],
) -> f64 {
    const OUTER_INTERVALS: usize = 1024;
    const INNER_INTERVALS: usize = 256;

    match region {
        "circle" => simpson(
            |x| {
                let dx = x - x0;
                let radicand = (hbr * hbr - dx * dx).max(0.0);
                let z_max = radicand.sqrt();
                simpson(
                    |z| foster_density(x, z, covariance),
                    -z_max,
                    z_max,
                    INNER_INTERVALS,
                )
            },
            x0 - hbr,
            x0 + hbr,
            OUTER_INTERVALS,
        ),
        "square" => simpson(
            |x| {
                simpson(
                    |z| foster_density(x, z, covariance),
                    -hbr,
                    hbr,
                    INNER_INTERVALS,
                )
            },
            x0 - hbr,
            x0 + hbr,
            OUTER_INTERVALS,
        ),
        // NASA's square-equivalent-area case uses half-width sqrt(pi)/2 * HBR.
        "squareEquArea" => {
            let half_width = std::f64::consts::PI.sqrt() * 0.5 * hbr;
            simpson(
                |x| {
                    simpson(
                        |z| foster_density(x, z, covariance),
                        -half_width,
                        half_width,
                        INNER_INTERVALS,
                    )
                },
                x0 - half_width,
                x0 + half_width,
                OUTER_INTERVALS,
            )
        }
        other => panic!("unsupported pinned Foster region: {other}"),
    }
}

#[test]
fn foster_reference_integral_reproduces_pinned_upstream_expectations() {
    let fixture: Value = serde_json::from_str(FIXTURE).expect("fixture JSON");
    let input = &fixture["encounter_input"];

    let r = sub3(parse_vec3(&input["r1_km"]), parse_vec3(&input["r2_km"]));
    let v = sub3(
        parse_vec3(&input["v1_km_per_s"]),
        parse_vec3(&input["v2_km_per_s"]),
    );

    // Reproduce the encounter-plane basis documented in NASA's pinned
    // Pc2D_Foster.m: y = unit(relative velocity), z = unit(r x v),
    // x = y x z. This is a test oracle, not the production Mycelix path.
    let y_axis = unit3(v);
    let z_axis = unit3(cross3(r, v));
    let x_axis = unit3(cross3(y_axis, z_axis));
    let combined = combine_covariances(
        parse_mat3(&input["cov1_km2"]),
        parse_mat3(&input["cov2_km2"]),
    );

    let projected = [
        project_covariance(x_axis, combined, x_axis),
        project_covariance(x_axis, combined, z_axis),
        project_covariance(z_axis, combined, x_axis),
        project_covariance(z_axis, combined, z_axis),
    ];
    let determinant = projected[0] * projected[3] - projected[1] * projected[2];
    assert!(determinant.is_finite() && determinant > 0.0);

    let x0 = norm3(r);
    let hbr = input["hbr_km"].as_f64().expect("numeric HBR");
    assert!(x0.is_finite() && hbr.is_finite() && hbr > 0.0);

    let cases = fixture["cases"].as_array().expect("cases array");
    for case in cases {
        let region = case["hard_body_region"].as_str().expect("region");
        let expected = case["expected_pc"].as_f64().expect("expected Pc");
        let tolerance = case["upstream_relative_tolerance"]
            .as_f64()
            .expect("relative tolerance");
        let actual = integrate_foster_case(region, x0, hbr, projected);

        assert!(actual.is_finite() && (0.0..=1.0).contains(&actual));
        assert!(
            (actual - expected).abs() <= expected.abs() * tolerance,
            "{region}: independent Foster integral {actual:.16e} did not match pinned upstream expectation {expected:.16e} at relative tolerance {tolerance}"
        );
    }
}
