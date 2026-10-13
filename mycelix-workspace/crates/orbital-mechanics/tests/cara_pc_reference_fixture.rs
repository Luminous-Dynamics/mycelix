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
            let scale = upper.abs().max(lower.abs()).max(1.0);
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
        "not_yet_run",
        "fixture metadata must not drift into claiming algorithm parity"
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
    assert_eq!(input["epoch"], Value::Null, "do not invent an epoch");

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
