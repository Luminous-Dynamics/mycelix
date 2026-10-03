// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
// Hearth integration test support — re-exports for test modules.


#[test]
fn test_semantic_validation_case_manifest_is_structurally_valid() {
    let manifest: serde_json::Value = serde_json::from_str(include_str!(
        "hearth-07-semantic-validation-cases.json"
    ))
    .expect("semantic validation case manifest must be valid JSON");

    assert_eq!(manifest["schema_version"], "HEARTH-SEMANTIC-0.7-CASESET-1");
    let cases = manifest["cases"]
        .as_array()
        .expect("semantic validation manifest must contain a cases array");
    assert_eq!(cases.len(), 3);

    for case in cases {
        assert!(case["case_id"].is_string());
        assert!(case["test"].is_string());
        assert!(case["zome"].is_string());
        assert!(case["operation"].is_string());
        assert!(case["invariant"].is_string());
        assert_eq!(case["boundary"], "integrity_validation");
    }
}
