use serde_json::{Map, Value};

use cos_conformance::canonical_derivation_receipt::canonical_sha256;
use cos_conformance::integral_interop::{
    selected_oad_design_semantic_commitment_checked, selected_oad_design_semantic_projection,
};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
}

fn path_tokens(path: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    for segment in path.split('.') {
        let mut rest = segment;
        while let Some(open) = rest.find('[') {
            if open > 0 {
                tokens.push(rest[..open].to_owned());
            }
            let close = rest[open..]
                .find(']')
                .expect("array path token must close");
            tokens.push(rest[open + 1..open + close].to_owned());
            rest = &rest[open + close + 1..];
        }
        if !rest.is_empty() {
            tokens.push(rest.to_owned());
        }
    }
    tokens
}

fn set_path(root: &mut Value, path: &str, replacement: Value) {
    let tokens = path_tokens(path);
    assert!(!tokens.is_empty());

    fn descend(current: &mut Value, tokens: &[String], replacement: Value) {
        if tokens.len() == 1 {
            match current {
                Value::Object(map) => {
                    map.insert(tokens[0].clone(), replacement);
                }
                Value::Array(values) => {
                    let index: usize = tokens[0].parse().expect("array index");
                    values[index] = replacement;
                }
                _ => panic!("cannot descend into scalar"),
            }
            return;
        }

        match current {
            Value::Object(map) => descend(
                map.get_mut(&tokens[0]).expect("object path component"),
                &tokens[1..],
                replacement,
            ),
            Value::Array(values) => {
                let index: usize = tokens[0].parse().expect("array index");
                descend(&mut values[index], &tokens[1..], replacement);
            }
            _ => panic!("cannot descend into scalar"),
        }
    }

    descend(root, &tokens, replacement);
}

fn reorder_object_at_path(root: &mut Value, path: &str) {
    let tokens = path_tokens(path);
    fn descend(current: &mut Value, tokens: &[String]) {
        if tokens.is_empty() {
            let object = current.as_object().expect("target must be object").clone();
            let mut reversed = Map::new();
            for (key, value) in object.into_iter().rev() {
                reversed.insert(key, value);
            }
            *current = Value::Object(reversed);
            return;
        }
        match current {
            Value::Object(map) => descend(
                map.get_mut(&tokens[0]).expect("object path component"),
                &tokens[1..],
            ),
            Value::Array(values) => {
                let index: usize = tokens[0].parse().expect("array index");
                descend(&mut values[index], &tokens[1..]);
            }
            _ => panic!("cannot descend into scalar"),
        }
    }
    descend(root, &tokens);
}

fn swap_array_at_path(root: &mut Value, path: &str) {
    let tokens = path_tokens(path);
    fn descend(current: &mut Value, tokens: &[String]) {
        if tokens.is_empty() {
            current
                .as_array_mut()
                .expect("target must be array")
                .swap(0, 1);
            return;
        }
        match current {
            Value::Object(map) => descend(
                map.get_mut(&tokens[0]).expect("object path component"),
                &tokens[1..],
            ),
            Value::Array(values) => {
                let index: usize = tokens[0].parse().expect("array index");
                descend(&mut values[index], &tokens[1..]);
            }
            _ => panic!("cannot descend into scalar"),
        }
    }
    descend(root, &tokens);
}

fn apply_vector(baseline: &Value, vector: &Value) -> Result<Value, String> {
    let operation = vector["operation"]
        .as_str()
        .ok_or_else(|| "missing operation".to_owned())?;
    let path = vector["path"]
        .as_str()
        .ok_or_else(|| "missing path".to_owned())?;

    let mut value = baseline.clone();
    match operation {
        "set" => set_path(&mut value, path, vector["value"].clone()),
        "reorder-object" => reorder_object_at_path(&mut value, path),
        "swap-array" => swap_array_at_path(&mut value, path),
        "reparse" => {
            let raw = vector["value"]
                .as_str()
                .ok_or_else(|| "reparse value must be a JSON string".to_owned())?;
            value = serde_json::from_str(raw).map_err(|error| error.to_string())?;
        }
        "hash-domain" => return Ok(vector["value"].clone()),
        other => return Err(format!("unsupported metamorphic operation: {other}")),
    }

    Ok(value)
}

#[test]
fn machine_readable_metamorphic_corpus_is_self_describing() {
    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_metamorphic_vectors.json"
    ))
    .expect("metamorphic corpus must be valid JSON");

    assert_eq!(corpus["profile"], "integral-interop-1");
    assert_eq!(
        corpus["projection_version"],
        "integral-interop-1-design-semantic-v1"
    );

    for vector in corpus["vectors"].as_array().expect("vectors array") {
        assert!(vector["id"].is_string());
        assert!(vector["operation"].is_string());
        assert!(vector["path"].is_string());
        assert!(vector["expected"].is_string());
    }
}

#[test]
fn executable_metamorphic_corpus_enforces_identity_invariants() {
    let baseline = fixture();
    let baseline_commitment =
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap();

    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_metamorphic_vectors.json"
    ))
    .expect("metamorphic corpus must be valid JSON");

    for vector in corpus["vectors"].as_array().expect("vectors array") {
        let id = vector["id"].as_str().unwrap();
        let expected = vector["expected"].as_str().unwrap();

        if id == "projection-version-domain" {
            let projection = selected_oad_design_semantic_projection(&baseline);
            let changed_domain = vector["value"].as_str().unwrap();
            assert_ne!(
                baseline_commitment,
                canonical_sha256(changed_domain, &projection),
                "{id}"
            );
            continue;
        }

        let mutated = apply_vector(&baseline, vector).unwrap();

        match expected {
            "same-commitment" => assert_eq!(
                baseline_commitment,
                selected_oad_design_semantic_commitment_checked(&mutated).unwrap(),
                "{id}"
            ),
            "different-commitment" => assert_ne!(
                baseline_commitment,
                selected_oad_design_semantic_commitment_checked(&mutated).unwrap(),
                "{id}"
            ),
            "reject-no-commitment" => {
                assert!(
                    selected_oad_design_semantic_commitment_checked(&mutated).is_err(),
                    "{id} must fail closed"
                );
            }
            other => panic!("unknown expected outcome: {other}"),
        }
    }
}

#[test]
fn unknown_top_level_fields_remain_outside_identity() {
    let baseline = fixture();
    let mut changed = baseline.clone();
    changed["future_top_level_field"] = serde_json::json!({
        "must_not": "enter the frozen semantic projection"
    });

    assert_eq!(
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap(),
        selected_oad_design_semantic_commitment_checked(&changed).unwrap()
    );
}
