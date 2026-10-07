use serde_json::Value;

use cos_conformance::integral_interop::{
    selected_oad_design_semantic_commitment_checked, validate_selected_oad_design_semantics,
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

fn remove_path(root: &mut Value, path: &str) {
    let tokens = path_tokens(path);
    assert!(!tokens.is_empty());

    fn descend(current: &mut Value, tokens: &[String]) {
        if tokens.len() == 1 {
            match current {
                Value::Object(map) => {
                    assert!(map.remove(&tokens[0]).is_some());
                }
                Value::Array(values) => {
                    let index: usize = tokens[0].parse().expect("array index");
                    values.remove(index);
                }
                _ => panic!("cannot descend into scalar"),
            }
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

fn apply_vector(baseline: &Value, vector: &Value) -> Value {
    let operation = vector["operation"].as_str().expect("operation");
    let path = vector["path"].as_str().expect("path");
    let mut value = baseline.clone();

    match operation {
        "none" => {}
        "set" => set_path(&mut value, path, vector["value"].clone()),
        "remove" => remove_path(&mut value, path),
        other => panic!("unsupported validation operation: {other}"),
    }

    value
}

#[test]
fn validation_corpus_is_machine_readable_and_self_describing() {
    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_validation_vectors.json"
    ))
    .expect("validation corpus must be valid JSON");

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
fn executable_validation_corpus_matches_the_frozen_boundary() {
    let baseline = fixture();
    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_validation_vectors.json"
    ))
    .expect("validation corpus must be valid JSON");

    for vector in corpus["vectors"].as_array().expect("vectors array") {
        let id = vector["id"].as_str().unwrap();
        let expected = vector["expected"].as_str().unwrap();
        let value = apply_vector(&baseline, vector);

        match expected {
            "accept" => {
                assert!(
                    validate_selected_oad_design_semantics(&value).is_ok(),
                    "{id} must accept"
                );
                assert!(
                    selected_oad_design_semantic_commitment_checked(&value).is_ok(),
                    "{id} must produce a commitment"
                );
            }
            "reject" => {
                assert!(
                    validate_selected_oad_design_semantics(&value).is_err(),
                    "{id} must reject"
                );
                assert!(
                    selected_oad_design_semantic_commitment_checked(&value).is_err(),
                    "{id} must produce no commitment"
                );
            }
            other => panic!("unknown validation outcome: {other}"),
        }
    }
}
