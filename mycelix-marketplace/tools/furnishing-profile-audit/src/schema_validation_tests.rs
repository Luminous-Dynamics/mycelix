#[cfg(test)]
mod schema_validation_tests {
    use serde_json::Value;
    use std::{fs, path::PathBuf};

    fn read_json(relative: &str) -> Value {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative);
        let bytes = fs::read(&path).unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()));
        serde_json::from_slice(&bytes).unwrap_or_else(|error| panic!("invalid JSON in {}: {error}", path.display()))
    }

    fn profile_validator() -> jsonschema::Validator {
        let schema = read_json("../../contracts/furnishing-catalog-profile-v0.1.schema.json");
        jsonschema::draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .expect("furnishing profile schema should compile")
    }

    fn request_validator() -> jsonschema::Validator {
        let schema = read_json("../../contracts/furnishing-adapter-request-v0.1.schema.json");
        jsonschema::draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .expect("discovery request schema should compile")
    }

    #[test]
    fn every_synthetic_profile_matches_json_schema() {
        let fixture = read_json("../../contracts/fixtures/furnishing-catalog-profile-v0.1.json");
        let profiles = fixture["profiles"].as_array().expect("profiles array");
        let validator = profile_validator();
        let mut failures = Vec::new();
        for (index, profile) in profiles.iter().enumerate() {
            for error in validator.iter_errors(profile) {
                failures.push(format!("profiles[{index}]{}: {error}", error.instance_path()));
            }
        }
        assert!(failures.is_empty(), "JSON Schema validation failed: {failures:#?}");
    }

    #[test]
    fn discovery_request_matches_json_schema() {
        let fixture = read_json("../../contracts/fixtures/furnishing-adapter-request-v0.1.json");
        let validator = request_validator();
        let failures: Vec<String> = validator
            .iter_errors(&fixture)
            .map(|error| format!("{}: {error}", error.instance_path()))
            .collect();
        assert!(failures.is_empty(), "request schema validation failed: {failures:#?}");
    }

    #[test]
    fn missing_profile_dimension_fails_json_schema() {
        let fixture = read_json("../../contracts/fixtures/furnishing-catalog-profile-v0.1.json");
        let mut profile = fixture["profiles"][0].clone();
        profile["catalog_item"]["geometry"].as_object_mut().unwrap().remove("width");
        assert!(!profile_validator().is_valid(&profile));
    }

    #[test]
    fn unauthorized_effect_fails_request_json_schema() {
        let mut request = read_json("../../contracts/fixtures/furnishing-adapter-request-v0.1.json");
        request["effects"]["reserve_stock"] = Value::Bool(true);
        assert!(!request_validator().is_valid(&request));
    }
}
