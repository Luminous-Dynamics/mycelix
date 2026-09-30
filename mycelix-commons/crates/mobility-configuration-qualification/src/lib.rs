use serde::Deserialize;
use std::collections::BTreeSet;
use std::fs;

const EXPECTED_COUNT: usize = 20;
const EXPECTED_PREFIX: &str = "MC-CONFIG-";

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
pub struct Corpus {
    pub schema_version: String,
    pub status: String,
    pub vectors: Vec<Vector>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
pub struct Vector {
    pub id: String,
    pub scenario: String,
    pub expected_outcome: String,
    pub forbidden_inference: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationResult {
    Valid,
    Invalid(String),
}

pub fn parse_corpus(json: &str) -> Result<Corpus, String> {
    serde_json::from_str(json).map_err(|e| format!("invalid qualification corpus: {e}"))
}

pub fn qualify(corpus: &Corpus) -> QualificationResult {
    if corpus.schema_version != "mobility-configuration-contract-qualification-v1" {
        return QualificationResult::Invalid("unexpected schema version".into());
    }
    if corpus.status != "semantic-qualification-only" {
        return QualificationResult::Invalid("qualification corpus has unsafe status".into());
    }
    if corpus.vectors.len() != EXPECTED_COUNT {
        return QualificationResult::Invalid(format!(
            "expected {EXPECTED_COUNT} vectors, found {}",
            corpus.vectors.len()
        ));
    }

    let ids: BTreeSet<_> = corpus.vectors.iter().map(|v| v.id.as_str()).collect();
    let expected_ids: BTreeSet<_> = (1..=EXPECTED_COUNT)
        .map(|n| format!("{EXPECTED_PREFIX}{n:03}"))
        .collect();

    if ids.len() != EXPECTED_COUNT || ids != expected_ids {
        return QualificationResult::Invalid("vector identifiers are incomplete or duplicated".into());
    }

    for vector in &corpus.vectors {
        if !vector.id.starts_with(EXPECTED_PREFIX)
            || vector.scenario.is_empty()
            || vector.expected_outcome.is_empty()
            || vector.forbidden_inference.is_empty()
        {
            return QualificationResult::Invalid(format!(
                "{} is missing required semantic fields",
                vector.id
            ));
        }
    }

    QualificationResult::Valid
}

pub fn load_and_qualify(path: &str) -> Result<(), String> {
    let json = fs::read_to_string(path)
        .map_err(|e| format!("failed to read {path}: {e}"))?;
    let corpus = parse_corpus(&json)?;
    match qualify(&corpus) {
        QualificationResult::Valid => Ok(()),
        QualificationResult::Invalid(reason) => Err(reason),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corpus() -> Corpus {
        parse_corpus(include_str!("../../../../../docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1.json"))
            .expect("bundled corpus must parse")
    }

    #[test]
    fn corpus_has_exactly_twenty_vectors() {
        assert_eq!(corpus().vectors.len(), 20);
    }

    #[test]
    fn corpus_has_exact_identifier_set() {
        assert_eq!(qualify(&corpus()), QualificationResult::Valid);
    }

    #[test]
    fn every_vector_preserves_a_forbidden_inference_boundary() {
        assert!(corpus()
            .vectors
            .iter()
            .all(|v| !v.forbidden_inference.trim().is_empty()));
    }

    #[test]
    fn duplicate_vector_ids_are_rejected() {
        let mut c = corpus();
        c.vectors[1].id = c.vectors[0].id.clone();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn missing_vector_is_rejected() {
        let mut c = corpus();
        c.vectors.pop();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn wrong_schema_is_rejected() {
        let mut c = corpus();
        c.schema_version = "other".into();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn authority_boundary_is_not_a_safety_claim() {
        let c = corpus();
        assert_eq!(c.status, "semantic-qualification-only");
        assert!(c.vectors.iter().any(|v| v.id == "MC-CONFIG-013"));
        assert!(c.vectors.iter().any(|v| v.id == "MC-CONFIG-017"));
    }
}
