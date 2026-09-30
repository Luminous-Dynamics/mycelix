use serde::Deserialize;
use thiserror::Error;

const CORPUS: &str = include_str!("../../../docs/aerocommons/AERO_AUTHORITY_CEILING_V1.json");

#[derive(Debug, Clone, Deserialize)]
pub struct Corpus {
    pub schema: String,
    pub status: String,
    pub authority_ceiling: String,
    pub nonclaims: Vec<String>,
    pub cases: Vec<Case>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Case {
    pub id: String,
    pub name: String,
    pub boundary: String,
    pub input: String,
    pub expected_validator_result: ValidatorResult,
    pub engineering_status: String,
    pub forbidden_inference: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum ValidatorResult {
    Valid,
    Invalid,
    Unresolved,
}

#[derive(Debug, Error)]
pub enum CorpusError {
    #[error("corpus JSON is invalid: {0}")]
    Parse(#[from] serde_json::Error),
    #[error("unexpected corpus schema: {0}")]
    Schema(String),
    #[error("corpus must contain exactly {expected} cases, found {actual}")]
    CaseCount { expected: usize, actual: usize },
    #[error("duplicate case id: {0}")]
    DuplicateId(String),
    #[error("missing nonclaim: {0}")]
    MissingNonclaim(&'static str),
    #[error("case {0} has an empty required field")]
    EmptyField(String),
}

pub fn load() -> Result<Corpus, CorpusError> {
    let corpus: Corpus = serde_json::from_str(CORPUS)?;
    validate(&corpus)?;
    Ok(corpus)
}

pub fn validate(corpus: &Corpus) -> Result<(), CorpusError> {
    if corpus.schema != "aerocommons-authority-ceiling-v1" {
        return Err(CorpusError::Schema(corpus.schema.clone()));
    }
    if corpus.authority_ceiling != "protocol_integrity_only" {
        return Err(CorpusError::Schema(corpus.authority_ceiling.clone()));
    }

    for required in [
        "physical_correctness",
        "safety",
        "airworthiness",
        "certification",
        "manufacturing_conformity",
    ] {
        if !corpus.nonclaims.iter().any(|value| value == required) {
            return Err(CorpusError::MissingNonclaim(required));
        }
    }

    const EXPECTED_CASES: usize = 14;
    if corpus.cases.len() != EXPECTED_CASES {
        return Err(CorpusError::CaseCount {
            expected: EXPECTED_CASES,
            actual: corpus.cases.len(),
        });
    }

    let mut ids = std::collections::BTreeSet::new();
    for case in &corpus.cases {
        if case.id.is_empty()
            || case.name.is_empty()
            || case.boundary.is_empty()
            || case.input.is_empty()
            || case.engineering_status.is_empty()
            || case.forbidden_inference.is_empty()
        {
            return Err(CorpusError::EmptyField(case.id.clone()));
        }
        if !ids.insert(case.id.clone()) {
            return Err(CorpusError::DuplicateId(case.id.clone()));
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_corpus_is_valid() {
        let corpus = load().expect("bundled authority-ceiling corpus must validate");
        assert_eq!(corpus.cases.len(), 14);
    }

    #[test]
    fn corpus_contains_both_rejection_and_non_escalation_cases() {
        let corpus = load().unwrap();
        assert!(corpus
            .cases
            .iter()
            .any(|case| case.expected_validator_result == ValidatorResult::Invalid));
        assert!(corpus
            .cases
            .iter()
            .any(|case| case.expected_validator_result == ValidatorResult::Valid));
        assert!(corpus
            .cases
            .iter()
            .any(|case| case.expected_validator_result == ValidatorResult::Unresolved));
    }

    #[test]
    fn identity_and_epistemic_boundaries_are_present() {
        let corpus = load().unwrap();
        for id in [
            "AC-AUTH-001",
            "AC-AUTH-002",
            "AC-AUTH-003",
            "AC-AUTH-005",
            "AC-AUTH-014",
        ] {
            assert!(corpus.cases.iter().any(|case| case.id == id));
        }
    }

    #[test]
    fn dependency_and_state_boundaries_are_present() {
        let corpus = load().unwrap();
        for id in ["AC-AUTH-006", "AC-AUTH-007", "AC-AUTH-011"] {
            assert!(corpus.cases.iter().any(|case| case.id == id));
        }
    }
}
