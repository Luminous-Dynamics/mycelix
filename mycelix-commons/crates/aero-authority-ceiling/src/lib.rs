use serde::{Deserialize, Serialize};
use thiserror::Error;

const CORPUS: &str =
    include_str!("../../../docs/aerocommons/AERO_AUTHORITY_CEILING_V1.json");
const FIXTURES: &str =
    include_str!("../../../docs/aerocommons/AERO_HOLOCHAIN_FIXTURE_CONTRACT_V1.json");

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

#[derive(Debug, Clone, Deserialize)]
pub struct FixtureContract {
    pub schema: String,
    pub status: String,
    pub validator_model: String,
    pub authority_ceiling: String,
    pub cases: Vec<FixtureCase>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct FixtureCase {
    pub id: String,
    pub operation_kind: String,
    pub operation_variant: String,
    pub subject_kind: String,
    pub dependency_mode: String,
    pub mutable_state_dependency: bool,
    pub expected_validator_result: ValidatorResult,
    pub engineering_status: String,
    pub forbidden_inference: String,
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
    #[error("fixture contract is invalid: {0}")]
    Fixture(String),
}

pub fn load() -> Result<Corpus, CorpusError> {
    let corpus: Corpus = serde_json::from_str(CORPUS)?;
    validate(&corpus)?;
    Ok(corpus)
}

pub fn load_fixture_contract() -> Result<FixtureContract, CorpusError> {
    let contract: FixtureContract = serde_json::from_str(FIXTURES)?;
    validate_fixture_contract(&contract)?;
    Ok(contract)
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

pub fn validate_fixture_contract(contract: &FixtureContract) -> Result<(), CorpusError> {
    if contract.schema != "aerocommons-holochain-fixture-contract-v1" {
        return Err(CorpusError::Fixture(format!(
            "unexpected schema: {}",
            contract.schema
        )));
    }
    if contract.validator_model != "holochain-integrity-validation" {
        return Err(CorpusError::Fixture(format!(
            "unexpected validator model: {}",
            contract.validator_model
        )));
    }
    if contract.authority_ceiling != "protocol_integrity_only" {
        return Err(CorpusError::Fixture(format!(
            "unexpected authority ceiling: {}",
            contract.authority_ceiling
        )));
    }
    if contract.cases.len() != 14 {
        return Err(CorpusError::Fixture(format!(
            "expected 14 fixture cases, found {}",
            contract.cases.len()
        )));
    }

    let corpus = load()?;
    let corpus_ids: std::collections::BTreeSet<_> =
        corpus.cases.iter().map(|case| case.id.as_str()).collect();
    let fixture_ids: std::collections::BTreeSet<_> = contract
        .cases
        .iter()
        .map(|case| case.id.as_str())
        .collect();

    if corpus_ids != fixture_ids {
        return Err(CorpusError::Fixture(
            "fixture IDs must exactly match authority-ceiling corpus IDs".into(),
        ));
    }

    for fixture in &contract.cases {
        if fixture.operation_kind.is_empty()
            || fixture.operation_variant.is_empty()
            || fixture.subject_kind.is_empty()
            || fixture.dependency_mode.is_empty()
            || fixture.engineering_status.is_empty()
            || fixture.forbidden_inference.is_empty()
        {
            return Err(CorpusError::Fixture(format!(
                "{} has an empty required field",
                fixture.id
            )));
        }

        let expected_operation = if fixture.id == "AC-AUTH-007" {
            "FlatOp::Link"
        } else {
            "FlatOp::CreateRecord"
        };
        if fixture.operation_kind != expected_operation {
            return Err(CorpusError::Fixture(format!(
                "{} declares unexpected Holochain 0.7 FlatOp kind {}",
                fixture.id, fixture.operation_kind
            )));
        }

        let expected_variant = if fixture.id == "AC-AUTH-007" {
            "OpLink::CreateLink"
        } else {
            "OpRecord::CreateEntry"
        };
        if fixture.operation_variant != expected_variant {
            return Err(CorpusError::Fixture(format!(
                "{} declares unexpected Holochain 0.7 operation variant {}",
                fixture.id, fixture.operation_variant
            )));
        }

        if fixture.mutable_state_dependency
            && fixture.expected_validator_result != ValidatorResult::Invalid
        {
            return Err(CorpusError::Fixture(format!(
                "{} marks mutable state as a validation dependency without rejecting it",
                fixture.id
            )));
        }

        let source = corpus
            .cases
            .iter()
            .find(|case| case.id == fixture.id)
            .expect("fixture IDs were checked above");

        if fixture.expected_validator_result != source.expected_validator_result
            || fixture.engineering_status != source.engineering_status
            || fixture.forbidden_inference != source.forbidden_inference
        {
            return Err(CorpusError::Fixture(format!(
                "{} diverges from the authority-ceiling corpus",
                fixture.id
            )));
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
    fn bundled_holochain_fixture_contract_is_valid() {
        let fixtures =
            load_fixture_contract().expect("bundled Holochain fixture contract must validate");
        assert_eq!(fixtures.cases.len(), 14);
    }

    #[test]
    fn fixture_contract_has_one_to_one_case_coverage() {
        let corpus = load().unwrap();
        let fixtures = load_fixture_contract().unwrap();

        let corpus_ids: std::collections::BTreeSet<_> =
            corpus.cases.iter().map(|case| case.id.as_str()).collect();
        let fixture_ids: std::collections::BTreeSet<_> =
            fixtures.cases.iter().map(|case| case.id.as_str()).collect();

        assert_eq!(corpus_ids, fixture_ids);
    }

    #[test]
    fn fixture_contract_uses_real_holochain_07_flat_op_names() {
        let fixtures = load_fixture_contract().unwrap();
        for case in &fixtures.cases {
            if case.id == "AC-AUTH-007" {
                assert_eq!(case.operation_kind, "FlatOp::Link");
                assert_eq!(case.operation_variant, "OpLink::CreateLink");
            } else {
                assert_eq!(case.operation_kind, "FlatOp::CreateRecord");
                assert_eq!(case.operation_variant, "OpRecord::CreateEntry");
            }
        }
    }

    #[test]
    fn mutable_state_is_only_used_by_rejection_fixture() {
        let fixtures = load_fixture_contract().unwrap();
        for case in &fixtures.cases {
            if case.mutable_state_dependency {
                assert_eq!(case.expected_validator_result, ValidatorResult::Invalid);
            }
        }
    }

    #[test]
    fn missing_dependency_is_unresolved_not_invalid() {
        let fixtures = load_fixture_contract().unwrap();
        let case = fixtures
            .cases
            .iter()
            .find(|case| case.id == "AC-AUTH-011")
            .unwrap();

        assert_eq!(case.dependency_mode, "addressable_valid_record");
        assert_eq!(case.expected_validator_result, ValidatorResult::Unresolved);
    }

    #[test]
    fn corpus_contains_all_three_protocol_outcomes() {
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
