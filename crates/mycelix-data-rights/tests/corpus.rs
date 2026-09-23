use mycelix_data_rights::{
    AssessmentState, DatasetSplitRole, DimensionDisposition, ExternalPrerequisite,
    IntendedDatasetUseV1, IntendedUseDimension, RetainedObligation, ReviewedTermsEvidenceV1,
    TermsCurrentness, assess_terms_compatibility,
};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Deserialize)]
struct Corpus {
    cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
struct Case {
    id: String,
    subjects: Vec<String>,
    split_role: DatasetSplitRole,
    reviewed_terms_evidence: EvidenceFixture,
    intended_use: BTreeSet<IntendedUseDimension>,
    expected_state: String,
    expected_obligations: BTreeSet<RetainedObligation>,
    #[serde(default)]
    external_prerequisites: BTreeSet<ExternalPrerequisite>,
}

#[derive(Debug, Deserialize)]
struct EvidenceFixture {
    currentness: TermsCurrentness,
    #[serde(default)]
    subject: Option<String>,
    #[serde(default)]
    dimensions: BTreeMap<IntendedUseDimension, DimensionDisposition>,
    #[serde(default)]
    obligations: BTreeSet<RetainedObligation>,
    #[serde(default)]
    per_source: Vec<SourceFixture>,
}

#[derive(Debug, Deserialize)]
struct SourceFixture {
    subject: String,
    #[serde(default)]
    evidence_ref: Option<String>,
    dimensions: BTreeMap<IntendedUseDimension, DimensionDisposition>,
    #[serde(default)]
    obligations: BTreeSet<RetainedObligation>,
}

fn expected_state(name: &str) -> AssessmentState {
    match name {
        "CompatibleWithReviewedTermsProfile" => AssessmentState::CompatibleWithReviewedTermsProfile,
        "IncompatibleWithReviewedTermsProfile" => {
            AssessmentState::IncompatibleWithReviewedTermsProfile
        }
        "HumanReviewRequired" => AssessmentState::HumanReviewRequired,
        "TermsEvidenceConflict" => AssessmentState::TermsEvidenceConflict,
        "TermsProfileExpiredOrSuperseded" => AssessmentState::TermsProfileExpiredOrSuperseded,
        "IntendedUseOutOfProfile" => AssessmentState::IntendedUseOutOfProfile,
        other => panic!("unknown fixture state: {other}"),
    }
}

fn materialize(case: &Case) -> (Vec<ReviewedTermsEvidenceV1>, IntendedDatasetUseV1) {
    let request = IntendedDatasetUseV1 {
        subjects: case.subjects.iter().cloned().collect(),
        dataset_split_role: case.split_role,
        dimensions: case.intended_use.clone(),
        external_prerequisites: case.external_prerequisites.clone(),
    };

    let evidence = if case.reviewed_terms_evidence.per_source.is_empty() {
        let subject = case
            .reviewed_terms_evidence
            .subject
            .clone()
            .unwrap_or_else(|| case.subjects[0].clone());
        vec![ReviewedTermsEvidenceV1 {
            subject,
            evidence_ref: format!("fixture:{}:0", case.id),
            currentness: case.reviewed_terms_evidence.currentness,
            dimensions: case.reviewed_terms_evidence.dimensions.clone(),
            obligations: case.reviewed_terms_evidence.obligations.clone(),
        }]
    } else {
        case.reviewed_terms_evidence
            .per_source
            .iter()
            .enumerate()
            .map(|(index, row)| ReviewedTermsEvidenceV1 {
                subject: row.subject.clone(),
                evidence_ref: row
                    .evidence_ref
                    .clone()
                    .unwrap_or_else(|| format!("fixture:{}:{index}", case.id)),
                currentness: case.reviewed_terms_evidence.currentness,
                dimensions: row.dimensions.clone(),
                obligations: row.obligations.clone(),
            })
            .collect()
    };

    (evidence, request)
}

#[test]
fn qualified_synthetic_corpus_v0_1_is_reproduced_exactly() {
    let raw =
        include_str!("../../../docs/embodied/fixtures/EMB_DATA_RIGHTS_COMPATIBILITY_V0_1.json");
    let corpus: Corpus = serde_json::from_str(raw).expect("qualified corpus JSON must parse");
    assert_eq!(corpus.cases.len(), 17);

    for case in &corpus.cases {
        let (evidence, request) = materialize(case);
        let assessment = assess_terms_compatibility(&evidence, &request)
            .unwrap_or_else(|error| panic!("{} failed: {error}", case.id));

        assert_eq!(
            assessment.state(),
            expected_state(&case.expected_state),
            "{}",
            case.id
        );

        let observed_obligations: BTreeSet<_> = assessment
            .retained_obligations()
            .iter()
            .map(|entry| entry.obligation())
            .collect();
        assert_eq!(
            observed_obligations, case.expected_obligations,
            "{}",
            case.id
        );

        assert_eq!(
            assessment.external_prerequisites(),
            &case.external_prerequisites,
            "{}",
            case.id
        );
    }
}

#[test]
fn corpus_order_is_not_semantic_authority() {
    let raw =
        include_str!("../../../docs/embodied/fixtures/EMB_DATA_RIGHTS_COMPATIBILITY_V0_1.json");
    let corpus: Corpus = serde_json::from_str(raw).unwrap();

    for case in corpus
        .cases
        .iter()
        .filter(|case| !case.reviewed_terms_evidence.per_source.is_empty())
    {
        let (mut evidence, request) = materialize(case);
        let expected = assess_terms_compatibility(&evidence, &request).unwrap();
        evidence.reverse();
        let reversed = assess_terms_compatibility(&evidence, &request).unwrap();
        assert_eq!(expected, reversed, "{}", case.id);
    }
}

#[test]
fn conflict_and_restrictive_composition_remain_distinct() {
    let raw =
        include_str!("../../../docs/embodied/fixtures/EMB_DATA_RIGHTS_COMPATIBILITY_V0_1.json");
    let corpus: Corpus = serde_json::from_str(raw).unwrap();
    let case = corpus
        .cases
        .iter()
        .find(|case| case.id == "C17-conflicting-reviewed-evidence")
        .expect("C17 must remain frozen");

    let (evidence, request) = materialize(case);
    assert_eq!(
        assess_terms_compatibility(&evidence, &request)
            .unwrap()
            .state(),
        AssessmentState::TermsEvidenceConflict
    );

    let mut second_subject_evidence = evidence.clone();
    second_subject_evidence[1].subject = "synthetic:dataset:u:v1".to_string();
    let mut second_subject_request = request.clone();
    second_subject_request
        .subjects
        .insert("synthetic:dataset:u:v1".to_string());

    assert_eq!(
        assess_terms_compatibility(&second_subject_evidence, &second_subject_request)
            .unwrap()
            .state(),
        AssessmentState::IncompatibleWithReviewedTermsProfile
    );
}
