use mycelix_manufacturing_process_refs::{ExternalSubjectRefV1, ProcessRecipeRefV1};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
struct RecipeCommitmentSubjectRefV1(ExternalSubjectRefV1);

impl RecipeCommitmentSubjectRefV1 {
    fn validate(&self) -> Result<(), String> {
        self.0.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum RoutingRecipeSelectorV1 {
    PublicRecipe(ProcessRecipeRefV1),
    PrivateCommitment(RecipeCommitmentSubjectRefV1),
}

impl RoutingRecipeSelectorV1 {
    fn validate(&self) -> Result<(), String> {
        match self {
            Self::PublicRecipe(reference) => reference.validate(),
            Self::PrivateCommitment(reference) => reference.validate(),
        }
    }

    fn is_qualification_evidence(&self) -> bool {
        false
    }

    fn is_execution_evidence(&self) -> bool {
        false
    }
}

fn subject(namespace: &str, id: &str, fill: char) -> ExternalSubjectRefV1 {
    use mycelix_manufacturing_process_refs::{DigestAlgorithmV1, DigestRefV1};
    ExternalSubjectRefV1 {
        namespace: namespace.into(),
        subject_id: id.into(),
        semantic_version: "1".into(),
        content_digest: DigestRefV1 {
            algorithm: DigestAlgorithmV1::Blake3,
            hex: std::iter::repeat_n(fill, 64).collect(),
        },
    }
}

#[test]
fn public_recipe_and_private_commitment_remain_distinct_after_round_trip() {
    let public = RoutingRecipeSelectorV1::PublicRecipe(ProcessRecipeRefV1(subject(
        "symthaea.mfg-recipe",
        "recipe-public-1",
        'a',
    )));
    let private = RoutingRecipeSelectorV1::PrivateCommitment(RecipeCommitmentSubjectRefV1(
        subject("symthaea.mfg-recipe-commitment", "commitment-1", 'b'),
    ));

    for selector in [public, private] {
        assert!(selector.validate().is_ok());
        let encoded = serde_json::to_string(&selector).unwrap();
        let decoded: RoutingRecipeSelectorV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, selector);
    }
}

#[test]
fn private_commitment_does_not_disclose_recipe_payload() {
    let selector = RoutingRecipeSelectorV1::PrivateCommitment(RecipeCommitmentSubjectRefV1(
        subject("symthaea.mfg-recipe-commitment", "commitment-opaque-42", 'c'),
    ));
    let encoded = serde_json::to_string(&selector).unwrap();
    assert!(!encoded.contains("feed_rate"));
    assert!(!encoded.contains("spindle_speed"));
    assert!(!encoded.contains("proprietary"));
}

#[test]
fn malformed_commitment_ref_fails_closed() {
    use mycelix_manufacturing_process_refs::{DigestAlgorithmV1, DigestRefV1};
    let malformed = RoutingRecipeSelectorV1::PrivateCommitment(RecipeCommitmentSubjectRefV1(
        ExternalSubjectRefV1 {
            namespace: "symthaea.mfg-recipe-commitment".into(),
            subject_id: "commitment-bad".into(),
            semantic_version: "1".into(),
            content_digest: DigestRefV1 {
                algorithm: DigestAlgorithmV1::Blake3,
                hex: "ABC".into(),
            },
        },
    ));
    assert!(malformed.validate().is_err());
}

#[test]
fn selector_never_implies_qualification_or_execution() {
    let selector = RoutingRecipeSelectorV1::PrivateCommitment(RecipeCommitmentSubjectRefV1(
        subject("symthaea.mfg-recipe-commitment", "commitment-qual-1", 'd'),
    ));
    assert!(!selector.is_qualification_evidence());
    assert!(!selector.is_execution_evidence());
}

#[test]
fn legacy_absence_is_not_an_implicit_recipe_default() {
    let selector: Option<RoutingRecipeSelectorV1> = None;
    assert!(selector.is_none());
}
