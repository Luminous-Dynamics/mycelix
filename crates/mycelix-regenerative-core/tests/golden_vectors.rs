use mycelix_regenerative_core::{
    BiocharBatchId, BiomassLotId, CoCompostedAmendmentBatchId, CompostBatchId, FieldTrialId,
    QualityProfileId, RegenerativeEvidenceBundleId, RegenerativeFacilityId, RegenerativeProjectId,
    RegenerativeRecipeId, RegenerativeSiteId, SoilPlotId, TreatmentArmId,
};
use serde_json::Value;
use std::collections::BTreeSet;

const GOLDEN: &str = include_str!("../test-vectors/regen_subject_ids_v1.json");

fn build(kind: &str, token: &str) -> Result<String, String> {
    macro_rules! make {
        ($ty:ty) => {
            <$ty>::new(token)
                .map(|id| id.to_string())
                .map_err(|error| error.to_string())
        };
    }
    match kind {
        "site" => make!(RegenerativeSiteId),
        "soil-plot" => make!(SoilPlotId),
        "biomass-lot" => make!(BiomassLotId),
        "biochar-batch" => make!(BiocharBatchId),
        "compost-batch" => make!(CompostBatchId),
        "co-composted-amendment-batch" => make!(CoCompostedAmendmentBatchId),
        "recipe" => make!(RegenerativeRecipeId),
        "field-trial" => make!(FieldTrialId),
        "treatment-arm" => make!(TreatmentArmId),
        "facility" => make!(RegenerativeFacilityId),
        "project" => make!(RegenerativeProjectId),
        "quality-profile" => make!(QualityProfileId),
        "evidence-bundle" => make!(RegenerativeEvidenceBundleId),
        other => Err(format!("unknown golden-vector kind {other}")),
    }
}

fn parses(kind: &str, canonical: &str) -> bool {
    macro_rules! parse {
        ($ty:ty) => {
            <$ty>::parse_canonical(canonical).is_ok()
        };
    }
    match kind {
        "site" => parse!(RegenerativeSiteId),
        "soil-plot" => parse!(SoilPlotId),
        "biomass-lot" => parse!(BiomassLotId),
        "biochar-batch" => parse!(BiocharBatchId),
        "compost-batch" => parse!(CompostBatchId),
        "co-composted-amendment-batch" => parse!(CoCompostedAmendmentBatchId),
        "recipe" => parse!(RegenerativeRecipeId),
        "field-trial" => parse!(FieldTrialId),
        "treatment-arm" => parse!(TreatmentArmId),
        "facility" => parse!(RegenerativeFacilityId),
        "project" => parse!(RegenerativeProjectId),
        "quality-profile" => parse!(QualityProfileId),
        "evidence-bundle" => parse!(RegenerativeEvidenceBundleId),
        _ => false,
    }
}

fn fixture() -> Value {
    serde_json::from_str(GOLDEN).expect("checked-in golden vectors must be valid JSON")
}

#[test]
fn golden_schema_and_kind_roster_are_frozen() {
    let fixture = fixture();
    assert_eq!(
        fixture["schema"].as_str(),
        Some("mycelix.regen.subject-id-golden-v1")
    );

    let observed: BTreeSet<_> = fixture["valid"]
        .as_array()
        .unwrap()
        .iter()
        .map(|row| row["kind"].as_str().unwrap())
        .collect();
    let expected: BTreeSet<_> = [
        "site",
        "soil-plot",
        "biomass-lot",
        "biochar-batch",
        "compost-batch",
        "co-composted-amendment-batch",
        "recipe",
        "field-trial",
        "treatment-arm",
        "facility",
        "project",
        "quality-profile",
        "evidence-bundle",
    ]
    .into_iter()
    .collect();
    assert_eq!(observed, expected);
}

#[test]
fn production_identity_code_reproduces_every_valid_golden_vector() {
    let fixture = fixture();
    let mut canonical_ids = BTreeSet::new();
    for row in fixture["valid"].as_array().unwrap() {
        let kind = row["kind"].as_str().unwrap();
        let token = row["token"].as_str().unwrap();
        let canonical = row["canonical"].as_str().unwrap();
        assert_eq!(build(kind, token).as_deref(), Ok(canonical));
        assert!(parses(kind, canonical), "failed to parse {canonical}");
        assert!(
            canonical_ids.insert(canonical),
            "duplicate canonical golden identity {canonical}"
        );
    }
}

#[test]
fn production_parser_rejects_every_invalid_golden_vector() {
    let fixture = fixture();
    for row in fixture["invalid"].as_array().unwrap() {
        let kind = row["kind"].as_str().unwrap();
        let canonical = row["canonical"].as_str().unwrap();
        let reason = row["reason"].as_str().unwrap();
        assert!(
            !parses(kind, canonical),
            "invalid vector unexpectedly parsed: kind={kind} canonical={canonical} reason={reason}"
        );
    }
}
