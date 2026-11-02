use gcpn_common_source_comparison::{
    Availability, CaseEligibility, ComparisonCase, ComparisonContract, ComparisonManifest,
    DatasetReleaseManifest, ForecastAvailability, ForecastInputRequirement, MetricManifest,
    ModelOutputManifest, ProductKind, RedistributionPolicy, SpatialSupportContract,
    VerificationContract,
};
use gcpn_paired_fss_adapter::{
    ForecastGridArtifact, PairedGridArtifact, VerificationGridArtifact, score_paired_manifest,
};
use gcpn_probabilistic_fss::{
    BlockBootstrapConfig, BoundaryPolicy, FssCase, FssConfig, MissingPolicy, ThresholdOperator,
};

const DIGEST_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const DIGEST_B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
const DIGEST_C: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
const DIGEST_D: &str = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
const DIGEST_E: &str = "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
const DIGEST_F: &str = "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";

fn model(model_id: &str, digest: &str) -> ModelOutputManifest {
    ModelOutputManifest {
        model_id: model_id.into(),
        model_version: "v1".into(),
        archive_release: "archive-v1".into(),
        artifact_sha256: digest.into(),
        predictand_id: "t2m".into(),
        unit: "K".into(),
        temporal_support_id: "hourly".into(),
        spatial_support_id: "grid-v1".into(),
        grid_sha256: DIGEST_A.into(),
        mask_sha256: DIGEST_B.into(),
        climatology_id: "climatology-v1".into(),
        product_kind: ProductKind::Ensemble,
        ensemble_members: Some(3),
        training_cutoff: Some(50),
        post_processing: None,
    }
}

fn forecast_availability(digest: &str, initialized_at: u64) -> ForecastAvailability {
    ForecastAvailability::Available {
        artifact_sha256: digest.into(),
        initialized_at,
        recorded_at: 300,
    }
}

fn verification_availability() -> Availability {
    Availability::Available {
        artifact_sha256: DIGEST_C.into(),
        recorded_at: 310,
    }
}

fn comparison_case(
    case_id: &str,
    initialization_time: u64,
    valid_time: u64,
    candidate: ForecastAvailability,
    reference: ForecastAvailability,
) -> ComparisonCase {
    ComparisonCase {
        case_id: case_id.into(),
        initialization_time,
        valid_time,
        lead_seconds: valid_time - initialization_time,
        eligibility: CaseEligibility::Eligible,
        candidate,
        reference,
        verification: verification_availability(),
    }
}

fn comparison_manifest() -> ComparisonManifest {
    ComparisonManifest {
        case_frame_id: "synthetic-frame-v1".into(),
        dataset_release: DatasetReleaseManifest {
            dataset_id: "verification-source".into(),
            release_id: "release-v1".into(),
            source_locator: "https://example.invalid/verification".into(),
            citation: "Synthetic verification fixture".into(),
            retrieved_at: 200,
            artifact_sha256: DIGEST_C.into(),
            license_id: "license-v1".into(),
            terms_url: "https://example.invalid/terms".into(),
            redistribution_policy: RedistributionPolicy::Unknown,
        },
        spatial_support: SpatialSupportContract {
            support_id: "grid-v1".into(),
            grid_sha256: DIGEST_A.into(),
            mask_sha256: DIGEST_B.into(),
            coordinate_convention: "local_cartesian".into(),
            area_weight_scheme: "uniform".into(),
        },
        verification: VerificationContract {
            source_id: "verification-source".into(),
            source_version: "version-v1".into(),
            release_id: "release-v1".into(),
            artifact_sha256: DIGEST_C.into(),
            observation_lineage_id: "verification-lineage-v1".into(),
            processing_epoch: "processing-v1".into(),
            as_of_cutoff: 400,
        },
        contract: ComparisonContract {
            comparison_id: "synthetic-paired-comparison-v1".into(),
            predictand_id: "t2m".into(),
            unit: "K".into(),
            temporal_support_id: "hourly".into(),
            spatial_support_id: "grid-v1".into(),
            climatology_id: "climatology-v1".into(),
            evaluation_start: 100,
            evaluation_end: 300,
            as_of_cutoff: 400,
            intrinsic_skill_claim: true,
        },
        candidate: model("candidate-model", DIGEST_D),
        reference: model("reference-model", DIGEST_E),
        metric: MetricManifest {
            metric_id: "pfss".into(),
            definition_version: "v1".into(),
            scoring_engine: "gcpn-pfss".into(),
            scoring_engine_version: "v1".into(),
            configuration_sha256: DIGEST_F.into(),
            input_requirement: ForecastInputRequirement::EnsembleOnly,
        },
        cases: vec![
            comparison_case(
                "case-a",
                100,
                110,
                forecast_availability(DIGEST_D, 100),
                forecast_availability(DIGEST_E, 100),
            ),
            comparison_case(
                "case-b",
                200,
                210,
                forecast_availability(DIGEST_F, 200),
                forecast_availability(DIGEST_A, 200),
            ),
        ],
    }
}

fn field(case_id: &str, forecast_events: [bool; 4]) -> FssCase {
    let observation_events = [true, false, false, false];
    let values = |events: [bool; 4]| {
        events
            .into_iter()
            .map(|event| Some(if event { 1.0 } else { 0.0 }))
            .collect()
    };
    FssCase {
        case_id: case_id.into(),
        rows: 2,
        columns: 2,
        forecast_members: vec![
            values(forecast_events),
            values(forecast_events),
            values(forecast_events),
        ],
        observation: values(observation_events),
        cell_weights: vec![1.0; 4],
        valid_mask: vec![true; 4],
    }
}

fn paired_artifact(
    case_id: &str,
    event_block_id: &str,
    candidate_digest: &str,
    reference_digest: &str,
    initialization_time: u64,
    valid_time: u64,
) -> PairedGridArtifact {
    let observation = [true, false, false, false];
    let miss = [false, false, false, false];
    PairedGridArtifact {
        case_id: case_id.into(),
        candidate: ForecastGridArtifact {
            case_id: case_id.into(),
            artifact_sha256: candidate_digest.into(),
            initialized_at: initialization_time,
            valid_time,
            lead_seconds: valid_time - initialization_time,
            recorded_at: 300,
            grid_sha256: DIGEST_A.into(),
            mask_sha256: DIGEST_B.into(),
            spatial_support_id: "grid-v1".into(),
            event_block_id: event_block_id.into(),
            field: field(case_id, observation),
        },
        reference: ForecastGridArtifact {
            case_id: case_id.into(),
            artifact_sha256: reference_digest.into(),
            initialized_at: initialization_time,
            valid_time,
            lead_seconds: valid_time - initialization_time,
            recorded_at: 300,
            grid_sha256: DIGEST_A.into(),
            mask_sha256: DIGEST_B.into(),
            spatial_support_id: "grid-v1".into(),
            event_block_id: event_block_id.into(),
            field: field(case_id, miss),
        },
        verification: VerificationGridArtifact {
            artifact_sha256: DIGEST_C.into(),
            recorded_at: 310,
        },
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest = comparison_manifest();
    let artifacts = vec![
        paired_artifact("case-a", "storm-a", DIGEST_D, DIGEST_E, 100, 110),
        paired_artifact("case-b", "storm-b", DIGEST_F, DIGEST_A, 200, 210),
    ];
    let config = FssConfig {
        threshold: 0.5,
        threshold_operator: ThresholdOperator::GreaterThanOrEqual,
        unit: "K".into(),
        window_width_cells: 1,
        boundary_policy: BoundaryPolicy::TruncateAtDomain,
        missing_policy: MissingPolicy::RejectCase,
    };
    let report = score_paired_manifest(
        &manifest,
        &artifacts,
        &config,
        &BlockBootstrapConfig {
            replicates: 128,
            seed: 20261102,
        },
    )?;

    println!("# Synthetic paired pFSS adapter demonstration; no real forecast data.");
    println!("{}", report.summary_line());
    print!("{}", report.to_csv());
    println!("score_calibration=not_assessed");
    println!("crps_brier_energy_variogram=not_computed");

    Ok(())
}
