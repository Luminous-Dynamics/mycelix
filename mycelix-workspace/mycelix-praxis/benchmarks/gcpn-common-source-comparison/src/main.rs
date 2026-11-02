use gcpn_common_source_comparison::{
    Availability, CaseEligibility, ComparisonCase, ComparisonContract, ComparisonManifest,
    DatasetReleaseManifest, ForecastInputRequirement, MetricManifest, ModelOutputManifest,
    ProductKind, RedistributionPolicy, SpatialSupportContract, VerificationContract,
};

const GRID: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const MASK: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
const VERIFY: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest = sample_manifest();
    let report = manifest.report()?;

    println!("# Contract and availability report; forecast scores are not calculated.");
    println!("comparison_id={}", report.comparison_id);
    println!("case_frame_id={}", report.case_frame_id);
    println!("verification_source={}", report.verification_source);
    println!("verification_version={}", report.verification_version);
    println!("as_of_cutoff={}", report.as_of_cutoff);
    println!("{}", report.summary_line());
    print!("{}", report.to_csv());
    println!("score_calculation=not_performed");

    Ok(())
}

fn sample_manifest() -> ComparisonManifest {
    ComparisonManifest {
        case_frame_id: "sample-frame-v1".into(),
        dataset_release: DatasetReleaseManifest {
            dataset_id: "era5".into(),
            release_id: "era5-release-v1".into(),
            source_locator: "https://example.invalid/era5".into(),
            citation: "Declared ERA5 source release".into(),
            retrieved_at: 240,
            artifact_sha256: VERIFY.into(),
            license_id: "declared-license".into(),
            terms_url: "https://example.invalid/terms".into(),
            redistribution_policy: RedistributionPolicy::Unknown,
        },
        spatial_support: SpatialSupportContract {
            support_id: "global-grid-v1".into(),
            grid_sha256: GRID.into(),
            mask_sha256: MASK.into(),
            coordinate_convention: "latitude_longitude".into(),
            area_weight_scheme: "cell_area".into(),
        },
        verification: VerificationContract {
            source_id: "era5".into(),
            source_version: "v1".into(),
            release_id: "era5-release-v1".into(),
            artifact_sha256: VERIFY.into(),
            observation_lineage_id: "era5-lineage-v1".into(),
            processing_epoch: "processing-v1".into(),
            as_of_cutoff: 300,
        },
        contract: ComparisonContract {
            comparison_id: "demo-common-source-v1".into(),
            predictand_id: "t2m".into(),
            unit: "K".into(),
            temporal_support_id: "hourly".into(),
            spatial_support_id: "global-grid-v1".into(),
            climatology_id: "era5-clim-v1".into(),
            evaluation_start: 100,
            evaluation_end: 200,
            as_of_cutoff: 300,
            intrinsic_skill_claim: true,
        },
        candidate: model(
            "candidate",
            "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
        ),
        reference: model(
            "baseline",
            "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
        ),
        metric: MetricManifest {
            metric_id: "rmse".into(),
            definition_version: "v1".into(),
            scoring_engine: "gcpn-score-engine".into(),
            scoring_engine_version: "v1".into(),
            configuration_sha256:
                "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".into(),
            input_requirement: ForecastInputRequirement::DeterministicOnly,
        },
        cases: vec![ComparisonCase {
            case_id: "case-001".into(),
            initialization_time: 100,
            valid_time: 106,
            lead_seconds: 6,
            eligibility: CaseEligibility::Eligible,
            candidate: Availability::Available {
                artifact_sha256: GRID.into(),
                recorded_at: 150,
            },
            reference: Availability::Available {
                artifact_sha256: MASK.into(),
                recorded_at: 150,
            },
            verification: Availability::Available {
                artifact_sha256: VERIFY.into(),
                recorded_at: 240,
            },
        }],
    }
}

fn model(model_id: &str, digest: &str) -> ModelOutputManifest {
    ModelOutputManifest {
        model_id: model_id.into(),
        model_version: "v1".into(),
        archive_release: "archive-v1".into(),
        artifact_sha256: digest.into(),
        predictand_id: "t2m".into(),
        unit: "K".into(),
        temporal_support_id: "hourly".into(),
        spatial_support_id: "global-grid-v1".into(),
        grid_sha256: GRID.into(),
        mask_sha256: MASK.into(),
        climatology_id: "era5-clim-v1".into(),
        product_kind: ProductKind::Deterministic,
        ensemble_members: None,
        training_cutoff: Some(50),
        post_processing: None,
    }
}
