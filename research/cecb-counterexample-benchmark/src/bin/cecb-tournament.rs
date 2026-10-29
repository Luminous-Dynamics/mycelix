//! Runs a deterministic, matched labelled-observation-budget transfer tournament.
//!
//! Output is JSONL so CI and downstream research tooling can retain immutable,
//! machine-readable evaluation receipts. These receipts are a synthetic baseline,
//! not a state-of-the-art comparison or real-world validation result.

use cecb_counterexample_benchmark::discovery::discover_univariate_threshold;
use cecb_counterexample_benchmark::protocol::{
    build_model_family_split, FamilyTaggedPoint,
};
use cecb_counterexample_benchmark::{structural_family_a, structural_family_b, LabelSource};

fn family_fixture() -> Vec<FamilyTaggedPoint> {
    let mut rows = structural_family_a()
        .into_iter()
        .map(|point| FamilyTaggedPoint {
            family_id: "family_a".to_owned(),
            point,
        })
        .collect::<Vec<_>>();
    rows.extend(
        structural_family_b()
            .into_iter()
            .map(|point| FamilyTaggedPoint {
                family_id: "family_b".to_owned(),
                point,
            }),
    );
    rows
}

fn main() -> Result<(), String> {
    let source_sha = std::env::var("CECB_SOURCE_SHA")
        .unwrap_or_else(|_| "unavailable".to_owned());
    let points = family_fixture();

    for budget in [3usize, 5, 9] {
        let mut split = build_model_family_split(
            &points,
            "family_b",
            LabelSource::LatentTruth,
            budget,
        )
        .map_err(|error| format!("split failed: {error:?}"))?;
        let candidate_view = split.candidate_view().clone();
        let result = discover_univariate_threshold(
            &candidate_view,
            split.development_oracle_mut(),
            budget,
        )
        .map_err(|error| format!("discovery failed: {error:?}"))?;
        let trusted_query_count = split.development_query_count();
        if result.queried_sample_ids.as_slice() != split.queried_development_sample_ids() {
            return Err("candidate trace does not match trusted query ledger".to_owned());
        }
        let report = split
            .evaluate(&result.submission, trusted_query_count)
            .map_err(|error| format!("evaluation failed: {error:?}"))?;

        let queried_ids = split
            .queried_development_sample_ids()
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(",");
        println!(
            concat!(
                "{",
                "\"schema_version\":\"cecb.discovery_receipt.v0.1\",",
                "\"source_sha\":\"{}\",",
                "\"split_rule\":\"{}\",",
                "\"development_family\":\"family_a\",",
                "\"holdout_family\":\"family_b\",",
                "\"method_id\":\"{}\",",
                "\"method_version\":\"{}\",",
                "\"observation_budget\":{},",
                "\"observations_used\":{},",
                "\"queried_sample_ids\":[{}],",
                "\"development_errors\":{},",
                "\"development_feasible_examples\":{},",
                "\"feature_index\":{},",
                "\"direction\":\"{}\",",
                "\"threshold\":{},",
                "\"holdout_sample_count\":{},",
                "\"candidate_query_count\":{},",
                "\"tp\":{},\"fp\":{},\"fn\":{},\"tn\":{}",
                "}"
            ),
            source_sha,
            split.candidate_view().manifest.split_rule,
            report.method_id,
            report.method_version,
            budget,
            trusted_query_count,
            queried_ids,
            result.model.development_errors,
            result.model.development_feasible_examples,
            result.model.feature_index,
            result.model.direction.as_str(),
            result.model.threshold,
            report.holdout_sample_count,
            report.candidate_query_count,
            report.score.true_positive,
            report.score.false_positive,
            report.score.false_negative,
            report.score.true_negative,
        );
    }

    Ok(())
}
