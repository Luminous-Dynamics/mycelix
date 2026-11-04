//! Matched-budget comparison of a 1D/2D exhaustive box hypothesis class
//! across axis-aligned, oblique and nonlinear known-truth landscapes.
//!
//! This is a bounded reference learner, not PRIM, CART, or a SOTA claim.
//! Receipts bind the source checkout, PR head and committed split manifest.

use cecb_counterexample_benchmark::discovery::discover_exhaustive_axis_box;
use cecb_counterexample_benchmark::protocol::build_protocol_split;
use cecb_counterexample_benchmark::{
    axis_aligned_landscape, nonlinear_interaction_landscape, oblique_landscape, LabelSource,
};

fn main() -> Result<(), String> {
    let source_sha = std::env::var("CECB_SOURCE_SHA")
        .unwrap_or_else(|_| "unavailable".to_owned());
    let pr_head_sha = std::env::var("CECB_PR_HEAD_SHA")
        .unwrap_or_else(|_| "unavailable".to_owned());
    let split_manifest_sha256 = std::env::var("CECB_SPLIT_MANIFEST_SHA256")
        .unwrap_or_else(|_| "unavailable".to_owned());

    let cases = vec![
        ("axis_aligned_2d", axis_aligned_landscape()),
        ("oblique_2d", oblique_landscape()),
        ("nonlinear_interaction_2d", nonlinear_interaction_landscape()),
    ];
    let budget = 41usize;

    for (case_id, points) in cases {
        let mut split = build_protocol_split(&points, LabelSource::LatentTruth, budget)
            .map_err(|error| format!("split failed for {case_id}: {error:?}"))?;
        let candidate_view = split.candidate_view().clone();
        let result = discover_exhaustive_axis_box(
            &candidate_view,
            split.development_oracle_mut(),
            budget,
        )
        .map_err(|error| format!("box discovery failed for {case_id}: {error:?}"))?;

        let trusted_query_count = split.development_query_count();
        if result.development_observations_used != trusted_query_count
            || result.queried_sample_ids.as_slice() != split.queried_development_sample_ids()
        {
            return Err(format!(
                "candidate query trace does not match trusted ledger for {case_id}"
            ));
        }
        let report = split
            .evaluate(&result.submission)
            .map_err(|error| format!("evaluation failed for {case_id}: {error:?}"))?;
        let queried_ids = split
            .queried_development_sample_ids()
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(",");

        println!(
            concat!(
                "{",
                "\"schema_version\":\"cecb.box_discovery_receipt.v0.1\",",
                "\"source_sha\":\"{}\",",
                "\"pr_head_sha\":\"{}\",",
                "\"split_manifest_sha256\":\"{}\",",
                "\"evaluator_version\":\"cecb.holdout_evaluator.v0.1\",",
                "\"random_seed\":null,",
                "\"observation_selection\":\"all_development_rows_in_source_order_v0.1\",",
                "\"split_rule\":\"{}\",",
                "\"case_id\":\"{}\",",
                "\"method_id\":\"{}\",",
                "\"method_version\":\"{}\",",
                "\"label_query_budget\":{},",
                "\"label_queries_used\":{},",
                "\"queried_sample_ids\":[{}],",
                "\"development_errors\":{},",
                "\"development_fp\":{},",
                "\"development_fn\":{},",
                "\"rule_complexity\":{},",
                "\"development_point_count\":{},",
                "\"holdout_sample_count\":{},",
                "\"candidate_query_count\":{},",
                "\"tp\":{},\"fp\":{},\"fn\":{},\"tn\":{}",
                "}"
            ),
            source_sha,
            pr_head_sha,
            split_manifest_sha256,
            split.candidate_view().manifest.split_rule,
            case_id,
            report.method_id,
            report.method_version,
            budget,
            trusted_query_count,
            queried_ids,
            result.model.score.false_positive + result.model.score.false_negative,
            result.model.score.false_positive,
            result.model.score.false_negative,
            result.model.complexity,
            candidate_view.development.len(),
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
