//! Matched-budget CART-style decision-tree tournament.
//!
//! This reference implementation is deterministic and depth-limited. It is not
//! asserted to match a vendor CART implementation or establish state of the art.
//! Receipts are JSONL and bind each run to the source and committed manifest SHA.

use cecb_counterexample_benchmark::discovery::discover_cart_tree;
use cecb_counterexample_benchmark::protocol::{
    build_model_family_split, build_protocol_split, FamilyTaggedPoint,
};
use cecb_counterexample_benchmark::{
    axis_aligned_landscape, nonlinear_interaction_landscape, oblique_landscape,
    structural_family_a, structural_family_b, LabelSource,
};

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

fn emit_shape_case(
    source_sha: &str,
    pr_head_sha: &str,
    manifest_sha: &str,
    case_id: &str,
    points: Vec<cecb_counterexample_benchmark::Point>,
    depth: usize,
) -> Result<(), String> {
    let budget = 41usize;
    let mut split = build_protocol_split(&points, LabelSource::LatentTruth, budget)
        .map_err(|error| format!("split failed for {case_id}: {error:?}"))?;
    let view = split.candidate_view().clone();
    let result = discover_cart_tree(
        &view,
        split.development_oracle_mut(),
        budget,
        depth,
    )
    .map_err(|error| format!("CART discovery failed for {case_id}: {error:?}"))?;
    let trusted_query_count = split.development_query_count();
    if result.development_observations_used != trusted_query_count
        || result.queried_sample_ids.as_slice() != split.queried_development_sample_ids()
    {
        return Err(format!(
            "candidate trace does not match trusted ledger for {case_id} depth {depth}"
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
            "\"schema_version\":\"cecb.cart_tree_receipt.v0.1\",",
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
            "\"max_depth\":{},",
            "\"label_query_budget\":{},",
            "\"label_queries_used\":{},",
            "\"queried_sample_ids\":[{}],",
            "\"development_errors\":{},",
            "\"rule_complexity\":{},",
            "\"development_point_count\":{},",
            "\"holdout_sample_count\":{},",
            "\"candidate_query_count\":{},",
            "\"tp\":{},\"fp\":{},\"fn\":{},\"tn\":{}",
            "}"
        ),
        source_sha,
        pr_head_sha,
        manifest_sha,
        view.manifest.split_rule,
        case_id,
        report.method_id,
        report.method_version,
        depth,
        budget,
        trusted_query_count,
        queried_ids,
        result.model.development_errors,
        result.model.complexity(),
        view.development.len(),
        report.holdout_sample_count,
        report.candidate_query_count,
        report.score.true_positive,
        report.score.false_positive,
        report.score.false_negative,
        report.score.true_negative,
    );
    Ok(())
}

fn emit_family_transfer(
    source_sha: &str,
    pr_head_sha: &str,
    manifest_sha: &str,
) -> Result<(), String> {
    let rows = family_fixture();
    let budget = 9usize;
    let depth = 3usize;
    let mut split = build_model_family_split(
        &rows,
        "family_b",
        LabelSource::LatentTruth,
        budget,
    )
    .map_err(|error| format!("family split failed: {error:?}"))?;
    let view = split.candidate_view().clone();
    let result = discover_cart_tree(
        &view,
        split.development_oracle_mut(),
        budget,
        depth,
    )
    .map_err(|error| format!("family CART discovery failed: {error:?}"))?;
    let trusted_query_count = split.development_query_count();
    if result.development_observations_used != trusted_query_count
        || result.queried_sample_ids.as_slice() != split.queried_development_sample_ids()
    {
        return Err("family-transfer trace does not match trusted ledger".to_owned());
    }
    let report = split
        .evaluate(&result.submission)
        .map_err(|error| format!("family evaluation failed: {error:?}"))?;
    let queried_ids = split
        .queried_development_sample_ids()
        .iter()
        .map(usize::to_string)
        .collect::<Vec<_>>()
        .join(",");

    println!(
        concat!(
            "{",
            "\"schema_version\":\"cecb.cart_tree_receipt.v0.1\",",
            "\"source_sha\":\"{}\",",
            "\"pr_head_sha\":\"{}\",",
            "\"split_manifest_sha256\":\"{}\",",
            "\"evaluator_version\":\"cecb.holdout_evaluator.v0.1\",",
            "\"random_seed\":null,",
            "\"observation_selection\":\"all_development_rows_in_source_order_v0.1\",",
            "\"split_rule\":\"{}\",",
            "\"case_id\":\"structural_family_reversal_a_to_b\",",
            "\"development_family\":\"family_a\",",
            "\"holdout_family\":\"family_b\",",
            "\"method_id\":\"{}\",",
            "\"method_version\":\"{}\",",
            "\"max_depth\":{},",
            "\"label_query_budget\":{},",
            "\"label_queries_used\":{},",
            "\"queried_sample_ids\":[{}],",
            "\"development_errors\":{},",
            "\"rule_complexity\":{},",
            "\"development_point_count\":{},",
            "\"holdout_sample_count\":{},",
            "\"candidate_query_count\":{},",
            "\"tp\":{},\"fp\":{},\"fn\":{},\"tn\":{}",
            "}"
        ),
        source_sha,
        pr_head_sha,
        manifest_sha,
        view.manifest.split_rule,
        report.method_id,
        report.method_version,
        depth,
        budget,
        trusted_query_count,
        queried_ids,
        result.model.development_errors,
        result.model.complexity(),
        view.development.len(),
        report.holdout_sample_count,
        report.candidate_query_count,
        report.score.true_positive,
        report.score.false_positive,
        report.score.false_negative,
        report.score.true_negative,
    );
    Ok(())
}

fn main() -> Result<(), String> {
    let source_sha = std::env::var("CECB_SOURCE_SHA")
        .unwrap_or_else(|_| "unavailable".to_owned());
    let pr_head_sha = std::env::var("CECB_PR_HEAD_SHA")
        .unwrap_or_else(|_| "unavailable".to_owned());
    let manifest_sha = std::env::var("CECB_SPLIT_MANIFEST_SHA256")
        .unwrap_or_else(|_| "unavailable".to_owned());

    let cases = vec![
        ("axis_aligned_2d", axis_aligned_landscape()),
        ("oblique_2d", oblique_landscape()),
        (
            "nonlinear_interaction_2d",
            nonlinear_interaction_landscape(),
        ),
    ];
    for (case_id, points) in cases {
        for depth in [1usize, 3] {
            emit_shape_case(
                &source_sha,
                &pr_head_sha,
                &manifest_sha,
                case_id,
                points.clone(),
                depth,
            )?;
        }
    }
    emit_family_transfer(&source_sha, &pr_head_sha, &manifest_sha)?;
    Ok(())
}
