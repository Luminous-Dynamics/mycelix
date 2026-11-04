use cecb_counterexample_benchmark::{
    axis_aligned_landscape, correlated_feasible_landscape, exhaustive_axis_aligned_boxes,
    irrelevant_dimensions_landscape, nonlinear_interaction_landscape, noisy_boundary_landscape,
    oblique_landscape, pareto_box_front, score_predictions, score_rule, structural_family_a,
    structural_family_b, LabelSource, Point, Rule, Score,
};

fn fraction_json(value: Option<(usize, usize)>) -> String {
    match value {
        Some((numerator, denominator)) => {
            format!(r#"{{"numerator":{numerator},"denominator":{denominator}}}"#)
        }
        None => "null".to_owned(),
    }
}

fn emit_score(
    case_id: &str,
    rule_id: &str,
    points: &[Point],
    score: Score,
    label_source: &str,
) {
    let dimensions = points.first().map_or(0, |point| point.features.len());
    println!(
        concat!(
            "{{",
            "\"schema_version\":\"cecb.counterexample_score.v0.1\",",
            "\"case_id\":\"{}\",",
            "\"rule_id\":\"{}\",",
            "\"label_source\":\"{}\",",
            "\"point_count\":{},",
            "\"feature_count\":{},",
            "\"evaluated_point_count\":{},",
            "\"infeasible_skipped\":{},",
            "\"tp\":{},\"fp\":{},\"fn\":{},\"tn\":{},",
            "\"precision\":{},\"recall\":{}",
            "}}"
        ),
        case_id,
        rule_id,
        label_source,
        points.len(),
        dimensions,
        score.evaluated_points(),
        score.infeasible_skipped,
        score.true_positive,
        score.false_positive,
        score.false_negative,
        score.true_negative,
        fraction_json(score.precision_counts()),
        fraction_json(score.recall_counts()),
    );
}

fn emit_rule(
    case_id: &str,
    rule_id: &str,
    points: &[Point],
    rule: &Rule,
    source: LabelSource,
) -> Result<(), String> {
    let score = score_rule(points, rule, source).map_err(|error| format!("{error:?}"))?;
    let source_id = match source {
        LabelSource::LatentTruth => "latent_truth",
        LabelSource::ObservedLabel => "observed_label",
    };
    emit_score(case_id, rule_id, points, score, source_id);
    Ok(())
}

fn emit_latent_oracle(case_id: &str, points: &[Point]) -> Result<(), String> {
    let predictions = points.iter().map(|point| point.latent_failure).collect::<Vec<_>>();
    let score = score_predictions(points, &predictions, LabelSource::LatentTruth)
        .map_err(|error| format!("{error:?}"))?;
    emit_score(case_id, "oracle_latent_evaluator_only", points, score, "latent_truth");
    Ok(())
}

fn emit_pair(
    case_id: &str,
    points: &[Point],
    source: LabelSource,
) -> Result<(), String> {
    emit_rule(
        case_id,
        "axis_rule_x0_ge_2",
        points,
        &Rule::feature_at_least(0, 2),
        source,
    )?;
    emit_latent_oracle(case_id, points)
}

fn emit_box_search_summary(case_id: &str, points: &[Point]) -> Result<(), String> {
    let candidates = exhaustive_axis_aligned_boxes(points, LabelSource::LatentTruth)
        .map_err(|error| format!("{error:?}"))?;
    let pareto = pareto_box_front(&candidates);
    let maximum_true_positives = candidates
        .iter()
        .map(|candidate| candidate.score.true_positive)
        .max()
        .unwrap_or(0);
    let minimum_false_positives_at_maximum_coverage = candidates
        .iter()
        .filter(|candidate| candidate.score.true_positive == maximum_true_positives)
        .map(|candidate| candidate.score.false_positive)
        .min()
        .unwrap_or(0);
    let exact_recovery_exists = candidates.iter().any(|candidate| {
        candidate.score.false_positive == 0 && candidate.score.false_negative == 0
    });
    let dimensions = points.first().map_or(0, |point| point.features.len());
    println!(
        concat!(
            "{{",
            "\"schema_version\":\"cecb.counterexample_box_search.v0.1\",",
            "\"case_id\":\"{}\",",
            "\"search_scope\":\"exhaustive_axis_aligned_boxes\",",
            "\"label_source\":\"latent_truth\",",
            "\"point_count\":{},\"feature_count\":{},",
            "\"distinct_prediction_regions\":{},\"pareto_front_count\":{},",
            "\"maximum_true_positives\":{},",
            "\"minimum_false_positives_at_maximum_true_positives\":{},",
            "\"exact_recovery_exists\":{}",
            "}}"
        ),
        case_id,
        points.len(),
        dimensions,
        candidates.len(),
        pareto.len(),
        maximum_true_positives,
        minimum_false_positives_at_maximum_coverage,
        exact_recovery_exists,
    );
    Ok(())
}

fn main() -> Result<(), String> {
    let axis = axis_aligned_landscape();
    emit_pair("axis_aligned_2d", &axis, LabelSource::LatentTruth)?;
    emit_rule(
        "axis_aligned_2d",
        "naive_all_positive",
        &axis,
        &Rule::new(vec![]),
        LabelSource::LatentTruth,
    )?;

    let oblique = oblique_landscape();
    emit_pair("oblique_2d", &oblique, LabelSource::LatentTruth)?;

    let nonlinear = nonlinear_interaction_landscape();
    emit_pair("nonlinear_interaction_2d", &nonlinear, LabelSource::LatentTruth)?;

    let noisy = noisy_boundary_landscape();
    emit_rule(
        "noisy_boundary_2d",
        "axis_rule_x0_ge_2",
        &noisy,
        &Rule::feature_at_least(0, 2),
        LabelSource::ObservedLabel,
    )?;
    emit_rule(
        "noisy_boundary_2d",
        "axis_rule_x0_ge_2",
        &noisy,
        &Rule::feature_at_least(0, 2),
        LabelSource::LatentTruth,
    )?;
    emit_latent_oracle("noisy_boundary_2d", &noisy)?;

    let irrelevant = irrelevant_dimensions_landscape();
    emit_pair("irrelevant_dimensions_8d", &irrelevant, LabelSource::LatentTruth)?;

    let constrained = correlated_feasible_landscape();
    emit_pair("correlated_feasible_2d", &constrained, LabelSource::LatentTruth)?;

    let family_a = structural_family_a();
    emit_pair("structural_family_a", &family_a, LabelSource::LatentTruth)?;
    let family_b = structural_family_b();
    emit_pair("structural_family_b", &family_b, LabelSource::LatentTruth)?;

    // Exhaustive search is deliberately limited to small spaces. These summary
    // rows quantify the best axis-aligned box frontier against known truth.
    emit_box_search_summary("axis_aligned_2d", &axis)?;
    emit_box_search_summary("oblique_2d", &oblique)?;
    emit_box_search_summary("nonlinear_interaction_2d", &nonlinear)?;

    Ok(())
}
