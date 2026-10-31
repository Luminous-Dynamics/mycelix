//! Dependence-cluster maximum split-conformal calibration for NRES-015.
//!
//! The calibration unit is a predeclared dependence cluster, not a source
//! family or row. Each cluster is summarized by the maximum NRES-014 block
//! W1 score. Split conformal is then applied to the exchangeable cluster-max
//! scores. This permits arbitrary dependence among blocks within a cluster
//! and gives simultaneous marginal coverage for every block in a future
//! same-design cluster, assuming cluster-max exchangeability.
//!
//! All clusters must have a fixed, predeclared block count. Too few clusters
//! produce an unbounded (null) radius. This module does not model dependence
//! between clusters, claim calibration-conditional coverage, or correct
//! arbitrary temporal drift. It is a synthetic public-development baseline.

use serde_json::{json, Value};
use std::collections::BTreeSet;

const BENCHMARK_ID: &str = "PRAXIS-NRES-015-DC1-CLUSTER-MAX-CONFORMAL-001";
const TOLERANCE: f64 = 1e-10;

#[derive(Clone)]
struct ClusterScore {
    cluster_id: String,
    block_scores: Vec<f64>,
}

impl ClusterScore {
    fn maximum(&self) -> f64 {
        self.block_scores.iter().copied().fold(0.0, f64::max)
    }
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn parse_clusters(
    raw: &[Value],
    group: &str,
    blocks_per_cluster: usize,
    cluster_ids: &mut BTreeSet<String>,
    family_ids: &mut BTreeSet<String>,
    block_ids: &mut BTreeSet<String>,
    errors: &mut Vec<String>,
) -> Vec<ClusterScore> {
    let mut parsed = Vec::new();
    for (cluster_index, cluster) in raw.iter().enumerate() {
        let cluster_id = string(cluster, "cluster_id").unwrap_or("").trim();
        if cluster_id.is_empty() {
            errors.push(format!("{group}[{cluster_index}]: cluster_id is missing"));
            continue;
        }
        if !cluster_ids.insert(cluster_id.to_owned()) {
            errors.push(format!("duplicate cluster_id across splits: {cluster_id}"));
        }
        let Some(blocks) = cluster.get("blocks").and_then(Value::as_array) else {
            errors.push(format!("{cluster_id}: blocks must be an array"));
            continue;
        };
        if blocks.len() != blocks_per_cluster {
            errors.push(format!(
                "{cluster_id}: expected exactly {blocks_per_cluster} blocks for fixed cluster design"
            ));
            continue;
        }

        let mut scores = Vec::with_capacity(blocks.len());
        let mut valid = true;
        for (block_index, block) in blocks.iter().enumerate() {
            let block_id = string(block, "block_id").unwrap_or("").trim();
            let family_id = string(block, "source_family_id").unwrap_or("").trim();
            if block_id.is_empty() || family_id.is_empty() {
                errors.push(format!("{cluster_id}[{block_index}]: block and source-family IDs are required"));
                valid = false;
                continue;
            }
            if !block_ids.insert(block_id.to_owned()) {
                errors.push(format!("duplicate block_id across splits: {block_id}"));
                valid = false;
            }
            if !family_ids.insert(family_id.to_owned()) {
                errors.push(format!("duplicate source_family_id across splits: {family_id}"));
                valid = false;
            }
            let score = block.get("w1_score").and_then(Value::as_f64);
            match score {
                Some(value) if value.is_finite() && value >= 0.0 => scores.push(value),
                _ => {
                    errors.push(format!("{cluster_id}[{block_index}]: w1_score must be finite and non-negative"));
                    valid = false;
                }
            }
        }
        if valid && scores.len() == blocks_per_cluster {
            parsed.push(ClusterScore {
                cluster_id: cluster_id.to_owned(),
                block_scores: scores,
            });
        }
    }
    parsed
}

/// Calibrate a simultaneous within-cluster threshold from exchangeable
/// dependence-cluster maxima of frozen NRES-014 block W1 scores.
pub fn evaluate_cluster_max_conformal(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    if string(fixture, "benchmark_id") != Some(BENCHMARK_ID) {
        errors.push("unexpected NRES-015-DC1 benchmark_id".to_owned());
    }
    if string(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-015-DC1 schema_version".to_owned());
    }
    if string(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("DC1 fixture must remain explicitly synthetic".to_owned());
    }
    if string(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("DC1 fixture must remain public-development-only".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("DC1 fixture must remain inactive in the curriculum".to_owned());
    }
    if string(fixture, "score_definition")
        != Some("NRES-014-block-W1-nonconformity-score")
    {
        errors.push("score_definition must identify the frozen NRES-014 block W1 score".to_owned());
    }

    let alpha = fixture.get("target_miscoverage").and_then(Value::as_f64);
    let Some(alpha) = alpha.filter(|value| value.is_finite() && *value > 0.0 && *value < 1.0) else {
        errors.push("target_miscoverage must be finite and strictly between zero and one".to_owned());
        return Err(errors);
    };
    let blocks_per_cluster = fixture
        .get("blocks_per_cluster")
        .and_then(Value::as_u64)
        .and_then(|value| usize::try_from(value).ok());
    let Some(blocks_per_cluster) =
        blocks_per_cluster.filter(|count| (1..=1024).contains(count))
    else {
        errors.push("blocks_per_cluster must be an integer in [1,1024]".to_owned());
        return Err(errors);
    };

    let Some(calibration_raw) = fixture
        .get("calibration_clusters")
        .and_then(Value::as_array)
    else {
        errors.push("calibration_clusters must be an array".to_owned());
        return Err(errors);
    };
    let Some(development_raw) = fixture
        .get("development_clusters")
        .and_then(Value::as_array)
    else {
        errors.push("development_clusters must be an array".to_owned());
        return Err(errors);
    };
    if calibration_raw.is_empty() {
        errors.push("at least one calibration cluster is required".to_owned());
    }
    if development_raw.is_empty() {
        errors.push("at least one development cluster is required".to_owned());
    }

    let mut cluster_ids = BTreeSet::new();
    let mut family_ids = BTreeSet::new();
    let mut block_ids = BTreeSet::new();
    let calibration = parse_clusters(
        calibration_raw,
        "calibration_clusters",
        blocks_per_cluster,
        &mut cluster_ids,
        &mut family_ids,
        &mut block_ids,
        &mut errors,
    );
    let development = parse_clusters(
        development_raw,
        "development_clusters",
        blocks_per_cluster,
        &mut cluster_ids,
        &mut family_ids,
        &mut block_ids,
        &mut errors,
    );
    if !errors.is_empty() {
        return Err(errors);
    }

    let mut calibration_scores: Vec<f64> =
        calibration.iter().map(ClusterScore::maximum).collect();
    calibration_scores.sort_by(f64::total_cmp);
    let calibration_count = calibration_scores.len();
    let rank = (((calibration_count + 1) as f64) * (1.0 - alpha)).ceil() as usize;
    let (radius, radius_unbounded) = if rank > calibration_count {
        (Value::Null, true)
    } else {
        (json!(calibration_scores[rank - 1]), false)
    };
    let radius_value = radius.as_f64();

    let mut covered_clusters = 0usize;
    let mut total_development_blocks = 0usize;
    let mut covered_blocks = 0usize;
    let mut development_reports = Vec::new();
    for cluster in &development {
        let cluster_score = cluster.maximum();
        let cluster_covered = radius_value
            .map(|threshold| cluster_score <= threshold + TOLERANCE)
            .unwrap_or(true);
        if cluster_covered {
            covered_clusters += 1;
        }
        let block_covered: Vec<bool> = cluster
            .block_scores
            .iter()
            .map(|score| {
                radius_value
                    .map(|threshold| *score <= threshold + TOLERANCE)
                    .unwrap_or(true)
            })
            .collect();
        covered_blocks += block_covered.iter().filter(|covered| **covered).count();
        total_development_blocks += block_covered.len();
        development_reports.push(json!({
            "cluster_id": cluster.cluster_id,
            "cluster_max_score": cluster_score,
            "simultaneous_cluster_covered": cluster_covered,
            "covered_block_count": block_covered.iter().filter(|covered| **covered).count(),
            "block_count": block_covered.len(),
            "block_coverage": block_covered
        }));
    }

    Ok(json!({
        "benchmark_id": BENCHMARK_ID,
        "method_id": "dependence-cluster-max-split-conformal",
        "status": "synthetic-public-development-evaluated",
        "evaluation_split": "public-development-only",
        "active_in_curriculum": false,
        "target_miscoverage": alpha,
        "target_marginal_coverage": 1.0 - alpha,
        "calibration_unit": "predeclared-dependence-cluster",
        "cluster_aggregation": "maximum NRES-014 block W1 score within each cluster",
        "blocks_per_cluster": blocks_per_cluster,
        "calibration_cluster_count": calibration_count,
        "calibration_block_count": calibration_count * blocks_per_cluster,
        "calibration_cluster_max_scores_sorted": calibration_scores,
        "conformal_rank_one_based": rank,
        "calibrated_cluster_max_radius": radius,
        "radius_unbounded_due_to_insufficient_clusters": radius_unbounded,
        "development_cluster_count": development.len(),
        "development_block_count": total_development_blocks,
        "development_cluster_coverage": {
            "numerator": covered_clusters,
            "denominator": development.len(),
            "value": covered_clusters as f64 / development.len() as f64
        },
        "development_block_coverage": {
            "numerator": covered_blocks,
            "denominator": total_development_blocks,
            "value": covered_blocks as f64 / total_development_blocks as f64
        },
        "development_clusters": development_reports,
        "guarantee_scope": "Under exchangeability of the fixed-design cluster-maximum scores, split conformal gives marginal 1-alpha coverage for a new cluster maximum, implying simultaneous marginal coverage of all blocks in a future same-design cluster.",
        "non_claims": [
            "No calibration-conditional coverage guarantee",
            "No population-law correction within a block",
            "No guarantee under dependence or shift between non-exchangeable clusters",
            "No guarantee for unseen cluster sizes or changed cluster construction",
            "No real-world or confirmatory result"
        ]
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn cluster(id: &str, scores: [f64; 3]) -> Value {
        json!({
            "cluster_id": id,
            "blocks": scores.iter().enumerate().map(|(i, score)| json!({
                "block_id": format!("{id}-B{i}"),
                "source_family_id": format!("{id}-F{i}"),
                "w1_score": score
            })).collect::<Vec<_>>()
        })
    }

    fn fixture(calibration: Vec<Value>, development: Vec<Value>, alpha: f64) -> Value {
        json!({
            "benchmark_id": BENCHMARK_ID,
            "schema_version": "1.0.0",
            "status": "synthetic-demonstration-for-review",
            "evaluation_split": "public-development-only",
            "active_in_curriculum": false,
            "score_definition": "NRES-014-block-W1-nonconformity-score",
            "target_miscoverage": alpha,
            "blocks_per_cluster": 3,
            "calibration_clusters": calibration,
            "development_clusters": development
        })
    }

    #[test]
    fn calibration_uses_cluster_max_not_family_row_count() {
        let calibration = vec![
            cluster("C1", [0.1, 0.2, 0.3]),
            cluster("C2", [0.4, 0.5, 0.1]),
            cluster("C3", [0.6, 0.1, 0.2]),
        ];
        let development = vec![cluster("D1", [0.1, 0.2, 0.3])];
        let report = evaluate_cluster_max_conformal(&fixture(calibration, development, 0.25)).unwrap();
        assert_eq!(report["calibration_cluster_count"], json!(3));
        assert_eq!(report["calibration_block_count"], json!(9));
        assert_eq!(report["calibration_cluster_max_scores_sorted"], json!([0.3, 0.5, 0.6]));
    }

    #[test]
    fn cluster_max_rank_is_the_conformal_sample_unit() {
        let calibration = vec![
            cluster("C1", [0.1, 0.4, 0.3]),
            cluster("C2", [0.3, 0.5, 0.45]),
            cluster("C3", [0.1, 0.6, 0.55]),
            cluster("C4", [0.25, 0.3, 0.2]),
            cluster("C5", [0.6, 0.8, 0.7]),
            cluster("C6", [0.45, 0.7, 0.65]),
            cluster("C7", [0.35, 0.55, 0.5]),
            cluster("C8", [0.5, 0.65, 0.6]),
            cluster("C9", [0.7, 0.9, 0.75]),
            cluster("C10", [0.75, 1.0, 0.85])
        ];
        let development = vec![cluster("D1", [0.3, 0.55, 0.7])];
        let report = evaluate_cluster_max_conformal(&fixture(calibration, development, 0.2)).unwrap();
        assert_eq!(report["conformal_rank_one_based"], json!(9));
        assert_eq!(report["calibrated_cluster_max_radius"], json!(0.9));
    }

    #[test]
    fn cluster_coverage_is_simultaneous_over_all_member_blocks() {
        let calibration = vec![
            cluster("C1", [0.1, 0.2, 0.3]),
            cluster("C2", [0.2, 0.3, 0.4]),
            cluster("C3", [0.3, 0.4, 0.5]),
            cluster("C4", [0.4, 0.5, 0.6])
        ];
        let development = vec![cluster("D1", [0.1, 0.2, 0.5]), cluster("D2", [0.1, 0.2, 0.7])];
        let report = evaluate_cluster_max_conformal(&fixture(calibration, development, 0.4)).unwrap();
        assert!(report["development_clusters"][0]["simultaneous_cluster_covered"].as_bool().unwrap());
        assert!(!report["development_clusters"][1]["simultaneous_cluster_covered"].as_bool().unwrap());
    }

    #[test]
    fn too_few_clusters_return_unbounded_radius() {
        let calibration = vec![cluster("C1", [0.1, 0.2, 0.3]), cluster("C2", [0.1, 0.2, 0.4])];
        let development = vec![cluster("D1", [100.0, 200.0, 300.0])];
        let report = evaluate_cluster_max_conformal(&fixture(calibration, development, 0.1)).unwrap();
        assert!(report["calibrated_cluster_max_radius"].is_null());
        assert_eq!(report["radius_unbounded_due_to_insufficient_clusters"], json!(true));
    }

    #[test]
    fn unbounded_radius_never_understates_development_coverage() {
        let calibration = vec![cluster("C1", [0.1, 0.2, 0.3])];
        let development = vec![cluster("D1", [10.0, 20.0, 30.0])];
        let report = evaluate_cluster_max_conformal(&fixture(calibration, development, 0.1)).unwrap();
        assert_eq!(report["development_cluster_coverage"]["value"], json!(1.0));
        assert_eq!(report["development_block_coverage"]["value"], json!(1.0));
    }

    #[test]
    fn duplicate_cluster_ids_across_splits_are_rejected() {
        let calibration = vec![cluster("C1", [0.1, 0.2, 0.3])];
        let development = vec![cluster("C1", [0.4, 0.5, 0.6])];
        assert!(evaluate_cluster_max_conformal(&fixture(calibration, development, 0.2)).is_err());
    }

    #[test]
    fn duplicate_source_families_are_rejected() {
        let mut calibration = vec![cluster("C1", [0.1, 0.2, 0.3]), cluster("C2", [0.4, 0.5, 0.6])];
        calibration[1]["blocks"][0]["source_family_id"] = json!("C1-F0");
        let development = vec![cluster("D1", [0.2, 0.3, 0.4])];
        assert!(evaluate_cluster_max_conformal(&fixture(calibration, development, 0.2)).is_err());
    }

    #[test]
    fn unequal_cluster_sizes_are_rejected() {
        let mut calibration = vec![cluster("C1", [0.1, 0.2, 0.3])];
        calibration[0]["blocks"].as_array_mut().unwrap().pop();
        let development = vec![cluster("D1", [0.2, 0.3, 0.4])];
        assert!(evaluate_cluster_max_conformal(&fixture(calibration, development, 0.2)).is_err());
    }

    #[test]
    fn negative_block_scores_are_rejected() {
        let calibration = vec![cluster("C1", [-0.1, 0.2, 0.3]), cluster("C2", [0.1, 0.2, 0.3])];
        let development = vec![cluster("D1", [0.2, 0.3, 0.4])];
        assert!(evaluate_cluster_max_conformal(&fixture(calibration, development, 0.2)).is_err());
    }

    #[test]
    fn invalid_alpha_is_rejected() {
        let calibration = vec![cluster("C1", [0.1, 0.2, 0.3])];
        let development = vec![cluster("D1", [0.2, 0.3, 0.4])];
        assert!(evaluate_cluster_max_conformal(&fixture(calibration, development, 0.0)).is_err());
    }

    #[test]
    fn active_curriculum_fixture_is_rejected() {
        let calibration = vec![cluster("C1", [0.1, 0.2, 0.3]), cluster("C2", [0.1, 0.2, 0.4])];
        let development = vec![cluster("D1", [0.2, 0.3, 0.4])];
        let mut input = fixture(calibration, development, 0.2);
        input["active_in_curriculum"] = json!(true);
        assert!(evaluate_cluster_max_conformal(&input).is_err());
    }

    #[test]
    fn scores_must_be_finite_and_non_negative() {
        let calibration = vec![cluster("C1", [f64::NAN, 0.2, 0.3]), cluster("C2", [0.1, 0.2, 0.3])];
        let development = vec![cluster("D1", [0.2, 0.3, 0.4])];
        let serialized = serde_json::to_value(calibration).unwrap_or(Value::Null);
        assert!(evaluate_cluster_max_conformal(&fixture(
            serialized.as_array().cloned().unwrap_or_default(),
            development,
            0.2
        )).is_err());
    }
}
