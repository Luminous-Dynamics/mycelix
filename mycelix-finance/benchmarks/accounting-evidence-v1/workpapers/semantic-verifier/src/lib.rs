#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const DAG_DOMAIN: &[u8] = b"mycelix:fin-acc-workpaper-dag:v0.1\0";
const NODE_DOMAIN: &[u8] = b"mycelix:fin-acc-workpaper:v0.1\0";

const FRONTIER_KEYS: &[&str] = &[
    "source_scope_manifest",
    "evidence_frontier",
    "policy_snapshot",
    "account_taxonomy",
    "authority_frontier",
    "knowledge_frontier",
    "validator_build",
];

const REQUIRED_READY_FRONTIERS: &[&str] = &[
    "source_scope_manifest",
    "evidence_frontier",
    "policy_snapshot",
    "account_taxonomy",
    "authority_frontier",
];

const AUTHORITY_SENSITIVE_KINDS: &[&str] = &[
    "journal_projection",
    "trial_balance_reconstruction",
    "financial_statement_projection",
    "independent_verification",
    "taxonomy_validation",
    "human_review",
    "close_readiness",
];

pub fn verify(input_bytes: &[u8], schema_bytes: &[u8]) -> Value {
    let input: Value = match serde_json::from_slice(input_bytes) {
        Ok(value) => value,
        Err(error) => return failure_report("invalid_input", format!("candidate JSON: {error}")),
    };
    let schema: Value = match serde_json::from_slice(schema_bytes) {
        Ok(value) => value,
        Err(error) => return failure_report("invalid_schema", format!("schema JSON: {error}")),
    };
    if let Err(error) = jsonschema::draft202012::meta::validate(&schema) {
        return failure_report("invalid_schema", format!("Draft 2020-12 schema: {error}"));
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(value) => value,
        Err(error) => {
            return failure_report("invalid_schema", format!("schema compilation: {error}"));
        }
    };
    let schema_errors = validator
        .iter_errors(&input)
        .map(|error| format!("{error} at {}", error.instance_path()))
        .collect::<Vec<_>>();
    if !schema_errors.is_empty() {
        let mut report = failure_report(
            "invalid_input",
            "candidate does not satisfy the pinned workpaper DAG schema".to_owned(),
        );
        report["errors"] = json!(schema_errors);
        return validate_report(report);
    }

    validate_semantics(&input, input_bytes, schema_bytes)
}

fn validate_semantics(input: &Value, input_bytes: &[u8], schema_bytes: &[u8]) -> Value {
    let mut errors = BTreeSet::<String>::new();
    let mut warnings = BTreeSet::<String>::new();
    let nodes = input["nodes"].as_array().cloned().unwrap_or_default();
    let mut node_map = BTreeMap::<String, Value>::new();

    for node in &nodes {
        let id = string(node, "node_id").unwrap_or_default().to_owned();
        if node_map.insert(id.clone(), node.clone()).is_some() {
            errors.insert(format!("duplicate_node_id:{id}"));
        }
    }

    let mut parents = BTreeMap::<String, BTreeSet<String>>::new();
    let mut children = BTreeMap::<String, BTreeSet<String>>::new();
    for id in node_map.keys() {
        parents.insert(id.clone(), BTreeSet::new());
        children.insert(id.clone(), BTreeSet::new());
    }

    for (id, node) in &node_map {
        let mut local = BTreeSet::new();
        for parent_value in array(node, "depends_on") {
            let parent = parent_value.as_str().unwrap_or_default().to_owned();
            if parent == *id {
                errors.insert(format!("self_dependency:{id}"));
                continue;
            }
            if !local.insert(parent.clone()) {
                errors.insert(format!("duplicate_dependency:{id}:{parent}"));
                continue;
            }
            if !node_map.contains_key(&parent) {
                errors.insert(format!("dangling_dependency:{id}:{parent}"));
                continue;
            }
            parents
                .entry(id.clone())
                .or_default()
                .insert(parent.clone());
            children.entry(parent).or_default().insert(id.clone());
        }
    }

    let (order, cycle) = topological_order(&node_map, &parents, &children);
    if !cycle.is_empty() {
        errors.insert(format!("dependency_cycle:{}", cycle.join("->")));
    }

    let dag_version = string(input, "dag_version").unwrap_or_default();
    validate_workpaper_commitments(&node_map, &mut errors);
    validate_parent_inputs(&node_map, &parents, dag_version, &mut errors);
    validate_required_frontier_bindings(&node_map, input, &mut errors);
    validate_node_authority(&node_map, &mut errors);
    validate_recipes(&node_map, &mut errors);
    validate_numeric_values(&node_map, &mut errors);

    let (direct_stale, stale_closure) = stale_frontier_closure(input, &node_map, &children);
    for id in &stale_closure {
        if let Some(node) = node_map.get(id) {
            let state = string(&node["result"], "state").unwrap_or_default();
            if state != "not_started" && state != "invalidated_stale" {
                errors.insert(format!("stale_node_not_invalidated:{id}:{state}"));
            }
        }
    }

    let required_kinds = string_set(&input["required_node_kinds"]);
    let present_required = node_map
        .values()
        .filter(|node| node["required_in_close"] == true)
        .filter_map(|node| string(node, "node_kind").map(str::to_owned))
        .collect::<BTreeSet<_>>();
    let missing_kinds = required_kinds
        .difference(&present_required)
        .cloned()
        .collect::<BTreeSet<_>>();
    if !missing_kinds.is_empty() {
        warnings.insert(format!(
            "required_node_kinds_missing:{}",
            missing_kinds.iter().cloned().collect::<Vec<_>>().join(",")
        ));
    }

    validate_required_reviews(&node_map, dag_version, &mut errors);
    validate_ready_disposition(
        input,
        &node_map,
        &required_kinds,
        &missing_kinds,
        &stale_closure,
        &mut errors,
    );

    let authority_escalation_detected = errors.iter().any(|reason| {
        reason.starts_with("non_authoritative_node_escalates_scope")
            || reason.starts_with("advisory_system_claims_authority")
            || reason.starts_with("human_decision_owner_kind_mismatch")
            || reason.starts_with("journal_projection_authority_mismatch")
            || reason.starts_with("independent_verification_authority_mismatch")
            || reason.starts_with("human_review_authority_mismatch")
    });
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let canonical_dag = canonical_dag_value(input);
    let graph_digest = canonical_digest(DAG_DOMAIN, &canonical_dag);
    let node_order = if cycle.is_empty() { order } else { Vec::new() };
    let directly_stale = direct_stale.iter().cloned().collect::<Vec<_>>();
    let transitively_stale = stale_closure
        .difference(&direct_stale)
        .cloned()
        .collect::<Vec<_>>();
    let invalidated = stale_closure
        .iter()
        .filter(|id| {
            node_map
                .get(*id)
                .is_some_and(|node| string(&node["result"], "state") == Some("invalidated_stale"))
        })
        .cloned()
        .collect::<Vec<_>>();

    let report = json!({
        "report_schema": "fin-acc-workpaper-dag-report.v0.1",
        "benchmark_id": input.get("dag_id"),
        "dag_version": input.get("dag_version"),
        "status": status,
        "close_disposition": input.get("close_disposition"),
        "claim_scope": "semantic_consistency_of_declared_synthetic_workpaper_dag_only",
        "digests": {
            "dag_sha256": sha256_hex(input_bytes),
            "canonical_dag_sha256": graph_digest,
            "schema_sha256": sha256_hex(schema_bytes),
            "verifier_source_sha256": sha256_hex(include_bytes!("lib.rs"))
        },
        "node_count": node_map.len(),
        "topological_order": node_order,
        "missing_required_node_kinds": missing_kinds,
        "directly_stale_nodes": directly_stale,
        "transitively_stale_nodes": transitively_stale,
        "invalidated_stale_nodes": invalidated,
        "errors": errors.into_iter().collect::<Vec<_>>(),
        "warnings": warnings.into_iter().collect::<Vec<_>>(),
        "authority_escalation_detected": authority_escalation_detected,
        "can_post_journals": false,
        "can_approve_or_close": false,
        "limitations": limitations()
    });
    validate_report(report)
}

fn validate_report(mut report: Value) -> Value {
    let schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(value) => value,
        Err(error) => {
            return failure_report("invalid_report", format!("embedded report schema: {error}"));
        }
    };
    if let Err(error) = jsonschema::draft202012::meta::validate(&schema) {
        return failure_report(
            "invalid_report",
            format!("embedded report schema meta-validation: {error}"),
        );
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(value) => value,
        Err(error) => {
            return failure_report(
                "invalid_report",
                format!("embedded report schema compilation: {error}"),
            );
        }
    };
    let issues = validator
        .iter_errors(&report)
        .map(|error| format!("{error} at {}", error.instance_path()))
        .collect::<Vec<_>>();
    if !issues.is_empty() {
        report["status"] = Value::String("FAIL".to_owned());
        if let Some(errors) = report["errors"].as_array_mut() {
            errors.extend(issues.into_iter().map(Value::String));
        }
    }
    report
}

fn failure_report(disposition: &str, error: String) -> Value {
    json!({
        "report_schema": "fin-acc-workpaper-dag-report.v0.1",
        "benchmark_id": Value::Null,
        "dag_version": Value::Null,
        "status": "FAIL",
        "close_disposition": "not_ready",
        "claim_scope": "semantic_consistency_of_declared_synthetic_workpaper_dag_only",
        "digests": {
            "dag_sha256": Value::Null,
            "canonical_dag_sha256": Value::Null,
            "schema_sha256": Value::Null,
            "verifier_source_sha256": sha256_hex(include_bytes!("lib.rs"))
        },
        "node_count": 0,
        "topological_order": [],
        "missing_required_node_kinds": [],
        "directly_stale_nodes": [],
        "transitively_stale_nodes": [],
        "invalidated_stale_nodes": [],
        "errors": [format!("{disposition}:{error}")],
        "warnings": [],
        "authority_escalation_detected": false,
        "can_post_journals": false,
        "can_approve_or_close": false,
        "limitations": limitations()
    })
}

fn topological_order(
    nodes: &BTreeMap<String, Value>,
    parents: &BTreeMap<String, BTreeSet<String>>,
    children: &BTreeMap<String, BTreeSet<String>>,
) -> (Vec<String>, Vec<String>) {
    let mut indegree = parents
        .iter()
        .map(|(id, entries)| (id.clone(), entries.len()))
        .collect::<BTreeMap<_, _>>();
    let mut ready = indegree
        .iter()
        .filter(|(_, degree)| **degree == 0)
        .map(|(id, _)| id.clone())
        .collect::<BTreeSet<_>>();
    let mut order = Vec::with_capacity(nodes.len());
    while let Some(id) = ready.pop_first() {
        order.push(id.clone());
        for child in children.get(&id).into_iter().flatten() {
            if let Some(degree) = indegree.get_mut(child) {
                *degree = degree.saturating_sub(1);
                if *degree == 0 {
                    ready.insert(child.clone());
                }
            }
        }
    }
    if order.len() == nodes.len() {
        (order, Vec::new())
    } else {
        let remaining = indegree
            .into_iter()
            .filter(|(_, degree)| *degree > 0)
            .map(|(id, _)| id)
            .collect::<Vec<_>>();
        let cycle = find_cycle(parents, &remaining).unwrap_or(remaining);
        (order, cycle)
    }
}

fn find_cycle(
    parents: &BTreeMap<String, BTreeSet<String>>,
    remaining: &[String],
) -> Option<Vec<String>> {
    fn visit(
        id: &str,
        parents: &BTreeMap<String, BTreeSet<String>>,
        allowed: &BTreeSet<String>,
        visiting: &mut Vec<String>,
        visited: &mut BTreeSet<String>,
    ) -> Option<Vec<String>> {
        if let Some(index) = visiting.iter().position(|item| item == id) {
            let mut cycle = visiting[index..].to_vec();
            cycle.push(id.to_owned());
            return Some(cycle);
        }
        if visited.contains(id) {
            return None;
        }
        visiting.push(id.to_owned());
        for parent in parents.get(id).into_iter().flatten() {
            if allowed.contains(parent) {
                if let Some(found) = visit(parent, parents, allowed, visiting, visited) {
                    return Some(found);
                }
            }
        }
        visiting.pop();
        visited.insert(id.to_owned());
        None
    }

    let allowed = remaining.iter().cloned().collect::<BTreeSet<_>>();
    let mut visited = BTreeSet::new();
    for id in remaining {
        if let Some(cycle) = visit(id, parents, &allowed, &mut Vec::new(), &mut visited) {
            return Some(cycle);
        }
    }
    None
}

fn validate_workpaper_commitments(nodes: &BTreeMap<String, Value>, errors: &mut BTreeSet<String>) {
    for (id, node) in nodes {
        if string(&node["result"], "state") != Some("completed") {
            continue;
        }
        let declared = string(&node["result"], "workpaper_commitment_sha256");
        let computed = workpaper_commitment(node);
        if declared != Some(computed.as_str()) {
            errors.insert(format!("workpaper_commitment_mismatch:{id}"));
        }
    }
}

fn workpaper_commitment(node: &Value) -> String {
    let mut canonical_node = node.clone();
    if let Some(result) = canonical_node
        .get_mut("result")
        .and_then(Value::as_object_mut)
    {
        result.remove("workpaper_commitment_sha256");
    }
    canonical_digest(NODE_DOMAIN, &canonical_node)
}

fn validate_parent_inputs(
    nodes: &BTreeMap<String, Value>,
    parents: &BTreeMap<String, BTreeSet<String>>,
    dag_version: &str,
    errors: &mut BTreeSet<String>,
) {
    for (id, node) in nodes {
        let expected = parents.get(id).cloned().unwrap_or_default();
        let prior_inputs = array(node, "inputs")
            .iter()
            .filter(|input| string(input, "purpose") == Some("prior_workpaper"))
            .collect::<Vec<_>>();
        let mut observed = BTreeSet::<String>::new();
        for input in &prior_inputs {
            let parent_id = string(&input["input_ref"], "ref_id")
                .unwrap_or_default()
                .to_owned();
            if !observed.insert(parent_id.clone()) {
                errors.insert(format!("duplicate_prior_workpaper_input:{id}:{parent_id}"));
            }
            if !expected.contains(&parent_id) {
                errors.insert(format!(
                    "prior_workpaper_input_without_edge:{id}:{parent_id}"
                ));
            }
            if !nodes.contains_key(&parent_id) {
                errors.insert(format!("prior_workpaper_parent_missing:{id}:{parent_id}"));
            }
            if string(&input["input_ref"], "version") != Some(dag_version) {
                errors.insert(format!("prior_workpaper_version_mismatch:{id}:{parent_id}"));
            }
        }
        for parent_id in expected.difference(&observed) {
            errors.insert(format!(
                "dependency_without_prior_workpaper_input:{id}:{parent_id}"
            ));
        }
        for input in prior_inputs {
            let parent_id = string(&input["input_ref"], "ref_id").unwrap_or_default();
            if let Some(parent) = nodes.get(parent_id) {
                let parent_state = string(&parent["result"], "state").unwrap_or_default();
                let child_state = string(&node["result"], "state").unwrap_or_default();
                let input_digest = string(&input["input_ref"], "sha256");
                let parent_digest = string(&parent["result"], "workpaper_commitment_sha256");
                if parent_state == "completed" {
                    if input_digest != parent_digest {
                        errors.insert(format!("parent_commitment_mismatch:{id}:{parent_id}"));
                    }
                } else if child_state == "completed" {
                    errors.insert(format!(
                        "completed_child_has_nonterminal_parent:{id}:{parent_id}"
                    ));
                } else if input_digest.is_some() {
                    errors.insert(format!(
                        "nonterminal_parent_claimed_as_completed:{id}:{parent_id}"
                    ));
                }
            }
        }
    }
}

fn canonical_dag_value(input: &Value) -> Value {
    let mut value = input.clone();
    if let Some(nodes) = value.get_mut("nodes").and_then(Value::as_array_mut) {
        nodes.sort_by(|left, right| string(left, "node_id").cmp(&string(right, "node_id")));
    }
    if let Some(kinds) = value
        .get_mut("required_node_kinds")
        .and_then(Value::as_array_mut)
    {
        kinds.sort_by(|left, right| left.as_str().cmp(&right.as_str()));
    }
    value
}

fn validate_node_authority(nodes: &BTreeMap<String, Value>, errors: &mut BTreeSet<String>) {
    for (id, node) in nodes {
        let kind = string(node, "node_kind").unwrap_or_default();
        let authority = string(node, "authority_class").unwrap_or_default();
        let owner = string(node, "owner_system").unwrap_or_default();

        if matches!(
            authority,
            "candidate_analysis" | "simulation_non_authoritative"
        ) && AUTHORITY_SENSITIVE_KINDS.contains(&kind)
        {
            errors.insert(format!(
                "non_authoritative_node_escalates_scope:{id}:{authority}:{kind}"
            ));
        }
        if matches!(owner, "symthaea" | "symtropy")
            && matches!(
                authority,
                "deterministic_financial_projection"
                    | "human_authorized_decision"
                    | "independent_verification"
            )
        {
            errors.insert(format!(
                "advisory_system_claims_authority:{id}:{owner}:{authority}"
            ));
        }
        if authority == "human_authorized_decision"
            && !(owner == "human_reviewer" && kind == "human_review")
        {
            errors.insert(format!("human_decision_owner_kind_mismatch:{id}"));
        }
        if kind == "journal_projection"
            && !(owner == "mycelix" && authority == "deterministic_financial_projection")
        {
            errors.insert(format!("journal_projection_authority_mismatch:{id}"));
        }
        if kind == "independent_verification"
            && !(owner == "mycelix" && authority == "independent_verification")
        {
            errors.insert(format!("independent_verification_authority_mismatch:{id}"));
        }
        if kind == "human_review"
            && !(owner == "human_reviewer" && authority == "human_authorized_decision")
        {
            errors.insert(format!("human_review_authority_mismatch:{id}"));
        }
    }
}

fn validate_required_reviews(
    nodes: &BTreeMap<String, Value>,
    dag_version: &str,
    errors: &mut BTreeSet<String>,
) {
    for (id, node) in nodes {
        if node["review_requirement"]["required"] != true
            || string(&node["result"], "state") != Some("completed")
        {
            continue;
        }
        let parent_commitment = string(&node["result"], "workpaper_commitment_sha256");
        let review_nodes = nodes
            .values()
            .filter(|candidate| {
                string(candidate, "node_kind") == Some("human_review")
                    && array(candidate, "depends_on")
                        .iter()
                        .any(|parent| parent.as_str() == Some(id))
                    && array(candidate, "inputs").iter().any(|input| {
                        string(input, "purpose") == Some("review_evidence")
                            && string(&input["input_ref"], "ref_id") == Some(id)
                    })
            })
            .collect::<Vec<_>>();
        if review_nodes.is_empty() {
            errors.insert(format!("required_human_review_node_missing:{id}"));
            continue;
        }
        let has_bound_completed_review = review_nodes.iter().any(|review| {
            let review_input = array(review, "inputs").iter().find(|input| {
                string(input, "purpose") == Some("review_evidence")
                    && string(&input["input_ref"], "ref_id") == Some(id)
            });
            let exact_binding = review_input.is_some_and(|input| {
                string(&input["input_ref"], "version") == Some(dag_version)
                    && string(&input["input_ref"], "sha256") == parent_commitment
            });
            exact_binding
                && string(&review["result"], "state") == Some("completed")
                && string(&review["result"], "terminal_disposition").is_some_and(|state| {
                    matches!(
                        state,
                        "completed_and_verified" | "completed_with_scope_limitations"
                    )
                })
        });
        if !has_bound_completed_review {
            errors.insert(format!(
                "completed_workpaper_missing_exact_completed_review:{id}"
            ));
        }
    }
}

fn validate_recipes(nodes: &BTreeMap<String, Value>, errors: &mut BTreeSet<String>) {
    for (id, node) in nodes {
        let Some(recipe) = node.get("calculation_recipe") else {
            continue;
        };
        let operation = string(recipe, "operation").unwrap_or_default();
        let inputs = array(recipe, "input_refs")
            .iter()
            .filter_map(|value| string(value, "ref_id"))
            .collect::<BTreeSet<_>>();

        for required in ["numerator", "denominator"] {
            if let Some(aggregation) = recipe.get(required) {
                let ref_id = string(&aggregation["input_ref"], "ref_id");
                if ref_id.is_none_or(|value| !inputs.contains(value)) {
                    errors.insert(format!("recipe_{required}_input_not_declared:{id}"));
                }
            }
        }
        if operation == "ratio" {
            if recipe.get("numerator").is_none() || recipe.get("denominator").is_none() {
                errors.insert(format!("ratio_missing_named_numerator_or_denominator:{id}"));
            }
            if string(recipe, "zero_denominator_disposition").is_none() {
                errors.insert(format!("ratio_missing_zero_denominator_disposition:{id}"));
            }
        }

        let join = string(recipe, "join_semantics").unwrap_or_default();
        let expected_cardinality = string(recipe, "expected_cardinality").unwrap_or_default();
        if join == "one_to_one" && !matches!(expected_cardinality, "one" | "zero_or_one") {
            errors.insert(format!("one_to_one_join_cardinality_mismatch:{id}"));
        }
        if join == "one_to_many_with_explicit_aggregation" && expected_cardinality != "one_to_many"
        {
            errors.insert(format!("one_to_many_join_cardinality_mismatch:{id}"));
        }
        if join == "many_to_many_with_explicit_aggregation"
            && expected_cardinality != "many_to_many"
        {
            errors.insert(format!("many_to_many_join_cardinality_mismatch:{id}"));
        }
        if join != "none" && array(recipe, "join_keys").is_empty() {
            errors.insert(format!("join_keys_missing:{id}"));
        }
        if matches!(
            join,
            "one_to_many_with_explicit_aggregation" | "many_to_many_with_explicit_aggregation"
        ) {
            let explicit_aggregation = matches!(
                operation,
                "ratio" | "weighted_average" | "grouped_aggregation"
            );
            if !explicit_aggregation
                || array(recipe, "join_keys").is_empty()
                || expected_cardinality == "unknown"
            {
                errors.insert(format!("nontrivial_join_without_explicit_aggregation:{id}"));
            }
            if operation == "grouped_aggregation" && array(recipe, "group_by").is_empty() {
                errors.insert(format!("grouped_join_missing_group_by:{id}"));
            }
        }
        if matches!(
            join,
            "one_to_many_with_explicit_aggregation" | "many_to_many_with_explicit_aggregation"
        ) && expected_cardinality.is_empty()
        {
            errors.insert(format!("join_expected_cardinality_missing:{id}"));
        }
        if let (Some(start), Some(end)) =
            (string(recipe, "period_start"), string(recipe, "period_end"))
        {
            if start > end {
                errors.insert(format!("recipe_period_reversed:{id}"));
            }
            if string(recipe, "cutoff_semantics").is_none() {
                errors.insert(format!("recipe_cutoff_semantics_missing:{id}"));
            }
        }
        if string(recipe, "null_semantics") == Some("profile_defined")
            && recipe.get("implementation_ref").is_none()
        {
            errors.insert(format!(
                "profile_defined_nulls_without_pinned_implementation:{id}"
            ));
        }
    }
}

fn validate_numeric_values(nodes: &BTreeMap<String, Value>, errors: &mut BTreeSet<String>) {
    for (id, node) in nodes {
        let result = &node["result"];
        let Some(numeric) = result.get("numeric_value") else {
            continue;
        };
        let encoding = string(numeric, "encoding").unwrap_or_default();
        if result.get("unit_profile").is_some()
            && result
                .get("numeric_value")
                .and_then(|value| value.get("unit_profile"))
                .is_some()
            && numeric["unit_profile"] != result["unit_profile"]
        {
            errors.insert(format!("numeric_result_unit_profile_mismatch:{id}"));
        }
        match encoding {
            "integer_minor_units" => {
                let value = string(numeric, "value").unwrap_or_default();
                if !canonical_integer(value) {
                    errors.insert(format!("noncanonical_integer_minor_units:{id}"));
                }
            }
            "fixed_decimal" => {
                let value = string(numeric, "value").unwrap_or_default();
                let scale = numeric["scale"].as_u64().unwrap_or(u64::MAX) as usize;
                if !canonical_decimal(value) || decimal_scale(value) != Some(scale) {
                    errors.insert(format!("fixed_decimal_scale_or_format_mismatch:{id}"));
                }
            }
            "exact_rational" => {
                let numerator = string(numeric, "numerator").unwrap_or_default();
                let denominator = string(numeric, "denominator").unwrap_or_default();
                if !canonical_integer(numerator) || !positive_integer(denominator) {
                    errors.insert(format!("noncanonical_exact_rational:{id}"));
                }
            }
            _ => errors.insert(format!("unsupported_numeric_encoding:{id}")),
        }
    }
}

fn required_frontier_keys(node_kind: &str) -> &'static [&'static str] {
    match node_kind {
        "source_inventory" => &["source_scope_manifest", "knowledge_frontier"],
        "evidence_population_qualification" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "knowledge_frontier",
            "validator_build",
        ],
        "assertion_workpaper" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "knowledge_frontier",
        ],
        "reconciliation_workpaper" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "account_taxonomy",
            "knowledge_frontier",
        ],
        "accounting_projection_candidate" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "policy_snapshot",
            "account_taxonomy",
            "knowledge_frontier",
        ],
        "journal_projection" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "policy_snapshot",
            "account_taxonomy",
            "authority_frontier",
            "knowledge_frontier",
            "validator_build",
        ],
        "trial_balance_reconstruction" | "financial_statement_projection" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "policy_snapshot",
            "account_taxonomy",
            "authority_frontier",
            "knowledge_frontier",
            "validator_build",
        ],
        "independent_verification" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "policy_snapshot",
            "account_taxonomy",
            "authority_frontier",
            "knowledge_frontier",
            "validator_build",
        ],
        "taxonomy_validation" => &[
            "source_scope_manifest",
            "policy_snapshot",
            "account_taxonomy",
            "authority_frontier",
            "knowledge_frontier",
            "validator_build",
        ],
        "human_review" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "authority_frontier",
            "knowledge_frontier",
        ],
        "close_readiness" => &[
            "source_scope_manifest",
            "evidence_frontier",
            "policy_snapshot",
            "account_taxonomy",
            "authority_frontier",
            "knowledge_frontier",
            "validator_build",
        ],
        _ => &[],
    }
}

fn validate_required_frontier_bindings(
    nodes: &BTreeMap<String, Value>,
    input: &Value,
    errors: &mut BTreeSet<String>,
) {
    for (id, node) in nodes {
        let kind = string(node, "node_kind").unwrap_or_default();
        for key in required_frontier_keys(kind) {
            if node["frontier_bindings"].get(*key).is_none() {
                errors.insert(format!("required_frontier_binding_missing:{id}:{key}"));
            }
            if input["frontiers"].get(*key).is_none() {
                errors.insert(format!("required_dag_frontier_missing:{id}:{key}"));
            }
        }
    }
}

fn stale_frontier_closure(
    input: &Value,
    nodes: &BTreeMap<String, Value>,
    children: &BTreeMap<String, BTreeSet<String>>,
) -> (BTreeSet<String>, BTreeSet<String>) {
    let mut direct = BTreeSet::new();
    let dag_frontiers = &input["frontiers"];
    for (id, node) in nodes {
        let bindings = &node["frontier_bindings"];
        let required = required_frontier_keys(string(node, "node_kind").unwrap_or_default());
        for key in FRONTIER_KEYS {
            let Some(bound) = bindings.get(*key) else {
                if required.contains(key) {
                    direct.insert(id.clone());
                }
                continue;
            };
            let Some(current) = dag_frontiers.get(*key) else {
                direct.insert(id.clone());
                continue;
            };
            if bound != current {
                direct.insert(id.clone());
            }
        }
    }
    let mut closure = direct.clone();
    let mut queue = direct.iter().cloned().collect::<Vec<_>>();
    while let Some(id) = queue.pop() {
        for child in children.get(&id).into_iter().flatten() {
            if closure.insert(child.clone()) {
                queue.push(child.clone());
            }
        }
    }
    (direct, closure)
}

fn validate_ready_disposition(
    input: &Value,
    nodes: &BTreeMap<String, Value>,
    required_kinds: &BTreeSet<String>,
    missing_kinds: &BTreeSet<String>,
    stale_nodes: &BTreeSet<String>,
    errors: &mut BTreeSet<String>,
) {
    let disposition = string(input, "close_disposition").unwrap_or_default();
    if !matches!(
        disposition,
        "ready_for_authorized_signoff" | "ready_with_scope_limitations"
    ) {
        return;
    }

    if !missing_kinds.is_empty() {
        errors.insert("ready_close_missing_required_node_kinds".to_owned());
    }
    for kind in required_kinds {
        let found = nodes.values().any(|node| {
            node["required_in_close"] == true && string(node, "node_kind") == Some(kind)
        });
        if !found {
            errors.insert(format!("ready_close_required_kind_absent:{kind}"));
        }
    }
    for id in stale_nodes {
        if nodes
            .get(id)
            .is_some_and(|node| node["required_in_close"] == true)
        {
            errors.insert(format!("ready_close_contains_stale_required_node:{id}"));
        }
    }
    for (id, node) in nodes
        .iter()
        .filter(|(_, node)| node["required_in_close"] == true)
    {
        if string(&node["result"], "state") != Some("completed") {
            errors.insert(format!("ready_close_required_node_not_completed:{id}"));
        }
        if !matches!(
            string(&node["result"], "terminal_disposition"),
            Some("completed_and_verified" | "completed_with_scope_limitations")
        ) {
            errors.insert(format!("ready_close_required_node_not_verified:{id}"));
        }
    }

    let scope_limitations = array(input, "known_scope_limitations");
    if disposition == "ready_with_scope_limitations" && scope_limitations.is_empty() {
        errors.insert("ready_with_scope_limitations_missing_rationale".to_owned());
    }
    if disposition == "ready_for_authorized_signoff" && !scope_limitations.is_empty() {
        errors.insert("unqualified_ready_disposition_has_scope_limitations".to_owned());
    }

    for frontier in REQUIRED_READY_FRONTIERS {
        if string(&input["frontiers"][*frontier], "qualification_status") != Some("qualified") {
            errors.insert(format!("ready_close_frontier_not_qualified:{frontier}"));
        }
    }
    for profile in [
        "accounting_framework_profile",
        "jurisdiction_profile",
        "currency_profile",
    ] {
        if string(&input["reporting_context"][profile], "qualification_status") != Some("qualified")
        {
            errors.insert(format!(
                "ready_close_reporting_profile_not_qualified:{profile}"
            ));
        }
    }
}

fn canonical_integer(value: &str) -> bool {
    if value == "0" {
        return true;
    }
    let unsigned = value.strip_prefix('-').unwrap_or(value);
    !unsigned.is_empty()
        && !unsigned.starts_with('0')
        && unsigned.bytes().all(|byte| byte.is_ascii_digit())
        && value != "-0"
}

fn positive_integer(value: &str) -> bool {
    !value.is_empty()
        && !value.starts_with('0')
        && value.bytes().all(|byte| byte.is_ascii_digit())
        && value.bytes().any(|byte| byte != b'0')
}

fn canonical_decimal(value: &str) -> bool {
    let (whole, fractional) = match value.split_once('.') {
        Some(parts) => parts,
        None => return false,
    };
    let magnitude = whole.strip_prefix('-').unwrap_or(whole);
    let whole_valid = canonical_integer(whole) || (whole.starts_with('-') && magnitude == "0");
    let all_fractional_zero = !fractional.is_empty() && fractional.bytes().all(|byte| byte == b'0');
    whole_valid
        && !fractional.is_empty()
        && fractional.bytes().all(|byte| byte.is_ascii_digit())
        && !(whole.starts_with('-') && magnitude == "0" && all_fractional_zero)
}

fn decimal_scale(value: &str) -> Option<usize> {
    value
        .split_once('.')
        .map(|(_, fractional)| fractional.len())
}

fn canonical_digest(domain: &[u8], value: &Value) -> String {
    let mut bytes = domain.to_vec();
    write_canonical_json(value, &mut bytes);
    hex::encode(Sha256::digest(bytes))
}

fn write_canonical_json(value: &Value, output: &mut Vec<u8>) {
    match value {
        Value::Null => output.extend_from_slice(b"null"),
        Value::Bool(value) => output.extend_from_slice(if *value { b"true" } else { b"false" }),
        Value::Number(value) => output.extend_from_slice(value.to_string().as_bytes()),
        Value::String(value) => output.extend_from_slice(
            serde_json::to_string(value)
                .expect("JSON string")
                .as_bytes(),
        ),
        Value::Array(values) => {
            output.push(b'[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    output.push(b',');
                }
                write_canonical_json(value, output);
            }
            output.push(b']');
        }
        Value::Object(values) => {
            let sorted = values.iter().collect::<BTreeMap<_, _>>();
            output.push(b'{');
            for (index, (key, value)) in sorted.iter().enumerate() {
                if index > 0 {
                    output.push(b',');
                }
                output.extend_from_slice(serde_json::to_string(key).expect("JSON key").as_bytes());
                output.push(b':');
                write_canonical_json(value, output);
            }
            output.push(b'}');
        }
    }
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

fn string_set(value: &Value) -> BTreeSet<String> {
    value
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect()
}

fn sha256_hex(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn limitations() -> Vec<&'static str> {
    vec![
        "the verifier proves consistency of declared graph and reference fields only; it does not authenticate source artifacts",
        "payload_sha256 is a commitment to externally stored payload bytes and cannot be recomputed when payload bytes are not supplied",
        "profile qualification flags are treated as evidence references supplied by the manifest; real jurisdiction, accounting and currency qualification requires separate authority review",
        "human review nodes are structural graph evidence only; this verifier does not authenticate reviewers, determine authority or prove professional sufficiency",
        "this verifier never authorizes journal posting, period close, tax filing, payment or other external action",
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Value {
        serde_json::from_str(include_str!(
            "../../examples/acc-syn-0001-minimal-dag-v0.1.json"
        ))
        .expect("DAG fixture")
    }

    fn errors(report: &Value) -> String {
        report["errors"].to_string()
    }

    fn set_frontier(node: &mut Value, key: &str, ref_id: &str) {
        node["frontier_bindings"][key]["ref_id"] = json!(ref_id);
    }

    #[test]
    fn frozen_smoke_graph_is_semantically_valid_but_not_ready() {
        let value = fixture();
        let input = serde_json::to_vec(&value).unwrap();
        let schema = include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json");
        let report = verify(&input, schema);
        assert_eq!(report["status"], "PASS");
        assert_eq!(report["close_disposition"], "not_ready");
        assert!(
            !report["missing_required_node_kinds"]
                .as_array()
                .unwrap()
                .is_empty()
        );
        assert!(!report["can_post_journals"].as_bool().unwrap());
        assert!(!report["can_approve_or_close"].as_bool().unwrap());
    }

    #[test]
    fn topological_order_is_independent_of_node_array_order() {
        let mut value = fixture();
        let input = serde_json::to_vec(&value).unwrap();
        let schema = include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json");
        let first = verify(&input, schema);
        value["nodes"].as_array_mut().unwrap().reverse();
        let second = verify(&serde_json::to_vec(&value).unwrap(), schema);
        assert_eq!(first["topological_order"], second["topological_order"]);
    }

    #[test]
    fn duplicate_node_ids_fail() {
        let mut value = fixture();
        let duplicate = value["nodes"][0].clone();
        value["nodes"].as_array_mut().unwrap().push(duplicate);
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("duplicate_node_id"));
    }

    #[test]
    fn dangling_dependency_fails() {
        let mut value = fixture();
        value["nodes"][1]["depends_on"] = json!(["missing-parent"]);
        value["nodes"][1]["inputs"][1]["input_ref"]["ref_id"] = json!("missing-parent");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("dangling_dependency"));
    }

    #[test]
    fn self_dependency_fails() {
        let mut value = fixture();
        value["nodes"][0]["depends_on"] = json!(["source-match-001"]);
        value["nodes"][0]["inputs"].as_array_mut().unwrap().push(json!({
            "input_ref":{"ref_id":"source-match-001","version":"0.1.0","source_kind":"prior_workpaper"},
            "purpose":"prior_workpaper"
        }));
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("self_dependency"));
    }

    #[test]
    fn two_node_cycle_reports_cycle() {
        let mut value = fixture();
        value["nodes"][0]["depends_on"] = json!(["bank-observation-001"]);
        value["nodes"][0]["inputs"].as_array_mut().unwrap().push(json!({
            "input_ref":{"ref_id":"bank-observation-001","version":"0.1.0","source_kind":"prior_workpaper"},
            "purpose":"prior_workpaper"
        }));
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("dependency_cycle"));
    }

    #[test]
    fn dependency_without_prior_workpaper_input_fails() {
        let mut value = fixture();
        value["nodes"][1]["inputs"]
            .as_array_mut()
            .unwrap()
            .retain(|item| item["purpose"] != "prior_workpaper");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("dependency_without_prior_workpaper_input"));
    }

    #[test]
    fn prior_input_without_dependency_fails() {
        let mut value = fixture();
        value["nodes"][0]["inputs"].as_array_mut().unwrap().push(json!({
            "input_ref":{"ref_id":"entity-quarantine-guard","version":"0.1.0","source_kind":"prior_workpaper"},
            "purpose":"prior_workpaper"
        }));
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("prior_workpaper_input_without_edge"));
    }

    #[test]
    fn frontier_change_stales_only_consumers_and_descendants() {
        let mut value = fixture();
        set_frontier(
            &mut value["nodes"][0],
            "evidence_frontier",
            "MUTATED-EVIDENCE",
        );
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert_eq!(report["directly_stale_nodes"], json!(["source-match-001"]));
        assert_eq!(
            report["transitively_stale_nodes"],
            json!(["bank-observation-001", "entity-quarantine-guard"])
        );
    }

    #[test]
    fn changed_child_frontier_is_directly_stale_without_affecting_sibling() {
        let mut value = fixture();
        set_frontier(
            &mut value["nodes"][1],
            "evidence_frontier",
            "MUTATED-EVIDENCE",
        );
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert_eq!(
            report["directly_stale_nodes"],
            json!(["bank-observation-001"])
        );
        assert_eq!(report["transitively_stale_nodes"], json!([]));
        assert!(
            !report["directly_stale_nodes"]
                .to_string()
                .contains("entity-quarantine-guard")
        );
    }

    #[test]
    fn stale_completed_or_in_progress_node_must_be_invalidated() {
        let mut value = fixture();
        set_frontier(
            &mut value["nodes"][0],
            "evidence_frontier",
            "MUTATED-EVIDENCE",
        );
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("stale_node_not_invalidated"));
    }

    #[test]
    fn candidate_analysis_cannot_claim_journal_projection() {
        let mut value = fixture();
        value["nodes"][0]["node_kind"] = json!("journal_projection");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("non_authoritative_node_escalates_scope"));
    }

    #[test]
    fn symthaea_cannot_claim_deterministic_financial_projection() {
        let mut value = fixture();
        value["nodes"][0]["authority_class"] = json!("deterministic_financial_projection");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("advisory_system_claims_authority"));
    }

    #[test]
    fn ready_close_with_unknown_scope_frontier_fails() {
        let mut value = fixture();
        value["close_disposition"] = json!("ready_for_authorized_signoff");
        value["frontiers"]["source_scope_manifest"]["qualification_status"] = json!("qualified");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("ready_close_frontier_not_qualified"));
        assert!(errors(&report).contains("ready_close_missing_required_node_kinds"));
    }

    #[test]
    fn ratio_without_denominator_fails() {
        let mut value = fixture();
        value["nodes"][0]["calculation_recipe"]["operation"] = json!("ratio");
        value["nodes"][0]["calculation_recipe"]
            .as_object_mut()
            .unwrap()
            .remove("denominator");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert_eq!(report["status"], "FAIL");
    }

    #[test]
    fn many_to_many_join_requires_aggregation() {
        let mut value = fixture();
        let recipe = &mut value["nodes"][0]["calculation_recipe"];
        recipe["join_semantics"] = json!("many_to_many_with_explicit_aggregation");
        recipe["expected_cardinality"] = json!("unknown");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("nontrivial_join_without_explicit_aggregation"));
    }

    #[test]
    fn fixed_decimal_must_match_declared_scale_and_reject_negative_zero() {
        let mut value = fixture();
        value["nodes"][0]["result"]["numeric_value"] = json!({
            "encoding":"fixed_decimal",
            "value":"-0.00",
            "scale":2,
            "unit_profile":{"ref_id":"SYNTH-ZAR","source_kind":"synthetic_currency"}
        });
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("fixed_decimal_scale_or_format_mismatch"));
    }

    #[test]
    fn unknown_numeric_encoding_fails_closed() {
        let mut value = fixture();
        value["nodes"][0]["result"]["numeric_value"] = json!({
            "encoding":"binary_float",
            "value":"0.1",
            "unit_profile":{"ref_id":"SYNTH-ZAR","source_kind":"synthetic_currency"}
        });
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert_eq!(report["status"], "FAIL");
    }

    #[test]
    fn negative_fractional_decimal_is_valid_but_negative_zero_is_not() {
        let mut value = fixture();
        value["nodes"][0]["result"]["numeric_value"] = json!({
            "encoding":"fixed_decimal",
            "value":"-0.25",
            "scale":2,
            "unit_profile":{"ref_id":"SYNTH-ZAR","source_kind":"synthetic_currency"}
        });
        assert!(
            errors(&verify(
                &serde_json::to_vec(&value).unwrap(),
                include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json")
            ))
            .is_empty()
        );
        value["nodes"][0]["result"]["numeric_value"]["value"] = json!("-0.00");
        assert!(
            errors(&verify(
                &serde_json::to_vec(&value).unwrap(),
                include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json")
            ))
            .contains("fixed_decimal_scale_or_format_mismatch")
        );
    }

    #[test]
    fn canonical_dag_digest_ignores_node_array_order() {
        let mut value = fixture();
        let schema = include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json");
        let first = verify(&serde_json::to_vec(&value).unwrap(), schema);
        value["nodes"].as_array_mut().unwrap().reverse();
        let second = verify(&serde_json::to_vec(&value).unwrap(), schema);
        assert_eq!(
            first["digests"]["canonical_dag_sha256"],
            second["digests"]["canonical_dag_sha256"]
        );
    }

    #[test]
    fn prior_workpaper_input_version_must_match_dag_version() {
        let mut value = fixture();
        value["nodes"][1]["inputs"][1]["input_ref"]["version"] = json!("0.2.0");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("prior_workpaper_version_mismatch"));
    }

    #[test]
    fn workpaper_commitment_binds_canonical_node_without_self_digest_field() {
        let mut value = fixture();
        let parent = &mut value["nodes"][0];
        parent["result"]["state"] = json!("completed");
        parent["result"]["terminal_disposition"] = json!("completed_and_verified");
        parent["result"]["payload_sha256"] = json!("a".repeat(64));
        parent["result"]["result_type"] = json!("SyntheticSourceMatch");
        parent["result"]["produced_at"] = json!("2026-11-05T19:00:00Z");
        parent["result"]["workpaper_commitment_sha256"] = json!("0".repeat(64));
        let commitment = workpaper_commitment(parent);
        parent["result"]["workpaper_commitment_sha256"] = json!(commitment);
        value["nodes"][1]["inputs"][1]["input_ref"]["sha256"] = json!(commitment);
        value["nodes"][2]["inputs"][1]["input_ref"]["sha256"] = json!(commitment);
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert_eq!(report["status"], "PASS");
    }

    #[test]
    fn completed_child_cannot_consume_nonterminal_parent() {
        let mut value = fixture();
        value["nodes"][1]["result"]["state"] = json!("completed");
        value["nodes"][1]["result"]["terminal_disposition"] = json!("completed_and_verified");
        value["nodes"][1]["result"]["payload_sha256"] = json!("a".repeat(64));
        value["nodes"][1]["result"]["result_type"] = json!("SyntheticChild");
        value["nodes"][1]["result"]["produced_at"] = json!("2026-11-05T19:00:00Z");
        value["nodes"][1]["result"]["workpaper_commitment_sha256"] = json!("b".repeat(64));
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert!(errors(&report).contains("completed_child_has_nonterminal_parent"));
    }
    #[test]
    fn current_policy_frontier_change_stales_parent_and_transitive_child_only() {
        let mut value = fixture();
        value["nodes"]
            .as_array_mut()
            .unwrap()
            .retain(|node| node["node_id"] != "entity-quarantine-guard");
        value["nodes"][0]["frontier_bindings"]["policy_snapshot"] =
            value["frontiers"]["policy_snapshot"].clone();
        value["frontiers"]["policy_snapshot"]["ref_id"] = json!("SYNTHETIC-ACCRUAL-POLICY-V2");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json"),
        );
        assert_eq!(report["directly_stale_nodes"], json!(["source-match-001"]));
        assert_eq!(
            report["transitively_stale_nodes"],
            json!(["bank-observation-001"])
        );
    }

    #[test]
    fn independent_verifier_must_bind_validator_build_frontier() {
        let mut value = fixture();
        value["nodes"][2]["frontier_bindings"]
            .as_object_mut()
            .unwrap()
            .remove("validator_build");
        let report = verify(
            &serde_json::to_vec(&value).unwrap(),
            include_bytes!("../../WORKPAPER_DAG_V0.1.schema.json")
        );
        assert!(
            errors(&report).contains(
                "required_frontier_binding_missing:entity-quarantine-guard:validator_build"
            )
        );
    }

}
