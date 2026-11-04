use jsonschema::draft202012;
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

type AppResult<T> = Result<T, String>;

const DAG_DEFAULT: &str = "../examples/acc-syn-0001-minimal-dag-v0.1.json";
const SCHEMA_DEFAULT: &str = "../WORKPAPER_DAG_V0.1.schema.json";

#[derive(Clone, Debug)]
struct Check {
    invariant: String,
    passed: bool,
    severity: &'static str,
    detail: String,
}

struct VerifyOutput {
    checks: Vec<Check>,
    topological_order: Vec<String>,
    missing_required_node_kinds: Vec<String>,
    stale_nodes: Vec<String>,
    affected_descendants: Vec<String>,
    ready_blockers: Vec<String>,
    invalid: bool,
    ready_eligible: bool,
}

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("fin-acc-workpaper-semantic-verifier: {error}");
        std::process::exit(2);
    }
}

fn run_cli() -> Result<(), Box<dyn Error>> {
    let mut dag_path = PathBuf::from(DAG_DEFAULT);
    let mut schema_path = PathBuf::from(SCHEMA_DEFAULT);
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--dag" => dag_path = PathBuf::from(next_arg(&mut args, "--dag")?),
            "--schema" => schema_path = PathBuf::from(next_arg(&mut args, "--schema")?),
            "--out" => output_path = Some(PathBuf::from(next_arg(&mut args, "--out")?)),
            "--help" | "-h" => {
                println!("FIN-ACC workpaper DAG semantic verifier\n\n--dag PATH --schema PATH --out PATH");
                return Ok(());
            }
            unknown => return Err(format!("unknown argument {unknown}").into()),
        }
    }

    let dag_bytes = fs::read(&dag_path)?;
    let schema_bytes = fs::read(&schema_path)?;
    let dag: Value = serde_json::from_slice(&dag_bytes)?;
    let schema: Value = serde_json::from_slice(&schema_bytes)?;
    let report = verify(&dag, &schema, &dag_bytes, &schema_bytes);
    let rendered = serde_json::to_string_pretty(&report)?;
    match output_path {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }
    if report["status"] == "INVALID" {
        std::process::exit(1);
    }
    Ok(())
}

fn next_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, Box<dyn Error>> {
    args.next().ok_or_else(|| format!("{flag} requires a value").into())
}

fn verify(dag: &Value, schema: &Value, dag_bytes: &[u8], schema_bytes: &[u8]) -> Value {
    let schema_errors = validate_schema(schema, dag);
    let mut output = VerifyOutput {
        checks: Vec::new(),
        topological_order: Vec::new(),
        missing_required_node_kinds: Vec::new(),
        stale_nodes: Vec::new(),
        affected_descendants: Vec::new(),
        ready_blockers: Vec::new(),
        invalid: !schema_errors.is_empty(),
        ready_eligible: false,
    };

    if schema_errors.is_empty() {
        verify_graph(dag, &mut output);
    }

    let status = if output.invalid {
        "INVALID"
    } else if output.ready_eligible {
        "READY_ELIGIBLE"
    } else {
        "VALID_NOT_READY"
    };
    let check_values = output
        .checks
        .iter()
        .map(|check| {
            json!({
                "invariant": check.invariant,
                "passed": check.passed,
                "severity": check.severity,
                "detail": check.detail
            })
        })
        .collect::<Vec<_>>();

    let mut report = json!({
        "report_schema": "fin-acc-workpaper-semantic-report.v0.1",
        "status": status,
        "dag_id": dag.get("dag_id"),
        "digests": {
            "dag_sha256": sha256_hex(dag_bytes),
            "schema_sha256": sha256_hex(schema_bytes),
            "verifier_source_sha256": verifier_source_digest(),
            "report_schema_sha256": sha256_hex(include_bytes!("../report.schema.json"))
        },
        "input_schema_validation": {
            "passed": schema_errors.is_empty(),
            "errors": schema_errors
        },
        "check_count": check_values.len(),
        "checks": check_values,
        "topological_order": output.topological_order,
        "missing_required_node_kinds": output.missing_required_node_kinds,
        "stale_nodes": output.stale_nodes,
        "affected_descendants": output.affected_descendants,
        "ready_blockers": output.ready_blockers,
        "report_schema_validation": { "passed": true, "errors": [] }
    });

    let errors = validate_report(&report);
    report["report_schema_validation"] = json!({
        "passed": errors.is_empty(),
        "errors": errors
    });
    if report["report_schema_validation"]["passed"] != true {
        report["status"] = Value::String("INVALID".to_owned());
    }
    report
}

fn validate_schema(schema: &Value, dag: &Value) -> Vec<String> {
    let Some(declared) = schema.get("$schema").and_then(Value::as_str) else {
        return vec!["schema missing $schema".to_owned()];
    };
    if !declared.contains("2020-12") {
        return vec![format!("expected Draft 2020-12, got {declared}")];
    }
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options().should_validate_formats(true).build(schema) {
        Ok(value) => value,
        Err(error) => return vec![format!("cannot compile schema: {error}")],
    };
    validator
        .iter_errors(dag)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect()
}

fn validate_report(report: &Value) -> Vec<String> {
    let schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(value) => value,
        Err(error) => return vec![format!("cannot parse report schema: {error}")],
    };
    if let Err(error) = jsonschema::draft202012::meta::validate(&schema) {
        return vec![format!("invalid report schema: {error}")];
    }
    let validator = match draft202012::options().should_validate_formats(true).build(&schema) {
        Ok(value) => value,
        Err(error) => return vec![format!("cannot compile report schema: {error}")],
    };
    validator
        .iter_errors(report)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect()
}

fn verify_graph(dag: &Value, out: &mut VerifyOutput) {
    let nodes = match dag.get("nodes").and_then(Value::as_array) {
        Some(value) => value,
        None => {
            add(out, "nodes_array", false, "error", "nodes must be an array");
            out.invalid = true;
            return;
        }
    };
    let mut map: BTreeMap<String, &Value> = BTreeMap::new();
    let mut duplicate_ids = BTreeSet::new();
    for node in nodes {
        let Some(id) = node.get("node_id").and_then(Value::as_str) else {
            add(out, "node_id_present", false, "error", "node has no string node_id");
            out.invalid = true;
            continue;
        };
        if map.insert(id.to_owned(), node).is_some() {
            duplicate_ids.insert(id.to_owned());
        }
    }
    add(
        out,
        "unique_node_ids",
        duplicate_ids.is_empty() && map.len() == nodes.len(),
        "error",
        format!("node_count={} unique_ids={} duplicates={:?}", nodes.len(), map.len(), duplicate_ids),
    );
    if !duplicate_ids.is_empty() || map.len() != nodes.len() {
        out.invalid = true;
    }

    let mut adjacency: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut indegree: BTreeMap<String, usize> = map.keys().map(|id| (id.clone(), 0)).collect();
    let mut edge_errors = Vec::new();
    let mut prior_errors = Vec::new();
    for (id, node) in &map {
        adjacency.entry(id.clone()).or_default();
        let dependencies = string_set(node.get("depends_on"));
        let mut seen = BTreeSet::new();
        if let Some(values) = node.get("depends_on").and_then(Value::as_array) {
            for dependency in values {
                let Some(parent) = dependency.as_str() else {
                    edge_errors.push(format!("{id}: non-string dependency"));
                    continue;
                };
                if !seen.insert(parent.to_owned()) {
                    edge_errors.push(format!("{id}: duplicate edge to {parent}"));
                    continue;
                }
                if parent == id {
                    edge_errors.push(format!("{id}: self-dependency"));
                    continue;
                }
                if !map.contains_key(parent) {
                    edge_errors.push(format!("{id}: dangling dependency {parent}"));
                    continue;
                }
                adjacency.entry(parent.to_owned()).or_default().insert(id.clone());
                *indegree.entry(id.clone()).or_default() += 1;
            }
        }
        let prior_values = prior_workpaper_ref_vec(node);
        let prior: BTreeSet<String> = prior_values.iter().cloned().collect();
        let dep_set: BTreeSet<String> = dependencies;
        if prior_values.len() != prior.len() || prior != dep_set {
            prior_errors.push(format!("{id}: depends_on={dep_set:?} prior_workpaper_inputs={prior_values:?}"));
        }
    }
    add(
        out,
        "dependency_references_valid",
        edge_errors.is_empty(),
        "error",
        if edge_errors.is_empty() { "all dependencies exist and are unique".to_owned() } else { edge_errors.join("; ") },
    );
    add(
        out,
        "prior_workpaper_inputs_match_edges",
        prior_errors.is_empty(),
        "error",
        if prior_errors.is_empty() { "each graph edge has exactly one prior_workpaper input".to_owned() } else { prior_errors.join("; ") },
    );
    if !edge_errors.is_empty() || !prior_errors.is_empty() {
        out.invalid = true;
    }

    if edge_errors.is_empty() {
        let (order, remaining) = topological_order(&map, &adjacency, &indegree);
        out.topological_order = order;
        let cycle_or_blocked = remaining;
        let acyclic = cycle_or_blocked.is_empty();
        add(
            out,
            "dependency_graph_acyclic",
            acyclic,
            "error",
            if acyclic { "DAG is acyclic".to_owned() } else { format!("cycle or cycle-dependent nodes: {cycle_or_blocked:?}") },
        );
        if !acyclic {
            out.invalid = true;
        }
    }

    verify_parent_commitments(dag, &map, out);
    verify_node_states_and_authority(&map, out);
    verify_recipes_and_numeric_values(&map, out);
    verify_frontier_staleness(dag, &map, &adjacency, out);
    verify_required_kinds_and_close_readiness(dag, &map, out);
}

fn verify_parent_commitments(dag: &Value, map: &BTreeMap<String, &Value>, out: &mut VerifyOutput) {
    let dag_version = string_field(dag, "dag_version").unwrap_or("");
    let mut errors = Vec::new();
    for (child_id, child) in map {
        for input in inputs_with_purpose(child, "prior_workpaper") {
            let reference = input.get("input_ref").unwrap_or(&Value::Null);
            let Some(parent_id) = reference.get("ref_id").and_then(Value::as_str) else {
                errors.push(format!("{child_id}: prior_workpaper input missing ref_id"));
                continue;
            };
            let Some(parent) = map.get(parent_id) else {
                errors.push(format!("{child_id}: prior_workpaper parent {parent_id} does not exist"));
                continue;
            };
            let parent_result = parent.get("result").unwrap_or(&Value::Null);
            let parent_state = string_field(parent_result, "state").unwrap_or("");
            let child_result = child.get("result").unwrap_or(&Value::Null);
            let child_state = string_field(child_result, "state").unwrap_or("");
            if parent_state == "completed" {
                let expected_digest = string_field(parent_result, "workpaper_commitment_sha256");
                let supplied_digest = string_field(reference, "sha256");
                let version = string_field(reference, "version");
                if expected_digest.is_none() || supplied_digest != expected_digest {
                    errors.push(format!("{child_id}: parent {parent_id} commitment digest mismatch/missing"));
                }
                if version != Some(dag_version) {
                    errors.push(format!("{child_id}: parent {parent_id} version must match DAG version"));
                }
            } else if child_state == "completed" {
                errors.push(format!("{child_id}: completed node depends on non-terminal parent {parent_id}"));
            }
        }
    }
    add(
        out,
        "parent_workpaper_commitments",
        errors.is_empty(),
        "error",
        if errors.is_empty() { "parent references are compatible with parent state".to_owned() } else { errors.join("; ") },
    );
    if !errors.is_empty() {
        out.invalid = true;
    }
}

fn verify_node_states_and_authority(map: &BTreeMap<String, &Value>, out: &mut VerifyOutput) {
    let mut state_errors = Vec::new();
    let mut authority_errors = Vec::new();
    let mut digest_errors = Vec::new();
    for (id, node) in map {
        let result = node.get("result").unwrap_or(&Value::Null);
        let state = string_field(result, "state").unwrap_or("");
        let disposition = string_field(result, "terminal_disposition").unwrap_or("");
        let state_ok = match state {
            "not_started" | "in_progress" => disposition == "not_terminal",
            "completed" => matches!(disposition, "completed_and_verified" | "completed_with_scope_limitations"),
            "invalidated_stale" => disposition == "blocked_by_stale_frontier",
            "aborted" => disposition == "timed_out_or_aborted",
            "blocked" => matches!(disposition, "needs_evidence" | "contradicted" | "needs_authorized_review" | "unsupported_profile" | "indeterminate" | "blocked_by_stale_frontier" | "failed_invariant" | "timed_out_or_aborted"),
            _ => false,
        };
        if !state_ok {
            state_errors.push(format!("{id}: state={state} disposition={disposition}"));
        }
        if state == "completed" {
            if string_field(result, "payload_sha256").is_none() || string_field(result, "workpaper_commitment_sha256").is_none() {
                digest_errors.push(format!("{id}: completed result requires payload and full workpaper commitments"));
            } else {
                let declared_payload = string_field(result, "payload_sha256").unwrap_or("");
                let calculated_payload = result_payload_digest(result);
                if declared_payload != calculated_payload {
                    digest_errors.push(format!("{id}: payload_sha256 mismatch"));
                }
                let declared_commitment = string_field(result, "workpaper_commitment_sha256").unwrap_or("");
                let calculated_commitment = workpaper_commitment_digest(node);
                if declared_commitment != calculated_commitment {
                    digest_errors.push(format!("{id}: workpaper_commitment_sha256 mismatch"));
                }
            }
        }

        let owner = string_field(node, "owner_system").unwrap_or("");
        let authority = string_field(node, "authority_class").unwrap_or("");
        if owner == "symthaea" && authority != "candidate_analysis" {
            authority_errors.push(format!("{id}: Symthaea must remain candidate_analysis, got {authority}"));
        }
        if owner == "symtropy" && authority != "simulation_non_authoritative" {
            authority_errors.push(format!("{id}: Symtropy must remain simulation_non_authoritative, got {authority}"));
        }
        if matches!(authority, "candidate_analysis" | "simulation_non_authoritative") {
            let kind = string_field(node, "node_kind").unwrap_or("");
            if matches!(kind, "journal_projection" | "close_readiness") {
                authority_errors.push(format!("{id}: non-authoritative owner cannot own {kind}"));
            }
            let result_type = string_field(result, "result_type").unwrap_or("").to_ascii_lowercase();
            if ["posted", "authorized", "settlement_final", "bank_settled", "actual_financial_fact"].iter().any(|term| result_type.contains(term)) {
                authority_errors.push(format!("{id}: candidate/simulation result type claims authority"));
            }
        }
    }
    add(out,"node_state_disposition_consistency",state_errors.is_empty(),"error",if state_errors.is_empty(){"all node states have compatible dispositions".to_owned()}else{state_errors.join("; ")});
    add(out,"completed_workpaper_commitments",digest_errors.is_empty(),"error",if digest_errors.is_empty(){"completed workpapers have matching payload/full commitments".to_owned()}else{digest_errors.join("; ")});
    add(out,"authority_non_escalation",authority_errors.is_empty(),"error",if authority_errors.is_empty(){"candidate/simulation nodes do not own authoritative stages".to_owned()}else{authority_errors.join("; ")});
    if !state_errors.is_empty() || !digest_errors.is_empty() || !authority_errors.is_empty() {
        out.invalid = true;
    }
}

fn verify_recipes_and_numeric_values(map: &BTreeMap<String, &Value>, out: &mut VerifyOutput) {
    let mut errors = Vec::new();
    for (id, node) in map {
        if let Some(recipe) = node.get("calculation_recipe") {
            let operation = string_field(recipe,"operation").unwrap_or("");
            let join = string_field(recipe,"join_semantics").unwrap_or("");
            let inputs = ref_ids(recipe.get("input_refs"));
            let input_set: BTreeSet<String> = inputs.into_iter().collect();
            if operation == "ratio" || operation == "weighted_average" {
                for field in ["numerator","denominator"] {
                    if let Some(agg) = recipe.get(field) {
                        let ref_id=string_field(agg.get("input_ref").unwrap_or(&Value::Null),"ref_id").unwrap_or("");
                        if !input_set.contains(ref_id) {
                            errors.push(format!("{id}: {field} input {ref_id} is not in recipe input_refs"));
                        }
                    } else {
                        errors.push(format!("{id}: {operation} requires {field} aggregation"));
                    }
                }
            }
            if operation == "ratio" {
                if !matches!(string_field(recipe,"zero_denominator_disposition").unwrap_or(""),"reject"|"indeterminate"|"needs_authorized_review") {
                    errors.push(format!("{id}: ratio requires fail-closed zero denominator disposition"));
                }
            }
            if join == "one_to_many_with_explicit_aggregation" || join == "many_to_many_with_explicit_aggregation" {
                if recipe.get("join_keys").and_then(Value::as_array).is_none_or(|v|v.is_empty()) {
                    errors.push(format!("{id}: many-valued join requires explicit join keys"));
                }
                if string_field(recipe,"expected_cardinality").unwrap_or("") == "unknown" {
                    errors.push(format!("{id}: many-valued join cannot declare unknown cardinality"));
                }
            }
            if string_field(recipe,"null_semantics").unwrap_or("") == "profile_defined"
                && recipe.get("implementation_ref").is_none() {
                errors.push(format!("{id}: profile_defined null semantics require a pinned implementation_ref"));
            }
            if let Some(denom)=recipe.get("denominator") {
                let agg=string_field(denom,"operation").unwrap_or("");
                if agg=="count" && string_field(denom,"field_path").is_none() {
                    errors.push(format!("{id}: denominator count requires an explicit counted field"));
                }
            }
        }
        if let Some(value) = node.pointer("/result/numeric_value") {
            if let Err(error) = verify_numeric_value(value) {
                errors.push(format!("{id}: {error}"));
            }
        }
    }
    add(out,"calculation_recipe_semantics",errors.is_empty(),"error",if errors.is_empty(){"recipes preserve exact inputs, denominators, cardinality and null semantics".to_owned()}else{errors.join("; ")});
    if !errors.is_empty() {
        out.invalid=true;
    }
}

fn verify_numeric_value(value: &Value) -> AppResult<()> {
    let encoding=string_field(value,"encoding").ok_or_else(||"numeric_value missing encoding".to_owned())?;
    match encoding {
        "integer_minor_units" => {
            let raw=string_field(value,"value").ok_or_else(||"integer minor units missing value".to_owned())?;
            parse_canonical_i128(raw)?;
        }
        "fixed_decimal" => {
            let raw=string_field(value,"value").ok_or_else(||"fixed decimal missing value".to_owned())?;
            let unsigned=raw.strip_prefix('-').unwrap_or(raw);
            let (whole,fraction)=unsigned.split_once('.').ok_or_else(||"fixed decimal requires decimal point".to_owned())?;
            let scale=value.get("scale").and_then(Value::as_u64).ok_or_else(||"fixed decimal missing integer scale".to_owned())?;
            if whole.is_empty() || !whole.bytes().all(|b|b.is_ascii_digit()) || (whole.len()>1 && whole.starts_with('0')) {
                return Err("fixed decimal whole part is not canonical".to_owned());
            }
            if fraction.is_empty() || !fraction.bytes().all(|b|b.is_ascii_digit()) {
                return Err("fixed decimal fraction is not canonical".to_owned());
            }
            if fraction.len() as u64 != scale {
                return Err(format!("fixed decimal textual scale {} differs from declared {scale}",fraction.len()));
            }
            if raw.starts_with('-') && whole=="0" && fraction.bytes().all(|b|b==b'0') {
                return Err("non-canonical negative zero decimal".to_owned());
            }
        }
        "exact_rational" => {
            let n=string_field(value,"numerator").ok_or_else(||"rational missing numerator".to_owned())?;
            let d=string_field(value,"denominator").ok_or_else(||"rational missing denominator".to_owned())?;
            parse_canonical_i128(n)?;
            let den=d.parse::<u128>().map_err(|_|"invalid rational denominator".to_owned())?;
            if den==0 || d.starts_with('0') || !d.bytes().all(|b|b.is_ascii_digit()) {
                return Err("rational denominator must be a canonical positive integer".to_owned());
            }
        }
        other => return Err(format!("unsupported numeric_value encoding {other}")),
    }
    let profile=value.get("unit_profile").and_then(|v|v.get("ref_id")).and_then(Value::as_str).unwrap_or("");
    if profile.is_empty() {
        return Err("numeric_value lacks unit profile".to_owned());
    }
    Ok(())
}

fn parse_canonical_i128(raw:&str)->AppResult<i128>{
    if raw=="-0"||raw.is_empty()||raw.trim()!=raw {return Err("non-canonical signed integer".to_owned());}
    let d=raw.strip_prefix('-').unwrap_or(raw);
    if d.is_empty()||!d.bytes().all(|b|b.is_ascii_digit())||(d.len()>1&&d.starts_with('0')) {return Err("non-canonical signed integer".to_owned());}
    raw.parse::<i128>().map_err(|_|"integer overflow".to_owned())
}

fn verify_frontier_staleness(
    dag:&Value,
    map:&BTreeMap<String,&Value>,
    adjacency:&BTreeMap<String,BTreeSet<String>>,
    out:&mut VerifyOutput
){
    let current=dag.get("frontiers").and_then(Value::as_object).cloned().unwrap_or_default();
    let mut direct_stale=BTreeSet::new();
    for (id,node) in map {
        let bindings=node.get("frontier_bindings").and_then(Value::as_object).cloned().unwrap_or_default();
        for (key,binding) in bindings {
            if let Some(now)=current.get(&key) {
                if canonical_json(&binding).ok()!=canonical_json(now).ok() {
                    direct_stale.insert(id.clone());
                }
            } else {
                direct_stale.insert(id.clone());
            }
        }
    }
    let mut affected=BTreeSet::new();
    let mut queue:VecDeque<String>=direct_stale.iter().cloned().collect();
    while let Some(id)=queue.pop_front(){
        if !affected.insert(id.clone()){continue;}
        for child in adjacency.get(&id).into_iter().flatten(){queue.push_back(child.clone());}
    }
    out.stale_nodes=direct_stale.iter().cloned().collect();
    out.affected_descendants=affected.iter().filter(|id|!direct_stale.contains(*id)).cloned().collect();
    let mut errors=Vec::new();
    for id in &affected {
        if let Some(node)=map.get(id) {
            let result=node.get("result").unwrap_or(&Value::Null);
            let state=string_field(result,"state").unwrap_or("");
            let disposition=string_field(result,"terminal_disposition").unwrap_or("");
            if state=="completed" {
                errors.push(format!("{id}: completed workpaper is stale/descends from stale frontier and was not invalidated"));
            } else if state=="invalidated_stale" && disposition!="blocked_by_stale_frontier" {
                errors.push(format!("{id}: stale state has wrong terminal disposition"));
            }
        }
    }
    add(out,"frontier_stale_propagation",errors.is_empty(),"error",if errors.is_empty(){format!("direct_stale={:?}; descendants={:?}",out.stale_nodes,out.affected_descendants)}else{errors.join("; ")});
    if !errors.is_empty(){out.invalid=true;}
}

fn verify_required_kinds_and_close_readiness(dag:&Value,map:&BTreeMap<String,&Value>,out:&mut VerifyOutput){
    let required=string_set(dag.get("required_node_kinds"));
    let present:BTreeSet<String>=map.values().filter_map(|n|string_field(n,"node_kind").map(str::to_owned)).collect();
    out.missing_required_node_kinds=required.difference(&present).cloned().collect();
    let close=string_field(dag,"close_disposition").unwrap_or("not_ready");
    let source_qualified=dag.pointer("/frontiers/source_scope_manifest/qualification_status").and_then(Value::as_str)==Some("qualified");
    let mut blockers=Vec::new();
    if !out.missing_required_node_kinds.is_empty(){blockers.push(format!("missing required node kinds: {:?}",out.missing_required_node_kinds));}
    if !source_qualified{blockers.push("source-scope manifest is not qualified".to_owned());}
    for (id,node) in map {
        if node.get("required_in_close").and_then(Value::as_bool)==Some(true) {
            let result=node.get("result").unwrap_or(&Value::Null);
            if string_field(result,"state")!=Some("completed") || !matches!(string_field(result,"terminal_disposition"),Some("completed_and_verified"|"completed_with_scope_limitations")) {
                blockers.push(format!("{id}: required close node is not completed and verified"));
            }
        }
    }
    let human_authorized=map.values().any(|node|{
        string_field(node,"node_kind")==Some("human_review")
            && string_field(node,"owner_system")==Some("human_reviewer")
            && string_field(node,"authority_class")==Some("human_authorized_decision")
            && string_field(node.pointer("/result"),"state")==Some("completed")
            && string_field(node.pointer("/result"),"terminal_disposition")==Some("completed_and_verified")
    });
    if !human_authorized{blockers.push("no completed human-authorized review node".to_owned());}
    if close=="ready_with_scope_limitations" && dag.get("known_scope_limitations").and_then(Value::as_array).is_none_or(|v|v.is_empty()){
        blockers.push("ready_with_scope_limitations requires named limitations".to_owned());
    }
    out.ready_blockers=blockers.clone();
    if close=="ready_for_authorized_signoff"||close=="ready_with_scope_limitations" {
        if blockers.is_empty(){out.ready_eligible=true;}else{
            add(out,"close_readiness_gates",false,"blocker",blockers.join("; "));
            out.invalid=true;
        }
    }else{
        add(out,"close_not_marked_ready",true,"info",format!("close_disposition={close}; blockers={}",blockers.len()));
    }
    add(out,"required_node_kinds_present",out.missing_required_node_kinds.is_empty(),"blocker",if out.missing_required_node_kinds.is_empty(){"all required node kinds are represented".to_owned()}else{format!("missing {:?}",out.missing_required_node_kinds)});
    add(out,"source_scope_qualified_for_readiness",source_qualified,"blocker",if source_qualified{"source-scope manifest is qualified".to_owned()}else{"source-scope manifest is unknown/unqualified; close cannot be ready".to_owned()});
}

fn topological_order(
    map:&BTreeMap<String,&Value>,
    adjacency:&BTreeMap<String,BTreeSet<String>>,
    indegree:&BTreeMap<String,usize>
)->(Vec<String>,Vec<String>){
    let mut degrees=indegree.clone();
    let mut ready:BTreeSet<String>=degrees.iter().filter(|(_,d)|**d==0).map(|(id,_)|id.clone()).collect();
    let mut order=Vec::new();
    while let Some(id)=ready.pop_first(){
        order.push(id.clone());
        for child in adjacency.get(&id).into_iter().flatten(){
            let degree=degrees.get_mut(child).expect("child has indegree");
            *degree-=1;
            if *degree==0{ready.insert(child.clone());}
        }
    }
    let remaining=map.keys().filter(|id|!order.contains(id)).cloned().collect();
    (order,remaining)
}

fn result_payload_digest(result:&Value)->String{
    let mut payload=Map::new();
    for key in ["result_type","numeric_value","limitations"] {
        if let Some(value)=result.get(key){payload.insert(key.to_owned(),value.clone());}
    }
    sha256_hex(&canonical_json(&Value::Object(payload)).unwrap_or_default())
}

fn workpaper_commitment_digest(node:&Value)->String{
    let mut copy=node.clone();
    if let Some(result)=copy.get_mut("result").and_then(Value::as_object_mut){
        result.remove("workpaper_commitment_sha256");
    }
    sha256_hex(&canonical_json(&copy).unwrap_or_default())
}

fn prior_workpaper_ref_vec(node:&Value)->Vec<String>{
    inputs_with_purpose(node,"prior_workpaper").into_iter()
        .filter_map(|input|string_field(input.get("input_ref").unwrap_or(&Value::Null),"ref_id").map(str::to_owned))
        .collect()
}

fn inputs_with_purpose<'a>(node:&'a Value,purpose:&str)->Vec<&'a Value>{
    node.get("inputs").and_then(Value::as_array).into_iter().flatten()
        .filter(|input|string_field(input,"purpose")==Some(purpose)).collect()
}

fn ref_ids(value:Option<&Value>)->Vec<String>{
    value.and_then(Value::as_array).into_iter().flatten()
        .filter_map(|v|string_field(v,"ref_id").map(str::to_owned)).collect()
}

fn string_set(value:Option<&Value>)->BTreeSet<String>{
    value.and_then(Value::as_array).into_iter().flatten()
        .filter_map(Value::as_str).map(str::to_owned).collect()
}

fn string_field<'a>(value:&'a Value,key:&str)->Option<&'a str>{
    value.get(key).and_then(Value::as_str)
}

fn add(out:&mut VerifyOutput,invariant:&str,passed:bool,severity:&'static str,detail:impl Into<String>){
    out.checks.push(Check{invariant:invariant.to_owned(),passed,severity,detail:detail.into()});
}

fn canonical_json(value:&Value)->AppResult<Vec<u8>>{
    let mut output=Vec::new();write_canonical(value,&mut output)?;Ok(output)
}

fn write_canonical(value:&Value,output:&mut Vec<u8>)->AppResult<()>{
    match value{
        Value::Null=>output.extend_from_slice(b"null"),
        Value::Bool(v)=>output.extend_from_slice(if *v{b"true"}else{b"false"}),
        Value::Number(v)=>output.extend_from_slice(serde_json::to_string(v).map_err(|e|e.to_string())?.as_bytes()),
        Value::String(v)=>output.extend_from_slice(serde_json::to_string(v).map_err(|e|e.to_string())?.as_bytes()),
        Value::Array(values)=>{
            output.push(b'[');
            for (i,value) in values.iter().enumerate(){if i>0{output.push(b',');}write_canonical(value,output)?;}
            output.push(b']');
        }
        Value::Object(object)=>{
            output.push(b'{');
            let mut entries=object.iter().collect::<Vec<_>>();
            entries.sort_by(|a,b|a.0.cmp(b.0));
            for (i,(key,value)) in entries.into_iter().enumerate(){
                if i>0{output.push(b',');}
                output.extend_from_slice(serde_json::to_string(key).map_err(|e|e.to_string())?.as_bytes());
                output.push(b':');write_canonical(value,output)?;
            }
            output.push(b'}');
        }
    }
    Ok(())
}

fn sha256_hex(bytes:&[u8])->String{hex::encode(Sha256::digest(bytes))}

fn verifier_source_digest()->String{
    let mut hasher=Sha256::new();
    for (path,bytes) in [
        ("Cargo.toml",include_bytes!("../Cargo.toml").as_slice()),
        ("README.md",include_bytes!("../README.md").as_slice()),
        ("report.schema.json",include_bytes!("../report.schema.json").as_slice()),
        ("src/main.rs",include_bytes!("main.rs").as_slice()),
    ]{
        hasher.update(path.as_bytes());hasher.update([0]);hasher.update(bytes);hasher.update([0]);
    }
    hex::encode(hasher.finalize())
}

#[cfg(test)]
mod tests{
    use super::*;

    fn fixture()->(Value,Value){
        let dag:Value=serde_json::from_str(include_str!("../../examples/acc-syn-0001-minimal-dag-v0.1.json")).expect("DAG example JSON");
        let schema:Value=serde_json::from_str(include_str!("../../WORKPAPER_DAG_V0.1.schema.json")).expect("DAG schema JSON");
        (dag,schema)
    }

    fn verified(dag:&Value,schema:&Value)->Value{
        let db=serde_json::to_vec(dag).unwrap();let sb=serde_json::to_vec(schema).unwrap();
        verify(dag,schema,&db,&sb)
    }

    #[test] fn example_is_valid_but_not_ready(){let(d,s)=fixture();let r=verified(&d,&s);assert_eq!(r["status"],"VALID_NOT_READY","{r:#}");}

    #[test] fn duplicate_node_id_is_invalid(){let(mut d,s)=fixture();let x=d["nodes"][0].clone();d["nodes"].as_array_mut().unwrap().push(x);assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn dangling_dependency_is_invalid(){let(mut d,s)=fixture();d["nodes"][0]["depends_on"]=json!(["missing"]);assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn self_dependency_is_invalid(){let(mut d,s)=fixture();d["nodes"][0]["depends_on"]=json!(["source-match-001"]);assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn two_node_cycle_is_invalid(){let(mut d,s)=fixture();d["nodes"][0]["depends_on"]=json!(["bank-observation-001"]);d["nodes"][0]["inputs"].as_array_mut().unwrap().push(json!({"input_ref":{"ref_id":"bank-observation-001","version":"0.1.0","source_kind":"prior_workpaper"},"purpose":"prior_workpaper"}));assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn graph_edge_without_prior_input_is_invalid(){let(mut d,s)=fixture();d["nodes"][1]["inputs"].as_array_mut().unwrap().retain(|x|x["purpose"]!="prior_workpaper");assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn prior_input_without_graph_edge_is_invalid(){let(mut d,s)=fixture();d["nodes"][0]["inputs"].as_array_mut().unwrap().push(json!({"input_ref":{"ref_id":"bank-observation-001","source_kind":"prior_workpaper"},"purpose":"prior_workpaper"}));assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn candidate_cannot_own_journal_projection(){let(mut d,s)=fixture();d["nodes"][0]["node_kind"]=json!("journal_projection");assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn symtropy_cannot_claim_authoritative_projection(){let(mut d,s)=fixture();d["nodes"][0]["owner_system"]=json!("symtropy");d["nodes"][0]["authority_class"]=json!("deterministic_financial_projection");assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn changed_frontier_marks_direct_consumer_stale(){let(mut d,s)=fixture();d["frontiers"]["evidence_frontier"]["version"]=json!("0.2");let r=verified(&d,&s);assert_eq!(r["stale_nodes"].as_array().unwrap().len(),3);}

    #[test] fn completed_child_of_nonterminal_parent_is_invalid(){let(mut d,s)=fixture();let child=&mut d["nodes"][1];child["result"]["state"]=json!("completed");child["result"]["terminal_disposition"]=json!("completed_and_verified");child["result"]["payload_sha256"]=json!("a".repeat(64));child["result"]["workpaper_commitment_sha256"]=json!("b".repeat(64));assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn missing_required_stages_do_not_make_nonready_graph_invalid(){let(d,s)=fixture();let r=verified(&d,&s);assert_eq!(r["status"],"VALID_NOT_READY");assert!(!r["missing_required_node_kinds"].as_array().unwrap().is_empty());}

    #[test] fn ready_close_with_unknown_scope_is_invalid(){let(mut d,s)=fixture();d["close_disposition"]=json!("ready_for_authorized_signoff");assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn ready_with_scope_limitations_requires_named_limitations(){let(mut d,s)=fixture();d["close_disposition"]=json!("ready_with_scope_limitations");d["known_scope_limitations"]=json!([]);assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn wrong_state_disposition_pair_is_invalid(){let(mut d,s)=fixture();d["nodes"][0]["result"]["state"]=json!("completed");d["nodes"][0]["result"]["terminal_disposition"]=json!("not_terminal");assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn completed_parent_requires_exact_commitment_digest(){
        let(mut d,s)=fixture();
        let parent=&mut d["nodes"][0];
        parent["result"]["state"]=json!("completed");
        parent["result"]["terminal_disposition"]=json!("completed_and_verified");
        parent["result"]["payload_sha256"]=json!("a".repeat(64));
        parent["result"]["result_type"]=json!("SyntheticInvoiceEvidenceMatch");
        parent["result"]["produced_at"]=json!("2026-11-04T18:00:01+02:00");
        parent["result"]["workpaper_commitment_sha256"]=json!("b".repeat(64));
        let r=verified(&d,&s);
        assert_eq!(r["status"],"INVALID");
        assert!(r["checks"].as_array().unwrap().iter().any(|c|c["invariant"]=="parent_workpaper_commitments"&&c["passed"]==false));
    }

    #[test] fn duplicate_prior_workpaper_input_is_rejected(){
        let(mut d,s)=fixture();
        let input=d["nodes"][1]["inputs"][1].clone();
        d["nodes"][1]["inputs"].as_array_mut().unwrap().push(input);
        assert_eq!(verified(&d,&s)["status"],"INVALID");
    }

    #[test] fn fixed_decimal_negative_zero_is_rejected(){
        let(mut d,s)=fixture();
        d["nodes"][0]["result"]["numeric_value"]=json!({"encoding":"fixed_decimal","value":"-0.00","scale":2,"unit_profile":{"ref_id":"rate"}});
        assert_eq!(verified(&d,&s)["status"],"INVALID");
    }

    #[test] fn rational_zero_denominator_is_rejected(){let(mut d,s)=fixture();d["nodes"][0]["result"]["numeric_value"]=json!({"encoding":"exact_rational","numerator":"3","denominator":"0","unit_profile":{"ref_id":"rate"}});assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn fixed_decimal_scale_mismatch_is_rejected(){let(mut d,s)=fixture();d["nodes"][0]["result"]["numeric_value"]=json!({"encoding":"fixed_decimal","value":"1.20","scale":3,"unit_profile":{"ref_id":"ratio"}});assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn negative_zero_is_rejected(){let(mut d,s)=fixture();d["nodes"][0]["result"]["numeric_value"]=json!({"encoding":"integer_minor_units","value":"-0","unit_profile":{"ref_id":"zar-cent"}});assert_eq!(verified(&d,&s)["status"],"INVALID");}

    #[test] fn node_order_does_not_change_canonical_topology(){let(mut d,s)=fixture();let a=verified(&d,&s);d["nodes"].as_array_mut().unwrap().reverse();let b=verified(&d,&s);assert_eq!(a["topological_order"],b["topological_order"]);}

    #[test] fn required_node_kinds_are_unique_and_enforced_by_schema(){let(mut d,s)=fixture();d["required_node_kinds"].as_array_mut().unwrap().push(json!("assertion_workpaper"));assert!(!verified(&d,&s)["input_schema_validation"]["passed"].as_bool().unwrap());}
}
