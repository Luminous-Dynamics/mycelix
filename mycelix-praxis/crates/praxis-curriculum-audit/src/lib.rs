// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic, read-only coverage audit for the active Praxis graph and
//! generated lesson manifest. Coverage is not a claim of lesson quality or
//! demonstrated learning efficacy.

use serde::Serialize;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

const SCHEMA_VERSION: u32 = 1;
const INCLUDE_MARKER: &str = "include_str!(";

#[derive(Debug, Clone)]
struct NodeInfo {
    grades: Vec<String>,
    subject: String,
    standards_by_framework: BTreeMap<String, BTreeSet<String>>,
    frameworks: BTreeSet<String>,
    lesson_assets: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct GroupKey {
    framework: String,
    grade_level: String,
    subject_area: String,
}

#[derive(Debug, Default)]
struct GroupAccumulator {
    nodes: usize,
    nodes_with_standards: usize,
    nodes_with_lessons: usize,
    nodes_missing_lessons: usize,
    lesson_assets: usize,
    standards: BTreeSet<String>,
}

#[derive(Debug, Serialize)]
struct CoverageGroup {
    framework: String,
    grade_level: String,
    subject_area: String,
    active_nodes: usize,
    nodes_with_standard_refs: usize,
    nodes_without_standard_refs: usize,
    distinct_standard_refs: usize,
    nodes_with_lessons: usize,
    nodes_missing_lessons: usize,
    lesson_assets: usize,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    schema_version: u32,
    active_document_count: usize,
    active_node_count: usize,
    active_edge_count: usize,
    active_nodes_with_standard_refs: usize,
    active_nodes_without_standard_refs: usize,
    distinct_standard_refs: usize,
    manifest_node_id_count: usize,
    manifest_asset_count: usize,
    active_nodes_with_lessons: usize,
    active_nodes_missing_lessons: usize,
    orphan_manifest_node_ids: Vec<String>,
    groups: Vec<CoverageGroup>,
    input_digest_blake3: String,
}

#[derive(Debug)]
struct AuditFailure(Vec<String>);

fn nonempty_string(value: Option<&Value>) -> Option<String> {
    value.and_then(Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .map(ToOwned::to_owned)
}

fn string_array(value: Option<&Value>) -> Vec<String> {
    value.and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .map(ToOwned::to_owned)
}

/// Extract JSON string literals from include_str! invocations. The source file
/// is the same one the application uses to declare the active curriculum.
fn embedded_json_paths(source: &str) -> Vec<String> {
    let mut paths = Vec::new();
    let mut cursor = 0;

    while let Some(relative_start) = source[cursor..].find(INCLUDE_MARKER) {
        let expression_start = cursor + relative_start + INCLUDE_MARKER.len();
        let Some(relative_end) = source[expression_start..].find(')') else {
            break;
        };
        let expression_end = expression_start + relative_end;
        let expression = &source[expression_start..expression_end];

        if let Some(open_quote) = expression.find('"') {
            let literal_start = open_quote + 1;
            if let Some(relative_quote_end) = expression[literal_start..].find('"') {
                let path = &expression[literal_start..literal_start + relative_quote_end];
                if path.ends_with(".json") {
                    paths.push(path.to_owned());
                }
            }
        }

        cursor = expression_end + 1;
    }

    paths
}

fn active_documents(workspace: &Path, errors: &mut Vec<String>) -> Vec<(PathBuf, Value)> {
    let source_path = workspace.join("apps/leptos/src/curriculum.rs");
    let source = match fs::read_to_string(&source_path) {
        Ok(text) => text,
        Err(error) => {
            errors.push(format!("cannot read {}: {error}", source_path.display()));
            return Vec::new();
        }
    };

    let Some(parent) = source_path.parent() else {
        errors.push(format!("invalid source path: {}", source_path.display()));
        return Vec::new();
    };
    let paths = embedded_json_paths(&source);
    if paths.is_empty() {
        errors.push(format!("{}: no embedded JSON curriculum documents found", source_path.display()));
        return Vec::new();
    }

    let mut seen = BTreeSet::new();
    let mut documents = Vec::new();
    for relative in paths {
        let unresolved = parent.join(&relative);
        let path = match unresolved.canonicalize() {
            Ok(path) => path,
            Err(error) => {
                errors.push(format!("cannot resolve curriculum include {relative:?}: {error}"));
                continue;
            }
        };
        if !path.starts_with(workspace) {
            errors.push(format!("curriculum include escapes workspace root: {relative}"));
            continue;
        }
        if !seen.insert(path.clone()) {
            errors.push(format!("curriculum document included more than once: {relative}"));
            continue;
        }
        match fs::read_to_string(&path) {
            Ok(text) => match serde_json::from_str::<Value>(&text) {
                Ok(value) if value.is_object() => documents.push((path, value)),
                Ok(_) => errors.push(format!("{}: top-level JSON value must be an object", path.display())),
                Err(error) => errors.push(format!("{}: invalid JSON: {error}", path.display())),
            },
            Err(error) => errors.push(format!("cannot read {}: {error}", path.display())),
        }
    }
    documents
}

fn load_manifest(workspace: &Path, errors: &mut Vec<String>) -> BTreeMap<String, usize> {
    let path = workspace.join("apps/leptos/static/content-manifest.json");
    let text = match fs::read_to_string(&path) {
        Ok(text) => text,
        Err(error) => {
            errors.push(format!("cannot read {}: {error}", path.display()));
            return BTreeMap::new();
        }
    };
    let value: Value = match serde_json::from_str(&text) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{}: invalid JSON: {error}", path.display()));
            return BTreeMap::new();
        }
    };
    let Some(lessons) = value.get("lessons").and_then(Value::as_object) else {
        errors.push(format!("{}: expected an object at key lessons", path.display()));
        return BTreeMap::new();
    };

    let mut result = BTreeMap::new();
    for (node_id, assets) in lessons {
        match assets.as_array() {
            Some(assets) => {
                result.insert(node_id.clone(), assets.len());
            }
            None => errors.push(format!(
                "{}: lesson assets for node ID {node_id:?} must be an array",
                path.display()
            )),
        }
    }
    result
}


fn add_digest_file(
    workspace: &Path,
    path: &Path,
    inputs: &mut BTreeMap<String, Vec<u8>>,
    errors: &mut Vec<String>,
) {
    let canonical = match path.canonicalize() {
        Ok(path) => path,
        Err(error) => {
            errors.push(format!("cannot resolve digest input {}: {error}", path.display()));
            return;
        }
    };
    if !canonical.starts_with(workspace) {
        errors.push(format!("digest input escapes workspace root: {}", path.display()));
        return;
    }
    let relative = match canonical.strip_prefix(workspace) {
        Ok(path) => path.to_string_lossy().replace('\\', "/"),
        Err(error) => {
            errors.push(format!("cannot make digest path relative: {error}"));
            return;
        }
    };
    match fs::read(&canonical) {
        Ok(bytes) => {
            inputs.insert(relative, bytes);
        }
        Err(error) => errors.push(format!("cannot read digest input {}: {error}", canonical.display())),
    }
}

fn collect_json_inputs(
    workspace: &Path,
    directory: &Path,
    inputs: &mut BTreeMap<String, Vec<u8>>,
    errors: &mut Vec<String>,
) {
    let entries = match fs::read_dir(directory) {
        Ok(entries) => entries,
        Err(error) => {
            errors.push(format!("cannot read generated lesson directory {}: {error}", directory.display()));
            return;
        }
    };
    for entry in entries {
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                errors.push(format!("cannot enumerate {}: {error}", directory.display()));
                continue;
            }
        };
        let path = entry.path();
        let kind = match entry.file_type() {
            Ok(kind) => kind,
            Err(error) => {
                errors.push(format!("cannot inspect {}: {error}", path.display()));
                continue;
            }
        };
        if kind.is_dir() {
            collect_json_inputs(workspace, &path, inputs, errors);
        } else if kind.is_symlink() {
            errors.push(format!("symlinks are not accepted in generated lesson inputs: {}", path.display()));
        } else if kind.is_file() && path.extension().and_then(|ext| ext.to_str()) == Some("json") {
            add_digest_file(workspace, &path, inputs, errors);
        }
    }
}

fn hash_named_inputs(inputs: &BTreeMap<String, Vec<u8>>) -> String {
    let mut hasher = blake3::Hasher::new();
    for (path, bytes) in inputs {
        hasher.update(&(path.len() as u64).to_le_bytes());
        hasher.update(path.as_bytes());
        hasher.update(&(bytes.len() as u64).to_le_bytes());
        hasher.update(bytes);
    }
    hasher.finalize().to_hex().to_string()
}

fn compute_input_digest(
    workspace: &Path,
    documents: &[(PathBuf, Value)],
    errors: &mut Vec<String>,
) -> Option<String> {
    let mut inputs = BTreeMap::new();
    add_digest_file(
        workspace,
        &workspace.join("apps/leptos/src/curriculum.rs"),
        &mut inputs,
        errors,
    );
    add_digest_file(
        workspace,
        &workspace.join("apps/leptos/static/content-manifest.json"),
        &mut inputs,
        errors,
    );
    for (path, _) in documents {
        add_digest_file(workspace, path, &mut inputs, errors);
    }
    collect_json_inputs(
        workspace,
        &workspace.join("examples/curriculum/caps/generated"),
        &mut inputs,
        errors,
    );
    if errors.is_empty() {
        Some(hash_named_inputs(&inputs))
    } else {
        None
    }
}

fn analyze(
    documents: &[(PathBuf, Value)],
    manifest: &BTreeMap<String, usize>,
    errors: &mut Vec<String>,
) -> AuditReport {
    let mut nodes: BTreeMap<String, NodeInfo> = BTreeMap::new();
    let mut edges: Vec<(PathBuf, String, String)> = Vec::new();
    let mut standards_all = BTreeSet::new();

    for (path, document) in documents {
        let metadata = document.get("metadata");
        let Some(raw_nodes) = document.get("nodes").and_then(Value::as_array) else {
            errors.push(format!("{}: nodes must be an array", path.display()));
            continue;
        };
        let raw_edges = match document.get("edges") {
            None => None,
            Some(value) => match value.as_array() {
                Some(edges) => Some(edges),
                None => {
                    errors.push(format!("{}: edges must be an array when present", path.display()));
                    None
                }
            },
        };

        for (index, node) in raw_nodes.iter().enumerate() {
            let Some(node_id) = nonempty_string(node.get("id")) else {
                errors.push(format!("{}: node {index} has no non-empty string ID", path.display()));
                continue;
            };
            if nodes.contains_key(&node_id) {
                errors.push(format!("duplicate active node ID {node_id:?} in an additional document: {}", path.display()));
                continue;
            }

            let grades = {
                let declared = string_array(node.get("grade_levels"));
                if declared.is_empty() {
                    vec![nonempty_string(metadata.and_then(|m| m.get("grade_level")))
                        .unwrap_or_else(|| "Unspecified".to_owned())]
                } else {
                    declared
                }
            };
            let subject = nonempty_string(node.get("subject_area"))
                .or_else(|| nonempty_string(metadata.and_then(|m| m.get("subject_area"))))
                .unwrap_or_else(|| "Unspecified".to_owned());

            let standards_array = node.get("academic_standards").and_then(Value::as_array);
            let mut standards_by_framework: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
            if let Some(items) = standards_array {
                for standard in items {
                    if let (Some(framework), Some(code)) = (
                        nonempty_string(standard.get("framework")),
                        nonempty_string(standard.get("code")),
                    ) {
                        let reference = format!("{framework}::{code}");
                        standards_all.insert(reference.clone());
                        standards_by_framework
                            .entry(framework)
                            .or_default()
                            .insert(reference);
                    }
                }
            }
            let mut frameworks: BTreeSet<String> =
                standards_by_framework.keys().cloned().collect();
            if frameworks.is_empty() {
                frameworks.insert(
                    nonempty_string(metadata.and_then(|m| m.get("framework")))
                        .unwrap_or_else(|| "Unmapped".to_owned()),
                );
            }

            nodes.insert(node_id.clone(), NodeInfo {
                grades,
                subject,
                frameworks,
                standards_by_framework,
                lesson_assets: manifest.get(&node_id).copied().unwrap_or(0),
            });
        }

        if let Some(raw_edges) = raw_edges {
            for (index, edge) in raw_edges.iter().enumerate() {
                let (Some(from), Some(to)) = (
                    nonempty_string(edge.get("from")),
                    nonempty_string(edge.get("to")),
                ) else {
                    errors.push(format!("{}: edge {index} must contain non-empty string from/to IDs", path.display()));
                    continue;
                };
                edges.push((path.clone(), from, to));
            }
        }
    }

    for (path, from, to) in &edges {
        if !nodes.contains_key(from) || !nodes.contains_key(to) {
            let mut missing = Vec::new();
            if !nodes.contains_key(from) { missing.push(format!("from={from:?}")); }
            if !nodes.contains_key(to) { missing.push(format!("to={to:?}")); }
            errors.push(format!(
                "{}: dangling edge {from:?} -> {to:?}; missing {}",
                path.display(), missing.join(", ")
            ));
        }
    }

    let active_ids: BTreeSet<String> = nodes.keys().cloned().collect();
    let lesson_ids: BTreeSet<String> = manifest.iter()
        .filter(|(_, count)| **count > 0)
        .map(|(id, _)| id.clone())
        .collect();
    let orphan_ids = lesson_ids.difference(&active_ids).cloned().collect::<Vec<_>>();
    let active_with_lessons = active_ids.intersection(&lesson_ids).count();
    let active_missing_lessons = active_ids.difference(&lesson_ids).count();
    let asset_count = manifest.values().sum();

    let nodes_with_standards = nodes
        .values()
        .filter(|node| !node.standards_by_framework.is_empty())
        .count();
    let mut group_map: BTreeMap<GroupKey, GroupAccumulator> = BTreeMap::new();
    for node in nodes.values() {
        for framework in &node.frameworks {
            for grade in &node.grades {
                let key = GroupKey {
                    framework: framework.clone(),
                    grade_level: grade.clone(),
                    subject_area: node.subject.clone(),
                };
                let group = group_map.entry(key).or_default();
                group.nodes += 1;
                group.lesson_assets += node.lesson_assets;
                if node.lesson_assets > 0 {
                    group.nodes_with_lessons += 1;
                } else {
                    group.nodes_missing_lessons += 1;
                }
                if let Some(standards) = node.standards_by_framework.get(framework) {
                    if !standards.is_empty() {
                        group.nodes_with_standards += 1;
                        group.standards.extend(standards.iter().cloned());
                    }
                }
            }
        }
    }

    let groups = group_map.into_iter().map(|(key, group)| CoverageGroup {
        framework: key.framework,
        grade_level: key.grade_level,
        subject_area: key.subject_area,
        active_nodes: group.nodes,
        nodes_with_standard_refs: group.nodes_with_standards,
        nodes_without_standard_refs: group.nodes - group.nodes_with_standards,
        distinct_standard_refs: group.standards.len(),
        nodes_with_lessons: group.nodes_with_lessons,
        nodes_missing_lessons: group.nodes_missing_lessons,
        lesson_assets: group.lesson_assets,
    }).collect();

    AuditReport {
        schema_version: SCHEMA_VERSION,
        active_document_count: documents.len(),
        active_node_count: nodes.len(),
        active_edge_count: edges.len(),
        active_nodes_with_standard_refs: nodes_with_standards,
        active_nodes_without_standard_refs: nodes.len() - nodes_with_standards,
        distinct_standard_refs: standards_all.len(),
        manifest_node_id_count: manifest.len(),
        manifest_asset_count: asset_count,
        active_nodes_with_lessons: active_with_lessons,
        active_nodes_missing_lessons: active_missing_lessons,
        orphan_manifest_node_ids: orphan_ids,
        groups,
        input_digest_blake3: String::new(),
    }
}

fn markdown(report: &AuditReport) -> String {
    let mut out = String::new();
    out.push_str("# Praxis curriculum coverage audit\n\n");
    out.push_str("> Coverage is not proof of lesson quality, learning efficacy, or accreditation. Counts use the active embedded graph and checked-in content manifest.\n\n");
    out.push_str(&format!("Input digest (BLAKE3): {}\n\n", report.input_digest_blake3));
    out.push_str("| Metric | Count |\n|---|---:|\n");
    out.push_str(&format!("| Active curriculum documents | {} |\n", report.active_document_count));
    out.push_str(&format!("| Active nodes | {} |\n", report.active_node_count));
    out.push_str(&format!("| Active edges | {} |\n", report.active_edge_count));
    out.push_str(&format!("| Nodes with standard references | {} |\n", report.active_nodes_with_standard_refs));
    out.push_str(&format!("| Nodes without standard references | {} |\n", report.active_nodes_without_standard_refs));
    out.push_str(&format!("| Distinct referenced standards | {} |\n", report.distinct_standard_refs));
    out.push_str(&format!("| Generated lesson assets | {} |\n", report.manifest_asset_count));
    out.push_str(&format!("| Active nodes with lessons | {} |\n", report.active_nodes_with_lessons));
    out.push_str(&format!("| Active nodes missing lessons | {} |\n", report.active_nodes_missing_lessons));
    out.push_str(&format!("| Orphan lesson-target IDs | {} |\n\n", report.orphan_manifest_node_ids.len()));
    out.push_str("| Framework | Grade | Subject | Nodes | With standards | With lessons | Missing lessons | Assets |\n");
    out.push_str("|---|---|---|---:|---:|---:|---:|---:|\n");
    for group in &report.groups {
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} |\n",
            group.framework, group.grade_level, group.subject_area, group.active_nodes,
            group.nodes_with_standard_refs, group.nodes_with_lessons,
            group.nodes_missing_lessons, group.lesson_assets
        ));
    }
    if !report.orphan_manifest_node_ids.is_empty() {
        out.push_str("\n## Orphan lesson-target IDs\n\n");
        for id in &report.orphan_manifest_node_ids {
            out.push_str(&format!("- {id}\n"));
        }
    }
    out
}

fn workspace_root() -> Result<PathBuf, String> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .map(Path::to_path_buf)
        .ok_or_else(|| "cannot locate mycelix-praxis workspace root from CARGO_MANIFEST_DIR".to_owned())?;
    path.canonicalize().map_err(|error| format!("cannot canonicalize workspace root: {error}"))
}

fn run() -> Result<(), AuditFailure> {
    let mut output_format = "json";
    for arg in env::args().skip(1) {
        match arg.as_str() {
            "--format=json" | "--json" => output_format = "json",
            "--format=markdown" | "--markdown" => output_format = "markdown",
            "--help" | "-h" => {
                println!("praxis-curriculum-audit [--format=json|markdown]\nRead-only audit of active graph integrity and lesson coverage.");
                return Ok(());
            }
            other => {
                eprintln!("unknown argument: {other}; use --help");
                return Err(AuditFailure(Vec::new()));
            }
        }
    }
    let workspace = workspace_root().map_err(|error| AuditFailure(vec![error]))?;
    let mut errors = Vec::new();
    let documents = active_documents(&workspace, &mut errors);
    let manifest = load_manifest(&workspace, &mut errors);
    let input_digest = compute_input_digest(&workspace, &documents, &mut errors);
    let mut report = analyze(&documents, &manifest, &mut errors);
    if let Some(digest) = input_digest {
        report.input_digest_blake3 = digest;
    }
    if !errors.is_empty() {
        for error in &errors { eprintln!("curriculum audit error: {error}"); }
        return Err(AuditFailure(errors));
    }
    match output_format {
        "json" => println!("{}", serde_json::to_string_pretty(&report)
            .map_err(|error| AuditFailure(vec![format!("cannot serialize report: {error}")]))?),
        "markdown" => print!("{}", markdown(&report)),
        _ => unreachable!("output format is constrained by argument parsing"),
    }
    Ok(())
}

pub fn run_cli() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(AuditFailure(errors)) if errors.is_empty() => ExitCode::from(2),
        Err(_) => ExitCode::FAILURE,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_document() -> Value {
        serde_json::json!({
            "metadata": {
                "framework": "TestFramework",
                "grade_level": "Grade3",
                "subject_area": "Mathematics"
            },
            "nodes": [
                {
                    "id": "n1",
                    "grade_levels": ["Grade3"],
                    "subject_area": "Mathematics",
                    "academic_standards": [{"framework":"TestFramework","code":"MATH.1"}]
                },
                {
                    "id": "n2",
                    "grade_levels": ["Grade3"],
                    "subject_area": "Mathematics",
                    "academic_standards": []
                }
            ],
            "edges": [{"from":"n1","to":"n2"}]
        })
    }

    #[test]
    fn lesson_evidence_schema_is_versioned_json() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../../schemas/lesson-evidence-record-v1.schema.json"
        ))
        .expect("lesson evidence schema must be valid JSON");

        assert_eq!(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema"
        );
        assert_eq!(schema["properties"]["schema_version"]["const"], 1);
        assert!(
            schema["properties"]["lesson_digest_blake3"]
                .is_object()
        );
        assert!(
            schema["properties"]["learning_studies"]
                .is_object()
        );
    }

    #[test]
    fn extracts_only_json_include_paths() {
        let source = "include_str!(\"a.json\"); include_str!(\"b.svg\"); include_str!(\"nested/c.json\");";
        assert_eq!(embedded_json_paths(source), vec!["a.json".to_owned(), "nested/c.json".to_owned()]);
    }

    #[test]
    fn reports_coverage_and_orphans() {
        let docs = vec![(PathBuf::from("fixture.json"), fixture_document())];
        let manifest = BTreeMap::from([
            ("n1".to_owned(), 2usize),
            ("orphan".to_owned(), 1usize),
        ]);
        let mut errors = Vec::new();
        let report = analyze(&docs, &manifest, &mut errors);
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(report.active_node_count, 2);
        assert_eq!(report.active_edge_count, 1);
        assert_eq!(report.active_nodes_with_standard_refs, 1);
        assert_eq!(report.distinct_standard_refs, 1);
        assert_eq!(report.manifest_asset_count, 3);
        assert_eq!(report.active_nodes_with_lessons, 1);
        assert_eq!(report.active_nodes_missing_lessons, 1);
        assert_eq!(report.orphan_manifest_node_ids, vec!["orphan".to_owned()]);
        assert_eq!(report.groups.len(), 1);
        assert_eq!(report.groups[0].nodes_missing_lessons, 1);
        assert_eq!(report.groups[0].nodes_without_standard_refs, 1);
    }

    #[test]
    fn duplicate_node_ids_are_rejected() {
        let docs = vec![
            (PathBuf::from("a.json"), serde_json::json!({"nodes":[{"id":"same"}],"edges":[]})),
            (PathBuf::from("b.json"), serde_json::json!({"nodes":[{"id":"same"}],"edges":[]})),
        ];
        let mut errors = Vec::new();
        let _ = analyze(&docs, &BTreeMap::new(), &mut errors);
        assert!(errors.iter().any(|error| error.contains("duplicate active node ID")));
    }

    #[test]
    fn dangling_edges_are_rejected() {
        let docs = vec![(
            PathBuf::from("a.json"),
            serde_json::json!({"nodes":[{"id":"present"}],"edges":[{"from":"present","to":"missing"}]}),
        )];
        let mut errors = Vec::new();
        let _ = analyze(&docs, &BTreeMap::new(), &mut errors);
        assert!(errors.iter().any(|error| error.contains("dangling edge")));
    }

    #[test]
    fn digest_changes_with_path_or_content_and_is_order_stable() {
        let mut first = BTreeMap::new();
        first.insert("a.json".to_owned(), b"content-a".to_vec());
        first.insert("b.json".to_owned(), b"content-b".to_vec());

        let mut same_inputs_reverse_insert = BTreeMap::new();
        same_inputs_reverse_insert.insert("b.json".to_owned(), b"content-b".to_vec());
        same_inputs_reverse_insert.insert("a.json".to_owned(), b"content-a".to_vec());
        assert_eq!(hash_named_inputs(&first), hash_named_inputs(&same_inputs_reverse_insert));

        let mut changed_content = first.clone();
        changed_content.insert("a.json".to_owned(), b"tampered".to_vec());
        assert_ne!(hash_named_inputs(&first), hash_named_inputs(&changed_content));
    }

    #[test]
    fn markdown_describes_the_limit_of_the_report() {
        let docs = vec![(PathBuf::from("fixture.json"), fixture_document())];
        let manifest = BTreeMap::from([("n1".to_owned(), 1usize)]);
        let mut errors = Vec::new();
        let report = analyze(&docs, &manifest, &mut errors);
        assert!(errors.is_empty(), "{errors:?}");
        let rendered = markdown(&report);
        assert!(rendered.contains("not proof of lesson quality"));
        assert!(rendered.contains("| TestFramework | Grade3 | Mathematics |"));
    }
}
