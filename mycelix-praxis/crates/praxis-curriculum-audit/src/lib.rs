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
struct ContentQualityFinding {
    lesson_id: String,
    asset_path: String,
    field: String,
    code: String,
    severity: String,
    message: String,
}

#[derive(Debug, Default)]
struct ContentQualityAudit {
    assets_scanned: usize,
    assets_with_findings: usize,
    findings: Vec<ContentQualityFinding>,
}

#[derive(Debug, Serialize)]
struct InactiveSourceReference {
    path: String,
    title: String,
    framework: String,
    grade_level: String,
    subject_area: String,
}

#[derive(Debug, Serialize)]
struct InactiveManifestSourceMatch {
    node_id: String,
    sources: Vec<InactiveSourceReference>,
}

#[derive(Debug, Default)]
struct InactiveSourceAudit {
    graph_documents_scanned: usize,
    orphan_ids_found_in_inactive_graphs: usize,
    orphan_ids_without_source_graph: Vec<String>,
    source_matches: Vec<InactiveManifestSourceMatch>,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    schema_version: u32,
    content_assets_scanned: usize,
    content_assets_with_quality_findings: usize,
    content_quality_findings: Vec<ContentQualityFinding>,
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
    inactive_source_graph_documents_scanned: usize,
    orphan_manifest_ids_found_in_inactive_graphs: usize,
    orphan_manifest_ids_without_source_graph: Vec<String>,
    inactive_manifest_source_matches: Vec<InactiveManifestSourceMatch>,
    groups: Vec<CoverageGroup>,
    input_digest_blake3: String,
}

#[derive(Debug)]
struct AuditFailure(Vec<String>);

fn nonempty_string(value: Option<&Value>) -> Option<String> {
    value
        .and_then(Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .map(ToOwned::to_owned)
}

fn string_array(value: Option<&Value>) -> Vec<String> {
    value
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .map(ToOwned::to_owned)
        .collect()
}

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
        errors.push(format!(
            "{}: no embedded JSON curriculum documents found",
            source_path.display()
        ));
        return Vec::new();
    }

    let mut seen = BTreeSet::new();
    let mut documents = Vec::new();
    for relative in paths {
        let unresolved = parent.join(&relative);
        let path = match unresolved.canonicalize() {
            Ok(path) => path,
            Err(error) => {
                errors.push(format!(
                    "cannot resolve curriculum include {relative:?}: {error}"
                ));
                continue;
            }
        };
        if !path.starts_with(workspace) {
            errors.push(format!(
                "curriculum include escapes workspace root: {relative}"
            ));
            continue;
        }
        if !seen.insert(path.clone()) {
            errors.push(format!(
                "curriculum document included more than once: {relative}"
            ));
            continue;
        }
        match fs::read_to_string(&path) {
            Ok(text) => match serde_json::from_str::<Value>(&text) {
                Ok(value) if value.is_object() => documents.push((path, value)),
                Ok(_) => errors.push(format!(
                    "{}: top-level JSON value must be an object",
                    path.display()
                )),
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
        errors.push(format!(
            "{}: expected an object at key lessons",
            path.display()
        ));
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
            errors.push(format!(
                "cannot resolve digest input {}: {error}",
                path.display()
            ));
            return;
        }
    };
    if !canonical.starts_with(workspace) {
        errors.push(format!(
            "digest input escapes workspace root: {}",
            path.display()
        ));
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
        Err(error) => errors.push(format!(
            "cannot read digest input {}: {error}",
            canonical.display()
        )),
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
            errors.push(format!(
                "cannot read generated lesson directory {}: {error}",
                directory.display()
            ));
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
            errors.push(format!(
                "symlinks are not accepted in generated lesson inputs: {}",
                path.display()
            ));
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
        &workspace.join("examples/curriculum"),
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
                    errors.push(format!(
                        "{}: edges must be an array when present",
                        path.display()
                    ));
                    None
                }
            },
        };

        for (index, node) in raw_nodes.iter().enumerate() {
            let Some(node_id) = nonempty_string(node.get("id")) else {
                errors.push(format!(
                    "{}: node {index} has no non-empty string ID",
                    path.display()
                ));
                continue;
            };
            if nodes.contains_key(&node_id) {
                errors.push(format!(
                    "duplicate active node ID {node_id:?} in an additional document: {}",
                    path.display()
                ));
                continue;
            }

            let grades = {
                let declared = string_array(node.get("grade_levels"));
                if declared.is_empty() {
                    vec![
                        nonempty_string(metadata.and_then(|m| m.get("grade_level")))
                            .unwrap_or_else(|| "Unspecified".to_owned()),
                    ]
                } else {
                    declared
                }
            };
            let subject = nonempty_string(node.get("subject_area"))
                .or_else(|| nonempty_string(metadata.and_then(|m| m.get("subject_area"))))
                .unwrap_or_else(|| "Unspecified".to_owned());

            let standards_array = match node.get("academic_standards") {
                None => None,
                Some(Value::Array(items)) => Some(items),
                Some(_) => {
                    errors.push(format!(
                        "{}: node {node_id:?} academic_standards must be an array",
                        path.display()
                    ));
                    None
                }
            };
            let mut standards_by_framework: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
            if let Some(items) = standards_array {
                for (standard_index, standard) in items.iter().enumerate() {
                    let Some(standard_object) = standard.as_object() else {
                        errors.push(format!(
                            "{}: node {node_id:?} standard {standard_index} must be an object",
                            path.display()
                        ));
                        continue;
                    };
                    let framework = nonempty_string(standard_object.get("framework"));
                    let code = nonempty_string(standard_object.get("code"));
                    let (Some(framework), Some(code)) = (framework, code) else {
                        errors.push(format!(
                            "{}: node {node_id:?} standard {standard_index} requires framework/code",
                            path.display()
                        ));
                        continue;
                    };
                    let reference = format!("{framework}::{code}");
                    standards_all.insert(reference.clone());
                    standards_by_framework
                        .entry(framework)
                        .or_default()
                        .insert(reference);
                }
            }
            let mut frameworks: BTreeSet<String> = standards_by_framework.keys().cloned().collect();
            if frameworks.is_empty() {
                frameworks.insert(
                    nonempty_string(metadata.and_then(|m| m.get("framework")))
                        .unwrap_or_else(|| "Unmapped".to_owned()),
                );
            }

            nodes.insert(
                node_id.clone(),
                NodeInfo {
                    grades,
                    subject,
                    frameworks,
                    standards_by_framework,
                    lesson_assets: manifest.get(&node_id).copied().unwrap_or(0),
                },
            );
        }

        if let Some(raw_edges) = raw_edges {
            for (index, edge) in raw_edges.iter().enumerate() {
                let (Some(from), Some(to)) = (
                    nonempty_string(edge.get("from")),
                    nonempty_string(edge.get("to")),
                ) else {
                    errors.push(format!(
                        "{}: edge {index} must contain non-empty string from/to IDs",
                        path.display()
                    ));
                    continue;
                };
                edges.push((path.clone(), from, to));
            }
        }
    }

    for (path, from, to) in &edges {
        if !nodes.contains_key(from) || !nodes.contains_key(to) {
            let mut missing = Vec::new();
            if !nodes.contains_key(from) {
                missing.push(format!("from={from:?}"));
            }
            if !nodes.contains_key(to) {
                missing.push(format!("to={to:?}"));
            }
            errors.push(format!(
                "{}: dangling edge {from:?} -> {to:?}; missing {}",
                path.display(),
                missing.join(", ")
            ));
        }
    }

    let active_ids: BTreeSet<String> = nodes.keys().cloned().collect();
    let lesson_ids: BTreeSet<String> = manifest
        .iter()
        .filter(|(_, count)| **count > 0)
        .map(|(id, _)| id.clone())
        .collect();
    let orphan_ids = lesson_ids
        .difference(&active_ids)
        .cloned()
        .collect::<Vec<_>>();
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
                if let Some(standards) = node.standards_by_framework.get(framework)
                    && !standards.is_empty()
                {
                    group.nodes_with_standards += 1;
                    group.standards.extend(standards.iter().cloned());
                }
            }
        }
    }

    let groups = group_map
        .into_iter()
        .map(|(key, group)| CoverageGroup {
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
        })
        .collect();

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
        inactive_source_graph_documents_scanned: 0,
        orphan_manifest_ids_found_in_inactive_graphs: 0,
        orphan_manifest_ids_without_source_graph: Vec::new(),
        inactive_manifest_source_matches: Vec::new(),
        groups,
        input_digest_blake3: String::new(),
        content_assets_scanned: 0,
        content_assets_with_quality_findings: 0,
        content_quality_findings: Vec::new(),
    }
}

fn collect_curriculum_json_paths(
    directory: &Path,
    workspace: &Path,
    excluded_root: &Path,
    output: &mut Vec<PathBuf>,
    errors: &mut Vec<String>,
) {
    let entries = match fs::read_dir(directory) {
        Ok(entries) => entries,
        Err(error) => {
            errors.push(format!(
                "cannot enumerate curriculum source directory {}: {error}",
                directory.display()
            ));
            return;
        }
    };

    for entry in entries {
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                errors.push(format!(
                    "cannot enumerate curriculum source directory {}: {error}",
                    directory.display()
                ));
                continue;
            }
        };
        let path = entry.path();
        let kind = match entry.file_type() {
            Ok(kind) => kind,
            Err(error) => {
                errors.push(format!(
                    "cannot inspect curriculum source path {}: {error}",
                    path.display()
                ));
                continue;
            }
        };

        if kind.is_dir() {
            let canonical = match path.canonicalize() {
                Ok(path) => path,
                Err(error) => {
                    errors.push(format!(
                        "cannot resolve curriculum source directory {}: {error}",
                        path.display()
                    ));
                    continue;
                }
            };
            if !canonical.starts_with(workspace) {
                errors.push(format!(
                    "curriculum source directory escapes workspace root: {}",
                    path.display()
                ));
                continue;
            }
            if !canonical.starts_with(excluded_root) {
                collect_curriculum_json_paths(
                    &canonical,
                    workspace,
                    excluded_root,
                    output,
                    errors,
                );
            }
        } else if kind.is_file()
            && path.extension().and_then(|value| value.to_str()) == Some("json")
        {
            let canonical = match path.canonicalize() {
                Ok(path) => path,
                Err(error) => {
                    errors.push(format!(
                        "cannot resolve curriculum source file {}: {error}",
                        path.display()
                    ));
                    continue;
                }
            };
            if !canonical.starts_with(workspace) {
                errors.push(format!(
                    "curriculum source file escapes workspace root: {}",
                    path.display()
                ));
                continue;
            }
            if !canonical.starts_with(excluded_root) {
                output.push(canonical);
            }
        }
    }
}

fn match_inactive_manifest_sources(
    active_ids: &BTreeSet<String>,
    manifest: &BTreeMap<String, usize>,
    documents: &[(String, Value)],
) -> InactiveSourceAudit {
    let orphan_ids: BTreeSet<String> = manifest
        .iter()
        .filter(|(id, count)| **count > 0 && !active_ids.contains(*id))
        .map(|(id, _)| id.clone())
        .collect();
    let mut by_node = BTreeMap::<String, BTreeMap<String, InactiveSourceReference>>::new();
    let mut graph_documents_scanned = 0;

    for (path, document) in documents {
        let Some(nodes) = document.get("nodes").and_then(Value::as_array) else {
            continue;
        };
        graph_documents_scanned += 1;
        let metadata = document.get("metadata");
        let framework = nonempty_string(metadata.and_then(|value| value.get("framework")))
            .unwrap_or_else(|| "Unspecified".to_owned());
        let default_grade = nonempty_string(metadata.and_then(|value| value.get("grade_level")))
            .unwrap_or_else(|| "Unspecified".to_owned());
        let default_subject = nonempty_string(metadata.and_then(|value| value.get("subject_area")))
            .unwrap_or_else(|| "Unspecified".to_owned());
        for node in nodes {
            let Some(node_id) = nonempty_string(node.get("id")) else {
                continue;
            };
            if !orphan_ids.contains(&node_id) {
                continue;
            }
            let reference = InactiveSourceReference {
                path: path.clone(),
                title: nonempty_string(node.get("title"))
                    .or_else(|| nonempty_string(node.get("name")))
                    .unwrap_or_else(|| "Untitled source node".to_owned()),
                framework: nonempty_string(node.get("framework"))
                    .unwrap_or_else(|| framework.clone()),
                grade_level: string_array(node.get("grade_levels"))
                    .first()
                    .cloned()
                    .or_else(|| nonempty_string(node.get("grade_level")))
                    .unwrap_or_else(|| default_grade.clone()),
                subject_area: nonempty_string(node.get("subject_area"))
                    .unwrap_or_else(|| default_subject.clone()),
            };
            by_node
                .entry(node_id)
                .or_default()
                .entry(reference.path.clone())
                .or_insert(reference);
        }
    }

    let source_matches = by_node
        .into_iter()
        .map(|(node_id, sources)| InactiveManifestSourceMatch {
            node_id,
            sources: sources.into_values().collect(),
        })
        .collect::<Vec<_>>();
    let matched_ids: BTreeSet<String> = source_matches
        .iter()
        .map(|matched| matched.node_id.clone())
        .collect();
    let orphan_ids_without_source_graph = orphan_ids
        .difference(&matched_ids)
        .cloned()
        .collect::<Vec<_>>();

    InactiveSourceAudit {
        graph_documents_scanned,
        orphan_ids_found_in_inactive_graphs: source_matches.len(),
        orphan_ids_without_source_graph,
        source_matches,
    }
}

fn audit_inactive_manifest_sources(
    workspace: &Path,
    active_documents: &[(PathBuf, Value)],
    manifest: &BTreeMap<String, usize>,
    errors: &mut Vec<String>,
) -> InactiveSourceAudit {
    let curriculum_root = workspace.join("examples/curriculum");
    let excluded_root = workspace.join("examples/curriculum/caps/generated");
    let active_paths: BTreeSet<PathBuf> = active_documents
        .iter()
        .map(|(path, _)| path.clone())
        .collect();
    let active_ids: BTreeSet<String> = active_documents
        .iter()
        .filter_map(|(_, document)| document.get("nodes").and_then(Value::as_array))
        .flat_map(|nodes| nodes.iter().filter_map(|node| nonempty_string(node.get("id"))))
        .collect();

    let mut paths = Vec::new();
    collect_curriculum_json_paths(
        &curriculum_root,
        workspace,
        &excluded_root,
        &mut paths,
        errors,
    );
    paths.sort();
    paths.dedup();

    let mut documents = Vec::new();
    for path in paths {
        if active_paths.contains(&path) {
            continue;
        }
        let raw = match fs::read_to_string(&path) {
            Ok(raw) => raw,
            Err(error) => {
                errors.push(format!(
                    "cannot read inactive curriculum source {}: {error}",
                    path.display()
                ));
                continue;
            }
        };
        let value: Value = match serde_json::from_str(&raw) {
            Ok(value) => value,
            Err(_) => continue,
        };
        if !value.get("nodes").is_some_and(Value::is_array) {
            continue;
        }
        let relative = match path.strip_prefix(workspace) {
            Ok(path) => path.to_string_lossy().replace('\\', "/"),
            Err(error) => {
                errors.push(format!(
                    "cannot relativize inactive curriculum source {}: {error}",
                    path.display()
                ));
                continue;
            }
        };
        documents.push((relative, value));
    }

    match_inactive_manifest_sources(&active_ids, manifest, &documents)
}

fn normalize_content(text: &str) -> String {
    text.split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .trim()
        .to_lowercase()
}

fn is_known_instructional_placeholder(text: &str) -> bool {
    let normalized = normalize_content(text);
    if normalized.is_empty() {
        return true;
    }
    let core = normalized.trim_end_matches(['.', '!', '?']).trim();

    let exact_placeholders = [
        "grade 3 mathematics content",
        "a common mistake in grade 3 mathematics",
        "the correct way",
        "used in grade 3 maths",
        "lorem ipsum",
        "todo",
        "placeholder",
        "answer",
    ];
    if exact_placeholders.contains(&core) {
        return true;
    }

    let template_phrases = [
        "answer: see worked solution",
        "what is important about grade 3 mathematics?",
        "a key word in this topic",
        "try the method for grade 3 mathematics.",
        "start with what you know about grade 3 mathematics.",
        "let's explore this concept together!",
        "every expert was once a beginner.",
    ];

    template_phrases
        .iter()
        .any(|phrase| normalized.contains(phrase))
        || (normalized.contains("solve a grade ") && normalized.contains(" mathematics problem"))
        || (core.starts_with("explain grade ") && core.contains(" mathematics"))
        || (core.starts_with("grade ") && core.ends_with(" mathematics content"))
}
fn add_text_finding(
    audit: &mut ContentQualityAudit,
    lesson_id: &str,
    asset_path: &str,
    field: &str,
    value: Option<&Value>,
) {
    let Some(text) = value.and_then(Value::as_str) else {
        return;
    };
    if is_known_instructional_placeholder(text) {
        audit.findings.push(ContentQualityFinding {
            lesson_id: lesson_id.to_owned(),
            asset_path: asset_path.to_owned(),
            field: field.to_owned(),
            code: "generic_placeholder".to_owned(),
            severity: "blocker".to_owned(),
            message: format!(
                "Known generic fallback text appears in {field}; this field cannot count as instructional evidence."
            ),
        });
    }
}

fn add_required_text_finding(
    audit: &mut ContentQualityAudit,
    lesson_id: &str,
    asset_path: &str,
    field: &str,
    value: Option<&Value>,
) {
    if nonempty_string(value).is_none() {
        audit.findings.push(ContentQualityFinding {
            lesson_id: lesson_id.to_owned(),
            asset_path: asset_path.to_owned(),
            field: field.to_owned(),
            code: "missing_required_text".to_owned(),
            severity: "blocker".to_owned(),
            message: format!("Required instructional text is missing or empty: {field}."),
        });
        return;
    }

    add_text_finding(audit, lesson_id, asset_path, field, value);
}

fn add_structure_finding(
    audit: &mut ContentQualityAudit,
    lesson_id: &str,
    asset_path: &str,
    field: &str,
    code: &str,
    message: &str,
) {
    audit.findings.push(ContentQualityFinding {
        lesson_id: lesson_id.to_owned(),
        asset_path: asset_path.to_owned(),
        field: field.to_owned(),
        code: code.to_owned(),
        severity: "blocker".to_owned(),
        message: message.to_owned(),
    });
}

fn audit_lesson_value(asset_path: &str, value: &Value, audit: &mut ContentQualityAudit) -> bool {
    let lesson_id = nonempty_string(value.get("node_id"))
        .or_else(|| nonempty_string(value.pointer("/lesson/standard_code")))
        .unwrap_or_else(|| "UNSPECIFIED".to_owned());
    let before = audit.findings.len();

    let Some(lesson) = value.get("lesson").filter(|lesson| lesson.is_object()) else {
        add_structure_finding(
            audit,
            &lesson_id,
            asset_path,
            "lesson",
            "missing_lesson_object",
            "Asset must contain a lesson object.",
        );
        return true;
    };

    add_required_text_finding(
        audit,
        &lesson_id,
        asset_path,
        "lesson.explanation",
        lesson.get("explanation"),
    );
    add_required_text_finding(
        audit,
        &lesson_id,
        asset_path,
        "lesson.title",
        lesson.get("title"),
    );

    let examples_value = lesson.get("examples").and_then(Value::as_array);
    if examples_value.is_none_or(Vec::is_empty) {
        add_structure_finding(
            audit,
            &lesson_id,
            asset_path,
            "lesson.examples",
            "missing_worked_examples",
            "Lesson must include at least one worked example or demonstration.",
        );
    }
    if let Some(examples) = examples_value {
        let mut seen = BTreeMap::<String, usize>::new();
        for (index, example) in examples.iter().enumerate() {
            let prefix = format!("lesson.examples[{index}]");
            add_required_text_finding(
                audit,
                &lesson_id,
                asset_path,
                &format!("{prefix}.problem"),
                example.get("problem"),
            );
            add_required_text_finding(
                audit,
                &lesson_id,
                asset_path,
                &format!("{prefix}.answer"),
                example.get("answer"),
            );
            let steps_value = example.get("steps").and_then(Value::as_array);
            if steps_value.is_none_or(Vec::is_empty) {
                add_structure_finding(
                    audit,
                    &lesson_id,
                    asset_path,
                    &format!("{prefix}.steps"),
                    "missing_worked_steps",
                    "Worked example must include at least one explicit solution step.",
                );
            }
            if let Some(steps) = steps_value {
                for (step_index, step) in steps.iter().enumerate() {
                    add_required_text_finding(
                        audit,
                        &lesson_id,
                        asset_path,
                        &format!("{prefix}.steps[{step_index}].instruction"),
                        step.get("instruction"),
                    );
                    add_required_text_finding(
                        audit,
                        &lesson_id,
                        asset_path,
                        &format!("{prefix}.steps[{step_index}].result"),
                        step.get("result"),
                    );
                }
            }
            if let Some(prompt) = nonempty_string(example.get("problem")) {
                let key = normalize_content(&prompt);
                if !key.is_empty() {
                    *seen.entry(key).or_default() += 1;
                }
            }
        }
        for (prompt, count) in seen {
            if count > 1 {
                audit.findings.push(ContentQualityFinding {
                    lesson_id: lesson_id.clone(),
                    asset_path: asset_path.to_owned(),
                    field: "lesson.examples".to_owned(),
                    code: "duplicate_worked_example".to_owned(),
                    severity: "warning".to_owned(),
                    message: format!(
                        "The same normalized worked-example prompt occurs {count} times: {prompt}"
                    ),
                });
            }
        }
    }

    let problems_value = lesson.get("practice_problems").and_then(Value::as_array);
    if problems_value.is_none_or(Vec::is_empty) {
        add_structure_finding(
            audit,
            &lesson_id,
            asset_path,
            "lesson.practice_problems",
            "missing_practice_items",
            "Lesson must include at least one independent practice item.",
        );
    }
    if let Some(problems) = problems_value {
        let mut seen = BTreeMap::<String, usize>::new();
        for (index, problem) in problems.iter().enumerate() {
            let prefix = format!("lesson.practice_problems[{index}]");
            add_required_text_finding(
                audit,
                &lesson_id,
                asset_path,
                &format!("{prefix}.question"),
                problem.get("question"),
            );
            add_required_text_finding(
                audit,
                &lesson_id,
                asset_path,
                &format!("{prefix}.answer"),
                problem.get("answer"),
            );
            add_required_text_finding(
                audit,
                &lesson_id,
                asset_path,
                &format!("{prefix}.explanation"),
                problem.get("explanation"),
            );

            if let Some(answer) = nonempty_string(problem.get("answer")) {
                let answer_key = normalize_content(&answer)
                    .trim_end_matches(['.', '!', '?'])
                    .to_owned();
                if let Some(distractors) = problem.get("distractors").and_then(Value::as_array) {
                    let mut seen_distractors = BTreeSet::new();
                    for distractor in distractors.iter().filter_map(Value::as_str) {
                        let distractor_key = normalize_content(distractor)
                            .trim_end_matches(['.', '!', '?'])
                            .to_owned();
                        if distractor_key.is_empty() {
                            continue;
                        }
                        if !seen_distractors.insert(distractor_key.clone()) {
                            audit.findings.push(ContentQualityFinding {
                                lesson_id: lesson_id.clone(),
                                asset_path: asset_path.to_owned(),
                                field: format!("{prefix}.distractors"),
                                code: "duplicate_distractor".to_owned(),
                                severity: "warning".to_owned(),
                                message: format!(
                                    "The same normalized distractor appears more than once: {distractor:?}"
                                ),
                            });
                        }
                        if distractor_key == answer_key {
                            audit.findings.push(ContentQualityFinding {
                                lesson_id: lesson_id.clone(),
                                asset_path: asset_path.to_owned(),
                                field: format!("{prefix}.distractors"),
                                code: "answer_in_distractors".to_owned(),
                                severity: "blocker".to_owned(),
                                message: format!(
                                    "The correct answer {answer:?} is also listed as a distractor; multiple-choice feedback can become contradictory."
                                ),
                            });
                        }
                    }
                }
            }

            if let Some(prompt) = nonempty_string(problem.get("question")) {
                let key = normalize_content(&prompt);
                if !key.is_empty() {
                    *seen.entry(key).or_default() += 1;
                }
            }
        }
        for (prompt, count) in seen {
            if count > 1 {
                audit.findings.push(ContentQualityFinding {
                    lesson_id: lesson_id.clone(),
                    asset_path: asset_path.to_owned(),
                    field: "lesson.practice_problems".to_owned(),
                    code: "duplicate_practice_prompt".to_owned(),
                    severity: "warning".to_owned(),
                    message: format!(
                        "The same normalized practice prompt occurs {count} times: {prompt}"
                    ),
                });
            }
        }
    }

    if let Some(hints) = lesson.get("hints").and_then(Value::as_array) {
        for (index, hint) in hints.iter().enumerate() {
            add_text_finding(
                audit,
                &lesson_id,
                asset_path,
                &format!("lesson.hints[{index}]"),
                Some(hint),
            );
        }
    }
    if let Some(vocabulary) = lesson.get("key_vocabulary").and_then(Value::as_array) {
        for (index, item) in vocabulary.iter().enumerate() {
            for key in ["term", "definition", "example_usage"] {
                add_text_finding(
                    audit,
                    &lesson_id,
                    asset_path,
                    &format!("lesson.key_vocabulary[{index}].{key}"),
                    item.get(key),
                );
            }
        }
    }
    if let Some(misconceptions) = lesson.get("misconceptions").and_then(Value::as_array) {
        for (index, item) in misconceptions.iter().enumerate() {
            for key in ["misconception", "correction", "why_students_think_this"] {
                add_text_finding(
                    audit,
                    &lesson_id,
                    asset_path,
                    &format!("lesson.misconceptions[{index}].{key}"),
                    item.get(key),
                );
            }
        }
    }

    if let Some(items) = value.pointer("/assessment/items").and_then(Value::as_array) {
        for (index, item) in items.iter().enumerate() {
            for key in ["question", "correct_answer", "hint", "rubric"] {
                let field = format!("assessment.items[{index}].{key}");
                if matches!(key, "question" | "correct_answer") {
                    add_required_text_finding(audit, &lesson_id, asset_path, &field, item.get(key));
                } else {
                    add_text_finding(audit, &lesson_id, asset_path, &field, item.get(key));
                }
            }
        }
    }

    if let Some(cards) = value.get("flashcards").and_then(Value::as_array) {
        for (index, card) in cards.iter().enumerate() {
            for key in ["front", "back"] {
                add_required_text_finding(
                    audit,
                    &lesson_id,
                    asset_path,
                    &format!("flashcards[{index}].{key}"),
                    card.get(key),
                );
            }
        }
    }

    audit.findings.len() > before
}

fn audit_manifest_assets(workspace: &Path, errors: &mut Vec<String>) -> ContentQualityAudit {
    let manifest_path = workspace.join("apps/leptos/static/content-manifest.json");
    let manifest_text = match fs::read_to_string(&manifest_path) {
        Ok(text) => text,
        Err(error) => {
            errors.push(format!("cannot read {}: {error}", manifest_path.display()));
            return ContentQualityAudit::default();
        }
    };
    let manifest: Value = match serde_json::from_str(&manifest_text) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!(
                "{}: invalid JSON: {error}",
                manifest_path.display()
            ));
            return ContentQualityAudit::default();
        }
    };
    let Some(lessons) = manifest.get("lessons").and_then(Value::as_object) else {
        errors.push(format!(
            "{}: lessons must be an object",
            manifest_path.display()
        ));
        return ContentQualityAudit::default();
    };

    let lesson_root = workspace.join("examples/curriculum/caps/generated");
    let mut audit = ContentQualityAudit::default();
    let mut seen_paths = BTreeSet::new();
    for (node_id, assets) in lessons {
        let Some(assets) = assets.as_array() else {
            continue;
        };
        for asset in assets {
            let Some(uri) = nonempty_string(asset.get("path")) else {
                errors.push(format!("manifest entry for {node_id:?} has no asset path"));
                continue;
            };
            let Some(relative) = uri.strip_prefix("/curriculum/generated/") else {
                errors.push(format!(
                    "manifest asset path outside curriculum root: {uri}"
                ));
                continue;
            };
            let unresolved = lesson_root.join(relative);
            let path = match unresolved.canonicalize() {
                Ok(path) => path,
                Err(error) => {
                    errors.push(format!("cannot resolve manifest asset {uri}: {error}"));
                    continue;
                }
            };
            let root = match lesson_root.canonicalize() {
                Ok(root) => root,
                Err(error) => {
                    errors.push(format!(
                        "cannot resolve lesson root {}: {error}",
                        lesson_root.display()
                    ));
                    break;
                }
            };
            if !path.starts_with(&root) {
                errors.push(format!("manifest asset escapes lesson root: {uri}"));
                continue;
            }
            let relative_path = match path.strip_prefix(workspace) {
                Ok(path) => path.to_string_lossy().replace('\\', "/"),
                Err(error) => {
                    errors.push(format!("cannot relativize asset {uri}: {error}"));
                    continue;
                }
            };
            if !seen_paths.insert(relative_path.clone()) {
                continue;
            }
            let raw = match fs::read_to_string(&path) {
                Ok(raw) => raw,
                Err(error) => {
                    errors.push(format!("cannot read manifest asset {uri}: {error}"));
                    continue;
                }
            };
            let value: Value = match serde_json::from_str(&raw) {
                Ok(value) => value,
                Err(error) => {
                    errors.push(format!("manifest asset {uri}: invalid JSON: {error}"));
                    continue;
                }
            };
            audit.assets_scanned += 1;
            if audit_lesson_value(&relative_path, &value, &mut audit) {
                audit.assets_with_findings += 1;
            }

            // A manifest key and the embedded lesson's stable ID should agree.
            let actual_id = nonempty_string(value.get("node_id"))
                .or_else(|| nonempty_string(value.pointer("/lesson/standard_code")));
            if actual_id.as_deref() != Some(node_id.as_str()) {
                errors.push(format!(
                    "manifest key {node_id:?} does not match lesson ID {:?} in {uri}",
                    actual_id
                ));
            }
        }
    }
    audit
}

fn markdown(report: &AuditReport) -> String {
    let mut out = String::new();
    out.push_str("# Praxis curriculum coverage audit\n\n");
    out.push_str("> Coverage is not proof of lesson quality or learning efficacy.\n");
    out.push_str("> Counts use the active embedded graph and checked-in content manifest.\n\n");
    out.push_str(&format!(
        "Input digest (BLAKE3): {}\n\n",
        report.input_digest_blake3
    ));
    out.push_str("| Metric | Count |\n|---|---:|\n");
    out.push_str(&format!(
        "| Active curriculum documents | {} |\n",
        report.active_document_count
    ));
    out.push_str(&format!(
        "| Active nodes | {} |\n",
        report.active_node_count
    ));
    out.push_str(&format!(
        "| Active edges | {} |\n",
        report.active_edge_count
    ));
    out.push_str(&format!(
        "| Nodes with standard references | {} |\n",
        report.active_nodes_with_standard_refs
    ));
    out.push_str(&format!(
        "| Nodes without standard references | {} |\n",
        report.active_nodes_without_standard_refs
    ));
    out.push_str(&format!(
        "| Distinct referenced standards | {} |\n",
        report.distinct_standard_refs
    ));
    out.push_str(&format!(
        "| Generated lesson assets | {} |\n",
        report.manifest_asset_count
    ));
    out.push_str(&format!(
        "| Active nodes with lessons | {} |\n",
        report.active_nodes_with_lessons
    ));
    out.push_str(&format!(
        "| Active nodes missing lessons | {} |\n",
        report.active_nodes_missing_lessons
    ));
    out.push_str(&format!(
        "| Orphan lesson-target IDs | {} |\n",
        report.orphan_manifest_node_ids.len()
    ));
    out.push_str(&format!(
        "| Orphan IDs found in inactive source graphs | {} |\n",
        report.orphan_manifest_ids_found_in_inactive_graphs
    ));
    out.push_str(&format!(
        "| Orphan IDs with no source graph match | {} |\n",
        report.orphan_manifest_ids_without_source_graph.len()
    ));
    out.push_str(&format!(
        "| Inactive source graph documents scanned | {} |\n",
        report.inactive_source_graph_documents_scanned
    ));
    out.push_str(&format!(
        "| Manifest lesson assets statically audited | {} |\n",
        report.content_assets_scanned
    ));
    out.push_str(&format!(
        "| Assets with quality findings | {} |\n",
        report.content_assets_with_quality_findings
    ));
    out.push_str(&format!(
        "| Static quality findings | {} |\n\n",
        report.content_quality_findings.len()
    ));
    out.push_str(
        "| Framework | Grade | Subject | Nodes | Standard refs | With lessons | Missing | Assets |\n",
    );
    out.push_str("|---|---|---|---:|---:|---:|---:|---:|\n");
    for group in &report.groups {
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} |\n",
            group.framework,
            group.grade_level,
            group.subject_area,
            group.active_nodes,
            group.nodes_with_standard_refs,
            group.nodes_with_lessons,
            group.nodes_missing_lessons,
            group.lesson_assets
        ));
    }

    if !report.orphan_manifest_node_ids.is_empty() {
        out.push_str("\n## Orphan lesson-target IDs\n\n");
        for id in &report.orphan_manifest_node_ids {
            out.push_str(&format!("- `{id}`\n"));
        }
    }

    if !report.inactive_manifest_source_matches.is_empty() {
        out.push_str("\n## Orphan lesson IDs with inactive source candidates\n\n");
        out.push_str(
            "These IDs exist in source curriculum graphs not embedded by the current app loader. They are candidates for deliberate activation or ID reconciliation, not proof that the source graph is safe to merge.\n\n",
        );
        for matched in report.inactive_manifest_source_matches.iter().take(40) {
            let sources = matched
                .sources
                .iter()
                .map(|source| format!("{} ({})", source.path, source.title))
                .collect::<Vec<_>>()
                .join("; ");
            out.push_str(&format!("- {}: {}\n", matched.node_id, sources));
        }
        if report.inactive_manifest_source_matches.len() > 40 {
            out.push_str(&format!(
                "\n{} additional IDs with source candidates are included in the JSON report.\n",
                report.inactive_manifest_source_matches.len() - 40
            ));
        }
    }

    if !report.orphan_manifest_ids_without_source_graph.is_empty() {
        out.push_str("\n## Orphan lesson IDs without an inactive source graph match\n\n");
        for id in report.orphan_manifest_ids_without_source_graph.iter().take(40) {
            out.push_str(&format!("- {}\n", id));
        }
        if report.orphan_manifest_ids_without_source_graph.len() > 40 {
            out.push_str(&format!(
                "\n{} additional IDs without a source graph match are included in the JSON report.\n",
                report.orphan_manifest_ids_without_source_graph.len() - 40
            ));
        }
    }

    if !report.content_quality_findings.is_empty() {
        out.push_str("\n## Static instructional-content findings\n\n");
        for finding in report.content_quality_findings.iter().take(25) {
            out.push_str(&format!(
                "- **{} / {}** — `{}` — {}\n",
                finding.severity, finding.code, finding.asset_path, finding.field
            ));
        }
        if report.content_quality_findings.len() > 25 {
            out.push_str(&format!(
                "\n{} additional findings are included in the JSON report.\n",
                report.content_quality_findings.len() - 25
            ));
        }
    }

    out
}

fn workspace_root() -> Result<PathBuf, String> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .map(Path::to_path_buf)
        .ok_or_else(|| {
            "cannot locate mycelix-praxis workspace root from CARGO_MANIFEST_DIR".to_owned()
        })?;
    path.canonicalize()
        .map_err(|error| format!("cannot canonicalize workspace root: {error}"))
}

fn run() -> Result<(), AuditFailure> {
    let mut output_format = "json";
    let mut fail_on_blockers = false;
    for arg in env::args().skip(1) {
        match arg.as_str() {
            "--format=json" | "--json" => output_format = "json",
            "--format=markdown" | "--markdown" => output_format = "markdown",
            "--fail-on-blockers" => fail_on_blockers = true,
            "--help" | "-h" => {
                println!(
                    "praxis-curriculum-audit [--format=json|markdown] [--fail-on-blockers]\nRead-only coverage and instructional placeholder audit."
                );
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
    let content_audit = audit_manifest_assets(&workspace, &mut errors);
    let input_digest = compute_input_digest(&workspace, &documents, &mut errors);
    let inactive_source_audit =
        audit_inactive_manifest_sources(&workspace, &documents, &manifest, &mut errors);
    let mut report = analyze(&documents, &manifest, &mut errors);
    report.inactive_source_graph_documents_scanned = inactive_source_audit.graph_documents_scanned;
    report.orphan_manifest_ids_found_in_inactive_graphs =
        inactive_source_audit.orphan_ids_found_in_inactive_graphs;
    report.orphan_manifest_ids_without_source_graph =
        inactive_source_audit.orphan_ids_without_source_graph;
    report.inactive_manifest_source_matches = inactive_source_audit.source_matches;
    report.content_assets_scanned = content_audit.assets_scanned;
    report.content_assets_with_quality_findings = content_audit.assets_with_findings;
    report.content_quality_findings = content_audit.findings;
    if let Some(digest) = input_digest {
        report.input_digest_blake3 = digest;
    }
    if !errors.is_empty() {
        for error in &errors {
            eprintln!("curriculum audit error: {error}");
        }
        return Err(AuditFailure(errors));
    }
    match output_format {
        "json" => println!(
            "{}",
            serde_json::to_string_pretty(&report)
                .map_err(|error| AuditFailure(vec![format!("cannot serialize report: {error}")]))?
        ),
        "markdown" => print!("{}", markdown(&report)),
        _ => unreachable!("output format is constrained by argument parsing"),
    }
    let blocker_count = report
        .content_quality_findings
        .iter()
        .filter(|finding| finding.severity == "blocker")
        .count();
    if fail_on_blockers && blocker_count > 0 {
        eprintln!(
            "curriculum quality gate failed: {blocker_count} blocker finding(s); see report above"
        );
        return Err(AuditFailure(vec![format!(
            "{blocker_count} instructional placeholder blocker(s)"
        )]));
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
        assert!(schema["properties"]["lesson_digest_blake3"].is_object());
        assert!(schema["properties"]["learning_studies"].is_object());
        assert!(
            schema["allOf"]
                .as_array()
                .is_some_and(|rules| rules.len() >= 5),
            "the evidence schema must retain its conditional release gates"
        );
        assert_eq!(
            schema["properties"]["review_state"]["enum"]
                .as_array()
                .unwrap()
                .len(),
            5
        );
    }

    #[test]
    fn pedbench_seed_corpus_is_versioned_synthetic_and_identity_unique() {
        let corpus: Value = serde_json::from_str(include_str!(
            "../../../benchmarks/pedbench-v1/seed-cases.json"
        ))
        .expect("PedBench seed corpus must be valid JSON");

        assert_eq!(corpus["schema_version"], 1);
        assert_eq!(corpus["dataset_id"], "praxis-pedbench-seed-v1");
        assert_eq!(
            corpus["status"],
            "synthetic_seed_cases_not_a_validated_benchmark"
        );
        assert!(
            corpus["privacy"]
                .as_str()
                .unwrap()
                .contains("synthetic")
        );

        let cases = corpus["cases"]
            .as_array()
            .expect("PedBench cases must be an array");
        assert!(
            cases.len() >= 6,
            "seed corpus should exercise multiple domains"
        );

        let mut seen = BTreeSet::new();
        for case in cases {
            let case_id = nonempty_string(case.get("case_id"))
                .expect("each benchmark case must have a stable case_id");
            assert!(
                seen.insert(case_id.clone()),
                "duplicate PedBench case ID: {case_id}"
            );
            for key in [
                "domain",
                "grade_or_course_level",
                "language",
                "objective_ref",
                "learner_state",
                "prompt",
                "response_under_test",
                "assistance_stage",
                "expected_behaviours",
                "prohibited_behaviours",
                "reference_labels",
                "severity",
                "annotation_status",
            ] {
                assert!(
                    case.get(key).is_some(),
                    "PedBench case {case_id} is missing {key}"
                );
            }
            assert!(
                case["expected_behaviours"]
                    .as_array()
                    .is_some_and(|items| !items.is_empty()),
                "PedBench case {case_id} must include expected behaviours"
            );
            assert!(
                case["prohibited_behaviours"]
                    .as_array()
                    .is_some_and(|items| !items.is_empty()),
                "PedBench case {case_id} must include prohibited behaviours"
            );
            assert_eq!(
                case["annotation_status"], "needs_two_independent_educator_reviews",
                "synthetic seed labels must not be presented as validated gold labels"
            );
        }
    }

    #[test]
    fn extracts_only_json_include_paths() {
        let source = concat!(
            r#"include_str!("a.json"); "#,
            r#"include_str!("b.svg"); "#,
            r#"include_str!("nested/c.json");"#
        );
        assert_eq!(
            embedded_json_paths(source),
            vec!["a.json".to_owned(), "nested/c.json".to_owned()]
        );
    }

    #[test]
    fn inactive_source_candidates_classify_orphans_without_activating_them() {
        let active_ids = BTreeSet::from(["active".to_owned()]);
        let manifest = BTreeMap::from([
            ("active".to_owned(), 1usize),
            ("inactive_match".to_owned(), 1usize),
            ("not_found".to_owned(), 1usize),
        ]);
        let sources = vec![
            (
                "examples/curriculum/caps/caps-math-3.json".to_owned(),
                serde_json::json!({
                    "metadata": {
                        "framework": "CAPS",
                        "grade_level": "Grade3",
                        "subject_area": "Mathematics"
                    },
                    "nodes": [
                        {"id":"inactive_match","title":"Addition and Subtraction"},
                        {"id":"active","title":"Already Active"}
                    ],
                    "edges": []
                }),
            ),
            (
                "examples/curriculum/all-curricula-unified.json".to_owned(),
                serde_json::json!({
                    "nodes": [{"id":"inactive_match","title":"Unified Addition"}],
                    "edges": []
                }),
            ),
        ];

        let audit = match_inactive_manifest_sources(&active_ids, &manifest, &sources);
        assert_eq!(audit.graph_documents_scanned, 2);
        assert_eq!(audit.orphan_ids_found_in_inactive_graphs, 1);
        assert_eq!(audit.source_matches.len(), 1);
        assert_eq!(audit.source_matches[0].node_id, "inactive_match");
        assert_eq!(audit.source_matches[0].sources.len(), 2);
        assert_eq!(
            audit.orphan_ids_without_source_graph,
            vec!["not_found".to_owned()]
        );
    }

    #[test]
    fn reports_coverage_and_orphans() {
        let docs = vec![(PathBuf::from("fixture.json"), fixture_document())];
        let manifest = BTreeMap::from([("n1".to_owned(), 2usize), ("orphan".to_owned(), 1usize)]);
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
            (
                PathBuf::from("a.json"),
                serde_json::json!({"nodes":[{"id":"same"}],"edges":[]}),
            ),
            (
                PathBuf::from("b.json"),
                serde_json::json!({"nodes":[{"id":"same"}],"edges":[]}),
            ),
        ];
        let mut errors = Vec::new();
        let _ = analyze(&docs, &BTreeMap::new(), &mut errors);
        assert!(
            errors
                .iter()
                .any(|error| error.contains("duplicate active node ID"))
        );
    }

    #[test]
    fn dangling_edges_are_rejected() {
        let docs = vec![(
            PathBuf::from("a.json"),
            serde_json::json!({
                "nodes": [{"id": "present"}],
                "edges": [{"from": "present", "to": "missing"}]
            }),
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
        assert_eq!(
            hash_named_inputs(&first),
            hash_named_inputs(&same_inputs_reverse_insert)
        );

        let mut changed_content = first.clone();
        changed_content.insert("a.json".to_owned(), b"tampered".to_vec());
        assert_ne!(
            hash_named_inputs(&first),
            hash_named_inputs(&changed_content)
        );

        let mut changed_path = BTreeMap::new();
        changed_path.insert("renamed.json".to_owned(), b"content-a".to_vec());
        changed_path.insert("b.json".to_owned(), b"content-b".to_vec());
        assert_ne!(hash_named_inputs(&first), hash_named_inputs(&changed_path));
    }

    #[test]
    fn framework_groups_do_not_cross_count_standard_references() {
        let document = serde_json::json!({
            "metadata": {
                "framework": "Fallback",
                "grade_level": "Grade3",
                "subject_area": "Mathematics"
            },
            "nodes": [{
                "id": "multi",
                "grade_levels": ["Grade3"],
                "subject_area": "Mathematics",
                "academic_standards": [
                    {"framework":"FrameworkA","code":"A.1"},
                    {"framework":"FrameworkB","code":"B.1"}
                ]
            }],
            "edges": []
        });
        let docs = vec![(PathBuf::from("multi.json"), document)];
        let mut errors = Vec::new();
        let report = analyze(&docs, &BTreeMap::new(), &mut errors);
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(report.groups.len(), 2);
        assert!(
            report
                .groups
                .iter()
                .all(|group| group.distinct_standard_refs == 1)
        );
        assert_eq!(report.distinct_standard_refs, 2);
    }

    #[test]
    fn malformed_standard_references_fail_closed() {
        let document = serde_json::json!({
            "nodes": [{
                "id": "bad",
                "academic_standards": [{"framework":"OnlyFramework"}]
            }],
            "edges": []
        });
        let docs = vec![(PathBuf::from("bad.json"), document)];
        let mut errors = Vec::new();
        let _ = analyze(&docs, &BTreeMap::new(), &mut errors);
        assert!(
            errors
                .iter()
                .any(|error| error.contains("requires framework/code"))
        );
    }

    #[test]
    fn static_content_audit_flags_placeholders_in_all_instructional_channels() {
        let value = serde_json::json!({
            "node_id": "CAPS.Mathematics.Gr3.MUL",
            "lesson": {
                "title": "Multiplication",
                "explanation": "Grade 3 mathematics content.",
                "examples": [
                    {
                        "problem": "Solve a Grade 3 Mathematics problem.",
                        "steps": [{"instruction": "Show your working", "result": "Calculate"}],
                        "answer": "Answer: See worked solution"
                    },
                    {
                        "problem": "Solve a Grade 3 Mathematics problem.",
                        "steps": [{"instruction": "Show your working", "result": "Calculate"}],
                        "answer": "Answer: See worked solution"
                    }
                ],
                "practice_problems": [{
                    "question": "What is important about Grade 3 Mathematics?",
                    "answer": "Answer.",
                    "explanation": "Grade 3 mathematics content."
                }],
                "hints": ["Try the method for Grade 3 Mathematics."],
                "key_vocabulary": [{
                    "term": "mathematics",
                    "definition": "A key word in this topic",
                    "example_usage": "Used in Grade 3 maths."
                }],
                "misconceptions": [{
                    "misconception": "A common mistake in Grade 3 Mathematics.",
                    "correction": "The correct way.",
                    "why_students_think_this": "Students need help."
                }]
            },
            "assessment": {
                "items": [{
                    "question": "Explain Grade 3 Mathematics.",
                    "correct_answer": "Answer.",
                    "hint": "2"
                }]
            },
            "flashcards": [{
                "front": "What is important about Grade 3 Mathematics?",
                "back": "Grade 3 mathematics content"
            }]
        });
        let mut audit = ContentQualityAudit::default();
        assert!(audit_lesson_value("fixture.json", &value, &mut audit));
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.field == "lesson.explanation")
        );
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.field == "assessment.items[0].question")
        );
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.field == "flashcards[0].back")
        );
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.code == "duplicate_worked_example")
        );
        assert!(
            !audit
                .findings
                .iter()
                .any(|finding| finding.code == "duplicate_practice_prompt")
        );
        assert!(audit.findings.len() >= 10);
    }

    #[test]
    fn static_content_audit_blocks_correct_answers_in_distractors() {
        let value = serde_json::json!({
            "node_id": "fixture.distractors",
            "lesson": {
                "title": "Multiplication",
                "explanation": "Multiplication represents equal groups of the same size.",
                "examples": [{
                    "problem": "Three groups of two objects are shown. How many objects are there?",
                    "steps": [{"instruction": "Count both groups.", "result": "2 + 2 + 2 = 6"}],
                    "answer": "6"
                }],
                "practice_problems": [{
                    "question": "How many objects are in 3 groups of 2?",
                    "answer": "6",
                    "explanation": "3 × 2 = 6.",
                    "distractors": ["4", " 4 ", " 6 ", "8"]
                }]
            }
        });
        let mut audit = ContentQualityAudit::default();
        assert!(audit_lesson_value("distractors.json", &value, &mut audit));
        assert!(audit.findings.iter().any(|finding| {
            finding.code == "answer_in_distractors"
                && finding.field == "lesson.practice_problems[0].distractors"
                && finding.severity == "blocker"
        }));
        assert!(audit.findings.iter().any(|finding| {
            finding.code == "duplicate_distractor"
                && finding.field == "lesson.practice_problems[0].distractors"
                && finding.severity == "warning"
        }));
    }

    #[test]
    fn static_content_audit_does_not_flag_specific_instruction() {
        let value = serde_json::json!({
            "node_id": "CCSS.Math.Content.3.OA.A.1",
            "lesson": {
                "title": "Interpret products of whole numbers",
                "explanation": "Multiplication represents equal groups. Four groups of three objects make twelve objects: 3 + 3 + 3 + 3 = 12, so 4 × 3 = 12.",
                "examples": [{
                    "problem": "There are 5 baskets with 3 oranges in each basket. How many oranges are there?",
                    "steps": [
                        {"instruction": "Count 5 equal groups of 3 oranges.", "result": "3 + 3 + 3 + 3 + 3"},
                        {"instruction": "Add the groups or multiply 5 by 3.", "result": "5 × 3 = 15"}
                    ],
                    "answer": "There are 15 oranges."
                }],
                "practice_problems": [{
                    "question": "How many wheels are on 4 bicycles if each bicycle has 2 wheels?",
                    "answer": "8",
                    "explanation": "Four groups of two wheels gives 4 × 2 = 8 wheels."
                }]
            }
        });
        let mut audit = ContentQualityAudit::default();
        assert!(!audit_lesson_value("good.json", &value, &mut audit));
        assert!(audit.findings.is_empty());
    }

    #[test]
    fn static_content_audit_rejects_missing_instruction_and_empty_practice() {
        let value = serde_json::json!({
            "node_id": "fixture.missing",
            "lesson": {
                "title": "",
                "explanation": "A real explanation with enough specific meaning.",
                "examples": [],
                "practice_problems": []
            }
        });
        let mut audit = ContentQualityAudit::default();
        assert!(audit_lesson_value("missing.json", &value, &mut audit));
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.code == "missing_required_text")
        );
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.code == "missing_worked_examples")
        );
        assert!(
            audit
                .findings
                .iter()
                .any(|finding| finding.code == "missing_practice_items")
        );

        let missing_object = serde_json::json!({"node_id":"fixture.no-lesson"});
        let mut second_audit = ContentQualityAudit::default();
        assert!(audit_lesson_value(
            "no-lesson.json",
            &missing_object,
            &mut second_audit
        ));
        assert!(
            second_audit
                .findings
                .iter()
                .any(|finding| finding.code == "missing_lesson_object")
        );
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
