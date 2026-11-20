//! Advisory AST inventory for Holochain OtherRole bridge calls.
//!
//! This module deliberately does not claim a complete source-to-DNA mapping.
//! It discovers static call sites and associates a source file with the nearest
//! ancestor hApp manifest as a context hint, then classifies profile-dependent cases
//! as candidates rather than asserting that matching role names prove DNA
//! identity. Findings remain advisory until mapping coverage is qualified. The nearest manifest is not proof of source-to-DNA ownership.

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use syn::spanned::Spanned;
use syn::visit::{self, Visit};
use syn::{Expr, ExprCall, ExprLit, ExprMethodCall, ExprPath, File, ItemConst, Lit};

#[derive(Debug, Deserialize)]
struct HappManifest {
    name: String,
    roles: Vec<HappRole>,
}

#[derive(Debug, Deserialize)]
struct HappRole {
    name: String,
    dna: DnaConfig,
}

#[derive(Debug, Deserialize)]
struct DnaConfig {
    path: String,
}

#[derive(Debug, Clone)]
struct ManifestInfo {
    path: PathBuf,
    root: PathBuf,
    name: String,
    role_names: BTreeSet<String>,
    dna_paths: BTreeMap<String, String>,
    git_blob_sha: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
struct CallSite {
    source_path: String,
    line: usize,
    nearest_manifest_context: Option<String>,
    nearest_manifest_happ_name: Option<String>,
    context_roles: Vec<String>,
    context_role_dna_paths: BTreeMap<String, String>,
    context_manifest_git_blob_sha: Option<String>,
    source_to_dna_identity: String,
    target_role: Option<String>,
    target_expression: String,
    classification: String,
    target_declared_in_nearest_manifest_context: bool,
    target_present_elsewhere: bool,
    matching_role_name_profiles: Vec<String>,
    caveat: String,
}

#[derive(Debug, Clone, Serialize)]
struct ManifestError {
    path: String,
    error: String,
}

#[derive(Default)]
struct ConstCollector {
    values: BTreeMap<String, Vec<String>>,
}

impl<'ast> Visit<'ast> for ConstCollector {
    fn visit_item_const(&mut self, node: &'ast ItemConst) {
        if let Expr::Lit(ExprLit { lit: Lit::Str(value), .. }) = node.expr.as_ref() {
            self.values
                .entry(node.ident.to_string())
                .or_default()
                .push(value.value());
        }
        visit::visit_item_const(self, node);
    }
}

struct OtherRoleVisitor<'a> {
    constants: &'a BTreeMap<String, Vec<String>>,
    calls: Vec<(usize, Option<String>, String)>,
}

impl<'a> OtherRoleVisitor<'a> {
    fn new(constants: &'a BTreeMap<String, Vec<String>>) -> Self {
        Self {
            constants,
            calls: Vec::new(),
        }
    }
}

impl<'ast> Visit<'ast> for OtherRoleVisitor<'_> {
    fn visit_expr_call(&mut self, node: &'ast ExprCall) {
        let is_other_role = match node.func.as_ref() {
            Expr::Path(ExprPath { path, .. }) => path
                .segments
                .last()
                .map(|segment| segment.ident == "OtherRole" && path.segments.iter().any(|segment| segment.ident == "CallTargetCell"))
                .unwrap_or(false),
            _ => false,
        };

        if is_other_role {
            let first = node.args.first();
            let target = first.and_then(|expr| extract_role_literal(expr, self.constants));
            let expression = first
                .map(|expr| quote_expression(expr))
                .unwrap_or_else(|| "<missing-argument>".to_string());
            self.calls.push((node.span().start().line, target, expression));
        }

        visit::visit_expr_call(self, node);
    }
}

fn quote_expression(expr: &Expr) -> String {
    format!("{expr:?}")
}

fn extract_role_literal(
    expr: &Expr,
    constants: &BTreeMap<String, Vec<String>>,
) -> Option<String> {
    match expr {
        Expr::Lit(ExprLit { lit: Lit::Str(value), .. }) => Some(value.value()),
        Expr::MethodCall(ExprMethodCall {
            method, receiver, ..
        }) if method == "into" || method == "to_owned" || method == "to_string" => {
            extract_role_literal(receiver, constants)
        }
        Expr::Path(ExprPath { path, .. }) => {
            let ident = path.segments.last()?.ident.to_string();
            let values = constants.get(&ident)?;
            if values.len() == 1 {
                values.first().cloned()
            } else {
                None
            }
        }
        Expr::Paren(paren) => extract_role_literal(&paren.expr, constants),
        Expr::Group(group) => extract_role_literal(&group.expr, constants),
        _ => None,
    }
}

const EXCLUDED_SOURCE_COMPONENTS: &[&str] = &[
    ".git",
    "target",
    "node_modules",
    "vendor",
    ".cache",
    "build",
    "dist",
    "archive",
    "archives",
    "kosmic-lab-workspace-migration-2026-05-31",
    "tests",
    "test",
    "examples",
    "benches",
];

fn path_has_excluded_component(path: &Path) -> bool {
    path.components().any(|component| {
        EXCLUDED_SOURCE_COMPONENTS
            .contains(&component.as_os_str().to_string_lossy().as_ref())
    })
}

/// Return only paths recorded in the Git index. An untracked local scratch
/// file must never silently change a report attributed to HEAD.
fn parse_tracked_paths(bytes: &[u8]) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    bytes
        .split(|byte| *byte == 0)
        .filter(|value| !value.is_empty())
        .map(|value| {
            let path = std::str::from_utf8(value)
                .map_err(|error| format!("tracked path is not UTF-8: {error}"))?;
            Ok(PathBuf::from(path))
        })
        .collect()
}

fn tracked_paths(root: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    let output = Command::new("git")
        .args(["ls-files", "-z"])
        .current_dir(root)
        .output()?;
    if !output.status.success() {
        return Err(format!(
            "git ls-files failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        )
        .into());
    }
    parse_tracked_paths(&output.stdout)
}

/// Refuse to attribute modified working-tree source to the committed HEAD.
/// Untracked paths are ignored by tracked_paths; staged/modified tracked files
/// are rejected here so the index and HEAD cannot be confused.
fn ensure_tracked_worktree_clean(root: &Path) -> Result<(), Box<dyn Error>> {
    let output = Command::new("git")
        .args(["status", "--porcelain", "--untracked-files=no"])
        .current_dir(root)
        .output()?;
    if !output.status.success() {
        return Err(format!(
            "git status failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        )
        .into());
    }
    let status = String::from_utf8(output.stdout)?;
    if !status.trim().is_empty() {
        return Err(format!(
            "tracked working tree is modified; commit or revert tracked changes before auditing:\n{}",
            status.trim()
        )
        .into());
    }
    Ok(())
}

fn is_happ_manifest(path: &Path) -> bool {
    let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
        return false;
    };
    name == "happ.yaml" || name.ends_with("-happ.yaml")
}

fn relative_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

fn git_blob_sha(root: &Path, relative: &str) -> Option<String> {
    let output = Command::new("git")
        .args(["rev-parse", &format!("HEAD:{relative}")])
        .current_dir(root)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    String::from_utf8(output.stdout)
        .ok()
        .map(|value| value.trim().to_string())
}

fn discover_manifests(
    root: &Path,
) -> Result<(Vec<ManifestInfo>, Vec<ManifestError>), Box<dyn Error>> {
    let mut manifests = Vec::new();
    let mut errors = Vec::new();

    for relative in tracked_paths(root)? {
        if path_has_excluded_component(&relative) || !is_happ_manifest(&relative) {
            continue;
        }
        let path = root.join(&relative);
        // Do not follow tracked symlink manifests: the audit must read a
        // repository-owned regular file, not arbitrary external content.
        let metadata = match fs::symlink_metadata(&path) {
            Ok(metadata) => metadata,
            Err(error) => {
                errors.push(ManifestError {
                    path: relative_path(root, &path),
                    error: format!("metadata read failed: {error}"),
                });
                continue;
            }
        };
        if !metadata.file_type().is_file() {
            continue;
        }
        let yaml = match fs::read_to_string(&path) {
            Ok(yaml) => yaml,
            Err(error) => {
                errors.push(ManifestError {
                    path: relative_path(root, &path),
                    error: format!("read failed: {error}"),
                });
                continue;
            }
        };
        let manifest: HappManifest = match yaml_serde::from_str(&yaml) {
            Ok(manifest) => manifest,
            Err(error) => {
                errors.push(ManifestError {
                    path: relative_path(root, &path),
                    error: format!("YAML parse failed: {error}"),
                });
                continue;
            }
        };

        let mut role_names = BTreeSet::new();
        let mut dna_paths = BTreeMap::new();
        let mut duplicate_roles = Vec::new();
        for role in &manifest.roles {
            if !role_names.insert(role.name.clone()) {
                duplicate_roles.push(role.name.clone());
            }
            dna_paths.insert(role.name.clone(), role.dna.path.clone());
        }
        if !duplicate_roles.is_empty() {
            errors.push(ManifestError {
                path: relative_path(root, &path),
                error: format!("duplicate roles: {}", duplicate_roles.join(", ")),
            });
            continue;
        }

        let root_dir = path.parent().unwrap_or(root).to_path_buf();
        let relative = relative_path(root, &path);
        manifests.push(ManifestInfo {
            path,
            root: root_dir,
            name: manifest.name,
            role_names,
            dna_paths,
            git_blob_sha: git_blob_sha(root, &relative),
        });
    }

    // Deepest manifest root wins when nested hApps/manifests exist.
    manifests.sort_by_key(|manifest| manifest.root.components().count());
    Ok((manifests, errors))
}

fn find_nearest_manifest<'a>(
    source: &Path,
    manifests: &'a [ManifestInfo],
) -> Option<&'a ManifestInfo> {
    manifests
        .iter()
        .filter(|manifest| source.starts_with(&manifest.root))
        .max_by_key(|manifest| manifest.root.components().count())
}

fn matching_role_name_profiles(
    caller_roles: &BTreeSet<String>,
    target_role: &str,
    manifests: &[ManifestInfo],
) -> Vec<String> {
    manifests
        .iter()
        .filter(|manifest| {
            target_role != ""
                && manifest.role_names.contains(target_role)
                && caller_roles.iter().any(|role| manifest.role_names.contains(role))
        })
        .map(|manifest| manifest.name.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

#[derive(Debug)]
struct TargetClassification {
    classification: &'static str,
    declared_in_context: bool,
    present_elsewhere: bool,
    matching_profile_candidates: Vec<String>,
}

fn classify_target(
    context: Option<&ManifestInfo>,
    target_role: Option<&str>,
    manifests: &[ManifestInfo],
) -> TargetClassification {
    match (context, target_role) {
        (None, _) => TargetClassification {
            classification: "source-context-unresolved",
            declared_in_context: false,
            present_elsewhere: false,
            matching_profile_candidates: Vec::new(),
        },
        (Some(_), None) => TargetClassification {
            classification: "dynamic-unresolved",
            declared_in_context: false,
            present_elsewhere: false,
            matching_profile_candidates: Vec::new(),
        },
        (Some(context), Some(target)) if context.role_names.contains(target) => {
            TargetClassification {
                classification: "nearest-manifest-declared",
                declared_in_context: true,
                present_elsewhere: true,
                matching_profile_candidates: vec![context.name.clone()],
            }
        }
        (Some(context), Some(target)) => {
            let present_elsewhere = manifests
                .iter()
                .any(|manifest| manifest.role_names.contains(target));
            let candidates =
                matching_role_name_profiles(&context.role_names, target, manifests);
            let classification = if !candidates.is_empty() {
                "profile-dependent-candidate"
            } else if present_elsewhere {
                "cross-profile-or-source-mapping-unresolved"
            } else {
                "missing-role-candidate"
            };
            TargetClassification {
                classification,
                declared_in_context: false,
                present_elsewhere,
                matching_profile_candidates: candidates,
            }
        }
    }
}

fn audit_source_file(
    root: &Path,
    source: &Path,
    manifests: &[ManifestInfo],
) -> Result<(Vec<CallSite>, usize, bool), Box<dyn Error>> {
    let text = fs::read_to_string(source)?;
    let raw_contains_other_role = text.contains("OtherRole");
    let syntax: File = syn::parse_file(&text)?;
    let mut const_collector = ConstCollector::default();
    const_collector.visit_file(&syntax);
    let mut visitor = OtherRoleVisitor::new(&const_collector.values);
    visitor.visit_file(&syntax);

    let context = find_nearest_manifest(source, manifests);
    let context_roles = context
        .map(|manifest| manifest.role_names.clone())
        .unwrap_or_default();

    let call_count = visitor.calls.len();
    let mut results = Vec::new();
    for (line, target_role, expression) in visitor.calls {
        let target_classification =
            classify_target(context, target_role.as_deref(), manifests);

        results.push(CallSite {
            source_path: relative_path(root, source),
            line,
            nearest_manifest_context: context.map(|manifest| relative_path(root, &manifest.path)),
            nearest_manifest_happ_name: context.map(|manifest| manifest.name.clone()),
            context_roles: context_roles.iter().cloned().collect(),
            context_role_dna_paths: context.map(|manifest| manifest.dna_paths.clone()).unwrap_or_default(),
            context_manifest_git_blob_sha: context.and_then(|manifest| manifest.git_blob_sha.clone()),
            source_to_dna_identity: "not-proven".to_string(),
            target_role,
            target_expression: expression,
            classification: target_classification.classification.to_string(),
            target_declared_in_nearest_manifest_context: target_classification.declared_in_context,
            target_present_elsewhere: target_classification.present_elsewhere,
            matching_role_name_profiles: target_classification.matching_profile_candidates,
            caveat: "Nearest-manifest context is not proof of source-to-DNA ownership. Role-name overlap in another profile is only a candidate mapping and does not prove DNA artifact identity.".to_string(),
        });
    }

    let textual_occurrences = text.matches("OtherRole").count();
    let unmatched_textual_occurrences = textual_occurrences.saturating_sub(call_count);
    Ok((results, unmatched_textual_occurrences, raw_contains_other_role && call_count == 0))
}

pub fn audit(root: &Path) -> Result<Value, Box<dyn Error>> {
    ensure_tracked_worktree_clean(root)?;
    let tracked = tracked_paths(root)?;
    let (manifests, manifest_errors) = discover_manifests(root)?;
    let commit_output = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(root)
        .output()?;
    if !commit_output.status.success() {
        return Err(format!(
            "unable to resolve Git HEAD for topology audit: {}",
            String::from_utf8_lossy(&commit_output.stderr).trim()
        )
        .into());
    }
    let source_commit = String::from_utf8(commit_output.stdout)?.trim().to_string();
    let tree_output = Command::new("git")
        .args(["rev-parse", "HEAD^{tree}"])
        .current_dir(root)
        .output()?;
    if !tree_output.status.success() {
        return Err("unable to resolve Git tree SHA for topology audit".into());
    }
    let source_tree_sha = String::from_utf8(tree_output.stdout)?.trim().to_string();

    let mut source_file_count = 0usize;
    let mut mapped_source_file_count = 0usize;
    let mut unscoped_source_file_count = 0usize;
    let mut call_sites = Vec::new();
    let mut unmatched_text_files = Vec::new();
    let mut ast_callsite_count = 0usize;
    let mut unmatched_textual_occurrence_count = 0usize;
    let mut parse_errors = Vec::new();

    for relative in tracked {
        if path_has_excluded_component(&relative)
            || relative.extension().and_then(|extension| extension.to_str()) != Some("rs")
        {
            continue;
        }
        let source_path = root.join(&relative);
        let metadata = match fs::symlink_metadata(&source_path) {
            Ok(metadata) => metadata,
            Err(error) => {
                parse_errors.push(json!({
                    "path": relative_path(root, &source_path),
                    "error": format!("metadata read failed: {error}")
                }));
                continue;
            }
        };
        if !metadata.file_type().is_file() {
            continue;
        }

        source_file_count += 1;
        let context = find_nearest_manifest(&source_path, &manifests);
        if context.is_some() {
            mapped_source_file_count += 1;
        } else {
            unscoped_source_file_count += 1;
        }

        match audit_source_file(root, &source_path, &manifests) {
            Ok((sites, unmatched_text_count, raw_only)) => {
                ast_callsite_count += sites.len();
                unmatched_textual_occurrence_count += unmatched_text_count;
                if unmatched_text_count > 0 || raw_only {
                    unmatched_text_files.push(json!({
                        "path": relative_path(root, &source_path),
                        "unmatched_textual_occurrence_count": unmatched_text_count,
                        "review_note": "May include comments, strings, macros or syntax not represented as a direct AST call; manual review required."
                    }));
                }
                call_sites.extend(sites);
            }
            Err(error) => parse_errors.push(json!({
                "path": relative_path(root, entry.path()),
                "error": error.to_string()
            })),
        }
    }

    call_sites.sort_by(|left, right| {
        left.source_path
            .cmp(&right.source_path)
            .then(left.line.cmp(&right.line))
            .then(left.target_expression.cmp(&right.target_expression))
    });

    let mut counts = BTreeMap::<String, usize>::new();
    for site in &call_sites {
        *counts.entry(site.classification.clone()).or_default() += 1;
    }

    Ok(json!({
        "schema_version": "mycelix-otherrole-topology-audit-v0.1",
        "report_kind": "advisory-static-ast-inventory",
        "source": {
            "repository": "Luminous-Dynamics/mycelix",
            "commit": source_commit,
            "tree_sha": source_tree_sha
        },
        "scope": {
            "manifest_count": manifests.len(),
            "rust_source_file_count": source_file_count,
            "source_selection": "git-tracked-files-only-clean-tracked-worktree",
            "excluded_path_components": EXCLUDED_SOURCE_COMPONENTS,
            "nearest_manifest_context_file_count": mapped_source_file_count,
            "without_nearest_manifest_context_file_count": unscoped_source_file_count,
            "nearest_manifest_context_coverage": if source_file_count == 0 {
                0.0
            } else {
                mapped_source_file_count as f64 / source_file_count as f64
            }
        },
        "manifests": manifests.iter().map(|manifest| json!({
            "name": manifest.name,
            "path": relative_path(root, &manifest.path),
            "git_blob_sha": manifest.git_blob_sha,
            "roles": manifest.role_names,
            "role_dna_paths": manifest.dna_paths
        })).collect::<Vec<_>>(),
        "manifest_errors": manifest_errors,
        "parse_errors": parse_errors,
        "ast_otherrole_callsite_count": ast_callsite_count,
        "unmatched_textual_occurrence_count": unmatched_textual_occurrence_count,
        "unmatched_text_files": unmatched_text_files,
        "classification_counts": counts,
        "call_sites": call_sites,
        "claim_ceiling": "Advisory inventory only. Nearest-manifest context does not prove source-to-DNA ownership. Role-name overlap does not prove DNA artifact identity; unmapped roots and dynamic targets remain unresolved."
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest_info(name: &str, roles: &[&str]) -> ManifestInfo {
        let role_names = roles.iter().map(|role| (*role).to_string()).collect();
        let dna_paths = roles
            .iter()
            .map(|role| ((*role).to_string(), format!("./{role}.dna")))
            .collect();
        ManifestInfo {
            path: PathBuf::from(format!("{name}/happ.yaml")),
            root: PathBuf::from(name),
            name: name.to_string(),
            role_names,
            dna_paths,
            git_blob_sha: None,
        }
    }

    #[test]
    fn tracked_path_parser_preserves_spaces_and_nested_paths() {
        let paths = parse_tracked_paths(
            b"mycelix-workspace/a.rs\0mycelix-workspace/path with spaces/b.rs\0",
        )
        .unwrap();
        assert_eq!(paths.len(), 2);
        assert_eq!(
            paths[1],
            PathBuf::from("mycelix-workspace/path with spaces/b.rs")
        );
    }

    #[test]
    fn excluded_components_are_exact_path_segments() {
        assert!(path_has_excluded_component(Path::new(
            "mycelix-workspace/crate/tests/bridge.rs"
        )));
        assert!(path_has_excluded_component(Path::new(
            "mycelix-workspace/crate/target/generated.rs"
        )));
        assert!(!path_has_excluded_component(Path::new(
            "mycelix-workspace/crate/contest/bridge.rs"
        )));
    }

    #[test]
    fn literal_other_role_argument_is_resolved() {
        let syntax: File = syn::parse_file(
            r#"
            fn f() {
                call(CallTargetCell::OtherRole("identity".into()));
            }
            "#,
        )
        .unwrap();
        let constants = BTreeMap::new();
        let mut visitor = OtherRoleVisitor::new(&constants);
        visitor.visit_file(&syntax);
        assert_eq!(visitor.calls.len(), 1);
        assert_eq!(visitor.calls[0].1.as_deref(), Some("identity"));
    }

    #[test]
    fn unique_string_constant_is_resolved() {
        let syntax: File = syn::parse_file(
            r#"
            const IDENTITY_ROLE: &str = "identity";
            fn f() {
                call(CallTargetCell::OtherRole(IDENTITY_ROLE.into()));
            }
            "#,
        )
        .unwrap();
        let mut collector = ConstCollector::default();
        collector.visit_file(&syntax);
        let mut visitor = OtherRoleVisitor::new(&collector.values);
        visitor.visit_file(&syntax);
        assert_eq!(visitor.calls.len(), 1);
        assert_eq!(visitor.calls[0].1.as_deref(), Some("identity"));
    }

    #[test]
    fn dynamic_role_expression_is_not_guessed() {
        let syntax: File = syn::parse_file(
            r#"
            fn f(index: u32) {
                call(CallTargetCell::OtherRole(format!("chat.{}", index)));
            }
            "#,
        )
        .unwrap();
        let constants = BTreeMap::new();
        let mut visitor = OtherRoleVisitor::new(&constants);
        visitor.visit_file(&syntax);
        assert_eq!(visitor.calls.len(), 1);
        assert!(visitor.calls[0].1.is_none());
    }

    #[test]
    fn nearest_manifest_declaration_is_not_called_runtime_valid() {
        let context = manifest_info("fabrication", &["fabrication", "identity"]);
        let result = classify_target(Some(&context), Some("identity"), &[context.clone()]);
        assert_eq!(result.classification, "nearest-manifest-declared");
        assert!(result.declared_in_context);
        assert_eq!(result.matching_profile_candidates, vec!["fabrication"]);
    }

    #[test]
    fn matching_role_names_in_another_profile_are_only_candidates() {
        let context = manifest_info("fabrication-standalone", &["fabrication"]);
        let candidate = manifest_info("unified", &["fabrication", "identity"]);
        let manifests = vec![context.clone(), candidate];
        let result = classify_target(Some(&context), Some("identity"), &manifests);
        assert_eq!(result.classification, "profile-dependent-candidate");
        assert!(!result.declared_in_context);
        assert!(result.present_elsewhere);
        assert_eq!(result.matching_profile_candidates, vec!["unified"]);
    }

    #[test]
    fn target_elsewhere_without_caller_role_overlap_stays_unresolved() {
        let context = manifest_info("fabrication-standalone", &["fabrication"]);
        let other = manifest_info("identity-app", &["identity"]);
        let result = classify_target(Some(&context), Some("identity"), &[context.clone(), other]);
        assert_eq!(
            result.classification,
            "cross-profile-or-source-mapping-unresolved"
        );
        assert!(!result.declared_in_context);
        assert!(result.present_elsewhere);
        assert!(result.matching_profile_candidates.is_empty());
    }

    #[test]
    fn absent_target_is_only_a_missing_role_candidate() {
        let context = manifest_info("fabrication-standalone", &["fabrication"]);
        let result = classify_target(Some(&context), Some("unknown"), &[context]);
        assert_eq!(result.classification, "missing-role-candidate");
        assert!(!result.present_elsewhere);
    }

    #[test]
    fn source_without_manifest_context_is_not_assigned_an_owner() {
        let result = classify_target(None, Some("identity"), &[]);
        assert_eq!(result.classification, "source-context-unresolved");
        assert!(!result.declared_in_context);
    }

    #[test]
    fn ambiguous_or_dynamic_target_is_not_guessed() {
        let context = manifest_info("fabrication-standalone", &["fabrication"]);
        let result = classify_target(Some(&context), None, &[context]);
        assert_eq!(result.classification, "dynamic-unresolved");
        assert!(!result.declared_in_context);
    }

    #[test]
    fn unrelated_function_named_otherrole_is_not_counted_as_holochain_dispatch() {
        let syntax: File = syn::parse_file(
            r#"
            fn f() {
                helper::OtherRole("identity");
            }
            "#,
        )
        .unwrap();
        let constants = BTreeMap::new();
        let mut visitor = OtherRoleVisitor::new(&constants);
        visitor.visit_file(&syntax);
        assert!(visitor.calls.is_empty());
    }

}
