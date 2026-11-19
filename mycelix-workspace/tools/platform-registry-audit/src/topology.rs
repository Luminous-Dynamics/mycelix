//! Advisory AST inventory for Holochain OtherRole bridge calls.
//!
//! This module deliberately does not claim a complete source-to-DNA mapping.
//! It discovers static call sites, associates a source file with the deepest
//! nearest ancestor hApp manifest as a context hint, and classifies profile-dependent cases
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
use walkdir::{DirEntry, WalkDir};

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

fn is_ignored_component(component: &str) -> bool {
    matches!(
        component,
        ".git"
            | "target"
            | "node_modules"
            | "vendor"
            | ".cache"
            | "build"
            | "dist"
            | "archive"
            | "archives"
            | "kosmic-lab-workspace-migration-2026-05-31"
    )
}

fn should_skip_entry(entry: &DirEntry) -> bool {
    entry
        .path()
        .components()
        .any(|component| {
            is_ignored_component(&component.as_os_str().to_string_lossy())
        })
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

    let walker = WalkDir::new(root)
        .follow_links(false)
        .into_iter()
        .filter_entry(|entry| !should_skip_entry(entry));

    for entry in walker {
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                errors.push(ManifestError {
                    path: error.path().map(|path| relative_path(root, path)).unwrap_or_default(),
                    error: error.to_string(),
                });
                continue;
            }
        };

        if !entry.file_type().is_file() || !is_happ_manifest(entry.path()) {
            continue;
        }

        let path = entry.path().to_path_buf();
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
) -> Result<(Vec<CallSite>, bool), Box<dyn Error>> {
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
            target_present_elsewhere,
            matching_role_name_profiles: matching,
            caveat: "Nearest-manifest context is not proof of source-to-DNA ownership. Role-name overlap in another profile is only a candidate mapping and does not prove DNA artifact identity.".to_string(),
        });
    }

    Ok((results, raw_contains_other_role && call_count == 0))
}

pub fn audit(root: &Path) -> Result<Value, Box<dyn Error>> {
    let (manifests, manifest_errors) = discover_manifests(root)?;
    let source_sha = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(root)
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .map(|value| value.trim().to_string());

    let mut source_file_count = 0usize;
    let mut mapped_source_file_count = 0usize;
    let mut unscoped_source_file_count = 0usize;
    let mut call_sites = Vec::new();
    let mut raw_text_only_files = Vec::new();
    let mut parse_errors = Vec::new();

    let walker = WalkDir::new(root)
        .follow_links(false)
        .into_iter()
        .filter_entry(|entry| !should_skip_entry(entry));

    for entry in walker {
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                parse_errors.push(json!({
                    "path": error.path().map(|path| relative_path(root, path)),
                    "error": error.to_string()
                }));
                continue;
            }
        };
        if !entry.file_type().is_file()
            || entry.path().extension().and_then(|extension| extension.to_str()) != Some("rs")
            || entry.path().components().any(|component| {
                matches!(
                    component.as_os_str().to_string_lossy().as_ref(),
                    "tests" | "test" | "examples" | "benches"
                )
            })
        {
            continue;
        }

        source_file_count += 1;
        let context = find_nearest_manifest(entry.path(), &manifests);
        if context.is_some() {
            mapped_source_file_count += 1;
        } else {
            unscoped_source_file_count += 1;
        }

        match audit_source_file(root, entry.path(), &manifests) {
            Ok((sites, raw_only)) => {
                if raw_only {
                    raw_text_only_files.push(relative_path(root, entry.path()));
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
            "commit": source_sha
        },
        "scope": {
            "manifest_count": manifests.len(),
            "rust_source_file_count": source_file_count,
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
        "raw_text_only_occurrence_files": raw_text_only_files,
        "classification_counts": counts,
        "call_sites": call_sites,
        "claim_ceiling": "Advisory inventory only. Nearest-manifest context does not prove source-to-DNA ownership. Role-name overlap does not prove DNA artifact identity; unmapped roots and dynamic targets remain unresolved."
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
