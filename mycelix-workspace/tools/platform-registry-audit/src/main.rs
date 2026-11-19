use serde::Deserialize;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const REGISTRY_PATH: &str = "docs/architecture/mycelix-platform-registry-v0.1.json";
const UNIFIED_PATH: &str = "mycelix-workspace/happs/mycelix-unified-happ.yaml";
const RESILIENCE_PATH: &str = "mycelix-workspace/happs/mycelix-resilience-happ.yaml";
const SCHEMA_VERSION: &str = "mycelix-platform-registry-v0.1";

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

#[derive(Debug)]
struct LoadedManifest {
    profile_id: &'static str,
    relative_path: &'static str,
    blob_sha: String,
    manifest: HappManifest,
}

fn git_output(root: &Path, args: &[&str]) -> Result<String, Box<dyn Error>> {
    let output = Command::new("git").args(args).current_dir(root).output()?;
    if !output.status.success() {
        return Err(format!(
            "git {:?} failed: {}",
            args,
            String::from_utf8_lossy(&output.stderr).trim()
        )
        .into());
    }
    Ok(String::from_utf8(output.stdout)?.trim().to_string())
}

fn validate_manifest(manifest: &HappManifest) -> Result<(), String> {
    if manifest.name.trim().is_empty() {
        return Err("hApp manifest name must not be empty".to_string());
    }
    if manifest.roles.is_empty() {
        return Err(format!("hApp '{}' has no roles", manifest.name));
    }

    let mut names = BTreeSet::new();
    for role in &manifest.roles {
        if role.name.trim().is_empty() {
            return Err(format!("hApp '{}' contains an empty role name", manifest.name));
        }
        if role.dna.path.trim().is_empty() {
            return Err(format!("role '{}' has an empty DNA path", role.name));
        }
        if !names.insert(role.name.as_str()) {
            return Err(format!(
                "hApp '{}' contains duplicate role '{}'",
                manifest.name, role.name
            ));
        }
    }
    Ok(())
}

fn load_manifest(
    root: &Path,
    profile_id: &'static str,
    relative_path: &'static str,
) -> Result<LoadedManifest, Box<dyn Error>> {
    let path = root.join(relative_path);
    let yaml = fs::read_to_string(&path)?;
    let manifest: HappManifest = yaml_serde::from_str(&yaml)?;
    validate_manifest(&manifest).map_err(std::io::Error::other)?;

    // Refuse to stamp a snapshot from an uncommitted manifest. The Git blob
    // SHA is evidence for the exact committed source, not merely file bytes.
    let status = git_output(root, &["status", "--porcelain", "--", relative_path])?;
    if !status.is_empty() {
        return Err(format!(
            "manifest '{}' has uncommitted changes; commit it before refreshing the registry",
            relative_path
        )
        .into());
    }

    let blob_sha = git_output(root, &["rev-parse", &format!("HEAD:{relative_path}")])?;
    Ok(LoadedManifest {
        profile_id,
        relative_path,
        blob_sha,
        manifest,
    })
}

fn current_source(root: &Path) -> Result<(String, String), Box<dyn Error>> {
    let commit = git_output(root, &["rev-parse", "HEAD"])?;
    let mut branch = git_output(root, &["branch", "--show-current"])?;
    if branch.is_empty() {
        branch = env::var("GITHUB_REF_NAME").unwrap_or_else(|_| "detached".to_string());
    }
    Ok((branch, commit))
}

fn manifest_roles(manifest: &HappManifest) -> Vec<String> {
    manifest.roles.iter().map(|role| role.name.clone()).collect()
}

fn manifest_role_map(manifest: &HappManifest) -> BTreeMap<String, String> {
    manifest
        .roles
        .iter()
        .map(|role| (role.name.clone(), role.dna.path.clone()))
        .collect()
}

fn build_snapshot(
    existing: &Value,
    unified: &LoadedManifest,
    resilience: &LoadedManifest,
    branch: &str,
    commit: &str,
) -> Result<Value, Box<dyn Error>> {
    let existing_roles: BTreeMap<String, Value> = existing
        .get("roles")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|role| {
            let id = role.get("role_id")?.as_str()?;
            Some((id.to_string(), role.clone()))
        })
        .collect();

    let mut roles = Vec::new();
    for role in &unified.manifest.roles {
        let mut value = existing_roles.get(&role.name).cloned().unwrap_or_else(|| {
            json!({
                "role_id": role.name,
                "dna_artifact_path": role.dna.path,
                "deployment_presence": "manifest-listed",
                "trust_classification": "unclassified",
                "authority_classification": "unclassified",
                "logical_domains": [],
                "notes": "New manifest role; trust, authority and domain ownership require explicit review."
            })
        });
        value["role_id"] = json!(role.name);
        value["dna_artifact_path"] = json!(role.dna.path);
        value["deployment_presence"] = json!("manifest-listed");
        if value.get("trust_classification").is_none() {
            value["trust_classification"] = json!("unclassified");
        }
        if value.get("authority_classification").is_none() {
            value["authority_classification"] = json!("unclassified");
        }
        if value.get("logical_domains").is_none() {
            value["logical_domains"] = json!([]);
        }
        roles.push(value);
    }

    let profiles = vec![
        json!({
            "profile_id": unified.profile_id,
            "manifest_path": unified.relative_path,
            "manifest_git_blob_sha": unified.blob_sha,
            "roles": manifest_roles(&unified.manifest)
        }),
        json!({
            "profile_id": resilience.profile_id,
            "manifest_path": resilience.relative_path,
            "manifest_git_blob_sha": resilience.blob_sha,
            "roles": manifest_roles(&resilience.manifest)
        }),
    ];

    let known_discrepancies = existing
        .get("known_discrepancies")
        .cloned()
        .unwrap_or_else(|| json!([]));

    Ok(json!({
        "schema_version": SCHEMA_VERSION,
        "snapshot_kind": "non-authoritative-manifest-derived-snapshot",
        "source": {
            "repository": "Luminous-Dynamics/mycelix",
            "branch": branch,
            "commit": commit,
            "manifest_path": unified.relative_path,
            "manifest_git_blob_sha": unified.blob_sha
        },
        "profiles": profiles,
        "roles": roles,
        "known_discrepancies": known_discrepancies
    }))
}

fn validate_snapshot(
    snapshot: &Value,
    unified: &LoadedManifest,
    resilience: &LoadedManifest,
    root: &Path,
) -> Result<(), Box<dyn Error>> {
    if snapshot["schema_version"] != SCHEMA_VERSION {
        return Err("registry schema_version does not match the expected v0.1 schema".into());
    }
    if snapshot["snapshot_kind"] != "non-authoritative-manifest-derived-snapshot" {
        return Err("registry snapshot_kind is missing or invalid".into());
    }
    if snapshot["source"]["repository"] != "Luminous-Dynamics/mycelix" {
        return Err("registry source repository is not Luminous-Dynamics/mycelix".into());
    }
    if snapshot["source"]["manifest_path"] != unified.relative_path {
        return Err("registry source manifest_path does not match the unified manifest".into());
    }
    if snapshot["source"]["manifest_git_blob_sha"] != unified.blob_sha {
        return Err("unified manifest Git blob SHA differs from the registry snapshot".into());
    }

    let source_commit = snapshot["source"]["commit"]
        .as_str()
        .ok_or("registry source.commit is missing")?;
    if source_commit.len() != 40 || !source_commit.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("registry source.commit must be a 40-character Git SHA".into());
    }

    let source_unified_blob = git_output(
        root,
        &["rev-parse", &format!("{source_commit}:{UNIFIED_PATH}")],
    )?;
    if source_unified_blob != unified.blob_sha {
        return Err("registry source.commit does not contain the declared unified manifest blob".into());
    }
    let source_resilience_blob = git_output(
        root,
        &["rev-parse", &format!("{source_commit}:{RESILIENCE_PATH}")],
    )?;
    if source_resilience_blob != resilience.blob_sha {
        return Err("registry source.commit does not contain the declared resilience manifest blob".into());
    }

    let profiles = snapshot["profiles"]
        .as_array()
        .ok_or("registry profiles must be an array")?;
    if profiles.len() != 2 {
        return Err(format!("expected exactly 2 deployment profiles, found {}", profiles.len()).into());
    }

    let mut profile_roles = BTreeMap::new();
    for profile in profiles {
        let profile_id = profile["profile_id"]
            .as_str()
            .ok_or("profile_id must be a string")?;
        let manifest = match profile_id {
            "unified" => unified,
            "resilience" => resilience,
            other => return Err(format!("unexpected profile_id '{other}'").into()),
        };
        if profile["manifest_path"] != manifest.relative_path {
            return Err(format!("profile '{profile_id}' manifest_path mismatch").into());
        }
        if profile["manifest_git_blob_sha"] != manifest.blob_sha {
            return Err(format!("profile '{profile_id}' manifest Git blob SHA mismatch").into());
        }
        let actual: Vec<String> = profile["roles"]
            .as_array()
            .ok_or_else(|| format!("profile '{profile_id}' roles must be an array"))?
            .iter()
            .map(|role| {
                role.as_str()
                    .map(str::to_string)
                    .ok_or_else(|| format!("profile '{profile_id}' contains a non-string role"))
            })
            .collect::<Result<_, _>>()?;
        let expected = manifest_roles(&manifest.manifest);
        if actual != expected {
            return Err(format!(
                "profile '{profile_id}' role set/order differs from the manifest; registry={actual:?}, manifest={expected:?}"
            )
            .into());
        }
        profile_roles.insert(profile_id.to_string(), actual);
    }

    let role_values = snapshot["roles"]
        .as_array()
        .ok_or("registry roles must be an array")?;
    let mut seen = BTreeSet::new();
    let mut role_map = BTreeMap::new();
    for role in role_values {
        let id = role["role_id"]
            .as_str()
            .ok_or("role_id must be a string")?;
        let path = role["dna_artifact_path"]
            .as_str()
            .ok_or_else(|| format!("role '{id}' is missing dna_artifact_path"))?;
        if !seen.insert(id.to_string()) {
            return Err(format!("registry contains duplicate role_id '{id}'").into());
        }
        role_map.insert(id.to_string(), path.to_string());

        let trust = role["trust_classification"]
            .as_str()
            .ok_or_else(|| format!("role '{id}' needs an explicit trust_classification"))?;
        if ![
            "unclassified",
            "private-source-chain",
            "permissioned-dht",
            "public-dht",
            "mixed-explicit-policy",
        ]
        .contains(&trust)
        {
            return Err(format!("role '{id}' has invalid trust classification '{trust}'").into());
        }
        let authority = role["authority_classification"]
            .as_str()
            .ok_or_else(|| format!("role '{id}' needs an explicit authority_classification"))?;
        if ![
            "unclassified",
            "peer-authored-declaration",
            "application-authority",
            "projection-only",
            "external-reference-only",
        ]
        .contains(&authority)
        {
            return Err(format!("role '{id}' has invalid authority classification '{authority}'").into());
        }
    }

    let expected_roles = manifest_role_map(&unified.manifest);
    if role_map != expected_roles {
        return Err("registry role IDs or DNA artifact paths differ from the unified manifest".into());
    }

    for (profile_id, role_ids) in profile_roles {
        for role_id in role_ids {
            if !role_map.contains_key(&role_id) {
                return Err(format!(
                    "profile '{profile_id}' references role '{role_id}' absent from registry roles"
                )
                .into());
            }
        }
    }

    Ok(())
}

fn atomic_write(path: &Path, contents: &str) -> Result<(), Box<dyn Error>> {
    let temp_path = path.with_extension("json.tmp");
    fs::write(&temp_path, contents)?;
    fs::rename(&temp_path, path)?;
    Ok(())
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut mode = "check";
    let mut root = env::current_dir()?;
    let args: Vec<String> = env::args().skip(1).collect();
    let mut index = 0;
    while index < args.len() {
        match args[index].as_str() {
            "--check" => mode = "check",
            "--write" => mode = "write",
            "--root" => {
                index += 1;
                let value = args.get(index).ok_or("--root requires a path")?;
                root = PathBuf::from(value);
            }
            "--help" | "-h" => {
                println!(
                    "Usage: mycelix-platform-registry-audit [--check|--write] [--root PATH]\n\
                     --check  verify the committed snapshot against hApp manifests (default)\n\
                     --write  refresh role/profile/path/hash data, preserving human annotations"
                );
                return Ok(());
            }
            unknown => return Err(format!("unknown argument '{unknown}'").into()),
        }
        index += 1;
    }

    let unified = load_manifest(&root, "unified", UNIFIED_PATH)?;
    let resilience = load_manifest(&root, "resilience", RESILIENCE_PATH)?;
    let registry_path = root.join(REGISTRY_PATH);
    let existing: Value = serde_json::from_slice(&fs::read(&registry_path)?)?;

    if mode == "write" {
        let (branch, commit) = current_source(&root)?;
        let updated = build_snapshot(&existing, &unified, &resilience, &branch, &commit)?;
        validate_snapshot(&updated, &unified, &resilience, &root)?;
        atomic_write(&registry_path, &format!("{}\n", serde_json::to_string_pretty(&updated)?))?;
        println!(
            "updated {} from {} roles (unified) and {} roles (resilience)",
            REGISTRY_PATH,
            unified.manifest.roles.len(),
            resilience.manifest.roles.len()
        );
    } else {
        validate_snapshot(&existing, &unified, &resilience, &root)?;
        println!(
            "PASS: registry matches {} unified roles and {} resilience roles",
            unified.manifest.roles.len(),
            resilience.manifest.roles.len()
        );
    }

    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("platform-registry-audit: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duplicate_roles_are_rejected() {
        let manifest = HappManifest {
            name: "test".to_string(),
            roles: vec![
                HappRole {
                    name: "commons".to_string(),
                    dna: DnaConfig { path: "a.dna".to_string() },
                },
                HappRole {
                    name: "commons".to_string(),
                    dna: DnaConfig { path: "b.dna".to_string() },
                },
            ],
        };
        assert!(validate_manifest(&manifest).unwrap_err().contains("duplicate role"));
    }

    #[test]
    fn empty_dna_path_is_rejected() {
        let manifest = HappManifest {
            name: "test".to_string(),
            roles: vec![HappRole {
                name: "commons".to_string(),
                dna: DnaConfig { path: "  ".to_string() },
            }],
        };
        assert!(validate_manifest(&manifest).unwrap_err().contains("empty DNA path"));
    }
}
