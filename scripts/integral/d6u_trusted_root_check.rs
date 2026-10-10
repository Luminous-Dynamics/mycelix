#![forbid(unsafe_code)]

//! Dependency-free Rust preflight for the D6U trusted source closure.
//!
//! This program does not execute candidate source. It rejects symlinked roots and
//! path components, snapshots each pinned file once, and computes the Git blob
//! identity from those exact bytes. Full behavioral tests for the Python candidate
//! programs remain separate and must still run in the restricted evaluator.

use std::env;
use std::fs;
use std::io::Write;
use std::path::{Component, Path, PathBuf};
use std::process::{Command, Stdio};

#[derive(Clone, Copy)]
struct Pin {
    label: &'static str,
    path: &'static str,
    blob: &'static str,
}

// These values are intentionally literal. The source file itself is independently
// blob-pinned by the calling workflow. Updating any pin requires reviewing the
// policy and the complete trust closure, not just accepting a new digest.
const TRUSTED_PINS: [Pin; 7] = [
    Pin {
        label: "policy",
        path: "docs/integral/d6u-trusted-builder-policy.json",
        blob: "f7c7581017cc5773243a2d25093e5de3cd522e2e",
    },
    Pin {
        label: "attestation workflow",
        path: ".github/workflows/d6u-trusted-evidence-attestation.yml",
        blob: "652f81c65cdf6d6138ff2c7f4be5b0bd74ae3d61",
    },
    Pin {
        label: "trusted artifact verifier",
        path: "scripts/integral/verify_d6u_trusted_artifacts.py",
        blob: "de5b3275dda9cbeaa880d26a9ff2dc0e58d7d65e",
    },
    Pin {
        label: "trusted artifact fetcher",
        path: "scripts/integral/fetch_d6u_trusted_artifact.py",
        blob: "4c4d67f6f9a77c512728a8723cf2aa708687a395",
    },
    Pin {
        label: "attestation verifier",
        path: "scripts/integral/verify_d6u_trusted_attestation.py",
        blob: "91331c11f6c6de76f44c8d977629971193e2c0ba",
    },
    Pin {
        label: "attestation predicate emitter",
        path: "scripts/integral/emit_d6u_trusted_attestation_predicate.py",
        blob: "ce32e161d3f45918ba8ea5545d02e233af331ead",
    },
    Pin {
        label: "attestation retention verifier",
        path: "scripts/integral/verify_d6u_trusted_attestation_retention.py",
        blob: "df1e4fd9ba3c924037ef7ae97d087751ac8e122f",
    },
];

fn safe_relative_components(value: &str) -> Result<Vec<PathBuf>, String> {
    if value.is_empty() || value.contains('\\') {
        return Err(format!("unsafe relative path spelling: {value:?}"));
    }

    let mut parts = Vec::new();
    for component in Path::new(value).components() {
        match component {
            Component::Normal(part) => parts.push(PathBuf::from(part)),
            Component::CurDir | Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(format!("non-canonical relative path: {value:?}"));
            }
        }
    }

    if parts.is_empty() {
        return Err("empty relative path".to_string());
    }
    Ok(parts)
}

fn candidate_root_from_arg(value: &str) -> Result<PathBuf, String> {
    if !value.starts_with('/') || value.contains('\\') {
        return Err(format!("candidate root is not a canonical absolute path: {value:?}"));
    }

    // Path::components() normalizes some spelling differences, so reject those
    // from the original CLI string before constructing a Path.
    let components: Vec<_> = value.split('/').collect();
    if components.first() != Some(&"")
        || components
            .iter()
            .skip(1)
            .any(|part| part.is_empty() || *part == "." || *part == "..")
    {
        return Err(format!("candidate root is not canonically spelled: {value:?}"));
    }

    Ok(PathBuf::from(value))
}

fn reject_symlinked_path_components(path: &Path) -> Result<(), String> {
    if !path.is_absolute() {
        return Err(format!("candidate root is not absolute: {}", path.display()));
    }

    let components: Vec<_> = path.components().collect();
    let mut cursor = PathBuf::new();
    for (index, component) in components.iter().enumerate() {
        match component {
            Component::RootDir => cursor.push(Path::new("/")),
            Component::Normal(part) => cursor.push(part),
            Component::CurDir | Component::ParentDir | Component::Prefix(_) => {
                return Err(format!("candidate root is not canonical: {}", path.display()));
            }
        }

        let metadata = fs::symlink_metadata(&cursor).map_err(|error| {
            format!("cannot inspect candidate-root component {}: {error}", cursor.display())
        })?;
        if metadata.file_type().is_symlink() {
            return Err(format!("candidate root traverses a symlink: {}", cursor.display()));
        }
        if index + 1 < components.len() && !metadata.is_dir() {
            return Err(format!(
                "candidate-root component is not a directory: {}",
                cursor.display()
            ));
        }
    }

    let metadata = fs::symlink_metadata(path)
        .map_err(|error| format!("cannot inspect candidate root {}: {error}", path.display()))?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(format!("candidate root is not a real directory: {}", path.display()));
    }
    Ok(())
}

fn read_candidate_snapshot(root: &Path, relative: &str) -> Result<Vec<u8>, String> {
    let components = safe_relative_components(relative)?;
    let mut cursor = root.to_path_buf();

    for (index, component) in components.iter().enumerate() {
        cursor.push(component);
        let metadata = fs::symlink_metadata(&cursor)
            .map_err(|error| format!("cannot inspect pinned path {relative}: {error}"))?;
        if metadata.file_type().is_symlink() {
            return Err(format!("pinned path traverses a symlink: {relative}"));
        }
        if index + 1 < components.len() && !metadata.is_dir() {
            return Err(format!("pinned path parent is not a directory: {relative}"));
        }
        if index + 1 == components.len() && !metadata.is_file() {
            return Err(format!("pinned path is not a regular file: {relative}"));
        }
    }

    // Hash the bytes returned here, rather than asking Git to reopen this path.
    // No candidate source is executed by this preflight.
    fs::read(&cursor).map_err(|error| format!("cannot read pinned path {relative}: {error}"))
}

fn git_blob_sha1(snapshot: &[u8]) -> Result<String, String> {
    let mut child = Command::new("git")
        .args(["hash-object", "--stdin"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| format!("cannot start git hash-object: {error}"))?;

    child
        .stdin
        .as_mut()
        .ok_or_else(|| "git hash-object stdin unavailable".to_string())?
        .write_all(snapshot)
        .map_err(|error| format!("cannot send source snapshot to git hash-object: {error}"))?;
    drop(child.stdin.take());

    let output = child
        .wait_with_output()
        .map_err(|error| format!("cannot collect git hash-object result: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "git hash-object failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }

    let digest = String::from_utf8(output.stdout)
        .map_err(|error| format!("git hash-object emitted non-UTF-8 output: {error}"))?
        .trim()
        .to_string();
    if digest.len() != 40 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!("git hash-object emitted a malformed SHA-1: {digest:?}"));
    }
    Ok(digest)
}

fn evaluate(candidate_root: &Path) -> Result<(), String> {
    reject_symlinked_path_components(candidate_root)?;

    for pin in TRUSTED_PINS {
        let snapshot = read_candidate_snapshot(candidate_root, pin.path)?;
        let observed = git_blob_sha1(&snapshot)?;
        if observed != pin.blob {
            return Err(format!(
                "trusted {label} blob mismatch for {path}: expected {expected}, observed {observed}",
                label = pin.label,
                path = pin.path,
                expected = pin.blob,
            ));
        }
        println!("D6U Rust source preflight: {label}: PASS ({observed})", label = pin.label);
    }

    println!("D6U Rust trusted-root preflight: PASS");
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 || args[1] != "--candidate-root" {
        eprintln!("usage: d6u_trusted_root_check --candidate-root ABSOLUTE_PATH");
        std::process::exit(2);
    }

    let candidate_root = match candidate_root_from_arg(&args[2]) {
        Ok(path) => path,
        Err(error) => {
            eprintln!("D6U Rust trusted-root preflight: REFUSED: {error}");
            std::process::exit(1);
        }
    };
    if let Err(error) = evaluate(&candidate_root) {
        eprintln!("D6U Rust trusted-root preflight: REFUSED: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn scratch_dir(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock should be after Unix epoch")
            .as_nanos();
        let path = env::temp_dir().join(format!(
            "d6u-rust-root-check-{label}-{}-{nonce}",
            std::process::id()
        ));
        fs::create_dir_all(&path).expect("scratch directory should be created");
        path
    }

    #[test]
    fn rejects_noncanonical_or_host_absolute_relative_paths() {
        assert!(safe_relative_components("docs/integral/policy.json").is_ok());
        assert!(safe_relative_components("../outside").is_err());
        assert!(safe_relative_components("docs/../outside").is_err());
        assert!(safe_relative_components("/etc/passwd").is_err());
        assert!(safe_relative_components(r"docs\policy.json").is_err());
        assert!(safe_relative_components(".").is_err());
        assert!(safe_relative_components("").is_err());

        assert!(candidate_root_from_arg("/tmp/candidate").is_ok());
        assert!(candidate_root_from_arg("relative/candidate").is_err());
        assert!(candidate_root_from_arg("/tmp/../candidate").is_err());
        assert!(candidate_root_from_arg("/tmp/./candidate").is_err());
        assert!(candidate_root_from_arg("/tmp//candidate").is_err());
        assert!(candidate_root_from_arg("/tmp/candidate/").is_err());
        assert!(candidate_root_from_arg(r"/tmp\\candidate").is_err());
    }

    #[test]
    fn git_blob_identity_uses_exact_source_bytes() {
        assert_eq!(
            git_blob_sha1(b"").expect("git hash-object should be available"),
            "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391"
        );
        assert_ne!(
            git_blob_sha1(b"validated snapshot\n").expect("git hash-object should be available"),
            git_blob_sha1(b"reopened path\n").expect("git hash-object should be available")
        );
    }

    #[test]
    fn rejects_a_candidate_root_reached_through_a_symlink() {
        use std::os::unix::fs::symlink;

        let scratch = scratch_dir("root-symlink");
        let real_root = scratch.join("real");
        fs::create_dir(&real_root).expect("real root should be created");
        let alias = scratch.join("alias");
        symlink(&real_root, &alias).expect("symlink should be created");

        assert!(reject_symlinked_path_components(&real_root).is_ok());
        assert!(reject_symlinked_path_components(&alias).is_err());
        fs::remove_dir_all(scratch).expect("scratch tree should be removed");
    }

    #[test]
    fn rejects_symlinked_intermediate_file_components() {
        use std::os::unix::fs::symlink;

        let scratch = scratch_dir("nested-symlink");
        let root = scratch.join("root");
        let real_directory = root.join("real");
        fs::create_dir_all(&real_directory).expect("real directory should be created");
        fs::write(real_directory.join("file.txt"), b"fixture")
            .expect("fixture file should be written");
        symlink(&real_directory, root.join("alias")).expect("symlink should be created");

        assert!(read_candidate_snapshot(&root, "real/file.txt").is_ok());
        assert!(read_candidate_snapshot(&root, "alias/file.txt").is_err());
        assert!(read_candidate_snapshot(&root, "../outside").is_err());
        fs::remove_dir_all(scratch).expect("scratch tree should be removed");
    }
}
