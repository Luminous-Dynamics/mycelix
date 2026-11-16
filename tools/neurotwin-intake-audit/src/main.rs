use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process;

use neurotwin_intake_audit::{DEFAULT_MIN_REPORT_N, audit_dataset, render_json};

fn output_path_outside_dataset(root: &Path, output: &Path) -> Result<PathBuf, String> {
    let parent = output
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let file_name = output
        .file_name()
        .ok_or_else(|| "output must name a file".to_string())?;
    let canonical_parent = fs::canonicalize(parent)
        .map_err(|_| "output parent directory is unavailable".to_string())?;
    let destination = canonical_parent.join(file_name);

    if destination.starts_with(root) {
        return Err("aggregate receipt must be written outside the dataset directory".to_string());
    }
    if let Ok(metadata) = fs::symlink_metadata(&destination)
        && (metadata.file_type().is_symlink() || !metadata.is_file())
    {
        return Err("output destination must be a regular file path".to_string());
    }

    Ok(destination)
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 2 || args.len() > 3 {
        return Err(
            "usage: neurotwin-intake-audit DATASET_ROOT OUTPUT.json [MIN_SUBJECT_COUNT]"
                .to_string(),
        );
    }

    let minimum_subject_count = if let Some(value) = args.get(2) {
        value
            .parse::<usize>()
            .map_err(|_| "MIN_SUBJECT_COUNT must be a positive integer".to_string())?
    } else {
        DEFAULT_MIN_REPORT_N
    };
    if minimum_subject_count < DEFAULT_MIN_REPORT_N {
        return Err(format!(
            "MIN_SUBJECT_COUNT cannot be below the built-in privacy floor of {DEFAULT_MIN_REPORT_N}"
        ));
    }

    let dataset_root = Path::new(&args[0]);
    let canonical_root =
        fs::canonicalize(dataset_root).map_err(|_| "dataset root is unavailable".to_string())?;
    let output = output_path_outside_dataset(&canonical_root, Path::new(&args[1]))?;

    // Pass the original root so audit_dataset can reject a symlink root.
    let audit = audit_dataset(dataset_root)?;
    let receipt = render_json(&audit, minimum_subject_count);
    fs::write(&output, receipt)
        .map_err(|_| "could not write aggregate intake receipt".to_string())?;

    eprintln!(
        "aggregate intake receipt written; subject identifiers were not emitted and modality contents were not read"
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("neurotwin-intake-audit: {error}");
        process::exit(2);
    }
}
