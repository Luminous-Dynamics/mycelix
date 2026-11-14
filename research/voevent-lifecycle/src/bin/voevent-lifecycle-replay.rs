use std::env;
use std::fs;
use std::process::ExitCode;

use voevent_lifecycle_replay::{ReplayManifest, replay_manifest};

const MAX_MANIFEST_BYTES: u64 = 64 * 1024 * 1024;

fn run() -> Result<(), String> {
    let mut arguments = env::args_os().skip(1);
    let Some(path) = arguments.next() else {
        return Err("usage: voevent-lifecycle-replay <receipt-manifest.json>".to_owned());
    };
    if arguments.next().is_some() {
        return Err("usage: voevent-lifecycle-replay <receipt-manifest.json>".to_owned());
    }

    let metadata = fs::metadata(&path)
        .map_err(|error| format!("cannot inspect manifest: {error}"))?;
    if metadata.len() > MAX_MANIFEST_BYTES {
        return Err(format!(
            "manifest exceeds the {} MiB input limit",
            MAX_MANIFEST_BYTES / (1024 * 1024)
        ));
    }

    let bytes = fs::read(&path).map_err(|error| format!("cannot read manifest: {error}"))?;
    let manifest: ReplayManifest = serde_json::from_slice(&bytes)
        .map_err(|error| format!("invalid receipt manifest JSON: {error}"))?;
    let replay = replay_manifest(manifest)
        .map_err(|error| format!("lifecycle replay rejected: {error}"))?;

    let output = serde_json::to_string_pretty(&replay)
        .map_err(|error| format!("cannot serialize replay output: {error}"))?;
    println!("{output}");
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::from(2)
        }
    }
}
