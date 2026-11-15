use std::env;
use std::fs::File;
use std::io::Read;
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

    // Read at most limit + 1 bytes so a file that grows after open cannot
    // bypass the limit and force an unbounded allocation.
    let file = File::open(&path).map_err(|error| format!("cannot open manifest: {error}"))?;
    let mut bytes = Vec::new();
    file.take(MAX_MANIFEST_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|error| format!("cannot read manifest: {error}"))?;
    if bytes.len() as u64 > MAX_MANIFEST_BYTES {
        return Err(format!(
            "manifest exceeds the {} MiB input limit",
            MAX_MANIFEST_BYTES / (1024 * 1024)
        ));
    }

    let manifest: ReplayManifest = serde_json::from_slice(&bytes)
        .map_err(|error| format!("invalid receipt manifest JSON: {error}"))?;
    let replay =
        replay_manifest(manifest).map_err(|error| format!("lifecycle replay rejected: {error}"))?;

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
