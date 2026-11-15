use std::env;
use std::fs::File;
use std::io::{Read, Take};
use std::process::ExitCode;

use voevent_lifecycle_replay::{
    ReceiptCheckpoint, ReceiptJournalManifest, replay_receipt_sequence,
};

const MAX_JOURNAL_BYTES: u64 = 64 * 1024 * 1024;
const MAX_CHECKPOINT_BYTES: u64 = 4096;

fn read_bounded(path: &std::ffi::OsStr, limit: u64, label: &str) -> Result<Vec<u8>, String> {
    let file = File::open(path).map_err(|error| format!("cannot open {label}: {error}"))?;
    let mut limited: Take<File> = file.take(limit + 1);
    let mut bytes = Vec::new();
    limited
        .read_to_end(&mut bytes)
        .map_err(|error| format!("cannot read {label}: {error}"))?;
    if bytes.len() as u64 > limit {
        return Err(format!("{label} exceeds the {} byte input limit", limit));
    }
    Ok(bytes)
}

fn run() -> Result<(), String> {
    let mut arguments = env::args_os().skip(1);
    let Some(journal_path) = arguments.next() else {
        return Err(
            "usage: voevent-receipt-replay <receipt-journal.json> <trusted-checkpoint.json>"
                .to_owned(),
        );
    };
    let Some(checkpoint_path) = arguments.next() else {
        return Err(
            "usage: voevent-receipt-replay <receipt-journal.json> <trusted-checkpoint.json>"
                .to_owned(),
        );
    };
    if arguments.next().is_some() {
        return Err(
            "usage: voevent-receipt-replay <receipt-journal.json> <trusted-checkpoint.json>"
                .to_owned(),
        );
    }

    let journal_bytes = read_bounded(&journal_path, MAX_JOURNAL_BYTES, "receipt journal")?;
    let checkpoint_bytes =
        read_bounded(&checkpoint_path, MAX_CHECKPOINT_BYTES, "trusted checkpoint")?;

    let manifest: ReceiptJournalManifest = serde_json::from_slice(&journal_bytes)
        .map_err(|error| format!("invalid receipt journal JSON: {error}"))?;
    let checkpoint: ReceiptCheckpoint = serde_json::from_slice(&checkpoint_bytes)
        .map_err(|error| format!("invalid trusted checkpoint JSON: {error}"))?;

    let replay = replay_receipt_sequence(manifest, &checkpoint)
        .map_err(|error| format!("receipt-sequence replay rejected: {error}"))?;
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
