//! Reproducible scale probe for durable journal startup replay.
//!
//! Generates a local synthetic J3 journal, then measures write/generation time
//! separately from opening and replaying it. Use /usr/bin/time -v around the
//! already-built executable to record peak resident memory. This is a diagnostic
//! harness, not a benchmark result or a production workload model.

use civ_econ_transition_validator::durable_journal::{DurableEffectJournal, EffectStatus};
use std::env;
use std::error::Error;
use std::fs::{self, DirBuilder, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const PROVIDER_PROFILE_DIGEST: &str =
    "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
const REQUEST_DIGEST: &str =
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const RECEIPT_DIGEST: &str =
    "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
const SOURCE_EVIDENCE_DIGEST: &str =
    "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
const MAX_EFFECTS: usize = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Scenario {
    Pending,
    Indeterminate,
    Acknowledged,
    Reconciled,
}

impl Scenario {
    fn parse(value: &str) -> Option<Self> {
        match value {
            "pending" => Some(Self::Pending),
            "indeterminate" => Some(Self::Indeterminate),
            "acknowledged" => Some(Self::Acknowledged),
            "reconciled" => Some(Self::Reconciled),
            _ => None,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::Pending => "pending",
            Self::Indeterminate => "indeterminate",
            Self::Acknowledged => "acknowledged",
            Self::Reconciled => "reconciled",
        }
    }

    fn expected_status(self) -> EffectStatus {
        match self {
            Self::Pending => EffectStatus::Pending,
            Self::Indeterminate => EffectStatus::Indeterminate,
            Self::Acknowledged | Self::Reconciled => EffectStatus::Acknowledged {
                receipt_digest: RECEIPT_DIGEST.to_owned(),
            },
        }
    }

    fn expected_unresolved(self, count: usize) -> usize {
        match self {
            Self::Pending | Self::Indeterminate => count,
            Self::Acknowledged | Self::Reconciled => 0,
        }
    }
}

struct TempDir(PathBuf);

impl TempDir {
    fn new() -> io::Result<Self> {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        let path = env::temp_dir().join(format!(
            "mycelix-journal-replay-scale-{}-{nonce}",
            std::process::id()
        ));
        let mut builder = DirBuilder::new();
        #[cfg(unix)]
        {
            use std::os::unix::fs::DirBuilderExt;
            builder.mode(0o700);
        }
        builder.create(&path)?;
        Ok(Self(path))
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        encoded.push(HEX[(byte >> 4) as usize] as char);
        encoded.push(HEX[(byte & 0x0f) as usize] as char);
    }
    encoded
}

fn invalid_input(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let count = args
        .next()
        .unwrap_or_else(|| "10000".to_owned())
        .parse::<usize>()?;
    if !(1..=MAX_EFFECTS).contains(&count) {
        return Err(invalid_input("effect count must be in 1..=1_000_000").into());
    }
    let scenario_name = args.next().unwrap_or_else(|| "acknowledged".to_owned());
    let scenario = Scenario::parse(&scenario_name).ok_or_else(|| {
        invalid_input("scenario must be pending, indeterminate, acknowledged, or reconciled")
    })?;
    if args.next().is_some() {
        return Err(invalid_input("provide at most an effect count and scenario").into());
    }

    let temp = TempDir::new()?;
    let journal_path = temp.0.join("effects.journal");
    let mut options = OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let file = options.open(&journal_path)?;
    let mut writer = BufWriter::new(file);

    let generation_started = Instant::now();
    for index in 0..count {
        let id = format!("scale-effect-{index:010}");
        let encoded_id = hex_encode(id.as_bytes());
        writeln!(
            writer,
            "J3\tB\t{}\t{}\t{}",
            encoded_id, REQUEST_DIGEST, PROVIDER_PROFILE_DIGEST
        )?;

        if matches!(scenario, Scenario::Indeterminate | Scenario::Reconciled) {
            writeln!(
                writer,
                "J3\tI\t{}\t{}\t{}",
                encoded_id, REQUEST_DIGEST, PROVIDER_PROFILE_DIGEST
            )?;
        }
        if matches!(scenario, Scenario::Acknowledged | Scenario::Reconciled) {
            writeln!(
                writer,
                "J3\tA\t{}\t{}\t{}\t{}\t{}",
                encoded_id,
                REQUEST_DIGEST,
                PROVIDER_PROFILE_DIGEST,
                RECEIPT_DIGEST,
                SOURCE_EVIDENCE_DIGEST
            )?;
        }
    }
    writer.flush()?;
    writer.get_ref().sync_all()?;
    let generation_elapsed = generation_started.elapsed();
    drop(writer);

    let journal_bytes = fs::metadata(&journal_path)?.len();
    let open_started = Instant::now();
    let journal = DurableEffectJournal::open(&journal_path)?;
    let open_and_replay_elapsed = open_started.elapsed();

    let first_id = "scale-effect-0000000000";
    let last_id = format!("scale-effect-{:010}", count - 1);
    let expected_status = scenario.expected_status();
    let first_status_matches = journal.status(first_id) == Some(expected_status.clone());
    let last_status_matches = journal.status(&last_id) == Some(expected_status);
    let replayed_effect_count = journal.effect_count();
    let replayed_unresolved_count = journal.unresolved_effect_count();
    if replayed_effect_count != count {
        return Err(io::Error::other(format!(
            "expected {count} effects after replay, recovered {replayed_effect_count}"
        )).into());
    }
    if replayed_unresolved_count != scenario.expected_unresolved(count) {
        return Err(io::Error::other(format!(
            "scenario {} expected {} unresolved effects, recovered {}",
            scenario.as_str(),
            scenario.expected_unresolved(count),
            replayed_unresolved_count
        )).into());
    }
    if !first_status_matches || !last_status_matches {
        return Err(io::Error::other("first/last replayed states do not match selected scenario").into());
    }

    println!("scenario={}", scenario.as_str());
    println!("effect_count={count}");
    println!("replayed_effect_count={replayed_effect_count}");
    println!("replayed_unresolved_effect_count={replayed_unresolved_count}");
    println!("all_counts_and_endpoint_states_verified=true");
    println!("journal_bytes={journal_bytes}");
    println!("bytes_per_effect={:.2}", journal_bytes as f64 / count as f64);
    println!("generation_ms={:.3}", generation_elapsed.as_secs_f64() * 1000.0);
    println!(
        "open_and_replay_ms={:.3}",
        open_and_replay_elapsed.as_secs_f64() * 1000.0
    );
    println!("memory_note=measure peak RSS with /usr/bin/time -v around this executable");

    drop(journal);
    Ok(())
}
