//! Reproducible scale probe for durable journal startup replay.
//!
//! Generates a local synthetic J3 journal, then measures write/generation time
//! separately from opening and replaying it. Use /usr/bin/time -v around the
//! already-built executable to record peak resident memory. This is a diagnostic
//! harness, not a benchmark result or a production workload model.

use civ_econ_transition_validator::durable_journal::DurableEffectJournal;
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
const MAX_EFFECTS: usize = 1_000_000;

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
    if args.next().is_some() {
        return Err(invalid_input("provide at most one effect-count argument").into());
    }
    if count == 0 || count > MAX_EFFECTS {
        return Err(invalid_input("effect count must be in 1..=1_000_000").into());
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
        writeln!(
            writer,
            "J3\tB\t{}\t{}\t{}",
            hex_encode(id.as_bytes()),
            REQUEST_DIGEST,
            PROVIDER_PROFILE_DIGEST,
        )?;
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
    let first_present = journal.status(first_id).is_some();
    let last_present = journal.status(&last_id).is_some();
    if !first_present || !last_present {
        return Err(io::Error::other("replay did not preserve first and last generated effect").into());
    }

    println!("effect_count={count}");
    println!("journal_bytes={journal_bytes}");
    println!("bytes_per_effect={:.2}", journal_bytes as f64 / count as f64);
    println!("generation_ms={:.3}", generation_elapsed.as_secs_f64() * 1000.0);
    println!(
        "open_and_replay_ms={:.3}",
        open_and_replay_elapsed.as_secs_f64() * 1000.0
    );
    println!("first_and_last_records_verified=true");
    println!("memory_note=measure peak RSS with /usr/bin/time -v around this executable");

    drop(journal);
    Ok(())
}
