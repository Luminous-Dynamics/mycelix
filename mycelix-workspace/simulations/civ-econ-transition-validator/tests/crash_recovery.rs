//! Process-boundary integration coverage for the durable effect journal.
//!
//! The "provider" below is a local deterministic fixture, not a real payment rail.
//! The child exits without unwinding after recording the external effect, leaving
//! a stale lock just as an abrupt process death would. The parent performs the
//! explicit test-only stale-lock recovery that production must gate on operator
//! evidence that the previous owner is no longer running.

use civ_econ_transition_validator::durable_journal::{BeginResult, DurableEffectJournal};
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

const CHILD_MODE: &str = "MYCELIX_CIV_ECON_CRASH_TEST_CHILD";
const JOURNAL_ENV: &str = "MYCELIX_CIV_ECON_CRASH_TEST_JOURNAL";
const PROVIDER_ENV: &str = "MYCELIX_CIV_ECON_CRASH_TEST_PROVIDER_LOG";
const EFFECT_ID: &str = "payment-crash-1";
const REQUEST_DIGEST: &str =
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const RECEIPT_DIGEST: &str =
    "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
const INTENTIONAL_CRASH_EXIT: i32 = 86;

struct TestDir(PathBuf);

impl TestDir {
    fn new() -> Self {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock should be after epoch")
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "mycelix-civ-econ-crash-test-{}-{timestamp}",
            std::process::id()
        ));
        fs::create_dir(&path).expect("create isolated test directory");
        Self(path)
    }
}

impl Drop for TestDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

/// Minimal driver contract: only a freshly synced intent authorizes dispatch.
/// Recovery states are deliberately non-dispatching and must be reconciled.
fn dispatch_only_if_started(
    journal: &mut DurableEffectJournal,
    provider_path: &Path,
) -> BeginResult {
    let result = journal
        .begin_effect(EFFECT_ID, REQUEST_DIGEST)
        .expect("begin or recover synthetic effect");

    if result == BeginResult::Started {
        let mut provider = OpenOptions::new()
            .create(true)
            .append(true)
            .open(provider_path)
            .expect("open synthetic provider ledger");
        writeln!(provider, "{EFFECT_ID}").expect("record synthetic external effect");
        provider
            .sync_all()
            .expect("sync synthetic provider ledger");
    }

    result
}

#[test]
fn crash_between_dispatch_and_acknowledgement_reconciles_without_duplicate_dispatch() {
    // Child mode: persist intent, perform one synthetic external effect, then
    // terminate without unwinding or writing an acknowledgement receipt.
    if std::env::var_os(CHILD_MODE).is_some() {
        let journal_path = PathBuf::from(
            std::env::var_os(JOURNAL_ENV).expect("child journal path is supplied"),
        );
        let provider_path = PathBuf::from(
            std::env::var_os(PROVIDER_ENV).expect("child provider log path is supplied"),
        );

        let mut journal =
            DurableEffectJournal::open(&journal_path).expect("open child journal");
        assert_eq!(
            dispatch_only_if_started(&mut journal, &provider_path),
            BeginResult::Started,
        );

        // Deliberately skip destructors: this models abrupt termination in the
        // critical window after the external effect but before journal ack.
        std::process::exit(INTENTIONAL_CRASH_EXIT);
    }

    let temp = TestDir::new();
    let journal_path = temp.0.join("effects.journal");
    let provider_path = temp.0.join("provider-ledger.log");

    let child_status = Command::new(std::env::current_exe().expect("test executable path"))
        .arg("--exact")
        .arg("crash_between_dispatch_and_acknowledgement_reconciles_without_duplicate_dispatch")
        .arg("--nocapture")
        .env(CHILD_MODE, "1")
        .env(JOURNAL_ENV, &journal_path)
        .env(PROVIDER_ENV, &provider_path)
        .output()
        .expect("launch crash-boundary child");
    assert_eq!(
        child_status.status.code(),
        Some(INTENTIONAL_CRASH_EXIT),
        "child must terminate at the injected crash point; stdout: {}; stderr: {}",
        String::from_utf8_lossy(&child_status.stdout),
        String::from_utf8_lossy(&child_status.stderr),
    );

    let lock_path = journal_path.with_file_name("effects.journal.lock");
    assert!(
        lock_path.exists(),
        "abrupt termination should leave a stale lock for explicit recovery",
    );
    // Test harness has observed child termination; this is explicit recovery,
    // not automatic stale-lock deletion by DurableEffectJournal::open.
    fs::remove_file(&lock_path).expect("explicitly recover dead child's stale lock");

    let provider_effects: Vec<String> = fs::read_to_string(&provider_path)
        .expect("read synthetic provider ledger")
        .lines()
        .map(str::to_owned)
        .collect();
    assert_eq!(provider_effects, vec![EFFECT_ID.to_owned()]);

    {
        let mut recovered =
            DurableEffectJournal::open(&journal_path).expect("reopen journal after crash");

        // A surviving intent is not permission to dispatch again. The caller
        // queries the synthetic provider ledger and records its observed result.
        assert_eq!(
            dispatch_only_if_started(&mut recovered, &provider_path),
            BeginResult::PendingNeedsReconciliation,
        );
        recovered
            .acknowledge_effect(EFFECT_ID, REQUEST_DIGEST, RECEIPT_DIGEST)
            .expect("record reconciled synthetic provider receipt");
    }

    let mut verified =
        DurableEffectJournal::open(&journal_path).expect("reopen reconciled journal");
    assert_eq!(
        verified.begin_effect(EFFECT_ID, REQUEST_DIGEST),
        Ok(BeginResult::AlreadyAcknowledged {
            receipt_digest: RECEIPT_DIGEST.to_owned(),
        }),
    );
    let provider_effects_after_recovery: Vec<String> = fs::read_to_string(&provider_path)
        .expect("read synthetic provider ledger after recovery")
        .lines()
        .map(str::to_owned)
        .collect();
    assert_eq!(
        provider_effects_after_recovery,
        vec![EFFECT_ID.to_owned()],
        "recovery must not issue a second synthetic external effect",
    );
}
