use mycelix_stewardship_runtime_store::{
    CasOutcomeV1, InsertOnceOutcomeV1, RuntimeStoreError, RuntimeStoreV1, StateCellKeyV1,
    StateCellValueV1, TransitionWriteV1,
};
use rusqlite::{params, Connection, TransactionBehavior};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

const NAMESPACE: &str = "runtime-preflight";
const GENESIS_MARKER: u8 = 40;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    let _program = args.next();
    let command = args.next().ok_or("missing command")?;
    match command.as_str() {
        "create-empty" => {
            let path = args.next().ok_or("missing db path")?;
            RuntimeStoreV1::create(path)?;
        }
        "init" => {
            let path = args.next().ok_or("missing db path")?;
            let mut store = RuntimeStoreV1::create(path)?;
            let outcome = store.insert_once(&key(), &state(0, GENESIS_MARKER))?;
            assert_eq!(outcome, InsertOnceOutcomeV1::Inserted);
        }
        "init-racer" => {
            let path = args.next().ok_or("missing db path")?;
            let ready = args.next().ok_or("missing ready path")?;
            let start = args.next().ok_or("missing start path")?;
            let mut store = RuntimeStoreV1::open(path)?;
            signal_ready_and_wait(&ready, &start)?;
            match store.insert_once(&key(), &state(0, GENESIS_MARKER))? {
                InsertOnceOutcomeV1::Inserted => println!("INSERTED"),
                InsertOnceOutcomeV1::ExistingExact => println!("EXISTING_EXACT"),
            }
        }
        "cas-racer" => {
            let path = args.next().ok_or("missing db path")?;
            let ready = args.next().ok_or("missing ready path")?;
            let start = args.next().ok_or("missing start path")?;
            let next_marker = parse_u8(args.next(), "next marker")?;
            let receipt_marker = parse_u8(args.next(), "receipt marker")?;
            let mut store = RuntimeStoreV1::open(path)?;
            signal_ready_and_wait(&ready, &start)?;
            match store.compare_and_swap(
                &key(),
                &transition(0, GENESIS_MARKER, next_marker, receipt_marker),
            )? {
                CasOutcomeV1::Applied(_) => println!("APPLIED"),
                CasOutcomeV1::Stale(_) => println!("STALE"),
            }
        }
        "check" => {
            let path = args.next().ok_or("missing db path")?;
            let expected_version: u64 = args
                .next()
                .ok_or("missing expected version")?
                .parse()?;
            let marker = parse_u8(args.next(), "marker")?;
            let receipt = args.next().ok_or("missing receipt or -")?;
            check_state(&path, expected_version, marker, &receipt)?;
        }
        "check-race-one" => {
            let path = args.next().ok_or("missing db path")?;
            let store = RuntimeStoreV1::open(path)?;
            let current = store.load_state(&key())?.ok_or("state missing")?;
            assert_eq!(current.version(), 1);
            let marker = current.commitment()[0];
            assert!(marker == 41 || marker == 42);
            let first = store.load_receipt(NAMESPACE, &[51])?.is_some();
            let second = store.load_receipt(NAMESPACE, &[52])?.is_some();
            assert_ne!(first, second, "exactly one competing receipt must exist");
            println!("RACE_WINNER_MARKER={marker}");
        }
        "abort-before" => std::process::abort(),
        "abort-after-update" => {
            let path = args.next().ok_or("missing db path")?;
            let mut connection = Connection::open(path)?;
            connection.busy_timeout(Duration::from_secs(5))?;
            let tx = connection.transaction_with_behavior(TransactionBehavior::Immediate)?;
            let affected = tx.execute(
                "UPDATE state_cells SET version = 1, commitment = ?1, payload = ?2 \
                 WHERE namespace = ?3 AND state_key = ?4 AND version = 0",
                params![vec![99_u8; 32], vec![99_u8], NAMESPACE, vec![1_u8]],
            )?;
            assert_eq!(affected, 1);
            std::process::abort();
        }
        "commit-then-abort" => {
            let path = args.next().ok_or("missing db path")?;
            let mut store = RuntimeStoreV1::open(path)?;
            match store.compare_and_swap(
                &key(),
                &transition(0, GENESIS_MARKER, 41, 61),
            )? {
                CasOutcomeV1::Applied(_) => std::process::abort(),
                CasOutcomeV1::Stale(_) => return Err("commit-then-abort unexpectedly stale".into()),
            }
        }
        "hold-lock" => {
            let path = args.next().ok_or("missing db path")?;
            let ready = args.next().ok_or("missing ready path")?;
            let millis: u64 = args.next().ok_or("missing hold millis")?.parse()?;
            let mut connection = Connection::open(path)?;
            let tx = connection.transaction_with_behavior(TransactionBehavior::Immediate)?;
            fs::write(&ready, b"ready")?;
            thread::sleep(Duration::from_millis(millis));
            tx.rollback()?;
        }
        "cas-expect-contention" => {
            let path = args.next().ok_or("missing db path")?;
            let mut store = RuntimeStoreV1::open(path)?;
            match store.compare_and_swap(
                &key(),
                &transition(0, GENESIS_MARKER, 41, 71),
            ) {
                Err(RuntimeStoreError::Sqlite(_)) => println!("CONTENTION"),
                Err(other) => return Err(format!("unexpected semantic error: {other}").into()),
                Ok(other) => return Err(format!("lock unexpectedly permitted transition: {other:?}").into()),
            }
        }
        other => return Err(format!("unknown command: {other}").into()),
    }
    Ok(())
}

fn key() -> StateCellKeyV1 {
    StateCellKeyV1::new(NAMESPACE, vec![1]).expect("fixed valid key")
}

fn state(version: u64, marker: u8) -> StateCellValueV1 {
    StateCellValueV1::new(version, [marker; 32], vec![marker]).expect("fixed valid state")
}

fn transition(
    expected_version: u64,
    expected_marker: u8,
    next_marker: u8,
    receipt_marker: u8,
) -> TransitionWriteV1 {
    TransitionWriteV1::new(
        expected_version,
        [expected_marker; 32],
        state(expected_version + 1, next_marker),
        vec![receipt_marker],
        [receipt_marker; 32],
        vec![receipt_marker],
    )
    .expect("fixed valid transition")
}

fn parse_u8(value: Option<String>, label: &str) -> Result<u8, Box<dyn Error>> {
    Ok(value.ok_or_else(|| format!("missing {label}"))?.parse()?)
}

fn signal_ready_and_wait(ready: &str, start: &str) -> Result<(), Box<dyn Error>> {
    fs::write(ready, b"ready")?;
    let deadline = Instant::now() + Duration::from_secs(20);
    while !Path::new(start).exists() {
        if Instant::now() >= deadline {
            return Err("start barrier timed out".into());
        }
        thread::sleep(Duration::from_millis(10));
    }
    Ok(())
}

fn check_state(
    path: &str,
    expected_version: u64,
    marker: u8,
    receipt: &str,
) -> Result<(), Box<dyn Error>> {
    let store = RuntimeStoreV1::open(path)?;
    let current = store.load_state(&key())?.ok_or("state missing")?;
    assert_eq!(current.version(), expected_version);
    assert_eq!(current.commitment(), [marker; 32]);
    if receipt != "-" {
        let receipt_marker: u8 = receipt.parse()?;
        assert!(store.load_receipt(NAMESPACE, &[receipt_marker])?.is_some());
    }
    println!("STATE_OK version={expected_version} marker={marker}");
    Ok(())
}
