// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_stewardship_runtime_store::{
    CasOutcomeV1, InsertOnceOutcomeV1, RuntimeStoreError, RuntimeStoreV1, StateCellKeyV1,
    StateCellValueV1, TransitionWriteV1, BUSY_TIMEOUT_MS_V1, PROFILE_ID_V1, SCHEMA_ID_V1,
    SCHEMA_VERSION_V1,
};
use rusqlite::Connection;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Barrier};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

fn temp_path(label: &str) -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock after unix epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "mycelix-stewardship-runtime-store-{label}-{}-{nonce}.sqlite",
        std::process::id()
    ))
}

fn remove_sqlite_files(path: &Path) {
    let _ = fs::remove_file(path);
    let base = path.to_string_lossy();
    let _ = fs::remove_file(format!("{base}-wal"));
    let _ = fs::remove_file(format!("{base}-shm"));
}

fn key(namespace: &str, suffix: u8) -> StateCellKeyV1 {
    StateCellKeyV1::new(namespace, vec![suffix]).expect("valid key")
}

fn state(version: u64, marker: u8) -> StateCellValueV1 {
    StateCellValueV1::new(version, [marker; 32], vec![marker, marker]).expect("valid state")
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
        vec![receipt_marker, 0xAA],
    )
    .expect("valid transition")
}

#[test]
fn create_and_open_freeze_exact_profile() {
    let path = temp_path("profile");
    let store = RuntimeStoreV1::create(&path).expect("create store");
    let profile = store.profile_snapshot().expect("profile snapshot");

    assert_eq!(profile.profile_id, PROFILE_ID_V1);
    assert_eq!(profile.schema_id, SCHEMA_ID_V1);
    assert_eq!(profile.schema_version, SCHEMA_VERSION_V1);
    assert_eq!(profile.journal_mode.to_ascii_lowercase(), "wal");
    assert_eq!(profile.synchronous, 2);
    assert_eq!(profile.foreign_keys, 1);
    assert_eq!(profile.trusted_schema, 0);
    assert_eq!(profile.busy_timeout_ms, BUSY_TIMEOUT_MS_V1 as i64);
    assert!(!profile.sqlite_version.is_empty());
    assert!(!profile.sqlite_source_id.is_empty());
    store.quick_check().expect("quick_check");
    drop(store);

    let reopened = RuntimeStoreV1::open(&path).expect("reopen store");
    reopened.verify_profile().expect("verify reopened profile");
    drop(reopened);
    remove_sqlite_files(&path);
}

#[test]
fn create_never_overwrites_existing_database() {
    let path = temp_path("create-once");
    let store = RuntimeStoreV1::create(&path).expect("create store");
    drop(store);

    let error = RuntimeStoreV1::create(&path).err().expect("must refuse overwrite");
    assert!(matches!(error, RuntimeStoreError::PathAlreadyExists));
    remove_sqlite_files(&path);
}

#[test]
fn single_assignment_is_exact_or_conflict() {
    let path = temp_path("insert-once");
    let mut store = RuntimeStoreV1::create(&path).expect("create store");
    let cell = key("capability-state", 1);
    let genesis = state(0, 10);

    assert_eq!(
        store.insert_once(&cell, &genesis).expect("first insert"),
        InsertOnceOutcomeV1::Inserted
    );
    assert_eq!(
        store.insert_once(&cell, &genesis).expect("exact duplicate"),
        InsertOnceOutcomeV1::ExistingExact
    );

    let conflicting = state(0, 11);
    let error = store
        .insert_once(&cell, &conflicting)
        .expect_err("conflicting genesis must reject");
    assert!(matches!(error, RuntimeStoreError::InitializationConflict));
    assert_eq!(store.load_state(&cell).expect("load"), Some(genesis));

    drop(store);
    remove_sqlite_files(&path);
}

#[test]
fn compare_and_swap_commits_state_and_receipt_together() {
    let path = temp_path("cas");
    let mut store = RuntimeStoreV1::create(&path).expect("create store");
    let cell = key("capability-state", 2);
    store
        .insert_once(&cell, &state(0, 20))
        .expect("insert genesis");

    let update = transition(0, 20, 21, 31);
    let applied = store
        .compare_and_swap(&cell, &update)
        .expect("cas") ;
    let committed = match applied {
        CasOutcomeV1::Applied(committed) => committed,
        CasOutcomeV1::Stale(_) => panic!("first exact CAS unexpectedly stale"),
    };
    assert_eq!(committed.state().version(), 1);
    assert_eq!(committed.state().commitment(), [21; 32]);
    assert_eq!(committed.receipt().from_version(), 0);
    assert_eq!(committed.receipt().to_version(), 1);
    assert_eq!(committed.receipt().receipt_commitment(), [31; 32]);

    let loaded = store
        .load_receipt(cell.namespace(), update.receipt_id())
        .expect("load receipt")
        .expect("receipt exists");
    assert_eq!(loaded, *committed.receipt());
    assert_eq!(store.load_state(&cell).expect("load"), Some(state(1, 21)));

    drop(store);
    remove_sqlite_files(&path);
}

#[test]
fn stale_compare_and_swap_does_not_mutate() {
    let path = temp_path("stale");
    let mut store = RuntimeStoreV1::create(&path).expect("create store");
    let cell = key("capability-state", 3);
    store
        .insert_once(&cell, &state(0, 30))
        .expect("insert genesis");
    store
        .compare_and_swap(&cell, &transition(0, 30, 31, 41))
        .expect("first cas");

    let stale = store
        .compare_and_swap(&cell, &transition(0, 30, 32, 42))
        .expect("stale result");
    match stale {
        CasOutcomeV1::Stale(current) => {
            assert_eq!(current.version(), 1);
            assert_eq!(current.commitment(), [31; 32]);
        }
        CasOutcomeV1::Applied(_) => panic!("stale CAS must not apply"),
    }
    assert_eq!(store.load_state(&cell).expect("load"), Some(state(1, 31)));

    drop(store);
    remove_sqlite_files(&path);
}

#[test]
fn two_connections_racing_same_prior_state_admit_one_successor() {
    let path = temp_path("race");
    let mut initializer = RuntimeStoreV1::create(&path).expect("create store");
    let cell = key("capability-state", 4);
    initializer
        .insert_once(&cell, &state(0, 40))
        .expect("insert genesis");
    drop(initializer);

    let barrier = Arc::new(Barrier::new(3));
    let mut handles = Vec::new();
    for (next_marker, receipt_marker) in [(41_u8, 51_u8), (42_u8, 52_u8)] {
        let path = path.clone();
        let cell = cell.clone();
        let barrier = Arc::clone(&barrier);
        handles.push(thread::spawn(move || {
            let mut store = RuntimeStoreV1::open(&path).expect("open racer");
            let update = transition(0, 40, next_marker, receipt_marker);
            barrier.wait();
            store.compare_and_swap(&cell, &update).expect("race cas")
        }));
    }
    barrier.wait();

    let outcomes: Vec<_> = handles
        .into_iter()
        .map(|handle| handle.join().expect("join racer"))
        .collect();
    let applied = outcomes
        .iter()
        .filter(|outcome| matches!(outcome, CasOutcomeV1::Applied(_)))
        .count();
    let stale = outcomes
        .iter()
        .filter(|outcome| matches!(outcome, CasOutcomeV1::Stale(_)))
        .count();
    assert_eq!(applied, 1);
    assert_eq!(stale, 1);

    let store = RuntimeStoreV1::open(&path).expect("reopen after race");
    assert_eq!(store.load_state(&cell).expect("load").expect("state").version(), 1);
    drop(store);
    remove_sqlite_files(&path);
}

#[test]
fn receipt_collision_rolls_back_state_update() {
    let path = temp_path("receipt-collision");
    let mut store = RuntimeStoreV1::create(&path).expect("create store");
    let first = key("effect-state", 5);
    let second = key("effect-state", 6);
    store.insert_once(&first, &state(0, 50)).expect("first genesis");
    store.insert_once(&second, &state(0, 60)).expect("second genesis");

    let shared_receipt_id = vec![0x77];
    let first_transition = TransitionWriteV1::new(
        0,
        [50; 32],
        state(1, 51),
        shared_receipt_id.clone(),
        [70; 32],
        vec![1],
    )
    .expect("first transition");
    store
        .compare_and_swap(&first, &first_transition)
        .expect("first cas");

    let second_transition = TransitionWriteV1::new(
        0,
        [60; 32],
        state(1, 61),
        shared_receipt_id,
        [71; 32],
        vec![2],
    )
    .expect("second transition");
    let error = store
        .compare_and_swap(&second, &second_transition)
        .expect_err("receipt collision must reject");
    assert!(matches!(error, RuntimeStoreError::ReceiptConflict));
    assert_eq!(store.load_state(&second).expect("load"), Some(state(0, 60)));

    drop(store);
    remove_sqlite_files(&path);
}

#[test]
fn online_backup_contains_latest_committed_state() {
    let source = temp_path("backup-source");
    let destination = temp_path("backup-destination");
    let mut store = RuntimeStoreV1::create(&source).expect("create store");
    let cell = key("effect-state", 7);
    store.insert_once(&cell, &state(0, 70)).expect("genesis");
    store
        .compare_and_swap(&cell, &transition(0, 70, 71, 81))
        .expect("cas");
    store.backup_to(&destination).expect("online backup");
    drop(store);

    let backup = RuntimeStoreV1::open(&destination).expect("open backup");
    backup.quick_check().expect("backup quick_check");
    assert_eq!(backup.load_state(&cell).expect("load"), Some(state(1, 71)));
    assert!(backup
        .load_receipt(cell.namespace(), &[81])
        .expect("receipt query")
        .is_some());
    drop(backup);

    remove_sqlite_files(&source);
    remove_sqlite_files(&destination);
}

#[test]
fn unsupported_schema_version_fails_closed() {
    let path = temp_path("schema-drift");
    let store = RuntimeStoreV1::create(&path).expect("create store");
    drop(store);

    let connection = Connection::open(&path).expect("raw diagnostic connection");
    connection
        .execute_batch("PRAGMA user_version = 2;")
        .expect("mutate version for adversarial test");
    drop(connection);

    let error = RuntimeStoreV1::open(&path)
        .err()
        .expect("schema drift must refuse open");
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch(_)));
    remove_sqlite_files(&path);
}

#[cfg(unix)]
#[test]
fn database_symlink_is_refused() {
    use std::os::unix::fs::symlink;

    let target = temp_path("symlink-target");
    let link = temp_path("symlink-link");
    let store = RuntimeStoreV1::create(&target).expect("create target");
    drop(store);
    symlink(&target, &link).expect("create symlink");

    let error = RuntimeStoreV1::open(&link)
        .err()
        .expect("symlink must refuse open");
    assert!(matches!(error, RuntimeStoreError::NonRegularFile));

    let _ = fs::remove_file(&link);
    remove_sqlite_files(&target);
}

#[test]
fn bounded_and_monotonic_inputs_fail_closed() {
    assert!(matches!(
        StateCellKeyV1::new(" bad ", vec![1]),
        Err(RuntimeStoreError::InvalidReference)
    ));
    assert!(matches!(
        StateCellValueV1::new(u64::MAX, [1; 32], Vec::<u8>::new()),
        Err(RuntimeStoreError::VersionOutOfRange)
    ));
    assert!(matches!(
        TransitionWriteV1::new(2, [1; 32], state(4, 2), vec![1], [3; 32], vec![]),
        Err(RuntimeStoreError::VersionStepRequired)
    ));
}
