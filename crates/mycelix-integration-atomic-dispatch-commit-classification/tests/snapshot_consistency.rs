use rusqlite::{Connection, TransactionBehavior};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

fn unique_db_path() -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock after epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "mycelix-commit-classifier-{}-{nonce}.sqlite",
        std::process::id()
    ))
}

fn remove_sqlite_files(path: &Path) {
    let _ = fs::remove_file(path);
    let mut wal = path.as_os_str().to_os_string();
    wal.push("-wal");
    let _ = fs::remove_file(PathBuf::from(wal));
    let mut shm = path.as_os_str().to_os_string();
    shm.push("-shm");
    let _ = fs::remove_file(PathBuf::from(shm));
}

#[test]
fn deferred_reread_cannot_splice_predecessor_and_successor_states() {
    let path = unique_db_path();
    remove_sqlite_files(&path);

    {
        let conn = Connection::open(&path).expect("create database");
        conn.pragma_update(None, "journal_mode", "WAL")
            .expect("enable WAL");
        conn.execute_batch(
            "CREATE TABLE integration_outbox (entry_id INTEGER PRIMARY KEY, stage INTEGER NOT NULL);\n\
             CREATE TABLE integration_dispatch_binding_v2 (entry_id INTEGER PRIMARY KEY);\n\
             INSERT INTO integration_outbox(entry_id, stage) VALUES (1, 4);",
        )
        .expect("seed predecessor");
    }

    let mut reader = Connection::open(&path).expect("open reader");
    let mut writer = Connection::open(&path).expect("open writer");
    let read_tx = reader
        .transaction_with_behavior(TransactionBehavior::Deferred)
        .expect("begin read snapshot");
    let stage_before: i64 = read_tx
        .query_row(
            "SELECT stage FROM integration_outbox WHERE entry_id = 1",
            [],
            |row| row.get(0),
        )
        .expect("read predecessor stage");
    assert_eq!(stage_before, 4);

    let write_tx = writer
        .transaction_with_behavior(TransactionBehavior::Immediate)
        .expect("begin writer");
    write_tx
        .execute(
            "UPDATE integration_outbox SET stage = 5 WHERE entry_id = 1",
            [],
        )
        .expect("advance stage");
    write_tx
        .execute(
            "INSERT INTO integration_dispatch_binding_v2(entry_id) VALUES (1)",
            [],
        )
        .expect("insert binding");
    write_tx.commit().expect("commit successor");

    let stage_same_snapshot: i64 = read_tx
        .query_row(
            "SELECT stage FROM integration_outbox WHERE entry_id = 1",
            [],
            |row| row.get(0),
        )
        .expect("reread stage in snapshot");
    let binding_same_snapshot: bool = read_tx
        .query_row(
            "SELECT EXISTS(SELECT 1 FROM integration_dispatch_binding_v2 WHERE entry_id = 1)",
            [],
            |row| row.get(0),
        )
        .expect("read binding in snapshot");
    assert_eq!(stage_same_snapshot, 4);
    assert!(!binding_same_snapshot);
    read_tx.commit().expect("close read snapshot");

    let stage_after: i64 = reader
        .query_row(
            "SELECT stage FROM integration_outbox WHERE entry_id = 1",
            [],
            |row| row.get(0),
        )
        .expect("read committed successor stage");
    let binding_after: bool = reader
        .query_row(
            "SELECT EXISTS(SELECT 1 FROM integration_dispatch_binding_v2 WHERE entry_id = 1)",
            [],
            |row| row.get(0),
        )
        .expect("read committed successor binding");
    assert_eq!(stage_after, 5);
    assert!(binding_after);

    drop(reader);
    drop(writer);
    remove_sqlite_files(&path);
}
