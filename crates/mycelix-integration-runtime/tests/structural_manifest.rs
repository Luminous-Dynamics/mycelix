use mycelix_integration_runtime::{
    RuntimeError, SqliteIntegrationStore, RUNTIME_ENFORCEMENT_PROFILE_V5,
    RUNTIME_STRUCTURAL_MANIFEST_V2,
};
use rusqlite::Connection;

#[test]
fn valid_runtime_satisfies_structural_manifest_v2() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("structural-valid.sqlite");

    let _store = SqliteIntegrationStore::open(&path).expect("fresh runtime must satisfy manifest");
    let _reopened = SqliteIntegrationStore::open(&path).expect("hardened runtime must reopen");

    assert_eq!(
        RUNTIME_STRUCTURAL_MANIFEST_V2,
        "mycelix-integration-runtime/structural-manifest-v2"
    );
}

#[test]
fn user_version_two_with_wrong_named_index_is_rejected() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("wrong-index.sqlite");
    initialize(&path);

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute_batch(
            r#"
            DROP INDEX integration_outbox_claim_idx;
            CREATE INDEX integration_outbox_claim_idx
                ON integration_outbox(stage, lease_until_ms);
            "#,
        )
        .expect("wrong same-name index fixture must be installed");
        assert_schema_version_two(&conn);
    }

    expect_manifest_rejection(&path, "index integration_outbox_claim_idx column order");
}

#[test]
fn user_version_two_with_extra_unique_constraint_is_rejected() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("extra-unique.sqlite");
    initialize(&path);

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute_batch(
            "CREATE UNIQUE INDEX hostile_execution_identity_unique\n\
             ON integration_execution_observation(entry_id, attempt_id);",
        )
        .expect("extra UNIQUE fixture must be installed");
        assert_schema_version_two(&conn);
    }

    expect_manifest_rejection(
        &path,
        "integration_execution_observation UNIQUE/primary-key index set",
    );
}

#[test]
fn unknown_managed_trigger_rejects_before_enforcement_repair() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("unknown-trigger.sqlite");
    initialize(&path);

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute_batch(
            r#"
            CREATE TRIGGER hostile_outbox_trigger
            BEFORE UPDATE ON integration_outbox
            BEGIN
                SELECT 1;
            END;

            UPDATE integration_runtime_enforcement
            SET enforcement_profile = 'tampered-before-manifest'
            WHERE singleton = 1;
            "#,
        )
        .expect("hostile trigger fixture must be installed");
        assert_schema_version_two(&conn);
    }

    expect_manifest_rejection(&path, "unqualified trigger hostile_outbox_trigger");

    let conn = Connection::open(&path).expect("verification connection must open");
    let profile: String = conn
        .query_row(
            "SELECT enforcement_profile FROM integration_runtime_enforcement WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("enforcement profile must remain readable");
    assert_eq!(profile, "tampered-before-manifest");

    let hostile_trigger_count: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master\n\
             WHERE type = 'trigger' AND name = 'hostile_outbox_trigger'",
            [],
            |row| row.get(0),
        )
        .expect("trigger catalog must remain readable");
    assert_eq!(hostile_trigger_count, 1);
}

#[test]
fn missing_required_base_index_is_not_silently_repaired() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("missing-index.sqlite");
    initialize(&path);

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute_batch("DROP INDEX integration_reconciliation_history_idx;")
            .expect("required index must be removable for fixture");
        assert_schema_version_two(&conn);
    }

    expect_manifest_rejection(&path, "required index integration_reconciliation_history_idx");
}

fn initialize(path: &std::path::Path) {
    let _store = SqliteIntegrationStore::open(path).expect("baseline runtime must initialize");
    let conn = Connection::open(path).expect("baseline verification connection must open");
    let profile: String = conn
        .query_row(
            "SELECT enforcement_profile FROM integration_runtime_enforcement WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("enforcement profile must exist");
    assert_eq!(profile, RUNTIME_ENFORCEMENT_PROFILE_V5);
}

fn assert_schema_version_two(conn: &Connection) {
    let version: i64 = conn
        .query_row("PRAGMA user_version", [], |row| row.get(0))
        .expect("user_version must be readable");
    assert_eq!(version, 2);
}

fn expect_manifest_rejection(path: &std::path::Path, detail: &str) {
    match SqliteIntegrationStore::open(path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains(RUNTIME_STRUCTURAL_MANIFEST_V2));
            assert!(
                message.contains(detail),
                "manifest rejection did not contain expected detail: {message}"
            );
        }
        Err(other) => panic!("unexpected structural-manifest error: {other:?}"),
        Ok(_) => panic!("same-version structural substitution must fail closed"),
    }
}
