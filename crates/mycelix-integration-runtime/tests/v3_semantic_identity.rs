use mycelix_integration_runtime::{
    SqliteIntegrationStore, RUNTIME_SEMANTIC_PROFILE_V31,
};
use rusqlite::Connection;

#[test]
fn semantic_profile_is_durable_reopen_stable_and_fail_closed_on_tamper() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("semantic-profile.sqlite");

    {
        let _store = SqliteIntegrationStore::open(&path).expect("new v3.1 store must open");
    }

    {
        let conn = Connection::open(&path).expect("raw verification connection must open");
        let stored: String = conn
            .query_row(
                "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
                [],
                |row| row.get(0),
            )
            .expect("semantic producer identity must be durable");
        assert_eq!(stored, RUNTIME_SEMANTIC_PROFILE_V31);
    }

    let _reopened = SqliteIntegrationStore::open(&path)
        .expect("a store with the exact current semantic producer identity must reopen");
    drop(_reopened);

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute(
            "UPDATE integration_runtime_semantics SET semantic_profile = ?1 WHERE singleton = 1",
            ["mycelix-integration-runtime/unknown-semantic-profile"],
        )
        .expect("fixture tamper must succeed");
    }

    assert!(
        SqliteIntegrationStore::open(&path).is_err(),
        "unknown durable semantic producer identity must fail closed even when the SQLite schema is readable"
    );
}
