// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_stewardship_runtime_store::RuntimeStoreV1;
use rusqlite::Connection;
use std::env;
use std::fs;
use std::io::{Error, ErrorKind};
use std::path::{Path, PathBuf};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args_os().nth(1).map(PathBuf::from).ok_or_else(|| {
        Error::new(ErrorKind::InvalidInput, "usage: profile_probe <new-sqlite-path>")
    })?;

    let store = RuntimeStoreV1::create(&path)?;
    println!("PRODUCT_PROFILE_ID={}", store.profile_snapshot()?.profile_id);

    // Deliberately inspect through an independent raw SQLite connection rather
    // than asking RuntimeStoreV1::verify_profile() to grade itself.
    let connection = Connection::open(&path)?;
    let page_size: i64 = connection.query_row("PRAGMA page_size", [], |row| row.get(0))?;
    let auto_vacuum: i64 = connection.query_row("PRAGMA auto_vacuum", [], |row| row.get(0))?;
    let encoding: String = connection.query_row("PRAGMA encoding", [], |row| row.get(0))?;
    let journal_mode: String = connection.query_row("PRAGMA journal_mode", [], |row| row.get(0))?;
    let user_version: i64 = connection.query_row("PRAGMA user_version", [], |row| row.get(0))?;
    let sqlite_version: String = connection.query_row("SELECT sqlite_version()", [], |row| row.get(0))?;
    let sqlite_source_id: String =
        connection.query_row("SELECT sqlite_source_id()", [], |row| row.get(0))?;

    assert_eq!(page_size, 4096, "independent page-size oracle");
    assert_eq!(auto_vacuum, 0, "independent auto-vacuum oracle");
    assert_eq!(encoding.to_ascii_uppercase(), "UTF-8", "independent encoding oracle");
    assert_eq!(journal_mode.to_ascii_lowercase(), "wal", "independent journal oracle");
    assert_eq!(user_version, 1, "independent user-version oracle");

    println!("PAGE_SIZE={page_size}");
    println!("AUTO_VACUUM={auto_vacuum}");
    println!("ENCODING={encoding}");
    println!("JOURNAL_MODE={journal_mode}");
    println!("USER_VERSION={user_version}");
    println!("SQLITE_VERSION={sqlite_version}");
    println!("SQLITE_SOURCE_ID={sqlite_source_id}");

    let mut options = Vec::new();
    {
        let mut statement = connection.prepare("PRAGMA compile_options")?;
        let rows = statement.query_map([], |row| row.get::<_, String>(0))?;
        for row in rows {
            options.push(row?);
        }
    }
    options.sort();
    for option in options {
        println!("COMPILE_OPTION={option}");
    }

    let mut schema = Vec::new();
    {
        let mut statement = connection.prepare(
            "SELECT type, name, COALESCE(sql, '') FROM sqlite_schema ORDER BY type, name",
        )?;
        let rows = statement.query_map([], |row| {
            Ok((
                row.get::<_, String>(0)?,
                row.get::<_, String>(1)?,
                row.get::<_, String>(2)?,
            ))
        })?;
        for row in rows {
            schema.push(row?);
        }
    }
    for (kind, name, sql) in schema {
        println!("SCHEMA_OBJECT={kind}|{name}|{}", sql.replace('\n', "\\n"));
    }

    let mut integrity = Vec::new();
    {
        let mut statement = connection.prepare("PRAGMA integrity_check")?;
        let rows = statement.query_map([], |row| row.get::<_, String>(0))?;
        for row in rows {
            integrity.push(row?);
        }
    }
    assert!(
        integrity.len() == 1 && integrity[0] == "ok",
        "independent full integrity oracle: {integrity:?}"
    );
    println!("INTEGRITY_CHECK=ok");

    print_file_mode("DB", &path)?;
    print_optional_file_mode("WAL", &sidecar_path(&path, "-wal"))?;
    print_optional_file_mode("SHM", &sidecar_path(&path, "-shm"))?;

    drop(connection);
    drop(store);
    Ok(())
}

fn sidecar_path(path: &Path, suffix: &str) -> PathBuf {
    let mut value = path.as_os_str().to_os_string();
    value.push(suffix);
    PathBuf::from(value)
}

fn print_optional_file_mode(label: &str, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    match fs::symlink_metadata(path) {
        Ok(_) => print_file_mode(label, path),
        Err(error) if error.kind() == ErrorKind::NotFound => {
            println!("{label}_PRESENT=false");
            Ok(())
        }
        Err(error) => Err(error.into()),
    }
}

fn print_file_mode(label: &str, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let metadata = fs::symlink_metadata(path)?;
    println!("{label}_PRESENT=true");
    println!("{label}_IS_FILE={}", metadata.is_file());
    println!("{label}_IS_SYMLINK={}", metadata.file_type().is_symlink());

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        println!("{label}_MODE={:o}", metadata.permissions().mode() & 0o777);
    }

    Ok(())
}
