// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#[cfg(unix)]
mod unix_permissions {
    use mycelix_stewardship_runtime_store::{RuntimeStoreError, RuntimeStoreV1};
    use std::fs;
    use std::os::unix::fs::PermissionsExt;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_path(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock after unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "mycelix-stewardship-runtime-store-perms-{label}-{}-{nonce}.sqlite",
            std::process::id()
        ))
    }

    fn remove_sqlite_files(path: &Path) {
        let _ = fs::remove_file(path);
        let base = path.to_string_lossy();
        let _ = fs::remove_file(format!("{base}-wal"));
        let _ = fs::remove_file(format!("{base}-shm"));
    }

    #[test]
    fn created_database_has_no_group_or_other_permissions() {
        let path = temp_path("create-mode");
        let store = RuntimeStoreV1::create(&path).expect("create exact store");
        let mode = fs::metadata(&path).expect("metadata").permissions().mode();
        assert_eq!(mode & 0o077, 0);
        drop(store);
        remove_sqlite_files(&path);
    }

    #[test]
    fn weak_main_database_permissions_fail_closed() {
        let path = temp_path("weak-main-mode");
        let store = RuntimeStoreV1::create(&path).expect("create exact store");
        drop(store);

        let mut permissions = fs::metadata(&path).expect("metadata").permissions();
        permissions.set_mode(0o644);
        fs::set_permissions(&path, permissions).expect("weaken permissions");

        let error = RuntimeStoreV1::open(&path)
            .err()
            .expect("weak permissions must fail closed");
        assert!(matches!(
            error,
            RuntimeStoreError::ProfileMismatch("runtime file permissions")
        ));
        remove_sqlite_files(&path);
    }
}
