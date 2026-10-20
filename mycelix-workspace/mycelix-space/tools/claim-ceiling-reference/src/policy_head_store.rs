// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reference durable storage for the monotonic policy lifecycle head.
//!
//! This module atomically serializes local writers and durably replaces a small,
//! integrity-checked state record. A local checksum detects corruption, not a coordinated
//! attacker rollback of both the file and its checksum. Callers must provide an
//! authenticated distribution context and independently authorize first-install bootstrap.
//! This is not a TUF implementation or an independent anti-rollback anchor.

use super::{
    policy_lifecycle::{
        PolicyLifecycleDispositionV1, PolicyLifecycleReceiptV1,
        PolicyLifecycleTrustedContextV1, InstalledPolicyHeadV1, evaluate_policy_release_bytes,
    },
    policy_profile::{policy_profile_schema_sha256, validate_policy_profile_bytes},
    sha256, validate_json_against_schema,
};
use fs2::FileExt;
use serde::{Deserialize, Serialize};
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
    process,
    sync::atomic::{AtomicU64, Ordering},
};

const STORE_SCHEMA: &[u8] = include_bytes!("../schemas/policy-head-store-v1.schema.json");
const STORE_RECORD_SCHEMA_VERSION: &str = "mycelix.policy-head-store.record.v1";
const MAX_STORE_RECORD_BYTES: usize = 64 * 1024;
const MAX_RELEASE_DESCRIPTOR_BYTES: usize = 64 * 1024;
const MAX_PROFILE_BYTES: usize = 1024 * 1024;
static TEMP_FILE_SEQUENCE: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootstrapModeV1 {
    /// Normal operation: an absent state file is an error, never an implicit first install.
    RequireExistingState,
    /// The caller asserts that a separate trusted provisioning decision authorized bootstrap.
    /// This enum value is only a declaration by the trusted caller; it is not cryptographic proof.
    ExplicitlyAuthorizedBootstrap,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolicyHeadStoreBodyV1 {
    schema_version: String,
    generation: u64,
    head: InstalledPolicyHeadV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolicyHeadStoreRecordV1 {
    schema_version: String,
    generation: u64,
    head: InstalledPolicyHeadV1,
    integrity_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CurrentState {
    generation: u64,
    head: Option<InstalledPolicyHeadV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyHeadPromotionV1 {
    pub schema_version: String,
    pub active_profile_id: String,
    pub lifecycle_receipt: PolicyLifecycleReceiptV1,
    pub generation_before: u64,
    pub generation_after: u64,
    pub state_changed: bool,
}

pub struct PolicyHeadStore {
    state_path: PathBuf,
    lock_path: PathBuf,
}

impl PolicyHeadStore {
    /// Open a store beneath an already-existing, trusted directory.
    ///
    /// The parent directory and lock-file namespace must not be writable by an
    /// untrusted principal. The constructor never creates or repairs trusted state.
    pub fn open(state_path: impl Into<PathBuf>) -> Result<Self, String> {
        let state_path = state_path.into();
        if state_path.file_name().is_none() {
            return Err("policy-head state path must name a file".to_owned());
        }
        let parent = parent_dir(&state_path);
        let metadata = fs::metadata(parent)
            .map_err(|error| format!("policy-head parent directory is unavailable: {error}"))?;
        if !metadata.is_dir() {
            return Err("policy-head parent path is not a directory".to_owned());
        }

        let mut lock_name = state_path.as_os_str().to_os_string();
        lock_name.push(".lock");
        Ok(Self {
            state_path,
            lock_path: PathBuf::from(lock_name),
        })
    }

    /// Validate the profile, acquire an OS-level exclusive lock, reload the authoritative
    /// state, evaluate against that state, and atomically advance the head only on acceptance.
    ///
    /// `expected_profile_id` must come from the trusted policy-id-to-profile mapping, not from
    /// the downloaded candidate. A missing head is accepted only when the caller explicitly
    /// asserts that bootstrap was independently authorized.
    pub fn promote_release(
        &self,
        release_bytes: &[u8],
        profile_bytes: &[u8],
        expected_profile_id: &str,
        trusted_context: &PolicyLifecycleTrustedContextV1,
        bootstrap_mode: BootstrapModeV1,
    ) -> Result<PolicyHeadPromotionV1, String> {
        if release_bytes.is_empty() || release_bytes.len() > MAX_RELEASE_DESCRIPTOR_BYTES {
            return Err(format!(
                "release descriptor must be between 1 and {MAX_RELEASE_DESCRIPTOR_BYTES} bytes"
            ));
        }
        if profile_bytes.is_empty() || profile_bytes.len() > MAX_PROFILE_BYTES {
            return Err(format!(
                "policy profile must be between 1 and {MAX_PROFILE_BYTES} bytes"
            ));
        }
        if expected_profile_id.trim().is_empty() {
            return Err("trusted expected profile ID must not be empty".to_owned());
        }

        // Check the actual compiled-in profile schema pin rather than trusting a caller's
        // declared schema digest. Profile structural validation is separate from policy fit.
        let actual_schema_sha256 = policy_profile_schema_sha256();
        if trusted_context.expected_policy_profile_schema_sha256 != actual_schema_sha256 {
            return Err(
                "trusted profile-schema pin does not match this binary's embedded schema".to_owned(),
            );
        }
        let profile = validate_policy_profile_bytes(profile_bytes)?;
        if profile.profile_id != expected_profile_id {
            return Err(format!(
                "profile identity mismatch: trusted mapping expects {expected_profile_id}, received {}",
                profile.profile_id
            ));
        }

        // Check again immediately before opening/locking. This catches a replaced state file
        // that is a symlink or non-regular file; the parent directory itself remains a trust
        // boundary and must not be writable by an attacker.
        reject_symlink_or_non_file_if_present(&self.lock_path, "policy-head lock file")?;
        let lock_file = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .open(&self.lock_path)
            .map_err(|error| format!("cannot open policy-head lock file: {error}"))?;
        FileExt::lock_exclusive(&lock_file)
            .map_err(|error| format!("cannot acquire exclusive policy-head lock: {error}"))?;

        let result = self.promote_while_locked(
            release_bytes,
            profile_bytes,
            profile.profile_id,
            trusted_context,
            bootstrap_mode,
        );
        // Dropping the lock descriptor releases the lock even if promotion returns an error.
        drop(lock_file);
        result
    }

    fn promote_while_locked(
        &self,
        release_bytes: &[u8],
        profile_bytes: &[u8],
        profile_id: String,
        trusted_context: &PolicyLifecycleTrustedContextV1,
        bootstrap_mode: BootstrapModeV1,
    ) -> Result<PolicyHeadPromotionV1, String> {
        let current = self.read_current_state()?;
        if current.head.is_none() && bootstrap_mode != BootstrapModeV1::ExplicitlyAuthorizedBootstrap {
            return Err(
                "policy-head state is absent; refusing implicit bootstrap or silent state reset"
                    .to_owned(),
            );
        }

        // Never trust a candidate-provided or caller-cached installed head. The head is
        // replaced with the current value read under the cross-process lock.
        let mut authoritative_context = trusted_context.clone();
        authoritative_context.installed_head = current.head.clone();

        let receipt = evaluate_policy_release_bytes(
            release_bytes,
            profile_bytes,
            &authoritative_context,
        )?;

        match receipt.disposition {
            PolicyLifecycleDispositionV1::Rejected => {
                if receipt.proposed_next_head.is_some() {
                    return Err(
                        "lifecycle invariant violated: rejected release proposed a next head"
                            .to_owned(),
                    );
                }
                Ok(PolicyHeadPromotionV1 {
                    schema_version: "mycelix.policy-head-promotion.v1".to_owned(),
                    active_profile_id: profile_id,
                    lifecycle_receipt: receipt,
                    generation_before: current.generation,
                    generation_after: current.generation,
                    state_changed: false,
                })
            }
            PolicyLifecycleDispositionV1::AlreadyCurrent => {
                if receipt.proposed_next_head != current.head {
                    return Err(
                        "lifecycle invariant violated: idempotent release disagrees with stored head"
                            .to_owned(),
                    );
                }
                Ok(PolicyHeadPromotionV1 {
                    schema_version: "mycelix.policy-head-promotion.v1".to_owned(),
                    active_profile_id: profile_id,
                    lifecycle_receipt: receipt,
                    generation_before: current.generation,
                    generation_after: current.generation,
                    state_changed: false,
                })
            }
            PolicyLifecycleDispositionV1::AcceptedNewRelease => {
                let new_head = receipt.proposed_next_head.clone().ok_or_else(|| {
                    "lifecycle invariant violated: accepted release has no proposed head".to_owned()
                })?;
                let next_generation = current
                    .generation
                    .checked_add(1)
                    .ok_or_else(|| "policy-head store generation overflow".to_owned())?;
                self.write_state_atomically(next_generation, &new_head)?;
                Ok(PolicyHeadPromotionV1 {
                    schema_version: "mycelix.policy-head-promotion.v1".to_owned(),
                    active_profile_id: profile_id,
                    lifecycle_receipt: receipt,
                    generation_before: current.generation,
                    generation_after: next_generation,
                    state_changed: true,
                })
            }
        }
    }

    fn read_current_state(&self) -> Result<CurrentState, String> {
        reject_symlink_or_non_file_if_present(&self.state_path, "policy-head state file")?;
        let file = match File::open(&self.state_path) {
            Ok(file) => file,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                return Ok(CurrentState {
                    generation: 0,
                    head: None,
                });
            }
            Err(error) => return Err(format!("cannot open policy-head state file: {error}")),
        };

        let mut bytes = Vec::new();
        file.take((MAX_STORE_RECORD_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(|error| format!("cannot read policy-head state file: {error}"))?;
        if bytes.is_empty() || bytes.len() > MAX_STORE_RECORD_BYTES {
            return Err(format!(
                "policy-head state file has invalid size (must be 1..={MAX_STORE_RECORD_BYTES} bytes)"
            ));
        }

        validate_json_against_schema(STORE_SCHEMA, &bytes, "policy-head state")?;
        let record: PolicyHeadStoreRecordV1 = serde_json::from_slice(&bytes)
            .map_err(|error| format!("policy-head state deserialization failed: {error}"))?;
        if record.schema_version != STORE_RECORD_SCHEMA_VERSION || record.generation == 0 {
            return Err("policy-head state has an unsupported schema or zero generation".to_owned());
        }
        validate_head(&record.head)?;

        let body = PolicyHeadStoreBodyV1 {
            schema_version: record.schema_version,
            generation: record.generation,
            head: record.head.clone(),
        };
        let body_bytes = serde_json::to_vec(&body)
            .map_err(|error| format!("cannot serialize policy-head integrity body: {error}"))?;
        let expected_integrity = sha256(&body_bytes);
        if record.integrity_sha256 != expected_integrity {
            return Err("policy-head state integrity digest mismatch; refusing to reset or repair".to_owned());
        }

        Ok(CurrentState {
            generation: record.generation,
            head: Some(record.head),
        })
    }

    fn write_state_atomically(
        &self,
        generation: u64,
        head: &InstalledPolicyHeadV1,
    ) -> Result<(), String> {
        validate_head(head)?;
        let body = PolicyHeadStoreBodyV1 {
            schema_version: STORE_RECORD_SCHEMA_VERSION.to_owned(),
            generation,
            head: head.clone(),
        };
        let body_bytes = serde_json::to_vec(&body)
            .map_err(|error| format!("cannot serialize policy-head state: {error}"))?;
        let record = PolicyHeadStoreRecordV1 {
            schema_version: body.schema_version.clone(),
            generation: body.generation,
            head: body.head.clone(),
            integrity_sha256: sha256(&body_bytes),
        };
        let bytes = serde_json::to_vec_pretty(&record)
            .map_err(|error| format!("cannot serialize policy-head record: {error}"))?;
        if bytes.len() > MAX_STORE_RECORD_BYTES {
            return Err("serialized policy-head state exceeds the hard size limit".to_owned());
        }
        validate_json_against_schema(STORE_SCHEMA, &bytes, "new policy-head state")?;

        let (mut temp_file, temp_path) = self.create_temporary_file()?;
        let write_result = temp_file
            .write_all(&bytes)
            .and_then(|()| temp_file.sync_all());
        drop(temp_file);
        if let Err(error) = write_result {
            let _ = fs::remove_file(&temp_path);
            return Err(format!("cannot write and sync temporary policy-head state: {error}"));
        }

        if let Err(error) = fs::rename(&temp_path, &self.state_path) {
            let _ = fs::remove_file(&temp_path);
            return Err(format!("cannot atomically replace policy-head state: {error}"));
        }

        // If this fails, the rename may already be visible but crash durability is uncertain.
        // Surface an indeterminate outcome; never report a clean commit without this sync.
        let parent = parent_dir(&self.state_path);
        let parent_dir_file = File::open(parent).map_err(|error| {
            format!(
                "policy-head commit outcome indeterminate after rename; cannot open parent directory for sync: {error}"
            )
        })?;
        parent_dir_file.sync_all().map_err(|error| {
            format!(
                "policy-head commit outcome indeterminate after rename; parent-directory sync failed: {error}"
            )
        })?;
        Ok(())
    }

    fn create_temporary_file(&self) -> Result<(File, PathBuf), String> {
        let parent = parent_dir(&self.state_path);
        let name = self
            .state_path
            .file_name()
            .expect("validated state path has a file name")
            .to_string_lossy();
        for _ in 0..100 {
            let sequence = TEMP_FILE_SEQUENCE.fetch_add(1, Ordering::Relaxed);
            let temp_path = parent.join(format!(
                ".{name}.tmp-{}-{sequence}",
                process::id()
            ));
            let mut options = OpenOptions::new();
            options.write(true).create_new(true);
            #[cfg(unix)]
            {
                use std::os::unix::fs::OpenOptionsExt;
                options.mode(0o600);
            }
            match options.open(&temp_path) {
                Ok(file) => return Ok((file, temp_path)),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => {
                    return Err(format!("cannot create temporary policy-head file: {error}"));
                }
            }
        }
        Err("cannot allocate a unique temporary policy-head file after 100 attempts".to_owned())
    }
}

fn parent_dir(path: &Path) -> &Path {
    path.parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."))
}

fn reject_symlink_or_non_file_if_present(path: &Path, label: &str) -> Result<(), String> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            Err(format!("{label} must not be a symlink"))
        }
        Ok(metadata) if !metadata.is_file() => Err(format!("{label} is not a regular file")),
        Ok(_) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(format!("cannot inspect {label}: {error}")),
    }
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_head(head: &InstalledPolicyHeadV1) -> Result<(), String> {
    if head.policy_id.trim().is_empty() || head.audience.trim().is_empty() {
        return Err("stored policy head requires non-empty policy ID and audience".to_owned());
    }
    if head.version == 0 || !valid_sha256(&head.target_profile_sha256) {
        return Err("stored policy head has an invalid version or profile digest".to_owned());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy_lifecycle::{
        AuthenticatedDistributionStatus, PolicyReleaseDescriptorV1,
    };
    use std::thread;

    const PROFILE_BYTES: &[u8] =
        include_bytes!("../fixtures/claim-ceiling-policy-profile-v1.json");
    const POLICY_ID: &str = "mycelix.policy.instrument.research";
    const AUDIENCE: &str = "mycelix.claim-ceiling.policy-check";

    fn test_context() -> PolicyLifecycleTrustedContextV1 {
        PolicyLifecycleTrustedContextV1 {
            expected_policy_id: POLICY_ID.to_owned(),
            expected_audience: AUDIENCE.to_owned(),
            expected_policy_profile_schema_sha256: policy_profile_schema_sha256(),
            minimum_revocation_epoch: 3,
            trusted_time_unix_seconds: Some(200),
            distribution_status: AuthenticatedDistributionStatus::VerifiedCurrent,
            // Deliberately not authoritative: the store must replace this value from disk.
            installed_head: None,
        }
    }

    fn release_bytes(version: u64) -> Vec<u8> {
        let descriptor = PolicyReleaseDescriptorV1 {
            schema_version: super::super::policy_lifecycle::RELEASE_DESCRIPTOR_SCHEMA_VERSION.to_owned(),
            policy_id: POLICY_ID.to_owned(),
            audience: AUDIENCE.to_owned(),
            version,
            revocation_epoch: 3,
            target_profile_sha256: sha256(PROFILE_BYTES),
            policy_profile_schema_sha256: policy_profile_schema_sha256(),
            issued_at_unix_seconds: 100,
            expires_at_unix_seconds: 300,
        };
        serde_json::to_vec(&descriptor).expect("serialize test release")
    }

    fn store_path(temp: &tempfile::TempDir) -> PathBuf {
        temp.path().join("policy-head.json")
    }

    fn store(temp: &tempfile::TempDir) -> PolicyHeadStore {
        PolicyHeadStore::open(store_path(temp)).expect("open policy-head store")
    }

    fn expected_profile_id() -> String {
        validate_policy_profile_bytes(PROFILE_BYTES)
            .expect("fixture profile is valid")
            .profile_id
    }

    fn promote(
        store: &PolicyHeadStore,
        version: u64,
        context: &PolicyLifecycleTrustedContextV1,
        bootstrap: BootstrapModeV1,
    ) -> Result<PolicyHeadPromotionV1, String> {
        store.promote_release(
            &release_bytes(version),
            PROFILE_BYTES,
            &expected_profile_id(),
            context,
            bootstrap,
        )
    }

    #[test]
    fn first_install_requires_explicit_bootstrap_and_advances_durable_generation() {
        let temp = tempfile::tempdir().unwrap();
        let store = store(&temp);
        let context = test_context();

        let implicit = promote(&store, 1, &context, BootstrapModeV1::RequireExistingState)
            .expect_err("missing state must not silently authorize bootstrap");
        assert!(implicit.contains("refusing implicit bootstrap"));
        assert!(!store_path(&temp).exists());

        let first = promote(
            &store,
            1,
            &context,
            BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
        )
        .expect("explicit bootstrap should install a valid authenticated fixture");
        assert_eq!(
            first.lifecycle_receipt.disposition,
            PolicyLifecycleDispositionV1::AcceptedNewRelease
        );
        assert_eq!((first.generation_before, first.generation_after), (0, 1));
        assert!(first.state_changed);
        assert!(!first.lifecycle_receipt.evidence_admission_granted);
        assert!(!first.lifecycle_receipt.action_authority_granted);

        // Reopen with a new store object to ensure the head comes from persisted bytes.
        let reopened = PolicyHeadStore::open(store_path(&temp)).unwrap();
        let replay = promote(
            &reopened,
            1,
            &context,
            BootstrapModeV1::RequireExistingState,
        )
        .expect("exact current release should be idempotent");
        assert_eq!(
            replay.lifecycle_receipt.disposition,
            PolicyLifecycleDispositionV1::AlreadyCurrent
        );
        assert_eq!((replay.generation_before, replay.generation_after), (1, 1));
        assert!(!replay.state_changed);
    }

    #[test]
    fn rejection_never_creates_or_advances_state() {
        let temp = tempfile::tempdir().unwrap();
        let store = store(&temp);
        let mut context = test_context();
        context.distribution_status = AuthenticatedDistributionStatus::InvalidSignature;

        let result = promote(
            &store,
            1,
            &context,
            BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
        )
        .expect("well-shaped invalid distribution state yields a rejection receipt");
        assert_eq!(
            result.lifecycle_receipt.disposition,
            PolicyLifecycleDispositionV1::Rejected
        );
        assert!(result.lifecycle_receipt.proposed_next_head.is_none());
        assert!(!result.state_changed);
        assert_eq!(result.generation_after, 0);
        assert!(!store_path(&temp).exists());
    }

    #[test]
    fn persisted_head_overrides_stale_caller_context_and_blocks_rollback() {
        let temp = tempfile::tempdir().unwrap();
        let store = store(&temp);
        let mut context = test_context();

        promote(
            &store,
            2,
            &context,
            BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
        )
        .expect("install version 2");

        // A cached context with no head cannot erase the persisted head. Version 1 is rejected.
        context.installed_head = None;
        let rollback = promote(
            &store,
            1,
            &context,
            BootstrapModeV1::RequireExistingState,
        )
        .expect("rollback attempt should yield a rejection receipt");
        assert_eq!(
            rollback.lifecycle_receipt.disposition,
            PolicyLifecycleDispositionV1::Rejected
        );
        assert!(rollback
            .lifecycle_receipt
            .violations
            .iter()
            .any(|violation| violation.code == "policy_version_rollback"));
        assert!(rollback.lifecycle_receipt.proposed_next_head.is_none());
        assert!(!rollback.state_changed);
        assert_eq!((rollback.generation_before, rollback.generation_after), (1, 1));

        // The exact installed release remains idempotent after the rejected rollback.
        let current = promote(
            &store,
            2,
            &context,
            BootstrapModeV1::RequireExistingState,
        )
        .expect("replay current release");
        assert_eq!(
            current.lifecycle_receipt.disposition,
            PolicyLifecycleDispositionV1::AlreadyCurrent
        );
        assert_eq!(current.generation_after, 1);
    }

    #[test]
    fn corrupt_state_fails_closed_without_automatic_repair() {
        let temp = tempfile::tempdir().unwrap();
        let store = store(&temp);
        let state_path = store_path(&temp);
        let corrupted = br#"{"schema_version":"wrong","generation":7}"#;
        fs::write(&state_path, corrupted).unwrap();

        let error = promote(
            &store,
            2,
            &test_context(),
            BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
        )
        .expect_err("corrupt existing state must not be mistaken for absent state");
        assert!(
            error.contains("policy-head state schema")
                || error.contains("additional properties")
                || error.contains("required"),
            "unexpected error: {error}"
        );
        assert_eq!(fs::read(state_path).unwrap(), corrupted);
    }

    #[test]
    fn trusted_profile_identity_and_schema_pin_are_mandatory() {
        let temp = tempfile::tempdir().unwrap();
        let store = store(&temp);
        let mut context = test_context();
        context.expected_policy_profile_schema_sha256 =
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".to_owned();

        assert!(store
            .promote_release(
                &release_bytes(1),
                PROFILE_BYTES,
                &expected_profile_id(),
                &context,
                BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
            )
            .unwrap_err()
            .contains("does not match this binary's embedded schema"));
        assert!(!store_path(&temp).exists());

        let error = store
            .promote_release(
                &release_bytes(1),
                PROFILE_BYTES,
                "mycelix.policy.wrong-profile",
                &test_context(),
                BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
            )
            .unwrap_err();
        assert!(error.contains("profile identity mismatch"));
        assert!(!store_path(&temp).exists());
    }

    #[test]
    fn concurrent_exact_bootstrap_is_serialized_and_idempotent() {
        let temp = tempfile::tempdir().unwrap();
        let path = store_path(&temp);
        let expected_profile_id = expected_profile_id();
        let context = test_context();
        let release = release_bytes(1);
        let profile = PROFILE_BYTES.to_vec();
        let store_a = PolicyHeadStore::open(path.clone()).unwrap();
        let store_b = PolicyHeadStore::open(path).unwrap();
        let id_a = expected_profile_id.clone();
        let id_b = expected_profile_id;
        let ctx_a = context.clone();
        let ctx_b = context;

        let a = thread::spawn(move || {
            store_a.promote_release(
                &release,
                &profile,
                &id_a,
                &ctx_a,
                BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
            )
        });
        let release_b = release_bytes(1);
        let profile_b = PROFILE_BYTES.to_vec();
        let b = thread::spawn(move || {
            store_b.promote_release(
                &release_b,
                &profile_b,
                &id_b,
                &ctx_b,
                BootstrapModeV1::ExplicitlyAuthorizedBootstrap,
            )
        });

        let outcomes = [a.join().unwrap().unwrap(), b.join().unwrap().unwrap()];
        let accepted = outcomes
            .iter()
            .filter(|outcome| {
                outcome.lifecycle_receipt.disposition
                    == PolicyLifecycleDispositionV1::AcceptedNewRelease
            })
            .count();
        let current = outcomes
            .iter()
            .filter(|outcome| {
                outcome.lifecycle_receipt.disposition
                    == PolicyLifecycleDispositionV1::AlreadyCurrent
            })
            .count();
        assert_eq!(accepted, 1);
        assert_eq!(current, 1);
        assert_eq!(outcomes.iter().filter(|o| o.state_changed).count(), 1);
    }
}
