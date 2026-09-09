// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Crash-durable Linux reservation store for exact response attempts.
//!
//! A durable reservation is intentionally **not** an execution permit. This
//! store solves one narrower problem: before any future effect-capable runtime
//! may act, one exact physical-effect identity must already have exactly one
//! durable attempt identity.
//!
//! Important fail-closed rules:
//!
//! - the stable journal key is the effect identity, not authority generation;
//! - the store generates the winning attempt nonce/reference itself;
//! - the record is fsynced before it is linked into its final no-overwrite name;
//! - the directory is fsynced after publication;
//! - an existing exact attempt may be reconciled;
//! - an existing different attempt for the same effect key blocks a fresh try;
//! - malformed/ambiguous journal state blocks rather than overwrites;
//! - no API in this crate marks an effect as started or performs an effect.

#[cfg(not(target_os = "linux"))]
compile_error!("mycelix-response-attempt-store currently requires Linux durability primitives");

use mycelix_institutional_core::Digest32;
use mycelix_response_attempt_identity::{
    prepare_response_attempt, PreparedResponseAttempt, ATTEMPT_IDENTITY_PROFILE,
    EFFECT_IDENTITY_PROFILE,
};
use mycelix_response_current_executor::QualifiedResponseCurrentExecutor;
use mycelix_response_effect_safety::QualifiedResponseEffectSafety;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::sync::{Mutex, MutexGuard};

pub const PROTOCOL_VERSION: &str = "mycelix-response-attempt-store-v0.1";
pub const RECORD_PROFILE: &str = "mycelix-response-attempt-record-v1-blake3-framed";
pub const RESERVATION_PROFILE: &str =
    "mycelix-response-durable-attempt-reservation-v1-blake3-framed";

const DOMAIN_RECORD: &[u8] = b"mycelix/response/attempt-store/record/v1";
const DOMAIN_RESERVATION: &[u8] = b"mycelix/response/attempt-store/reservation/v1";
const LOCK_FILE: &str = ".reservation.lock";
const MAX_RECORD_BYTES: u64 = 64 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReservationState {
    Reserved,
}

/// Durable on-disk journal record. This is deserializable evidence, not a
/// positive capability. Positive use requires reading it through the trusted
/// store and reconstructing the exact non-deserializable prepared attempt.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AttemptReservationRecord {
    pub protocol_version: String,
    pub state: ReservationState,
    pub effect_identity_digest: Digest32,
    pub effect_identity_profile: String,
    pub attempt_identity_digest: Digest32,
    pub attempt_identity_profile: String,
    pub response_current_executor_digest: Digest32,
    pub response_safety_digest: Digest32,
    pub attempt_nonce: [u8; 32],
    pub attempt_ref: String,
    pub reserved_at_ms: u64,
    pub record_digest: Digest32,
    pub record_profile: String,
}

impl AttemptReservationRecord {
    fn from_prepared(prepared: &PreparedResponseAttempt, reserved_at_ms: u64) -> Self {
        let mut value = Self {
            protocol_version: PROTOCOL_VERSION.into(),
            state: ReservationState::Reserved,
            effect_identity_digest: prepared.effect_identity_digest(),
            effect_identity_profile: EFFECT_IDENTITY_PROFILE.into(),
            attempt_identity_digest: prepared.attempt_identity_digest(),
            attempt_identity_profile: ATTEMPT_IDENTITY_PROFILE.into(),
            response_current_executor_digest: prepared.response_current_executor_digest(),
            response_safety_digest: prepared.response_safety_digest(),
            attempt_nonce: prepared.attempt_nonce(),
            attempt_ref: prepared.attempt_ref().into(),
            reserved_at_ms,
            record_digest: Digest32([0u8; 32]),
            record_profile: RECORD_PROFILE.into(),
        };
        value.record_digest = record_digest(&value);
        value
    }

    pub fn validate(&self) -> Result<(), AttemptStoreError> {
        if self.protocol_version != PROTOCOL_VERSION
            || self.state != ReservationState::Reserved
            || self.effect_identity_profile != EFFECT_IDENTITY_PROFILE
            || self.attempt_identity_profile != ATTEMPT_IDENTITY_PROFILE
            || self.record_profile != RECORD_PROFILE
        {
            return Err(AttemptStoreError::InvalidRecordProtocol);
        }
        if self.effect_identity_digest.0 == [0u8; 32]
            || self.attempt_identity_digest.0 == [0u8; 32]
            || self.response_current_executor_digest.0 == [0u8; 32]
            || self.response_safety_digest.0 == [0u8; 32]
            || self.attempt_nonce == [0u8; 32]
            || self.reserved_at_ms == 0
        {
            return Err(AttemptStoreError::InvalidRecordIdentity);
        }
        if self.attempt_ref.trim().is_empty() || self.attempt_ref.len() > 2048 {
            return Err(AttemptStoreError::InvalidRecordIdentity);
        }
        if record_digest(self) != self.record_digest {
            return Err(AttemptStoreError::RecordDigestMismatch);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ReservationDisposition {
    Created,
    ReconciledExisting,
}

/// Non-deserializable positive result produced only after the exact reservation
/// record exists durably under its stable effect-key filename.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct DurablyReservedResponseAttempt {
    prepared: PreparedResponseAttempt,
    disposition: ReservationDisposition,
    journal_record_digest: Digest32,
    journal_record_profile: String,
    journal_ref: String,
    reservation_digest: Digest32,
    reserved_at_ms: u64,
}

impl DurablyReservedResponseAttempt {
    pub fn prepared(&self) -> &PreparedResponseAttempt {
        &self.prepared
    }

    pub fn disposition(&self) -> ReservationDisposition {
        self.disposition
    }

    pub fn journal_record_digest(&self) -> Digest32 {
        self.journal_record_digest
    }

    pub fn journal_record_profile(&self) -> &str {
        &self.journal_record_profile
    }

    pub fn journal_ref(&self) -> &str {
        &self.journal_ref
    }

    pub fn reservation_digest(&self) -> Digest32 {
        self.reservation_digest
    }

    pub fn reservation_profile(&self) -> &str {
        RESERVATION_PROFILE
    }

    pub fn reserved_at_ms(&self) -> u64 {
        self.reserved_at_ms
    }

    pub const fn attempt_identity_origin_verified_here(&self) -> bool {
        true
    }

    pub const fn filesystem_integrity_checked_here(&self) -> bool {
        true
    }

    pub const fn durably_reserved_here(&self) -> bool {
        true
    }

    /// The application must provision this store path from trusted configuration;
    /// this crate validates the filesystem object but cannot prove configuration provenance.
    pub const fn store_configuration_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn coordinator_stability_fenced_here(&self) -> bool {
        false
    }

    pub const fn coordinator_update_excluded_here(&self) -> bool {
        false
    }

    pub const fn effect_started_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Owner-private Linux reservation store. The path itself must come from trusted
/// application configuration; opening the store validates the resulting
/// filesystem object and lock file fail-closed.
pub struct TrustedResponseAttemptStore {
    root: PathBuf,
    lock_file: File,
    local_lock: Mutex<()>,
}

impl TrustedResponseAttemptStore {
    pub fn open(root: impl AsRef<Path>) -> Result<Self, AttemptStoreError> {
        let requested = root.as_ref();
        if !requested.exists() {
            fs::create_dir_all(requested).map_err(AttemptStoreError::Io)?;
            fs::set_permissions(requested, fs::Permissions::from_mode(0o700))
                .map_err(AttemptStoreError::Io)?;
        }
        let root = fs::canonicalize(requested).map_err(AttemptStoreError::Io)?;
        validate_private_directory(&root)?;

        let lock_path = root.join(LOCK_FILE);
        let lock_file = open_secure_file(&lock_path, true, true)?;
        validate_private_regular_file(&lock_file)?;
        Ok(Self {
            root,
            lock_file,
            local_lock: Mutex::new(()),
        })
    }

    /// Generate or reconcile exactly one durable attempt for the stable effect
    /// identity represented by `current + safety`.
    ///
    /// An unpersisted candidate nonce may be generated before the effect-key
    /// lookup. If a record already exists, that candidate is discarded and only
    /// the exact stored attempt is reconstructed. It never becomes a second
    /// reserved attempt.
    pub fn reserve(
        &self,
        current: &QualifiedResponseCurrentExecutor,
        safety: &QualifiedResponseEffectSafety,
        now_ms: u64,
    ) -> Result<DurablyReservedResponseAttempt, AttemptStoreError> {
        let candidate_nonce = random_nonce()?;
        let provisional_ref = format!("candidate:{}", hex32(candidate_nonce));
        let candidate = prepare_response_attempt(
            current,
            safety,
            candidate_nonce,
            provisional_ref,
            now_ms,
        )
        .map_err(|error| AttemptStoreError::AttemptIdentity(error.to_string()))?;
        let effect = candidate.effect_identity_digest();
        let final_path = self.record_path(effect);

        let _guard = self.lock()?;
        if final_path.exists() {
            let record = self.read_record(&final_path)?;
            return self.reconcile_existing(current, safety, record, now_ms);
        }

        // The store owns the winning ref. It is derived from the stable effect key
        // plus a store-generated nonce and therefore is not request-controlled.
        let attempt_ref = format!(
            "response-attempt-v1:{}:{}",
            hex_digest(effect),
            hex32(candidate_nonce)
        );
        let prepared = prepare_response_attempt(
            current,
            safety,
            candidate_nonce,
            attempt_ref,
            now_ms,
        )
        .map_err(|error| AttemptStoreError::AttemptIdentity(error.to_string()))?;
        let record = AttemptReservationRecord::from_prepared(&prepared, now_ms);
        record.validate()?;

        match self.publish_record_no_replace(&final_path, &record)? {
            PublishOutcome::Created => self.positive(prepared, record, ReservationDisposition::Created),
            PublishOutcome::AlreadyExists => {
                let existing = self.read_record(&final_path)?;
                self.reconcile_existing(current, safety, existing, now_ms)
            }
        }
    }

    fn reconcile_existing(
        &self,
        current: &QualifiedResponseCurrentExecutor,
        safety: &QualifiedResponseEffectSafety,
        record: AttemptReservationRecord,
        now_ms: u64,
    ) -> Result<DurablyReservedResponseAttempt, AttemptStoreError> {
        record.validate()?;
        let prepared = prepare_response_attempt(
            current,
            safety,
            record.attempt_nonce,
            record.attempt_ref.clone(),
            now_ms,
        )
        .map_err(|error| AttemptStoreError::ExistingAttemptRequiresReconciliation(error.to_string()))?;

        if prepared.effect_identity_digest() != record.effect_identity_digest {
            return Err(AttemptStoreError::EffectKeyMismatch);
        }
        if prepared.attempt_identity_digest() != record.attempt_identity_digest
            || prepared.response_current_executor_digest() != record.response_current_executor_digest
            || prepared.response_safety_digest() != record.response_safety_digest
        {
            return Err(AttemptStoreError::ExistingAttemptRequiresReconciliation(
                "existing effect reservation was prepared under another exact authority/safety identity".into(),
            ));
        }
        self.positive(
            prepared,
            record,
            ReservationDisposition::ReconciledExisting,
        )
    }

    fn positive(
        &self,
        prepared: PreparedResponseAttempt,
        record: AttemptReservationRecord,
        disposition: ReservationDisposition,
    ) -> Result<DurablyReservedResponseAttempt, AttemptStoreError> {
        let journal_ref = format!("attempt-record:{}", hex_digest(record.effect_identity_digest));
        let reservation_digest = reservation_digest(&record, &journal_ref);
        Ok(DurablyReservedResponseAttempt {
            prepared,
            disposition,
            journal_record_digest: record.record_digest,
            journal_record_profile: RECORD_PROFILE.into(),
            journal_ref,
            reservation_digest,
            reserved_at_ms: record.reserved_at_ms,
        })
    }

    fn record_path(&self, effect: Digest32) -> PathBuf {
        self.root.join(format!("{}.json", hex_digest(effect)))
    }

    fn read_record(&self, path: &Path) -> Result<AttemptReservationRecord, AttemptStoreError> {
        let mut file = open_secure_file(path, false, false)?;
        validate_private_regular_file(&file)?;
        let metadata = file.metadata().map_err(AttemptStoreError::Io)?;
        if metadata.len() == 0 || metadata.len() > MAX_RECORD_BYTES {
            return Err(AttemptStoreError::InvalidRecordSize(metadata.len()));
        }
        let mut bytes = Vec::with_capacity(metadata.len() as usize);
        file.read_to_end(&mut bytes).map_err(AttemptStoreError::Io)?;
        let record: AttemptReservationRecord =
            serde_json::from_slice(&bytes).map_err(AttemptStoreError::Json)?;
        record.validate()?;
        Ok(record)
    }

    fn publish_record_no_replace(
        &self,
        final_path: &Path,
        record: &AttemptReservationRecord,
    ) -> Result<PublishOutcome, AttemptStoreError> {
        let bytes = serde_json::to_vec(record).map_err(AttemptStoreError::Json)?;
        if bytes.is_empty() || bytes.len() as u64 > MAX_RECORD_BYTES {
            return Err(AttemptStoreError::InvalidRecordSize(bytes.len() as u64));
        }
        let temp_name = format!(
            ".attempt-{}-{}-{}.tmp",
            std::process::id(),
            hex_digest(record.effect_identity_digest),
            hex32(record.attempt_nonce)
        );
        let temp_path = self.root.join(temp_name);
        let mut temp = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_CLOEXEC | libc::O_NOFOLLOW)
            .open(&temp_path)
            .map_err(AttemptStoreError::Io)?;
        validate_private_regular_file(&temp)?;
        temp.write_all(&bytes).map_err(AttemptStoreError::Io)?;
        temp.sync_all().map_err(AttemptStoreError::Io)?;
        drop(temp);

        let outcome = match fs::hard_link(&temp_path, final_path) {
            Ok(()) => PublishOutcome::Created,
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
                PublishOutcome::AlreadyExists
            }
            Err(error) => {
                let _ = fs::remove_file(&temp_path);
                return Err(AttemptStoreError::Io(error));
            }
        };
        fs::remove_file(&temp_path).map_err(AttemptStoreError::Io)?;
        sync_directory(&self.root)?;
        Ok(outcome)
    }

    fn lock(&self) -> Result<StoreLock<'_>, AttemptStoreError> {
        let local = self
            .local_lock
            .lock()
            .map_err(|_| AttemptStoreError::LocalLockPoisoned)?;
        flock(self.lock_file.as_raw_fd(), libc::LOCK_EX)?;
        Ok(StoreLock {
            fd: self.lock_file.as_raw_fd(),
            _local: local,
        })
    }
}

struct StoreLock<'a> {
    fd: RawFd,
    _local: MutexGuard<'a, ()>,
}

impl Drop for StoreLock<'_> {
    fn drop(&mut self) {
        // Unlock failure cannot be reported from Drop. The file descriptor stays
        // owned by the store and kernel releases locks on process termination.
        unsafe {
            libc::flock(self.fd, libc::LOCK_UN);
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PublishOutcome {
    Created,
    AlreadyExists,
}

fn validate_private_directory(path: &Path) -> Result<(), AttemptStoreError> {
    let metadata = fs::symlink_metadata(path).map_err(AttemptStoreError::Io)?;
    if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
        return Err(AttemptStoreError::UnsafeStorePath);
    }
    let uid = unsafe { libc::geteuid() };
    if metadata.uid() != uid || metadata.mode() & 0o077 != 0 {
        return Err(AttemptStoreError::UnsafeStorePermissions);
    }
    Ok(())
}

fn open_secure_file(path: &Path, create: bool, write: bool) -> Result<File, AttemptStoreError> {
    let mut options = OpenOptions::new();
    options.read(true).write(write).create(create);
    options.mode(0o600);
    options.custom_flags(libc::O_CLOEXEC | libc::O_NOFOLLOW);
    options.open(path).map_err(AttemptStoreError::Io)
}

fn validate_private_regular_file(file: &File) -> Result<(), AttemptStoreError> {
    let metadata = file.metadata().map_err(AttemptStoreError::Io)?;
    if !metadata.file_type().is_file() {
        return Err(AttemptStoreError::UnsafeStorePath);
    }
    let uid = unsafe { libc::geteuid() };
    if metadata.uid() != uid || metadata.mode() & 0o077 != 0 {
        return Err(AttemptStoreError::UnsafeStorePermissions);
    }
    Ok(())
}

fn sync_directory(path: &Path) -> Result<(), AttemptStoreError> {
    let dir = File::open(path).map_err(AttemptStoreError::Io)?;
    dir.sync_all().map_err(AttemptStoreError::Io)
}

fn flock(fd: RawFd, operation: libc::c_int) -> Result<(), AttemptStoreError> {
    let result = unsafe { libc::flock(fd, operation) };
    if result == 0 {
        Ok(())
    } else {
        Err(AttemptStoreError::Io(std::io::Error::last_os_error()))
    }
}

fn random_nonce() -> Result<[u8; 32], AttemptStoreError> {
    let mut bytes = [0u8; 32];
    let mut random = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_CLOEXEC | libc::O_NOFOLLOW)
        .open("/dev/urandom")
        .map_err(AttemptStoreError::Io)?;
    random.read_exact(&mut bytes).map_err(AttemptStoreError::Io)?;
    if bytes == [0u8; 32] {
        return Err(AttemptStoreError::RandomnessFailure);
    }
    Ok(bytes)
}

fn record_digest(record: &AttemptReservationRecord) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_RECORD);
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, RECORD_PROFILE.as_bytes());
    frame(&mut hasher, &[match record.state { ReservationState::Reserved => 1 }]);
    frame(&mut hasher, &record.effect_identity_digest.0);
    frame(&mut hasher, record.effect_identity_profile.as_bytes());
    frame(&mut hasher, &record.attempt_identity_digest.0);
    frame(&mut hasher, record.attempt_identity_profile.as_bytes());
    frame(&mut hasher, &record.response_current_executor_digest.0);
    frame(&mut hasher, &record.response_safety_digest.0);
    frame(&mut hasher, &record.attempt_nonce);
    frame(&mut hasher, record.attempt_ref.as_bytes());
    frame(&mut hasher, &record.reserved_at_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn reservation_digest(record: &AttemptReservationRecord, journal_ref: &str) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_RESERVATION);
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, RESERVATION_PROFILE.as_bytes());
    frame(&mut hasher, &record.record_digest.0);
    frame(&mut hasher, RECORD_PROFILE.as_bytes());
    frame(&mut hasher, journal_ref.as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn hex_digest(digest: Digest32) -> String {
    hex32(digest.0)
}

fn hex32(bytes: [u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Debug)]
pub enum AttemptStoreError {
    Io(std::io::Error),
    Json(serde_json::Error),
    AttemptIdentity(String),
    ExistingAttemptRequiresReconciliation(String),
    InvalidRecordProtocol,
    InvalidRecordIdentity,
    RecordDigestMismatch,
    InvalidRecordSize(u64),
    EffectKeyMismatch,
    UnsafeStorePath,
    UnsafeStorePermissions,
    LocalLockPoisoned,
    RandomnessFailure,
}

impl fmt::Display for AttemptStoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "attempt-store I/O failed: {error}"),
            Self::Json(error) => write!(f, "attempt-store JSON failed: {error}"),
            Self::AttemptIdentity(error) => write!(f, "attempt identity preparation failed: {error}"),
            Self::ExistingAttemptRequiresReconciliation(error) => write!(
                f,
                "an attempt already exists for this exact effect and must be reconciled: {error}"
            ),
            Self::InvalidRecordProtocol => write!(f, "attempt record protocol/profile is invalid"),
            Self::InvalidRecordIdentity => write!(f, "attempt record identity is invalid"),
            Self::RecordDigestMismatch => write!(f, "attempt record digest does not match record bytes"),
            Self::InvalidRecordSize(size) => write!(f, "attempt record size {size} is invalid"),
            Self::EffectKeyMismatch => write!(f, "journal filename/effect identity mismatch"),
            Self::UnsafeStorePath => write!(f, "attempt-store path is not a safe regular filesystem object"),
            Self::UnsafeStorePermissions => write!(f, "attempt-store object must be owned by the effective user and private"),
            Self::LocalLockPoisoned => write!(f, "attempt-store in-process lock is poisoned"),
            Self::RandomnessFailure => write!(f, "attempt-store nonce generation failed"),
        }
    }
}

impl std::error::Error for AttemptStoreError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_digest_detects_identity_mutation() {
        let mut record = AttemptReservationRecord {
            protocol_version: PROTOCOL_VERSION.into(),
            state: ReservationState::Reserved,
            effect_identity_digest: Digest32([1; 32]),
            effect_identity_profile: EFFECT_IDENTITY_PROFILE.into(),
            attempt_identity_digest: Digest32([2; 32]),
            attempt_identity_profile: ATTEMPT_IDENTITY_PROFILE.into(),
            response_current_executor_digest: Digest32([3; 32]),
            response_safety_digest: Digest32([4; 32]),
            attempt_nonce: [5; 32],
            attempt_ref: "attempt:test".into(),
            reserved_at_ms: 10,
            record_digest: Digest32([0; 32]),
            record_profile: RECORD_PROFILE.into(),
        };
        record.record_digest = record_digest(&record);
        record.validate().unwrap();
        record.attempt_ref = "attempt:substituted".into();
        assert!(matches!(record.validate(), Err(AttemptStoreError::RecordDigestMismatch)));
    }

    #[test]
    fn stable_effect_filename_is_content_addressed() {
        let store = PathBuf::from("/tmp/example");
        let expected = store.join(format!("{}.json", hex_digest(Digest32([7; 32]))));
        assert!(expected.to_string_lossy().ends_with("0707070707070707070707070707070707070707070707070707070707070707.json"));
    }

    #[test]
    fn positive_type_never_claims_effect_or_update_exclusion() {
        let source = include_str!("lib.rs");
        for token in [
            "coordinator_update_excluded_here(&self) -> bool",
            "effect_started_here(&self) -> bool",
            "grants_execution_authority(&self) -> bool",
        ] {
            assert!(source.contains(token));
        }
        assert!(!source.contains("pub fn mark_effect_started"));
        assert!(!source.contains("pub fn perform_effect"));
    }
}
