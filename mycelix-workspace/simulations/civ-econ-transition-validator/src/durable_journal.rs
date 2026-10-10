//! Append-only local effect journal for CIV-ECON-001A recovery.
//!
//! This persists an intent before external dispatch. After restart, a surviving
//! Pending record requires reconciliation; it is never permission to replay.
//! It is not an exactly-once distributed transaction protocol.

use std::collections::HashMap;
use std::error::Error;
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JournalError {
    InvalidPath,
    InvalidEffectId,
    InvalidDigest,
    LockHeld,
    Poisoned,
    UnknownEffect,
    RequestConflict,
    ReceiptConflict,
    CorruptJournal { line: usize, reason: &'static str },
    Io { operation: &'static str, kind: io::ErrorKind },
}

impl fmt::Display for JournalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPath => write!(f, "journal path must name a regular file in an existing directory"),
            Self::InvalidEffectId => write!(f, "effect identity is empty or non-canonical"),
            Self::InvalidDigest => write!(f, "digest must be canonical sha256 lowercase hex"),
            Self::LockHeld => write!(f, "journal is already open or has a stale lock; inspect before recovery"),
            Self::Poisoned => write!(f, "journal had a write/sync failure; reopen and reconcile before further use"),
            Self::UnknownEffect => write!(f, "effect identity has no durable begin record"),
            Self::RequestConflict => write!(f, "effect identity was reused with a different request digest"),
            Self::ReceiptConflict => write!(f, "effect identity was acknowledged with a different receipt digest"),
            Self::CorruptJournal { line, reason } => write!(f, "corrupt effect journal at line {line}: {reason}"),
            Self::Io { operation, kind } => write!(f, "journal I/O failed during {operation}: {kind}"),
        }
    }
}
impl Error for JournalError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EffectStatus {
    Pending,
    Indeterminate,
    Acknowledged { receipt_digest: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BeginResult {
    /// Newly appended and synced; only this result permits dispatch attempt.
    Started,
    /// A begin record survived. Do not automatically dispatch again.
    PendingNeedsReconciliation,
    IndeterminateNeedsReconciliation,
    AlreadyAcknowledged { receipt_digest: String },
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AcknowledgeResult { Acknowledged, AlreadyAcknowledged }
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IndeterminateResult {
    Marked,
    AlreadyIndeterminate,
    AlreadyAcknowledged { receipt_digest: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct JournalEntry {
    request_digest: String,
    status: EffectStatus,
}

struct LockGuard { path: PathBuf }
impl Drop for LockGuard {
    fn drop(&mut self) {
        // A failed removal intentionally leaves a fail-closed stale lock.
        let _ = fs::remove_file(&self.path);
    }
}

pub struct DurableEffectJournal {
    path: PathBuf,
    file: File,
    _lock: LockGuard,
    entries: HashMap<String, JournalEntry>,
    poisoned: bool,
}

impl DurableEffectJournal {
    /// Open a single-writer journal. Parent directory must exist.
    /// A lock left by a crash is not broken automatically.
    pub fn open<P: AsRef<Path>>(requested: P) -> Result<Self, JournalError> {
        let requested = requested.as_ref();
        let name = requested.file_name().ok_or(JournalError::InvalidPath)?.to_os_string();
        if name.to_string_lossy().trim().is_empty() {
            return Err(JournalError::InvalidPath);
        }
        let requested_parent = requested.parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        let parent = fs::canonicalize(requested_parent)
            .map_err(|err| io_error("canonicalize journal directory", err))?;
        let path = parent.join(&name);

        match fs::symlink_metadata(&path) {
            Ok(meta) if meta.file_type().is_symlink() || !meta.is_file() => return Err(JournalError::InvalidPath),
            Ok(_) => {}
            Err(err) if err.kind() == io::ErrorKind::NotFound => {}
            Err(err) => return Err(io_error("inspect journal path", err)),
        }

        let lock_path = parent.join(format!("{}.lock", name.to_string_lossy()));
        let mut lock_file = OpenOptions::new().write(true).create_new(true).open(&lock_path)
            .map_err(|err| if err.kind() == io::ErrorKind::AlreadyExists {
                JournalError::LockHeld
            } else {
                io_error("create exclusive lock", err)
            })?;
        let lock = LockGuard { path: lock_path };
        lock_file.write_all(format!("civ-effect-journal-lock-v1\npid={}\n", std::process::id()).as_bytes())
            .map_err(|err| io_error("write lock marker", err))?;
        lock_file.sync_all().map_err(|err| io_error("sync lock marker", err))?;
        sync_directory(&parent)?;

        let mut file = OpenOptions::new().read(true).append(true).create(true).open(&path)
            .map_err(|err| io_error("open journal", err))?;
        file.sync_all().map_err(|err| io_error("sync journal on open", err))?;
        sync_directory(&parent)?;
        file.seek(SeekFrom::Start(0)).map_err(|err| io_error("rewind journal", err))?;
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).map_err(|err| io_error("read journal", err))?;
        file.seek(SeekFrom::End(0)).map_err(|err| io_error("seek journal end", err))?;
        let entries = replay(&bytes)?;

        Ok(Self { path, file, _lock: lock, entries, poisoned: false })
    }

    pub fn path(&self) -> &Path { &self.path }
    pub fn status(&self, id: &str) -> Option<EffectStatus> {
        self.entries.get(id).map(|entry| entry.status.clone())
    }

    /// Sorted IDs whose outcomes remain unresolved.
    pub fn unresolved_effect_ids(&self) -> Vec<String> {
        let mut ids: Vec<String> = self.entries.iter().filter_map(|(id, entry)| {
            match &entry.status {
                EffectStatus::Pending | EffectStatus::Indeterminate => Some(id.clone()),
                EffectStatus::Acknowledged { .. } => None,
            }
        }).collect();
        ids.sort();
        ids
    }

    /// Persist and sync a request before dispatch.
    /// Only Started permits a first dispatch. Every other result forbids an
    /// automatic duplicate dispatch and requires reconciliation or reuse of
    /// the already acknowledged receipt.
    pub fn begin_effect(&mut self, id: &str, request_digest: &str) -> Result<BeginResult, JournalError> {
        self.ensure_healthy()?;
        validate_effect_id(id)?;
        validate_digest(request_digest)?;
        if let Some(existing) = self.entries.get(id) {
            if existing.request_digest != request_digest { return Err(JournalError::RequestConflict); }
            return Ok(match &existing.status {
                EffectStatus::Pending => BeginResult::PendingNeedsReconciliation,
                EffectStatus::Indeterminate => BeginResult::IndeterminateNeedsReconciliation,
                EffectStatus::Acknowledged { receipt_digest } => BeginResult::AlreadyAcknowledged {
                    receipt_digest: receipt_digest.clone()
                },
            });
        }
        let record = format!("J1\tB\t{}\t{}\n", encode_hex(id.as_bytes()), request_digest);
        self.append_and_sync(&record)?;
        self.entries.insert(id.to_owned(), JournalEntry {
            request_digest: request_digest.to_owned(),
            status: EffectStatus::Pending
        });
        Ok(BeginResult::Started)
    }

    /// Persist a source-bound receipt after the outcome has been established
    /// under the exact external rail / authority profile.
    pub fn acknowledge_effect(
        &mut self, id: &str, request_digest: &str, receipt_digest: &str
    ) -> Result<AcknowledgeResult, JournalError> {
        self.ensure_healthy()?;
        validate_effect_id(id)?;
        validate_digest(request_digest)?;
        validate_digest(receipt_digest)?;
        let existing = self.entries.get(id).ok_or(JournalError::UnknownEffect)?;
        if existing.request_digest != request_digest { return Err(JournalError::RequestConflict); }
        if let EffectStatus::Acknowledged { receipt_digest: old } = &existing.status {
            return if old == receipt_digest { Ok(AcknowledgeResult::AlreadyAcknowledged) }
                else { Err(JournalError::ReceiptConflict) };
        }
        let record = format!("J1\tA\t{}\t{}\t{}\n", encode_hex(id.as_bytes()), request_digest, receipt_digest);
        self.append_and_sync(&record)?;
        if let Some(entry) = self.entries.get_mut(id) {
            entry.status = EffectStatus::Acknowledged { receipt_digest: receipt_digest.to_owned() };
        }
        Ok(AcknowledgeResult::Acknowledged)
    }

    /// Record uncertainty; explicit reconciliation may later acknowledge it.
    pub fn mark_indeterminate(
        &mut self, id: &str, request_digest: &str
    ) -> Result<IndeterminateResult, JournalError> {
        self.ensure_healthy()?;
        validate_effect_id(id)?;
        validate_digest(request_digest)?;
        let existing = self.entries.get(id).ok_or(JournalError::UnknownEffect)?;
        if existing.request_digest != request_digest { return Err(JournalError::RequestConflict); }
        match &existing.status {
            EffectStatus::Acknowledged { receipt_digest } =>
                return Ok(IndeterminateResult::AlreadyAcknowledged { receipt_digest: receipt_digest.clone() }),
            EffectStatus::Indeterminate => return Ok(IndeterminateResult::AlreadyIndeterminate),
            EffectStatus::Pending => {}
        }
        let record = format!("J1\tI\t{}\t{}\n", encode_hex(id.as_bytes()), request_digest);
        self.append_and_sync(&record)?;
        if let Some(entry) = self.entries.get_mut(id) { entry.status = EffectStatus::Indeterminate; }
        Ok(IndeterminateResult::Marked)
    }

    fn ensure_healthy(&self) -> Result<(), JournalError> {
        if self.poisoned { Err(JournalError::Poisoned) } else { Ok(()) }
    }

    fn append_and_sync(&mut self, record: &str) -> Result<(), JournalError> {
        if let Err(err) = self.file.write_all(record.as_bytes()).and_then(|()| self.file.sync_all()) {
            // The write may have reached disk despite an error. Do not append
            // again until a full reopen/replay resolves what persisted.
            self.poisoned = true;
            return Err(io_error("append and sync event", err));
        }
        Ok(())
    }
}

fn io_error(operation: &'static str, err: io::Error) -> JournalError {
    JournalError::Io { operation, kind: err.kind() }
}
fn sync_directory(path: &Path) -> Result<(), JournalError> {
    File::open(path).and_then(|dir| dir.sync_all())
        .map_err(|err| io_error("sync parent directory", err))
}
fn validate_effect_id(id: &str) -> Result<(), JournalError> {
    if id.is_empty() || id.trim() != id || id.bytes().any(|b| b == b'\n' || b == b'\r' || b == b'\0') {
        return Err(JournalError::InvalidEffectId);
    }
    Ok(())
}
fn validate_digest(digest: &str) -> Result<(), JournalError> {
    let Some(hex) = digest.strip_prefix("sha256:") else { return Err(JournalError::InvalidDigest); };
    if hex.len() != 64 || !hex.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)) {
        return Err(JournalError::InvalidDigest);
    }
    Ok(())
}
fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}
fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}
fn decode_hex(value: &str, line: usize) -> Result<Vec<u8>, JournalError> {
    if value.is_empty() || value.len() % 2 != 0
        || !value.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(JournalError::CorruptJournal { line, reason: "effect id is not canonical lowercase hex" });
    }
    let mut out = Vec::with_capacity(value.len() / 2);
    for pair in value.as_bytes().chunks_exact(2) {
        let high = hex_nibble(pair[0]).ok_or(JournalError::CorruptJournal { line, reason: "invalid hex" })?;
        let low = hex_nibble(pair[1]).ok_or(JournalError::CorruptJournal { line, reason: "invalid hex" })?;
        out.push((high << 4) | low);
    }
    Ok(out)
}

fn replay(bytes: &[u8]) -> Result<HashMap<String, JournalEntry>, JournalError> {
    if bytes.is_empty() { return Ok(HashMap::new()); }
    if !bytes.ends_with(b"\n") {
        return Err(JournalError::CorruptJournal {
            line: bytes.iter().filter(|b| **b == b'\n').count() + 1,
            reason: "truncated record; never repair by silently discarding bytes"
        });
    }
    let text = std::str::from_utf8(bytes).map_err(|_| JournalError::CorruptJournal {
        line: 1, reason: "journal is not valid UTF-8"
    })?;
    let mut entries: HashMap<String, JournalEntry> = HashMap::new();
    for (index, line) in text.split_terminator('\n').enumerate() {
        let line_no = index + 1;
        let fields: Vec<&str> = line.split('\t').collect();
        let (event, encoded_id, request_digest) = match fields.as_slice() {
            [version, event, id, request] if *version == "J1" && (*event == "B" || *event == "I") =>
                (*event, *id, *request),
            [version, event, id, request, _receipt] if *version == "J1" && *event == "A" =>
                (*event, *id, *request),
            [version, ..] if *version != "J1" =>
                return Err(JournalError::CorruptJournal { line: line_no, reason: "unsupported journal record version" }),
            _ => return Err(JournalError::CorruptJournal { line: line_no, reason: "invalid record shape or version" }),
        };
        let id_bytes = decode_hex(encoded_id, line_no)?;
        let id = String::from_utf8(id_bytes).map_err(|_| JournalError::CorruptJournal {
            line: line_no, reason: "effect id is not UTF-8"
        })?;
        validate_effect_id(&id).map_err(|_| JournalError::CorruptJournal {
            line: line_no, reason: "effect id is empty or non-canonical"
        })?;
        validate_digest(request_digest).map_err(|_| JournalError::CorruptJournal {
            line: line_no, reason: "request digest is invalid"
        })?;
        match event {
            "B" => {
                if entries.contains_key(&id) {
                    return Err(JournalError::CorruptJournal { line: line_no, reason: "duplicate begin event" });
                }
                entries.insert(id, JournalEntry { request_digest: request_digest.to_owned(), status: EffectStatus::Pending });
            }
            "I" => {
                let entry = entries.get_mut(&id).ok_or(JournalError::CorruptJournal {
                    line: line_no, reason: "indeterminate event precedes begin"
                })?;
                if entry.request_digest != request_digest || entry.status != EffectStatus::Pending {
                    return Err(JournalError::CorruptJournal { line: line_no, reason: "invalid indeterminate transition" });
                }
                entry.status = EffectStatus::Indeterminate;
            }
            "A" => {
                let receipt = fields[4];
                validate_digest(receipt).map_err(|_| JournalError::CorruptJournal {
                    line: line_no, reason: "receipt digest is invalid"
                })?;
                let entry = entries.get_mut(&id).ok_or(JournalError::CorruptJournal {
                    line: line_no, reason: "acknowledgement precedes begin"
                })?;
                if entry.request_digest != request_digest
                    || !matches!(&entry.status, EffectStatus::Pending | EffectStatus::Indeterminate)
                {
                    return Err(JournalError::CorruptJournal { line: line_no, reason: "invalid acknowledgement transition" });
                }
                entry.status = EffectStatus::Acknowledged { receipt_digest: receipt.to_owned() };
            }
            _ => unreachable!("record shape validates event discriminator"),
        }
    }
    Ok(entries)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::{SystemTime, UNIX_EPOCH};

    static NEXT_TEMP: AtomicU64 = AtomicU64::new(0);
    struct TempDir(PathBuf);
    impl TempDir {
        fn new() -> Self {
            let nonce = SystemTime::now().duration_since(UNIX_EPOCH)
                .expect("clock should be after epoch").as_nanos();
            let path = std::env::temp_dir().join(format!(
                "mycelix-civ-econ-journal-{}-{}-{}",
                std::process::id(), nonce, NEXT_TEMP.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&path).expect("create isolated test dir");
            Self(path)
        }
        fn journal_path(&self) -> PathBuf { self.0.join("effects.journal") }
    }
    impl Drop for TempDir {
        fn drop(&mut self) { let _ = fs::remove_dir_all(&self.0); }
    }

    const REQUEST_A: &str = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const REQUEST_B: &str = "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const RECEIPT_A: &str = "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
    const RECEIPT_B: &str = "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";

    #[test]
    fn pending_intent_survives_restart_and_is_not_replayed_automatically() {
        let temp = TempDir::new();
        {
            let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
            assert_eq!(journal.begin_effect("payment-1", REQUEST_A), Ok(BeginResult::Started));
            assert_eq!(journal.status("payment-1"), Some(EffectStatus::Pending));
        }
        let mut recovered = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(recovered.begin_effect("payment-1", REQUEST_A), Ok(BeginResult::PendingNeedsReconciliation));
        assert_eq!(recovered.unresolved_effect_ids(), vec!["payment-1".to_owned()]);
    }

    #[test]
    fn acknowledgement_survives_restart_and_begin_is_idempotent() {
        let temp = TempDir::new();
        {
            let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
            journal.begin_effect("payment-2", REQUEST_A).unwrap();
            assert_eq!(journal.acknowledge_effect("payment-2", REQUEST_A, RECEIPT_A), Ok(AcknowledgeResult::Acknowledged));
        }
        let mut recovered = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(recovered.begin_effect("payment-2", REQUEST_A),
            Ok(BeginResult::AlreadyAcknowledged { receipt_digest: RECEIPT_A.to_owned() }));
        assert!(recovered.unresolved_effect_ids().is_empty());
    }

    #[test]
    fn uncertain_effect_can_be_reconciled_to_acknowledged() {
        let temp = TempDir::new();
        {
            let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
            journal.begin_effect("payment-3", REQUEST_A).unwrap();
            assert_eq!(journal.mark_indeterminate("payment-3", REQUEST_A), Ok(IndeterminateResult::Marked));
        }
        let mut recovered = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(recovered.status("payment-3"), Some(EffectStatus::Indeterminate));
        recovered.acknowledge_effect("payment-3", REQUEST_A, RECEIPT_A).unwrap();
        assert_eq!(recovered.status("payment-3"), Some(EffectStatus::Acknowledged { receipt_digest: RECEIPT_A.to_owned() }));
    }

    #[test]
    fn same_effect_id_with_different_request_is_rejected() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        journal.begin_effect("payment-4", REQUEST_A).unwrap();
        assert_eq!(journal.begin_effect("payment-4", REQUEST_B), Err(JournalError::RequestConflict));
    }

    #[test]
    fn acknowledgement_must_match_request_and_receipt() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(journal.acknowledge_effect("missing", REQUEST_A, RECEIPT_A), Err(JournalError::UnknownEffect));
        journal.begin_effect("payment-5", REQUEST_A).unwrap();
        assert_eq!(journal.acknowledge_effect("payment-5", REQUEST_B, RECEIPT_A), Err(JournalError::RequestConflict));
        journal.acknowledge_effect("payment-5", REQUEST_A, RECEIPT_A).unwrap();
        assert_eq!(journal.acknowledge_effect("payment-5", REQUEST_A, RECEIPT_B), Err(JournalError::ReceiptConflict));
        assert_eq!(journal.acknowledge_effect("payment-5", REQUEST_A, RECEIPT_A), Ok(AcknowledgeResult::AlreadyAcknowledged));
    }

    #[test]
    fn concurrent_open_is_fail_closed() {
        let temp = TempDir::new();
        let _first = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::LockHeld)));
    }

    #[test]
    fn truncated_journal_is_not_silently_repaired() {
        let temp = TempDir::new();
        fs::write(temp.journal_path(), b"B\t7061796d656e742d36\tsha256:aaaaaaaa").unwrap();
        assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::CorruptJournal { .. })));
        assert!(!temp.journal_path().with_file_name("effects.journal.lock").exists());
    }

    #[test]
    fn out_of_order_acknowledgement_is_corruption() {
        let temp = TempDir::new();
        fs::write(temp.journal_path(), format!("J1\tA\t{}\t{}\t{}\n", encode_hex(b"never-begun"), REQUEST_A, RECEIPT_A)).unwrap();
        assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::CorruptJournal { .. })));
    }

    #[test]
    fn unresolved_effect_ids_are_sorted_and_acknowledged_ids_are_excluded() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        journal.begin_effect("z-last", REQUEST_A).unwrap();
        journal.begin_effect("a-first", REQUEST_B).unwrap();
        journal.mark_indeterminate("a-first", REQUEST_B).unwrap();
        journal.begin_effect("done", REQUEST_A).unwrap();
        journal.acknowledge_effect("done", REQUEST_A, RECEIPT_A).unwrap();
        assert_eq!(journal.unresolved_effect_ids(), vec!["a-first".to_owned(), "z-last".to_owned()]);
    }

    #[test]
    fn stale_corruption_lock_is_released_when_open_fails() {
        let temp = TempDir::new();
        fs::write(temp.journal_path(), b"not-a-journal\n").unwrap();
        for _ in 0..2 {
            assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::CorruptJournal { .. })));
        }
    }

    #[test]
    fn unsupported_journal_record_version_fails_closed() {
        let temp = TempDir::new();
        fs::write(temp.journal_path(), format!("J2\tB\t{}\t{}\n", encode_hex(b"effect"), REQUEST_A)).unwrap();
        assert!(matches!(
            DurableEffectJournal::open(temp.journal_path()),
            Err(JournalError::CorruptJournal { reason: "unsupported journal record version", .. })
        ));
    }

    #[test]
    fn noncanonical_effect_ids_and_digests_are_rejected() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(journal.begin_effect(" payment ", REQUEST_A), Err(JournalError::InvalidEffectId));
        assert_eq!(journal.begin_effect("payment", "SHA256:bad"), Err(JournalError::InvalidDigest));
    }
}
