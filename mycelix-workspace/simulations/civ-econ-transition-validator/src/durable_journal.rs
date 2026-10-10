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

/// Prevent malformed local history from causing unbounded per-record decoding.
const MAX_EFFECT_ID_BYTES: usize = 1024;
const MAX_JOURNAL_RECORD_BYTES: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JournalError {
    InvalidPath,
    InsecurePermissions,
    InvalidEffectId,
    InvalidDigest,
    LockHeld,
    Poisoned,
    UnknownEffect,
    RequestConflict,
    ProviderProfileConflict,
    ProviderProfileUnbound,
    ReceiptConflict,
    EvidenceConflict,
    CorruptJournal { line: usize, reason: &'static str },
    Io { operation: &'static str, kind: io::ErrorKind },
}

impl fmt::Display for JournalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPath => write!(f, "journal path must name a regular file in an existing directory"),
            Self::InsecurePermissions => write!(f, "journal file permissions grant group or other users access"),
            Self::InvalidEffectId => write!(f, "effect identity is empty or non-canonical"),
            Self::InvalidDigest => write!(f, "digest must be canonical sha256 lowercase hex"),
            Self::LockHeld => write!(f, "journal is already open or has a stale lock; inspect before recovery"),
            Self::Poisoned => write!(f, "journal had a write/sync failure; reopen and reconcile before further use"),
            Self::UnknownEffect => write!(f, "effect identity has no durable begin record"),
            Self::RequestConflict => write!(f, "effect identity was reused with a different request digest"),
            Self::ProviderProfileConflict => write!(f, "effect identity was reused with a different provider-profile digest"),
            Self::ProviderProfileUnbound => write!(f, "legacy effect has no durable provider-profile binding; explicit reconciliation is required"),
            Self::ReceiptConflict => write!(f, "effect identity was acknowledged with a different receipt digest"),
            Self::EvidenceConflict => write!(f, "effect identity was acknowledged with different or missing source evidence"),
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
    /// A legacy J1/J2 pending effect has no recorded rail identity and cannot
    /// be resumed under a caller-selected provider automatically.
    ProviderProfileUnboundNeedsReconciliation,
    AlreadyAcknowledged {
        receipt_digest: String,
        source_evidence_digest: Option<String>,
        provider_profile_digest: Option<String>,
    },
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AcknowledgeResult { Acknowledged, AlreadyAcknowledged }
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IndeterminateResult {
    Marked,
    AlreadyIndeterminate,
    AlreadyAcknowledged {
        receipt_digest: String,
        source_evidence_digest: Option<String>,
        provider_profile_digest: Option<String>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct JournalEntry {
    request_digest: String,
    /// None for legacy J1/J2 records, which did not bind the provider rail.
    provider_profile_digest: Option<String>,
    status: EffectStatus,
    /// None for legacy J1 acknowledgements; J2/J3 can persist this digest.
    source_evidence_digest: Option<String>,
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
        let mut lock_options = OpenOptions::new();
        lock_options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            lock_options.mode(0o600);
        }
        let mut lock_file = lock_options.open(&lock_path)
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

        let mut journal_options = OpenOptions::new();
        journal_options.read(true).append(true).create(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            journal_options.mode(0o600);
        }
        let mut file = journal_options.open(&path)
            .map_err(|err| io_error("open journal", err))?;
        ensure_private_file(&file)?;
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

    /// Source-evidence digest for an acknowledged effect, or None for unresolved
    /// effects and legacy J1 acknowledgements that did not persist this binding.
    pub fn source_evidence_digest(&self, id: &str) -> Option<&str> {
        self.entries.get(id).and_then(|entry| entry.source_evidence_digest.as_deref())
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
    pub fn begin_effect(
        &mut self,
        id: &str,
        request_digest: &str,
        provider_profile_digest: &str,
    ) -> Result<BeginResult, JournalError> {
        self.ensure_healthy()?;
        validate_effect_id(id)?;
        validate_digest(request_digest)?;
        validate_digest(provider_profile_digest)?;
        if let Some(existing) = self.entries.get(id) {
            if existing.request_digest != request_digest {
                return Err(JournalError::RequestConflict);
            }
            if let Some(recorded_profile) = existing.provider_profile_digest.as_deref() {
                if recorded_profile != provider_profile_digest {
                    return Err(JournalError::ProviderProfileConflict);
                }
            } else if !matches!(&existing.status, EffectStatus::Acknowledged { .. }) {
                return Ok(BeginResult::ProviderProfileUnboundNeedsReconciliation);
            }
            return Ok(match &existing.status {
                EffectStatus::Pending => BeginResult::PendingNeedsReconciliation,
                EffectStatus::Indeterminate => BeginResult::IndeterminateNeedsReconciliation,
                EffectStatus::Acknowledged { receipt_digest } => BeginResult::AlreadyAcknowledged {
                    receipt_digest: receipt_digest.clone(),
                    source_evidence_digest: existing.source_evidence_digest.clone(),
                    provider_profile_digest: existing.provider_profile_digest.clone(),
                },
            });
        }
        let record = format!(
            "J3\tB\t{}\t{}\t{}\n",
            encode_hex(id.as_bytes()),
            request_digest,
            provider_profile_digest,
        );
        self.append_and_sync(&record)?;
        self.entries.insert(id.to_owned(), JournalEntry {
            request_digest: request_digest.to_owned(),
            provider_profile_digest: Some(provider_profile_digest.to_owned()),
            status: EffectStatus::Pending,
            source_evidence_digest: None,
        });
        Ok(BeginResult::Started)
    }

    /// Persist a source-bound receipt after the outcome has been established
    /// under the exact external rail / authority profile.
    pub fn acknowledge_effect(
        &mut self,
        id: &str,
        request_digest: &str,
        provider_profile_digest: &str,
        receipt_digest: &str,
        source_evidence_digest: &str,
    ) -> Result<AcknowledgeResult, JournalError> {
        self.ensure_healthy()?;
        validate_effect_id(id)?;
        validate_digest(request_digest)?;
        validate_digest(provider_profile_digest)?;
        validate_digest(receipt_digest)?;
        validate_digest(source_evidence_digest)?;
        let existing = self.entries.get(id).ok_or(JournalError::UnknownEffect)?;
        if existing.request_digest != request_digest {
            return Err(JournalError::RequestConflict);
        }
        match existing.provider_profile_digest.as_deref() {
            Some(recorded) if recorded == provider_profile_digest => {}
            Some(_) => return Err(JournalError::ProviderProfileConflict),
            None => return Err(JournalError::ProviderProfileUnbound),
        }
        if let EffectStatus::Acknowledged { receipt_digest: old } = &existing.status {
            if old != receipt_digest {
                return Err(JournalError::ReceiptConflict);
            }
            if existing.source_evidence_digest.as_deref() != Some(source_evidence_digest) {
                return Err(JournalError::EvidenceConflict);
            }
            return Ok(AcknowledgeResult::AlreadyAcknowledged);
        }
        let record = format!(
            "J3\tA\t{}\t{}\t{}\t{}\t{}\n",
            encode_hex(id.as_bytes()),
            request_digest,
            provider_profile_digest,
            receipt_digest,
            source_evidence_digest,
        );
        self.append_and_sync(&record)?;
        if let Some(entry) = self.entries.get_mut(id) {
            entry.status = EffectStatus::Acknowledged { receipt_digest: receipt_digest.to_owned() };
            entry.source_evidence_digest = Some(source_evidence_digest.to_owned());
        }
        Ok(AcknowledgeResult::Acknowledged)
    }

    /// Record uncertainty; explicit reconciliation may later acknowledge it.
    pub fn mark_indeterminate(
        &mut self,
        id: &str,
        request_digest: &str,
        provider_profile_digest: &str,
    ) -> Result<IndeterminateResult, JournalError> {
        self.ensure_healthy()?;
        validate_effect_id(id)?;
        validate_digest(request_digest)?;
        validate_digest(provider_profile_digest)?;
        let existing = self.entries.get(id).ok_or(JournalError::UnknownEffect)?;
        if existing.request_digest != request_digest {
            return Err(JournalError::RequestConflict);
        }
        match existing.provider_profile_digest.as_deref() {
            Some(recorded) if recorded == provider_profile_digest => {}
            Some(_) => return Err(JournalError::ProviderProfileConflict),
            None => return Err(JournalError::ProviderProfileUnbound),
        }
        match &existing.status {
            EffectStatus::Acknowledged { receipt_digest } =>
                return Ok(IndeterminateResult::AlreadyAcknowledged {
                    receipt_digest: receipt_digest.clone(),
                    source_evidence_digest: existing.source_evidence_digest.clone(),
                    provider_profile_digest: existing.provider_profile_digest.clone(),
                }),
            EffectStatus::Indeterminate => return Ok(IndeterminateResult::AlreadyIndeterminate),
            EffectStatus::Pending => {}
        }
        let record = format!(
            "J3\tI\t{}\t{}\t{}\n",
            encode_hex(id.as_bytes()),
            request_digest,
            provider_profile_digest,
        );
        self.append_and_sync(&record)?;
        if let Some(entry) = self.entries.get_mut(id) {
            entry.status = EffectStatus::Indeterminate;
            entry.source_evidence_digest = None;
        }
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

#[cfg(unix)]
fn ensure_private_file(file: &File) -> Result<(), JournalError> {
    use std::os::unix::fs::PermissionsExt;
    let metadata = file.metadata().map_err(|err| io_error("inspect journal permissions", err))?;
    if metadata.permissions().mode() & 0o077 != 0 {
        return Err(JournalError::InsecurePermissions);
    }
    Ok(())
}

#[cfg(not(unix))]
fn ensure_private_file(_file: &File) -> Result<(), JournalError> {
    // The owner-only mode and permission-bit check are Unix-specific. Other
    // platforms require their native ACL/permission policy to be verified.
    Ok(())
}
fn validate_effect_id(id: &str) -> Result<(), JournalError> {
    if id.is_empty() || id.len() > MAX_EFFECT_ID_BYTES || id.trim() != id || id.bytes().any(|b| b == b'\n' || b == b'\r' || b == b'\0') {
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
        if line.len() > MAX_JOURNAL_RECORD_BYTES {
            return Err(JournalError::CorruptJournal {
                line: line_no, reason: "record exceeds maximum size"
            });
        }
        let fields: Vec<&str> = line.split('\t').collect();
        let (event, encoded_id, request_digest, provider_profile_digest, receipt_digest, source_evidence_digest) =
            match fields.as_slice() {
                [version, event, id, request]
                    if (*version == "J1" || *version == "J2")
                        && (*event == "B" || *event == "I") =>
                {
                    (*event, *id, *request, None, None, None)
                }
                [version, event, id, request, profile]
                    if *version == "J3" && (*event == "B" || *event == "I") =>
                {
                    (*event, *id, *request, Some(*profile), None, None)
                }
                [version, event, id, request, receipt]
                    if *version == "J1" && *event == "A" =>
                {
                    (*event, *id, *request, None, Some(*receipt), None)
                }
                [version, event, id, request, receipt, evidence]
                    if *version == "J2" && *event == "A" =>
                {
                    (*event, *id, *request, None, Some(*receipt), Some(*evidence))
                }
                [version, event, id, request, profile, receipt, evidence]
                    if *version == "J3" && *event == "A" =>
                {
                    (*event, *id, *request, Some(*profile), Some(*receipt), Some(*evidence))
                }
                [version, ..] if *version != "J1" && *version != "J2" && *version != "J3" =>
                    return Err(JournalError::CorruptJournal {
                        line: line_no, reason: "unsupported journal record version"
                    }),
                _ => return Err(JournalError::CorruptJournal {
                    line: line_no, reason: "invalid record shape or version"
                }),
            };
        if encoded_id.len() > MAX_EFFECT_ID_BYTES * 2 {
            return Err(JournalError::CorruptJournal {
                line: line_no, reason: "effect id exceeds maximum size"
            });
        }
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
        if let Some(profile) = provider_profile_digest {
            validate_digest(profile).map_err(|_| JournalError::CorruptJournal {
                line: line_no, reason: "provider-profile digest is invalid"
            })?;
        }
        match event {
            "B" => {
                if entries.contains_key(&id) {
                    return Err(JournalError::CorruptJournal {
                        line: line_no, reason: "duplicate begin event"
                    });
                }
                entries.insert(id, JournalEntry {
                    request_digest: request_digest.to_owned(),
                    provider_profile_digest: provider_profile_digest.map(str::to_owned),
                    status: EffectStatus::Pending,
                    source_evidence_digest: None,
                });
            }
            "I" => {
                let entry = entries.get_mut(&id).ok_or(JournalError::CorruptJournal {
                    line: line_no, reason: "indeterminate event precedes begin"
                })?;
                if entry.request_digest != request_digest
                    || entry.provider_profile_digest.as_deref() != provider_profile_digest
                    || entry.status != EffectStatus::Pending
                {
                    return Err(JournalError::CorruptJournal {
                        line: line_no, reason: "invalid indeterminate transition or provider binding"
                    });
                }
                entry.status = EffectStatus::Indeterminate;
            }
            "A" => {
                let receipt = receipt_digest.ok_or(JournalError::CorruptJournal {
                    line: line_no, reason: "acknowledgement receipt is missing"
                })?;
                validate_digest(receipt).map_err(|_| JournalError::CorruptJournal {
                    line: line_no, reason: "receipt digest is invalid"
                })?;
                if let Some(evidence) = source_evidence_digest {
                    validate_digest(evidence).map_err(|_| JournalError::CorruptJournal {
                        line: line_no, reason: "source evidence digest is invalid"
                    })?;
                }
                let entry = entries.get_mut(&id).ok_or(JournalError::CorruptJournal {
                    line: line_no, reason: "acknowledgement precedes begin"
                })?;
                if entry.request_digest != request_digest
                    || entry.provider_profile_digest.as_deref() != provider_profile_digest
                    || !matches!(&entry.status, EffectStatus::Pending | EffectStatus::Indeterminate)
                {
                    return Err(JournalError::CorruptJournal {
                        line: line_no, reason: "invalid acknowledgement transition or provider binding"
                    });
                }
                entry.status = EffectStatus::Acknowledged { receipt_digest: receipt.to_owned() };
                entry.source_evidence_digest = source_evidence_digest.map(str::to_owned);
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

    fn write_journal_fixture(path: &Path, contents: &[u8]) {
        fs::write(path, contents).expect("write journal fixture");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(path, fs::Permissions::from_mode(0o600))
                .expect("restrict journal fixture permissions");
        }
    }

    const PROVIDER_A: &str = "sha256:1111111111111111111111111111111111111111111111111111111111111111";
    const PROVIDER_B: &str = "sha256:2222222222222222222222222222222222222222222222222222222222222222";
    const REQUEST_A: &str = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const REQUEST_B: &str = "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const RECEIPT_A: &str = "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
    const RECEIPT_B: &str = "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
    const SOURCE_EVIDENCE_A: &str = "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
    const SOURCE_EVIDENCE_B: &str = "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";

    #[test]
    fn pending_intent_survives_restart_and_is_not_replayed_automatically() {
        let temp = TempDir::new();
        {
            let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
            assert_eq!(journal.begin_effect("payment-1", REQUEST_A, PROVIDER_A), Ok(BeginResult::Started));
            assert_eq!(journal.status("payment-1"), Some(EffectStatus::Pending));
        }
        let mut recovered = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(recovered.begin_effect("payment-1", REQUEST_A, PROVIDER_A), Ok(BeginResult::PendingNeedsReconciliation));
        assert_eq!(recovered.unresolved_effect_ids(), vec!["payment-1".to_owned()]);
    }

    #[test]
    fn acknowledgement_survives_restart_and_begin_is_idempotent() {
        let temp = TempDir::new();
        {
            let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
            journal.begin_effect("payment-2", REQUEST_A, PROVIDER_A).unwrap();
            assert_eq!(journal.acknowledge_effect("payment-2", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A), Ok(AcknowledgeResult::Acknowledged));
        }
        let mut recovered = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(recovered.begin_effect("payment-2", REQUEST_A, PROVIDER_A),
            Ok(BeginResult::AlreadyAcknowledged {
                receipt_digest: RECEIPT_A.to_owned(),
                source_evidence_digest: Some(SOURCE_EVIDENCE_A.to_owned()),
                provider_profile_digest: Some(PROVIDER_A.to_owned()),
            }));
        assert!(recovered.unresolved_effect_ids().is_empty());
    }

    #[test]
    fn uncertain_effect_can_be_reconciled_to_acknowledged() {
        let temp = TempDir::new();
        {
            let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
            journal.begin_effect("payment-3", REQUEST_A, PROVIDER_A).unwrap();
            assert_eq!(journal.mark_indeterminate("payment-3", REQUEST_A, PROVIDER_A), Ok(IndeterminateResult::Marked));
        }
        let mut recovered = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(recovered.status("payment-3"), Some(EffectStatus::Indeterminate));
        recovered.acknowledge_effect("payment-3", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A).unwrap();
        assert_eq!(recovered.status("payment-3"), Some(EffectStatus::Acknowledged { receipt_digest: RECEIPT_A.to_owned() }));
    }

    #[test]
    fn same_effect_id_with_different_request_is_rejected() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        journal.begin_effect("payment-4", REQUEST_A, PROVIDER_A).unwrap();
        assert_eq!(journal.begin_effect("payment-4", REQUEST_B, PROVIDER_A), Err(JournalError::RequestConflict));
    }

    #[test]
    fn acknowledgement_must_match_request_and_receipt() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(journal.acknowledge_effect("missing", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A), Err(JournalError::UnknownEffect));
        journal.begin_effect("payment-5", REQUEST_A, PROVIDER_A).unwrap();
        assert_eq!(journal.acknowledge_effect("payment-5", REQUEST_B, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A), Err(JournalError::RequestConflict));
        journal.acknowledge_effect("payment-5", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A).unwrap();
        assert_eq!(journal.acknowledge_effect("payment-5", REQUEST_A, PROVIDER_A, RECEIPT_B, SOURCE_EVIDENCE_A), Err(JournalError::ReceiptConflict));
        assert_eq!(journal.acknowledge_effect("payment-5", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A), Ok(AcknowledgeResult::AlreadyAcknowledged));
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
        write_journal_fixture(&temp.journal_path(), b"B\t7061796d656e742d36\tsha256:aaaaaaaa");
        assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::CorruptJournal { .. })));
        assert!(!temp.journal_path().with_file_name("effects.journal.lock").exists());
    }

    #[test]
    fn out_of_order_acknowledgement_is_corruption() {
        let temp = TempDir::new();
        write_journal_fixture(&temp.journal_path(), format!("J1\tA\t{}\t{}\t{}\n", encode_hex(b"never-begun"), REQUEST_A, RECEIPT_A).as_bytes());
        assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::CorruptJournal { .. })));
    }

    #[test]
    fn unresolved_effect_ids_are_sorted_and_acknowledged_ids_are_excluded() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        journal.begin_effect("z-last", REQUEST_A, PROVIDER_A).unwrap();
        journal.begin_effect("a-first", REQUEST_B, PROVIDER_A).unwrap();
        journal.mark_indeterminate("a-first", REQUEST_B, PROVIDER_A).unwrap();
        journal.begin_effect("done", REQUEST_A, PROVIDER_A).unwrap();
        journal.acknowledge_effect("done", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A).unwrap();
        assert_eq!(journal.unresolved_effect_ids(), vec!["a-first".to_owned(), "z-last".to_owned()]);
    }

    #[test]
    fn stale_corruption_lock_is_released_when_open_fails() {
        let temp = TempDir::new();
        write_journal_fixture(&temp.journal_path(), b"not-a-journal\n");
        for _ in 0..2 {
            assert!(matches!(DurableEffectJournal::open(temp.journal_path()), Err(JournalError::CorruptJournal { .. })));
        }
    }

    #[test]
    fn unsupported_journal_record_version_fails_closed() {
        let temp = TempDir::new();
        write_journal_fixture(&temp.journal_path(), format!("J4\tB\t{}\t{}\n", encode_hex(b"effect"), REQUEST_A).as_bytes());
        assert!(matches!(
            DurableEffectJournal::open(temp.journal_path()),
            Err(JournalError::CorruptJournal { reason: "unsupported journal record version", .. })
        ));
    }

    #[test]
    fn effect_identity_cannot_be_rebound_to_another_provider_profile() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        journal.begin_effect("payment-profile", REQUEST_A, PROVIDER_A).unwrap();

        assert_eq!(
            journal.begin_effect("payment-profile", REQUEST_A, PROVIDER_B),
            Err(JournalError::ProviderProfileConflict),
        );
        assert_eq!(
            journal.acknowledge_effect(
                "payment-profile", REQUEST_A, PROVIDER_B, RECEIPT_A, SOURCE_EVIDENCE_A
            ),
            Err(JournalError::ProviderProfileConflict),
        );
        assert_eq!(
            journal.mark_indeterminate("payment-profile", REQUEST_A, PROVIDER_B),
            Err(JournalError::ProviderProfileConflict),
        );
    }

    #[test]
    fn legacy_pending_effect_requires_provider_binding_reconciliation() {
        let temp = TempDir::new();
        let path = temp.journal_path();
        let encoded_id = encode_hex(b"legacy-pending");
        write_journal_fixture(&path, format!("J1\tB\t{}\t{}\n", encoded_id, REQUEST_A).as_bytes());

        let mut journal = DurableEffectJournal::open(&path).unwrap();
        assert_eq!(
            journal.begin_effect("legacy-pending", REQUEST_A, PROVIDER_A),
            Ok(BeginResult::ProviderProfileUnboundNeedsReconciliation),
        );
        assert_eq!(
            journal.acknowledge_effect(
                "legacy-pending", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A
            ),
            Err(JournalError::ProviderProfileUnbound),
        );
    }

    #[test]
    fn legacy_j2_pending_effect_requires_provider_binding_reconciliation() {
        let temp = TempDir::new();
        let path = temp.journal_path();
        let encoded_id = encode_hex(b"legacy-j2-pending");
        write_journal_fixture(&path, format!("J2\tB\t{}\t{}\n", encoded_id, REQUEST_A).as_bytes());

        let mut journal = DurableEffectJournal::open(&path).unwrap();
        assert_eq!(
            journal.begin_effect("legacy-j2-pending", REQUEST_A, PROVIDER_A),
            Ok(BeginResult::ProviderProfileUnboundNeedsReconciliation),
        );
    }

    #[test]
    fn j3_replay_rejects_provider_profile_change_between_begin_and_ack() {
        let temp = TempDir::new();
        let path = temp.journal_path();
        let encoded_id = encode_hex(b"cross-profile");
        write_journal_fixture(
            &path,
            format!(
                "J3\tB\t{}\t{}\t{}\nJ3\tA\t{}\t{}\t{}\t{}\t{}\n",
                encoded_id, REQUEST_A, PROVIDER_A,
                encoded_id, REQUEST_A, PROVIDER_B, RECEIPT_A, SOURCE_EVIDENCE_A
            ).as_bytes(),
        );

        assert!(matches!(
            DurableEffectJournal::open(&path),
            Err(JournalError::CorruptJournal {
                reason: "invalid acknowledgement transition or provider binding",
                ..
            })
        ));
        assert!(!path.with_file_name("effects.journal.lock").exists());
    }

    #[test]
    fn acknowledgement_cannot_be_rebound_to_different_source_evidence() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        journal.begin_effect("payment-evidence", REQUEST_A, PROVIDER_A).unwrap();
        journal.acknowledge_effect(
            "payment-evidence", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_A
        ).unwrap();

        assert_eq!(
            journal.acknowledge_effect(
                "payment-evidence", REQUEST_A, PROVIDER_A, RECEIPT_A, SOURCE_EVIDENCE_B
            ),
            Err(JournalError::EvidenceConflict),
        );
        assert_eq!(
            journal.source_evidence_digest("payment-evidence"),
            Some(SOURCE_EVIDENCE_A),
        );
    }

    #[test]
    fn legacy_j1_acknowledgement_replays_without_claiming_source_binding() {
        let temp = TempDir::new();
        let path = temp.journal_path();
        let encoded_id = encode_hex(b"legacy-payment");
        write_journal_fixture(
            &path,
            format!(
                "J1\tB\t{}\t{}\nJ1\tA\t{}\t{}\t{}\n",
                encoded_id, REQUEST_A, encoded_id, REQUEST_A, RECEIPT_A
            ).as_bytes(),
        );

        let mut journal = DurableEffectJournal::open(&path).unwrap();
        assert_eq!(
            journal.begin_effect("legacy-payment", REQUEST_A, PROVIDER_A),
            Ok(BeginResult::AlreadyAcknowledged {
                receipt_digest: RECEIPT_A.to_owned(),
                source_evidence_digest: None,
                provider_profile_digest: None,
            }),
        );
        assert_eq!(journal.source_evidence_digest("legacy-payment"), None);
    }

    #[test]
    fn j2_acknowledgement_with_invalid_source_evidence_fails_closed() {
        let temp = TempDir::new();
        let path = temp.journal_path();
        let encoded_id = encode_hex(b"bad-evidence");
        write_journal_fixture(
            &path,
            format!(
                "J2\tB\t{}\t{}\nJ2\tA\t{}\t{}\t{}\tsha256:BAD\n",
                encoded_id, REQUEST_A, encoded_id, REQUEST_A, RECEIPT_A
            ).as_bytes(),
        );

        assert!(matches!(
            DurableEffectJournal::open(&path),
            Err(JournalError::CorruptJournal {
                reason: "source evidence digest is invalid",
                ..
            })
        ));
        assert!(!path.with_file_name("effects.journal.lock").exists());
    }

    #[cfg(unix)]
    #[test]
    fn created_journal_and_lock_are_owner_only() {
        use std::os::unix::fs::PermissionsExt;

        let temp = TempDir::new();
        let journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        let journal_mode = fs::metadata(journal.path()).unwrap().permissions().mode();
        let lock_path = journal.path().with_file_name("effects.journal.lock");
        let lock_mode = fs::metadata(lock_path).unwrap().permissions().mode();

        assert_eq!(journal_mode & 0o077, 0);
        assert_eq!(lock_mode & 0o077, 0);
    }

    #[cfg(unix)]
    #[test]
    fn existing_journal_with_group_or_other_permissions_is_rejected() {
        use std::os::unix::fs::PermissionsExt;

        let temp = TempDir::new();
        let path = temp.journal_path();
        fs::write(&path, b"").unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o640)).unwrap();

        assert!(matches!(
            DurableEffectJournal::open(&path),
            Err(JournalError::InsecurePermissions)
        ));
        assert!(!path.with_file_name("effects.journal.lock").exists());
        assert_eq!(fs::read(&path).unwrap(), b"");
    }


    #[test]
    fn oversized_effect_id_is_rejected_before_append() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        let oversized_id = "e".repeat(MAX_EFFECT_ID_BYTES + 1);

        assert_eq!(
            journal.begin_effect(&oversized_id, REQUEST_A, PROVIDER_A),
            Err(JournalError::InvalidEffectId),
        );
        assert!(journal.unresolved_effect_ids().is_empty());
    }

    #[test]
    fn oversized_encoded_effect_id_fails_closed_before_decoding() {
        let temp = TempDir::new();
        let encoded_id = "a".repeat(MAX_EFFECT_ID_BYTES * 2 + 2);
        let record = format!("J3\\tB\\t{}\\t{}\\t{}\\n", encoded_id, REQUEST_A, PROVIDER_A);
        write_journal_fixture(&temp.journal_path(), record.as_bytes());

        assert!(matches!(
            DurableEffectJournal::open(temp.journal_path()),
            Err(JournalError::CorruptJournal {
                reason: "effect id exceeds maximum size",
                ..
            })
        ));
    }

    #[test]
    fn oversized_journal_record_fails_closed_before_field_parsing() {
        let temp = TempDir::new();
        let mut record = "malformed\\t".to_owned();
        record.push_str(&"x".repeat(MAX_JOURNAL_RECORD_BYTES));
        record.push('\n');
        write_journal_fixture(&temp.journal_path(), record.as_bytes());

        assert!(matches!(
            DurableEffectJournal::open(temp.journal_path()),
            Err(JournalError::CorruptJournal {
                reason: "record exceeds maximum size",
                ..
            })
        ));
    }

    #[test]
    fn noncanonical_effect_ids_and_digests_are_rejected() {
        let temp = TempDir::new();
        let mut journal = DurableEffectJournal::open(temp.journal_path()).unwrap();
        assert_eq!(journal.begin_effect(" payment ", REQUEST_A, PROVIDER_A), Err(JournalError::InvalidEffectId));
        assert_eq!(journal.begin_effect("payment", "SHA256:bad", PROVIDER_A), Err(JournalError::InvalidDigest));
    }
}
