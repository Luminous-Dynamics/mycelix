use std::{
    fs::{self, File, OpenOptions},
    io::{self, Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
};

#[cfg(unix)]
use std::os::unix::fs::OpenOptionsExt;

use fs2::FileExt;
use mycelix_content_core::{BlobDescriptorV1, ContentDigestV1, DigestAlgorithmV1};
use parking_lot::Mutex;
use tempfile::NamedTempFile;

use crate::{error::CasErrorV1, hasher::hash_reader};

const LOCK_FILE: &str = ".cas.lock";
const BLOBS_DIR: &str = "blobs";
const STAGING_DIR: &str = "staging";
const DIGEST_HEX_LEN: usize = 64;

#[derive(Debug, Clone)]
pub struct CasConfigV1 {
    pub root: PathBuf,
    pub quota_bytes: u64,
}

impl CasConfigV1 {
    pub fn new(root: impl Into<PathBuf>, quota_bytes: u64) -> Self {
        Self {
            root: root.into(),
            quota_bytes,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityV1 {
    pub quota_bytes: u64,
    pub used_bytes: u64,
    pub reserved_bytes: u64,
    pub available_bytes: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PutOutcomeV1 {
    Stored { bytes: u64 },
    AlreadyPresent { bytes: u64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuditReportV1 {
    pub verified_blobs: u64,
    pub verified_bytes: u64,
}

#[derive(Debug)]
pub struct VerifiedBlobV1 {
    file: File,
    size_bytes: u64,
}

impl VerifiedBlobV1 {
    pub fn size_bytes(&self) -> u64 {
        self.size_bytes
    }

    pub fn into_file(self) -> File {
        self.file
    }
}

impl Read for VerifiedBlobV1 {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        self.file.read(buffer)
    }
}

impl Seek for VerifiedBlobV1 {
    fn seek(&mut self, position: SeekFrom) -> io::Result<u64> {
        self.file.seek(position)
    }
}

#[derive(Debug, Default)]
struct UsageStateV1 {
    used_bytes: u64,
    reserved_bytes: u64,
}

struct ReservationV1<'a> {
    usage: &'a Mutex<UsageStateV1>,
    bytes: u64,
    committed: bool,
}

impl ReservationV1<'_> {
    fn commit(&mut self) {
        let mut usage = self.usage.lock();
        debug_assert!(usage.reserved_bytes >= self.bytes);
        usage.reserved_bytes -= self.bytes;
        usage.used_bytes += self.bytes;
        self.committed = true;
    }
}

impl Drop for ReservationV1<'_> {
    fn drop(&mut self) {
        if self.committed {
            return;
        }
        let mut usage = self.usage.lock();
        debug_assert!(usage.reserved_bytes >= self.bytes);
        usage.reserved_bytes -= self.bytes;
    }
}

/// Crash-safe local content-addressed store.
///
/// The final blob tree is immutable-by-convention and contains only digest-named,
/// read-only regular files. Ingest happens in a same-filesystem staging directory,
/// then promotes without clobbering an existing digest path.
pub struct LocalCasV1 {
    root: PathBuf,
    blobs_dir: PathBuf,
    staging_dir: PathBuf,
    quota_bytes: u64,
    usage: Mutex<UsageStateV1>,
    lock_file: File,
}

impl LocalCasV1 {
    pub fn open(config: CasConfigV1) -> Result<Self, CasErrorV1> {
        ensure_real_directory(&config.root)?;

        let lock_path = config.root.join(LOCK_FILE);
        reject_symlink_if_present(&lock_path)?;
        let lock_file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&lock_path)
            .map_err(|source| CasErrorV1::io(&lock_path, source))?;
        if let Err(source) = lock_file.try_lock_exclusive() {
            if source.kind() == io::ErrorKind::WouldBlock {
                return Err(CasErrorV1::AlreadyLocked(config.root));
            }
            return Err(CasErrorV1::io(lock_path, source));
        }

        let blobs_dir = config.root.join(BLOBS_DIR);
        let staging_dir = config.root.join(STAGING_DIR);
        ensure_real_directory(&blobs_dir)?;
        ensure_real_directory(&staging_dir)?;
        for algorithm in [DigestAlgorithmV1::Blake3_256, DigestAlgorithmV1::Sha256] {
            ensure_real_directory(&blobs_dir.join(algorithm.tag()))?;
        }

        cleanup_staging(&staging_dir)?;
        let used_bytes = scan_blob_tree(&blobs_dir)?;
        if used_bytes > config.quota_bytes {
            return Err(CasErrorV1::QuotaExceeded {
                quota_bytes: config.quota_bytes,
                used_bytes,
                reserved_bytes: 0,
                requested_bytes: 0,
            });
        }

        Ok(Self {
            root: config.root,
            blobs_dir,
            staging_dir,
            quota_bytes: config.quota_bytes,
            usage: Mutex::new(UsageStateV1 {
                used_bytes,
                reserved_bytes: 0,
            }),
            lock_file,
        })
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn capacity(&self) -> CapacityV1 {
        let usage = self.usage.lock();
        let occupied = usage.used_bytes.saturating_add(usage.reserved_bytes);
        CapacityV1 {
            quota_bytes: self.quota_bytes,
            used_bytes: usage.used_bytes,
            reserved_bytes: usage.reserved_bytes,
            available_bytes: self.quota_bytes.saturating_sub(occupied),
        }
    }

    pub fn put<R: Read>(
        &self,
        expected: &BlobDescriptorV1,
        mut reader: R,
    ) -> Result<PutOutcomeV1, CasErrorV1> {
        expected.validate()?;
        let final_path = self.blob_path(expected.digest);

        match self.open_verified(expected) {
            Ok(_) => {
                return Ok(PutOutcomeV1::AlreadyPresent {
                    bytes: expected.size_bytes,
                });
            }
            Err(CasErrorV1::NotFound(_)) => {}
            Err(other) => return Err(other),
        }

        let mut reservation = self.reserve(expected.size_bytes)?;
        let mut temp = NamedTempFile::new_in(&self.staging_dir)
            .map_err(|source| CasErrorV1::io(&self.staging_dir, source))?;

        // Write exactly the reserved length at most. Then probe one extra sender byte
        // without persisting it so a dishonest sender cannot consume unreserved disk.
        let mut bounded = (&mut reader).take(expected.size_bytes);
        let (actual_digest, actual_size) = hash_reader(
            expected.digest.algorithm,
            &mut bounded,
            |chunk| temp.as_file_mut().write_all(chunk),
        )
        .map_err(|source| CasErrorV1::io(temp.path(), source))?;

        if actual_size != expected.size_bytes {
            return Err(CasErrorV1::SizeMismatch {
                expected: expected.size_bytes,
                actual: actual_size,
            });
        }

        let mut extra = [0_u8; 1];
        let extra_read = reader
            .read(&mut extra)
            .map_err(|source| CasErrorV1::io(temp.path(), source))?;
        if extra_read != 0 {
            return Err(CasErrorV1::SizeMismatch {
                expected: expected.size_bytes,
                actual: expected.size_bytes.saturating_add(extra_read as u64),
            });
        }

        if actual_digest != expected.digest {
            return Err(CasErrorV1::DigestMismatch {
                expected: expected.digest,
                actual: actual_digest,
            });
        }

        temp.as_file()
            .sync_all()
            .map_err(|source| CasErrorV1::io(temp.path(), source))?;
        make_read_only(temp.as_file(), temp.path())?;
        temp.as_file()
            .sync_all()
            .map_err(|source| CasErrorV1::io(temp.path(), source))?;

        match temp.persist_noclobber(&final_path) {
            Ok(_final_file) => {
                // The blob is visible now. Account for it before any subsequent
                // directory fsync can fail so capacity never understates visible data.
                reservation.commit();
                let parent = final_path
                    .parent()
                    .expect("digest path always has an algorithm directory");
                sync_directory(parent)?;
                sync_directory(&self.staging_dir)?;
                Ok(PutOutcomeV1::Stored {
                    bytes: expected.size_bytes,
                })
            }
            Err(error) if error.error.kind() == io::ErrorKind::AlreadyExists => {
                drop(error.file);
                self.open_verified(expected)?;
                Ok(PutOutcomeV1::AlreadyPresent {
                    bytes: expected.size_bytes,
                })
            }
            Err(error) => {
                let source = error.error;
                drop(error.file);
                Err(CasErrorV1::io(final_path, source))
            }
        }
    }

    /// Verifies a digest-addressed blob without requiring caller-supplied size metadata.
    /// The returned handle is the same handle that was hashed and is rewound to byte zero.
    pub fn open_verified_digest(
        &self,
        digest: ContentDigestV1,
    ) -> Result<VerifiedBlobV1, CasErrorV1> {
        self.open_verified_inner(digest, None)
    }

    /// Verifies both exact-byte identity and the caller's declared blob size.
    pub fn open_verified(&self, expected: &BlobDescriptorV1) -> Result<File, CasErrorV1> {
        expected.validate()?;
        self.open_verified_inner(expected.digest, Some(expected.size_bytes))
            .map(VerifiedBlobV1::into_file)
    }

    pub fn contains_verified(&self, expected: &BlobDescriptorV1) -> Result<bool, CasErrorV1> {
        match self.open_verified(expected) {
            Ok(_) => Ok(true),
            Err(CasErrorV1::NotFound(_)) => Ok(false),
            Err(other) => Err(other),
        }
    }

    pub fn contains_digest_verified(&self, digest: ContentDigestV1) -> Result<bool, CasErrorV1> {
        match self.open_verified_digest(digest) {
            Ok(_) => Ok(true),
            Err(CasErrorV1::NotFound(_)) => Ok(false),
            Err(other) => Err(other),
        }
    }

    /// Re-hashes every immutable blob currently present in the store.
    pub fn audit_all(&self) -> Result<AuditReportV1, CasErrorV1> {
        let mut report = AuditReportV1 {
            verified_blobs: 0,
            verified_bytes: 0,
        };
        for algorithm in [DigestAlgorithmV1::Blake3_256, DigestAlgorithmV1::Sha256] {
            let algorithm_dir = self.blobs_dir.join(algorithm.tag());
            for entry in fs::read_dir(&algorithm_dir)
                .map_err(|source| CasErrorV1::io(&algorithm_dir, source))?
            {
                let entry = entry.map_err(|source| CasErrorV1::io(&algorithm_dir, source))?;
                let path = entry.path();
                let metadata = fs::symlink_metadata(&path)
                    .map_err(|source| CasErrorV1::io(&path, source))?;
                validate_final_file_metadata(&path, &metadata)?;
                let digest = digest_from_path(algorithm, &path)?;
                let verified = self.open_verified_digest(digest)?;
                report.verified_blobs = report
                    .verified_blobs
                    .checked_add(1)
                    .ok_or(CasErrorV1::UsageOverflow)?;
                report.verified_bytes = report
                    .verified_bytes
                    .checked_add(verified.size_bytes())
                    .ok_or(CasErrorV1::UsageOverflow)?;
            }
        }
        Ok(report)
    }

    fn open_verified_inner(
        &self,
        digest: ContentDigestV1,
        expected_size: Option<u64>,
    ) -> Result<VerifiedBlobV1, CasErrorV1> {
        let path = self.blob_path(digest);
        let metadata = match fs::symlink_metadata(&path) {
            Ok(metadata) => metadata,
            Err(source) if source.kind() == io::ErrorKind::NotFound => {
                return Err(CasErrorV1::NotFound(digest));
            }
            Err(source) => return Err(CasErrorV1::io(&path, source)),
        };
        validate_final_file_metadata(&path, &metadata)?;

        let mut file = open_blob_no_follow(&path)?;
        let opened_metadata = file
            .metadata()
            .map_err(|source| CasErrorV1::io(&path, source))?;
        if let Some(expected_size) = expected_size
            && opened_metadata.len() != expected_size
        {
            return Err(CasErrorV1::SizeMismatch {
                expected: expected_size,
                actual: opened_metadata.len(),
            });
        }

        let (actual_digest, actual_size) = hash_reader(digest.algorithm, &mut file, |_| Ok(()))
            .map_err(|source| CasErrorV1::io(&path, source))?;
        if actual_size != opened_metadata.len() {
            return Err(CasErrorV1::SizeMismatch {
                expected: opened_metadata.len(),
                actual: actual_size,
            });
        }
        if actual_digest != digest {
            return Err(CasErrorV1::DigestMismatch {
                expected: digest,
                actual: actual_digest,
            });
        }
        file.seek(SeekFrom::Start(0))
            .map_err(|source| CasErrorV1::io(&path, source))?;
        Ok(VerifiedBlobV1 {
            file,
            size_bytes: actual_size,
        })
    }

    fn reserve(&self, requested_bytes: u64) -> Result<ReservationV1<'_>, CasErrorV1> {
        let mut usage = self.usage.lock();
        let occupied = usage
            .used_bytes
            .checked_add(usage.reserved_bytes)
            .ok_or(CasErrorV1::UsageOverflow)?;
        let projected = occupied
            .checked_add(requested_bytes)
            .ok_or(CasErrorV1::UsageOverflow)?;
        if projected > self.quota_bytes {
            return Err(CasErrorV1::QuotaExceeded {
                quota_bytes: self.quota_bytes,
                used_bytes: usage.used_bytes,
                reserved_bytes: usage.reserved_bytes,
                requested_bytes,
            });
        }
        usage.reserved_bytes = usage
            .reserved_bytes
            .checked_add(requested_bytes)
            .ok_or(CasErrorV1::UsageOverflow)?;
        drop(usage);
        Ok(ReservationV1 {
            usage: &self.usage,
            bytes: requested_bytes,
            committed: false,
        })
    }

    fn blob_path(&self, digest: ContentDigestV1) -> PathBuf {
        self.blobs_dir
            .join(digest.algorithm.tag())
            .join(hex::encode(digest.bytes))
    }
}

impl Drop for LocalCasV1 {
    fn drop(&mut self) {
        let _ = FileExt::unlock(&self.lock_file);
    }
}

fn open_blob_no_follow(path: &Path) -> Result<File, CasErrorV1> {
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    options.custom_flags(libc::O_NOFOLLOW);
    options.open(path).map_err(|source| CasErrorV1::io(path, source))
}

fn reject_symlink_if_present(path: &Path) -> Result<(), CasErrorV1> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            Err(CasErrorV1::UnexpectedEntry(path.to_path_buf()))
        }
        Ok(_) => Ok(()),
        Err(source) if source.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(source) => Err(CasErrorV1::io(path, source)),
    }
}

fn ensure_real_directory(path: &Path) -> Result<(), CasErrorV1> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.is_dir() && !metadata.file_type().is_symlink() => Ok(()),
        Ok(_) => Err(CasErrorV1::InvalidDirectory(path.to_path_buf())),
        Err(source) if source.kind() == io::ErrorKind::NotFound => {
            fs::create_dir_all(path).map_err(|source| CasErrorV1::io(path, source))?;
            let metadata =
                fs::symlink_metadata(path).map_err(|source| CasErrorV1::io(path, source))?;
            if metadata.is_dir() && !metadata.file_type().is_symlink() {
                Ok(())
            } else {
                Err(CasErrorV1::InvalidDirectory(path.to_path_buf()))
            }
        }
        Err(source) => Err(CasErrorV1::io(path, source)),
    }
}

fn cleanup_staging(staging_dir: &Path) -> Result<(), CasErrorV1> {
    let mut removed_any = false;
    for entry in fs::read_dir(staging_dir).map_err(|source| CasErrorV1::io(staging_dir, source))? {
        let entry = entry.map_err(|source| CasErrorV1::io(staging_dir, source))?;
        let path = entry.path();
        let metadata = fs::symlink_metadata(&path).map_err(|source| CasErrorV1::io(&path, source))?;
        if !metadata.is_file() || metadata.file_type().is_symlink() {
            return Err(CasErrorV1::UnexpectedEntry(path));
        }
        fs::remove_file(&path).map_err(|source| CasErrorV1::io(&path, source))?;
        removed_any = true;
    }
    if removed_any {
        sync_directory(staging_dir)?;
    }
    Ok(())
}

fn scan_blob_tree(blobs_dir: &Path) -> Result<u64, CasErrorV1> {
    let mut used_bytes = 0_u64;
    for entry in fs::read_dir(blobs_dir).map_err(|source| CasErrorV1::io(blobs_dir, source))? {
        let entry = entry.map_err(|source| CasErrorV1::io(blobs_dir, source))?;
        let algorithm_path = entry.path();
        let metadata = fs::symlink_metadata(&algorithm_path)
            .map_err(|source| CasErrorV1::io(&algorithm_path, source))?;
        if !metadata.is_dir() || metadata.file_type().is_symlink() {
            return Err(CasErrorV1::UnexpectedEntry(algorithm_path));
        }
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name != DigestAlgorithmV1::Blake3_256.tag() && name != DigestAlgorithmV1::Sha256.tag() {
            return Err(CasErrorV1::UnexpectedEntry(algorithm_path));
        }
        for blob_entry in fs::read_dir(&algorithm_path)
            .map_err(|source| CasErrorV1::io(&algorithm_path, source))?
        {
            let blob_entry = blob_entry.map_err(|source| CasErrorV1::io(&algorithm_path, source))?;
            let path = blob_entry.path();
            let metadata =
                fs::symlink_metadata(&path).map_err(|source| CasErrorV1::io(&path, source))?;
            validate_final_file_metadata(&path, &metadata)?;
            used_bytes = used_bytes
                .checked_add(metadata.len())
                .ok_or(CasErrorV1::UsageOverflow)?;
        }
    }
    Ok(used_bytes)
}

fn validate_final_file_metadata(path: &Path, metadata: &fs::Metadata) -> Result<(), CasErrorV1> {
    if !metadata.is_file() || metadata.file_type().is_symlink() {
        return Err(CasErrorV1::UnexpectedEntry(path.to_path_buf()));
    }
    if !metadata.permissions().readonly() {
        return Err(CasErrorV1::MutableBlobFile(path.to_path_buf()));
    }
    validate_digest_filename(path)
}

fn validate_digest_filename(path: &Path) -> Result<(), CasErrorV1> {
    let Some(name) = path.file_name().and_then(|value| value.to_str()) else {
        return Err(CasErrorV1::InvalidBlobFilename(path.to_path_buf()));
    };
    if name.len() != DIGEST_HEX_LEN
        || !name
            .as_bytes()
            .iter()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(byte))
    {
        return Err(CasErrorV1::InvalidBlobFilename(path.to_path_buf()));
    }
    Ok(())
}

fn digest_from_path(
    algorithm: DigestAlgorithmV1,
    path: &Path,
) -> Result<ContentDigestV1, CasErrorV1> {
    validate_digest_filename(path)?;
    let name = path
        .file_name()
        .and_then(|value| value.to_str())
        .ok_or_else(|| CasErrorV1::InvalidBlobFilename(path.to_path_buf()))?;
    let mut bytes = [0_u8; 32];
    hex::decode_to_slice(name, &mut bytes)
        .map_err(|_| CasErrorV1::InvalidBlobFilename(path.to_path_buf()))?;
    Ok(ContentDigestV1 { algorithm, bytes })
}

fn make_read_only(file: &File, path: &Path) -> Result<(), CasErrorV1> {
    let mut permissions = file
        .metadata()
        .map_err(|source| CasErrorV1::io(path, source))?
        .permissions();
    permissions.set_readonly(true);
    file.set_permissions(permissions)
        .map_err(|source| CasErrorV1::io(path, source))
}

fn sync_directory(path: &Path) -> Result<(), CasErrorV1> {
    #[cfg(unix)]
    {
        let directory = File::open(path).map_err(|source| CasErrorV1::io(path, source))?;
        directory
            .sync_all()
            .map_err(|source| CasErrorV1::io(path, source))?;
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read as _;

    fn descriptor(bytes: &[u8], algorithm: DigestAlgorithmV1) -> BlobDescriptorV1 {
        BlobDescriptorV1::from_bytes(algorithm, bytes, None)
    }

    fn open_cas(root: &Path, quota_bytes: u64) -> LocalCasV1 {
        LocalCasV1::open(CasConfigV1::new(root, quota_bytes)).expect("open CAS")
    }

    #[test]
    fn round_trip_is_verified_and_rewound() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let bytes = b"content fabric";
        let expected = descriptor(bytes, DigestAlgorithmV1::Blake3_256);
        assert_eq!(
            cas.put(&expected, &bytes[..]).unwrap(),
            PutOutcomeV1::Stored {
                bytes: bytes.len() as u64
            }
        );
        let mut file = cas.open_verified(&expected).unwrap();
        let mut actual = Vec::new();
        file.read_to_end(&mut actual).unwrap();
        assert_eq!(actual, bytes);
        assert!(cas.contains_verified(&expected).unwrap());
    }

    #[test]
    fn digest_only_verified_open_derives_size_and_rewinds_same_handle() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let bytes = b"digest-only";
        let expected = descriptor(bytes, DigestAlgorithmV1::Sha256);
        cas.put(&expected, &bytes[..]).unwrap();
        let mut verified = cas.open_verified_digest(expected.digest).unwrap();
        assert_eq!(verified.size_bytes(), bytes.len() as u64);
        let mut actual = Vec::new();
        verified.read_to_end(&mut actual).unwrap();
        assert_eq!(actual, bytes);
        assert!(cas.contains_digest_verified(expected.digest).unwrap());
    }

    #[test]
    fn duplicate_put_is_idempotent_and_does_not_grow_usage() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let bytes = b"same";
        let expected = descriptor(bytes, DigestAlgorithmV1::Sha256);
        cas.put(&expected, &bytes[..]).unwrap();
        let before = cas.capacity();
        assert_eq!(
            cas.put(&expected, &bytes[..]).unwrap(),
            PutOutcomeV1::AlreadyPresent {
                bytes: bytes.len() as u64
            }
        );
        assert_eq!(cas.capacity(), before);
    }

    #[test]
    fn truncated_stream_never_promotes() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let expected = descriptor(b"12345", DigestAlgorithmV1::Blake3_256);
        let error = cas.put(&expected, &b"1234"[..]).unwrap_err();
        assert!(matches!(error, CasErrorV1::SizeMismatch { actual: 4, .. }));
        assert!(!cas.contains_verified(&expected).unwrap());
        assert_eq!(cas.capacity().used_bytes, 0);
        assert_eq!(cas.capacity().reserved_bytes, 0);
    }

    #[test]
    fn oversized_stream_is_probed_but_extra_byte_is_not_persisted() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let expected = descriptor(b"1234", DigestAlgorithmV1::Blake3_256);
        let error = cas.put(&expected, &b"12345-many-more"[..]).unwrap_err();
        assert!(matches!(error, CasErrorV1::SizeMismatch { actual: 5, .. }));
        assert_eq!(cas.capacity().used_bytes, 0);
        assert_eq!(cas.capacity().reserved_bytes, 0);
    }

    #[test]
    fn wrong_digest_never_promotes() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let expected = descriptor(b"good", DigestAlgorithmV1::Blake3_256);
        let error = cas.put(&expected, &b"evil"[..]).unwrap_err();
        assert!(matches!(error, CasErrorV1::DigestMismatch { .. }));
        assert_eq!(cas.capacity().used_bytes, 0);
    }

    #[test]
    fn quota_is_reserved_before_ingest() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 3);
        let expected = descriptor(b"four", DigestAlgorithmV1::Blake3_256);
        let error = cas.put(&expected, &b"four"[..]).unwrap_err();
        assert!(matches!(error, CasErrorV1::QuotaExceeded { .. }));
        assert_eq!(cas.capacity().used_bytes, 0);
    }

    #[test]
    fn usage_is_reconstructed_after_restart() {
        let dir = tempfile::tempdir().unwrap();
        let bytes = b"persistent";
        let expected = descriptor(bytes, DigestAlgorithmV1::Sha256);
        {
            let cas = open_cas(dir.path(), 1024);
            cas.put(&expected, &bytes[..]).unwrap();
            assert_eq!(cas.capacity().used_bytes, bytes.len() as u64);
        }
        let reopened = open_cas(dir.path(), 1024);
        assert_eq!(reopened.capacity().used_bytes, bytes.len() as u64);
        assert!(reopened.contains_verified(&expected).unwrap());
    }

    #[test]
    fn staging_files_from_interrupted_ingest_are_cleaned_on_open() {
        let dir = tempfile::tempdir().unwrap();
        let staging = dir.path().join(STAGING_DIR);
        fs::create_dir_all(&staging).unwrap();
        let abandoned = staging.join("abandoned-partial");
        fs::write(&abandoned, b"partial").unwrap();
        let _cas = open_cas(dir.path(), 1024);
        assert!(!abandoned.exists());
    }

    #[test]
    fn second_process_owner_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let _first = open_cas(dir.path(), 1024);
        let second = LocalCasV1::open(CasConfigV1::new(dir.path(), 1024));
        assert!(matches!(second, Err(CasErrorV1::AlreadyLocked(_))));
    }

    #[test]
    fn audit_detects_corrupted_blob_bytes() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let bytes = b"original";
        let expected = descriptor(bytes, DigestAlgorithmV1::Blake3_256);
        cas.put(&expected, &bytes[..]).unwrap();
        let path = cas.blob_path(expected.digest);
        let mut permissions = fs::metadata(&path).unwrap().permissions();
        permissions.set_readonly(false);
        fs::set_permissions(&path, permissions.clone()).unwrap();
        fs::write(&path, b"tampered").unwrap();
        permissions.set_readonly(true);
        fs::set_permissions(&path, permissions).unwrap();
        let error = cas.audit_all().unwrap_err();
        assert!(matches!(error, CasErrorV1::DigestMismatch { .. }));
    }

    #[test]
    fn audit_reports_verified_totals() {
        let dir = tempfile::tempdir().unwrap();
        let cas = open_cas(dir.path(), 1024);
        let a = descriptor(b"a", DigestAlgorithmV1::Blake3_256);
        let b = descriptor(b"bb", DigestAlgorithmV1::Sha256);
        cas.put(&a, &b"a"[..]).unwrap();
        cas.put(&b, &b"bb"[..]).unwrap();
        assert_eq!(
            cas.audit_all().unwrap(),
            AuditReportV1 {
                verified_blobs: 2,
                verified_bytes: 3
            }
        );
    }

    #[cfg(unix)]
    #[test]
    fn symlink_in_immutable_tree_is_rejected_on_startup() {
        use std::os::unix::fs::symlink;

        let dir = tempfile::tempdir().unwrap();
        let algorithm_dir = dir
            .path()
            .join(BLOBS_DIR)
            .join(DigestAlgorithmV1::Blake3_256.tag());
        fs::create_dir_all(&algorithm_dir).unwrap();
        let target = dir.path().join("target");
        fs::write(&target, b"x").unwrap();
        symlink(&target, algorithm_dir.join("a".repeat(64))).unwrap();
        let opened = LocalCasV1::open(CasConfigV1::new(dir.path(), 1024));
        assert!(matches!(opened, Err(CasErrorV1::UnexpectedEntry(_))));
    }

    #[test]
    fn unexpected_blob_tree_entries_fail_closed() {
        let dir = tempfile::tempdir().unwrap();
        let blobs = dir.path().join(BLOBS_DIR);
        fs::create_dir_all(&blobs).unwrap();
        fs::write(blobs.join("surprise"), b"not a digest directory").unwrap();
        let opened = LocalCasV1::open(CasConfigV1::new(dir.path(), 1024));
        assert!(matches!(opened, Err(CasErrorV1::UnexpectedEntry(_))));
    }
}
