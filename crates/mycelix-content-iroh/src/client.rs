use std::{
    fs,
    io::{Seek, SeekFrom},
    path::PathBuf,
};

use iroh::{Endpoint, EndpointAddr};
use mycelix_content_core::{BlobDescriptorV1, ContentDigestV1, DigestAlgorithmV1};
use mycelix_content_node::{LocalCasV1, PutOutcomeV1};
use sha2::Digest as _;
use tempfile::NamedTempFile;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::{
    ContentRequestV1, ContentResponseHeaderV1, ContentResponseStatusV1, TransportErrorV1,
    CONTENT_ALPN_V1, RESPONSE_HEADER_LEN_V1,
};

#[derive(Debug, Clone)]
pub struct ContentClientConfigV1 {
    pub temp_dir: PathBuf,
    pub max_blob_bytes: u64,
}

impl ContentClientConfigV1 {
    pub fn new(temp_dir: impl Into<PathBuf>, max_blob_bytes: u64) -> Self {
        Self {
            temp_dir: temp_dir.into(),
            max_blob_bytes,
        }
    }

    pub fn validate(&self) -> Result<(), TransportErrorV1> {
        if self.max_blob_bytes == 0 {
            return Err(TransportErrorV1::ZeroMaxBlobSize);
        }
        let metadata = fs::symlink_metadata(&self.temp_dir)
            .map_err(|_| TransportErrorV1::InvalidTempDirectory(self.temp_dir.clone()))?;
        if !metadata.is_dir() || metadata.file_type().is_symlink() {
            return Err(TransportErrorV1::InvalidTempDirectory(self.temp_dir.clone()));
        }
        Ok(())
    }
}

pub struct VerifiedDownloadV1 {
    temp: NamedTempFile,
    descriptor: BlobDescriptorV1,
}

impl std::fmt::Debug for VerifiedDownloadV1 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VerifiedDownloadV1")
            .field("descriptor", &self.descriptor)
            .finish_non_exhaustive()
    }
}

impl VerifiedDownloadV1 {
    pub fn descriptor(&self) -> &BlobDescriptorV1 {
        &self.descriptor
    }

    pub fn reopen(&self) -> Result<std::fs::File, TransportErrorV1> {
        Ok(self.temp.reopen()?)
    }

    /// Imports the already network-verified download through CF-03's normal
    /// ingest path. The deliberate second verification/copy keeps transport
    /// code from acquiring a privileged CAS promotion path.
    pub fn import_into_cas(self, cas: &LocalCasV1) -> Result<PutOutcomeV1, TransportErrorV1> {
        let mut file = self.temp.reopen()?;
        file.seek(SeekFrom::Start(0))?;
        Ok(cas.put(&self.descriptor, &mut file)?)
    }
}

#[derive(Debug, Clone)]
pub struct ContentClientV1 {
    endpoint: Endpoint,
    config: ContentClientConfigV1,
}

impl ContentClientV1 {
    pub fn new(
        endpoint: Endpoint,
        config: ContentClientConfigV1,
    ) -> Result<Self, TransportErrorV1> {
        config.validate()?;
        Ok(Self { endpoint, config })
    }

    pub fn endpoint(&self) -> &Endpoint {
        &self.endpoint
    }

    pub async fn fetch(
        &self,
        remote: EndpointAddr,
        expected: &BlobDescriptorV1,
    ) -> Result<VerifiedDownloadV1, TransportErrorV1> {
        self.config.validate()?;
        expected.validate()?;
        if expected.size_bytes > self.config.max_blob_bytes {
            return Err(TransportErrorV1::BlobTooLarge {
                size_bytes: expected.size_bytes,
                max_bytes: self.config.max_blob_bytes,
            });
        }

        let connection = self
            .endpoint
            .connect(remote, CONTENT_ALPN_V1)
            .await
            .map_err(|error| TransportErrorV1::Connect(error.to_string()))?;
        let (mut send, mut recv) = connection
            .open_bi()
            .await
            .map_err(|error| TransportErrorV1::Stream(error.to_string()))?;

        let request = ContentRequestV1 {
            digest: expected.digest,
        }
        .encode();
        AsyncWriteExt::write_all(&mut send, &request).await?;
        send.finish()
            .map_err(|error| TransportErrorV1::Stream(error.to_string()))?;

        let mut header_bytes = [0_u8; RESPONSE_HEADER_LEN_V1];
        AsyncReadExt::read_exact(&mut recv, &mut header_bytes).await?;
        let header = ContentResponseHeaderV1::decode(&header_bytes)?;
        match header.status {
            ContentResponseStatusV1::Ok => {}
            ContentResponseStatusV1::NotFound => {
                connection.close(0u32.into(), b"not found");
                return Err(TransportErrorV1::RemoteNotFound(expected.digest));
            }
            ContentResponseStatusV1::Busy => {
                connection.close(0u32.into(), b"busy");
                return Err(TransportErrorV1::RemoteBusy);
            }
            ContentResponseStatusV1::IntegrityFailure => {
                connection.close(0u32.into(), b"integrity failure");
                return Err(TransportErrorV1::RemoteIntegrityFailure);
            }
            ContentResponseStatusV1::ProtocolError => {
                connection.close(0u32.into(), b"protocol error");
                return Err(TransportErrorV1::RemoteProtocolError);
            }
            ContentResponseStatusV1::InternalError => {
                connection.close(0u32.into(), b"internal error");
                return Err(TransportErrorV1::RemoteInternalError);
            }
        }

        if header.size_bytes != expected.size_bytes {
            connection.close(0u32.into(), b"size mismatch");
            return Err(TransportErrorV1::SizeMismatch {
                expected: expected.size_bytes,
                actual: header.size_bytes,
            });
        }
        if header.size_bytes > self.config.max_blob_bytes {
            connection.close(0u32.into(), b"blob too large");
            return Err(TransportErrorV1::BlobTooLarge {
                size_bytes: header.size_bytes,
                max_bytes: self.config.max_blob_bytes,
            });
        }

        let temp = NamedTempFile::new_in(&self.config.temp_dir)?;
        let std_out = temp.reopen()?;
        let mut out = tokio::fs::File::from_std(std_out);
        let mut hasher = TransferHasherV1::new(expected.digest.algorithm);
        let mut remaining = header.size_bytes;
        let mut buffer = vec![0_u8; 64 * 1024];

        while remaining > 0 {
            let wanted = usize::try_from(remaining.min(buffer.len() as u64))
                .expect("bounded by buffer length");
            let read = AsyncReadExt::read(&mut recv, &mut buffer[..wanted]).await?;
            if read == 0 {
                return Err(TransportErrorV1::SizeMismatch {
                    expected: header.size_bytes,
                    actual: header.size_bytes - remaining,
                });
            }
            hasher.update(&buffer[..read]);
            AsyncWriteExt::write_all(&mut out, &buffer[..read]).await?;
            remaining -= read as u64;
        }

        let mut trailing = [0_u8; 1];
        if AsyncReadExt::read(&mut recv, &mut trailing).await? != 0 {
            return Err(TransportErrorV1::TrailingData);
        }

        AsyncWriteExt::flush(&mut out).await?;
        out.sync_all().await?;
        drop(out);

        let actual = ContentDigestV1 {
            algorithm: expected.digest.algorithm,
            bytes: hasher.finish(),
        };
        if actual != expected.digest {
            return Err(TransportErrorV1::DigestMismatch {
                expected: expected.digest,
                actual,
            });
        }

        connection.close(0u32.into(), b"verified");
        Ok(VerifiedDownloadV1 {
            temp,
            descriptor: expected.clone(),
        })
    }
}

enum TransferHasherV1 {
    Blake3(blake3::Hasher),
    Sha256(sha2::Sha256),
}

impl TransferHasherV1 {
    fn new(algorithm: DigestAlgorithmV1) -> Self {
        match algorithm {
            DigestAlgorithmV1::Blake3_256 => Self::Blake3(blake3::Hasher::new()),
            DigestAlgorithmV1::Sha256 => Self::Sha256(sha2::Sha256::new()),
        }
    }

    fn update(&mut self, bytes: &[u8]) {
        match self {
            Self::Blake3(hasher) => {
                hasher.update(bytes);
            }
            Self::Sha256(hasher) => {
                hasher.update(bytes);
            }
        }
    }

    fn finish(self) -> [u8; 32] {
        match self {
            Self::Blake3(hasher) => *hasher.finalize().as_bytes(),
            Self::Sha256(hasher) => {
                let digest = hasher.finalize();
                let mut out = [0_u8; 32];
                out.copy_from_slice(&digest);
                out
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{io::Read as _, net::Ipv4Addr, sync::Arc};

    use iroh::{endpoint::presets, protocol::Router};
    use mycelix_content_node::CasConfigV1;

    use super::*;
    use crate::{ContentProviderConfigV1, ContentProviderV1};

    async fn direct_endpoint() -> Endpoint {
        Endpoint::builder(presets::Minimal)
            .clear_ip_transports()
            .bind_addr((Ipv4Addr::LOCALHOST, 0))
            .expect("configure loopback transport")
            .bind()
            .await
            .expect("bind iroh endpoint")
    }

    #[tokio::test]
    async fn full_blob_round_trip_and_cas_import() {
        let provider_root = tempfile::tempdir().unwrap();
        let receiver_root = tempfile::tempdir().unwrap();
        let download_dir = tempfile::tempdir().unwrap();
        let provider_cas = Arc::new(
            LocalCasV1::open(CasConfigV1::new(provider_root.path(), 1024 * 1024)).unwrap(),
        );
        let receiver_cas =
            LocalCasV1::open(CasConfigV1::new(receiver_root.path(), 1024 * 1024)).unwrap();
        let bytes = b"content fabric over iroh";
        let expected = BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Sha256, bytes, None);
        provider_cas.put(&expected, &bytes[..]).unwrap();

        let provider_endpoint = direct_endpoint().await;
        let provider = ContentProviderV1::new(
            Arc::clone(&provider_cas),
            ContentProviderConfigV1::default(),
        )
        .unwrap();
        let router = Router::builder(provider_endpoint)
            .accept(CONTENT_ALPN_V1, provider)
            .spawn();

        let client_endpoint = direct_endpoint().await;
        let client = ContentClientV1::new(
            client_endpoint,
            ContentClientConfigV1::new(download_dir.path(), 1024 * 1024),
        )
        .unwrap();
        let download = client
            .fetch(router.endpoint().addr(), &expected)
            .await
            .unwrap();
        let mut downloaded = Vec::new();
        download.reopen().unwrap().read_to_end(&mut downloaded).unwrap();
        assert_eq!(downloaded, bytes);
        download.import_into_cas(&receiver_cas).unwrap();
        assert!(receiver_cas.contains_verified(&expected).unwrap());

        client.endpoint().close().await;
        router.shutdown().await.unwrap();
    }

    #[test]
    fn temp_directory_symlink_is_rejected() {
        #[cfg(unix)]
        {
            use std::os::unix::fs::symlink;
            let root = tempfile::tempdir().unwrap();
            let real = root.path().join("real");
            fs::create_dir(&real).unwrap();
            let link = root.path().join("link");
            symlink(&real, &link).unwrap();
            assert!(matches!(
                ContentClientConfigV1::new(&link, 1024).validate(),
                Err(TransportErrorV1::InvalidTempDirectory(_))
            ));
        }
    }
}
