use std::{fmt, sync::Arc, time::Duration};

use iroh::{
    endpoint::{Connection, SendStream},
    protocol::{AcceptError, ProtocolHandler},
};
use mycelix_content_node::{CasErrorV1, LocalCasV1};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    sync::Semaphore,
};

use crate::{
    ContentRequestV1, ContentResponseHeaderV1, ContentResponseStatusV1, TransportErrorV1,
    REQUEST_LEN_V1,
};

const MAX_PROVIDER_CONCURRENCY_V1: usize = 64;
const ACK_WAIT_V1: Duration = Duration::from_secs(5);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContentProviderConfigV1 {
    pub max_concurrent_transfers: usize,
}

impl Default for ContentProviderConfigV1 {
    fn default() -> Self {
        Self {
            max_concurrent_transfers: 8,
        }
    }
}

impl ContentProviderConfigV1 {
    pub fn validate(self) -> Result<Self, TransportErrorV1> {
        if !(1..=MAX_PROVIDER_CONCURRENCY_V1).contains(&self.max_concurrent_transfers) {
            return Err(TransportErrorV1::InvalidConcurrency {
                value: self.max_concurrent_transfers,
                maximum: MAX_PROVIDER_CONCURRENCY_V1,
            });
        }
        Ok(self)
    }
}

#[derive(Clone)]
pub struct ContentProviderV1 {
    cas: Arc<LocalCasV1>,
    slots: Arc<Semaphore>,
}

impl fmt::Debug for ContentProviderV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ContentProviderV1")
            .field("available_slots", &self.slots.available_permits())
            .finish_non_exhaustive()
    }
}

impl ContentProviderV1 {
    pub fn new(
        cas: Arc<LocalCasV1>,
        config: ContentProviderConfigV1,
    ) -> Result<Self, TransportErrorV1> {
        let config = config.validate()?;
        Ok(Self {
            cas,
            slots: Arc::new(Semaphore::new(config.max_concurrent_transfers)),
        })
    }
}

impl ProtocolHandler for ContentProviderV1 {
    async fn accept(&self, connection: Connection) -> Result<(), AcceptError> {
        let (mut send, mut recv) = connection.accept_bi().await?;
        let mut request_bytes = [0_u8; REQUEST_LEN_V1];
        AsyncReadExt::read_exact(&mut recv, &mut request_bytes).await?;
        let mut trailing = [0_u8; 1];
        if AsyncReadExt::read(&mut recv, &mut trailing).await? != 0 {
            send_status(&mut send, ContentResponseStatusV1::ProtocolError).await?;
            return Ok(());
        }

        let request = match ContentRequestV1::decode(&request_bytes) {
            Ok(request) => request,
            Err(_) => {
                send_status(&mut send, ContentResponseStatusV1::ProtocolError).await?;
                return Ok(());
            }
        };

        let _permit = match self.slots.clone().try_acquire_owned() {
            Ok(permit) => permit,
            Err(_) => {
                send_status(&mut send, ContentResponseStatusV1::Busy).await?;
                return Ok(());
            }
        };

        let cas = Arc::clone(&self.cas);
        let digest = request.digest;
        let opened = tokio::task::spawn_blocking(move || cas.open_verified_digest(digest)).await;
        let blob = match opened {
            Ok(Ok(blob)) => blob,
            Ok(Err(CasErrorV1::NotFound(_))) => {
                send_status(&mut send, ContentResponseStatusV1::NotFound).await?;
                return Ok(());
            }
            Ok(Err(error)) if is_integrity_failure(&error) => {
                send_status(&mut send, ContentResponseStatusV1::IntegrityFailure).await?;
                return Ok(());
            }
            Ok(Err(_)) | Err(_) => {
                send_status(&mut send, ContentResponseStatusV1::InternalError).await?;
                return Ok(());
            }
        };

        let size_bytes = blob.size_bytes();
        AsyncWriteExt::write_all(&mut send, &ContentResponseHeaderV1::ok(size_bytes).encode())
            .await?;
        let mut file = tokio::fs::File::from_std(blob.into_file());
        let copied = tokio::io::copy(&mut file, &mut send).await?;
        if copied != size_bytes {
            connection.close(500u32.into(), b"local blob length changed during transfer");
            return Err(AcceptError::from_err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "verified blob length changed while streaming",
            )));
        }
        finish_bounded(&mut send).await?;
        Ok(())
    }
}

fn is_integrity_failure(error: &CasErrorV1) -> bool {
    matches!(
        error,
        CasErrorV1::UnexpectedEntry(_)
            | CasErrorV1::InvalidBlobFilename(_)
            | CasErrorV1::MutableBlobFile(_)
            | CasErrorV1::SizeMismatch { .. }
            | CasErrorV1::DigestMismatch { .. }
    )
}

async fn send_status(
    send: &mut SendStream,
    status: ContentResponseStatusV1,
) -> Result<(), AcceptError> {
    AsyncWriteExt::write_all(send, &ContentResponseHeaderV1::error(status).encode()).await?;
    finish_bounded(send).await
}

async fn finish_bounded(send: &mut SendStream) -> Result<(), AcceptError> {
    send.finish()?;
    let _ = tokio::time::timeout(ACK_WAIT_V1, send.stopped()).await;
    Ok(())
}
