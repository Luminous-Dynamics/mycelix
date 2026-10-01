//! Dedicated browser ↔ renderer capability IPC.
//!
//! This transport is intentionally separate from the authenticated Spore
//! service channel in `runtime.rs`. A valid Spore BridgeCodec frame never
//! establishes renderer identity or capability authority.
//!
//! Security boundary:
//! `UnixListener -> live peer credentials -> RendererSessionManager ->
//! CapabilityIngress -> grant/policy authorization`.
//!
//! The renderer endpoint is broker-owned. The renderer never supplies its
//! process identity, site, or navigation generation as authority.

use crate::capability::CapabilityRequest;
use crate::lifecycle::RendererBinding;
use crate::session::{CapabilityIngress, CapabilityIngressError, RendererSessionManager};
use std::fmt;
use std::path::{Path, PathBuf};
use tokio::io::AsyncReadExt;
use tokio::net::{UnixListener, UnixStream};

/// Renderer capability IPC protocol version.
pub const RENDERER_IPC_VERSION: u16 = 1;

/// Renderer capability frames are intentionally much smaller than the general
/// Spore content frame limit. Capability requests should be small control
/// messages, not content transport.
pub const MAX_RENDERER_IPC_FRAME_SIZE: usize = 64 * 1024;

/// A dedicated renderer IPC listener. It owns no Spore state and has no access
/// to BridgeCodec authentication.
#[derive(Debug)]
pub struct RendererIpcListener {
    listener: UnixListener,
    socket_path: PathBuf,
}

#[derive(Debug)]
pub enum RendererIpcError {
    Bind(std::io::Error),
    Io(std::io::Error),
    InvalidFrameLength(usize),
    InvalidVersion(u16),
    Capability(CapabilityIngressError),
    SocketPermissions(std::io::Error),
}

impl fmt::Display for RendererIpcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bind(error) => write!(f, "renderer IPC bind failed: {error}"),
            Self::Io(error) => write!(f, "renderer IPC I/O failed: {error}"),
            Self::InvalidFrameLength(size) => write!(
                f,
                "renderer IPC frame length {size} exceeds the {MAX_RENDERER_IPC_FRAME_SIZE}-byte bound"
            ),
            Self::InvalidVersion(version) => {
                write!(f, "unsupported renderer IPC protocol version {version}")
            }
            Self::Capability(error) => write!(f, "renderer capability admission failed: {error}"),
            Self::SocketPermissions(error) => {
                write!(f, "renderer IPC socket permission hardening failed: {error}")
            }
        }
    }
}

impl std::error::Error for RendererIpcError {}

impl From<CapabilityIngressError> for RendererIpcError {
    fn from(error: CapabilityIngressError) -> Self {
        Self::Capability(error)
    }
}

impl RendererIpcListener {
    /// Bind a private filesystem Unix socket for renderer capability IPC.
    ///
    /// Existing paths are never replaced implicitly. Callers must perform
    /// explicit stale-socket cleanup as part of browser startup/recovery.
    pub fn bind(path: impl Into<PathBuf>) -> Result<Self, RendererIpcError> {
        let socket_path = path.into();
        if !socket_path.is_absolute() {
            return Err(RendererIpcError::Bind(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "renderer IPC socket path must be absolute",
            )));
        }
        if socket_path.exists() {
            return Err(RendererIpcError::Bind(std::io::Error::new(
                std::io::ErrorKind::AlreadyExists,
                "refusing to replace existing renderer IPC socket",
            )));
        }

        let listener = UnixListener::bind(&socket_path).map_err(RendererIpcError::Bind)?;

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(
                &socket_path,
                std::fs::Permissions::from_mode(0o600),
            )
            .map_err(RendererIpcError::SocketPermissions)?;
        }

        Ok(Self {
            listener,
            socket_path,
        })
    }

    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// Accept exactly one renderer connection and bind it to the browser's
    /// authoritative process/generation.
    pub async fn accept(
        &self,
        authoritative: RendererBinding,
    ) -> Result<RendererCapabilityConnection, RendererIpcError> {
        let (stream, _) = self.listener.accept().await.map_err(RendererIpcError::Io)?;
        RendererCapabilityConnection::from_stream(stream, authoritative)
    }
}

/// One live browser ↔ renderer capability connection.
///
/// The connection owns the session state; callers cannot provide a peer ID to
/// authorize requests. Every request is re-bound to the live Unix peer.
#[derive(Debug)]
pub struct RendererCapabilityConnection {
    stream: UnixStream,
    authoritative: RendererBinding,
    ingress: CapabilityIngress,
}

impl RendererCapabilityConnection {
    pub fn from_stream(
        stream: UnixStream,
        authoritative: RendererBinding,
    ) -> Result<Self, RendererIpcError> {
        let process = authoritative.process;
        let generation = authoritative.generation;
        let mut sessions = RendererSessionManager::new();
        sessions
            .establish_from_stream(&stream, process, generation)
            .map_err(|error| RendererIpcError::Capability(
                CapabilityIngressError::Session(error),
            ))?;

        Ok(Self {
            stream,
            authoritative,
            ingress: CapabilityIngress::new(sessions),
        })
    }

    pub fn authoritative(&self) -> &RendererBinding {
        &self.authoritative
    }

    /// Receive one bounded capability frame and pass it through the sole
    /// capability ingress.
    pub async fn receive_request(&mut self) -> Result<CapabilityRequest, RendererIpcError> {
        let payload = read_bounded_frame(&mut self.stream).await?;
        self.ingress
            .admit(
                &self.stream,
                self.authoritative.generation,
                &self.authoritative,
                &payload,
            )
            .map_err(Into::into)
    }

    /// Close the session and invalidate all session-local sequencing state.
    pub fn close(&mut self) {
        self.ingress.close();
    }
}

async fn read_bounded_frame(stream: &mut UnixStream) -> Result<Vec<u8>, RendererIpcError> {
    let mut length = [0u8; 4];
    stream
        .read_exact(&mut length)
        .await
        .map_err(RendererIpcError::Io)?;

    let size = u32::from_be_bytes(length) as usize;
    if size == 0 || size > MAX_RENDERER_IPC_FRAME_SIZE {
        return Err(RendererIpcError::InvalidFrameLength(size));
    }

    let mut payload = vec![0u8; size];
    stream
        .read_exact(&mut payload)
        .await
        .map_err(RendererIpcError::Io)?;

    // Version is intentionally the first field of the dedicated envelope.
    // Decode only the bounded transport envelope here; privileged policy
    // remains in CapabilityIngress.
    let envelope: RendererIpcEnvelopeV1 =
        rmp_serde::from_slice(&payload).map_err(|error| {
            RendererIpcError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("invalid renderer IPC envelope: {error}"),
            ))
        })?;

    if envelope.version != RENDERER_IPC_VERSION {
        return Err(RendererIpcError::InvalidVersion(envelope.version));
    }
    if envelope.payload.is_empty() || envelope.payload.len() > MAX_RENDERER_IPC_FRAME_SIZE {
        return Err(RendererIpcError::InvalidFrameLength(envelope.payload.len()));
    }

    Ok(envelope.payload)
}

/// Versioned transport envelope. The body remains an opaque, bounded
/// capability payload until it reaches CapabilityIngress.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RendererIpcEnvelopeV1 {
    pub version: u16,
    pub payload: Vec<u8>,
}

impl RendererIpcEnvelopeV1 {
    pub fn new(payload: Vec<u8>) -> Result<Self, RendererIpcError> {
        if payload.is_empty() || payload.len() > MAX_RENDERER_IPC_FRAME_SIZE {
            return Err(RendererIpcError::InvalidFrameLength(payload.len()));
        }
        Ok(Self {
            version: RENDERER_IPC_VERSION,
            payload,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::{Capability, CapabilityResource, OriginBinding, RequestId};
    use crate::identity::{AgentClusterId, RendererProcessId, SiteIdentity};
    use tokio::io::AsyncWriteExt;
    use tokio::net::UnixStream;

    fn binding(process: RendererProcessId, generation: u64) -> RendererBinding {
        RendererBinding {
            process,
            site: SiteIdentity::new("https://example.com").unwrap(),
            origin: OriginBinding::new("https://example.com").unwrap(),
            agent_cluster: AgentClusterId::new(1).unwrap(),
            generation,
        }
    }

    fn request_payload(id: u64) -> Vec<u8> {
        rmp_serde::to_vec(&CapabilityRequest {
            request_id: RequestId::new(id).unwrap(),
            capability: Capability::NetworkFetch,
            origin: OriginBinding::new("https://example.com").unwrap(),
            resource: Some(CapabilityResource::new("https://example.com/data").unwrap()),
        })
        .unwrap()
    }

    #[test]
    fn envelope_is_versioned_and_bounded() {
        let payload = request_payload(1);
        let envelope = RendererIpcEnvelopeV1::new(payload.clone()).unwrap();
        assert_eq!(envelope.version, RENDERER_IPC_VERSION);
        assert_eq!(envelope.payload, payload);
        assert!(RendererIpcEnvelopeV1::new(vec![0; MAX_RENDERER_IPC_FRAME_SIZE + 1]).is_err());
    }

    #[tokio::test]
    async fn malformed_or_oversized_frame_is_rejected_before_capability_policy() {
        let (mut writer, reader) = UnixStream::pair().unwrap();
        writer.write_all(&(0u32).to_be_bytes()).await.unwrap();

        let mut connection = RendererCapabilityConnection::from_stream(
            reader,
            binding(
                RendererProcessId::new(std::process::id() as u64).unwrap(),
                1,
            ),
        )
        .unwrap();

        assert!(matches!(
            connection.receive_request().await,
            Err(RendererIpcError::InvalidFrameLength(0))
        ));
    }

    #[tokio::test]
    async fn valid_capability_request_crosses_only_dedicated_renderer_path() {
        let (mut writer, reader) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let request = request_payload(1);
        let envelope = rmp_serde::to_vec(&RendererIpcEnvelopeV1::new(request).unwrap()).unwrap();
        writer
            .write_all(&(envelope.len() as u32).to_be_bytes())
            .await
            .unwrap();
        writer.write_all(&envelope).await.unwrap();

        let mut connection =
            RendererCapabilityConnection::from_stream(reader, binding(process, 1)).unwrap();
        let admitted = connection.receive_request().await.unwrap();
        assert_eq!(admitted.request_id, RequestId::new(1).unwrap());
    }

    #[tokio::test]
    async fn session_close_invalidates_capability_ingress() {
        let (mut writer, reader) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let request = request_payload(1);
        let envelope = rmp_serde::to_vec(&RendererIpcEnvelopeV1::new(request).unwrap()).unwrap();
        writer
            .write_all(&(envelope.len() as u32).to_be_bytes())
            .await
            .unwrap();
        writer.write_all(&envelope).await.unwrap();

        let mut connection =
            RendererCapabilityConnection::from_stream(reader, binding(process, 2)).unwrap();
        connection.close();

        assert!(matches!(
            connection.receive_request().await,
            Err(RendererIpcError::Capability(
                CapabilityIngressError::Session(_)
            ))
        ));
    }
}
