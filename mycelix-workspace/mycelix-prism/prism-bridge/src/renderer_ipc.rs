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

use crate::capability::{CapabilityRequest, RequestId, RendererSessionId};
use crate::authority::{RendererSecurityController, RendererSecurityError};
use crate::lifecycle::RendererBinding;
use crate::session::CapabilityIngressError;
use std::sync::{Arc, Mutex};
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
    controller: Arc<Mutex<RendererSecurityController>>,
}

#[derive(Debug)]
pub enum RendererIpcError {
    Bind(std::io::Error),
    Io(std::io::Error),
    InvalidFrameLength(usize),
    InvalidVersion(u16),
    Capability(CapabilityIngressError),
    SecurityController(RendererSecurityError),
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
            Self::SecurityController(error) => write!(f, "renderer security controller rejected operation: {error:?}"),
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
    pub fn bind(
        path: impl Into<PathBuf>,
        controller: Arc<Mutex<RendererSecurityController>>,
    ) -> Result<Self, RendererIpcError> {
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
            controller,
        })
    }

    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// Accept exactly one renderer connection and bind it to the browser's
    /// authoritative process/generation.
    pub async fn accept(&self) -> Result<RendererCapabilityConnection, RendererIpcError> {
        let (stream, _) = self.listener.accept().await.map_err(RendererIpcError::Io)?;
        RendererCapabilityConnection::from_stream(stream, Arc::clone(&self.controller))
    }
}

/// One live browser ↔ renderer capability connection.
///
/// The connection owns the session state; callers cannot provide a peer ID to
/// authorize requests. Every request is re-bound to the live Unix peer.
#[derive(Debug)]
pub struct RendererCapabilityConnection {
    stream: UnixStream,
    controller: Arc<Mutex<RendererSecurityController>>,
    session_id: RendererSessionId,
}

impl RendererCapabilityConnection {
    pub fn from_stream(
        stream: UnixStream,
        controller: Arc<Mutex<RendererSecurityController>>,
    ) -> Result<Self, RendererIpcError> {
        {
            let mut guard = controller.lock().map_err(|_| RendererIpcError::SecurityController(
                RendererSecurityError::NoActiveBinding,
            ))?;
            let session_id = guard
                .establish_renderer_session(&stream)
                .map_err(RendererIpcError::SecurityController)?;
            return Ok(Self { stream, controller, session_id });
        }
    }

    pub fn authoritative(&self) -> Result<RendererBinding, RendererIpcError> {
        let guard = self.controller.lock().map_err(|_| RendererIpcError::SecurityController(
            RendererSecurityError::NoActiveBinding,
        ))?;
        guard.binding().cloned().ok_or(RendererIpcError::SecurityController(
            RendererSecurityError::NoActiveBinding,
        ))
    }

    /// Receive one bounded capability frame and pass it through the sole
    /// capability ingress.
    pub async fn receive_request(&mut self) -> Result<CapabilityRequest, RendererIpcError> {
        let payload = read_bounded_frame(&mut self.stream).await?;
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
        let mut guard = self.controller.lock().map_err(|_| RendererIpcError::SecurityController(
            RendererSecurityError::NoActiveBinding,
        ))?;
        let authoritative = guard.binding().cloned().ok_or(RendererIpcError::SecurityController(
            RendererSecurityError::NoActiveBinding,
        ))?;
        guard
            .ingress_mut()
            .admit_envelope(
                &self.stream,
                authoritative.generation,
                self.session_id,
                envelope.request_id,
                &envelope.payload,
            )
            .map_err(Into::into)
    }

    /// Close the session and invalidate all session-local sequencing state.
    pub fn close(&mut self) {
        if let Ok(mut guard) = self.controller.lock() {
            guard.ingress_mut().close_if_current(self.session_id);
        }
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

    Ok(payload)
}

/// Versioned transport envelope. The body remains an opaque, bounded
/// capability payload until it reaches CapabilityIngress.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RendererIpcEnvelopeV1 {
    pub version: u16,
    pub request_id: RequestId,
    pub payload: Vec<u8>,
}

impl RendererIpcEnvelopeV1 {
    pub fn new(request_id: RequestId, payload: Vec<u8>) -> Result<Self, RendererIpcError> {
        if payload.is_empty() || payload.len() > MAX_RENDERER_IPC_FRAME_SIZE {
            return Err(RendererIpcError::InvalidFrameLength(payload.len()));
        }
        Ok(Self {
            version: RENDERER_IPC_VERSION,
            request_id,
            payload,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::authority::RendererSecurityController;
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


    fn controller(process: RendererProcessId, generation: u64) -> Arc<Mutex<RendererSecurityController>> {
        let mut controller = RendererSecurityController::new();
        controller
            .commit_navigation(
                process,
                SiteIdentity::new("https://example.com").unwrap(),
                OriginBinding::new("https://example.com").unwrap(),
                AgentClusterId::new(1).unwrap(),
            )
            .unwrap();
        assert_eq!(controller.generation(), generation);
        Arc::new(Mutex::new(controller))
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
        let envelope = RendererIpcEnvelopeV1::new(RequestId::new(1).unwrap(), payload.clone()).unwrap();
        assert_eq!(envelope.version, RENDERER_IPC_VERSION);
        assert_eq!(envelope.payload, payload);
        assert!(RendererIpcEnvelopeV1::new(RequestId::new(1).unwrap(), vec![0; MAX_RENDERER_IPC_FRAME_SIZE + 1]).is_err());
    }

    #[tokio::test]
    async fn malformed_or_oversized_frame_is_rejected_before_capability_policy() {
        let (mut writer, reader) = UnixStream::pair().unwrap();
        writer.write_all(&(0u32).to_be_bytes()).await.unwrap();

        let mut connection = RendererCapabilityConnection::from_stream(
            reader,
            controller(RendererProcessId::new(std::process::id() as u64).unwrap(), 1),
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
        let envelope = rmp_serde::to_vec(&RendererIpcEnvelopeV1::new(RequestId::new(1).unwrap(), request).unwrap()).unwrap();
        writer
            .write_all(&(envelope.len() as u32).to_be_bytes())
            .await
            .unwrap();
        writer.write_all(&envelope).await.unwrap();

        let mut connection =
            RendererCapabilityConnection::from_stream(reader, controller(process, 1)).unwrap();
        let admitted = connection.receive_request().await.unwrap();
        assert_eq!(admitted.request_id, RequestId::new(1).unwrap());
    }

    #[tokio::test]
    async fn envelope_request_id_must_match_capability_payload() {
        let (mut writer, reader) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let envelope = rmp_serde::to_vec(
            &RendererIpcEnvelopeV1::new(RequestId::new(2).unwrap(), request_payload(1)).unwrap(),
        )
        .unwrap();
        writer
            .write_all(&(envelope.len() as u32).to_be_bytes())
            .await
            .unwrap();
        writer.write_all(&envelope).await.unwrap();

        let mut connection =
            RendererCapabilityConnection::from_stream(reader, binding(process, 1)).unwrap();
        assert!(matches!(
            connection.receive_request().await,
            Err(RendererIpcError::Capability(
                CapabilityIngressError::RequestIdMismatch
            ))
        ));
    }

    #[tokio::test]
    async fn stale_connection_cannot_inherit_or_close_replacement_session() {
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let shared = controller(process, 1);

        let (_writer1, reader1) = UnixStream::pair().unwrap();
        let mut first = RendererCapabilityConnection::from_stream(reader1, Arc::clone(&shared)).unwrap();

        {
            let mut guard = shared.lock().unwrap();
            guard.commit_navigation(
                process,
                SiteIdentity::new("https://example.com").unwrap(),
                OriginBinding::new("https://example.com").unwrap(),
                AgentClusterId::new(2).unwrap(),
            ).unwrap();
        }

        let (_writer2, reader2) = UnixStream::pair().unwrap();
        let second = RendererCapabilityConnection::from_stream(reader2, Arc::clone(&shared)).unwrap();

        first.close();

        let guard = shared.lock().unwrap();
        assert_eq!(guard.sessions().current().unwrap().session_id, second.session_id);
        drop(guard);
    }

    #[tokio::test]
    async fn session_close_invalidates_capability_ingress() {
        let (mut writer, reader) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let request = request_payload(1);
        let envelope = rmp_serde::to_vec(&RendererIpcEnvelopeV1::new(RequestId::new(1).unwrap(), request).unwrap()).unwrap();
        writer
            .write_all(&(envelope.len() as u32).to_be_bytes())
            .await
            .unwrap();
        writer.write_all(&envelope).await.unwrap();

        let mut connection =
            RendererCapabilityConnection::from_stream(reader, controller(process, 2)).unwrap();
        connection.close();

        assert!(matches!(
            connection.receive_request().await,
            Err(RendererIpcError::Capability(
                CapabilityIngressError::Session(_)
            ))
        ));
    }
}
