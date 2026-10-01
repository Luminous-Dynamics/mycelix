//! Live renderer IPC session authentication and lifecycle binding.
//!
//! This module is the concrete bridge between the authenticated Unix transport
//! and the broker-side capability session contract. Renderer-provided process,
//! origin, site, and generation values are never used to establish authority.
//!
//! On Unix, Tokio exposes peer credentials for Unix sockets. Prism requires a
//! peer PID here and binds it to the browser-assigned RendererProcessId before
//! establishing an authenticated renderer session. Platforms that cannot
//! provide a peer PID fail closed rather than silently degrading to an
//! unauthenticated identity.

use crate::capability::{
    AuthenticatedRendererSession, CapabilityError, CapabilityRequest, RendererPeerId,
    RendererSessionId, RequestId,
};
use crate::identity::{IdentityError, RendererProcessId};
use crate::lifecycle::RendererBinding;
use std::fmt;
use tokio::net::UnixStream;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthenticatedPeer {
    pub peer_id: RendererPeerId,
    pub process: RendererProcessId,
}

#[derive(Debug)]
pub enum RendererSessionManagerError {
    PeerCredentials(std::io::Error),
    MissingPeerProcessId,
    PeerProcessMismatch {
        expected: RendererProcessId,
        observed: RendererProcessId,
    },
    Session(CapabilityError),
    Identity(IdentityError),
    SessionAlreadyActive,
}

impl fmt::Display for RendererSessionManagerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PeerCredentials(error) => write!(f, "could not authenticate renderer peer: {error}"),
            Self::MissingPeerProcessId => {
                f.write_str("renderer peer credentials did not expose a process id")
            }
            Self::PeerProcessMismatch { expected, observed } => write!(
                f,
                "renderer peer process mismatch: expected={}, observed={}",
                expected.0, observed.0
            ),
            Self::Session(error) => write!(f, "renderer session error: {error}"),
            Self::Identity(error) => write!(f, "renderer identity error: {error}"),
            Self::SessionAlreadyActive => f.write_str("renderer session is already active"),
        }
    }
}

impl std::error::Error for RendererSessionManagerError {}

impl From<CapabilityError> for RendererSessionManagerError {
    fn from(error: CapabilityError) -> Self {
        Self::Session(error)
    }
}

impl From<IdentityError> for RendererSessionManagerError {
    fn from(error: IdentityError) -> Self {
        Self::Identity(error)
    }
}

/// Extract and validate the operating-system identity of a connected peer.
///
/// The returned PID is an observation about the live IPC endpoint. It becomes
/// a Prism security principal only after the privileged browser controller
/// confirms that it is the expected RendererProcessId.
pub fn authenticate_peer(
    stream: &UnixStream,
) -> Result<AuthenticatedPeer, RendererSessionManagerError> {
    let credentials = stream
        .peer_cred()
        .map_err(RendererSessionManagerError::PeerCredentials)?;
    let pid = credentials
        .pid()
        .ok_or(RendererSessionManagerError::MissingPeerProcessId)?;
    if pid <= 0 {
        return Err(RendererSessionManagerError::MissingPeerProcessId);
    }
    let pid = pid as u64;
    Ok(AuthenticatedPeer {
        peer_id: RendererPeerId::new(pid)?,
        process: RendererProcessId::new(pid)?,
    })
}

/// Broker-owned manager for the one live renderer attachment represented by a
/// bridge connection.
///
/// A manager instance owns at most one active session. Navigation and teardown
/// should close the current session before grants are reused for a new
/// generation.
#[derive(Debug, Default)]
pub struct RendererSessionManager {
    current: Option<AuthenticatedRendererSession>,
}

impl RendererSessionManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn current(&self) -> Option<&AuthenticatedRendererSession> {
        self.current.as_ref()
    }

    pub fn current_session_id(&self) -> Option<RendererSessionId> {
        self.current.as_ref().map(|session| session.session_id)
    }

    pub fn establish_from_stream(
        &mut self,
        stream: &UnixStream,
        authoritative_process: RendererProcessId,
        generation: u64,
    ) -> Result<&AuthenticatedRendererSession, RendererSessionManagerError> {
        if self.current.as_ref().is_some_and(|session| session.is_active()) {
            return Err(RendererSessionManagerError::SessionAlreadyActive);
        }

        let peer = authenticate_peer(stream)?;
        if peer.process != authoritative_process {
            return Err(RendererSessionManagerError::PeerProcessMismatch {
                expected: authoritative_process,
                observed: peer.process,
            });
        }

        let session_id = fresh_session_id()?;
        self.current = Some(AuthenticatedRendererSession::establish(
            session_id,
            peer.peer_id,
            peer.process,
            generation,
        ));
        Ok(self.current.as_ref().expect("session was just established"))
    }

    /// Authenticate the live IPC endpoint again before advancing session state.
    /// The caller never supplies the peer identity as an authority value.
    pub fn accept_request(
        &mut self,
        stream: &UnixStream,
        generation: u64,
        request_id: RequestId,
    ) -> Result<(), RendererSessionManagerError> {
        let session_id = self.current_session_id().ok_or(RendererSessionManagerError::Session(
            CapabilityError::SessionClosed,
        ))?;
        self.accept_request_for_session(stream, generation, session_id, request_id)
    }

    /// Admit a request only for the session that owns the renderer connection.
    /// This prevents an old socket from inheriting authority after the broker
    /// replaces the active session for the same OS process.
    pub fn accept_request_for_session(
        &mut self,
        stream: &UnixStream,
        generation: u64,
        expected_session: RendererSessionId,
        request_id: RequestId,
    ) -> Result<(), RendererSessionManagerError> {
        let peer = authenticate_peer(stream)?;
        let session = self
            .current
            .as_mut()
            .ok_or(RendererSessionManagerError::Session(
                CapabilityError::SessionClosed,
            ))?;
        if session.session_id != expected_session {
            return Err(RendererSessionManagerError::Session(
                CapabilityError::PeerMismatch,
            ));
        }
        session.accept_request(peer.peer_id, generation, request_id)?;
        Ok(())
    }

    pub fn close(&mut self) {
        if let Some(session) = self.current.as_mut() {
            session.close();
        }
        self.current = None;
    }

    pub fn close_if_current(&mut self, expected_session: RendererSessionId) -> bool {
        if self.current_session_id() != Some(expected_session) {
            return false;
        }
        self.close();
        true
    }
}

/// The only broker ingress that turns an authenticated renderer payload into a
/// capability request.
///
/// This is intentionally separate from generic MessagePack decoding. Decoding
/// proves only that bytes form a syntactically valid request. Admission first
/// re-authenticates the live IPC peer and checks session generation/request
/// sequencing, then binds the request to the browser-authoritative renderer
/// identity. Callers can pass the returned request to capability/grant policy;
/// they must not authorize directly from decoded transport bytes.
///
/// The renderer-supplied origin remains a claim and must equal the
/// browser-authoritative committed origin before the request is admitted.
#[derive(Debug)]
pub struct CapabilityIngress {
    sessions: RendererSessionManager,
}

#[derive(Debug)]
pub enum CapabilityIngressError {
    Decode(crate::BridgeError),
    Session(RendererSessionManagerError),
    OriginMismatch,
    ProcessMismatch,
    GenerationMismatch,
    RequestIdMismatch,
}

impl fmt::Display for CapabilityIngressError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Decode(error) => write!(f, "capability request decode failed: {error}"),
            Self::Session(error) => write!(f, "capability session admission failed: {error}"),
            Self::OriginMismatch => f.write_str("renderer origin does not match authoritative origin"),
            Self::ProcessMismatch => {
                f.write_str("renderer session process does not match authoritative renderer process")
            }
            Self::GenerationMismatch => {
                f.write_str("renderer session generation does not match authoritative generation")
            }
            Self::RequestIdMismatch => {
                f.write_str("renderer capability envelope request id does not match payload request id")
            }
        }
    }
}

impl std::error::Error for CapabilityIngressError {}

impl CapabilityIngress {
    pub fn new(sessions: RendererSessionManager) -> Self {
        Self { sessions }
    }

    pub fn sessions(&self) -> &RendererSessionManager {
        &self.sessions
    }

    pub fn sessions_mut(&mut self) -> &mut RendererSessionManager {
        &mut self.sessions
    }

    /// Admit one raw capability payload after live peer/session authentication.
    pub fn admit(
        &mut self,
        stream: &UnixStream,
        generation: u64,
        authoritative: &RendererBinding,
        payload: &[u8],
    ) -> Result<CapabilityRequest, CapabilityIngressError> {
        let request: CapabilityRequest =
            crate::decode_payload(payload).map_err(CapabilityIngressError::Decode)?;
        self.admit_decoded(stream, generation, authoritative, request.request_id, request)
    }

    /// Admit a versioned transport envelope. The envelope request ID is
    /// authenticated against the live peer/session before the capability body
    /// is decoded. This keeps expensive/complex capability decoding behind the
    /// authenticated session boundary.
    pub fn admit_envelope(
        &mut self,
        stream: &UnixStream,
        generation: u64,
        authoritative: &RendererBinding,
        session_id: RendererSessionId,
        request_id: RequestId,
        payload: &[u8],
    ) -> Result<CapabilityRequest, CapabilityIngressError> {
        self.sessions
            .accept_request_for_session(stream, generation, session_id, request_id)
            .map_err(CapabilityIngressError::Session)?;

        let request: CapabilityRequest =
            crate::decode_payload(payload).map_err(CapabilityIngressError::Decode)?;
        if request.request_id != request_id {
            return Err(CapabilityIngressError::RequestIdMismatch);
        }

        self.check_authoritative_binding(generation, authoritative, &request)?;
        Ok(request)
    }

    fn admit_decoded(
        &mut self,
        stream: &UnixStream,
        generation: u64,
        authoritative: &RendererBinding,
        request_id: RequestId,
        request: CapabilityRequest,
    ) -> Result<CapabilityRequest, CapabilityIngressError> {
        self.sessions
            .accept_request(stream, generation, request_id)
            .map_err(CapabilityIngressError::Session)?;
        self.check_authoritative_binding(generation, authoritative, &request)?;
        Ok(request)
    }

    fn check_authoritative_binding(
        &self,
        generation: u64,
        authoritative: &RendererBinding,
        request: &CapabilityRequest,
    ) -> Result<(), CapabilityIngressError> {
        let session = self
            .sessions
            .current()
            .expect("accepted request requires an active session");

        if session.renderer_process != authoritative.process {
            return Err(CapabilityIngressError::ProcessMismatch);
        }
        if generation != authoritative.generation {
            return Err(CapabilityIngressError::GenerationMismatch);
        }
        if request.origin != authoritative.origin {
            return Err(CapabilityIngressError::OriginMismatch);
        }

        Ok(())
    }

    pub fn close(&mut self) {
        self.sessions.close();
    }

    pub fn close_if_current(&mut self, session_id: RendererSessionId) -> bool {
        self.sessions.close_if_current(session_id)
    }
}

fn fresh_session_id() -> Result<RendererSessionId, RendererSessionManagerError> {
    let mut bytes = [0u8; 16];
    getrandom::fill(&mut bytes).map_err(|error| {
        RendererSessionManagerError::PeerCredentials(std::io::Error::other(format!(
            "random session id generation failed: {error}"
        )))
    })?;
    RendererSessionId::new(u128::from_be_bytes(bytes)).map_err(Into::into)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::{Capability, CapabilityResource, OriginBinding};
    use crate::identity::{AgentClusterId, SiteIdentity};
    use tokio::net::UnixStream;

    fn binding(host: &str, process: RendererProcessId, generation: u64) -> RendererBinding {
        RendererBinding {
            process,
            site: SiteIdentity::new(format!("https://{host}")).unwrap(),
            origin: OriginBinding::new(format!("https://{host}")).unwrap(),
            agent_cluster: AgentClusterId::new(1).unwrap(),
            generation,
        }
    }

    fn payload(request_id: u64, origin: &str) -> Vec<u8> {
        crate::encode_frame(&CapabilityRequest {
            request_id: RequestId::new(request_id).unwrap(),
            capability: Capability::NetworkFetch,
            origin: OriginBinding::new(origin).unwrap(),
            resource: Some(CapabilityResource::new("https://example.com/data").unwrap()),
        })
        .unwrap()[4..]
            .to_vec()
    }

    #[tokio::test]
    async fn authenticates_live_unix_peer_and_binds_browser_process() {
        let (left, right) = UnixStream::pair().unwrap();
        let peer = authenticate_peer(&left).unwrap();
        let expected = RendererProcessId::new(std::process::id() as u64).unwrap();
        assert_eq!(peer.process, expected);
        assert_eq!(peer.peer_id.0, expected.0);
        let _ = right;
    }

    #[tokio::test]
    async fn manager_rejects_authoritative_process_mismatch() {
        let (left, right) = UnixStream::pair().unwrap();
        let mut manager = RendererSessionManager::new();
        let wrong = RendererProcessId::new((std::process::id() as u64).saturating_add(1)).unwrap();
        assert!(matches!(
            manager.establish_from_stream(&left, wrong, 1),
            Err(RendererSessionManagerError::PeerProcessMismatch { .. })
        ));
        assert!(manager.current().is_none());
        let _ = right;
    }

    #[tokio::test]
    async fn manager_binds_requests_to_live_peer_and_generation() {
        let (left, right) = UnixStream::pair().unwrap();
        let mut manager = RendererSessionManager::new();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        manager.establish_from_stream(&left, process, 11).unwrap();

        manager.accept_request(&left, 11, RequestId::new(1).unwrap()).unwrap();
        assert!(matches!(
            manager.accept_request(&left, 10, RequestId::new(2).unwrap()),
            Err(RendererSessionManagerError::Session(
                CapabilityError::GenerationMismatch
            ))
        ));
        manager.accept_request(&left, 11, RequestId::new(2).unwrap()).unwrap();
        let _ = right;
    }

    #[tokio::test]
    async fn capability_ingress_rejects_forged_origin() {
        let (left, right) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let mut ingress = CapabilityIngress::new(RendererSessionManager::new());
        ingress
            .sessions_mut()
            .establish_from_stream(&left, process, 7)
            .unwrap();

        let request = ingress.admit(
            &left,
            7,
            &binding("victim.example", process, 7),
            &payload(1, "https://attacker.example"),
        );
        assert!(matches!(request, Err(CapabilityIngressError::OriginMismatch)));
        let _ = right;
    }

    #[tokio::test]
    async fn capability_ingress_rejects_stale_generation_before_policy() {
        let (left, right) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let mut ingress = CapabilityIngress::new(RendererSessionManager::new());
        ingress
            .sessions_mut()
            .establish_from_stream(&left, process, 8)
            .unwrap();

        let request = ingress.admit(
            &left,
            7,
            &binding("example.com", process, 7),
            &payload(1, "https://example.com"),
        );
        assert!(matches!(
            request,
            Err(CapabilityIngressError::Session(
                RendererSessionManagerError::Session(CapabilityError::GenerationMismatch)
            ))
        ));
        let _ = right;
    }

    #[tokio::test]
    async fn capability_ingress_rejects_request_replay() {
        let (left, right) = UnixStream::pair().unwrap();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let mut ingress = CapabilityIngress::new(RendererSessionManager::new());
        ingress
            .sessions_mut()
            .establish_from_stream(&left, process, 9)
            .unwrap();
        let authoritative = binding("example.com", process, 9);

        ingress
            .admit(&left, 9, &authoritative, &payload(1, "https://example.com"))
            .unwrap();
        let replay = ingress.admit(&left, 9, &authoritative, &payload(1, "https://example.com"));
        assert!(matches!(
            replay,
            Err(CapabilityIngressError::Session(
                RendererSessionManagerError::Session(CapabilityError::RequestIdReplay { .. })
            ))
        ));
        let _ = right;
    }

    #[tokio::test]
    async fn close_invalidates_live_session() {
        let (left, right) = UnixStream::pair().unwrap();
        let mut manager = RendererSessionManager::new();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        manager.establish_from_stream(&left, process, 3).unwrap();
        manager.close();
        assert!(manager.current().is_none());
        let _ = right;
    }
}
