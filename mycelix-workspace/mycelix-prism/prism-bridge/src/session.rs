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
    AuthenticatedRendererSession, CapabilityError, RendererPeerId, RendererSessionId,
    RequestId,
};
use crate::identity::RendererProcessId;
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
        process: RendererProcessId::new(pid)
            .map_err(RendererSessionManagerError::Session)?,
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

    pub fn accept_request(
        &mut self,
        peer_id: RendererPeerId,
        generation: u64,
        request_id: RequestId,
    ) -> Result<(), RendererSessionManagerError> {
        let session = self
            .current
            .as_mut()
            .ok_or(RendererSessionManagerError::Session(
                CapabilityError::SessionClosed,
            ))?;
        session.accept_request(peer_id, generation, request_id)?;
        Ok(())
    }

    pub fn close(&mut self) {
        if let Some(session) = self.current.as_mut() {
            session.close();
        }
        self.current = None;
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
    use tokio::net::UnixStream;

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
    async fn manager_binds_requests_to_authenticated_peer_and_generation() {
        let (left, right) = UnixStream::pair().unwrap();
        let mut manager = RendererSessionManager::new();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        manager.establish_from_stream(&left, process, 11).unwrap();

        let peer = manager.current().unwrap().peer_id;
        manager.accept_request(peer, 11, RequestId::new(1).unwrap()).unwrap();
        assert!(matches!(
            manager.accept_request(peer, 10, RequestId::new(2).unwrap()),
            Err(RendererSessionManagerError::Session(
                CapabilityError::GenerationMismatch
            ))
        ));
        assert!(matches!(
            manager.accept_request(RendererPeerId::new(peer.0 + 1).unwrap(), 11, RequestId::new(2).unwrap()),
            Err(RendererSessionManagerError::Session(CapabilityError::PeerMismatch))
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
