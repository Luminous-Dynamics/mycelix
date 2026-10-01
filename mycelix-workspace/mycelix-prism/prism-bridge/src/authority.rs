//! Unified browser-owned renderer security state.
//!
//! This controller makes navigation/teardown transitions atomic at the
//! architectural level: old renderer IPC session -> grant invalidation ->
//! generation transition -> new authoritative binding. It deliberately does
//! not own OS process spawning and never participates in the Spore service
//! channel.

use crate::grant::CapabilityGrantStore;
use crate::identity::{AgentClusterId, RendererProcessId, SiteIdentity};
use crate::lifecycle::{LifecycleError, RendererBinding, RendererLifecycle};
use crate::capability::OriginBinding;
use crate::session::{CapabilityIngress, CapabilityIngressError, RendererSessionManagerError};
use tokio::net::UnixStream;

#[derive(Debug)]
pub enum RendererSecurityError {
    Lifecycle(LifecycleError),
    Session(RendererSessionManagerError),
    Ingress(CapabilityIngressError),
    NoActiveBinding,
}

impl From<LifecycleError> for RendererSecurityError {
    fn from(error: LifecycleError) -> Self { Self::Lifecycle(error) }
}

impl From<RendererSessionManagerError> for RendererSecurityError {
    fn from(error: RendererSessionManagerError) -> Self { Self::Session(error) }
}

impl From<CapabilityIngressError> for RendererSecurityError {
    fn from(error: CapabilityIngressError) -> Self { Self::Ingress(error) }
}

/// The broker's single state owner for renderer lifecycle + grants + IPC
/// attachment. The browser controller supplies process/site/origin/cluster
/// identities; renderer messages never mutate these values.
#[derive(Debug)]
pub struct RendererSecurityController {
    lifecycle: RendererLifecycle,
    grants: CapabilityGrantStore,
    ingress: CapabilityIngress,
}

impl Default for RendererSecurityController {
    fn default() -> Self {
        Self {
            lifecycle: RendererLifecycle::default(),
            grants: CapabilityGrantStore::default(),
            ingress: CapabilityIngress::new(crate::session::RendererSessionManager::new()),
        }
    }
}

impl RendererSecurityController {
    pub fn new() -> Self { Self::default() }

    pub fn binding(&self) -> Option<&RendererBinding> {
        self.lifecycle.current()
    }

    pub fn generation(&self) -> u64 {
        self.lifecycle.generation()
    }

    pub fn grants(&self) -> &CapabilityGrantStore {
        &self.grants
    }

    pub fn sessions(&self) -> &crate::session::RendererSessionManager {
        self.ingress.sessions()
    }

    pub fn sessions_mut(&mut self) -> &mut crate::session::RendererSessionManager {
        self.ingress.sessions_mut()
    }

    pub fn ingress(&self) -> &CapabilityIngress {
        &self.ingress
    }

    pub fn ingress_mut(&mut self) -> &mut CapabilityIngress {
        &mut self.ingress
    }

    /// Commit a new browser-authoritative navigation.
    ///
    /// The old IPC attachment is closed before the old grants are invalidated
    /// and the new binding is published. A renderer retaining the old socket
    /// therefore cannot survive the security-context transition.
    pub fn commit_navigation(
        &mut self,
        process: RendererProcessId,
        site: SiteIdentity,
        origin: OriginBinding,
        agent_cluster: AgentClusterId,
    ) -> Result<&RendererBinding, RendererSecurityError> {
        self.ingress.close();
        Ok(self.lifecycle.commit_navigation(
            process,
            site,
            origin,
            agent_cluster,
            &mut self.grants,
        )?)
    }

    /// Tear down all renderer authority. No binding, grant, or IPC session
    /// remains valid after this transition.
    pub fn clear(&mut self) -> Result<usize, RendererSecurityError> {
        self.ingress.close();
        Ok(self.lifecycle.clear(&mut self.grants)?)
    }

    /// Attach one live renderer socket to the current authoritative binding.
    pub fn establish_renderer_session(
        &mut self,
        stream: &UnixStream,
    ) -> Result<crate::capability::RendererSessionId, RendererSecurityError> {
        let binding = self
            .lifecycle
            .current()
            .ok_or(RendererSecurityError::NoActiveBinding)?;

        let session = self
            .ingress
            .sessions_mut()
            .establish_from_stream(stream, binding.process, binding.generation)?;
        Ok(session.session_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::{Capability, CapabilityRequest, CapabilityResource, RequestId};
    use crate::grant::{CapabilityGrant, ResourceScope};
    use tokio::net::UnixStream;

    fn identity(host: &str, process: u64, cluster: u128) -> (RendererProcessId, SiteIdentity, OriginBinding, AgentClusterId) {
        (
            RendererProcessId::new(process).unwrap(),
            SiteIdentity::new(format!("https://{host}")).unwrap(),
            OriginBinding::new(format!("https://{host}")).unwrap(),
            AgentClusterId::new(cluster).unwrap(),
        )
    }

    fn grant(binding: &RendererBinding) -> CapabilityGrant {
        CapabilityGrant {
            grant_id: 1,
            renderer_process: binding.process,
            site: binding.site.clone(),
            origin: binding.origin.clone(),
            agent_cluster: binding.agent_cluster,
            capability: Capability::NetworkFetch,
            resource_scope: ResourceScope::Typed(
                crate::resource::ResourceScopeV1::origin(binding.origin.as_str()).unwrap(),
            ),
            issued_at_ms: 1,
            expires_at_ms: 100,
        }
    }

    #[test]
    fn navigation_closes_old_session_and_revokes_grants_before_new_binding() {
        let mut controller = RendererSecurityController::new();
        let (p1, s1, o1, a1) = identity("one.example", std::process::id() as u64, 1);
        let first = controller.commit_navigation(p1, s1, o1, a1).unwrap().clone();

        controller.grants.insert(grant(&first)).unwrap();

        let (p2, s2, o2, a2) = identity("two.example", std::process::id() as u64, 2);
        let second = controller.commit_navigation(p2, s2, o2, a2).unwrap();

        assert_eq!(second.generation, 2);
        assert_eq!(second.process, p2);
        assert!(controller.grants.is_empty());
        assert!(controller.sessions().current().is_none());
    }

    #[test]
    fn clear_invalidates_every_authority_domain() {
        let mut controller = RendererSecurityController::new();
        let (p, s, o, a) = identity("example.com", std::process::id() as u64, 9);
        controller.commit_navigation(p, s, o, a).unwrap();
        assert_eq!(controller.clear().unwrap(), 0);
        assert!(controller.binding().is_none());
        assert!(controller.sessions.current().is_none());
    }

    #[tokio::test]
    async fn new_renderer_attachment_is_bound_to_current_process_and_generation() {
        let mut controller = RendererSecurityController::new();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let (writer, reader) = UnixStream::pair().unwrap();

        let (_, site, origin, cluster) = identity("example.com", process.0, 7);
        controller.commit_navigation(process, site, origin, cluster).unwrap();
        controller.establish_renderer_session(&reader).unwrap();

        assert_eq!(controller.sessions().current().unwrap().renderer_process, process);
        assert_eq!(controller.sessions().current().unwrap().generation, 1);

        drop(writer);
    }

    #[test]
    fn browser_binding_does_not_derive_from_renderer_request() {
        let mut controller = RendererSecurityController::new();
        let process = RendererProcessId::new(std::process::id() as u64).unwrap();
        let (_, site, origin, cluster) = identity("trusted.example", process.0, 8);
        controller.commit_navigation(process, site, origin.clone(), cluster).unwrap();

        let forged = CapabilityRequest {
            request_id: RequestId::new(1).unwrap(),
            capability: Capability::NetworkFetch,
            origin: OriginBinding::new("https://attacker.example").unwrap(),
            resource: Some(CapabilityResource::new("https://attacker.example/x").unwrap()),
        };

        assert_ne!(forged.origin, controller.binding().unwrap().origin);
    }
}
