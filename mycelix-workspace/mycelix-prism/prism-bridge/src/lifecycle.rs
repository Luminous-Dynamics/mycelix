//! Authoritative renderer lifecycle and navigation-bound grant invalidation.
//!
//! This is a deterministic state contract, not an OS process supervisor. The
//! privileged browser controller supplies committed identities; renderer
//! messages cannot construct or replace this binding.

use crate::capability::OriginBinding;
use crate::grant::CapabilityGrantStore;
use crate::identity::{AgentClusterId, RendererProcessId, SiteIdentity};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RendererBinding {
    pub process: RendererProcessId,
    pub site: SiteIdentity,
    pub origin: OriginBinding,
    pub agent_cluster: AgentClusterId,
    /// Incremented on every committed navigation and teardown transition.
    pub generation: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleError {
    GenerationExhausted,
}

#[derive(Debug, Default)]
pub struct RendererLifecycle {
    current: Option<RendererBinding>,
    generation: u64,
}

impl RendererLifecycle {
    pub fn current(&self) -> Option<&RendererBinding> {
        self.current.as_ref()
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Install the browser-authoritative identity after navigation commit.
    /// Existing grants are invalidated before the new binding is published.
    pub fn commit_navigation(
        &mut self,
        process: RendererProcessId,
        site: SiteIdentity,
        origin: OriginBinding,
        agent_cluster: AgentClusterId,
        grants: &mut CapabilityGrantStore,
    ) -> Result<&RendererBinding, LifecycleError> {
        let next = self.generation.checked_add(1).ok_or(LifecycleError::GenerationExhausted)?;
        grants.revoke_all();
        self.generation = next;
        self.current = Some(RendererBinding {
            process,
            site,
            origin,
            agent_cluster,
            generation: next,
        });
        Ok(self.current.as_ref().expect("binding was just installed"))
    }

    /// Invalidate all renderer authority on teardown or tab destruction.
    pub fn clear(
        &mut self,
        grants: &mut CapabilityGrantStore,
    ) -> Result<usize, LifecycleError> {
        let next = self.generation.checked_add(1).ok_or(LifecycleError::GenerationExhausted)?;
        let revoked = grants.revoke_all();
        self.current = None;
        self.generation = next;
        Ok(revoked)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::{Capability, CapabilityRequest, CapabilityResource, RequestId};
    use crate::grant::{CapabilityGrant, ResourceScope};

    fn identity(host: &str, process_id: u64, cluster_id: u128) -> (RendererProcessId, SiteIdentity, OriginBinding, AgentClusterId) {
        (
            RendererProcessId::new(process_id).unwrap(),
            SiteIdentity::new(format!("https://{host}")).unwrap(),
            OriginBinding::new(format!("https://{host}")).unwrap(),
            AgentClusterId::new(cluster_id).unwrap(),
        )
    }

    fn grant(process: RendererProcessId, site: SiteIdentity, origin: OriginBinding, cluster: AgentClusterId, id: u128) -> CapabilityGrant {
        CapabilityGrant {
            grant_id: id,
            renderer_process: process,
            site,
            origin,
            agent_cluster: cluster,
            capability: Capability::NetworkFetch,
            resource_scope: ResourceScope::Any,
            issued_at_ms: 1,
            expires_at_ms: 100,
        }
    }

    #[test]
    fn committed_navigation_replaces_binding_and_revokes_old_grants() {
        let mut lifecycle = RendererLifecycle::default();
        let mut grants = CapabilityGrantStore::default();
        let (p1, s1, o1, a1) = identity("attacker.example", 1, 10);
        grants.insert(grant(p1, s1.clone(), o1.clone(), a1, 1)).unwrap();

        let first = lifecycle.commit_navigation(p1, s1, o1, a1, &mut grants).unwrap().clone();
        assert_eq!(first.generation, 1);
        assert!(grants.is_empty());

        let (p2, s2, o2, a2) = identity("victim.example", 2, 20);
        grants.insert(grant(p2, s2.clone(), o2.clone(), a2, 2)).unwrap();
        let second = lifecycle.commit_navigation(p2, s2.clone(), o2.clone(), a2, &mut grants).unwrap();
        assert_eq!(second.generation, 2);
        assert_eq!(second.origin, o2);
        assert_eq!(second.site, s2);
        assert!(grants.is_empty());
    }

    #[test]
    fn teardown_clears_binding_and_revokes_grants() {
        let mut lifecycle = RendererLifecycle::default();
        let mut grants = CapabilityGrantStore::default();
        let (process, site, origin, cluster) = identity("example.com", 7, 9);
        grants.insert(grant(process, site.clone(), origin.clone(), cluster, 7)).unwrap();
        lifecycle.commit_navigation(process, site, origin, cluster, &mut grants).unwrap();

        assert_eq!(lifecycle.clear(&mut grants).unwrap(), 0);
        assert!(lifecycle.current().is_none());
        assert_eq!(lifecycle.generation(), 2);
        assert!(grants.is_empty());
    }

    #[test]
    fn binding_is_browser_supplied_not_request_derived() {
        let mut lifecycle = RendererLifecycle::default();
        let mut grants = CapabilityGrantStore::default();
        let (process, site, origin, cluster) = identity("trusted.example", 4, 44);
        lifecycle.commit_navigation(process, site, origin.clone(), cluster, &mut grants).unwrap();

        let forged = CapabilityRequest {
            request_id: RequestId::new(1).unwrap(),
            capability: Capability::NetworkFetch,
            origin: OriginBinding::new("https://attacker.example").unwrap(),
            resource: Some(CapabilityResource::new("https://attacker.example/x").unwrap()),
        };
        assert_ne!(forged.origin, lifecycle.current().unwrap().origin);
    }
}
