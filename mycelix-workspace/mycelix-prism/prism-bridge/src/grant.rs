//! Broker-owned capability grants.
//!
//! Authentication tells the broker which renderer sent a request.
//! Authorization is a separate state machine: the broker binds a grant to an
//! authoritative process, site, origin, capability, resource scope, and
//! expiration. Renderer-supplied identity claims are never sufficient.

use crate::capability::{
    Capability, CapabilityRequest, CapabilityResponse, DenialReason, OriginBinding,
};
use crate::identity::{AgentClusterId, RendererProcessId, SiteIdentity};
use crate::resource::{ResourceIdentity, ResourceScopeV1};

pub const MAX_ACTIVE_GRANTS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResourceScope {
    Any,
    /// Transitional compatibility scope. Never use this variant for new
    /// privileged authority; it exists only while older broker callers are
    /// migrated to typed resource identities.
    LegacyPrefix(String),
    /// Parsed/canonical broker-owned resource scope. Sensitive authorization
    /// paths must use this variant rather than comparing renderer strings.
    Typed(ResourceScopeV1),
}

impl ResourceScope {
    pub fn allows(&self, resource: Option<&str>) -> bool {
        match self {
            Self::Any => true,
            Self::LegacyPrefix(prefix) => resource.is_some_and(|value| {
                let Some(suffix) = value.strip_prefix(prefix) else {
                    return false;
                };
                // A raw string prefix is not a URL boundary: for example,
                // "https://cdn.example.com.attacker.invalid" must not match
                // "https://cdn.example.com". Permit exact matches and
                // hierarchical URL delimiters only. A prefix ending in '/'
                // explicitly denotes a path subtree.
                suffix.is_empty()
                    || prefix.ends_with('/')
                    || suffix.starts_with('/')
                    || suffix.starts_with('?')
                    || suffix.starts_with('#')
            }),
            Self::Typed(scope) => resource
                .and_then(|value| ResourceIdentity::parse_url(value).ok())
                .is_some_and(|identity| scope.allows(&identity)),
        }
    }

    /// Authorize an already-parsed broker-owned identity. Legacy string
    /// scopes intentionally cannot authorize through this path.
    pub fn allows_identity(&self, resource: Option<&ResourceIdentity>) -> bool {
        match self {
            Self::Any => true,
            Self::LegacyPrefix(_) => false,
            Self::Typed(scope) => resource.is_some_and(|identity| scope.allows(identity)),
        }
    }
}

/// Broker-owned authorization state. issued_at_ms and expires_at_ms are
/// monotonic policy timestamps supplied by the broker's trusted clock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapabilityGrant {
    pub grant_id: u128,
    pub renderer_process: RendererProcessId,
    pub site: SiteIdentity,
    pub origin: OriginBinding,
    pub agent_cluster: AgentClusterId,
    pub capability: Capability,
    pub resource_scope: ResourceScope,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
}

impl CapabilityGrant {
    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.expires_at_ms > self.issued_at_ms
            && now_ms >= self.issued_at_ms
            && now_ms < self.expires_at_ms
    }

    pub fn authorizes(
        &self,
        renderer_process: RendererProcessId,
        site: &SiteIdentity,
        origin: &OriginBinding,
        agent_cluster: AgentClusterId,
        request: &CapabilityRequest,
        now_ms: u64,
    ) -> Result<(), DenialReason> {
        if !self.is_active_at(now_ms) {
            return Err(DenialReason::CapabilityNotGranted);
        }
        if self.renderer_process != renderer_process || self.site != *site {
            return Err(DenialReason::OriginMismatch);
        }
        if self.agent_cluster != agent_cluster
            || self.origin != *origin
            || request.origin != *origin
        {
            return Err(DenialReason::OriginMismatch);
        }
        if self.capability != request.capability {
            return Err(DenialReason::CapabilityNotGranted);
        }
        let resource = request.resource.as_ref().map(|r| r.as_str());
        if matches!(&self.resource_scope, ResourceScope::Typed(_)) {
            let Some(identity) = resource.and_then(|value| ResourceIdentity::parse_url(value).ok()) else {
                return Err(DenialReason::ResourcePolicyDenied);
            };
            if !self.resource_scope.allows_identity(Some(&identity)) {
                return Err(DenialReason::ResourcePolicyDenied);
            }
        } else if !self.resource_scope.allows(resource) {
            return Err(DenialReason::ResourcePolicyDenied);
        }
        Ok(())
    }
}

/// Minimal deterministic broker authorization state. The process/site/origin
/// arguments are authoritative browser state; they are not taken from the
/// renderer request.
#[derive(Debug, Default)]
pub struct CapabilityGrantStore {
    grants: Vec<CapabilityGrant>,
}

impl CapabilityGrantStore {
    pub fn insert(&mut self, grant: CapabilityGrant) -> Result<(), GrantError> {
        if self.grants.len() >= MAX_ACTIVE_GRANTS {
            return Err(GrantError::CapacityExceeded);
        }
        if grant.grant_id == 0 || grant.expires_at_ms <= grant.issued_at_ms {
            return Err(GrantError::InvalidGrant);
        }
        if self.grants.iter().any(|existing| existing.grant_id == grant.grant_id) {
            return Err(GrantError::DuplicateGrant);
        }
        self.grants.push(grant);
        Ok(())
    }

    pub fn revoke(&mut self, grant_id: u128) -> bool {
        let before = self.grants.len();
        self.grants.retain(|grant| grant.grant_id != grant_id);
        before != self.grants.len()
    }

    /// Revoke every grant during a committed navigation or renderer teardown.
    /// The broker must call this only after updating its authoritative binding.
    pub fn revoke_all(&mut self) -> usize {
        let revoked = self.grants.len();
        self.grants.clear();
        revoked
    }

    pub fn authorize(
        &self,
        renderer_process: RendererProcessId,
        site: &SiteIdentity,
        origin: &OriginBinding,
        agent_cluster: AgentClusterId,
        request: &CapabilityRequest,
        now_ms: u64,
    ) -> CapabilityResponse {
        let authorized = self.grants.iter().any(|grant| {
            grant
                .authorizes(
                    renderer_process,
                    site,
                    origin,
                    agent_cluster,
                    request,
                    now_ms,
                )
                .is_ok()
        });

        if authorized {
            CapabilityResponse::Allowed {
                request_id: request.request_id,
                capability: request.capability,
            }
        } else {
            CapabilityResponse::Denied {
                request_id: request.request_id,
                reason: DenialReason::CapabilityNotGranted,
            }
        }
    }

    pub fn len(&self) -> usize {
        self.grants.len()
    }

    pub fn is_empty(&self) -> bool {
        self.grants.is_empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrantError {
    CapacityExceeded,
    InvalidGrant,
    DuplicateGrant,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::{CapabilityResource, RequestId};
    use crate::identity::{AgentClusterId, RendererProcessId, SiteIdentity};

    fn fixture() -> (
        RendererProcessId,
        SiteIdentity,
        OriginBinding,
        AgentClusterId,
        CapabilityRequest,
        CapabilityGrant,
    ) {
        let process = RendererProcessId::new(7).unwrap();
        let site = SiteIdentity::new("https://example.com").unwrap();
        let origin = OriginBinding::new("https://example.com").unwrap();
        let cluster = AgentClusterId::new(9).unwrap();
        let request = CapabilityRequest {
            request_id: RequestId::new(1).unwrap(),
            capability: Capability::NetworkFetch,
            origin: origin.clone(),
            resource: Some(CapabilityResource::new("https://cdn.example.com/a.js").unwrap()),
        };
        let grant = CapabilityGrant {
            grant_id: 11,
            renderer_process: process,
            site: site.clone(),
            origin: origin.clone(),
            agent_cluster: cluster,
            capability: Capability::NetworkFetch,
            resource_scope: ResourceScope::Typed(
                ResourceScopeV1::path_prefix("https://cdn.example.com", "/").unwrap(),
            ),
            issued_at_ms: 100,
            expires_at_ms: 200,
        };
        (process, site, origin, cluster, request, grant)
    }

    #[test]
    fn matching_authoritative_binding_allows() {
        let (process, site, origin, cluster, request, grant) = fixture();
        assert!(grant.authorizes(process, &site, &origin, cluster, &request, 150).is_ok());
    }

    #[test]
    fn renderer_origin_claim_cannot_override_authoritative_binding() {
        let (process, site, _origin, cluster, mut request, grant) = fixture();
        request.origin = OriginBinding::new("https://attacker.example").unwrap();
        let response = CapabilityGrantStore {
            grants: vec![grant],
        }
        .authorize(
            process,
            &site,
            &OriginBinding::new("https://example.com").unwrap(),
            cluster,
            &request,
            150,
        );
        assert!(matches!(
            response,
            CapabilityResponse::Denied {
                reason: DenialReason::CapabilityNotGranted,
                ..
            }
        ));
    }

    #[test]
    fn wrong_process_is_denied() {
        let (_, site, origin, cluster, request, grant) = fixture();
        let wrong_process = RendererProcessId::new(8).unwrap();
        assert_eq!(
            grant.authorizes(wrong_process, &site, &origin, cluster, &request, 150),
            Err(DenialReason::OriginMismatch)
        );
    }

    #[test]
    fn expired_grant_is_denied() {
        let (process, site, origin, cluster, request, grant) = fixture();
        assert_eq!(
            grant.authorizes(process, &site, &origin, cluster, &request, 200),
            Err(DenialReason::CapabilityNotGranted)
        );
    }

    #[test]
    fn prefix_scope_respects_url_authority_and_path_boundaries() {
        let scope = ResourceScope::LegacyPrefix("https://cdn.example.com".into());
        assert!(scope.allows(Some("https://cdn.example.com")));
        assert!(scope.allows(Some("https://cdn.example.com/app.js")));
        assert!(scope.allows(Some("https://cdn.example.com?version=1")));
        assert!(scope.allows(Some("https://cdn.example.com#fragment")));
        assert!(!scope.allows(Some("https://cdn.example.com.attacker.invalid/app.js")));
        assert!(!scope.allows(Some("https://cdn.example.com.evil")));
        assert!(!scope.allows(Some("http://cdn.example.com/app.js")));

        let subtree = ResourceScope::LegacyPrefix("https://cdn.example.com/assets/".into());
        assert!(subtree.allows(Some("https://cdn.example.com/assets/app.js")));
        assert!(subtree.allows(Some("https://cdn.example.com/assets/../private")));
    }

    #[test]
    fn resource_scope_is_enforced() {
        let (process, site, origin, cluster, request, mut grant) = fixture();
        grant.resource_scope = ResourceScope::Typed(
            ResourceScopeV1::exact_url("https://cdn.example.com/other.js").unwrap(),
        );
        assert_eq!(
            grant.authorizes(process, &site, &origin, cluster, &request, 150),
            Err(DenialReason::ResourcePolicyDenied)
        );
    }

    #[test]
    fn typed_resource_scope_rejects_malformed_renderer_resource() {
        let (process, site, origin, cluster, mut request, mut grant) = fixture();
        grant.resource_scope = ResourceScope::Typed(
            ResourceScopeV1::origin("https://cdn.example.com").unwrap(),
        );
        request.resource = Some(CapabilityResource::new(
            "https://cdn.example.com.attacker.invalid/a.js",
        ).unwrap());
        assert_eq!(
            grant.authorizes(process, &site, &origin, cluster, &request, 150),
            Err(DenialReason::ResourcePolicyDenied)
        );

        request.resource = Some(CapabilityResource::new(
            "https://cdn.example.com/assets/../private.js",
        ).unwrap());
        assert_eq!(
            grant.authorizes(process, &site, &origin, cluster, &request, 150),
            Err(DenialReason::ResourcePolicyDenied)
        );
    }

    #[test]
    fn typed_scope_cannot_be_bypassed_by_legacy_prefix_semantics() {
        let (process, site, origin, cluster, request, mut grant) = fixture();
        grant.resource_scope =
            ResourceScope::Typed(ResourceScopeV1::host("https://cdn.example.com").unwrap());
        let mut forged = request.clone();
        forged.resource = Some(
            CapabilityResource::new("https://cdn.example.com.attacker.invalid/a.js").unwrap(),
        );
        assert_eq!(
            grant.authorizes(process, &site, &origin, cluster, &forged, 150),
            Err(DenialReason::ResourcePolicyDenied)
        );
    }

    #[test]
    fn duplicate_grants_are_rejected() {
        let (_, _, _, _, _, grant) = fixture();
        let mut store = CapabilityGrantStore::default();
        store.insert(grant.clone()).unwrap();
        assert_eq!(store.insert(grant), Err(GrantError::DuplicateGrant));
        assert_eq!(store.len(), 1);
    }

    #[test]
    fn revoke_all_clears_every_grant() {
        let (_, site, origin, cluster, _, mut grant) = fixture();
        let mut store = CapabilityGrantStore::default();
        store.insert(grant.clone()).unwrap();
        grant.grant_id = 12;
        grant.site = site;
        grant.origin = origin;
        grant.agent_cluster = cluster;
        store.insert(grant).unwrap();

        assert_eq!(store.revoke_all(), 2);
        assert!(store.is_empty());
        assert_eq!(store.revoke_all(), 0);
    }

    #[test]
    fn revocation_is_explicit() {
        let (_, _, _, _, _, grant) = fixture();
        let mut store = CapabilityGrantStore::default();
        store.insert(grant).unwrap();
        assert!(store.revoke(11));
        assert!(store.is_empty());
        assert!(!store.revoke(11));
    }
}
