//! Broker-owned resource identities and scopes.
//!
//! Renderer resource strings are untrusted input. This module provides the
//! next authority boundary: parse and restrict a resource before applying a
//! capability scope. It deliberately does not perform network I/O or target
//! admission.

use url::Url;

pub const MAX_RESOURCE_URL_LEN: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResourceIdentity {
    Url(Url),
}

impl ResourceIdentity {
    pub fn parse_url(value: &str) -> Result<Self, ResourceError> {
        if value.is_empty() || value.len() > MAX_RESOURCE_URL_LEN {
            return Err(ResourceError::InvalidUrl);
        }
        let url = Url::parse(value).map_err(|_| ResourceError::InvalidUrl)?;
        if !matches!(url.scheme(), "http" | "https") {
            return Err(ResourceError::UnsupportedScheme);
        }
        if url.username() != "" || url.password().is_some() {
            return Err(ResourceError::UserinfoNotAllowed);
        }
        if url.host_str().is_none() {
            return Err(ResourceError::MissingHost);
        }
        Ok(Self::Url(url))
    }

    pub fn as_url(&self) -> &Url {
        match self {
            Self::Url(url) => url,
        }
    }

    pub fn origin(&self) -> String {
        self.as_url().origin().ascii_serialization()
    }

    pub fn host(&self) -> Option<&str> {
        self.as_url().host_str()
    }

    pub fn path(&self) -> &str {
        self.as_url().path()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResourceScopeV1 {
    ExactUrl(String),
    Origin(String),
    Host { origin: String, host: String },
    PathPrefix { origin: String, path: String },
}

impl ResourceScopeV1 {
    pub fn exact_url(value: &str) -> Result<Self, ResourceError> {
        let identity = ResourceIdentity::parse_url(value)?;
        Ok(Self::ExactUrl(identity.as_url().to_string()))
    }

    pub fn origin(value: &str) -> Result<Self, ResourceError> {
        let identity = ResourceIdentity::parse_url(value)?;
        Ok(Self::Origin(identity.origin()))
    }

    pub fn host(value: &str) -> Result<Self, ResourceError> {
        let identity = ResourceIdentity::parse_url(value)?;
        Ok(Self::Host {
            origin: identity.origin(),
            host: identity.host().ok_or(ResourceError::MissingHost)?.to_owned(),
        })
    }

    pub fn path_prefix(origin: &str, path: &str) -> Result<Self, ResourceError> {
        let identity = ResourceIdentity::parse_url(origin)?;
        if !path.starts_with('/') || path.contains('?') || path.contains('#') {
            return Err(ResourceError::InvalidPath);
        }
        // Path scopes are policy inputs, not URL references. Resolve them
        // against the canonical origin and require the serialized path to be
        // unchanged so dot-segments and special-scheme backslashes cannot
        // silently broaden or relocate the scope.
        let canonical = identity
            .as_url()
            .join(path)
            .map_err(|_| ResourceError::InvalidPath)?;
        if canonical.origin().ascii_serialization() != identity.origin()
            || canonical.path() != path
            || canonical.query().is_some()
            || canonical.fragment().is_some()
        {
            return Err(ResourceError::InvalidPath);
        }
        Ok(Self::PathPrefix {
            origin: identity.origin(),
            path: path.to_owned(),
        })
    }

    pub fn allows(&self, resource: &ResourceIdentity) -> bool {
        match self {
            Self::ExactUrl(expected) => resource.as_url().to_string() == *expected,
            Self::Origin(expected) => resource.origin() == *expected,
            Self::Host { origin, host } => {
                resource.origin() == *origin && resource.host() == Some(host.as_str())
            },
            Self::PathPrefix { origin, path } => {
                resource.origin() == *origin
                    && (resource.path() == path
                        || (resource.path().starts_with(path)
                            && (path.ends_with('/')
                                || resource.path()[path.len()..].starts_with('/'))))
            }
        }
    }
}



#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum NetworkAddressClassV1 {
    Global,
    Loopback,
    LinkLocal,
    Private,
    Multicast,
    Unspecified,
    Reserved,
}

impl NetworkAddressClassV1 {
    pub fn from_ip(ip: std::net::IpAddr) -> Self {
        match ip {
            std::net::IpAddr::V4(ip) => {
                let octets = ip.octets();
                if ip.is_loopback() {
                    Self::Loopback
                } else if ip.is_link_local() {
                    Self::LinkLocal
                } else if ip.is_private() {
                    Self::Private
                } else if ip.is_multicast() {
                    Self::Multicast
                } else if ip.is_unspecified() {
                    Self::Unspecified
                } else if (octets[0] == 192 && octets[1] == 0 && octets[2] == 0)
                    || (octets[0] == 198 && octets[1] == 51 && octets[2] == 100)
                    || (octets[0] == 203 && octets[1] == 0 && octets[2] == 113)
                {
                    Self::Reserved
                } else {
                    Self::Global
                }
            }
            std::net::IpAddr::V6(ip) => {
                if ip.is_loopback() {
                    Self::Loopback
                } else if ip.is_unspecified() {
                    Self::Unspecified
                } else if ip.is_multicast() {
                    Self::Multicast
                } else if (ip.segments()[0] & 0xfe00) == 0xfc00 {
                    Self::Private
                } else if (ip.segments()[0] & 0xffc0) == 0xfe80 {
                    Self::LinkLocal
                } else {
                    Self::Global
                }
            }
        }
    }

    pub fn allowed_for_public_web(self) -> bool {
        matches!(self, Self::Global)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetworkTargetPolicyV1 {
    pub allow_private: bool,
    pub allow_loopback: bool,
    pub allow_link_local: bool,
    pub allow_multicast: bool,
    pub allow_unspecified: bool,
    pub allow_reserved: bool,
}

impl Default for NetworkTargetPolicyV1 {
    fn default() -> Self {
        Self {
            allow_private: false,
            allow_loopback: false,
            allow_link_local: false,
            allow_multicast: false,
            allow_unspecified: false,
            allow_reserved: false,
        }
    }
}

impl NetworkTargetPolicyV1 {
    pub fn allows(self, class: NetworkAddressClassV1) -> bool {
        match class {
            NetworkAddressClassV1::Global => true,
            NetworkAddressClassV1::Private => self.allow_private,
            NetworkAddressClassV1::Loopback => self.allow_loopback,
            NetworkAddressClassV1::LinkLocal => self.allow_link_local,
            NetworkAddressClassV1::Multicast => self.allow_multicast,
            NetworkAddressClassV1::Unspecified => self.allow_unspecified,
            NetworkAddressClassV1::Reserved => self.allow_reserved,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceError {
    InvalidUrl,
    UnsupportedScheme,
    UserinfoNotAllowed,
    MissingHost,
    InvalidPath,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_url_is_canonicalized_by_the_url_parser() {
        let scope = ResourceScopeV1::exact_url("https://example.com:443/a").unwrap();
        let resource = ResourceIdentity::parse_url("https://example.com/a").unwrap();
        assert!(scope.allows(&resource));
    }

    #[test]
    fn origin_scope_does_not_cross_scheme_or_port() {
        let scope = ResourceScopeV1::origin("https://example.com").unwrap();
        assert!(scope.allows(&ResourceIdentity::parse_url("https://example.com/a").unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url("http://example.com/a").unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url("https://example.com:8443/a").unwrap()));
    }

    #[test]
    fn host_scope_binds_scheme_and_effective_port() {
        let scope = ResourceScopeV1::host("https://cdn.example.com").unwrap();
        assert!(scope.allows(&ResourceIdentity::parse_url(
            "https://cdn.example.com/a"
        ).unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url(
            "http://cdn.example.com/a"
        ).unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url(
            "https://cdn.example.com:8443/a"
        ).unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url(
            "https://cdn.example.com.attacker.invalid/a"
        ).unwrap()));
    }

    #[test]
    fn path_prefix_is_hierarchical() {
        let scope = ResourceScopeV1::path_prefix("https://example.com", "/assets/").unwrap();
        assert!(scope.allows(&ResourceIdentity::parse_url("https://example.com/assets/app.js").unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url("https://example.com/assets-evil/app.js").unwrap()));
        assert_eq!(
            ResourceScopeV1::path_prefix("https://example.com", "/assets/../"),
            Err(ResourceError::InvalidPath)
        );
        assert_eq!(
            ResourceScopeV1::path_prefix("https://example.com", "/assets?x=1"),
            Err(ResourceError::InvalidPath)
        );
    }

    #[test]
    fn url_parser_canonicalizes_dot_segments_before_scope_evaluation() {
        let resource = ResourceIdentity::parse_url(
            "https://example.com/assets/../private.js",
        ).unwrap();
        assert_eq!(resource.path(), "/private.js");

        let encoded = ResourceIdentity::parse_url(
            "https://example.com/assets/%2e%2e/private.js",
        ).unwrap();
        assert_eq!(encoded.path(), "/private.js");
    }

    #[test]
    fn credentials_are_not_a_resource_identity() {
        assert_eq!(
            ResourceIdentity::parse_url("https://user:secret@example.com/a"),
            Err(ResourceError::UserinfoNotAllowed)
        );
    }

    #[test]
    fn network_address_policy_denies_local_classes_by_default() {
        use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};

        let policy = NetworkTargetPolicyV1::default();
        assert!(policy.allows(NetworkAddressClassV1::from_ip(
            IpAddr::V4(Ipv4Addr::new(93, 184, 216, 34))
        )));
        assert!(!policy.allows(NetworkAddressClassV1::from_ip(
            IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))
        )));
        assert!(!policy.allows(NetworkAddressClassV1::from_ip(
            IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1))
        )));
        assert!(!policy.allows(NetworkAddressClassV1::from_ip(
            IpAddr::V6(Ipv6Addr::LOCALHOST)
        )));
    }

    #[test]
    fn network_address_classification_is_explicit_for_ipv6_ula_and_link_local() {
        use std::net::{IpAddr, Ipv6Addr};

        assert_eq!(
            NetworkAddressClassV1::from_ip(IpAddr::V6(
                "fc00::1".parse::<Ipv6Addr>().unwrap()
            )),
            NetworkAddressClassV1::Private
        );
        assert_eq!(
            NetworkAddressClassV1::from_ip(IpAddr::V6(
                "fe80::1".parse::<Ipv6Addr>().unwrap()
            )),
            NetworkAddressClassV1::LinkLocal
        );
    }

    #[test]
    fn non_web_schemes_are_not_resource_targets() {
        assert_eq!(
            ResourceIdentity::parse_url("file:///etc/passwd"),
            Err(ResourceError::UnsupportedScheme)
        );
    }
}
