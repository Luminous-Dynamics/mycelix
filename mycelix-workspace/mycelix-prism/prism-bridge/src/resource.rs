//! Broker-owned resource identities and scopes.
//!
//! Renderer resource strings are untrusted input. This module provides the
//! next authority boundary: parse/canonicalize a resource before applying a
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
    Host(String),
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
        Ok(Self::Host(identity.host().ok_or(ResourceError::MissingHost)?.to_owned()))
    }

    pub fn path_prefix(origin: &str, path: &str) -> Result<Self, ResourceError> {
        let identity = ResourceIdentity::parse_url(origin)?;
        if !path.starts_with('/') {
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
            Self::Host(expected) => resource.host() == Some(expected.as_str()),
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
    fn host_scope_does_not_match_a_suffix_attacker() {
        let scope = ResourceScopeV1::host("https://cdn.example.com").unwrap();
        assert!(scope.allows(&ResourceIdentity::parse_url("https://cdn.example.com/a").unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url("https://cdn.example.com.attacker.invalid/a").unwrap()));
    }

    #[test]
    fn path_prefix_is_hierarchical() {
        let scope = ResourceScopeV1::path_prefix("https://example.com", "/assets/").unwrap();
        assert!(scope.allows(&ResourceIdentity::parse_url("https://example.com/assets/app.js").unwrap()));
        assert!(!scope.allows(&ResourceIdentity::parse_url("https://example.com/assets-evil/app.js").unwrap()));
    }

    #[test]
    fn credentials_are_not_a_resource_identity() {
        assert_eq!(
            ResourceIdentity::parse_url("https://user:secret@example.com/a"),
            Err(ResourceError::UserinfoNotAllowed)
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
