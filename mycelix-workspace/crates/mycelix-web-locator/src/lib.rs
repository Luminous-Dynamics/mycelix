#![forbid(unsafe_code)]
//! Bounded WHATWG web-locator parsing and projection.
//!
//! This crate has deliberately **zero network authority**. It parses a bounded
//! caller-supplied HTTP(S) locator using the workspace `url` implementation and
//! exposes typed parser observations. It does not resolve DNS, classify endpoint
//! addresses, open sockets, perform HTTP, create EPI artifacts, or authorize a
//! fetch.

use std::{fmt, net::{Ipv4Addr, Ipv6Addr}};
use url::{Host, ParseError, Url};

/// Maximum caller-supplied locator length admitted by the V1 parser profile.
pub const MAX_SUPPLIED_LOCATOR_BYTES: usize = 4096;

/// Machine-readable authority ceiling for this crate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WebLocatorAuthorityScopeV1 {
    /// Parser/projection facts only. No endpoint or network authority.
    ParserProjectionOnly,
}

/// Closed parser profile for the first web-locator tranche.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WebUrlProfileV1 {
    /// Parse only HTTPS web locators.
    HttpsOnly,
    /// Parse HTTP and HTTPS web locators.
    HttpAndHttps,
}

impl WebUrlProfileV1 {
    /// Stable profile identifier suitable for evidence/qualification metadata.
    pub const fn id(self) -> &'static str {
        match self {
            Self::HttpsOnly => "mycelix:web-url:https-only:v1",
            Self::HttpAndHttps => "mycelix:web-url:http-https:v1",
        }
    }

    fn admits_scheme(self, scheme: &str) -> bool {
        match self {
            Self::HttpsOnly => scheme == "https",
            Self::HttpAndHttps => matches!(scheme, "http" | "https"),
        }
    }
}

/// Bounded raw locator supplied by the caller.
///
/// `Debug` intentionally redacts the content because URLs can contain secrets
/// in userinfo, paths, or query parameters. Exact bytes remain available only
/// through the explicit [`Self::as_str`] accessor.
#[derive(Clone, Eq, PartialEq)]
pub struct SuppliedLocatorV1(String);

impl SuppliedLocatorV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, WebLocatorError> {
        let value = value.into();
        if value.is_empty() {
            return Err(WebLocatorError::InputEmpty);
        }
        if value.len() > MAX_SUPPLIED_LOCATOR_BYTES {
            return Err(WebLocatorError::InputTooLong {
                bytes: value.len(),
                max: MAX_SUPPLIED_LOCATOR_BYTES,
            });
        }
        if value.chars().any(char::is_control) {
            return Err(WebLocatorError::DisallowedControlCharacter);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn len_bytes(&self) -> usize {
        self.0.len()
    }
}

impl fmt::Debug for SuppliedLocatorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SuppliedLocatorV1")
            .field("bytes", &self.0.len())
            .field("value", &"<redacted>")
            .finish()
    }
}

/// Presence classification for URL userinfo without exposing reusable secret
/// values through the ordinary parser result API.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UserInfoPresenceV1 {
    None,
    UsernameOnly,
    UsernameAndPassword,
}

/// Typed semantic host produced by the WHATWG URL parser.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ParsedHostV1 {
    Domain(String),
    Ipv4(Ipv4Addr),
    Ipv6(Ipv6Addr),
}

impl ParsedHostV1 {
    /// Parser-serialized host text. IPv6 literals include brackets to match URL
    /// host serialization.
    pub fn serialized(&self) -> String {
        match self {
            Self::Domain(domain) => domain.clone(),
            Self::Ipv4(addr) => addr.to_string(),
            Self::Ipv6(addr) => format!("[{addr}]"),
        }
    }
}

/// Checked parser observation for one supplied locator.
///
/// The exact normalized URL is intentionally available through an explicit
/// accessor because it is evidence about parser behavior. `Debug` omits it to
/// reduce accidental credential/query leakage.
#[derive(Clone, Eq, PartialEq)]
pub struct ParsedWebUrlV1 {
    profile: WebUrlProfileV1,
    supplied: SuppliedLocatorV1,
    serialized_url: String,
    scheme: String,
    userinfo: UserInfoPresenceV1,
    host: ParsedHostV1,
    normalized_port: Option<u16>,
    effective_port: u16,
    path: String,
    query: Option<String>,
    fragment: Option<String>,
    target_uri: Option<String>,
}

impl ParsedWebUrlV1 {
    pub const fn authority_scope(&self) -> WebLocatorAuthorityScopeV1 {
        WebLocatorAuthorityScopeV1::ParserProjectionOnly
    }

    pub fn profile(&self) -> WebUrlProfileV1 {
        self.profile
    }

    pub fn supplied(&self) -> &SuppliedLocatorV1 {
        &self.supplied
    }

    /// Exact parser-serialized URL. This may contain sensitive userinfo or
    /// query material; callers should not log it indiscriminately.
    pub fn serialized_url(&self) -> &str {
        &self.serialized_url
    }

    pub fn scheme(&self) -> &str {
        &self.scheme
    }

    pub fn userinfo(&self) -> UserInfoPresenceV1 {
        self.userinfo
    }

    pub fn host(&self) -> &ParsedHostV1 {
        &self.host
    }

    /// Port remaining after WHATWG parser normalization. Explicit default
    /// ports may normalize to `None`; this MUST NOT be interpreted as proof
    /// that the caller omitted a port from the supplied locator.
    pub fn normalized_port(&self) -> Option<u16> {
        self.normalized_port
    }

    pub fn effective_port(&self) -> u16 {
        self.effective_port
    }

    pub fn path(&self) -> &str {
        &self.path
    }

    pub fn query(&self) -> Option<&str> {
        self.query.as_deref()
    }

    pub fn fragment(&self) -> Option<&str> {
        self.fragment.as_deref()
    }

    /// Server-facing HTTP(S) target URI projection with fragment removed.
    ///
    /// V1 deliberately returns `None` when URL userinfo exists. The parser
    /// observation is still retained, but a credential-bearing locator is not
    /// projected into a downstream target representation by this crate.
    pub fn target_uri(&self) -> Option<&str> {
        self.target_uri.as_deref()
    }
}

impl fmt::Debug for ParsedWebUrlV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ParsedWebUrlV1")
            .field("profile", &self.profile)
            .field("supplied_bytes", &self.supplied.len_bytes())
            .field("scheme", &self.scheme)
            .field("userinfo", &self.userinfo)
            .field("host", &self.host)
            .field("normalized_port", &self.normalized_port)
            .field("effective_port", &self.effective_port)
            .field("fragment_present", &self.fragment.is_some())
            .field("authority_scope", &self.authority_scope())
            .finish()
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum WebLocatorError {
    InputEmpty,
    InputTooLong { bytes: usize, max: usize },
    DisallowedControlCharacter,
    ParseFailure(ParseError),
    UnsupportedScheme,
    MissingHost,
    MissingEffectivePort,
}

impl fmt::Display for WebLocatorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputEmpty => f.write_str("locator input is empty"),
            Self::InputTooLong { bytes, max } => {
                write!(f, "locator input is {bytes} bytes; maximum is {max}")
            }
            Self::DisallowedControlCharacter => {
                f.write_str("locator input contains a disallowed control character")
            }
            Self::ParseFailure(error) => write!(f, "web URL parser rejected locator: {error}"),
            Self::UnsupportedScheme => f.write_str("URL scheme is not admitted by parser profile"),
            Self::MissingHost => f.write_str("web URL has no semantic host"),
            Self::MissingEffectivePort => f.write_str("web URL has no effective port"),
        }
    }
}

impl std::error::Error for WebLocatorError {}

/// Parse and project one bounded web locator under an explicit profile.
///
/// This function performs no DNS resolution or network I/O and creates no
/// target-admission authority.
pub fn parse_web_locator(
    supplied: SuppliedLocatorV1,
    profile: WebUrlProfileV1,
) -> Result<ParsedWebUrlV1, WebLocatorError> {
    let parsed = Url::parse(supplied.as_str()).map_err(WebLocatorError::ParseFailure)?;

    if !profile.admits_scheme(parsed.scheme()) {
        return Err(WebLocatorError::UnsupportedScheme);
    }

    let host = match parsed.host().ok_or(WebLocatorError::MissingHost)? {
        Host::Domain(domain) => ParsedHostV1::Domain(domain.to_owned()),
        Host::Ipv4(addr) => ParsedHostV1::Ipv4(addr),
        Host::Ipv6(addr) => ParsedHostV1::Ipv6(addr),
    };

    let userinfo = match (parsed.username().is_empty(), parsed.password().is_some()) {
        (true, false) => UserInfoPresenceV1::None,
        (_, true) => UserInfoPresenceV1::UsernameAndPassword,
        (false, false) => UserInfoPresenceV1::UsernameOnly,
    };

    let effective_port = parsed
        .port_or_known_default()
        .ok_or(WebLocatorError::MissingEffectivePort)?;

    let target_uri = if userinfo == UserInfoPresenceV1::None {
        let mut target = parsed.clone();
        target.set_fragment(None);
        Some(target.to_string())
    } else {
        None
    };

    Ok(ParsedWebUrlV1 {
        profile,
        serialized_url: parsed.to_string(),
        scheme: parsed.scheme().to_owned(),
        userinfo,
        host,
        normalized_port: parsed.port(),
        effective_port,
        path: parsed.path().to_owned(),
        query: parsed.query().map(ToOwned::to_owned),
        fragment: parsed.fragment().map(ToOwned::to_owned),
        target_uri,
        supplied,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const SEED: &str = include_str!(
        "../../../docs/epistemic/fixtures/WEB_LOCATOR_TEST_001_SEED_V0_1.json"
    );

    #[test]
    fn seed_parser_vectors_match_frozen_authoring_expectations() {
        let root: Value = serde_json::from_str(SEED).expect("seed JSON must parse");
        let cases = root["cases"].as_array().expect("cases must be an array");

        for case in cases {
            let id = case["case_id"].as_str().expect("case id");
            let input = case["input"].as_str().expect("input");
            let expected_parse = case["expected_parse"].as_str().expect("expected_parse");
            let supplied = SuppliedLocatorV1::new(input).expect("seed input must pass outer bound");
            let result = parse_web_locator(supplied, WebUrlProfileV1::HttpsOnly);

            if expected_parse == "failure" {
                assert!(result.is_err(), "{id}: expected parser failure");
                continue;
            }

            let parsed = result.unwrap_or_else(|error| panic!("{id}: unexpected error: {error}"));
            assert_eq!(
                parsed.serialized_url(),
                case["expected_serialized_url"].as_str().expect("serialized URL"),
                "{id}: serialized URL"
            );
            assert_eq!(
                parsed.host().serialized(),
                case["expected_semantic_host"].as_str().expect("semantic host"),
                "{id}: semantic host"
            );

            match case["expected_target_uri"].as_str() {
                Some(expected) => assert_eq!(parsed.target_uri(), Some(expected), "{id}: target URI"),
                None => assert_eq!(parsed.target_uri(), None, "{id}: target URI must be absent"),
            }

            assert_eq!(
                parsed.authority_scope(),
                WebLocatorAuthorityScopeV1::ParserProjectionOnly,
                "{id}: authority ceiling"
            );
        }
    }

    #[test]
    fn debug_output_does_not_leak_userinfo_secret() {
        let supplied = SuppliedLocatorV1::new("https://user:super-secret@example.org/path?q=secret")
            .expect("bounded locator");
        let parsed = parse_web_locator(supplied.clone(), WebUrlProfileV1::HttpsOnly)
            .expect("parser observation");

        let supplied_debug = format!("{supplied:?}");
        let parsed_debug = format!("{parsed:?}");
        assert!(!supplied_debug.contains("super-secret"));
        assert!(!parsed_debug.contains("super-secret"));
        assert!(!parsed_debug.contains("q=secret"));
        assert_eq!(parsed.userinfo(), UserInfoPresenceV1::UsernameAndPassword);
        assert_eq!(parsed.target_uri(), None);
    }

    #[test]
    fn https_only_profile_rejects_http_without_network_authority() {
        let supplied = SuppliedLocatorV1::new("http://example.org/").expect("bounded locator");
        assert_eq!(
            parse_web_locator(supplied, WebUrlProfileV1::HttpsOnly),
            Err(WebLocatorError::UnsupportedScheme)
        );
    }

    #[test]
    fn raw_input_survives_default_port_normalization() {
        let supplied = SuppliedLocatorV1::new("https://example.org:443/a").expect("bounded locator");
        let parsed = parse_web_locator(supplied, WebUrlProfileV1::HttpsOnly).expect("parsed");
        assert_eq!(parsed.supplied().as_str(), "https://example.org:443/a");
        assert_eq!(parsed.serialized_url(), "https://example.org/a");
        assert_eq!(parsed.normalized_port(), None);
        assert_eq!(parsed.effective_port(), 443);
    }

    #[test]
    fn percent_encoded_reserved_data_is_not_double_decoded() {
        let once = parse_web_locator(
            SuppliedLocatorV1::new("https://example.org/a%2Fb").unwrap(),
            WebUrlProfileV1::HttpsOnly,
        )
        .unwrap();
        let twice = parse_web_locator(
            SuppliedLocatorV1::new("https://example.org/a%252Fb").unwrap(),
            WebUrlProfileV1::HttpsOnly,
        )
        .unwrap();

        assert_eq!(once.path(), "/a%2Fb");
        assert_eq!(twice.path(), "/a%252Fb");
        assert_ne!(once.serialized_url(), twice.serialized_url());
    }

    #[test]
    fn parser_projection_does_not_expose_target_admission_type() {
        let parsed = parse_web_locator(
            SuppliedLocatorV1::new("https://example.org/").unwrap(),
            WebUrlProfileV1::HttpsOnly,
        )
        .unwrap();
        assert_eq!(parsed.authority_scope(), WebLocatorAuthorityScopeV1::ParserProjectionOnly);
    }
}
