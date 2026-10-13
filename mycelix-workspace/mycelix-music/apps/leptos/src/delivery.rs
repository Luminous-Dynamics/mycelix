// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Media delivery planning for the browser client.
//!
//! This module intentionally performs no network I/O. It converts catalog
//! metadata into a bounded, ordered set of HTTPS gateway candidates so the
//! player can recover from one gateway being unavailable.
//!
//! IMPORTANT: HTTPS gateway delivery establishes transport security, not
//! client-side verification that the returned bytes match an IPFS CID. A
//! native IPFS/trustless-gateway verifier is a separate future implementation.

/// Version of the local delivery-manifest contract.
pub const MEDIA_ASSET_MANIFEST_SCHEMA_V1: u16 = 1;

/// The storage address attached to a logical media asset.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ContentAddress {
    /// IPFS content identifier. The path check below prevents URL/path
    /// injection; cryptographic CID decoding and byte verification are not
    /// performed by this browser-side planning module.
    IpfsCid(String),
}

/// Minimal presentation-time manifest.
///
/// This is an adapter over current catalog records, not yet a signed,
/// multi-rendition manifest loaded from Holochain.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MediaAssetManifestV1 {
    pub schema_version: u16,
    pub asset_id: String,
    pub content_address: ContentAddress,
    pub expected_byte_length: Option<u64>,
}

impl MediaAssetManifestV1 {
    pub fn for_ipfs(asset_id: impl Into<String>, cid: impl Into<String>) -> Self {
        Self {
            schema_version: MEDIA_ASSET_MANIFEST_SCHEMA_V1,
            asset_id: asset_id.into(),
            content_address: ContentAddress::IpfsCid(cid.into()),
            expected_byte_length: None,
        }
    }

    pub fn validate(&self) -> Result<(), DeliveryPlanError> {
        if self.schema_version != MEDIA_ASSET_MANIFEST_SCHEMA_V1 {
            return Err(DeliveryPlanError::UnsupportedManifestVersion);
        }
        if self.asset_id.trim().is_empty() || self.asset_id.len() > 512 {
            return Err(DeliveryPlanError::InvalidAssetId);
        }
        if self.expected_byte_length == Some(0) {
            return Err(DeliveryPlanError::EmptyAsset);
        }

        match &self.content_address {
            ContentAddress::IpfsCid(cid) => validate_cid_path_component(cid)?,
        }

        Ok(())
    }
}

/// Deployment-configured HTTPS gateway origin.
///
/// Only an operator-controlled build setting should be passed to `parse`;
/// never accept this value from publication metadata or a provider lease.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrustedHttpsGatewayOrigin(String);

impl TrustedHttpsGatewayOrigin {
    pub fn parse(origin: &str) -> Result<Self, DeliveryPlanError> {
        let Some(authority) = origin.strip_prefix("https://") else {
            return Err(DeliveryPlanError::InvalidGatewayOrigin);
        };

        let mut authority_parts = authority.split(':');
        let host = authority_parts.next().unwrap_or_default();
        let port = authority_parts.next();
        let has_extra_port_separator = authority_parts.next().is_some();
        let valid_host = !host.is_empty()
            && host.len() <= 253
            && host.split('.').all(|label| {
                !label.is_empty()
                    && label.len() <= 63
                    && label.as_bytes().first().is_some_and(u8::is_ascii_alphanumeric)
                    && label.as_bytes().last().is_some_and(u8::is_ascii_alphanumeric)
                    && label
                        .bytes()
                        .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            });
        let valid_port = port.is_none_or(|value| {
            !value.is_empty()
                && value.bytes().all(|byte| byte.is_ascii_digit())
                && value.parse::<u16>().is_ok_and(|number| number > 0)
        });

        if authority.is_empty()
            || authority.chars().any(|ch| matches!(ch, '/' | '?' | '#' | '@' | '\\' | '%'))
            || has_extra_port_separator
            || !valid_host
            || !valid_port
        {
            return Err(DeliveryPlanError::InvalidGatewayOrigin);
        }

        Ok(Self(origin.to_string()))
    }

    fn url_for_cid(&self, cid: &str) -> String {
        format!("{}/ipfs/{}", self.0, cid)
    }
}

/// Candidate HTTPS gateways supported by the initial resolver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IpfsGateway {
    IpfsIo,
    DwebLink,
}

impl IpfsGateway {
    pub const fn base_url(self) -> &'static str {
        match self {
            Self::IpfsIo => "https://ipfs.io/ipfs",
            Self::DwebLink => "https://dweb.link/ipfs",
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::IpfsIo => "ipfs.io",
            Self::DwebLink => "dweb.link",
        }
    }
}

/// What the route itself guarantees. This deliberately does not claim that
/// the client verified the returned media against its content address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerificationLevel {
    HttpsTransportOnly,
}

/// Provider kind helps telemetry distinguish the configured platform gateway
/// from public IPFS gateway fallbacks without treating either as trusted bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeliveryProvider {
    ConfiguredHttpsOrigin,
    PublicIpfsGateway(IpfsGateway),
}

/// A retrieval option in deterministic preference order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeliveryCandidate {
    pub provider: DeliveryProvider,
    pub url: String,
    pub priority: u8,
    pub verification: VerificationLevel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeliveryPlanError {
    UnsupportedManifestVersion,
    InvalidAssetId,
    InvalidContentAddress,
    InvalidGatewayOrigin,
    EmptyAsset,
}

impl std::fmt::Display for DeliveryPlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::UnsupportedManifestVersion => "unsupported media manifest version",
            Self::InvalidAssetId => "invalid media asset identifier",
            Self::InvalidContentAddress => "invalid IPFS content identifier path component",
            Self::InvalidGatewayOrigin => "configured media gateway must be a valid HTTPS origin",
            Self::EmptyAsset => "media asset cannot declare a zero-byte payload",
        };
        f.write_str(message)
    }
}

/// Derive ordered gateway candidates from the validated manifest.
///
/// The URLs are constructed only from fixed HTTPS origins and a validated
/// single path component. Arbitrary schemes, hosts, query strings, fragments,
/// encoded separators, and path traversal are rejected before URL creation.
pub fn delivery_candidates(
    manifest: &MediaAssetManifestV1,
) -> Result<Vec<DeliveryCandidate>, DeliveryPlanError> {
    delivery_candidates_with_origin(manifest, None)
}

/// Build the candidate plan with an optional operator-configured gateway first.
///
/// The configured origin is intentionally separate from immutable asset metadata;
/// deployments can set `MYCELIX_MEDIA_GATEWAY_ORIGIN=https://media.example.org`.
/// The current release does not assume such a service is deployed.
pub fn delivery_candidates_with_origin(
    manifest: &MediaAssetManifestV1,
    configured_origin: Option<&TrustedHttpsGatewayOrigin>,
) -> Result<Vec<DeliveryCandidate>, DeliveryPlanError> {
    manifest.validate()?;

    let cid = match &manifest.content_address {
        ContentAddress::IpfsCid(cid) => cid,
    };

    let mut candidates = Vec::with_capacity(3);
    if let Some(origin) = configured_origin {
        candidates.push(DeliveryCandidate {
            provider: DeliveryProvider::ConfiguredHttpsOrigin,
            url: origin.url_for_cid(cid),
            priority: 0,
            verification: VerificationLevel::HttpsTransportOnly,
        });
    }

    for gateway in [IpfsGateway::IpfsIo, IpfsGateway::DwebLink] {
        candidates.push(DeliveryCandidate {
            provider: DeliveryProvider::PublicIpfsGateway(gateway),
            url: format!("{}/{}", gateway.base_url(), cid),
            priority: candidates.len() as u8,
            verification: VerificationLevel::HttpsTransportOnly,
        });
    }

    Ok(candidates)
}

/// Resolve using the optional HTTPS origin supplied at compile time by the
/// deployer. Invalid non-empty configuration fails closed instead of being
/// interpolated into a URL or silently bypassed.
pub fn delivery_candidates_from_build_config(
    manifest: &MediaAssetManifestV1,
) -> Result<Vec<DeliveryCandidate>, DeliveryPlanError> {
    let configured_origin = option_env!("MYCELIX_MEDIA_GATEWAY_ORIGIN")
        .map(TrustedHttpsGatewayOrigin::parse)
        .transpose()?;
    delivery_candidates_with_origin(manifest, configured_origin.as_ref())
}

/// Select the next candidate after a failed request without exceeding the
/// bounded candidate list. Returning `None` means the plan is exhausted.
pub fn next_candidate_index(current_index: usize, candidate_count: usize) -> Option<usize> {
    current_index
        .checked_add(1)
        .filter(|next_index| *next_index < candidate_count)
}

/// Validate the text envelope of common IPFS CID encodings before using it
/// as a single URL path component. This accepts CIDv0 Base58BTC text and
/// CIDv1 lowercase Base32 text. It deliberately does not decode the CID
/// structure or verify the multihash; a canonical CID library and verified
/// block/CAR retrieval are required for cryptographic content verification.
fn validate_cid_path_component(cid: &str) -> Result<(), DeliveryPlanError> {
    const BASE58_BTC: &[u8] =
        b"123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz";

    let cid_v0_base58btc = cid.len() == 46
        && cid.starts_with("Qm")
        && cid.bytes().all(|byte| BASE58_BTC.contains(&byte));

    // CIDv1's lowercase base32 multibase prefix is 'b'. The smallest common
    // well-known CID text form, bafkqaaa, is eight characters.
    let cid_v1_base32 = cid.len() >= 8
        && cid.len() <= 256
        && cid.starts_with('b')
        && cid
            .bytes()
            .skip(1)
            .all(|byte| byte.is_ascii_lowercase() || (b'2'..=b'7').contains(&byte));

    if !cid_v0_base58btc && !cid_v1_base32 {
        return Err(DeliveryPlanError::InvalidContentAddress);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_CID: &str =
        "bafybeigdyrzt5sfp7udm7hu76uh7y26nf3efuylqabf3oclgtqy55fbzdi";

    fn manifest(cid: &str) -> MediaAssetManifestV1 {
        MediaAssetManifestV1::for_ipfs("song:example-1", cid)
    }

    #[test]
    fn provides_two_distinct_https_gateways_in_stable_order() {
        let candidates = delivery_candidates(&manifest(SAMPLE_CID)).unwrap();

        assert_eq!(candidates.len(), 2);
        assert_eq!(
            candidates[0].provider,
            DeliveryProvider::PublicIpfsGateway(IpfsGateway::IpfsIo)
        );
        assert_eq!(
            candidates[0].url,
            format!("https://ipfs.io/ipfs/{SAMPLE_CID}")
        );
        assert_eq!(
            candidates[1].provider,
            DeliveryProvider::PublicIpfsGateway(IpfsGateway::DwebLink)
        );
        assert_eq!(
            candidates[1].url,
            format!("https://dweb.link/ipfs/{SAMPLE_CID}")
        );
        assert!(candidates[0].priority < candidates[1].priority);
    }

    #[test]
    fn configured_gateway_is_first_but_public_fallbacks_remain() {
        let configured = TrustedHttpsGatewayOrigin::parse("https://media.example.org").unwrap();
        let candidates =
            delivery_candidates_with_origin(&manifest(SAMPLE_CID), Some(&configured)).unwrap();

        assert_eq!(candidates.len(), 3);
        assert_eq!(candidates[0].provider, DeliveryProvider::ConfiguredHttpsOrigin);
        assert_eq!(
            candidates[0].url,
            format!("https://media.example.org/ipfs/{SAMPLE_CID}")
        );
        assert_eq!(candidates[0].priority, 0);
        assert_eq!(candidates[1].priority, 1);
        assert_eq!(candidates[2].priority, 2);
    }

    #[test]
    fn rejects_non_https_or_non_origin_gateway_configuration() {
        for origin in [
            "http://media.example.org",
            "https://user@media.example.org",
            "https://media.example.org/path",
            "https://media.example.org?token=x",
            "https://media.example.org#fragment",
            "https://media.example.org/%2f",
            "https://-invalid.example.org",
            "https://media..example.org",
            "https://media.example.org:abc",
            "https://media.example.org:0",
            "https://media.example.org:65536",
            "https://[::1]:8080",
            "",
        ] {
            assert_eq!(
                TrustedHttpsGatewayOrigin::parse(origin).unwrap_err(),
                DeliveryPlanError::InvalidGatewayOrigin,
                "expected rejection for {origin:?}"
            );
        }
    }

    #[test]
    fn explicitly_does_not_claim_client_side_content_verification() {
        let candidates = delivery_candidates(&manifest(SAMPLE_CID)).unwrap();

        assert!(candidates
            .iter()
            .all(|candidate| candidate.verification == VerificationLevel::HttpsTransportOnly));
    }

    #[test]
    fn accepts_cidv0_base58btc_text_form() {
        let candidates = delivery_candidates(&manifest(
            "QmYwAPJzv5CZsnAzt8auVTLk5c5k8m7E2d8kqv7s8W9Z2x",
        ))
        .unwrap();

        assert_eq!(candidates.len(), 2);
    }

    #[test]
    fn accepts_short_well_known_cidv1_base32_text_form() {
        let candidates = delivery_candidates(&manifest("bafkqaaa")).unwrap();

        assert_eq!(candidates.len(), 2);
    }

    #[test]
    fn rejects_non_cid_words_and_invalid_text_encodings() {
        for cid in [
            "hello",
            "not-a-cid",
            "bafy",             // too short to be a CID text form
            "bafyabc0",         // base32 excludes 0 and 1
            "bafyabc1",
            "Qmshort",
            "Qm00000000000000000000000000000000000000000000", // Base58BTC excludes 0
            "zQmYwAPJzv5CZsnAzt8auVTLk5c5k8m7E2d8kqv7s8W9Z2x", // unsupported multibase here
        ] {
            assert_eq!(
                delivery_candidates(&manifest(cid)).unwrap_err(),
                DeliveryPlanError::InvalidContentAddress,
                "expected rejection for {cid:?}"
            );
        }
    }

    #[test]
    fn rejects_empty_cid() {
        assert_eq!(
            delivery_candidates(&manifest("")).unwrap_err(),
            DeliveryPlanError::InvalidContentAddress
        );
    }

    #[test]
    fn rejects_path_traversal_and_url_injection() {
        for cid in [
            "../private",
            "bafy/other",
            "bafy?download=1",
            "bafy#fragment",
            "https://attacker.invalid/cid",
            "%2fetc%2fpasswd",
            "has space",
        ] {
            assert_eq!(
                delivery_candidates(&manifest(cid)).unwrap_err(),
                DeliveryPlanError::InvalidContentAddress,
                "expected rejection for {cid:?}"
            );
        }
    }

    #[test]
    fn candidate_advancement_is_bounded_and_overflow_safe() {
        assert_eq!(next_candidate_index(0, 2), Some(1));
        assert_eq!(next_candidate_index(1, 2), None);
        assert_eq!(next_candidate_index(0, 0), None);
        assert_eq!(next_candidate_index(usize::MAX, usize::MAX), None);
    }

    #[test]
    fn rejects_unknown_manifest_versions() {
        let mut value = manifest(SAMPLE_CID);
        value.schema_version = 2;

        assert_eq!(
            delivery_candidates(&value).unwrap_err(),
            DeliveryPlanError::UnsupportedManifestVersion
        );
    }

    #[test]
    fn rejects_empty_asset_identity_and_zero_byte_media() {
        let mut no_identity = manifest(SAMPLE_CID);
        no_identity.asset_id = " ".into();
        assert_eq!(
            delivery_candidates(&no_identity).unwrap_err(),
            DeliveryPlanError::InvalidAssetId
        );

        let mut zero_bytes = manifest(SAMPLE_CID);
        zero_bytes.expected_byte_length = Some(0);
        assert_eq!(
            delivery_candidates(&zero_bytes).unwrap_err(),
            DeliveryPlanError::EmptyAsset
        );
    }
}
