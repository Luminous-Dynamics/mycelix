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

/// What the route itself guarantees. This deliberately does not claim that
/// the client verified the returned media against its content address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerificationLevel {
    HttpsTransportOnly,
}

/// The trust domain of the selected route. Configured origins are operator
/// supplied; their bytes are still not independently verified against the CID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeliveryProvider {
    ConfiguredHttpsOrigin,
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
    TooManyGatewayOrigins,
    EmptyAsset,
}

impl std::fmt::Display for DeliveryPlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::UnsupportedManifestVersion => "unsupported media manifest version",
            Self::InvalidAssetId => "invalid media asset identifier",
            Self::InvalidContentAddress => "invalid IPFS content identifier path component",
            Self::InvalidGatewayOrigin => "configured media gateway must be a valid HTTPS origin",
            Self::TooManyGatewayOrigins => "configured media gateway pool exceeds the maximum size",
            Self::EmptyAsset => "media asset cannot declare a zero-byte payload",
        };
        f.write_str(message)
    }
}

/// Maximum number of explicitly configured HTTPS origins. A bounded pool
/// keeps browser retry and connection fan-out predictable.
pub const MAX_CONFIGURED_GATEWAY_ORIGINS: usize = 8;

/// Parse an explicit, ordered list of HTTPS origins supplied by the operator.
///
/// Example: `https://media.example.org,https://media-backup.example.org`.
/// Public gateways are supported only when the operator deliberately lists them;
/// they are never added as an implicit fallback. Duplicate origins are removed
/// while preserving the first occurrence.
pub fn parse_gateway_origins(
    configured_origins: &str,
) -> Result<Vec<TrustedHttpsGatewayOrigin>, DeliveryPlanError> {
    let values: Vec<&str> = configured_origins.split(',').map(str::trim).collect();
    if values.is_empty() || values.iter().any(|value| value.is_empty()) {
        return Err(DeliveryPlanError::InvalidGatewayOrigin);
    }
    if values.len() > MAX_CONFIGURED_GATEWAY_ORIGINS {
        return Err(DeliveryPlanError::TooManyGatewayOrigins);
    }

    let mut origins: Vec<TrustedHttpsGatewayOrigin> = Vec::with_capacity(values.len());
    for value in values {
        let origin = TrustedHttpsGatewayOrigin::parse(value)?;
        if !origins.contains(&origin) {
            origins.push(origin);
        }
    }
    Ok(origins)
}

/// Derive ordered candidates only from an explicit gateway pool. An empty pool
/// returns no candidates, rather than silently disclosing requests to a public
/// gateway. URLs are built only from validated HTTPS origins and CID path
/// components; catalog metadata cannot choose the gateway origin.
pub fn delivery_candidates(
    manifest: &MediaAssetManifestV1,
) -> Result<Vec<DeliveryCandidate>, DeliveryPlanError> {
    delivery_candidates_with_origins(manifest, &[])
}

pub fn delivery_candidates_with_origins(
    manifest: &MediaAssetManifestV1,
    configured_origins: &[TrustedHttpsGatewayOrigin],
) -> Result<Vec<DeliveryCandidate>, DeliveryPlanError> {
    manifest.validate()?;
    if configured_origins.len() > MAX_CONFIGURED_GATEWAY_ORIGINS {
        return Err(DeliveryPlanError::TooManyGatewayOrigins);
    }

    let cid = match &manifest.content_address {
        ContentAddress::IpfsCid(cid) => cid,
    };

    let mut candidates = Vec::with_capacity(configured_origins.len());
    for origin in configured_origins {
        let url = origin.url_for_cid(cid);
        if candidates.iter().any(|candidate: &DeliveryCandidate| candidate.url == url) {
            continue;
        }
        candidates.push(DeliveryCandidate {
            provider: DeliveryProvider::ConfiguredHttpsOrigin,
            url,
            priority: candidates.len() as u8,
            verification: VerificationLevel::HttpsTransportOnly,
        });
    }

    Ok(candidates)
}

/// Resolve using an explicit compile-time gateway pool. Invalid non-empty
/// configuration fails closed. With no configured pool, no third-party
/// gateway is contacted; deployment documentation should explain how to select
/// an approved gateway and the privacy trade-offs of public gateways.
pub fn delivery_candidates_from_build_config(
    manifest: &MediaAssetManifestV1,
) -> Result<Vec<DeliveryCandidate>, DeliveryPlanError> {
    let configured_origins = match option_env!("MYCELIX_MEDIA_GATEWAY_ORIGINS") {
        Some(value) => parse_gateway_origins(value)?,
        None => Vec::new(),
    };
    delivery_candidates_with_origins(manifest, &configured_origins)
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
    fn does_not_silently_fall_back_to_hard_coded_public_gateways() {
        let candidates = delivery_candidates(&manifest(SAMPLE_CID)).unwrap();
        assert!(candidates.is_empty());
    }

    #[test]
    fn configured_gateway_pool_preserves_order_and_uses_only_explicit_origins() {
        let configured =
            parse_gateway_origins("https://media.example.org, https://dweb.link").unwrap();
        let candidates =
            delivery_candidates_with_origins(&manifest(SAMPLE_CID), &configured).unwrap();

        assert_eq!(candidates.len(), 2);
        assert_eq!(candidates[0].provider, DeliveryProvider::ConfiguredHttpsOrigin);
        assert_eq!(
            candidates[0].url,
            format!("https://media.example.org/ipfs/{SAMPLE_CID}")
        );
        assert_eq!(
            candidates[1].url,
            format!("https://dweb.link/ipfs/{SAMPLE_CID}")
        );
        assert_eq!(candidates[0].priority, 0);
        assert_eq!(candidates[1].priority, 1);
        assert!(candidates
            .iter()
            .all(|candidate| candidate.verification == VerificationLevel::HttpsTransportOnly));
    }

    #[test]
    fn gateway_pool_deduplicates_without_reordering() {
        let configured = parse_gateway_origins(
            "https://media.example.org,https://dweb.link,https://media.example.org",
        )
        .unwrap();
        assert_eq!(configured.len(), 2);
        assert_eq!(configured[0].0, "https://media.example.org");
        assert_eq!(configured[1].0, "https://dweb.link");
    }

    #[test]
    fn gateway_pool_rejects_empty_entries_and_excessive_size() {
        assert_eq!(
            parse_gateway_origins("").unwrap_err(),
            DeliveryPlanError::InvalidGatewayOrigin
        );
        assert_eq!(
            parse_gateway_origins("https://media.example.org,").unwrap_err(),
            DeliveryPlanError::InvalidGatewayOrigin
        );
        let too_many = (0..=MAX_CONFIGURED_GATEWAY_ORIGINS)
            .map(|index| format!("https://gateway{index}.example.org"))
            .collect::<Vec<_>>()
            .join(",");
        assert_eq!(
            parse_gateway_origins(&too_many).unwrap_err(),
            DeliveryPlanError::TooManyGatewayOrigins
        );
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
        let configured = parse_gateway_origins("https://media.example.org").unwrap();
        let candidates =
            delivery_candidates_with_origins(&manifest(SAMPLE_CID), &configured).unwrap();

        assert_eq!(candidates.len(), 1);
        assert!(candidates
            .iter()
            .all(|candidate| candidate.verification == VerificationLevel::HttpsTransportOnly));
    }

    #[test]
    fn accepts_cidv0_base58btc_text_form() {
        manifest("QmYwAPJzv5CZsnAzt8auVTLk5c5k8m7E2d8kqv7s8W9Z2x")
            .validate()
            .unwrap();
    }

    #[test]
    fn accepts_short_well_known_cidv1_base32_text_form() {
        manifest("bafkqaaa").validate().unwrap();
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
