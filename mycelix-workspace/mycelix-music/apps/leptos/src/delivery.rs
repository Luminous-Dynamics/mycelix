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

/// A retrieval option in deterministic preference order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeliveryCandidate {
    pub provider: IpfsGateway,
    pub url: String,
    pub priority: u8,
    pub verification: VerificationLevel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeliveryPlanError {
    UnsupportedManifestVersion,
    InvalidAssetId,
    InvalidContentAddress,
    EmptyAsset,
}

impl std::fmt::Display for DeliveryPlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::UnsupportedManifestVersion => "unsupported media manifest version",
            Self::InvalidAssetId => "invalid media asset identifier",
            Self::InvalidContentAddress => "invalid IPFS content identifier path component",
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
    manifest.validate()?;

    let cid = match &manifest.content_address {
        ContentAddress::IpfsCid(cid) => cid,
    };

    let gateways = [IpfsGateway::IpfsIo, IpfsGateway::DwebLink];
    Ok(gateways
        .into_iter()
        .enumerate()
        .map(|(index, provider)| DeliveryCandidate {
            provider,
            url: format!("{}/{}", provider.base_url(), cid),
            priority: index as u8,
            verification: VerificationLevel::HttpsTransportOnly,
        })
        .collect())
}

/// Select the next candidate after a failed request without exceeding the
/// bounded candidate list. Returning `None` means the plan is exhausted.
pub fn next_candidate_index(current_index: usize, candidate_count: usize) -> Option<usize> {
    current_index
        .checked_add(1)
        .filter(|next_index| *next_index < candidate_count)
}

/// Conservative path-component validation, not a substitute for decoding a
/// CID using a canonical IPFS library. It prevents a catalog string from
/// escaping the single path component used in the generated gateway URL.
fn validate_cid_path_component(cid: &str) -> Result<(), DeliveryPlanError> {
    if cid.is_empty()
        || cid.len() > 256
        || !cid.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_'
        })
    {
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
        assert_eq!(candidates[0].provider, IpfsGateway::IpfsIo);
        assert_eq!(
            candidates[0].url,
            format!("https://ipfs.io/ipfs/{SAMPLE_CID}")
        );
        assert_eq!(candidates[1].provider, IpfsGateway::DwebLink);
        assert_eq!(
            candidates[1].url,
            format!("https://dweb.link/ipfs/{SAMPLE_CID}")
        );
        assert!(candidates[0].priority < candidates[1].priority);
    }

    #[test]
    fn explicitly_does_not_claim_client_side_content_verification() {
        let candidates = delivery_candidates(&manifest(SAMPLE_CID)).unwrap();

        assert!(candidates
            .iter()
            .all(|candidate| candidate.verification == VerificationLevel::HttpsTransportOnly));
    }

    #[test]
    fn accepts_legacy_style_cid_as_a_safe_path_component() {
        let candidates = delivery_candidates(&manifest(
            "QmYwAPJzv5CZsnAzt8auVTLk5c5k8m7E2d8kqv7s8W9Z2x",
        ))
        .unwrap();

        assert_eq!(candidates.len(), 2);
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
