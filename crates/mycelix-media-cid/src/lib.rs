// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Typed admission policy for media CIDs.
//!
//! Parsing establishes that an identifier is structurally valid under the CID
//! format. It does not establish that provider-returned bytes match the CID,
//! that the bytes decode as playable media, or that publication rights are valid.
//! Those are separate verification and authorization gates.

use cid::{Cid, Version};
use std::str::FromStr;

const MAX_CID_TEXT_BYTES: usize = 256;
const CODEC_RAW: u64 = 0x55;
const CODEC_DAG_PB: u64 = 0x70;
const MULTIHASH_SHA2_256: u64 = 0x12;
const SHA2_256_DIGEST_BYTES: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CidAdmissionError {
    Empty,
    TooLong,
    InvalidEncoding,
    NonCanonicalEncoding,
    UnsupportedVersion,
    UnsupportedCodec,
    UnsupportedMultihash,
}

impl std::fmt::Display for CidAdmissionError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::Empty => "CID is empty",
            Self::TooLong => "CID exceeds the text length limit",
            Self::InvalidEncoding => "CID encoding is invalid",
            Self::NonCanonicalEncoding => "CID is not in the canonical text representation",
            Self::UnsupportedVersion => "CID version is not supported by the Music delivery profile",
            Self::UnsupportedCodec => "CID codec is not supported by the Music delivery profile",
            Self::UnsupportedMultihash => "CID hash function or digest size is not supported by the Music delivery profile",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for CidAdmissionError {}

/// Parse and admit a CID used by the current Music HTTP-gateway profile.
///
/// Policy is deliberately narrower than the full CID ecosystem:
/// - canonical CIDv0 base58btc, or canonical CIDv1 base32-lower text;
/// - dag-pb (UnixFS-compatible path) or raw codec;
/// - sha2-256 multihash with a 32-byte digest.
///
/// This allowlist is an explicit initial compatibility profile, not a claim
/// that other valid CIDs are malformed or unsafe in every possible system.
pub fn parse_media_cid(value: &str) -> Result<Cid, CidAdmissionError> {
    if value.is_empty() {
        return Err(CidAdmissionError::Empty);
    }
    if value.len() > MAX_CID_TEXT_BYTES {
        return Err(CidAdmissionError::TooLong);
    }

    let cid = Cid::from_str(value).map_err(|_| CidAdmissionError::InvalidEncoding)?;

    // Display uses CIDv0 base58btc and CIDv1 base32-lower. Requiring a
    // round-trip makes path construction deterministic and rejects alternate
    // textual multibase encodings for the same binary identifier.
    if cid.to_string() != value {
        return Err(CidAdmissionError::NonCanonicalEncoding);
    }

    match cid.version() {
        Version::V0 => {
            // The CID library itself enforces the CIDv0 dag-pb / sha2-256
            // encoding constraints; retain the profile check here explicitly.
            if cid.codec() != CODEC_DAG_PB {
                return Err(CidAdmissionError::UnsupportedCodec);
            }
        }
        Version::V1 => match cid.codec() {
            CODEC_RAW | CODEC_DAG_PB => {}
            _ => return Err(CidAdmissionError::UnsupportedCodec),
        },
        _ => return Err(CidAdmissionError::UnsupportedVersion),
    }

    if cid.hash().code() != MULTIHASH_SHA2_256
        || cid.hash().size() != SHA2_256_DIGEST_BYTES
    {
        return Err(CidAdmissionError::UnsupportedMultihash);
    }

    Ok(cid)
}

/// Validate without exposing the parser's concrete CID representation.
pub fn validate_media_cid(value: &str) -> Result<(), CidAdmissionError> {
    parse_media_cid(value).map(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cid::multihash::Multihash;

    const CID_V0: &str = "QmTPcW343HGMdoxarwvHHoPhkbo5GfNYjnZkyW5DBtpvLe";
    const CID_V1_DAG_PB: &str =
        "bafybeigdyrzt5sfp7udm7hu76uh7y26nf3efuylqabf3oclgtqy55fbzdi";

    #[test]
    fn accepts_canonical_v0_and_v1_media_profiles() {
        let v0 = parse_media_cid(CID_V0).unwrap();
        assert_eq!(v0.version(), Version::V0);
        assert_eq!(v0.codec(), CODEC_DAG_PB);
        assert_eq!(v0.hash().code(), MULTIHASH_SHA2_256);
        assert_eq!(v0.hash().size(), SHA2_256_DIGEST_BYTES);

        let v1 = parse_media_cid(CID_V1_DAG_PB).unwrap();
        assert_eq!(v1.version(), Version::V1);
        assert_eq!(v1.codec(), CODEC_DAG_PB);
        assert_eq!(v1.hash().code(), MULTIHASH_SHA2_256);
        assert_eq!(v1.hash().size(), SHA2_256_DIGEST_BYTES);
    }

    #[test]
    fn accepts_raw_cidv1_sha256() {
        let hash = Multihash::<64>::wrap(MULTIHASH_SHA2_256, &[0x42; 32]).unwrap();
        let raw_cid = Cid::new_v1(CODEC_RAW, hash).to_string();
        let parsed = parse_media_cid(&raw_cid).unwrap();
        assert_eq!(parsed.codec(), CODEC_RAW);
    }

    #[test]
    fn rejects_bad_or_injection_shaped_identifiers() {
        for value in [
            "",
            "Qmshort",
            "bafyabc0",
            "bafybeigdyrzt5sfp7udm7hu76uh7y26nf3efuylqabf3oclgtqy55fbzdi?download=1",
            "../private",
            "/ipfs/QmTPcW343HGMdoxarwvHHoPhkbo5GfNYjnZkyW5DBtpvLe",
            "https://attacker.invalid/content",
        ] {
            assert!(parse_media_cid(value).is_err(), "unexpectedly admitted {value:?}");
        }
    }

    #[test]
    fn rejects_supported_syntax_with_unsupported_codec() {
        let hash = Multihash::<64>::wrap(MULTIHASH_SHA2_256, &[0x24; 32]).unwrap();
        let other_codec = Cid::new_v1(0x71, hash).to_string();
        assert_eq!(
            parse_media_cid(&other_codec),
            Err(CidAdmissionError::UnsupportedCodec)
        );
    }

    #[test]
    fn rejects_unsupported_multihash_profile() {
        let hash = Multihash::<64>::wrap(0x13, &[0x24; 32]).unwrap();
        let other_hash = Cid::new_v1(CODEC_DAG_PB, hash).to_string();
        assert_eq!(
            parse_media_cid(&other_hash),
            Err(CidAdmissionError::UnsupportedMultihash)
        );
    }

    #[test]
    fn rejects_oversized_text_before_parsing() {
        let value = format!("b{}", "a".repeat(MAX_CID_TEXT_BYTES));
        assert_eq!(parse_media_cid(&value), Err(CidAdmissionError::TooLong));
    }

    #[test]
    fn validates_same_canonical_contract_as_parser() {
        assert_eq!(validate_media_cid(CID_V0), Ok(()));
        assert_eq!(
            validate_media_cid("Qmshort"),
            Err(CidAdmissionError::InvalidEncoding)
        );
    }
}
