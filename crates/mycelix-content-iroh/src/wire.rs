use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};

use crate::WireErrorV1;

pub const CONTENT_ALPN_V1: &[u8] = b"mycelix/content/1";
pub const REQUEST_LEN_V1: usize = 44;
pub const RESPONSE_HEADER_LEN_V1: usize = 24;

const REQUEST_MAGIC_V1: &[u8; 8] = b"MYCFGET1";
const RESPONSE_MAGIC_V1: &[u8; 8] = b"MYCFRES1";
const SCHEMA_VERSION_V1: u8 = 1;
const OP_GET_V1: u8 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContentRequestV1 {
    pub digest: ContentDigestV1,
}

impl ContentRequestV1 {
    pub fn encode(self) -> [u8; REQUEST_LEN_V1] {
        let mut out = [0_u8; REQUEST_LEN_V1];
        out[..8].copy_from_slice(REQUEST_MAGIC_V1);
        out[8] = SCHEMA_VERSION_V1;
        out[9] = OP_GET_V1;
        out[10] = encode_algorithm(self.digest.algorithm);
        out[11] = 0;
        out[12..44].copy_from_slice(&self.digest.bytes);
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, WireErrorV1> {
        if bytes.len() != REQUEST_LEN_V1 {
            return Err(WireErrorV1::InvalidLength);
        }
        if &bytes[..8] != REQUEST_MAGIC_V1 {
            return Err(WireErrorV1::InvalidMagic);
        }
        if bytes[8] != SCHEMA_VERSION_V1 {
            return Err(WireErrorV1::UnsupportedVersion);
        }
        if bytes[9] != OP_GET_V1 {
            return Err(WireErrorV1::UnsupportedOperation);
        }
        if bytes[11] != 0 {
            return Err(WireErrorV1::NonZeroReserved);
        }
        let algorithm = decode_algorithm(bytes[10])?;
        let mut digest = [0_u8; 32];
        digest.copy_from_slice(&bytes[12..44]);
        Ok(Self {
            digest: ContentDigestV1 {
                algorithm,
                bytes: digest,
            },
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ContentResponseStatusV1 {
    Ok = 0,
    NotFound = 1,
    Busy = 2,
    IntegrityFailure = 3,
    ProtocolError = 4,
    InternalError = 5,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContentResponseHeaderV1 {
    pub status: ContentResponseStatusV1,
    pub size_bytes: u64,
}

impl ContentResponseHeaderV1 {
    pub fn ok(size_bytes: u64) -> Self {
        Self {
            status: ContentResponseStatusV1::Ok,
            size_bytes,
        }
    }

    pub fn error(status: ContentResponseStatusV1) -> Self {
        debug_assert!(status != ContentResponseStatusV1::Ok);
        Self {
            status,
            size_bytes: 0,
        }
    }

    pub fn encode(self) -> [u8; RESPONSE_HEADER_LEN_V1] {
        let mut out = [0_u8; RESPONSE_HEADER_LEN_V1];
        out[..8].copy_from_slice(RESPONSE_MAGIC_V1);
        out[8] = SCHEMA_VERSION_V1;
        out[9] = self.status as u8;
        out[16..24].copy_from_slice(&self.size_bytes.to_be_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, WireErrorV1> {
        if bytes.len() != RESPONSE_HEADER_LEN_V1 {
            return Err(WireErrorV1::InvalidLength);
        }
        if &bytes[..8] != RESPONSE_MAGIC_V1 {
            return Err(WireErrorV1::InvalidMagic);
        }
        if bytes[8] != SCHEMA_VERSION_V1 {
            return Err(WireErrorV1::UnsupportedVersion);
        }
        if bytes[10..16].iter().any(|byte| *byte != 0) {
            return Err(WireErrorV1::NonZeroReserved);
        }
        let status = match bytes[9] {
            0 => ContentResponseStatusV1::Ok,
            1 => ContentResponseStatusV1::NotFound,
            2 => ContentResponseStatusV1::Busy,
            3 => ContentResponseStatusV1::IntegrityFailure,
            4 => ContentResponseStatusV1::ProtocolError,
            5 => ContentResponseStatusV1::InternalError,
            _ => return Err(WireErrorV1::UnsupportedStatus),
        };
        let mut size = [0_u8; 8];
        size.copy_from_slice(&bytes[16..24]);
        let size_bytes = u64::from_be_bytes(size);
        if status != ContentResponseStatusV1::Ok && size_bytes != 0 {
            return Err(WireErrorV1::NonZeroReserved);
        }
        Ok(Self { status, size_bytes })
    }
}

fn encode_algorithm(algorithm: DigestAlgorithmV1) -> u8 {
    match algorithm {
        DigestAlgorithmV1::Blake3_256 => 1,
        DigestAlgorithmV1::Sha256 => 2,
    }
}

fn decode_algorithm(value: u8) -> Result<DigestAlgorithmV1, WireErrorV1> {
    match value {
        1 => Ok(DigestAlgorithmV1::Blake3_256),
        2 => Ok(DigestAlgorithmV1::Sha256),
        _ => Err(WireErrorV1::UnsupportedAlgorithm),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trip_preserves_algorithm_and_digest() {
        let request = ContentRequestV1 {
            digest: ContentDigestV1 {
                algorithm: DigestAlgorithmV1::Sha256,
                bytes: [0x5a; 32],
            },
        };
        let encoded = request.encode();
        assert_eq!(&encoded[..8], b"MYCFGET1");
        assert_eq!(encoded[8], 1);
        assert_eq!(encoded[9], 1);
        assert_eq!(encoded[10], 2);
        assert_eq!(encoded[11], 0);
        assert_eq!(ContentRequestV1::decode(&encoded).unwrap(), request);
    }

    #[test]
    fn request_reserved_byte_is_fail_closed() {
        let mut encoded = ContentRequestV1 {
            digest: ContentDigestV1 {
                algorithm: DigestAlgorithmV1::Blake3_256,
                bytes: [7; 32],
            },
        }
        .encode();
        encoded[11] = 1;
        assert_eq!(
            ContentRequestV1::decode(&encoded),
            Err(WireErrorV1::NonZeroReserved)
        );
    }

    #[test]
    fn response_round_trip_is_fixed_width() {
        let header = ContentResponseHeaderV1::ok(42);
        let encoded = header.encode();
        assert_eq!(encoded.len(), RESPONSE_HEADER_LEN_V1);
        assert_eq!(ContentResponseHeaderV1::decode(&encoded).unwrap(), header);
    }

    #[test]
    fn error_response_cannot_smuggle_payload_length() {
        let mut encoded =
            ContentResponseHeaderV1::error(ContentResponseStatusV1::NotFound).encode();
        encoded[23] = 1;
        assert_eq!(
            ContentResponseHeaderV1::decode(&encoded),
            Err(WireErrorV1::NonZeroReserved)
        );
    }
}
