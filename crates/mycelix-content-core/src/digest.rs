use serde::{Deserialize, Serialize};
use sha2::{Digest as _, Sha256};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum DigestAlgorithmV1 {
    Blake3_256,
    Sha256,
}

impl DigestAlgorithmV1 {
    pub fn tag(self) -> &'static str {
        match self {
            Self::Blake3_256 => "blake3-256",
            Self::Sha256 => "sha256",
        }
    }
}

/// Digest of the exact immutable blob bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ContentDigestV1 {
    pub algorithm: DigestAlgorithmV1,
    pub bytes: [u8; 32],
}

impl ContentDigestV1 {
    pub fn compute(algorithm: DigestAlgorithmV1, bytes: &[u8]) -> Self {
        let digest = match algorithm {
            DigestAlgorithmV1::Blake3_256 => *blake3::hash(bytes).as_bytes(),
            DigestAlgorithmV1::Sha256 => {
                let digest = Sha256::digest(bytes);
                let mut out = [0u8; 32];
                out.copy_from_slice(&digest);
                out
            }
        };
        Self {
            algorithm,
            bytes: digest,
        }
    }

    pub(crate) fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(8 + self.algorithm.tag().len() + 32);
        super::canonical::append_field(&mut out, self.algorithm.tag().as_bytes());
        out.extend_from_slice(&self.bytes);
        out
    }
}
