use std::io::{self, Read};

use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use sha2::{Digest as _, Sha256};

const BUFFER_BYTES: usize = 64 * 1024;

enum StreamingHasherV1 {
    Blake3(blake3::Hasher),
    Sha256(Sha256),
}

impl StreamingHasherV1 {
    fn new(algorithm: DigestAlgorithmV1) -> Self {
        match algorithm {
            DigestAlgorithmV1::Blake3_256 => Self::Blake3(blake3::Hasher::new()),
            DigestAlgorithmV1::Sha256 => Self::Sha256(Sha256::new()),
        }
    }

    fn update(&mut self, bytes: &[u8]) {
        match self {
            Self::Blake3(hasher) => {
                hasher.update(bytes);
            }
            Self::Sha256(hasher) => {
                hasher.update(bytes);
            }
        }
    }

    fn finalize(self, algorithm: DigestAlgorithmV1) -> ContentDigestV1 {
        let bytes = match self {
            Self::Blake3(hasher) => *hasher.finalize().as_bytes(),
            Self::Sha256(hasher) => {
                let digest = hasher.finalize();
                let mut out = [0_u8; 32];
                out.copy_from_slice(&digest);
                out
            }
        };
        ContentDigestV1 { algorithm, bytes }
    }
}

pub(crate) fn hash_reader<R: Read>(
    algorithm: DigestAlgorithmV1,
    reader: &mut R,
    mut on_chunk: impl FnMut(&[u8]) -> io::Result<()>,
) -> io::Result<(ContentDigestV1, u64)> {
    let mut hasher = StreamingHasherV1::new(algorithm);
    let mut total = 0_u64;
    let mut buffer = vec![0_u8; BUFFER_BYTES];

    loop {
        let read = reader.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        let chunk = &buffer[..read];
        total = total
            .checked_add(read as u64)
            .ok_or_else(|| io::Error::other("stream length overflow"))?;
        hasher.update(chunk);
        on_chunk(chunk)?;
    }

    Ok((hasher.finalize(algorithm), total))
}
