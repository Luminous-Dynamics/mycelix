//! Fixed-width digest primitive.

/// A 32-byte normative digest value.
///
/// ASSURE-002A defines the representation only. Hash computation is introduced
/// in a later tranche so the first kernel remains dependency-free.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Digest([u8; Self::LEN]);

impl Digest {
    /// Digest width in bytes.
    pub const LEN: usize = 32;

    /// Construct a digest from exactly 32 bytes.
    pub const fn from_bytes(bytes: [u8; Self::LEN]) -> Self {
        Self(bytes)
    }

    /// Borrow the digest bytes.
    pub const fn as_bytes(&self) -> &[u8; Self::LEN] {
        &self.0
    }

    /// Consume the value and return its bytes.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digest_is_exactly_32_bytes() {
        assert_eq!(core::mem::size_of::<Digest>(), Digest::LEN);
    }
}
