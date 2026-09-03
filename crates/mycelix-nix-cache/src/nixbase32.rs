const ALPHABET: &[u8; 32] = b"0123456789abcdfghijklmnpqrsvwxyz";

pub fn is_nix_base32_byte(byte: u8) -> bool {
    ALPHABET.contains(&byte)
}

pub fn encode_nix_base32(bytes: &[u8]) -> String {
    let len = bytes.len().saturating_mul(8).div_ceil(5);
    let mut out = String::with_capacity(len);
    for n in (0..len).rev() {
        let bit = n * 5;
        let index = bit / 8;
        let shift = bit % 8;
        let low = u16::from(bytes[index]) >> shift;
        let high = if index + 1 < bytes.len() {
            u16::from(bytes[index + 1]) << (8 - shift)
        } else {
            0
        };
        out.push(ALPHABET[((low | high) & 0x1f) as usize] as char);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_known_nix_sha256_vector() {
        let digest = [
            0x2c, 0xf2, 0x4d, 0xba, 0x5f, 0xb0, 0xa3, 0x0e, 0x26, 0xe8, 0x3b, 0x2a, 0xc5,
            0xb9, 0xe2, 0x9e, 0x1b, 0x16, 0x1e, 0x5c, 0x1f, 0xa7, 0x42, 0x5e, 0x73, 0x04,
            0x33, 0x62, 0x93, 0x8b, 0x98, 0x24,
        ];
        assert_eq!(
            encode_nix_base32(&digest),
            "094qif9n4cq4fdg459qzbhg1c6wywawwaaivx0k0x8xhbyx4vwic"
        );
    }
}
