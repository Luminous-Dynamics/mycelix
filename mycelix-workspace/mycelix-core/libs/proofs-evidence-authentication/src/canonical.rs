use serde::{
    Deserialize, Deserializer, Serialize, Serializer,
    de::Error as DeError,
};
use sha2::{Digest, Sha256};

pub const QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1: &str =
    "mycelix-qualification-receipt-canonical-v1";
pub const MAX_RECEIPT_IDENTIFIER_BYTES_V1: usize = 1024;
pub const MAX_RECEIPT_NONCLAIMS_V1: usize = 64;
pub const MAX_RECEIPT_NONCLAIM_BYTES_V1: usize = 4096;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum QualificationReceiptCanonicalizationV1 {
    BinaryV1,
}

impl QualificationReceiptCanonicalizationV1 {
    pub const fn profile_id(self) -> &'static str {
        match self {
            Self::BinaryV1 => QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1,
        }
    }
}

impl Serialize for QualificationReceiptCanonicalizationV1 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(self.profile_id())
    }
}

impl<'de> Deserialize<'de> for QualificationReceiptCanonicalizationV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        match value.as_str() {
            QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1 => Ok(Self::BinaryV1),
            _ => Err(D::Error::custom("unsupported qualification receipt canonicalization")),
        }
    }
}

const QUALIFICATION_RECEIPT_DOMAIN_V1: &[u8] =
    b"mycelix:qualification-receipt:canonical:v1\0";

/// Exact SHA-256 digest used by the v1 receipt contract.
///
/// Serde wire form is exactly 64 lowercase hexadecimal characters. This is an
/// interchange representation only; qualification-receipt identity is defined by
/// the versioned binary canonicalization profile below.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Sha256DigestV1 {
    bytes: [u8; 32],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sha256DigestParseErrorV1 {
    InvalidHex,
}

impl Sha256DigestV1 {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self { bytes }
    }

    pub fn from_hex(value: &str) -> Result<Self, Sha256DigestParseErrorV1> {
        decode_lower_hex::<32>(value)
            .map(Self::from_bytes)
            .ok_or(Sha256DigestParseErrorV1::InvalidHex)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.bytes
    }

    pub fn to_hex(&self) -> String {
        encode_hex(&self.bytes)
    }
}

impl Serialize for Sha256DigestV1 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for Sha256DigestV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::from_hex(&value).map_err(|_| D::Error::custom("invalid lowercase SHA-256 digest"))
    }
}

/// Git object identity with the hash algorithm explicit.
///
/// Serde wire form is `sha1:<40 lowercase hex>` or `sha256:<64 lowercase hex>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GitObjectIdV1 {
    Sha1([u8; 20]),
    Sha256([u8; 32]),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GitObjectIdParseErrorV1 {
    InvalidFormat,
    InvalidHex,
}

impl GitObjectIdV1 {
    pub const fn sha1(bytes: [u8; 20]) -> Self {
        Self::Sha1(bytes)
    }

    pub const fn sha256(bytes: [u8; 32]) -> Self {
        Self::Sha256(bytes)
    }

    pub fn sha1_from_hex(value: &str) -> Result<Self, GitObjectIdParseErrorV1> {
        decode_lower_hex::<20>(value)
            .map(Self::Sha1)
            .ok_or(GitObjectIdParseErrorV1::InvalidHex)
    }

    pub fn sha256_from_hex(value: &str) -> Result<Self, GitObjectIdParseErrorV1> {
        decode_lower_hex::<32>(value)
            .map(Self::Sha256)
            .ok_or(GitObjectIdParseErrorV1::InvalidHex)
    }

    pub fn from_wire(value: &str) -> Result<Self, GitObjectIdParseErrorV1> {
        if let Some(hex) = value.strip_prefix("sha1:") {
            Self::sha1_from_hex(hex)
        } else if let Some(hex) = value.strip_prefix("sha256:") {
            Self::sha256_from_hex(hex)
        } else {
            Err(GitObjectIdParseErrorV1::InvalidFormat)
        }
    }

    pub fn to_hex(&self) -> String {
        match self {
            Self::Sha1(bytes) => encode_hex(bytes),
            Self::Sha256(bytes) => encode_hex(bytes),
        }
    }

    pub fn to_wire(&self) -> String {
        match self {
            Self::Sha1(_) => format!("sha1:{}", self.to_hex()),
            Self::Sha256(_) => format!("sha256:{}", self.to_hex()),
        }
    }
}

impl Serialize for GitObjectIdV1 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.to_wire())
    }
}

impl<'de> Deserialize<'de> for GitObjectIdV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::from_wire(&value).map_err(|_| D::Error::custom("invalid Git object ID"))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct QualificationReceiptDigestV1 {
    pub canonicalization: QualificationReceiptCanonicalizationV1,
    pub sha256: Sha256DigestV1,
}

impl QualificationReceiptDigestV1 {
    pub const fn canonicalization_profile(&self) -> &'static str {
        self.canonicalization.profile_id()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QualificationResultV1 {
    Pass,
    Fail,
    RecordedOnly,
}

/// Backend-neutral qualification claim whose canonical bytes are authenticated later.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationReceiptV1 {
    pub receipt_version: u32,
    pub qualification_profile: String,
    pub statement_profile: String,
    pub theorem_profile: String,
    pub subject: GitObjectIdV1,
    pub dependency_graph_digest: Sha256DigestV1,
    pub measured_security_receipt_digest: Sha256DigestV1,
    pub coherence_policy_id: String,
    pub coherence_result_digest: Sha256DigestV1,
    pub qualification_corpus_digest: Sha256DigestV1,
    pub execution_capsule_digest: Sha256DigestV1,
    pub result: QualificationResultV1,
    /// Semantic set. Canonicalization sorts and deduplicates entries.
    pub nonclaims: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReceiptCanonicalizationErrorV1 {
    UnsupportedReceiptVersion { actual: u32 },
    EmptyField { field: &'static str },
    IdentifierTooLong {
        field: &'static str,
        max_bytes: usize,
        actual_bytes: usize,
    },
    MissingNonclaims,
    EmptyNonclaim { index: usize },
    TooManyNonclaims { max: usize, actual: usize },
    NonclaimTooLong {
        index: usize,
        max_bytes: usize,
        actual_bytes: usize,
    },
}

impl QualificationReceiptV1 {
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReceiptCanonicalizationErrorV1> {
        self.validate()?;

        let mut out = Vec::with_capacity(512);
        out.extend_from_slice(QUALIFICATION_RECEIPT_DOMAIN_V1);
        put_u32(&mut out, self.receipt_version);
        put_str(&mut out, &self.qualification_profile);
        put_str(&mut out, &self.statement_profile);
        put_str(&mut out, &self.theorem_profile);
        put_git_object_id(&mut out, self.subject);
        put_digest(&mut out, self.dependency_graph_digest);
        put_digest(&mut out, self.measured_security_receipt_digest);
        put_str(&mut out, &self.coherence_policy_id);
        put_digest(&mut out, self.coherence_result_digest);
        put_digest(&mut out, self.qualification_corpus_digest);
        put_digest(&mut out, self.execution_capsule_digest);
        out.push(match self.result {
            QualificationResultV1::Pass => 1,
            QualificationResultV1::Fail => 2,
            QualificationResultV1::RecordedOnly => 3,
        });

        let nonclaims = self.normalized_nonclaims()?;
        put_u32(&mut out, nonclaims.len() as u32);
        for nonclaim in nonclaims {
            put_str(&mut out, &nonclaim);
        }

        Ok(out)
    }

    pub fn digest(&self) -> Result<QualificationReceiptDigestV1, ReceiptCanonicalizationErrorV1> {
        let bytes = self.canonical_bytes()?;
        let digest: [u8; 32] = Sha256::digest(bytes).into();
        Ok(QualificationReceiptDigestV1 {
            canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
            sha256: Sha256DigestV1::from_bytes(digest),
        })
    }

    fn validate(&self) -> Result<(), ReceiptCanonicalizationErrorV1> {
        if self.receipt_version != 1 {
            return Err(ReceiptCanonicalizationErrorV1::UnsupportedReceiptVersion {
                actual: self.receipt_version,
            });
        }

        for (field, value) in [
            ("qualification_profile", self.qualification_profile.as_str()),
            ("statement_profile", self.statement_profile.as_str()),
            ("theorem_profile", self.theorem_profile.as_str()),
            ("coherence_policy_id", self.coherence_policy_id.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(ReceiptCanonicalizationErrorV1::EmptyField { field });
            }
            if value.len() > MAX_RECEIPT_IDENTIFIER_BYTES_V1 {
                return Err(ReceiptCanonicalizationErrorV1::IdentifierTooLong {
                    field,
                    max_bytes: MAX_RECEIPT_IDENTIFIER_BYTES_V1,
                    actual_bytes: value.len(),
                });
            }
        }

        let _ = self.normalized_nonclaims()?;
        Ok(())
    }

    fn normalized_nonclaims(&self) -> Result<Vec<String>, ReceiptCanonicalizationErrorV1> {
        if self.nonclaims.is_empty() {
            return Err(ReceiptCanonicalizationErrorV1::MissingNonclaims);
        }
        if self.nonclaims.len() > MAX_RECEIPT_NONCLAIMS_V1 {
            return Err(ReceiptCanonicalizationErrorV1::TooManyNonclaims {
                max: MAX_RECEIPT_NONCLAIMS_V1,
                actual: self.nonclaims.len(),
            });
        }
        for (index, value) in self.nonclaims.iter().enumerate() {
            if value.trim().is_empty() {
                return Err(ReceiptCanonicalizationErrorV1::EmptyNonclaim { index });
            }
            if value.len() > MAX_RECEIPT_NONCLAIM_BYTES_V1 {
                return Err(ReceiptCanonicalizationErrorV1::NonclaimTooLong {
                    index,
                    max_bytes: MAX_RECEIPT_NONCLAIM_BYTES_V1,
                    actual_bytes: value.len(),
                });
            }
        }
        let mut nonclaims = self.nonclaims.clone();
        nonclaims.sort_unstable();
        nonclaims.dedup();
        Ok(nonclaims)
    }
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_str(out: &mut Vec<u8>, value: &str) {
    // v1 validation bounds all strings far below u32::MAX.
    put_u32(out, value.len() as u32);
    out.extend_from_slice(value.as_bytes());
}

fn put_digest(out: &mut Vec<u8>, digest: Sha256DigestV1) {
    out.extend_from_slice(digest.as_bytes());
}

fn put_git_object_id(out: &mut Vec<u8>, id: GitObjectIdV1) {
    match id {
        GitObjectIdV1::Sha1(bytes) => {
            out.push(1);
            out.extend_from_slice(&bytes);
        }
        GitObjectIdV1::Sha256(bytes) => {
            out.push(2);
            out.extend_from_slice(&bytes);
        }
    }
}

fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn decode_lower_hex<const N: usize>(value: &str) -> Option<[u8; N]> {
    if value.len() != N * 2 {
        return None;
    }
    let mut out = [0_u8; N];
    let bytes = value.as_bytes();
    for index in 0..N {
        let high = hex_nibble(bytes[index * 2])?;
        let low = hex_nibble(bytes[index * 2 + 1])?;
        out[index] = (high << 4) | low;
    }
    Some(out)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn receipt() -> QualificationReceiptV1 {
        QualificationReceiptV1 {
            receipt_version: 1,
            qualification_profile: "myc-zkp-range-001aq".into(),
            statement_profile: "range-membership-v1".into(),
            theorem_profile: "range-membership-v1-air".into(),
            subject: GitObjectIdV1::sha1([0xaa; 20]),
            dependency_graph_digest: digest(1),
            measured_security_receipt_digest: digest(2),
            coherence_policy_id: "coherence-v1".into(),
            coherence_result_digest: digest(3),
            qualification_corpus_digest: digest(4),
            execution_capsule_digest: digest(5),
            result: QualificationResultV1::Pass,
            nonclaims: vec![
                "witness privacy not established".into(),
                "application authority not granted".into(),
            ],
        }
    }

    #[test]
    fn digest_is_stable_under_nonclaim_order_and_duplicates() {
        let a = receipt();
        let mut b = receipt();
        b.nonclaims.reverse();
        b.nonclaims.push("application authority not granted".into());
        assert_eq!(a.digest().unwrap(), b.digest().unwrap());
    }

    #[test]
    fn theorem_bearing_mutation_changes_digest() {
        let a = receipt();
        let mut b = receipt();
        b.subject = GitObjectIdV1::sha1([0xbb; 20]);
        assert_ne!(a.digest().unwrap(), b.digest().unwrap());
    }

    #[test]
    fn receipt_version_and_nonclaims_fail_closed() {
        let mut r = receipt();
        r.receipt_version = 2;
        assert!(matches!(
            r.digest(),
            Err(ReceiptCanonicalizationErrorV1::UnsupportedReceiptVersion { actual: 2 })
        ));

        let mut r = receipt();
        r.nonclaims.clear();
        assert_eq!(
            r.digest(),
            Err(ReceiptCanonicalizationErrorV1::MissingNonclaims)
        );
    }

    #[test]
    fn resource_bounds_fail_closed() {
        let mut r = receipt();
        r.qualification_profile = "x".repeat(MAX_RECEIPT_IDENTIFIER_BYTES_V1 + 1);
        assert!(matches!(
            r.digest(),
            Err(ReceiptCanonicalizationErrorV1::IdentifierTooLong {
                field: "qualification_profile",
                ..
            })
        ));

        let mut r = receipt();
        r.nonclaims = (0..=MAX_RECEIPT_NONCLAIMS_V1)
            .map(|index| format!("nonclaim-{index}"))
            .collect();
        assert!(matches!(
            r.digest(),
            Err(ReceiptCanonicalizationErrorV1::TooManyNonclaims { .. })
        ));
    }

    #[test]
    fn typed_git_ids_do_not_conflate_sha1_and_sha256() {
        let sha1 = GitObjectIdV1::sha1([0x11; 20]);
        let sha256 = GitObjectIdV1::sha256([0x11; 32]);
        assert_ne!(sha1, sha256);
        assert_eq!(sha1.to_hex().len(), 40);
        assert_eq!(sha256.to_hex().len(), 64);
    }

    #[test]
    fn wire_formats_are_explicit_and_round_trip() {
        let digest = digest(0xab);
        let digest_json = serde_json::to_string(&digest).unwrap();
        assert_eq!(digest_json, format!("\"{}\"", "ab".repeat(32)));
        assert_eq!(serde_json::from_str::<Sha256DigestV1>(&digest_json).unwrap(), digest);

        let git = GitObjectIdV1::sha1([0xcd; 20]);
        let git_json = serde_json::to_string(&git).unwrap();
        assert_eq!(git_json, format!("\"sha1:{}\"", "cd".repeat(20)));
        assert_eq!(serde_json::from_str::<GitObjectIdV1>(&git_json).unwrap(), git);

        let profile = QualificationReceiptCanonicalizationV1::BinaryV1;
        let profile_json = serde_json::to_string(&profile).unwrap();
        assert_eq!(
            profile_json,
            format!(
                "\"{}\"",
                QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1
            )
        );
        assert_eq!(
            serde_json::from_str::<QualificationReceiptCanonicalizationV1>(&profile_json).unwrap(),
            profile
        );
    }
}
