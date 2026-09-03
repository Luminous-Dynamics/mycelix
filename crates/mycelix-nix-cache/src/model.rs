use std::collections::BTreeMap;

use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use thiserror::Error;

use crate::nixbase32::{encode_nix_base32, is_nix_base32_byte};

pub const NIX_STORE_DIR_V1: &str = "/nix/store";
const STORE_HASH_LEN: usize = 32;
const MAX_STORE_NAME_LEN: usize = 211;
const MAX_SIGNATURE_LEN: usize = 512;
const MAX_CA_LEN: usize = 512;

#[derive(Debug, Error)]
pub enum NixCacheErrorV1 {
    #[error("invalid Nix store hash: {0}")]
    InvalidStoreHash(String),
    #[error("invalid Nix store path: {0}")]
    InvalidStorePath(String),
    #[error("invalid Nix store path base name: {0}")]
    InvalidStorePathBaseName(String),
    #[error("raw NAR publication requires a SHA-256 CAS digest")]
    NarDigestMustBeSha256,
    #[error("raw NAR size must be non-zero")]
    ZeroNarSize,
    #[error("duplicate Nix reference: {0}")]
    DuplicateReference(String),
    #[error("invalid Nix signature line")]
    InvalidSignature,
    #[error("invalid Nix content-address field")]
    InvalidContentAddress,
    #[error("duplicate Nix store hash in publication catalog: {0}")]
    DuplicateStoreHash(String),
    #[error("Nix cache must bind to loopback: {0}")]
    NonLoopbackBind(std::net::SocketAddr),
    #[error("verification concurrency must be non-zero")]
    ZeroVerificationConcurrency,
    #[error("verification concurrency {requested} exceeds maximum {maximum}")]
    VerificationConcurrencyTooHigh { requested: usize, maximum: usize },
    #[error("failed to bind Nix cache at {addr}: {source}")]
    Bind {
        addr: std::net::SocketAddr,
        #[source]
        source: std::io::Error,
    },
    #[error("Nix cache server failed: {0}")]
    Serve(#[source] std::io::Error),
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NixStoreHashV1(String);

impl NixStoreHashV1 {
    pub fn parse(value: &str) -> Result<Self, NixCacheErrorV1> {
        if value.len() != STORE_HASH_LEN
            || !value.as_bytes().iter().copied().all(is_nix_base32_byte)
        {
            return Err(NixCacheErrorV1::InvalidStoreHash(value.to_string()));
        }
        Ok(Self(value.to_string()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NixStorePathBaseNameV1 {
    value: String,
    hash: NixStoreHashV1,
}

impl NixStorePathBaseNameV1 {
    pub fn parse(value: &str) -> Result<Self, NixCacheErrorV1> {
        let Some((hash, name)) = value.split_once('-') else {
            return Err(NixCacheErrorV1::InvalidStorePathBaseName(
                value.to_string(),
            ));
        };
        let hash = NixStoreHashV1::parse(hash)?;
        if name.is_empty()
            || name.len() > MAX_STORE_NAME_LEN
            || !name.as_bytes().iter().all(|byte| {
                byte.is_ascii_alphanumeric()
                    || matches!(*byte, b'+' | b'-' | b'.' | b'_' | b'?' | b'=')
            })
        {
            return Err(NixCacheErrorV1::InvalidStorePathBaseName(
                value.to_string(),
            ));
        }
        Ok(Self {
            value: value.to_string(),
            hash,
        })
    }

    pub fn as_str(&self) -> &str {
        &self.value
    }

    pub fn hash(&self) -> &NixStoreHashV1 {
        &self.hash
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NixStorePathV1 {
    value: String,
    base_name: NixStorePathBaseNameV1,
}

impl NixStorePathV1 {
    pub fn parse(value: &str) -> Result<Self, NixCacheErrorV1> {
        let prefix = format!("{NIX_STORE_DIR_V1}/");
        let Some(base_name) = value.strip_prefix(&prefix) else {
            return Err(NixCacheErrorV1::InvalidStorePath(value.to_string()));
        };
        if base_name.contains('/') {
            return Err(NixCacheErrorV1::InvalidStorePath(value.to_string()));
        }
        let base_name = NixStorePathBaseNameV1::parse(base_name)?;
        Ok(Self {
            value: value.to_string(),
            base_name,
        })
    }

    pub fn as_str(&self) -> &str {
        &self.value
    }

    pub fn base_name(&self) -> &NixStorePathBaseNameV1 {
        &self.base_name
    }

    pub fn hash(&self) -> &NixStoreHashV1 {
        self.base_name.hash()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct NixSignatureV1(String);

impl NixSignatureV1 {
    pub fn parse(value: &str) -> Result<Self, NixCacheErrorV1> {
        if value.is_empty()
            || value.len() > MAX_SIGNATURE_LEN
            || value.bytes().any(|byte| matches!(byte, b'\r' | b'\n'))
        {
            return Err(NixCacheErrorV1::InvalidSignature);
        }
        let Some((key_name, signature)) = value.split_once(':') else {
            return Err(NixCacheErrorV1::InvalidSignature);
        };
        if key_name.is_empty()
            || signature.is_empty()
            || !key_name.bytes().all(|byte| {
                byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'-' | b'_')
            })
            || !signature
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'+' | b'/' | b'='))
        {
            return Err(NixCacheErrorV1::InvalidSignature);
        }
        Ok(Self(value.to_string()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NixContentAddressV1(String);

impl NixContentAddressV1 {
    pub fn parse(value: &str) -> Result<Self, NixCacheErrorV1> {
        if value.is_empty()
            || value.len() > MAX_CA_LEN
            || !value.bytes().all(|byte| byte.is_ascii_graphic())
        {
            return Err(NixCacheErrorV1::InvalidContentAddress);
        }
        Ok(Self(value.to_string()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NixCacheEntryV1 {
    store_path: NixStorePathV1,
    nar_digest: ContentDigestV1,
    nar_size: u64,
    references: Vec<NixStorePathBaseNameV1>,
    deriver: Option<NixStorePathBaseNameV1>,
    signatures: Vec<NixSignatureV1>,
    ca: Option<NixContentAddressV1>,
}

impl NixCacheEntryV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        store_path: &str,
        nar_digest: ContentDigestV1,
        nar_size: u64,
        references: Vec<&str>,
        deriver: Option<&str>,
        signatures: Vec<&str>,
        ca: Option<&str>,
    ) -> Result<Self, NixCacheErrorV1> {
        if nar_digest.algorithm != DigestAlgorithmV1::Sha256 {
            return Err(NixCacheErrorV1::NarDigestMustBeSha256);
        }
        if nar_size == 0 {
            return Err(NixCacheErrorV1::ZeroNarSize);
        }

        let mut references = references
            .into_iter()
            .map(NixStorePathBaseNameV1::parse)
            .collect::<Result<Vec<_>, _>>()?;
        references.sort();
        if let Some(pair) = references.windows(2).find(|pair| pair[0] == pair[1]) {
            return Err(NixCacheErrorV1::DuplicateReference(
                pair[0].as_str().to_string(),
            ));
        }

        let deriver = deriver
            .map(NixStorePathBaseNameV1::parse)
            .transpose()?;
        let mut signatures = signatures
            .into_iter()
            .map(NixSignatureV1::parse)
            .collect::<Result<Vec<_>, _>>()?;
        signatures.sort();
        signatures.dedup();
        let ca = ca.map(NixContentAddressV1::parse).transpose()?;

        Ok(Self {
            store_path: NixStorePathV1::parse(store_path)?,
            nar_digest,
            nar_size,
            references,
            deriver,
            signatures,
            ca,
        })
    }

    pub fn store_path(&self) -> &NixStorePathV1 {
        &self.store_path
    }

    pub fn nar_digest(&self) -> ContentDigestV1 {
        self.nar_digest
    }

    pub fn nar_size(&self) -> u64 {
        self.nar_size
    }

    pub fn references(&self) -> &[NixStorePathBaseNameV1] {
        &self.references
    }

    pub fn deriver(&self) -> Option<&NixStorePathBaseNameV1> {
        self.deriver.as_ref()
    }

    pub fn signatures(&self) -> &[NixSignatureV1] {
        &self.signatures
    }

    pub fn content_address(&self) -> Option<&NixContentAddressV1> {
        self.ca.as_ref()
    }

    pub fn nar_hash_text(&self) -> String {
        format!("sha256:{}", encode_nix_base32(&self.nar_digest.bytes))
    }

    pub fn nar_url(&self) -> String {
        format!(
            "nar/{}-{}.nar",
            self.store_path.hash().as_str(),
            encode_nix_base32(&self.nar_digest.bytes)
        )
    }

    pub fn render_narinfo(&self) -> String {
        let nar_hash = self.nar_hash_text();
        let references = self
            .references
            .iter()
            .map(NixStorePathBaseNameV1::as_str)
            .collect::<Vec<_>>()
            .join(" ");
        let deriver = self
            .deriver
            .as_ref()
            .map_or("unknown-deriver", NixStorePathBaseNameV1::as_str);

        let mut output = format!(
            "StorePath: {}\nURL: {}\nCompression: none\nFileHash: {}\nFileSize: {}\nNarHash: {}\nNarSize: {}\nReferences: {}\nDeriver: {}\n",
            self.store_path.as_str(),
            self.nar_url(),
            nar_hash,
            self.nar_size,
            nar_hash,
            self.nar_size,
            references,
            deriver,
        );
        for signature in &self.signatures {
            output.push_str("Sig: ");
            output.push_str(signature.as_str());
            output.push('\n');
        }
        if let Some(ca) = &self.ca {
            output.push_str("CA: ");
            output.push_str(ca.as_str());
            output.push('\n');
        }
        output
    }
}

#[derive(Debug, Clone, Default)]
pub struct NixCacheCatalogV1 {
    by_store_hash: BTreeMap<NixStoreHashV1, NixCacheEntryV1>,
}

impl NixCacheCatalogV1 {
    pub fn new(entries: Vec<NixCacheEntryV1>) -> Result<Self, NixCacheErrorV1> {
        let mut by_store_hash = BTreeMap::new();
        for entry in entries {
            let key = entry.store_path().hash().clone();
            if by_store_hash.insert(key.clone(), entry).is_some() {
                return Err(NixCacheErrorV1::DuplicateStoreHash(key.0));
            }
        }
        Ok(Self { by_store_hash })
    }

    pub fn entry(&self, store_hash: &NixStoreHashV1) -> Option<&NixCacheEntryV1> {
        self.by_store_hash.get(store_hash)
    }

    pub fn is_empty(&self) -> bool {
        self.by_store_hash.is_empty()
    }

    pub fn len(&self) -> usize {
        self.by_store_hash.len()
    }
}
