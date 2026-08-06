// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pluggable authority-signing boundary.
//!
//! Protocol objects should depend on a public-key/signature capability rather
//! than directly owning private key bytes. `SigningKey` implements this trait
//! for local development and offline bootstrap. Production deployments may
//! provide an adapter backed by a hardware security module, cloud KMS, remote
//! signer, or threshold-signature service without changing canonical bytes.

use crate::{Error, Result};
use ed25519_dalek::{Signer, SigningKey, VerifyingKey};
#[cfg(unix)]
use serde::{Deserialize, Serialize};

/// Synchronous Ed25519 signing capability used by authority protocols.
///
/// Implementations must return a 64-byte Ed25519 signature over the exact
/// supplied message. Callers independently verify every returned signature
/// against `verifying_key()` before persistence.
pub trait AuthoritySigner: Send + Sync {
    /// Stable non-secret identifier useful for operator audit logs.
    fn key_id(&self) -> &str;

    /// Public Ed25519 verification key.
    fn verifying_key(&self) -> VerifyingKey;

    /// Sign canonical protocol bytes.
    fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>>;
}

impl AuthoritySigner for SigningKey {
    fn key_id(&self) -> &str {
        "software-ed25519"
    }

    fn verifying_key(&self) -> VerifyingKey {
        SigningKey::verifying_key(self)
    }

    fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>> {
        Ok(self.sign(message).to_bytes().to_vec())
    }
}

/// Small wrapper for deployments that need an explicit key identifier while
/// still using a local software key during migration or testing.
pub struct NamedSoftwareAuthoritySigner {
    key_id: String,
    key: SigningKey,
}

impl NamedSoftwareAuthoritySigner {
    pub fn new(key_id: impl Into<String>, key: SigningKey) -> Result<Self> {
        let key_id = key_id.into();
        validate_signer_key_id(&key_id)?;
        let trimmed = key_id.trim();
        Ok(Self {
            key_id: trimmed.to_string(),
            key,
        })
    }
}

impl AuthoritySigner for NamedSoftwareAuthoritySigner {
    fn key_id(&self) -> &str {
        &self.key_id
    }

    fn verifying_key(&self) -> VerifyingKey {
        self.key.verifying_key()
    }

    fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>> {
        Ok(self.key.sign(message).to_bytes().to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signature, Verifier};

    #[test]
    fn software_signer_returns_verifiable_signature() {
        let signer = NamedSoftwareAuthoritySigner::new(
            "hsm-slot:development",
            SigningKey::from_bytes(&[61; 32]),
        )
        .unwrap();
        let message = b"canonical authority bytes";
        let signature = signer.sign_message(message).unwrap();
        signer
            .verifying_key()
            .verify(message, &Signature::try_from(signature.as_slice()).unwrap())
            .unwrap();
        assert_eq!(signer.key_id(), "hsm-slot:development");
    }

    #[cfg(unix)]
    #[test]
    fn unix_socket_signer_verifies_remote_identity_and_signature() {
        use ed25519_dalek::Signer;
        use std::io::{Read, Write};
        use std::os::unix::net::UnixListener;

        let directory = tempfile::tempdir().unwrap();
        let socket_path = directory.path().join("authority-signer.sock");
        let remote_key = SigningKey::from_bytes(&[62; 32]);
        let listener = UnixListener::bind(&socket_path).unwrap();
        let public_key_hex = hex::encode(remote_key.verifying_key().to_bytes());
        let server = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut request_bytes = Vec::new();
            stream.read_to_end(&mut request_bytes).unwrap();
            let request: serde_json::Value = serde_json::from_slice(&request_bytes).unwrap();
            let message = hex::decode(request["message_hex"].as_str().unwrap()).unwrap();
            let signature = remote_key.sign(&message).to_bytes();
            let response = serde_json::json!({
                "protocol": "mycelix-authority-signer-v1",
                "key_id": "pkcs11:slot-7:epoch",
                "public_key_hex": public_key_hex,
                "signature_hex": hex::encode(signature),
            });
            stream
                .write_all(serde_json::to_string(&response).unwrap().as_bytes())
                .unwrap();
        });

        let signer = UnixSocketAuthoritySigner::new(
            "pkcs11:slot-7:epoch",
            SigningKey::from_bytes(&[62; 32]).verifying_key(),
            socket_path,
            std::time::Duration::from_secs(2),
        )
        .unwrap();
        let signature = signer.sign_message(b"governed epoch bytes").unwrap();
        signer
            .verifying_key()
            .verify(
                b"governed epoch bytes",
                &Signature::try_from(signature.as_slice()).unwrap(),
            )
            .unwrap();
        server.join().unwrap();
    }
}

/// Request sent to a local authority-signing agent over a Unix-domain socket.
///
/// The agent owns the PKCS#11, TPM, cloud-KMS, or threshold-signature session.
/// Mycelix sends exact canonical bytes and independently verifies the returned
/// Ed25519 signature before any protocol object is persisted.
#[cfg(unix)]
#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct UnixSignerRequest<'a> {
    protocol: &'static str,
    operation: &'static str,
    key_id: &'a str,
    message_hex: String,
}

#[cfg(unix)]
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct UnixSignerResponse {
    protocol: String,
    key_id: String,
    public_key_hex: String,
    signature_hex: String,
}

/// Concrete remote signer adapter for a local Unix-domain signing agent.
///
/// The adapter never invokes a shell, applies read/write timeouts, caps the
/// response size, verifies the agent identity, and verifies every signature.
/// A production agent may use PKCS#11, TPM2, cloud KMS, or an independently
/// operated threshold signer behind this narrow protocol.
#[cfg(unix)]
pub struct UnixSocketAuthoritySigner {
    key_id: String,
    verifying_key: VerifyingKey,
    socket_path: std::path::PathBuf,
    timeout: std::time::Duration,
}

#[cfg(unix)]
impl UnixSocketAuthoritySigner {
    pub fn new(
        key_id: impl Into<String>,
        verifying_key: VerifyingKey,
        socket_path: impl Into<std::path::PathBuf>,
        timeout: std::time::Duration,
    ) -> Result<Self> {
        let key_id = key_id.into();
        validate_signer_key_id(&key_id)?;
        let socket_path = socket_path.into();
        if !socket_path.is_absolute() {
            return Err(Error::Validation(
                "authority signer socket path must be absolute".to_string(),
            ));
        }
        if timeout.is_zero() || timeout > std::time::Duration::from_secs(60) {
            return Err(Error::Validation(
                "authority signer socket timeout must be between 1 millisecond and 60 seconds"
                    .to_string(),
            ));
        }
        Ok(Self {
            key_id: key_id.trim().to_string(),
            verifying_key,
            socket_path,
            timeout,
        })
    }

    fn request_signature(&self, message: &[u8]) -> Result<Vec<u8>> {
        use std::io::{Read, Write};
        use std::os::unix::net::UnixStream;

        const MAX_SIGNER_RESPONSE_BYTES: u64 = 64 * 1024;
        const SIGNER_PROTOCOL: &str = "mycelix-authority-signer-v1";

        let mut stream = UnixStream::connect(&self.socket_path).map_err(|error| {
            Error::Storage(format!(
                "failed to connect to authority signer socket {}: {error}",
                self.socket_path.display()
            ))
        })?;
        stream
            .set_read_timeout(Some(self.timeout))
            .map_err(|error| Error::Storage(error.to_string()))?;
        stream
            .set_write_timeout(Some(self.timeout))
            .map_err(|error| Error::Storage(error.to_string()))?;

        let request = UnixSignerRequest {
            protocol: SIGNER_PROTOCOL,
            operation: "sign_ed25519",
            key_id: &self.key_id,
            message_hex: hex::encode(message),
        };
        let request_bytes = serde_json::to_vec(&request)?;
        if request_bytes.len() > 1024 * 1024 {
            return Err(Error::Validation(
                "authority signer request exceeds one MiB".to_string(),
            ));
        }
        stream
            .write_all(&request_bytes)
            .and_then(|_| stream.write_all(b"\n"))
            .and_then(|_| stream.flush())
            .map_err(|error| Error::Storage(error.to_string()))?;
        stream
            .shutdown(std::net::Shutdown::Write)
            .map_err(|error| Error::Storage(error.to_string()))?;

        let mut response_bytes = Vec::new();
        stream
            .take(MAX_SIGNER_RESPONSE_BYTES + 1)
            .read_to_end(&mut response_bytes)
            .map_err(|error| Error::Storage(error.to_string()))?;
        if response_bytes.is_empty() || response_bytes.len() as u64 > MAX_SIGNER_RESPONSE_BYTES {
            return Err(Error::VerificationFailed(
                "authority signer response is empty or exceeds 64 KiB".to_string(),
            ));
        }
        let response: UnixSignerResponse = serde_json::from_slice(&response_bytes)?;
        if response.protocol != SIGNER_PROTOCOL || response.key_id != self.key_id {
            return Err(Error::VerificationFailed(
                "authority signer response does not match the requested protocol or key id"
                    .to_string(),
            ));
        }
        let public_key = decode_fixed_hex::<32>(&response.public_key_hex, "signer public key")?;
        if public_key != self.verifying_key.to_bytes() {
            return Err(Error::VerificationFailed(
                "authority signer returned an unexpected public key".to_string(),
            ));
        }
        let signature = hex::decode(response.signature_hex.trim())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let parsed = ed25519_dalek::Signature::try_from(signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        use ed25519_dalek::Verifier;
        self.verifying_key
            .verify(message, &parsed)
            .map_err(|error| Error::VerificationFailed(error.to_string()))?;
        Ok(signature)
    }
}

#[cfg(unix)]
impl AuthoritySigner for UnixSocketAuthoritySigner {
    fn key_id(&self) -> &str {
        &self.key_id
    }

    fn verifying_key(&self) -> VerifyingKey {
        self.verifying_key
    }

    fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>> {
        self.request_signature(message)
    }
}

fn validate_signer_key_id(key_id: &str) -> Result<()> {
    let trimmed = key_id.trim();
    if trimmed.is_empty() || trimmed.len() > 256 || trimmed.chars().any(char::is_control) {
        return Err(Error::Validation(
            "authority signer key id must contain 1-256 printable bytes".to_string(),
        ));
    }
    Ok(())
}

#[cfg(unix)]
fn decode_fixed_hex<const N: usize>(value: &str, label: &str) -> Result<[u8; N]> {
    let decoded = hex::decode(value.trim()).map_err(|error| Error::Crypto(error.to_string()))?;
    decoded
        .try_into()
        .map_err(|_| Error::VerificationFailed(format!("{label} must decode to exactly {N} bytes")))
}
