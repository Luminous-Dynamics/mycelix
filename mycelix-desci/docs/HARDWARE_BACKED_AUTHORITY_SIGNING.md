# Hardware-Backed Authority Signing

Mycelix-DeSci authority protocols depend on the `AuthoritySigner` capability
rather than requiring direct ownership of Ed25519 private bytes.

```rust
pub trait AuthoritySigner: Send + Sync {
    fn key_id(&self) -> &str;
    fn verifying_key(&self) -> ed25519_dalek::VerifyingKey;
    fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>>;
}
```

`SigningKey` and `NamedSoftwareAuthoritySigner` are reference implementations.
A production adapter may use an HSM, cloud KMS, remote signer, or threshold
signing service, provided it produces standard Ed25519 signatures over the exact
canonical bytes.

## Mandatory adapter properties

- The signer returns its public key before use.
- The returned signature is independently verified by the caller.
- The adapter never changes, hashes again, prefixes, serializes, or truncates
  the supplied message.
- `key_id` is stable, non-secret, and suitable for audit logs.
- Timeouts, authorization failure, and malformed signatures fail closed.
- Signing requests are authenticated and protected against replay at the remote
  service boundary.
- Private material is non-exportable where the platform supports it.
- Audit logs identify the protocol domain, request digest, key ID, result, and
  operator or workload identity without logging signed payloads containing
  sensitive data.

## Key separation

Use distinct hardware identities for:

1. credential administrators;
2. credential acceptance;
3. scientific authority receipts;
4. transactional outbox delivery;
5. database epoch promotion;
6. external authority-write lease issuance; and
7. independent transparency witnesses.

The database-epoch and fenced PostgreSQL stores reject cross-domain key reuse.
The lease-issuer key must remain outside the database host and must not be reused
as a scientific actor, receipt, acceptance, epoch, witness, or outbox-delivery
key.

## Availability and recovery

Hardware protection must not create an unrecoverable single point of failure.
Maintain a governed recovery plan with independently controlled backup keys or
threshold shares. Rotation should preserve every historical public key needed
for verification while removing obsolete private authority.

A lost private key does not justify editing old signed records. A compromised
key requires a governed incident response, a new epoch or service-key
transition, and explicit compromise evidence.

## Unix-domain remote signer adapter

The API can delegate outbox-envelope signing to a local Unix-domain signer
agent. The service sends one newline-delimited JSON request:

```json
{
  "protocol": "mycelix-authority-signer-v1",
  "key_id": "pkcs11:slot-7:outbox",
  "message_hex": "<exact canonical bytes>"
}
```

The agent returns the same protocol and key ID, its Ed25519 public key, and a
signature. The caller rejects protocol, identity, public-key, size, timeout, or
signature mismatches. The socket path must be absolute and access-controlled by
the operating system.

Configure exactly one outbox signing mechanism:

```text
DESCI_AUTHORITY_OUTBOX_SIGNER_SOCKET=/run/authority-signer/outbox.sock
DESCI_AUTHORITY_OUTBOX_SIGNER_KEY_ID=pkcs11:slot-7:outbox
DESCI_AUTHORITY_OUTBOX_SIGNER_PUBLIC_KEY=<64-hex-character-ed25519-public-key>
DESCI_AUTHORITY_OUTBOX_SIGNER_TIMEOUT_MILLIS=5000
```

The adapter intentionally does not define authentication beyond the protected
Unix socket. A production agent should additionally bind peer credentials,
workload identity, key policy, request rate, and protocol-domain allowlists.

## Current implementation boundary

This tranche supplies the signer capability, an independently verified Unix
socket adapter, and the authority-write lease protocol. It does not ship a
vendor-specific PKCS#11, cloud-KMS, TPM, or threshold-signing agent. The remote
agent and external lease issuer remain deployment responsibilities and require
integration tests against the selected hardware or service before production.
