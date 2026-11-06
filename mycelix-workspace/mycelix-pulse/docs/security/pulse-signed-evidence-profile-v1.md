# Pulse signed evidence profile v1 — design specification

**Status:** proposed profile / schema only  
**Schema:** `pulse-signed-evidence-bundle-v1.schema.json`  
**Related issue:** #5863  
**Not yet implemented:** signature generation/verification, trust-root validation, COSE/SCITT registration, or runtime capture.

## 1. Threat and claim model

The current multi-conductor v2 auditor checks a partial-order history and exact structural links among host grant/call observations, source-chain authorship, receiving validation, DHT observer state and remote signals. It does not authenticate the strings or receipts supplied by an exporter.

The next trust progression must be explicit:

```text
schema-valid
  -> structurally consistent
  -> role-authenticated statement
  -> issuer trusted for that role and predicate
  -> optional transparency registration verified
  -> policy permits relying on the statement now
```

Never collapse these layers into a single `verified=true` value.

A signature proves that a particular key signed particular bytes under a given cryptographic profile. It does not prove that the asserted event occurred, that a conductor was honest, or that a permission is currently valid. A SCITT receipt proves that a signed statement was registered under a Transparency Service's policy and VDS. It does not validate the truth of the statement payload.

## 2. Standards composition

### in-toto / DSSE route

Use the in-toto Attestation Framework statement/predicate model for the role-specific semantic payload. A DSSE envelope authenticates the exact payload bytes and payload type. The verifier must:
1. verify the DSSE signature over the exact decoded payload byte sequence using DSSE pre-authentication encoding;
2. only after at least one permitted signature verifies, parse the payload as an in-toto Statement;
3. check `_type`, `subject`, `predicateType`, schema and predicate;
4. require the signed `issuer_key_id` to match the exact verifying key;
5. bind subject digests and role claims to the trust policy;
6. validate the predicate's exact event/lineage semantics.

The first Rust v1 verifier currently uses a deliberately narrow **single role-signature per attestation** rule. It rejects multiple signatures as an incomplete/unsupported threshold configuration rather than guessing whether any-one, all-of, or quorum semantics were intended. Multi-issuer threshold policy requires an explicit later profile version.

Do not parse and reserialize JSON before signature verification. The signature covers the exact payload bytes.

in-toto separates predicate, statement, envelope and bundle. Use its versioning/monotonic extension rules deliberately; a security-critical Pulse predicate remains explicitly versioned and closed. Unknown critical predicate types or required claims must never be interpreted as allow.

### SCITT / COSE route

For high-value qualification receipts, evaluate an existing RFC 9943 SCITT Transparency Service and VDS. The selected SCITT Signed Statement uses the registered media type `application/scitt-statement+cose`; receipts use `application/scitt-receipt+cose`.

The relying party must verify:
- the COSE_Sign1 signature and protected issuer/subject claims;
- the issuer key against the role-specific trust policy;
- the statement subject and exact statement digest;
- each COSE receipt against a separately trusted TS key and the selected receipt profile;
- inclusion and any supported consistency/non-equivocation proof;
- the TS registration policy and registration time.

A Signed Statement may be registered with multiple independent TSs, producing multiple receipts. Preserve each receipt as a separate item. Never silently replace a receipt from one TS with another.

SCITT defines the signed-statement / receipt formats and VDS transparency architecture; it does not provide Pulse's issuer trust policy, conductor semantics, or application-level truth evaluation. Those remain Pulse profile responsibilities.

## 3. Role separation

Every role-specific observation is signed by an issuer authorized for that role. The bundle does not let the aggregator sign all observations.

| Role | Predicate subject | Expected evidence |
|---|---|---|
| `host_authority` | exact conductor and cell | host grant ActionHash, grantor/grantee, function scope, authority index, host allow/deny, revoke ActionHash |
| `action_author` | exact authoring agent and cell | ActionHash, entry type, canonical payload digest, source-chain sequence |
| `validator` | exact receiving conductor, DNA and integrity-WASM | ActionHash, author conductor, accepted/rejected disposition, integrity-WASM digest, validator result reference |
| `observer` | exact observer conductor/cell | ActionHash, accepted validator statement, projection state/epoch, ancestry decision |
| `client_outcome` | caller and logical operation | operation ID, attempt ID, acknowledgement/unknown/rejected/committed outcome |
| `independent_auditor` | verifier build and evidence bundle | input digests, verification policy, findings and claim ceiling |

The issuer's signed predicate must contain its issuer role, issuer identity, event ID and exact subject digests. The unprotected bundle metadata is not an authority source.

## 4. Bundle schema

The proposed JSON schema provides a transport/indexing wrapper for:
- exact repository/source/trace/tool commitments;
- conductor-specific DNA, CellId and integrity-WASM lineage;
- a separately identified and content-addressed trust policy;
- role-separated attestations;
- either DSSE/in-toto envelope bytes or SCITT COSE Signed Statement bytes and optional COSE receipts;
- causal edges referencing the attestation IDs that support them;
- an explicit bundle claim ceiling.

The schema does not cryptographically verify an envelope or parse CBOR/COSE. A schema-valid bundle must still pass the cryptographic and semantic verifier. There is no default trust path.

The schema is intentionally strict on the wrapper. The payload and COSE internals are validated by their standard-specific parsers after signature verification.

## 5. Trust policy

The bundle's trust-policy reference and the supplied trust-policy document must agree on the policy digest, policy ID, semantic version, and root-keyset ID. Key IDs must be nonempty and unique within the policy. The root-keyset ID is a consistency label; it does **not** authenticate the trust policy by itself.


The trust policy is an independently authenticated input (the first local verifier only digest-binds it to the bundle and reports trust relative to the supplied policy; root-signature/currentness verification remains P2). It binds:
- issuer key ID and public key/certificate chain;
- role and predicate types permitted for that key;
- agent/conductor identity and DNA lineage;
- accepted workflow/tool/validator build profiles;
- key validity interval, rotation and compromise/revocation status;
- historical validation time and current policy effective time;
- permitted transparency-service issuer keys and receipt profiles.

A statement's own `issuer_role` cannot grant itself that role. The trust policy must authorize the key for the role.

For historical evidence, preserve the distinction between:
- signature valid at statement time;
- trusted timestamp / proof-of-existence time;
- later key compromise or revocation information;
- whether current policy permits relying on the historical claim now.

This should compose with #1354; do not build a second long-term archival-renewal architecture.

## 6. Privacy and publication

Do not sign or publish:
- CapSecret bytes or raw authentication tokens;
- message plaintext, decrypted subject/body, or unnecessary recipient metadata;
- private keys;
- unredacted stable agent identities unless the policy and participants explicitly authorize disclosure.

Where public transparency is useful, sign a commitment to a restricted evidence subject and publish only the public statement fields needed for audit. The relying party must be able to retrieve the exact restricted subject and verify its digest, or return `UnverifiableSubject`.

A digest is a commitment to bytes, not proof of the bytes' truth or permission to disclose their underlying content.

## 7. Verification result taxonomy

The verifier must return at least:
- `STRUCTURAL_ONLY`: schema and structural checks only;
- `ROLE_AUTHENTICATED`: signatures valid and issuer keys authorized for their roles;
- `TRANSPARENTLY_REGISTERED`: one or more trusted SCITT receipts verify against the exact signed statement;
- `INCOMPLETE`: required evidence, key, subject bytes, trust material or causal reference missing;
- `FAIL`: signature, role, subject, lineage, receipt or semantic contradiction;
- `HISTORICALLY_VERIFIED`: historical signature/evidence valid under a named policy/time, but current use remains a separate decision;
- `INDETERMINATE`: current policy cannot decide because key status, compromise time or historical material is unavailable.

The bundle's `claim_ceiling` is metadata, not a self-awarded trust level. A verifier derives the actual result from the cryptographic and semantic checks it performed.

## 8. Required tests before use

- signature mutation and one-byte payload substitution;
- wrong key / wrong role / wrong predicate / wrong source commit;
- action hash, payload digest, DNA, CellId or integrity-WASM mismatch;
- validator statement with a rejected disposition;
- valid signature but missing exact accepted validator statement;
- replayed event ID, grant ID, logical operation ID or attempt ID;
- SCITT receipt from a different statement or untrusted TS key;
- invalid inclusion proof, inconsistent VDS proof, and receipts from two independent TSs;
- stale trust policy and key revocation / rotation at distinct historical times;
- subject unavailable or subject differs from committed digest;
- public statement that attempts to leak secret or message data.

No cryptographic pass is claimed until a second implementation verifies the produced statement/receipt bundle and negative controls fail as expected.

## 9. Sources

- in-toto Attestation Framework v1.2: https://github.com/in-toto/attestation/tree/main/spec/v1
- SCITT RFC 9943: https://www.rfc-editor.org/rfc/rfc9943.html
- COSE Receipts RFC 9942: https://www.rfc-editor.org/rfc/rfc9942.html
- #5863: role-separated signed lifecycle evidence and optional SCITT transparency
- #4657: durable retrieval of exact attested subjects
- #2069: frozen GitHub attestation verifier execution profile
- #1354: long-term validation and cryptographic renewal
