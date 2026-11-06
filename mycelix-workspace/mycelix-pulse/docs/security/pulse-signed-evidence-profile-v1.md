# Pulse signed evidence profile v1 — design specification

**Status:** proposed profile with a locally implemented DSSE verifier prototype  
**Schema:** `pulse-signed-evidence-bundle-v1.schema.json`  
**Related issue:** #5863  
**Implemented in the local verifier prototype:** Ed25519 DSSE signature verification over exact PAE bytes, signature-before-parse, in-toto statement/subject binding, one-signer-per-attestation role authorization, exact conductor/agent/DNA/CellId/WASM binding, signed event-to-trace semantic binding, evidence-backed causal-edge checks, detached root-signature verification for exact trust-policy bytes, and an out-of-band pinned root/policy-snapshot mode.  
**Not yet implemented:** production signing/key custody, controlled root rotation/threshold transitions, key validity intervals and historical compromise adjudication, COSE/SCITT receipt verification/VDS proofs, a second independent implementation of the root-policy verifier, and runtime conductor evidence capture.

## 0. Current implementation status and claim ceiling — 6 November 2026

The Rust prototype is at `tools/pulse-signed-evidence-audit`. The exact PR #5771 head `8b9a2de05c1c562e6f6135fc2645679e756a6cff` passed the Signed Evidence Audit workflow run #90:
- pinned Rust 1.96.0;
- `cargo fmt --check`;
- per-run dependency lock generation;
- 26 signature, role, subject, trust-policy, causal-edge and negative-control tests;
- `cargo clippy --all-targets -- -D warnings`;
- exact verifier receipt and artifact upload.

Run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/42215970758  
Job: `127885945664`  
Artifact: `13501032365`

The verifier checks the Ed25519 DSSE signature over exact PAE bytes before parsing signed JSON, and requires the signed issuer key ID to equal the exact verifying key. V1 deliberately allows exactly one role-specific signature per attestation. It rejects ambiguous multi-signature envelopes as `INCOMPLETE` rather than inventing any-of/all-of/quorum semantics.

The verifier binds signed predicates to the exact event ID, source/type, payload fields, source commit, trace digest and conductor lineage. It validates request/response, source-action, validator and observer causal edges against separately signed endpoint observations. A signature over a fabricated or contradictory trace event does not pass semantic binding.

**Trust-root boundary:** the legacy CLI mode remains `ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY`. A new opt-in root-pinned mode verifies a detached DSSE signature over the exact trust-policy bytes before parsing them. It requires a relying-party-provisioned pin that fixes the Ed25519 root key fingerprint and the exact accepted policy ID, semantic version, epoch, root-keyset ID and SHA-256 digest. Replayed older policy snapshots fail the exact epoch/digest pin. When this check and the ordinary evidence audit both pass, the result is `ROOT_AUTHENTICATED_POLICY_SNAPSHOT`; this means the policy snapshot was signed by the pinned root, not that the real-world events are true or that any permission is currently valid.

The v1 root mode pins a single root and one exact current policy snapshot. Root rotation is intentionally unsupported in this increment: changing the root or policy snapshot requires controlled out-of-band replacement of the pin. A current-root cross-signature/dual-control rotation chain, rollback-resistant durable state store, threshold policies and historical compromise-time adjudication remain future work. The root pin file is a trust anchor and must never be taken from the evidence bundle or an untrusted download.

**SCITT boundary:** SCITT statement/receipt fields receive structural and digest checks, but COSE signatures, TS issuer keys, receipt authenticity and VDS inclusion/consistency proofs are not verified. Structurally valid SCITT input therefore remains `INCOMPLETE`. No runtime conductor truth is established by the synthetic signing-key fixtures.

## Trust-root lifecycle model and implementation follow-up

A finite TLA+ model has now been added to PR #5771:
- PulseTrustRootLifecycle.tla;
- PulseTrustRootLifecycle.cfg (safe child-policy update);
- PulseTrustRootLifecycle-self-root-mutant.cfg;
- PulseTrustRootLifecycle-rollback-mutant.cfg;
- PulseTrustRootLifecycle-revocation-mutant.cfg.

The model treats the relying party's root pin as out-of-band state. A valid policy update must be signed by the already trusted root, extend the current policy digest, and increase the policy epoch. It separately checks that a currently revoked issuer key cannot authenticate a current decision. It distinguishes current-policy verification from historical signature verification; historical validity is not a permission to use a compromised key for new decisions.

The safe model and all three mutants were executed by the pinned TLC workflow on 6 November 2026 (run #319, see [exact run](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42251431435)). The safe root-policy transition exhausted its state queue; the self-asserted-root mutant violated `OnlyPinnedRootCanAuthorizePolicy`; the rollback mutant violated `PolicyEpochNeverRollsBack`; and the revoked-key mutant violated `NoRevokedKeyAcceptedUnderCurrentPolicy`. This is finite abstract evidence, not cryptographic implementation verification.

The first implementation increment is now in the Rust verifier. It checks the detached root DSSE signature over exact policy bytes before parsing, validates the out-of-band root key fingerprint, and pins the exact current policy digest and epoch. It deliberately does not implement root rotation. The exact-source Rust test run for this implementation is pending; the prior 26/26 DSSE test pass predates these root-mode changes. The detailed implementation/acceptance work remains tracked in [#5895](https://github.com/Luminous-Dynamics/mycelix/issues/5895).

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

### Exact trace-event binding (implemented in the local verifier prototype)

A whole-trace digest alone does not prove that a signed predicate describes a particular event inside that trace. For every event-level role attestation, the verifier now resolves exactly one `trace.events[]` item by `event_id` and requires the signed `predicate.details` to contain the exact closed field set:

- `event_type`;
- `source`;
- `conductor_id`;
- `local_sequence`;
- `evidence_ref`;
- complete `event_data`;
- `agent_key_ref`, `dna_hash`, `cell_id`, and `integrity_wasm_sha256`.

The signed values must match the exact parsed trace event and its conductor lineage. The role must also match the event's permitted source/type: host authority ↔ conductor host grant/revoke/call; action author ↔ source-chain action; validator ↔ validation disposition; observer ↔ projection observation; client outcome ↔ client result. A valid Ed25519 signature over a fabricated or contradictory event claim now fails semantic binding.

The `independent_auditor` role is a trace-level attestation, not a synthetic conductor event. Its event ID is `trace:<trace_id>`, and its signed details bind trace ID, source commit, trace digest, schema digest and tool-build digest.

A causal edge is trusted only when its basis includes DSSE-verified attestations for both exact endpoint event IDs. A known but invalid/SCITT-unverified attestation, or a valid attestation for an unrelated event, cannot support the edge. The signed-evidence bundle schema now requires at least two distinct basis attestation IDs for an edge.

The v1 relation predicates are intentionally narrow:

| Relation | Required event pair | Required semantic binding |
|---|---|---|
| `program_order` | any supported event pair | same conductor and strictly increasing local_sequence |
| `request_response` | host_call → domain_effect / caller_outcome; domain_effect → caller_outcome | exact operation_id and attempt_id |
| `source_action_dependency` | host_grant_issued → capability_claim_accepted | exact grant_action_hash |
| `validator_receipt` | action_authored → validation_disposition | exact ActionHash, author conductor and receiving validator conductor |
| `observer_confirmation` | validation_disposition → projection_observed | exact ActionHash and exact validator_event_id |
| `test_harness_barrier` | synthetic fixtures only | forbidden for runtime_capture |

Unsupported pair/relation combinations are INCOMPLETE; contradictions are FAIL. The DAG cycle check remains separate: acyclic ordering does not make an unsupported or false relation true. The current positive integration fixture signs two events from one conductor and checks a valid program_order edge. Unit regressions exercise each relation predicate and its negative cases.

These checks establish semantic binding between signed bytes and the supplied trace subject. They still do not prove the real-world truth of the trace or authenticate the external trust root.

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

## 5A. Detached root-authenticated policy snapshot v1

The verifier now supports two explicit modes:

1. **Legacy relative mode:** `--trust-policy TRUST.json` verifies issuer signatures relative to that supplied policy and returns `ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY`. It must not be described as root-authenticated.
2. **Pinned-root mode:** add `--root-policy-envelope ROOT-DSSE.json --trusted-root-pin ROOT-PIN.json`. Both arguments are required together. A missing partner is a CLI error; failed root verification never falls back to relative mode.

The root envelope uses payload type `application/vnd.pulse-trust-policy+json;version=1`, exactly one Ed25519 signature and the exact trust-policy file bytes as its Base64 payload. The verifier performs these operations in order:

1. Read the policy and detached envelope as raw bytes.
2. Load the relying-party root pin from a separate, trusted configuration path.
3. Check the pinned key ID against the SHA-256 fingerprint of the pinned 32-byte Ed25519 public key.
4. Verify that the policy bytes match the pin's exact SHA-256 digest.
5. Check the envelope's fixed payload type, exactly-one-signature rule and pinned root key ID.
6. Decode the payload and require byte-for-byte equality with the supplied policy file.
7. Verify Ed25519 over DSSE PAE bytes using the pinned root key.
8. Only after signature success, parse the policy and compare policy ID, semantic version, positive epoch and root-keyset ID with the out-of-band pin.
9. Run the normal bundle/trace/issuer-role semantic audit against that exact policy.

The pin schema is `pulse-trusted-root-pin-v1.schema.json`; the detached envelope schema is `pulse-trust-policy-root-envelope-v1.schema.json`. The trust-policy and bundle schemas now both require `policy_epoch`. The bundle's digest-bound trust-policy reference must match the parsed policy epoch.

**Anti-rollback claim:** v1 does not attempt to discover the latest policy from a remote service. Instead, the relying party pins one exact accepted policy digest and epoch in trusted local configuration. A correctly root-signed old policy is still rejected when it differs from that pin. Updating the accepted policy requires a controlled update to the relying-party pin. The pin itself is the local trust anchor and needs protected deployment/storage, review and backup.

**Deliberate limitations:** one pinned root, one root signature and one exact policy snapshot. No automatic root rotation, cross-signing, threshold/quorum policy, online transparency lookup, durable policy-history ledger, or historical key-compromise adjudication is implemented. The model checks root self-promotion/rollback/revoked-key invariants, but the code does not yet establish a root-transition protocol. SCITT remains structurally checked only.

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
