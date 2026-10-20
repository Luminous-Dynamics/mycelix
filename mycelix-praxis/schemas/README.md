# Praxis Schemas

This directory contains schema definitions for Mycelix Praxis.

## Structure

```
schemas/
├── symthaea-advisory-v1.schema.json # Strict Symthaea response structure
├── symthaea-advisory-request-v1.schema.json # Minimized, digest-bound request body
├── examples/
│   └── symthaea-advisory-request-v1.json # Sanitized request-body example
├── vc/                           # Verifiable Credentials schemas
│   ├── EduAchievementCredential.schema.json
│   └── contexts/
│       └── praxis-v1.jsonld      # JSON-LD context
└── dht/                          # Holochain DHT entry schemas
    └── entries.md                # Entry definitions reference
```

## Verifiable Credentials (VC)

Praxis uses [W3C Verifiable Credentials](https://www.w3.org/TR/vc-data-model/) for educational achievements.

### EduAchievementCredential

Represents completion or achievement in a course, tied to:
- **Course**: Which course was completed
- **Model**: Which ML model version assessed the learner (for provenance)
- **Rubric**: Assessment criteria used
- **Score/Band**: Achievement level

See `vc/EduAchievementCredential.schema.json` for full schema.

### JSON-LD Context

The `contexts/praxis-v1.jsonld` file defines our custom vocabulary for:
- Course and model identifiers
- Federated learning provenance
- Achievement scoring

## DHT Entries

Holochain entry types are defined in Rust code (in `zomes/*/src/`), with documentation in `dht/entries.md`.

Key entry types:
- **Course**: Learning content and structure
- **FlRound**: Federated learning round metadata
- **FlUpdate**: Gradient/update submissions (commitments only)
- **VerifiableCredential**: W3C VCs for achievements
- **Proposal**: Governance proposals

## Symthaea Advisory Response v1

`symthaea-advisory-v1.schema.json` defines the strict JSON shape for an optional
Symthaea learning recommendation. It rejects unknown fields and bounds the action,
reason, identifiers, confidence metadata, and content-reference list.

The JSON Schema is only the structural boundary. Runtime Praxis validation must
also match the exact configured model/version, active skill and curriculum revision,
request digest, trusted per-turn action and reason allowlists, and content-ID
allowlist. Schema-valid output is not proof of factual correctness, learning
effectiveness, mastery, credential eligibility, or authorization.

The Rust `parse_symthaea_advice_response` helper enforces a 16 KiB raw-byte
ceiling before deserializing and rejects malformed/unknown-field responses.
The host-only `crates/praxis-symthaea-adapter` crate provides
`read_bounded_symthaea_advice(reader)`, which uses `Read::take(limit + 1)` to
avoid buffering an oversized stream in full, then delegates to that strict
parser. Prefer `read_bounded_and_validate_symthaea_advice(reader, context)`,
which also invokes the trusted semantic validator and returns a
`ValidatedAdvisorySuggestion`; the lower-level decoder alone is not trusted
advice.

The companion `symthaea-advisory-request-v1.schema.json` freezes the strict
request-body fields and bounded action/reason/content-ID allowlists. The
host-only `prepare_advisory_request(policy)` builder generates a fresh
128-bit lowercase-hex `request_nonce` from the OS-provided cryptographic random
source; callers do not supply it. Entropy failure aborts preparation. Reuse can
make an old digest match a later otherwise-identical turn, so prepare once per
exchange and reuse the same `PreparedAdvisoryRequest` bytes/digest for retries;
consume each response once and prepare a new request for a new turn. The nonce
is not authentication. The fixed nonce in the sanitized example is illustrative
only and must never be reused at runtime. The adapter pins its direct
`getrandom` dependency to `=0.2.17`; the wider Praxis workspace currently has
no committed Cargo lockfile, so the full dependency graph remains a build/CI
qualification item. The builder sorts allowed
actions/reasons in protocol order and content IDs lexically, rejects
duplicate/invalid policy inputs and overlong allowlists before copying them,
caps the body at 16 KiB, and derives `input_digest()` from the exact immutable
bytes exposed by `body_bytes()`. The static conformance checker requires both
request and response schemas to match the Rust field sets and enums. Keep this prepared request alive throughout its exchange and use
`read_and_validate_response_once(reader, advisor_enabled, sandbox_active)`.
It binds validation to the same model, version, skill, curriculum revision,
action/reason lists, content IDs, and digest, and rejects a second response
once a bounded body has been received. A reader failure remains retryable with
the same prepared body/digest. Send the body unchanged and carry the digest
separately in authenticated transport metadata rather than a self-referential
body field. The sanitized
`examples/symthaea-advisory-request-v1.json` shows the exact request-body
shape, with illustrative identifiers and no digest or learner history. The
static checker validates its field set/order, action/reason enum membership,
reference bounds, and digest exclusion.

The inspected [Symthaea API router](https://github.com/Luminous-Dynamics/symthaea/blob/b9af191f380d635f0459b24284c1d1fd6a5516aa/src/api/mod.rs)
is a benchmark API, not a Praxis learning-advice endpoint. Do not send these
request bodies to `POST /v1/submit` or assume the API's loopback default
provides authentication. See the Praxis contract documentation for the pinned
source review and transport prerequisites.

If a persistent connection returns an oversized body, its owner should
discard/close the connection rather than reuse a stream with unread body bytes.
Transport timeouts, process authentication, learner consent, and production
call-site wiring remain responsibilities of the host adapter/integration.
JSON Schema cannot enforce a limit on the original wire-body byte length.

## Validation

- **VC schemas**: Validated against JSON Schema draft 2020-12
- **DHT entries**: Validated by Holochain zome validation functions

## Versioning

Schemas follow semantic versioning:
- **Major**: Breaking changes (e.g., remove required field)
- **Minor**: Backward-compatible additions (e.g., add optional field)
- **Patch**: Documentation or clarification

Current version: **0.1.0**

## Tools

Validate VC schemas:
```bash
# Using ajv-cli (npm install -g ajv-cli)
ajv validate -s vc/EduAchievementCredential.schema.json -d examples/credential.json
```

Generate TypeScript types from schemas:
```bash
# Using json-schema-to-typescript
cd apps/web
npx json-schema-to-typescript ../../schemas/vc/*.schema.json -o src/types/
```

## Examples

See `examples/` directory for sample credentials and entry payloads (TODO).

## References

- [W3C Verifiable Credentials](https://www.w3.org/TR/vc-data-model/)
- [JSON-LD 1.1](https://www.w3.org/TR/json-ld11/)
- [JSON Schema](https://json-schema.org/)
- [Holochain Entry Types](https://developer.holochain.org/concepts/entry-types/)
