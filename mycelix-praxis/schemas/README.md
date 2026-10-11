# Praxis Schemas

This directory contains schema definitions for Mycelix Praxis.

## Structure

```
schemas/
├── symthaea-advisory-v1.schema.json # Strict Symthaea response structure
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
advice. If a persistent connection returns an oversized body, its owner should
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
