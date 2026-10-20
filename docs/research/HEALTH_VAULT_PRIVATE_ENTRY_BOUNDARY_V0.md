# Health Vault Private Entry Boundary v0 (draft)

**Status:** source/privacy hardening proposal under review; not a qualified deployment or migration.
**Related issues:** [HEALTH-PRIVACY-001 / #5315](https://github.com/Luminous-Dynamics/mycelix/issues/5315), [CONSENT-ENFORCEMENT-001 / #5281](https://github.com/Luminous-Dynamics/mycelix/issues/5281).
**Stacked on:** [#5279 append-only consent revocation](https://github.com/Luminous-Dynamics/mycelix/pull/5279), which is stacked on #5271.

## Finding

The Health Vault integrity enum previously declared `HealthRecord`, `Biometric`, and `ConsentGrant` without explicit private visibility. Holochain's entry-type contract makes private visibility an explicit entry definition; a Rust comment calling a record private does not configure the DHT visibility. The coordinator also created index links from the agent to health/biometric/consent entries and linked a record-type tag to a record hash. Link actions expose correlation metadata even when their targets contain private entry content.

This is an architecture/configuration finding, **not evidence that a particular user or attacker retrieved data**.

## Changes in this slice

- Marks `HealthRecord`, `Biometric`, `ConsentGrant`, and `ConsentRevocation` entry definitions explicitly private.
- Removes coordinator writes that create public owner/record-type links for these records.
- Changes owner-local listing functions to `ChainQueryFilter` queries on their own source chain with private entry content included.
- Rejects creation or deletion of the five former public health-vault index link types.
- Keeps the old link enum variants in place (no reordering); they remain defined only for compatibility/validation but are no longer a supported indexing mechanism.
- Makes a consent grant immutable. Updating an existing grant cannot create a new action hash that escapes the revocation event's binding; changing authorization requires a new grant.
- Adapts revocation validation to use the referenced creation action header, because validators cannot assume they can fetch private grant contents from shared DHT state. The owner-side coordinator still reads and checks the actual local grant content before writing a revocation.

## What this does and does not hide

Private entry definitions keep entry **contents** on the owner's source chain. They do not make the existence and all metadata of source-chain actions anonymous. Holochain action headers, action hashes, author, timestamp, and entry-type/index information can remain observable. Removing the index links reduces avoidable leakage, especially explicit record-category tags and owner-to-record graph links; it is not an anonymity guarantee.

If the threat model requires concealing whether an action is a health record vs biometric vs consent, a later protocol should consider a unified encrypted envelope and a local encrypted index. That is a distinct wire/schema change and should be benchmarked rather than claimed here.

## Critical migration boundary

Changing entry visibility modifies the integrity definition and therefore the DNA definition/hash. It does **not** retroactively privatize entries already published on an older DNA/network. Published content cannot be made secret simply by changing code in the next version.

Before deployment:
1. Build and install the new DNA as an explicitly versioned privacy boundary; do not assume it is a drop-in migration of existing source-chain data.
2. Provide an owner-controlled, user-visible export/re-encryption path for data the owner still has locally.
3. Mark old records as `LegacyPublicExposure` or an equivalent visible warning in migration tooling; do not label them private simply because the new entry definition is private.
4. Do not automatically copy old records, grant metadata, or public links to another network without a user-controlled migration and disclosure decision.
5. Test that unauthorized peers cannot retrieve **new** private entry contents from the DHT and that the owner can still query their own data after restart.

## Frozen qualification plan

The mirrored ignored Sweettest suite now includes:
- `test_health_private_entries_are_owner_local`: owner creates and reads private content; another agent's own source-chain query returns no owner records.
- `test_health_consent_revocation_is_idempotent_and_regrant_is_new`: grant -> revoke -> repeated revoke returns the same event -> effective list excludes the old grant -> an explicit new grant creates a distinct effective action.

These tests have been added but **not executed** in this authoring session. The locality test checks source-chain query isolation; it is not a direct remote-DHT fetch test and must not be described as one.

- Confirm all four sensitive entry definitions declare private visibility.
- Native tests prove the coordinator selectors use the local chain and private entries are decodable by the owner.
- Native integrity tests reject all legacy index-link create/delete actions and reject updates/deletes that would change a grant or erase revocation evidence.
- Multi-agent conductor test: owner creates a health record and consent; attacker searches the shared DHT and cannot fetch the content; owner can still list and decode it locally.
- Multi-agent disclosure test remains separate: an authorized grantee should receive only the selected content via an explicit owner-controlled function call, never by direct DHT access.
- Root/workspace source mirrors and build artifacts must correspond to the same exact source commit.
- Exact-head native tests, WASM build, strict Clippy and conductor tests must all pass; queued or skipped is not PASS.

## Proposed benchmarks (targets, not results)

- Zero newly published raw health/biometric/consent entry contents on the shared DHT.
- Zero new public health index links, including record-type links.
- 100% of migration outputs accurately classify legacy-public vs new-private records.
- Zero unauthorized responses containing protected record payloads in the frozen multi-agent suite.
- Report p50/p95/p99 owner-local query cost and the remaining observable metadata fields; avoid a single global privacy score that hides content exposure.
- Zero privacy-gate regressions accepted in exchange for latency or throughput gains.

## Research references

- Holochain entry visibility: https://developer.holochain.org/build/entries/
- Holochain source chains, public actions and private entry content: https://developer.holochain.org/build/working-with-data/
- Holochain capability-based authorization for the next grantee read boundary: https://developer.holochain.org/build/capabilities/

This note is a protocol and qualification contract. It does not claim production readiness, POPIA compliance, retroactive deletion of legacy public data, complete metadata privacy, or end-to-end revocation enforcement.
