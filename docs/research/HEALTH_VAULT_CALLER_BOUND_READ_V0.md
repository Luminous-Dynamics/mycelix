# Health Vault Caller-Bound Read and Privacy Hardening v0 (draft)

**Status:** implementation slice under review; not end-to-end qualified.  
**Related work:** [CONSENT-LIFECYCLE-001 / #5240](https://github.com/Luminous-Dynamics/mycelix/issues/5240), [CONSENT-ENFORCEMENT-001 / #5281](https://github.com/Luminous-Dynamics/mycelix/issues/5281), [expiry projection #5271](https://github.com/Luminous-Dynamics/mycelix/pull/5271), and [append-only revocation #5279](https://github.com/Luminous-Dynamics/mycelix/pull/5279).

## Paradigm shift

A user-visible consent flag is not access control. The remote caller must be authenticated by the transport/capability layer; authorization must be reevaluated at the owner's protected read boundary; scope and lifecycle state must be checked before any payload is returned; and an unavailable source must not be replaced by cached authorization.

The current implementation slice uses an **Assigned Holochain capability** for one remote function, pinned to the intended grantee agent. The secret is returned to the owner's local call result for separate private delivery and is not included in the shared ConsentGrant entry. Remote grants default to a **30-day expiry**; indefinite remote access requires an explicit `allow_indefinite = true` API input, while the Personal UI only uses the bounded default. The private offer includes its expiry so the intended grantee can inspect the grant term. The owner-side remote handler also checks actual call provenance, the original grant author, the exact grantee, supported record categories, expiry/not-before, and the owner's current local revocation history. Revocation deletes the specific local CapGrant and appends the existing immutable revocation event. See the Holochain capability model: https://docs.rs/hdk/0.6.1/hdk/capability/fn.create_cap_grant.html and https://docs.rs/hdk/0.6.1/hdk/capability/fn.create_cap_claim.html.

## Critical privacy correction

Holochain entry types default to public unless their entry definitions explicitly set `visibility = "private"` (see https://developer.holochain.org/build/entries/ and https://docs.rs/hdi/0.7.1/hdi/prelude/attr.hdk_entry_types.html). The Health Vault's previous `EntryTypes` enum did not explicitly configure visibility for `HealthRecord`, `Biometric`, `ConsentGrant`, or `ConsentRevocation`. This patch marks all four private so **future entries under the upgraded DNA definition do not publish their payloads to the DHT**.

This does **not** erase or recall health values that were already published under earlier public definitions. Treat those entries as potentially disclosed. Holochain action and link metadata remain public; the current local-index links can still reveal the existence/timing of owner records or consent/revocation events. The default UI creates a bounded 30-day remote grant and never offers indefinite sharing. The old `RecordTypeToRecord` write with the record type in its public link tag was removed for future records, but existing links remain. Full metadata minimization is separate follow-up work.

A DNA upgrade must be planned and tested for existing cells; changing entry visibility is a protocol/data migration concern, not a claim that past public entries become private retroactively.

## Caller-bound read contract

1. The owner's `get_shared_records_remote` endpoint is invoked via an Assigned capability limited to that exact function. No unrestricted read capability is introduced.
2. The endpoint derives requester identity from Holochain call provenance, not a caller-key field in the request.
3. The cited grant must be the owner's original `ConsentGrant` create action; the grantee must match the authenticated requester.
4. The owner evaluates active/not-before/expiry/revocation at read time and rejects unavailable or invalid policy data.
5. Requested categories must be non-empty, bounded, supported, and a subset of the grant's category scope. A wildcard is honored only when the grant explicitly includes it.
6. The local grantee wrapper reads a matching local `CapClaim`, calls the owner, and returns an error on authorization or network failure. It does not turn cached grant state into proof of access.
7. The response contains only requested category rows; no public audit event contains the raw payload. A complete minimal access-receipt design is still outstanding.
8. Revocation prevents subsequent successful remote calls once the owner has committed the capability deletion, assuming the owner is reachable. It cannot recall data already returned, downloaded, screenshotted, or copied.

## Adversarial Sweettest plan added (not yet executed)

The new ignored multi-agent scenario uses an owner, intended grantee, and attacker. It creates synthetic vaccination and medication records, grants only vaccination, verifies only that category returns, rejects medication scope, attempts to use the stolen secret from the attacker's different agent key, revokes twice and compares the returned immutable event hash, rejects a post-revocation read, and verifies the owner effective list is empty.

This is test source, not a test pass. Native tests, WASM build, strict Clippy and multi-agent conductor execution must pass on the exact PR head before describing the path as qualified.

## Frozen qualification targets (targets, not results)

- Zero unauthorized successful reads in the frozen adversarial suite.
- 100% rejection of unsupported/out-of-scope, expired, inactive, revoked, malformed, and unauthenticated requests.
- Zero fail-open decisions on source unavailable or capability denial.
- Zero record payloads published in DHT entries for new writes under the private definitions.
- Zero raw health payloads or secrets in public consent/revocation metadata and logs.
- 100% root/workspace mirror parity.
- 100% deterministic revocation projection across duplicate/replayed events.
- Report p50/p95/p99 owner-side authorization latency, capability denial rate, network failure behavior, response bytes, and cost per correct authorized request; performance never compensates for a safety failure.
- Explicitly test concurrent read/revoke race, restart, stale local CapClaim, duplicate capability claims, expiry boundaries, owner offline/partition, previous public-record migration, and UI secret-handling.

## Industry comparison

HL7 FHIR R5 Consent is the interoperability reference for actors, policy context, purposes, effective periods and permit/deny decisions: https://hl7.org/fhir/R5/consent.html. This code does not yet model purpose or field-level scopes and is not a complete FHIR implementation. The near-term goal is stronger runtime authorization semantics on top of a source-native Holochain capability, followed by an explicit versioned mapping to interoperability vocabulary. No SOTA superiority claim is made until a fixed adversarial suite and comparable latency/resource benchmark are run.

## Remaining limitations

- Consent purpose and field-level scope are not yet represented in the current grant schema.
- Legacy entries created under the old public entry definitions may already have been published; this change cannot undo that.
- Public action and link metadata may still reveal existence and timing; only the old typed `RecordTypeToRecord` link creation is stopped here.
- No durable, minimal access receipt has been added yet.
- This is not a claim of POPIA, clinical, or production security compliance.
