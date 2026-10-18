# CSCP Import Replay and Idempotency Contract

**Status:** research contract, with an in-memory Rust reference implementation in draft PR #5182.
**Tracker:** [CSCP-REPLAY-001](https://github.com/Luminous-Dynamics/mycelix/issues/5190).
**Parent:** [Creator Sovereignty Control Plane RFC](CREATOR_SOVEREIGNTY_CONTROL_PLANE_RFC.md).
**Research checked:** 2026-10-18.

## Executive thesis

A retry is not a new economic event. When the same distributor statement, rights export, or contributor record is delivered twice—due to API retries, file re-import, process restart or operator action—it must not silently become two royalties, two split claims, two permissions or two cash-reconciliation links.

The paradigm shift is from "import the file again" to **deduplicate by the identity of the observed source and transformation, then compare the resulting effect**. It must distinguish a true retry from an intentional reprocessing under a newer schema/parser/mapping rule. The source payload and transformation versions are part of that distinction.

## Processing identity

The current experimental key includes a domain-separated digest over:

- source-system ID and optional source-record ID;
- schema name and exact schema version;
- parser name and exact parser version;
- raw-payload BLAKE3 digest and exact byte length;
- sorted field source paths and typed treatments;
- target paths plus mapping-rule IDs and versions for mapped fields;
- reason codes for retained-raw and unsupported-retained fields.

The key intentionally excludes import timestamp and raw-storage locator. Importing the same bytes later, or moving them between controlled storage locations, should not create a second logical transformation. The byte digest is calculated from exact observed payload bytes; the locator remains provenance metadata, not payload identity.

A schema/parser/mapping upgrade produces a new key. It is treated as a new explicit processing revision, not as a retry of the previous interpretation.

## Result fingerprint and decision

A second, domain-separated fingerprint binds the processing outcome:

- parser inventory state: complete-as-reported, partial or unknown;
- result class: imported, imported-with-warnings or rejected;
- before and after passport snapshot digests;
- sorted diagnostics, including severity, optional source path, code and detail.

The reference ledger has three outcomes:

| Condition | Result | Required behavior |
|---|---|---|
| Processing key is unseen | Recorded | Store key/result fingerprint in the ledger |
| Same key and same result fingerprint | AlreadyPresent | Do not create a second ledger entry or effect |
| Same key but different result fingerprint | ResultConflict | Fail closed and leave the ledger unchanged |

A conflict is not a reason to overwrite the old receipt. Retain both source/provenance evidence and route the changed result to review. If the parser or mapping intentionally changed, the new version should produce a new key and a reviewable reprocessing event.

## Critical boundary: the current code is not durable

Draft PR #5182 contains a small in-memory ImportReplayLedger plus source-level tests for retries, changed bytes, parser/mapping revisions and result conflicts. It has **not** been verified by a passing Rust test run at the exact current PR head.

The ledger is not integrated with the database import transaction and cannot guarantee exactly-once effects across process restarts or concurrent workers. In production, the durable key and canonical mutation must commit atomically under a unique constraint or equivalent transaction boundary. If a mutation commits but the acknowledgement is lost, a retry must read the committed receipt and must not apply the mutation again.

## Required transactional algorithm

1. Capture the exact source payload and verify its digest/length.
2. Validate source schema/profile and compute the processing key before canonical mutation.
3. Begin a database transaction and reserve the key under a unique constraint.
4. If the key already exists, compare the durable result fingerprint. Return the stored receipt for an exact match; abort and create a reviewable conflict for a mismatch.
5. Parse/map in a staged transaction or isolated candidate snapshot; validate IDs, source coverage, split/claim references and permission scopes.
6. Apply canonical changes, write the provenance manifest and result fingerprint, and mark the replay key committed in the same transaction.
7. Commit once. Return the durable receipt, which carries the exact commit/snapshot digest.
8. On retry, timeout or crash recovery, resolve by the durable key/receipt—not by guessing from current source values.
9. Preserve corrections and parser upgrades as explicit new revisions. Never silently rewrite or delete a prior provenance receipt.

If the backing store cannot atomically commit a unique replay key with the mutation, the adapter must use an outbox/inbox or equivalent transactional pattern; a process-local mutex is insufficient when there are multiple workers or restarts.

## Adversarial benchmark gates

These targets are future integration gates, not results of the in-memory prototype.

| Benchmark | Target |
|---|---:|
| Exact retry of same source, schema/parser/mapping and result | 100% AlreadyPresent; no second effect |
| Timestamp/storage locator change only | 100% idempotent when bytes and transformation identity are unchanged |
| One-byte payload change | 100% new processing key |
| Parser/schema/mapping version change | 100% distinct explicit revision key |
| Same key with changed outcome, diagnostics or before/after digest | 100% conflict; zero state/ledger mutation |
| Concurrent duplicates at 1, 10, 100 and 1,000 workers | Exactly one durable effect and one stable durable receipt |
| Crash before transaction commit | Zero partial canonical mutation |
| Crash after commit but before response acknowledgement | Retry returns prior receipt; zero duplicate effect |
| Process restart and failover to a second worker | 100% durable replay-key lookup correctness |
| Duplicate statement/cash-link replay | Zero duplicate financial records or false cash receipts |
| Permission/rights event replay | Zero resurrected grants, duplicate claims or hidden rights-state change |
| Conflict explainability | 100% conflicts retain prior receipt + new source/provenance evidence and explain the difference |
| Private source access | Zero cross-creator exposure of raw payloads or statement metadata |

## Implementation order

1. Compile and execute the current Rust reference model at an exact pinned SHA; correct source errors without weakening the invariants.
2. Add a durable SQLx/Postgres replay-key table with a unique constraint, receipt/result digest, and state-transition audit.
3. Put the replay key, proposed canonical mutation, provenance receipt and resulting snapshot digest within the same transaction.
4. Add concurrency tests and fault injection around begin/commit/acknowledgement; cover duplicate uploads and changed parser versions.
5. Reconcile royalty/revenue statement lines separately from actual cash transactions.
6. Only then expose the import API to DDEX/platform adapters. Keep source exports/manual imports supported even when an external API is unavailable.

