# Health Vault Append-Only Consent Revocation v0 (draft)

**State:** implementation slice under review; not a qualified end-to-end authorization protocol.  
**Related issue:** [CONSENT-LIFECYCLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5240).  
**Stacked on:** expiry projection in [PR #5271](https://github.com/Luminous-Dynamics/mycelix/pull/5271).

## Contract

A consent grant is identified by its original `ConsentGrant` creation action hash. Revocation is a separate immutable `ConsentRevocation` entry referencing exactly that action. The Holochain creation action supplies the event timestamp; the revocation payload does not duplicate a caller-supplied time or reason.

The coordinator rejects a revoke request unless the target is the owner's original ConsentGrant create action. The integrity validator for the **private** revocation event checks signed public action metadata (original action is a create and the revocation author matches the original action author); it does not try to decode the private ConsentGrant payload on DHT peers. Private grant content is intentionally unavailable to other peers' validators. Consent and revocation history links must be authored on the same agent base and cannot be deleted; the coordinator also ignores foreign-authored grants even if a legacy link exists. Revocation entries cannot be updated, and Health Vault deletes are rejected. Updates are bound to the original action author.

The coordinator operation is idempotent on the owner's local source-linked history: repeating a revocation request returns the existing event rather than creating a second one. A new permission must be a **new grant**. Updating/reactivating a revoked grant cannot clear the revocation event.

The effective-list projection now derives effectiveness from all of:
- `active == true`;
- `created_at <= evaluation_time`;
- `expires_at == None` or `expires_at > evaluation_time`;
- no owner-authored revocation event bound to the grant's original creation action.

The new history endpoint returns the owner's revocation records so filtering the effective list does not erase the audit trail.

## Authority and privacy boundaries

This slice is **owner-side lifecycle accounting**, not proof of immediate revocation enforcement at every grantee or downstream data consumer. A consumer that cannot observe or validate fresh owner revocation evidence cannot safely infer that the grant remains effective. Protected reads and presentations must independently fail closed on stale/unknown policy state, and that enforcement is still a separate qualification requirement.

The revocation event contains only the grant action hash; it deliberately contains no raw health value, reason, identity document, key or credential payload. The event hash is still correlatable metadata, so the deployment's entry visibility and DHT publication policy must be reviewed before production use. This change does not claim that prior disclosures or recipient-held copies can be recalled.

## Frozen adversarial qualification plan

Before calling the lifecycle qualified, add and run native, WASM and multi-agent tests on the exact PR head:

1. Grant owner revokes a grant and the owner projection no longer reports it effective.
2. Repeating the revoke operation returns the same revocation action hash and creates no new semantic effect.
3. A different agent's direct zome call cannot revoke the victim's grant.
4. A revocation targeting a non-grant action, an update action rather than the original create action, or a missing action fails closed.
5. Direct attempted update/delete of a revocation entry and delete of its history link fail integrity validation.
6. Owner updates cannot be authored by a different agent; changing `active` after revocation does not restore effectiveness.
7. Expiry before, exactly at and after evaluation time is checked alongside revocation; event replay and event-order permutations produce the same revoked result.
8. A consumer with unavailable or stale revocation evidence does not present access as verified/current.
9. Root and `mycelix-workspace` copies are byte-identical, and the final manifest/lock/WASM artifacts are built from the same exact source SHA.

## Proposed benchmark targets (not results)

- 100% pass on the frozen lifecycle and negative-control cases.
- Zero successful unauthorized revocation attempts.
- Zero revoked grants returned by the owner's effective-list endpoints.
- Zero event/link update or delete operations that erase a qualified revocation.
- 100% deterministic projection agreement for replayed or reordered revocation events.
- Report p50/p95/p99 source-local evaluation latency and cost per correct decision, without treating performance as compensation for security failures.
- Before any end-to-end claim, test network partitions, lagging DHT visibility, cached policy, consumer restarts, and partial failures at every protected read/presentation boundary.

## Industry interoperability comparison

HL7 FHIR R5 Consent models choices that permit or deny actors to perform actions under specific policy contexts, purposes and periods ([FHIR R5 Consent](https://hl7.org/fhir/R5/consent.html)). This Mycelix record remains a smaller source-native representation and is **not** a complete FHIR implementation. Future work should map actor/subject, purpose, provision/data scope, effective period, controller and status semantics into an interoperability projection rather than silently equating a single `active` boolean with policy compliance.
