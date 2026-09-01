# mycelix-sovereign-access

Portable protocol types and policy invariants for **accountable person-linked access** in the Sovereign Intelligence Fabric (SIF).

## Constitutional rule

> An identifiable lookup MUST create an accountable receipt. The subject MUST be notified immediately by default. Delayed notice is a separately authorized, time-bounded exception and MUST eventually become visible.

This crate is intentionally small and wire-neutral. It does not depend on Holochain, Xenia, Symthaea, a specific signature suite, or a legal jurisdiction.

## v0.1 vertical slice

```text
requester
   |
   v
QueryCapability
   |  validate purpose / authority / expiry / budget / disclosure ceiling
   v
local policy evaluation ----------------------------+
   |                                                |
   +--> AccessDecision::Denied                      +--> AccessDecision::Allowed
   |        disclosure = None                            minimal disclosure
   |                                                |
   +--------------------+---------------------------+
                        v
                  AccessReceipt
                        |
        +---------------+----------------+
        |                                |
        v                                v
NotificationPolicy::Immediate     NotificationPolicy::Delayed
        |                         + independent authorization
        |                         + release_at
        |                         + mandatory_release_at
        +---------------+----------------+
                        v
               CitizenNotification
                        |
                        v
              inspect / contest / prove
```

## Required invariants

1. **Every person-linked lookup is budgeted.** A `PersonLinked` capability with a zero query budget is invalid.
2. **Aggregate is not a back door.** `AggregateOnly` capabilities cannot carry a person-query budget and cannot request evidentiary artifacts.
3. **Purpose and authority are explicit.** Empty purpose, authority, policy, or authorization-proof references are invalid.
4. **Disclosure is bounded.** A receipt cannot disclose more than its capability permitted.
5. **Denied lookups disclose no subject data.** The denied attempt still has a receipt.
6. **Immediate notice is the default representation.** The notification becomes available at lookup time.
7. **Delay is a positive authorization.** It requires a reason code, authority, approver(s), proof reference, release time, and hard mandatory-release deadline.
8. **Delay cannot become disappearance.** `release_at` may never exceed `mandatory_release_at`.
9. **Receipts bind policy and provenance.** A receipt carries decision-proof, session-provenance, and receipt-proof references plus optional append-only predecessor commitment.
10. **Subjects can contest.** Citizen notifications require a stable contest route/reference.

## Integration responsibilities

### Mycelix

- Persist receipts in append-only / validation-governed records.
- Resolve subject notification endpoints without publishing sensitive lookup contents globally.
- Verify identity/authority credentials and policy references.
- Deliver citizen notifications and expose inspect/contest workflows.
- Provide independent oversight / threshold-authorization policies for delayed notice.

### Xenia

- Authenticate requester and endpoint sessions.
- Bind `session_provenance_ref` to a replay-protected, signed/PQC-secured session transcript.
- Carry revocation and operator-role state into authorization decisions.
- Never treat transport authentication alone as sufficient authority to query a subject.

### Symthaea

- Evaluate local/federated predicates without requiring raw-data centralization where possible.
- Produce verifiable-computation / ZK references when an applicable proof circuit exists.
- Detect anomalous lookup patterns, purpose drift, query-budget abuse, and suspicious delayed-notice behavior.
- Treat model inference about a person as person-linked access when the inference can materially affect that person.

## Non-goals for v0.1

- No claim of legal compliance or lawful authority in any jurisdiction.
- No signature or ZK implementation in this crate.
- No notification transport.
- No global public ledger of sensitive receipt contents.
- No claim that an `authorization_proof_ref` is valid merely because it is non-empty; integrators MUST cryptographically verify referenced evidence.
- No bulk surveillance API.

## Test status

The source includes unit tests for the core policy invariants and serde wire round-tripping. They still require execution in repository CI / the project Nix environment before this tranche should be merged.
