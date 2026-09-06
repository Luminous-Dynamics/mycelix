# Authority Operational Policy Leased Runtime v0.1 — Normative Invariants

Status: **implemented lease-complete composition candidate; deliberately unprovisioned**

## 1. Historical provider remains unchanged

The existing `authority_operational_policy_provider` and its strict #172 projection remain intact. This sibling endpoint exists only for consumers that explicitly preserve a dynamic evidence lease.

## 2. Discovery is candidate-only

The raw candidate provider returns policy semantics plus record references only. It cannot return verified policy receipts, proof receipts, currentness or qualified objects.

## 3. Record proof and adoption proof remain independent

Each exact locally recomputed policy digest/profile is sent separately to the record-proof verifier and institutional adoption-proof verifier. Neither proof domain can substitute for the other.

## 4. Lease-complete pure qualification is local

The coordinator runs `qualify_coverage_policy_leased` and `qualify_context_policy_leased` locally after their proof calls. Their leases are intersected at a final host-time sample before transport.

## 5. Currentness remains downstream

The returned legacy-compatible policy receipts are not current authority. #115 must still prove exact generation-bound `Active` currentness and #116 must join that currentness to the exact semantics.

## 6. Lease envelope grants no authority

`LeasedEvidence<OperationalPolicyCandidateBundle>` is transport only. The consuming current-freshness runtime must still perform #115/#116 and cap final authority to this lease.

## 7. Containment

Workspace-only, absent from `dna.yaml`, no state projection, no lifecycle mutation, no external effects and `operational = false`.
