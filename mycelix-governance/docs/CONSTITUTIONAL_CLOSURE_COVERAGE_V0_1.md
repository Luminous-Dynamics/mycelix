# Constitutional Closure Coverage v0.1

Status: experimental, non-activating lifecycle adapter.

This tranche exists because a retained closure record is not always current lifecycle authority.
If admissible finality evidence later appears inside an interval previously declared complete,
`constitutional-temporal-provenance` correctly preserves both the closure and the contradictory
evidence and raises a temporal integrity fault. A lifecycle caller must therefore not ask only
whether a historical watermark covered `R-1`.

## Decision surface

`assess_pre_revocation_coverage(state, R)` returns one of:

- `EmptyPreRevocationInterval`: `R == 1`; no positive earlier effective sequence exists;
- `Open`: current accepted closure does not cover every admissible earlier effective sequence;
- `Closed`: a currently authoritative closure covers `R-1`, and the exact closure object plus
  finality-domain/policy/observation-order provenance is returned;
- `IntegrityFault`: temporal assumptions for the domain are contradictory; lifecycle
  terminalization must halt even though historical closure records remain inspectable.

`permits_terminal_revocation()` is deliberately fail-closed: only `EmptyPreRevocationInterval`
and `Closed` return true.

## Ordering rule

Closure coverage compares only quantities in the finality/effective-order domain:

`closure.closed_through_effective_seq >= revocation_effective_seq - 1`

Verifier observation order is not used to decide legal precedence. This remains true when the
003B3 order policy is `Independent` and the numeric values of effective and observed counters are
not comparable.

## Fault dominance

An active temporal integrity fault is checked before the `R == 1` shortcut and before historical
closure coverage. This is intentionally conservative: once the finality domain's accepted closure
assumptions are internally contradictory, lifecycle code must not manufacture fresh terminal
authority from retained records until reviewed recovery/remedy establishes a new trusted state.

## Evidence retention

The adapter never deletes or rewrites closure/finality evidence. `Closed` returns the complete
`EvidenceClosure` value rather than only an integer watermark so downstream `ClaimResolutionReceipt`
work can cite the exact proof object and policy version that justified terminalization.

## Boundaries

This crate does not authenticate closure signatures, witness authority, consensus commitments, or
Holochain state. It consumes only a `TemporalEvidenceState` whose evidence has already passed the
003B3 semantic boundary. It does not implement claim lifecycle transitions; #1427 remains the owner
of `PendingExecutable`, `BlockedAwaitingEvidenceClosure`, `RejectedConflict`, `RevokedClosed`, and
`IntegrityHalted` semantics.

No runtime/Holochain activation is implied by this tranche.
