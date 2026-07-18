# Mycelix Marketplace status

**Updated: 2026-07-14**

## Classification

Mycelix Marketplace is a **research-grade pre-alpha under stabilization**.

The repository has a substantial Rust/Holochain backend and an extensive SvelteKit frontend. The immediate goal is not additional feature breadth. It is to establish one honest, reproducible, adversarially defensible marketplace path.

## Supported stabilization target

Two agents must be able to:

1. install the same hApp bundle;
2. create and discover a listing;
3. create a transaction bound to that listing and its seller;
4. confirm, ship, deliver, and settle through valid state transitions;
5. restart and recover their state;
6. leave one authorized review; and
7. reproduce the journey in mandatory CI.

A second required scenario must file a dispute, assign eligible arbitrators, accept one vote per assigned arbitrator, and finalize a uniquely derivable result.

## Current blockers

- Frontend wrappers and backend zome exports are not yet one canonical contract.
- Several product surfaces refer to zomes or functions not included in the marketplace DNA.
- Security-sensitive invariants are still enforced in coordinator code instead of—or in addition to—integrity validation.
- Reputation mutation is not yet fully derived from authenticated marketplace events.
- The live multi-agent harness is not yet the release gate.
- Payment/finance and optional identity/knowledge roles are not self-contained in this repository.

## Change policy during stabilization

Accepted:

- security and integrity fixes;
- frontend/backend contract convergence;
- deterministic test fixtures;
- clean-checkout build and CI fixes;
- removal or explicit quarantine of mocks;
- evidence and documentation corrections.

Deferred until the trust loop is proven:

- new marketplace verticals;
- additional scoring dimensions;
- new payment methods;
- broader federation features;
- stronger Byzantine-tolerance claims.

Historical status and completion reports remain useful as development history, but they are superseded by this file and `docs/STATUS_AND_CLAIMS.md` for present-tense claims.
