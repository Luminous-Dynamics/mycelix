# Legacy Constitution Quarantine

Status: **P0 containment for binding governance**

The legacy `constitution_integrity` / `constitution` zome source remains in the repository and governance Cargo workspace for history, tests, and migration. It is intentionally **not packaged in the binding governance DNA**.

## Why it is quarantined

The legacy API predates the new institutional-authority and binding-voting model. Its current coordinator/integrity contract contains authority semantics that are not strong enough to serve as a constitutional trust root:

- initial charter legitimacy is not derived from immutable network configuration;
- a higher-version charter can enter the create path without proving the exact amendment lineage that authorized it;
- amendment ratification depends on the legacy voting surface rather than the new snapshot-bound binding tally;
- governance-parameter mutation accepts a `proposal_id` reference without proving that the exact proposal authorized the exact parameter/value transition;
- new governance parameters have a bootstrap path that does not establish institutional authority.

Keeping this zome callable while introducing `constitution_authority` would create two competing constitutional authority planes. The safer migration rule is one binding authority plane at a time.

## Runtime effect

The new `constitution_authority` zome is the only constitutional authority zome packaged in the binding governance DNA.

Legacy source code is **not deleted**. Existing historical networks that were created with the legacy DNA continue to have their existing DNA hash and behavior. This change affects newly packed/installed governance DNA from this branch.

The execution coordinator's legacy `GovernanceAction::UpdateParameter` path currently cross-calls the zome named `constitution`. With the legacy zome quarantined, that action fails explicitly because its target is unavailable. It does not report a phantom successful parameter change.

This temporary loss of parameter mutation is intentional. Parameter mutation must not be re-enabled until it is expressed as an exact constitutional transition governed by the verified authority chain.

## What remains available

- proposal and deliberation records;
- binding-voting work as it becomes verified;
- threshold-signing containment;
- councils/jurisdiction/budgeting source paths subject to their own authority reviews;
- DNA-bound `constitution_authority` genesis lookup;
- read/audit of source-level legacy constitution code outside the packaged authority surface.

## Re-enable criteria for constitutional mutation

Do not restore a mutable constitutional zome to the binding DNA merely by renaming the legacy functions. A replacement amendment adapter must prove, fail-closed, all of the following:

1. the current parent constitution is derived from the exact DNA-bound genesis and a unique verified lineage;
2. the proposed child statement increments the parent version exactly once and binds the exact parent digest;
3. the amendment payload is canonically committed;
4. the parent constitution's exact amendment policy governs the transition;
5. the proposal carries the exact institution/jurisdiction/rulebook authority context;
6. the electorate snapshot and civic-right verifier policy are constitutionally authorized;
7. the exact binding tally is verified under the parent-authorized voting profile;
8. the exact threshold/institutional authorization is verified under the parent-authorized authority profile;
9. the transition nonce/replay domain is fresh;
10. a competing verified child for the same parent causes ambiguity/denial rather than timestamp or DHT-arrival selection;
11. the resulting statement/transition is append-only and independently replayable;
12. challenge/appeal/recovery procedures are explicit rather than hidden model- or operator-driven rewrites.

## Governance parameters

Governance parameters become part of the constitutional statement through the committed parameter-set digest. A parameter update is therefore not a privileged key/value write. It is a constitutional or rulebook transition whose payload must identify the exact old state, new state, proposal, tally, authorization, and transition lineage.

Deployments may later define lower-cost delegated parameter policies for non-constitutional tuning, but that delegation itself must be granted by the constitution/rulebook and capability-bound to explicit parameter namespaces and limits.

## Advisory systems

Phi/consciousness measurements, reputation, Symthaea recommendations, ethics/reflection signals, stake, and optimization scores may inform deliberation or request governed review. They are not constitutional authority and cannot bypass this quarantine.
