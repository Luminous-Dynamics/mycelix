# Governance Constitutional Trust-Root Migration

Status: **Draft / fail-closed migration**

The legacy mutable `constitution` zome remains readable for compatibility, but it is **not** a valid trust root for new binding governance.

## Authority rule

Binding governance derives constitutional authority only from:

1. the exact `governance_constitution` manifest committed in Holochain DNA properties;
2. the semantic genesis statement derived from that manifest; and
3. for successors, a verified parent -> child constitutional transition whose binding tally, threshold authorization, amendment payload, policy and replay nonce are all bound to the exact transition digest.

A DHT entry, proposal ID, author identity, coordinator call path, model score, Phi score, reputation score, or threshold signature by itself is insufficient constitutional authority.

## Deployment gate

The repository manifest intentionally ships with:

`governance_constitution: null`

A production governance DNA MUST NOT be packed or installed until an operator supplies an exact reviewed `ConstitutionGenesisManifest`. Because DNA properties are integrity modifiers, changing that manifest creates a different DNA/network identity.

Required pre-pack evidence:

- exact charter bytes + digest profile;
- exact parameter manifest + digest profile;
- exact rulebook bytes + digest profile;
- exact amendment-policy bytes + digest profile;
- exact binding-vote profile;
- exact threshold-authority profile;
- explicit network/institution/constitution IDs;
- reviewed effective timestamp;
- reproducible manifest digest.

No private key or privileged writer is part of constitutional genesis.

## Legacy containment

Until migration is complete:

- legacy charter/parameter records MAY be displayed as historical/compatibility state;
- legacy `create_charter`, `set_parameter`, `ratify_amendment`, and weighted tally paths MUST NOT be used as authority inputs by new rights, signing, execution, verifier-policy, or constitutional-transition code;
- clients MUST surface `constitution_authority.get_constitution_authority_status` and treat `legacy_constitution_authoritative: false` as normative;
- missing/invalid DNA constitutional properties disable binding constitutional governance;
- successor constitutional transitions remain disabled until the transition verifier is wired end-to-end.

## Successor transition gate

A future `ConstitutionTransitionRecord` becomes binding only after all of the following are verified against the parent statement:

- exact parent statement digest;
- exact child statement digest;
- sequential version increment;
- unchanged network/institution/constitution identity;
- amendment policy exactly matches the parent statement;
- binding tally uses the exact binding-vote profile authorized by the parent;
- threshold authorization uses the exact threshold-authority profile authorized by the parent;
- proposal/amendment payload is bound to the exact transition;
- transition nonce is fresh and replay-protected;
- the binding tally, threshold authorization and amendment payload are independently verified, not trusted because a receipt-shaped object exists;
- transition finality is established before consumers adopt the child as current.

## Migration order

1. Land DNA-bound constitutional genesis.
2. Land score-independent binding voting and its credential/finality verifier.
3. Land proposal-owned institutional authority and trusted threshold authorization.
4. Add constitutional transition verification using those verified outputs.
5. Require verified constitutional lineage in rights/verifier/signing/execution paths.
6. Migrate legacy charter and parameter UIs to compatibility/history views.
7. Remove or permanently capability-gate legacy mutation endpoints only after all supported deployments have migrated.

## Non-goals

This migration does not make constitutions immutable forever. It makes **the procedure for changing them explicit, exact, reviewable and cryptographically/institutionally bound**. It also does not grant bootstrap operators permanent authority: the DNA commits genesis semantics, while later authority comes from the amendment process committed by the parent constitution.
