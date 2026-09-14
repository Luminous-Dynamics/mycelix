# IG-007A1 — ObservedSourceBound Mycelix voting profile

Status: **ObservedSourceBound / MeasurementOnly**

Tracks: #869, #851, #855, #856  
Parent measurement tranche: draft #853

This tranche freezes a machine-readable profile of the voting semantics observed in the exact production source bound by IG-007A0. It is intentionally **not** the target policy design.

## Authority boundary

The profile authority is:

`ObservedSourceBound`

This means the profile is derived from exact source identities and independently validated as a stable description. It does not imply executable equivalence has been proved against every production path, that the bound Git subject is the currently deployed release, or that the mechanism is fair, safe, legitimate, Sybil-resistant or desirable.

## Exact bound source

Production subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Voting coordinator Git blob:

`969b845e6186cbcad507c742a718060844f82eb2`

Voting integrity Git blob:

`658562c8dfaf6a2f1b97a7bfd5cf0fc8a5ab6e66`

README/documentation is explicitly not an executable authority source for this profile.

## Frozen profile commitment

Canonical profile payload SHA-256:

`cbbbb3553ce465be989b5f096364ea97ccc9b1c5d6aae67d80177df8d8109763`

SHA-256 is used only as a reproducible profile-content fingerprint.

## Current mechanism distinctions preserved

The profile deliberately retains the current split:

- legacy direct vote → `multiplicative-bounded-v1`;
- explicit Phi-weighted vote → `additive-composite-v1`;
- delegated Phi vote → `additive-composite-v1`;
- ZK eligibility proof voting → distinct `zk-eligibility-proof-v1`.

It also preserves the current missing-Phi distinction:

- multiplicative path: unavailable Phi produces neutral consciousness multiplier `1.0`;
- additive path: unavailable Phi is materialized as `Phi=0.0` and the formula is provenance-insensitive.

## Current tier-policy observations

The source-bound tier constants are frozen as:

| Tier | Phi | Quorum | Absolute floor | Approval | Timelock |
|---|---:|---:|---:|---:|---:|
| Basic | .30 | .15 | 3 | .50 | 24h |
| Major | .40 | .25 | 5 | .60 | 72h |
| Constitutional | .60 | .40 | 10 | .67 | 168h |

The profile does **not** invent the missing ProposalType → ProposalTier mapping.

## Known gaps retained as mechanism state

The profile serializes, rather than repairs:

- #851 — split voting-weight authority;
- #855 — caller-supplied tier in important Phi vote/tally paths;
- #856 — ethics escalation uses the effective tier for quorum fraction and approval threshold but the original input tier for the absolute quorum floor.

A future corrected profile must be a successor. Existing Symthaea experiments remain bound to this v1 observation.

## Why this matters for Symthaea

IG-008 can now consume an exact current-observed mechanism profile rather than copying constants or using README intent.

That enables a before/after research lineage:

`ObservedSourceBound_v1 → counterexample → policy change → successor profile → differential experiment`

instead of retroactively pretending the old mechanism had successor semantics.

## Validator

`validate_ig007a1_observed_profile.py` fails closed on source-subject/blob drift, weight-profile drift, caller-tier or partial-escalation gaps being silently rewritten, tier constant drift, missing #851/#855/#856 observations, unknown source-binding keys, aggregate verdict fields such as `safe`, `fair`, `sybil_proof` or `governance_score`, and canonical commitment mismatch.

## Non-claims

This tranche does not authorize a production migration and does not determine which weight/tier policy should replace the observed state.
