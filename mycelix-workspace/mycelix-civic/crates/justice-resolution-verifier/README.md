# Justice Resolution Verifier v0.1

`justice-resolution-verifier` is the pure qualification kernel between authoritative Civic Justice runtime state and the final semantic outcomes defined by `justice-resolution-types`.

It does **not** fetch Holochain records and does not establish that caller-supplied snapshots are authentic. A future Justice-owned runtime adapter must fetch exact immutable records/policy and construct the transport-neutral input cut.

## Trust boundary

```text
exact Case / Arbitration / Decision / Remedy / Appeal / policy records
        ↓ Justice-owned runtime adapter
exact transport-neutral snapshots
        ↓ justice-resolution-verifier
VerifiedJusticeMonetaryRemedyV1
        ├── FinalMonetaryRemedyV1
        └── JusticeVerificationReceiptV1
```

The adapter owns record authenticity and exact-current-state retrieval. The verifier owns deterministic semantic qualification. Neither Business nor Finance participates in either step.

## Deliberately narrow v0.1 rule

The first verifier implements only one frozen decision-rule profile:

`justice.two-party-prevailing-party-full-award@1`

It supports exactly:

- a two-party case;
- a full decision for either complainant or respondent;
- a panel whose active members are `accepted && !recused`;
- explicit integer quorum and support thresholds supplied by an exact versioned policy record;
- Justice monetary remedy kinds `Compensation` or `Restitution`;
- a remedy whose responsible party is the losing party;
- beneficiary derived as the prevailing party;
- exact case subject propagated into the remedy;
- integer amount plus exact semantic unit;
- finality from explicit appeal evidence/history.

It intentionally does **not** support split awards, third-party beneficiaries, joint/several liability, multi-party cases, percentage awards, multiple remedy allocation, or arbitrary policy DSLs. Those require separately versioned profiles.

## Runtime `finalized` is not authority

`DecisionSnapshotV1::declared_finalized` is retained only so the corpus can prove it is ignored.

A `true` flag cannot make an appeal-window decision final early. A `false` flag cannot defeat independently qualified finality evidence.

Finality is derived from:

- explicit qualification time plus exact no-live-appeal evidence after the appeal deadline; or
- an exact appeal resolution affirming the decision.

An active appeal denies current finality. A changed/reversed/remanded appeal resolution denies the original remedy as the current executable result.

## Panel and vote qualification

The verifier rejects:

- duplicate panel identities;
- votes from non-panel actors;
- votes from unaccepted or recused panel members;
- duplicate votes;
- insufficient quorum;
- insufficient support for the declared decision outcome.

The quorum/support numbers are not hidden constants. They come from `FullAwardPolicyV1`, whose exact record reference and semantic profile/version must be supplied. v0.1 accepts only `justice.two-party-prevailing-party-full-award@1`.

A future runtime adapter must still prove that the policy record is authoritative/current/applicable.

## Positive receipt

Success returns a non-forgeable-within-this-crate `VerifiedJusticeMonetaryRemedyV1` containing:

- the final semantic `FinalMonetaryRemedyV1`;
- `JusticeVerificationReceiptV1`.

The receipt preserves:

- exact case/arbitration/decision refs;
- exact remedy index;
- policy ref + semantic profile/version;
- quorum/support thresholds;
- exact eligible panel set;
- exact voter set and support count;
- explicit qualification time;
- exact finality basis/evidence.

This avoids reducing a consequential qualification to an opaque boolean and lets downstream evidence retain the reason a result qualified without embedding raw runtime records.

## Determinism and authority

The verifier reads no system clock, randomness, network, filesystem, process, environment, database, Holochain host function, Business type, or Finance type.

Same explicit input cut produces the same output/denial.

The verifier does **not** prove:

- snapshot authenticity;
- DHT record/action identity;
- policy authority/currentness;
- negative-query completeness for no-live-appeal evidence;
- cross-zome/DNA history completeness.

Those remain the next runtime-adapter boundary before any Justice→Finance/Business bridge should be considered production-complete.
