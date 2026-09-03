# mycelix-governance-rights

Score-independent civic rights and binding ballots for Mycelix governance.

## Constitutional separation

> Credentials and institutional capabilities establish eligibility. Scores may inform deliberation, but never create, remove, or multiply a core civic right.

The binding path deliberately contains no Φ/consciousness score, reputation score, stake/wealth value, participation score, domain reputation, model score, or generic advisory weight.

## Why this tranche exists

The current governance implementation has accumulated useful experimental mechanisms—Φ-weighted voting, reputation/trust weighting, stake modifiers, quadratic voting, collective-mirror analysis, ethics assessments, and voting-bloc detection.

Several of those are useful **signals**. They become dangerous when they determine whether a person may exercise a core right or how many binding votes that person's opinion is worth.

The replacement architecture separates three concepts:

1. **Eligibility** — explicit institution/jurisdiction/rulebook-bound credential or governance authority.
2. **Binding ballot** — one eligible principal contributes one ballot in the binding tally domain.
3. **Advisory analysis** — optional Phi/reputation/model/collective metrics used for explanation, reflection, anomaly detection, or voluntary decision support, never as the binding ballot weight.

## Civic eligibility

`CivicEligibilityGrant` binds:

- principal;
- institution;
- optional jurisdiction;
- exact rulebook/version/digest;
- explicit rights;
- non-score eligibility source(s);
- validity period;
- host-verified grant proof reference.

Supported source categories are explicit credentials or institutional decisions: membership, personhood, citizenship, role, governance decision, or delegation.

`authorize_right()` accepts only the grant, right requirement, and time. Its API has no place to supply a consciousness/reputation/stake/model score.

## Binding ballot

`BindingBallot` contains:

- ballot ID;
- proposal reference;
- voter principal;
- eligibility-grant reference;
- For / Against / Abstain;
- timestamp;
- proof reference.

There is no weight field.

A ballot can be checked against a `RightPermit`, which binds the voter to the same eligibility grant and requires the `Vote` right to still be active at cast time.

## Binding quorum and approval

`BindingTallyPolicy` uses integer basis points and an absolute voter-count floor.

The tally computes quorum from the **eligible-person population**:

`required = max(ceil(eligible_voters * quorum_bp / 10_000), absolute_quorum_floor)`

It then counts unique eligible principals, rejecting duplicate voters and duplicate ballot IDs.

Approval uses the ratio of raw For vs Against ballots after quorum. Abstentions count toward participation but not the decisive denominator.

This fixes a category error in the legacy path where a fractional quorum threshold could be compared directly with weighted vote totals.

## Advisory systems remain useful

Existing systems can continue to calculate and publish:

- Phi/consciousness coverage;
- K-vector/reputation signals;
- participation history;
- stake/economic context;
- domain expertise;
- quadratic preference intensity;
- polarization / topology / collective-mirror metrics;
- ethics disclosures;
- cartel/anomaly warnings.

Those outputs should become `AdvisoryAssessment` or institutional evidence. They can trigger transparent review processes **only when the rulebook explicitly defines that process**, but should not silently revoke voting, challenge, appeal, notice, evidence-inspection, contest, or exit rights.

## Migration target for the Holochain governance zomes

### Binding voting

Current `cast_vote` should migrate to:

1. author-bound DID / identity check;
2. explicit civic-eligibility proof for the institution and proposal rulebook;
3. one-principal-one-ballot deduplication;
4. `BindingBallot` persistence;
5. raw-count `BindingTally`.

The current consciousness-gate call should not determine whether a validly eligible person may vote.

### Phi-weighted and quadratic voting

Keep these as clearly named **non-binding advisory / simulation** modes. Their tallies must not advance proposal status or become the threshold-signing authority input.

They can answer useful questions such as: "Would expertise-weighted preferences differ from the civic result?" or "Is there suspicious concentration?" without becoming the constitutional result themselves.

### Guardian veto challenge / override

Challenge and override participation should use explicit eligibility/role/capability policy, not a minimum Φ score. A consciousness or risk assessment may be attached as advisory evidence but does not establish the right.

### Appeals and contests

Appeal, challenge, evidence inspection, notice, and contest rights should be modeled explicitly and must not depend on a model score. Rulebooks may define objective scope/standing requirements, but those requirements should be credential/capability/evidence based and auditable.

## Sybil resistance

Removing consciousness as a rights gate does **not** mean removing Sybil defense.

Sybil defense belongs in identity and eligibility:

- author-bound DIDs;
- verified membership/personhood/role credentials;
- revocation checks;
- one-principal-one-ballot constraints;
- privacy-preserving eligibility proofs where appropriate;
- duplicate/credential-reuse detection;
- explicit institutional electorate snapshots or auditable population registries.

A consciousness estimate is not a substitute for personhood or membership proof.

## Non-goals

This crate does not choose who should be a member of any institution, create a universal voting entitlement, or prescribe one political system. It establishes a narrower invariant: **once an institution defines eligibility and rights, binding civic power should derive from those explicit rules rather than opaque or mutable scores.**
