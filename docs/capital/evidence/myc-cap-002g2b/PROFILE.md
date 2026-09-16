# MYC-CAP-002G2B — Constitution amendment transition v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Qualify one append-only constitution transition from epoch N to N+1 without allowing current designation, emergency power, refinancing, custody change, or a generic governance vote to become a universal constitution-edit key.

Core separation:

```text
current constitution
!= amendment authority

amendment authorization
!= successor designation

accepted amendment
!= legal validity
```

## Conservative v1 amendment classes

v1 freezes exactly:

```text
POLICY_PARAMETER
REPRESENTATION
CONSTITUTIONAL_STRUCTURE
PROTECTED_INVARIANT
```

Only `POLICY_PARAMETER` is permitted in the first executable profile.

The other classes remain prohibited until separately designed and qualified.

## Amendment-specific authority

A G2A currentness receipt is used only to establish that the **prior constitution is current**.

It does not authorize the amendment.

Amendment authorization is separately checked against the G2B amendment profile:

- exact project and registry;
- exact prior epoch/profile;
- exact successor profile;
- exact amendment class;
- class-specific chamber quorum/approval thresholds;
- independent-enforcer concurrence;
- public notice/evidence reference;
- exact G2A currentness receipt digest.

No emergency field exists in the authorization schema.

## Semantic diff theorem

The verifier computes the changed-field set itself from prior and successor G1 profiles.

Caller-supplied change summaries are not authority.

For G1 fields that are semantically sets, v1 normalizes ordering before comparison:

- chamber membership;
- conflict-code membership;
- required-recusal conflict membership.

Thus harmless reordering does not create a fake constitutional amendment.

The first `POLICY_PARAMETER` profile allows only these exact paths:

```text
/actions/OPERATOR_RENEWAL/required_chambers/PUBLIC/approval_ppm
/actions/OPERATOR_RENEWAL/required_chambers/PUBLIC/quorum_ppm
/actions/OPERATOR_RENEWAL/required_chambers/USERS/approval_ppm
/actions/OPERATOR_RENEWAL/required_chambers/USERS/quorum_ppm
```

Any hidden semantic change outside the allowed set blocks the transition.

## Protected invariants

Both prior and successor profiles must pass the G1 structural validator under the frozen protected context.

Therefore v1 rejects:

- asset-lock removal becoming permissible;
- steward-seat sale becoming permissible;
- weakening protected-action enforcer requirements;
- a `CAPITAL` chamber;
- protected chamber-topology changes.

## Non-resetting epoch theorem

```text
successor_epoch == prior_epoch + 1
```

is mandatory.

Refinancing, operator replacement, custody change, insolvency, migration, or SPV replacement are not epoch-reset events.

The transition binds a previous-transition digest for future append-only lineage composition.

## No self-amendment race

The supplied G2A currentness receipt must bind the prior profile and prior epoch.

The successor constitution cannot establish its own authorization or currentness.

Even `transition_state = ACCEPTED` fixes:

```text
successor_designation_established = false
```

A separate designation event must make the successor current.

## Output

The deterministic `ConstitutionAmendmentReceipt` exposes:

- prior/successor epoch and profile digests;
- amendment class;
- verifier-computed changed paths;
- exact currentness/authorization/transition commitments;
- blockers;
- `transition_state = ACCEPTED | BLOCKED`;
- explicit nonclaims.

Every receipt fixes:

```text
successor_designation_established = false
legal_validity_established = false
democratic_legitimacy_established = false
```

## Deterministic commitments

Amendment profile semantic SHA-256:

`454e3c00a47f1e970ebf9a1bfa688515e70ccca0aeb824dc50352476c1322398`

Prior G1 profile semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

Successor G1 profile semantic SHA-256:

`84cbcaff53366ab611402fc5114e9f4eafa066ef753f8e722ae9caff13b79284`

Prior G2A currentness receipt SHA-256:

`929f5cdd8eef25e56d557a17a4f4a5ab59694579555d60ad75dc90a937973a1e`

Canonical fixture SHA-256:

`6900a6be578b706e7014d5f433352a183bc8b0cb24a5e9e21e44307652b830c6`

Frozen receipt SHA-256:

`d26ee8750ef11b23809321501cdd2dc8e7d491b9e12a5baa5ace9afea85b339b`

Verifier SHA-256:

`b25e7ec56b3ce79fd84cf62e140a43cb7faa5878515e6516f00083e559c247b6`

Regression-suite SHA-256:

`4a32119fd6ab812da030e54c4c267de8f59182138f11f24f8895f27f4c2f85ad`

## Local preflight

The stdlib suite passes **30/30** locally, covering:

- accepted N→N+1 policy amendment;
- epoch skip and profile substitution;
- empty/same-profile successor;
- hidden unauthorized changes;
- asset-lock and steward-seat protection;
- capital-chamber injection;
- protected-enforcer weakening;
- historical/revoked/pending prior currentness;
- currentness project/registry/profile substitution;
- prohibited amendment classes;
- emergency-field injection;
- enforcer substitution/non-concurrence;
- quorum and approval failures;
- authorization project/registry/epoch/successor/currentness substitution;
- malformed previous-transition commitment;
- authority contamination;
- semantic-set reordering;
- deterministic reconstruction and strong nonclaims.

Local PASS is not hosted qualification.

## Nonclaims

Even a hosted PASS would establish only that one supplied constitution transition satisfies one frozen amendment profile over supplied evidence.

It would not establish:

- successor designation/currentness;
- legal or constitutional-law validity;
- democratic legitimacy;
- ballot or real-world identity authenticity;
- social consensus;
- enforceability in a particular jurisdiction;
- moral correctness or wisdom of the amendment.
