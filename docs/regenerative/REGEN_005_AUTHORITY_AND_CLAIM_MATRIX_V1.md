# REGEN-005 — Regenerative Authority and Claim Matrix v1

Status: normative architecture draft; no runtime or physical authority.

Program: #940

Parent constitution: REGEN-000 / #935

## 1. Purpose

REGEN crosses environmental evidence, agriculture, materials, markets, finance, governance, climate accounting, Symthaea modeling, and eventually physical infrastructure. Those systems must not infer authority from each other merely because their data is composable.

This matrix freezes which classes of artifact may establish which propositions and, equally importantly, which stronger propositions they may **not** establish.

The governing rule is:

```text
fact about a subject
!= permission over the subject
```

and:

```text
strong evidence in one claim domain
!= authority in another claim domain
```

## 2. Planes

REGEN uses ten distinct planes.

1. **Subject identity** — what thing/trial/batch/site/profile is being referenced.
2. **Observation** — evidence about physical or reported state.
3. **Lineage** — how a derived/inferred/forecast/scenario product was produced.
4. **Qualification / suitability** — whether evidence satisfies an adopted domain rule.
5. **Model / recommendation** — analytical or counterfactual decision support.
6. **Governance / policy** — legitimately adopted institutional rules and decisions.
7. **Market / contract** — offers, purchases, service agreements, custody, settlement.
8. **Climate / accounting claim** — carbon or other climate-domain proposition.
9. **Execution authority** — permission for one exact consequential action.
10. **Physical effect / outcome** — what actually happened in the physical world.

No plane silently subsumes another.

## 3. Cross-plane invariant

```text
SubjectIdentity
    != Observation
    != Qualification
    != Recommendation
    != GovernanceDecision
    != MarketContract
    != ClimateClaim
    != ExecutionAuthority
    != PhysicalEffect
```

Lineage is orthogonal: it explains production/provenance of an evidence product; it does not upgrade the product into authority or truth.

## 4. Authority matrix

| Layer / artifact | May establish | Must not establish by itself |
| --- | --- | --- |
| REGEN-002 subject ID | stable typed semantic reference | existence, ownership, quality, currentness, safety, authority |
| PEF environmental observation | reported/observed/derived/inferred/forecast/scenario datum with unit/uncertainty/space/time/provenance | agronomic suitability, policy, execution authority |
| PEF evidence lineage | declared computation/provenance DAG | model correctness, scientific validity, authority |
| REGEN quality-profile reference | exact profile revision and scope | issuer authenticity, adoption legitimacy, material conformance |
| REGEN quality-profile adoption | declared institutional applicability of one exact profile revision | current trusted authority, conformance, suitability, execution |
| Laboratory/test result | measured result under declared method/evidence | universal product safety, field efficacy, governance authority |
| Feedstock provenance record | declared/evidenced source/custody/retention/contamination facts | sustainable extraction permission, process suitability unless separately evaluated |
| Biochar/compost batch lineage | input/process/output/custody identity and evidence | agronomic benefit, carbon removal, market permission |
| Agronomic suitability assessment | suitability under exact soil/crop/site/profile evidence | universal recipe, carbon-credit eligibility, execution permission |
| Field-trial result | observations/comparisons for one declared protocol/context | general causal law, universal recommendation, execution authority |
| Symthaea prediction | model-derived future/latent estimate with uncertainty | observation, policy, authority, physical truth |
| Symthaea recommendation | recommendation among already admissible candidates | candidate eligibility, governance legitimacy, execution authority |
| Marketplace listing | offer/availability assertion under marketplace rules | quality/safety truth beyond bound evidence, ownership beyond custody/authority evidence |
| Supply-chain/custody evidence | movement/custody/source events | product quality, agronomic suitability, carbon credit |
| Commons/Property state | stewardship/title/use-right state under its own authority | agronomic truth, climate truth, model correctness |
| Finance state | balances, claims, payment/settlement/financing state | physical outcome, scientific truth, public legitimacy |
| Governance/Praxis decision | adopted rule/decision under qualified governance authority | physical truth, scientific correctness, external legal validity automatically |
| Climate project/MRV evidence | climate-domain project/methodology/evidence state | agronomic safety/benefit, food safety, physical-action permission |
| Carbon credit/claim state | climate/accounting proposition under Climate authority | soil suitability, crop benefit, ownership of underlying land/material |
| Integration runtime prepared action | durable effect intent/state under integration semantics | current authority if not freshly supplied; physical success before reconciliation |
| Physical controller/local interlock | bounded machine action under local safety constraints | governance legitimacy, model truth, broader resource rights |
| Physical outcome observation | evidence of resulting physical state/effect | retrospective authorization of an unauthorized action |

## 5. Subject identity plane

REGEN subject IDs answer only:

> Which semantic subject is this record talking about?

They do not answer:

- does it exist physically?
- who owns it?
- who may change it?
- is it current?
- is it safe?
- does it pass a standard?
- is it suitable for use?

A stable ID may survive many revisions, measurements, owners, operators, and policy contexts.

## 6. Observation plane

Environmental observations preserve their relationship to reality explicitly.

```text
Reported
Observed
Derived
Inferred
Forecast
Scenario
```

A Scenario datum MUST NOT be laundered into operational truth merely because a policy or model consumes it.

An Observed datum is still not universal truth: instrument quality, calibration, representativeness, sampling, custody, spatial/temporal support and uncertainty remain separate evidence questions.

## 7. Lineage plane

Lineage answers:

> How was this non-raw evidence product produced from its roots?

It does not answer:

> Was the model scientifically correct?

or:

> Is the output authorized for action?

A complete reproducibility capsule is useful evidence. It does not become epistemic or execution authority merely because it is complete.

## 8. Quality profile plane

The following remain distinct:

```text
profile exists
!= exact profile bytes known
!= issuer authenticated
!= institution validly adopted profile
!= profile currently applicable
!= material conforms
!= material suitable for this use
```

REGEN-003 intentionally establishes only structural profile reference/adoption semantics.

Conformance requires actual evidence and evaluation against an adopted rule set.

Suitability may require additional context beyond conformance.

## 9. Feedstock plane

A biomass lot may have evidence for:

- source;
- custody;
- contamination;
- moisture/composition;
- prior use;
- ecological retention decision;
- competing-use decision.

None of those facts alone creates extraction permission.

```text
resource occurrence
!= recoverable allocation
!= legal/community permission
!= qualified feedstock
```

## 10. Batch plane

A batch lineage may establish that an output came from declared input/process/custody evidence.

It must preserve the non-equivalences:

```text
batch exists
!= batch passes quality profile
!= batch is suitable for one soil
!= batch improves crop outcome
!= batch stores qualifying carbon
```

A later stronger claim must reference the exact batch identity and independent evidence supporting that stronger proposition.

## 11. Agronomic plane

Agronomic claims are context-bound.

At minimum, a suitability or response proposition may depend on:

- exact batch/material;
- soil/plot;
- crop;
- climate/water context;
- application rate/method;
- time horizon;
- baseline state;
- uncertainty.

Therefore:

```text
beneficial at site A
!= beneficial at site B
```

A successful trial may support a bounded evidence claim. It does not create a universal recipe.

## 12. Symthaea plane

Symthaea may produce:

- predictions;
- scenario projections;
- comparisons;
- Pareto fronts;
- uncertainty analysis;
- candidate experiments;
- expected-information analysis;
- recommendations.

Symthaea MUST NOT create hard eligibility.

The allowed composition is:

```text
hard domain constraints
    -> qualified candidate set
    -> Symthaea comparison / recommendation
```

Not:

```text
Symthaea preference
    -> make otherwise-ineligible candidate eligible
```

Model confidence is evidence about a model/output. It is never an authority token.

## 13. Governance plane

Community/institutional adoption must come from its own legitimate authority chain.

REGEN does not define one universal governance system.

A governance process may adopt:

- quality profiles;
- ecological floors;
- local procurement preferences;
- essential-service obligations;
- trial protocols;
- budgets;
- infrastructure projects;
- authorized operators.

A governance decision still does not rewrite scientific evidence. Policy may decide what to do under uncertainty; it may not relabel uncertainty as certainty.

## 14. Market plane

A marketplace listing is not a quality oracle.

Listings for consequential regenerative materials SHOULD bind the exact evidence/qualification state required by marketplace policy.

A listing MUST NOT strengthen the claim simply because a seller typed a stronger label.

```text
seller description
!= verified quality
```

Likewise payment does not validate the physical or scientific proposition.

## 15. Finance plane

Financing and payment authority are separate from technical and ecological truth.

Capital may fund a facility. It does not thereby gain:

- protocol authority;
- scientific authority;
- agronomic authority;
- community constitutional authority;
- permanent commons ownership by default.

Shared long-lived regenerative assets may compose Capital-to-Commons, but REGEN does not duplicate that financing state machine.

## 16. Climate / carbon plane

This firewall is mandatory:

```text
AgronomicEvidence
    ||
    || independent authority/evidence paths
    \/
ClimateCarbonEvidence
```

Forbidden shortcuts include:

```text
crop yield improved -> carbon credit
biochar batch exists -> carbon credit
soil carbon measured once -> permanence
carbon certified -> agronomically safe
carbon credited -> ecologically beneficial
```

Climate owns climate-project/credit authority under its own qualified methodology and evidence rules.

## 17. Execution plane

No earlier plane creates execution authority implicitly.

A future consequential action should require an explicit chain comparable to:

```text
legitimate principal / institution
+ exact subject
+ exact action
+ current qualified authority
+ fresh relevant evidence
+ deterministic hard-constraint checks
+ bounded action lifetime
+ local physical safety/interlocks
-> eligible prepared action
```

External execution must preserve commit uncertainty:

```text
Committed
DefinitelyNotCommitted
IndeterminateCommit
```

A returned error is not automatically `DefinitelyNotCommitted`.

## 18. Physical effect plane

Action intent and physical outcome are different.

```text
authorized command sent
!= command received
!= actuator moved
!= intended physical state achieved
```

Where outcomes matter, post-action evidence/reconciliation must be explicit.

Observing a desired result later does not retroactively authorize an action that lacked authority at execution time.

## 19. Human standing

No regenerative analysis may turn dependency on human skill into authority over a person.

```text
required skill
!= assigned person
!= compelled labor
```

Governance, labor rights, consent, and human standing remain separate.

## 20. Minimum adversarial corpus

Future executable stacks SHOULD include attacks where:

- a valid ID is treated as ownership proof;
- an observation is treated as policy;
- a complete lineage is treated as scientific truth;
- an adopted profile is treated as conformance;
- conformance is treated as suitability;
- suitability is treated as carbon eligibility;
- a Symthaea recommendation widens hard eligibility;
- a marketplace label strengthens evidence;
- investment creates governance rights not present in the financing constitution;
- carbon status is treated as agronomic safety;
- an old authorization is treated as current;
- an external timeout is treated as definitely not committed;
- physical success is used to excuse missing authority.

## 21. Relationship to immediate PRs

- REGEN-002 defines subject identity only.
- REGEN-003 defines quality-profile reference/adoption structure only.
- REGEN-004 freezes cross-repository subject-ID framing.
- REGEN-006 will define exact convergence with PEF evidence/lineage.
- REGEN-010+ may add domain evidence profiles only after these boundaries are stable.

## 22. Deliberate non-claims

REGEN-005 is a normative matrix. It does not prove any current Mycelix/Symthaea implementation enforces every row. Each consequential bridge must earn its own executable evidence.
