# REGEN Phase-E Cross-Repository Ownership Contract v1

Status: architecture / dependency-direction freeze

Program: Luminous-Dynamics/mycelix#940

Parent context: REGEN-040..042A

## Purpose

Phase-E resilience work now spans Mycelix and Symthaea. Both repositories may contain types, tests, model adapters, or documentation about the same real-world service/dependency concept, but they must not become competing authorities.

This contract freezes the ownership and information-flow direction before executable resilience work grows.

## Governing theorem

```text
Mycelix authoritative/adopted state
        |
        | exact immutable revision refs
        v
Symthaea deterministic analysis / simulation
        |
        | evidence-bound model receipts
        v
Mycelix evidence / review surfaces
```

not:

```text
Mycelix state <-> mutable Symthaea copy
```

and not:

```text
Symthaea model output -> authoritative state mutation
```

## 1. Mycelix owns authoritative/adopted profile identity

Mycelix owns or references the identity/revision of socially or operationally adopted facts and profiles, including where applicable:

- essential-service definitions;
- served population/scope;
- assessment horizon;
- adopted minimum service requirements;
- rights/custody/permissions;
- ecological obligations;
- water/energy/domain authority refs;
- service dependency declarations;
- qualified substitution declarations;
- campaign preregistration identity;
- evidence snapshot identity;
- institutional policy/profile identity.

Mycelix does not thereby own scientific truth about modeled outcomes.

## 2. Domain systems remain authoritative inside Mycelix

REGEN itself must not duplicate authoritative state already owned by Mycelix Water, Energy, Commons/Property, Supply Chain, Marketplace, Identity, Governance, Praxis, Manufacturing/Craft, or other domain systems.

REGEN stores/reference-binds the exact domain revision needed for a proposition.

```text
REGEN reference
!= copied authoritative truth
```

## 3. Symthaea owns model execution, not adopted state

Symthaea may own deterministic or explicitly stochastic implementations for:

- dependency traversal;
- shock transition algebra;
- service-capacity calculations;
- common-cause analysis;
- reserve/runway calculations;
- substitution search;
- repair-resource contention;
- scenario simulation;
- sensitivity analysis;
- counterfactual comparison;
- bottleneck discovery;
- experiment proposal generation.

Those implementations operate on exact imported snapshots/revisions.

They do not create the authority that makes a service requirement, right, permission, ecological limit, or emergency profile binding.

## 4. Immutable import envelope

Every Symthaea Phase-E execution should receive an immutable input envelope conceptually containing:

```text
RegenAssessmentInput {
    assessment_id,
    service_profile_refs,
    dependency_graph_ref,
    evidence_snapshot_ref,
    authority_profile_refs,
    ecology_profile_refs,
    quality_safety_profile_refs,
    campaign_ref,
    model_input_digest,
}
```

The exact schema may evolve.

The required property is that the execution binds exact revisions rather than performing live mutable lookups during the scientific run.

## 5. No hidden live re-resolution

Once a campaign starts, Symthaea must not silently re-query mutable Mycelix state and incorporate later changes into the same evidence lineage.

If authoritative state changes:

```text
input revision A -> run A
input revision B -> run B
```

not:

```text
run A silently mutates into B
```

## 6. Model receipt boundary

Symthaea returns model evidence, not authoritative state.

Conceptually:

```text
RegenModelReceipt {
    input_envelope_ref,
    model_revision,
    toolchain_or_runtime_ref,
    fixture_or_seed_refs,
    transition_or_analysis_receipts,
    outcome_vector,
    unresolved_items,
    proposition,
    non_claims,
}
```

The receipt may be admitted into Mycelix evidence/review surfaces under the appropriate evidence semantics.

## 7. Model receipt != observation

```text
Symthaea model result
!= environmental observation
!= service-delivery observation
!= executed recovery
!= authoritative policy state
```

A model result must retain its evidence class and lineage when returned to Mycelix.

## 8. Recommendation boundary

Symthaea may emit a recommendation/proposal artifact separately from a scientific result.

```text
model result
-> optional recommendation
-> human/institutional review
```

A recommendation cannot directly mutate:

- adopted service minima;
- ecological obligations;
- rights/custody;
- procurement commitments;
- emergency authority;
- infrastructure controls.

## 9. No duplicate canonical service identity

Symthaea must not create a second canonical service identifier namespace for Mycelix-owned services.

Its local types should carry opaque Mycelix service/profile IDs or exact adapter identifiers.

A Symthaea convenience enum may classify behavior internally, but it cannot replace the upstream identity.

## 10. No duplicate canonical dependency graph

Symthaea may construct an execution graph from a frozen Mycelix dependency-graph revision.

That execution graph is a derived model artifact.

If Symthaea discovers a missing dependency, the correct result is an issue/proposal/evidence artifact such as:

```text
MissingDependencyCandidate
```

not silent insertion into the authoritative graph.

## 11. Proposed dependency change != adopted dependency change

```text
Symthaea discovers candidate edge
!= Mycelix dependency graph revised
```

A new authoritative edge requires the appropriate Mycelix evidence/review/adoption path and a new graph revision.

## 12. Service-floor change firewall

A simulation may show that an adopted service floor is difficult, expensive, or impossible under a fixture.

That does not permit the model to lower the floor in order to produce a PASS.

```text
model infeasibility
!= permission to rewrite requirement
```

A changed requirement creates a new authoritative profile revision.

## 13. Hard-gate ownership

Rights, ecological, safety, quality, and authority gates remain inputs to the model.

Symthaea may evaluate whether a candidate satisfies a supplied gate when the deterministic evaluation contract is explicitly delegated to it, but it cannot invent a PASS when the required authoritative evidence is absent.

```text
Unresolved upstream gate
-> unresolved candidate
```

unless the exact upstream profile specifies another state.

## 14. Exact revision echo

Every meaningful Symthaea receipt should echo the exact upstream revisions actually consumed.

A downstream reviewer must be able to answer:

- which service definition?
- which dependency graph?
- which evidence snapshot?
- which rights/ecology/quality profile?
- which campaign definition?
- which Symthaea model revision?

without reconstructing the answer from logs.

## 15. Cross-repo schema compatibility

Wire/adaptor schemas should be versioned independently from either repository's internal Rust type layout.

Conceptually:

```text
mycelix internal state
-> versioned REGEN assessment schema
-> symthaea internal model state
```

This prevents a refactor in one repository from silently changing scientific semantics in the other.

## 16. Canonical serialization / commitment

Where a cross-repo digest is used as an evidence commitment, the serialized representation and version must be explicit.

A Rust debug string or arbitrary map iteration order is not a canonical scientific commitment.

## 17. Unknown remains unknown across the bridge

Adapters must preserve tri-state/plural epistemic semantics.

```text
Missing != false
Unknown != unavailable
Unresolved != failed
```

The bridge cannot coerce an upstream unknown into a convenient default for the model.

## 18. Units remain explicit

Cross-repo numeric values must bind their quantity/basis semantics.

No adapter may silently convert:

- stock to flow;
- wet mass to dry mass;
- capacity to delivered service;
- power to energy;
- nominal capacity to usable capacity;
- modeled quantity to observed quantity.

## 19. Time semantics remain explicit

The input envelope should preserve observation time, snapshot/revision time, campaign start, horizon, and any currentness state needed by the adopted profile.

A fresh model execution does not make stale input evidence fresh.

## 20. Local simulation cache has no authority

Symthaea may cache imported snapshots for deterministic replay/performance.

A cache entry is never an authoritative current-state source.

Any current-state claim must resolve through the appropriate Mycelix/domain evidence path.

## 21. Campaign ownership

Mycelix owns the preregistered campaign identity and exact fixture/profile references.

Symthaea owns the execution implementation used to apply that frozen campaign.

Therefore:

```text
campaign revision
!= model revision
```

Both are bound in the execution receipt.

## 22. Shock transition ownership

REGEN-042/042A defines campaign/effect semantics at the cross-domain contract level.

Symthaea may implement the reducer/kernel.

An implementation-specific optimization must preserve the normative transition semantics or create a new model revision and requalification requirement.

## 23. Randomness policy

The campaign identifies whether randomness is permitted and which seed/sample set belongs to the run.

Symthaea supplies the deterministic execution of that policy.

It must not introduce hidden RNG when the campaign is deterministic.

## 24. Qualification split

Mycelix and Symthaea qualification answer different propositions.

Mycelix-side qualification may establish that:

- a profile/evidence/campaign artifact has the required structure, lineage, identity, and adoption references.

Symthaea-side qualification may establish that:

- an exact model implementation deterministically/validly executes the declared contract over frozen fixtures.

Neither qualification automatically transfers to the other proposition.

## 25. Joint theorem requires both sides

A bounded end-to-end resilience-analysis theorem requires at least:

```text
qualified authoritative/adopted input contract
+ qualified model implementation
+ exact bridge schema
+ exact frozen input snapshot
+ exact execution receipt
= reviewable end-to-end model result
```

not real-world resilience truth.

## 26. Failure independence

A Symthaea crash, queue failure, or model error must not mutate authoritative Mycelix state.

A Mycelix/domain service outage may prevent fresh authoritative snapshots, but an already-frozen offline scientific fixture remains replayable if its evidence package is available.

## 27. No control-plane dependency

Phase-E analysis must remain outside any hard real-time physical control loop.

```text
Symthaea resilience simulation unavailable
!= essential service actuator must stop
```

Physical continuity remains owned by the relevant local infrastructure/device/control systems.

## 28. Evidence return path

A Symthaea execution receipt can be referenced by Mycelix as model evidence only after the appropriate admission/validation step.

The return path must not relabel `Scenario`/`Derived`/`Inferred` content as raw `Observed` evidence.

## 29. Counterexample return path

A discovered counterexample or missing dependency should be especially easy to return.

Conceptually:

```text
CounterexampleReceipt {
    frozen_input_ref,
    model_ref,
    invariant_ref,
    minimal_witness,
    affected_service_refs,
    affected_dependency_refs,
}
```

This allows Mycelix maintainers/institutions to decide whether a contract/profile revision is warranted without Symthaea editing it directly.

## 30. Cross-repo drift detection

CI should eventually include fixtures that encode the same canonical bridge payload in both repositories and verify:

- schema version compatibility;
- canonical digest identity;
- exact unknown-state preservation;
- unit/time semantics;
- service/dependency ID preservation;
- rejection of unsupported newer schema versions.

## 31. Version skew

A newer Mycelix schema revision presented to an older Symthaea model must not be silently parsed as an older schema if semantics changed.

The model should return an explicit unsupported-schema state until an adapter/model revision is qualified.

## 32. No backward inference of authority

A successful Symthaea simulation cannot be used to infer that missing upstream authority must therefore have existed.

```text
model path feasible
!= path authorized
```

## 33. Existing Symthaea REGEN-040/041 documents

Symthaea's REGEN-040/041 documents are treated as **assessor/model hardening profiles**, not competing authoritative definitions of the Mycelix REGEN-040/041 service and dependency contracts.

Their useful additional invariants—such as verifying fallback availability/capacity and preventing shared-reserve double counting—belong in the model implementation/qualification layer and may later motivate upstream contract revisions through explicit review.

## 34. Naming discipline going forward

Where a REGEN number already names the cross-domain Mycelix contract, Symthaea follow-up work should prefer an implementation/profile suffix rather than creating a second generic contract with the same apparent ownership.

Examples:

```text
REGEN-042        Mycelix campaign contract
REGEN-042A       Mycelix transition-semantics hardening
REGEN-042S       possible Symthaea executable model profile
```

The exact suffix convention may evolve; the no-split-authority rule does not.

## 35. Deliberate non-claims

This architecture contract establishes no real service sufficiency, disaster resilience, emergency authority, rights determination, ecological eligibility, infrastructure safety, procurement approval, policy legitimacy, or physical-action authority.

It establishes only which repository/layer is allowed to assert which class of proposition and how exact immutable revisions cross the boundary.
