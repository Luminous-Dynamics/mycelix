# Content Fabric External Planner Protocol v1

Status: **CF-06C contract**

## Purpose

CF-06C creates a narrow interoperability boundary between Mycelix placement policy and external recommendation engines such as Symthaea.

The protocol exists so an external planner does **not** need Holochain, Iroh, CAS, marketplace, lease, payment, or executor access.

```text
CF-06A hard-qualified pool
        +
CF-06B deterministic baseline
        |
        v
ExternalPlannerRequestV1
        |
        | JSON / IPC / local process / other transport
        v
external recommendation engine
        |
        v
ExternalPlannerRecommendationV1
        |
        v
request/input replay checks
        |
        v
CF-06A validate_selection()
        |
        v
AcceptedExternalPlannerRecommendationV1
        |
        v
recommendation only
```

## Identity versus transport encoding

JSON is a convenience interoperability encoding only.

**JSON bytes are not canonical identity bytes.**

`ExternalPlannerRequestIdV1` and `ExternalPlannerAcceptanceIdV1` are instead derived from explicit, domain-separated, length-framed fields. Different JSON serializers may therefore interoperate without changing the semantic audit identity.

## Request construction

Mycelix builds an external request from the same inputs used by CF-06B:

- validated `StorageIntentV1`;
- `PolicyQualifiedPoolV1`;
- `PlannerNormalizationProfileV1`;
- soft recommendation evidence.

The deterministic baseline is generated first. The request then carries:

- `PlannerInputIdV1`;
- storage-intent ID;
- exact placement target;
- explicit evaluation time;
- normalization profile and profile ID;
- committed preference weights;
- minimum replica count;
- hard failure-domain diversity requirements;
- deterministic baseline proposal ID;
- baseline selected subset;
- one candidate entry for every qualified candidate.

Each candidate exposes only planning-facing metadata:

- availability action;
- advertisement action;
- provider identity;
- deterministic baseline rank;
- total and component normalized penalties;
- soft-evidence usability state;
- missing weighted dimensions;
- accepted hard-policy failure-domain values needed to reason about diversity.

This is sufficient to support alternative ranking strategies without exporting execution authority.

## Request commitment

`ExternalPlannerRequestIdV1` commits all decision-relevant request fields, including:

- planner input ID;
- storage intent and target;
- time;
- normalization profile;
- preferences;
- replica/diversity constraints;
- deterministic baseline identity/subset;
- complete ordered candidate feature surface.

Mutation of a request after construction therefore changes the recomputed request ID.

The request ID is an **audit commitment, not a signature**. It does not prove who produced the request or that CF-06A legitimately ran.

## Recommendation contract

`ExternalPlannerRecommendationV1` must contain:

- schema version;
- exact request ID;
- exact planner-input ID;
- lowercase engine ID token;
- engine version/revision token;
- complete best-to-worst candidate ranking;
- selected subset.

A recommendation for an old request cannot be replayed against a newer request because both request ID and planner-input ID must match.

## Complete ranking rule

The ranking must be a complete, duplicate-free permutation of the exact request candidate universe.

The external engine therefore cannot:

- hide qualified candidates from its audit output;
- invent candidates;
- duplicate a candidate to bias interpretation;
- return only the providers it selected.

This makes comparison with the deterministic baseline straightforward.

## Selection rule

Selected availability actions must:

- be unique;
- exist in the ranking/request;
- preserve their relative order from the external ranking; and
- pass `PolicyQualifiedPoolV1::validate_selection()`.

The external engine may rank a same-site candidate highly, but it cannot have Mycelix accept a subset that violates the hard site-diversity requirement.

## Accepted recommendation

After all checks pass, Mycelix creates `AcceptedExternalPlannerRecommendationV1`.

It remains explicitly:

```text
PlannerAuthorityV1::RecommendationOnly
```

Its stable acceptance ID commits:

- request and planner-input IDs;
- storage intent and exact target;
- evaluation time;
- external engine ID/version;
- authority marker;
- complete external ranking;
- selected subset.

Mutation after acceptance therefore invalidates the audit ID.

## Unknown evidence isolation

Soft telemetry referring to an action outside the qualified candidate universe is ignored by CF-06B and does not alter the external request identity.

Unrelated telemetry cannot perturb a target placement decision.

## Symthaea boundary

Symthaea should consume this protocol as a narrow local/IPC contract rather than importing Mycelix infrastructure internals.

A Symthaea implementation may use HDC, predictive models, historical outcome learning, or other reasoning to improve the ranking. It receives no additional authority from doing so.

The expected rule is:

```text
Symthaea may recommend.
Mycelix decides whether the recommendation satisfies hard policy.
An executor separately decides whether authorized execution may occur.
```

## Non-authority

CF-06C does not:

- sign or create leases;
- grant Iroh reads;
- move or delete bytes;
- write Holochain entries;
- establish policy attestations;
- pay providers;
- settle receipts;
- prove content integrity;
- authorize execution.
