# Mycelix Frontend Truth Invariants v1

Status: **normative frontend contract**

Scope: shared Mycelix frontend primitives, shells, and domain applications that present state derived from Mycelix/Holochain/runtime sources.

The governing rule is:

> **Presentation may compress truth, but it must never strengthen it.**

A UI may hide implementation detail, shorten an explanation, or defer expert evidence behind progressive disclosure. It must not turn uncertainty into certainty, presence into validity, permission into authority, local state into federated state, or a completed prerequisite into a completed effect.

These invariants define semantic boundaries. Visual style, copy, icons, animation, and layout may vary by domain only while preserving them.

## FTI-001 — Unknown is a first-class state

`Unknown` means the application cannot currently establish the relevant fact.

It MUST NOT be rendered or treated as:

- `Empty` — established absence of items;
- `Missing` — established absence of expected evidence;
- `Unavailable` — established inability to retrieve/provide a surface;
- `Degraded` — established impaired operation;
- `Live`, `Fresh`, `Present`, or any other positive state.

Qualification implication: state mappings and fallback branches MUST contain explicit Unknown coverage when the source model permits uncertainty.

## FTI-002 — Empty, Missing, Unavailable, and Unknown are not synonyms

The frontend MUST preserve these meanings:

- **Empty:** the container/surface is available and is established to contain no items.
- **Missing:** an expected record/evidence item is established to be absent.
- **Unavailable:** the source or operation cannot presently be accessed/performed.
- **Unknown:** existence/availability cannot presently be established.

A component MAY use simpler user-facing copy, but its semantic state and accessible name MUST preserve the distinction.

## FTI-003 — Evidence presence is not evidence validity

`Present` means evidence is available to the caller.

`Present` MUST NOT by itself render or announce:

- `Verified`;
- `Valid`;
- `Authentic`;
- `Authorized`;
- `Trusted`;
- `Sufficient`.

A success color, checkmark, tooltip, accessible label, or surrounding copy counts as a presentation claim and is subject to this invariant.

Qualification implication: a generic evidence-availability component MUST use informational/neutral semantics for `Present`, not verification/success semantics.

## FTI-004 — Verification requires a verification result

A UI may render `Verified`, `Valid`, or an equivalent claim only when the state supplied to that component explicitly represents the result of the corresponding authoritative verification procedure.

The frontend MUST NOT infer verification from:

- record presence;
- a DID string;
- a signature field merely existing;
- trust tier;
- successful retrieval;
- successful transport connection;
- a provenance trail having no availability gaps;
- a prior screen having called something verified.

Verification state SHOULD be separately typed from evidence availability, freshness, trust tier, and connection state.

## FTI-005 — Trust tier is not identity or authority proof

A Mycelix trust/consciousness tier is a presentation of the supplied tier state only.

It MUST NOT imply:

- identity verification;
- credential validity;
- capability possession;
- delegation validity;
- authorization for the current action;
- evidence validity.

Unknown or unrecognized tier input MUST remain Unknown; it MUST NOT silently downgrade to Observer or another known tier.

## FTI-006 — Freshness cannot be hidden by detail copy

Freshness state MUST remain semantically available when display detail is substituted.

For example, visible copy such as `14 minutes old` MUST still expose `Stale` when the supplied state is stale.

The frontend MUST preserve at least:

- Fresh;
- Aging;
- Stale;
- Unknown.

When the source model distinguishes them, **no freshness assertion** and **freshness explicitly Unknown** MUST remain distinct.

## FTI-007 — Stale-present evidence remains both present and stale

A stale record MUST NOT be converted into Missing merely because it is old.

Conversely, stale evidence MUST NOT be rendered as current/fresh merely because it is present.

The valid presentation is the conjunction of the supplied states: evidence is **Present** and freshness is **Stale**.

## FTI-008 — Provenance gaps remain visible

An ordered provenance/explanation surface MUST NOT filter Missing, Unavailable, or Unknown evidence steps merely to make the chain appear complete.

A gap is itself consequential information.

A trail in which every step is `Present` establishes only **evidence availability closure**. It does NOT establish:

- chain validity;
- causal sufficiency;
- authorization;
- cryptographic verification;
- semantic correctness.

## FTI-009 — Provenance inspection is not proof language

Generic provenance links SHOULD use neutral labels such as `Inspect` or `View record` unless the supplied state specifically establishes a proof object and its verification status.

Generic UI MUST NOT rename an available record to `proof` merely because it participates in a consequential chain.

## FTI-010 — Pending, authorized, executed, and settled are distinct

Lifecycle states MUST not be visually upgraded.

At minimum:

`Draft != Proposed != Authorized != Pending != Executed != Failed`

Where applicable:

`Queued != Submitted != Federated != Confirmed != Settled`

Examples of forbidden upgrades:

- a queued payment rendered as settled;
- an approved proposal rendered as executed;
- an authorized action rendered as completed;
- a locally accepted record rendered as federated.

## FTI-011 — Local truth is not remote consensus

Offline/local-first UX MAY acknowledge durable local acceptance immediately when the local authority model permits it.

It MUST NOT present that local state as:

- remote replication complete;
- federation complete;
- counterparty receipt confirmed;
- network consensus established.

`Saved locally; will share when connected` is materially different from `Shared`.

## FTI-012 — Connected is not healthy

Transport connectivity, source availability, freshness, and system health are separate signals.

A connected socket/session MUST NOT by itself imply:

- current data;
- healthy dependencies;
- successful replication;
- valid identity;
- valid authority.

Healthy-state copy SHOULD summarize multiple established signals rather than promote connection state into health.

## FTI-013 — Capability actionability follows current authority

An action that requires a capability/authorization MUST cease to be actionable when the frontend's authoritative state says that capability is revoked, expired, absent, or invalid for the action.

Cached UI affordances MUST NOT preserve actionability after authority loss.

The frontend MUST NOT invent authority from prior success, trust tier, identity display, or route access.

## FTI-014 — Passed governance decisions are not execution authority

A vote/proposal result and authority to execute its consequences are distinct.

`Passed` MUST NOT be presented as `Executed` or as sufficient execution authority unless the authoritative backend contract explicitly establishes that transition.

Timelocks, vetoes, required authorizations, execution receipts, and settlement records MUST remain independently representable when they exist in the domain model.

## FTI-015 — AI interpretation cannot become authority

Natural-language or AI-assisted interaction MUST preserve this boundary:

`intent -> interpretation -> typed proposed action -> preview -> user authorization -> capability/authority check -> execution`

An AI interpretation MUST NOT bypass, manufacture, weaken, or substitute for user authorization or capability/authority checks.

A confidence score from an AI model is not an authority signal.

## FTI-016 — Visual encoding is never the sole semantic channel

Color, glow, animation, icon shape, or spatial position MUST NOT be the only carrier of consequential state such as:

- verification;
- freshness;
- connection;
- availability;
- governance lifecycle;
- capability status;
- evidence gaps.

Consequential state MUST be available in text/semantics accessible to assistive technology.

## FTI-017 — Reduced motion does not reduce truth

When `prefers-reduced-motion` is active, removing animation MUST NOT remove the only indication of state.

Animation may reinforce state; it may not define state.

## FTI-018 — Error rendering cannot mutate domain truth

A component, renderer, route, or presentation failure MUST NOT mutate the underlying domain action state merely to recover the UI.

Examples:

- a failed replication indicator cannot erase a locally durable save;
- a renderer panic cannot mark a pending action failed unless the action itself failed;
- retrying presentation cannot duplicate a consequential action without a new authorized execution path.

## FTI-019 — Human-friendly summaries remain inspectable

Mycelix SHOULD present ordinary users with concise claims such as:

- `Working offline; 3 changes will sync later`;
- `Some information may be outdated`;
- `Allowed to submit this proposal until Sept 8`.

When the underlying system exposes evidence/provenance, consequential summaries SHOULD support progressive disclosure into the records that justify the summary.

Progressive disclosure may hide complexity by default; it MUST NOT discard or rewrite contradictory evidence.

## FTI-020 — Domain theming cannot change semantic meaning

Domains MAY override palette, typography, density, motion, and ornamental language.

They MUST preserve shared semantic meanings for trust, freshness, evidence, availability, lifecycle, error, and authority states.

A theme MUST NOT use success semantics for a state that the shared contract defines as neutral/unknown/warning merely because the domain prefers that aesthetic.

## Qualification ladder

A frontend change affecting consequential state SHOULD be qualified through the strongest applicable subset of:

1. Rust formatting and unit tests;
2. wasm/browser compilation;
3. semantic state-transition tests;
4. component rendering tests;
5. keyboard/focus/accessibility tests;
6. responsive/mobile tests;
7. offline/degraded-state tests;
8. visual regression tests;
9. browser end-to-end tests;
10. authority/truth-boundary tests.

The following regressions are release blockers when applicable:

- Unknown rendered as a stronger known state;
- Present rendered as Verified without a verification result;
- stale rendered as fresh/current;
- queued/pending rendered as executed/settled;
- local-only state rendered as remotely confirmed;
- revoked/expired authority remaining actionable;
- provenance gaps being hidden from an explanation chain;
- inaccessible state that is conveyed only by color/icon/animation.

## Review rule

When a new frontend state is introduced, reviewers SHOULD ask:

1. **What fact does this state actually establish?**
2. **What stronger fact might a user reasonably infer from its copy or visuals?**
3. **Can Missing, Unknown, Unavailable, Stale, Pending, or Revoked be represented without falling through to a stronger default?**
4. **What backend result authorizes any `Verified`, `Authorized`, `Executed`, or `Settled` claim?**
5. **Can assistive technology recover the same consequential distinction?**

If the answer to (4) is only “the frontend inferred it,” the claim is too strong.
