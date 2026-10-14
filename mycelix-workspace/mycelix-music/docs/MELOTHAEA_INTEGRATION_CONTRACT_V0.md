# Melothaea Integration Contract v0

**Status:** proposal / contract-first design  
**Owner boundary:** Mycelix Music and Melothaea remain separate products.  
**Implementation status:** this document does not claim a working integration or a qualified frontend build.

## 1. Product boundary

- **Mycelix Music** owns the creator-facing music product: projects, canonical work identity, playback, library, sharing, permissions, and decisions to keep or publish.
- **Melothaea** is an independently developed musical-intelligence product: theory-aware analysis, bounded composition proposals, motif/structure reasoning, and explanations of proposed musical effects.
- **Integration** is an optional adapter in Mycelix Music. Mycelix Music must remain useful when Melothaea is unavailable, disabled, or incompatible. Melothaea must remain independently testable and usable by other clients.
- Do not merge product identities, roadmaps, or release claims. Do not move musical-cognition authority into the Leptos UI.

## 2. First vertical slice

1. The creator explicitly selects a work/rendition in Mycelix Music.
2. The creator selects a bounded intention, including what should be preserved and what may change.
3. Mycelix Music sends a versioned request through an adapter.
4. Melothaea returns a proposal with explicit subject identity, proposed artifact/score identity, changes, formal validation results, and provenance.
5. Mycelix Music displays the proposal as a proposal—not as the current/canonical work—and lets the creator audition, accept, or reject it explicitly.
6. Accepting a proposal creates or selects a distinct rendition; it never silently overwrites the source work.

Do not start with an autonomous rewrite, background upload, or broad frontend redesign.

## 3. Contract requirements

The first transport-neutral contract should define the following fields. Exact Rust types and serialization are implementation work to be chosen after auditing the live crates and supported transport.

### Request

- schema_version
- request_id (unique per invocation)
- work_id and source_rendition_id (stable identities, not transient UI row indexes)
- source_content_digest and digest algorithm where available
- intent (the requested musical effect)
- preserve (explicit invariants/regions that must remain unchanged)
- change_scope (the regions/dimensions Melothaea may propose changing)
- constraints (theory, renderer, format, and resource limits as applicable)
- consent / execution mode, including whether the operation is local-only or permits an explicitly named remote service

### Response

- schema_version and echoed request_id
- exact source work/rendition identity and digest observed by Melothaea
- proposal_id and proposed artifact/score identity or content digest
- machine-readable change summary and affected regions
- formal validation results, with each check independently named and classified
- provenance: engine/build identity, relevant configuration, recipe/input references, and reproducibility limitations
- explicit status: proposed, rejected, unsupported, or failed; never imply that a proposal has been accepted
- structured diagnostics for incompatibility, stale input, invalid output, or unsupported constraints

Do not invent a rendition identity for an artifact that has not been materialized. Do not represent missing validation as success. Do not collapse different validation dimensions into a single “quality” score.

## 4. Authority and safety invariants

1. **Source immutability:** proposals do not mutate the selected source work or rendition.
2. **Identity binding:** a response is rejected if its request ID, source identity, or source digest does not match the in-flight request.
3. **Stale-result rejection:** changing the selected work, cancelling, or superseding a request prevents late responses from becoming active UI state.
4. **Formal constraints first:** Melothaea recommendations cannot bypass canonical theory validation, preservation obligations, or renderer capability checks.
5. **Explicit creator choice:** auditioning is not acceptance; acceptance is not publication; publication is not permission for training or reuse.
6. **Privacy by default:** no source audio, score, or private metadata leaves the creator's chosen execution boundary without explicit disclosure and consent.
7. **Independent availability:** disabled or failed Melothaea integration must not break playback, library access, or other Mycelix Music workflows.
8. **Truthful evidence:** distinguish request construction, proposal returned, formal validation, successful rendering, saved artifact, and browser acceptance as separate states.

## 5. Leptos UX

Add an opt-in entry point such as **Explore with Melothaea** on a selected work. The initial workspace should show:

- selected source identity and digest/status;
- preserve/change controls with explicit defaults;
- request progress, cancellation, and errors;
- proposal summary and per-check validation;
- audition action separate from accept/reject;
- clear return to the source rendition;
- provenance and limitations, without claiming objective artistic quality.

Reuse Mycelix Music's authoritative playback state. Do not add a second independent audio player or let proposal audition overwrite canonical selection. If the existing player cannot represent proposal audition safely, first define the smallest adapter to its current state model rather than duplicating playback state.

## 6. Qualification plan

Before describing this as integrated, qualify these gates against exact source revisions:

1. **Contract tests:** schema/version compatibility, round trips, unknown versions, missing fields, and malformed responses.
2. **Identity tests:** wrong request ID, wrong source ID/digest, duplicate response, and late response after cancellation/supersession all fail closed.
3. **Musical validity:** invalid or unsupported proposals remain visible as rejected/unsupported and cannot be accepted as validated.
4. **Playback tests:** audition uses the shared player, source identity remains unchanged, and return restores the prior source/time/play state where supported.
5. **Persistence tests:** accepting creates a distinct stable rendition and preserves source + recipe/provenance; failed saves do not show success.
6. **Privacy tests:** local-only mode does not transmit source material; remote execution requires explicit opt-in.
7. **Build/browser gates:** Rust tests, formatting, WASM compile, warnings-denied lint/Clippy as applicable, then real-browser acceptance on the exact head.

A green contract test is not proof of musical quality, human preference, cognition correctness, production readiness, or secure payment settlement.

## 7. Explicit non-goals for v0

- merging Mycelix Music and Melothaea into one product;
- autonomous source replacement or publication;
- claims that a structurally different score is better music;
- a new playback engine;
- global learning or model training from creator content;
- royalty/payment changes;
- a physical crate rename or broad workspace migration.

## 8. Repository audit findings (2026-10-09)

The initial source audit found several concrete constraints that should shape implementation:

- The authoritative public product repository is `Luminous-Dynamics/mycelix`; the canonical source inspected for this proposal is `mycelix-workspace/mycelix-music`, including `apps/leptos`. New Mycelix Music product changes should be proposed against that public repository. The standalone public repository `Luminous-Dynamics/Mycelix-Music` is archived and must not be treated as the active source. The private `Luminous-Dynamics/luminous-dynamics` monorepo is not the canonical public destination for new product changes.
- The current Leptos app defines its own PlayerState in apps/leptos/src/app.rs and routes /, /discover, /artist, /dashboard, /upload, /gallery, and /about. There is no Melothaea route in the inspected app.rs, and the current player state is not shown to be the Symthaea Muse UI's shared audition reducer. Therefore, do not assume that the two player models can be joined by simply adding a navigation link.
- The app's Cargo.toml depends on symthaea-muse with default features disabled. The inspected symthaea-muse manifest defines theory as an optional feature and studio as theory plus optional Axum/Tokio/server dependencies. This is not evidence that a direct browser/WASM call path is already available or qualified.
- IMPLEMENTATION_STATUS.md (reviewed 2026-07-16) explicitly records unresolved build and integration gates for the Leptos UI and other release surfaces. That dated status must be rechecked against current exact-head CI before making present-tense release claims.
- The Melothaea program issue describes a bounded cognition-to-theory path; it does not by itself establish a stable external service contract or a Mycelix Music integration API.

Consequently, the safest first implementation is likely a small, versioned adapter around a supported, independently testable Melothaea boundary—not a direct dependency from the browser UI on a server-oriented feature, and not an attempt to share playback state by reaching across product internals. The exact transport (in-process library, local service, or explicit remote service) remains an open decision until feature and target compatibility are verified.

## 9. Next engineering decision

Audit the live Mycelix Music and Melothaea manifests, feature flags, public APIs, and existing score/rendition identity types. Select the narrowest supported call boundary only after verifying whether the current implementation can be consumed as a library, a local service, or another explicit transport. Do not add a path dependency based on assumptions about crate availability or feature compatibility.


## 10. Live Melothaea source and qualification audit (2026-10-09)

The dedicated Melothaea development source currently lives in the public `Luminous-Dynamics/symthaea` repository, tracked by [program issue #3898](https://github.com/Luminous-Dynamics/symthaea/issues/3898). This is a separate product/program boundary; it is not a reason to merge Mycelix Music with Symthaea or to rename this app.

The two earliest relevant implementation proposals are still open draft PRs:

- [MEL-001 / #3896](https://github.com/Luminous-Dynamics/symthaea/pull/3896), head `8a9c7f0db15287d32e8edaf4985dc593b7511f2f`, introduces bounded, advisory cognitive selection among already theory-valid symbolic alternatives. Its stated authority order keeps theory, preservation, and compositional obligations ahead of effect-fit.
- [MEL-002 / #3897](https://github.com/Luminous-Dynamics/symthaea/pull/3897), head `b021930d96b86c0b32c851d0be7b51300207b152`, connects selection to actual symbolic score content and checks content identity. Its stated scope is an integration test; it does not switch the live Studio composition path to cognition-enabled selection.

Those proposals are useful architectural evidence, but neither defines a stable external Mycelix Music API. Neither establishes that a browser-compatible library/service is available, that a proposal can be rendered in this app, or that musical quality improves.

The current `symthaea-muse` manifest on Symthaea `main` still names the package `symthaea-muse`. Its `theory` feature gates symbolic-theory capabilities; its `studio` feature includes `theory` and server-oriented optional dependencies including Axum/Tokio. The inspected `muse_studio` binary is a local web app, but the reviewed source and two draft MEL PRs do not establish a versioned, supported cross-product endpoint for Mycelix Music. Do not turn the existence of a local Studio binary into an assumed HTTP contract or put its server stack into the WASM bundle.

### Current qualification state

For the exact MEL-001 and MEL-002 heads above, the workflow lookup returned associated CI runs `35339582278` and `35339916951` with conclusion `cancelled`; their corresponding PR Governance runs `35339582135` and `35339916926` completed successfully. Governance success is not the Rust/WASM/test qualification. Neither CI run is a PASS.

The open [CI-SCHEDULER-001 issue #4276](https://github.com/Luminous-Dynamics/symthaea/issues/4276) records a wider hosted-runner/Actions admission problem, with its most recent update on 2026-09-27 and queue observations dated 2026-09-19. This audit does not independently establish the current repository-wide queue count, but the exact MEL heads checked here remain **not CI-qualified**. Do not infer source failure from cancellation or convert it to a pass. Do not promote these draft PRs based on their descriptions alone.

### Integration choice

Until a versioned supported boundary and an executable qualification path exist, Mycelix Music should implement only its product-side proposal UX and adapter contract. The adapter may return an explicit `Unavailable` / `Unsupported` outcome and must not fabricate a successful proposal.

A later transport selection must be supported by evidence for all of the following:

1. a versioned request/response interface owned by Melothaea, not inferred from internal Rust types or the Studio UI;
2. an explicit local-only default and disclosure/consent before any remote transfer;
3. exact source-rendition identity and content digest binding;
4. cancellation/supersession semantics so stale responses cannot alter the active proposal;
5. formally validated proposal artifacts with stable content identity and provenance;
6. independent exact-head compile, contract, rendering, persistence, and browser qualification.

The first integration slice should stop at **request → validated proposal → separate audition → explicit accept/reject**. It should not ship until the relevant boundary can execute end-to-end. An unavailable or unqualified Melothaea adapter must leave Mycelix Music's catalog, player, queue, and ordinary creator workflows functional.
