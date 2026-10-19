# Hearth UX Contract v1

**Status:** implementation contract; not a claim of user-test or accessibility conformance.
**Product surface:** `mycelix-hearth/apps/leptos` (Rust + Leptos 0.8).
**Design intent:** calm, resident-led coordination—not an operations console.

## Product paradigm

### 1. Intent before inventory

Lead with human intent and household tasks ("what needs attention today?"), not device IDs, zome names, internal scores, or a wall of controls. Keep precise domain controls one level deeper for residents who need them.

### 2. Truth before reassurance

Connection state, source availability, freshness, identity, and authority are different facts. A green connection dot must never imply that all content is live, current, or actionable. Prefer explicit `Unknown`, `Preview`, `Stale`, `Partial`, and `Established` states over false certainty.

This first UX slice adds resident-facing wording for connection states and the connected-but-partially-loaded case. The current page loader still seeds sample records and only refreshes a subset of domains. This notice is a warning, **not** proof of source provenance, not an authorization gate, and not a claim that a mutation is safe.

### 3. Calm by default, progressive disclosure

The home surface should answer:
- What needs my attention?
- What changed since I last looked?
- What can I do next, and what will happen if I do it?

Keep ordinary status calm and sparse; use strong interruption only for genuine hazards, uncertain writes, or safety-critical states. Offer details, provenance, and advanced controls on demand.

### 4. Resident agency, including shared households

Never silently choose between multiple active households by list order. Remembered choices are a convenience only, and are valid only after rebinding to a freshly established active catalog. Explain who will be affected by consequential actions; preview changes before committing; distinguish "suggested", "approved", "sent", and "confirmed". Preserve a way to say "not now", undo where semantics permit, and opt out of recommendations.

### 5. Accessibility is a product constraint

- Keyboard access to every destination and action; visible focus that is not hidden by sticky UI.
- Primary mobile actions target 48 CSS px where layout permits; 44 CSS px is the minimum project design baseline for navigation controls.
- Never encode status by color alone; announce status changes politely, not continuously.
- Respect `prefers-reduced-motion`, zoom/text enlargement, narrow screens, screen readers, and touch/pointer/keyboard input.
- Use plain language for resident-facing errors; keep stack traces and diagnostic detail out of default household copy.

### 6. Local-first and privacy-preserving

Show when an action is local preview, local-only, queued, or synchronized. Do not turn household presence, health, care obligations, or personal routines into engagement scores. Minimize default disclosure, make audience/retention visible near sensitive content, and avoid sending raw audio/video or detailed presence history off-device by default.

## Status vocabulary

| State | Resident-facing meaning | UX rule |
|---|---|---|
| Preview / Mock | Sample content and local demo interactions | Never call it the resident's live household |
| Connecting | Transport is not established yet | Explain that visible content is not yet verified |
| Connected, unverified | Transport exists; the whole domain snapshot is not source-qualified | Explicitly warn that some content may still be sample data |
| Reconnecting / disconnected | Currentness cannot be assumed | Say data may be stale or sample-derived |
| Established | Exact active-household binding and per-domain provenance have been qualified | Only this future state may support a complete/current snapshot claim; it still does not replace backend authorization |

Transport connectivity is not active membership, signing readiness, complete DHT knowledge, freshness, or mutation authority.

## Benchmarks

These are **targets to measure**, not achieved results:

### Trust and correctness
- 100% of tested connection states receive clear resident-facing wording.
- Zero sessions say "fully live" while any displayed domain remains seeded or unverified.
- 100% of household-scoped actions eventually bind to a fresh, exact active-household identity before promotion out of preview/read-only mode.
- Zero accepted mutations can be attributed to a preview/demo identity or stale selected-household generation.
- Every domain panel eventually exposes source status and last-established/freshness context; unknown timestamps remain unknown.

### Task usability
- At least 90% first-try completion across five core tasks: find today's care items, find a pending decision, express gratitude, reach household settings/privacy, and understand whether the view is live.
- 5/5 tasks completable keyboard-only, with visible focus and no keyboard trap.
- At least 4/5 first-time participants correctly distinguish preview from connected-but-partial state after one short exposure.
- Record median and p90 time-to-destination, misclick/backtrack rate, abandonment, and status-comprehension answers. A small test is directional evidence, not statistical proof.

### Responsive and accessible interaction
- Navigation targets at least 44 CSS px high; aim for 48 px on touch layouts.
- No horizontal page overflow at 320, 360, 390, 768, and 1280 CSS px except intentionally scrollable content.
- Visible focus indicator persists on every keyboard-reachable interactive control; test at 200% zoom and with reduced motion enabled.
- Target WCAG 2.2 AA checks for keyboard, focus, target size, contrast, labels, status messages, and error prevention; do not claim conformance without an audit.

### Responsiveness and resilience
- p95 immediate local input feedback <=100 ms on the reference low-end mobile profile; show pending state within 100 ms for remote actions.
- On a local network, target p95 for resident-visible read state <=500 ms after a response is available; report network time separately.
- Preserve meaningful navigation and explicit empty/unavailable states for 72 hours without WAN when the local data plane is available.
- No success toast before the authoritative result is known. Timeout/retry UI must distinguish "failed" from "outcome unknown".

## Qualification protocol

1. Capture a fixed task script and screen-reader/keyboard observations before changing layouts.
2. Use the same viewport/device profile, seeded dataset, locale, and task order for baseline and candidate.
3. Validate labels against the actual transport/source state, not screenshots alone.
4. Test empty, unavailable, partial, stale, conflicting, reconnecting, and multi-household states.
5. Have an independent reviewer check the actual active-household and mutation-binding contracts; UI copy cannot prove backend safety.
6. Retain dated captures, commit SHA, exact test command/output, scenario counts, and failures. Queued/unexecuted is not pass.

## Scope of this tranche

- Add a visible Leptos data-trust notice for connection and partial-source states.
- Add a keyboard-native secondary navigation for existing routes missing from the primary navigation.
- Improve navigation touch-target size.
- Do not change Holochain source semantics, household selection, signing, mutation dispatch, or physical-device authority.
