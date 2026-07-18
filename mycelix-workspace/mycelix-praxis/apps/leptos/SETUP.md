# Praxis Leptos App -- Developer Setup

## Prerequisites

- Rust stable (1.94+) with `wasm32-unknown-unknown` target
- Trunk (`cargo install trunk`)
- wasm-bindgen-cli 0.2.114 (`~/.cargo/bin/wasm-bindgen`)
- Holochain tools (provided by `nix develop` from `mycelix-praxis/`)

## Quick Start (Local Mode)

```bash
cd mycelix-praxis/apps/leptos
trunk serve --port 3001
# Open http://localhost:3001
```

The app starts in **Local** mode. Progress is stored in the browser and no
conductor connection is attempted. Use the navbar selector to change modes:

| Mode | Data provenance | Conductor behavior |
| --- | --- | --- |
| Demo | Clearly labeled representative examples | Never connects |
| Local | Learner-owned browser state | Never connects |
| Live | Holochain records | Authenticated connection and authorized signer are required; failures stay visible |

Changing modes reloads the app so providers cannot retain state from the prior
data source.

## Full Stack (Real Conductor)

```bash
# Terminal 1: Build WASM zomes + start conductor
cd mycelix-praxis
nix develop
CARGO_TARGET_DIR=target cargo build --target wasm32-unknown-unknown --release
hc dna pack dna-minimal/ -o dna-minimal/praxis-minimal.dna
hc app pack happ-minimal/ -o happ-minimal/praxis-minimal.happ
hc sandbox generate -a praxis happ-minimal/praxis-minimal.happ --run=8888

# Terminal 2: Serve Leptos app
cd mycelix-praxis/apps/leptos
trunk serve --port 3001
# Open http://localhost:3001
```

Starting a conductor is only the transport half of Live mode. A production
browser session should be launched by a compatible Holochain host that injects
the standard `window.__HC_LAUNCHER_ENV__` app port/token and
`window.__HC_ZOME_CALL_SIGNER__` callback. The shared provider reads the token
as a `Uint8Array`; it never treats a base64 or arbitrary string as token bytes.

For a non-launcher development shell, inject an issued app token explicitly as
`window.__HC_AUTH_TOKEN = new Uint8Array(...)` and provide a host signer that
implements the contract in `mycelix-leptos-client/docs/browser-signing.md`.
Keep signing keys outside the Leptos WASM module.

Select **Live** only after those host capabilities are present. The badge
switches to "Live" only after the authenticated transport and authorized signer
are both ready. Connection, authentication, signing, serialization, decode,
and zome-call errors remain visible in the Live status banner; they do not
silently change the application to Demo mode.

## Architecture

```
Browser:
  [Leptos WASM App]
    |-- AppMode (Demo | Local | Live; persisted explicitly)
    |-- HolochainProvider (Live only: authenticated WebSocket + host signer, no fallback)
    |-- ConsciousnessProvider (simulated Phi/neuromod at 20Hz via coupled oscillators)
    '-- LearningEngineProvider (consciousness -> learning decisions at 1Hz)

Holochain Conductor (port 8888):
  '-- Praxis DNA
       |-- learning         (courses, modules, progress tracking)
       |-- fl               (federated learning, gradient aggregation)
       |-- credential       (W3C VCs, assessments, epistemic classification)
       |-- dao              (governance proposals, quadratic/simple voting)
       |-- srs              (spaced repetition SM-2, review scheduling)
       |-- gamification     (XP, badges, streaks, activity feed)
       |-- adaptive         (BKT, ZPD, VARK, grade adaptation, recommendations)
       |-- pods             (learning communities)
       |-- knowledge        (curriculum graph, standards alignment)
       |-- classroom        (teacher/student/parent roles, class management)
       |-- praxis-bridge    (cross-cluster bridge to unified hApp)
       '-- integration      (cross-zome orchestration)
```

## Pages

| Route | Component | Description |
|-------|-----------|-------------|
| `/` | `HomePage` | Feature cards linking to core workflows |
| `/courses` | `CoursesPage` | Course listing and enrollment |
| `/review` | `ReviewPage` | SM-2 flashcard review with quality ratings (0-5) |
| `/dashboard` | `DashboardPage` | XP/level, streak, due reviews, skills, consciousness, recommendations, activity |
| `/skill-map` | `SkillMapPage` | Tiered curriculum tree with mastery nodes, filter by grade/subject, detail panel |
| `/teacher` | `TeacherDashboardPage` | Class mastery heatmap, at-risk alerts, skill averages, teacher actions |
| `/governance` | `GovernancePage` | Proposal creation, quadratic/simple voting, weighted tallies |
| `/credentials` | `CredentialsPage` | W3C VC cards, epistemic classification (E/N/M), verification |

## Project Layout

```
apps/leptos/
|-- Trunk.toml              # Build config (target: index.html, dist: dist/, port: 3001)
|-- index.html              # HTML shell loaded by Trunk
|-- src/
|   |-- main.rs             # Leptos mount point
|   |-- app.rs              # Router + provider nesting (Holochain > Consciousness > LearningEngine)
|   |-- holochain.rs        # HolochainCtx, ConnectionBadge, mock fallback
|   |-- consciousness.rs    # ConsciousnessProvider, simulated engine, ConsciousnessCard
|   |-- learning_engine.rs  # Cognitive-aware learning decisions (break enforcement, difficulty, VARK)
|   |-- components/         # Shared UI components
|   '-- pages/
|       |-- mod.rs
|       |-- home.rs          # FeatureCard grid
|       |-- courses.rs       # Course listing
|       |-- review.rs        # Flashcard state machine (Loading -> Front -> Back -> Complete)
|       |-- dashboard.rs     # 6 dashboard cards + recommendations + activity feed
|       |-- skill_map.rs     # Tiered tree with FilterBar, SkillNodeCard, NodeDetail
|       |-- teacher.rs       # MasteryHeatmap, AtRiskPanel, ClassStatsCard, SkillBreakdown
|       |-- governance.rs    # ProposalCard, ProposalDetail, VoteBar, CreateProposalForm
|       '-- credentials.rs   # CredentialCard, CredentialDetail, EpistemicBadge, verification
'-- style/                   # CSS stylesheets
```

## Key Design Decisions

**Explicit data provenance**: Demo examples, learner-owned local state, and Live
Holochain records are distinct application modes. The shared Holochain context
records transport connectivity, zome-call signing readiness, and its latest
contract error separately so even remaining legacy page fallbacks cannot
conceal Live failures. New data surfaces should branch on `AppMode` before
fetching and must never substitute demo records after a Live call fails.

**Consciousness integration**: The `ConsciousnessProvider` wraps the app and exposes reactive
signals (`ConsciousnessState`) to all child components. The `LearningEngineProvider` consumes
these signals at 1 Hz to make adaptive decisions (break timing, difficulty targeting, VARK
mode selection). Neuromodulator proxies (cortisol, acetylcholine, oxytocin) are derived from
the primary signals (dopamine, serotonin, norepinephrine).

**Mastery encoding**: Skill mastery is stored as permille (0-1000) throughout the codebase for
integer precision. UI display divides by 10 to show percentages.

**SM-2 quality scale**: The review page uses the standard SM-2 scale (0-5) where 0 = blackout,
3 = hard recall, 5 = perfect recall. Ratings >= 3 count as correct for accuracy calculations.

## Building for Production

```bash
trunk build --release
# Output in dist/ -- serve with any static file server
```

## Wiring the Spore WASM Engine

The consciousness module currently runs a coupled-oscillator simulation. To connect the real
Symthaea Spore kernel, see the detailed instructions in `src/consciousness.rs` (lines 29-51).
The short version:

1. Copy Spore `pkg/` to `static/spore/`
2. Add the Spore init script to `index.html`
3. Replace `SimulatedEngine` with `SporeEngineBridge` in `ConsciousnessProvider`
4. Add `<link data-trunk rel="copy-dir" href="static/spore" />` to `index.html`

The reactive signal plumbing stays identical -- only the data source changes.
