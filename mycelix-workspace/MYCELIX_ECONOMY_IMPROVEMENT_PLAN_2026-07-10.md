# Mycelix Economy — Improvement & Redesign Plan (2026-07-10)

**Scope**: the `mycelix-finance` cluster (13 zome dirs: 8 wired, 4 scaffold, 1 shared) plus the
economic design corpus (Constitution §VIII.5, Economic/Commons/Metabolism charters, the
tend-economics paper, `simulations/`). Author: review session 2026-07-10, grounded in a
3-agent code pass (file:line evidence) + 2-agent doc pass + the existing ABM simulation suite.

**One-line verdict**: The three-currency *vision* is genuinely distinctive and worth keeping;
the **TEND** implementation is a model of how to do conserved value on a DHT. But right now the
economy's anti-gaming guarantees are **paper** — reputation is self-declared, SAP can be minted
by anyone, and the one anti-accumulation lever the docs lead with (demurrage rate) is
*empirically inert* in our own simulation. This is a **harden + targeted-redesign** job, not a
rewrite.

> **Coordination**: Phase 0 overlaps the P0 author-binding program
> (`MASTER_ROADMAP.md` P0-#1, `MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md` — finance = 25
> Class-A). **Do not duplicate that worklist.** This plan calls out the economic bugs that go
> *beyond* author-binding (severed recognition→score, unlocked stakes, DKG replay, inert
> demurrage, doc incoherence) and sequences the design work that the security pass doesn't cover.

---

## What's actually good (preserve — don't touch in a redesign)

- **TEND conservation stack** is the reference implementation: DHT link-read +
  `follow_update_chain` (not `query()`-local), create-then-verify race guard, optimistic-lock
  read-back, 2-phase `PendingBalanceAdjustment` for crash recovery, symmetric `hours.round()`
  zero-sum, bilateral settlement with treasury callback (`tend/coordinator` ~2967-3010). **Every
  other ledger should be rebuilt to this standard.**
- **Fail-closed tier gates** (`verify_citizen/participant/steward_tier`, `shared:786-841`) — these
  are enforced via `?`, i.e. NOT the decorative `gate_civic` bug (P0-#5). Correct pattern.
- **Integrity author-binding in 2 of 8 zomes** (`staking:448`, `recognition:259`) — the correct
  template; the team already knows the fix, it just wasn't generalized.
- **Treasury/commons balances** use `u64` + `checked_add/sub`; the 25% `inalienable_reserve` is a
  real integer-ratio guard (`treasury/integrity:488`); `currency-mint` has genuine crash-recovery
  and a `retire_currency` zero-sum assertion.
- **price-oracle aggregation** is real: 1-week window, top/bottom-10% trim, accuracy-EMA-weighted
  median, min-2-unique-reporter gate (`price-oracle/coordinator:494-706`).
- **Real ABM simulation** (`simulations/`, 10 seeds, 623 sweeps, Ostrom-Huerta-calibrated) — a
  rare and valuable asset. Extend it (Phase 5), don't restart it.

---

## The four problem classes

### Class A — Economic-security holes (the anti-gaming guarantees are currently paper)

Ranked by blast radius. Most are author-binding (tracked in the P0 triage); the **starred** ones
are *design/architecture* bugs that binding alone won't fix.

1. `bridge::deposit_collateral` — **worst mechanism design in the cluster.** No asset is ever
   locked/escrowed; a self-reported "ETH/USDC" amount mints SAP at **100% LTV, no overcollateralization**;
   SAP is credited immediately while status is still `Pending`; `confirm_deposit` is confirmed *by the
   depositor themself*; the oracle-rate check **fails open** (accepts the claimed rate on oracle failure);
   `update_collateral_health` computes a "Liquidation" label but seizes/burns nothing; vault rate-limit
   uses `query()` (local chain) and is bypassed when `vault_total==0`
   (`bridge:415-541, 1367-1382, 1587, 2303-2358`).
2. `currency-mint::mint_genesis_sap` — unauthenticated, unbounded SAP mint. The STARK "verification"
   only checks that `proof_bytes` is **non-empty** (comment admits it should call `mycelix-zkp-core`);
   previously **no sensor_id+timestamp dedup → replayable unlimited mint** per claimed kWh.
   **Replay FIXED (2026-07-10):** each `(sensor_id, timestamp)` claim now writes a dedup marker
   (`genesis-mint:{sensor}:{ts}` anchor + `AnchorLinks`) *before* crediting, and a repeat claim is
   rejected — one proof mints once. (The stubbed STARK check itself still needs the real
   `mycelix-zkp-core` verification — separate crypto-integration follow-up.)
3. `payments::credit_sap` / `debit_sap` — public unguarded externs; mint or drain any DID.
   (This is the P0 "unauthenticated SAP self-mint".) **`debit_sap` FIXED (2026-07-10):**
   `verify_caller_is_did(member_did)` added — only the balance owner may debit, closing
   "drain any DID"; verified safe across all three callers (`send_payment`, bridge
   `process_payment`, `redeem_collateral`), which each already check caller == member and run
   in that member's agent context. **`credit_sap` still open** — can't take a caller==member
   guard (legit credits target *other* members in a transfer AND the caller's own balance in a
   bridge deposit/pool withdrawal); needs the transfer refactor (see Phase 1).
4. ⭐ **MYCEL reputation is self-declared** — `update_mycel_score` (`recognition:347-394`)
   recomputes the composite from **caller-supplied** participation/recognition/active_months. No
   function folds received `RecognitionEvent`s into the score, so `MIN_MYCEL_TO_GIVE`,
   `MAX_RECOGNITIONS_PER_CYCLE`, recognizer-weighting, and the `recognition:259` author-binding all
   guard a **write-only telemetry entry that no longer influences the score it protects.** A member
   self-assigns `mycel_score = 1.0` → inflates staking weight (≤2.0), TEND mediator eligibility
   (`>0.5`), recognition-giving rights. **This is the single most important non-obvious finding.**
5. Wide-open `RegisterUpdate`/`RegisterDelete` + missing integrity author-binding across 6 of 8
   wired zomes (P0 triage).
6. ⭐ **Treasury DKG allocation replay** — `execute_dkg_allocation` (`treasury:1379-1454`) verifies a
   signature over `"treasury_allocation:{id}:{amount}:{recipient}"` with **no nonce / no consumed-sig
   check**; the recipient replays one valid committee signature in a loop to drain the treasury.
7. ⭐ **`request_allocation`** (`treasury`) — had no caller auth, no destination entry; any agent
   looped it to burn any pool to the 25% reserve floor. **FIXED (2026-07-10):** now
   `verify_caller_is_did(requester_did)` + `verify_governance_or_bootstrap()` (added the
   `GovernanceAgents` link type + `register_governance_agent` extern to treasury, matching
   recognition/staking/tend). Commons-pool allocations are governance-gated (bootstrap-open until the
   first governance agent registers — same posture as the rest of the cluster; global bootstrap-open
   is separate item #14). Note the *no-recipient* functional gap (allocation reduces the pool but
   credits no one) remains — a separate enhancement.
8. ⭐ **Staked SAP is never locked** — `create_stake` made no payments call; collateral was a notional
   number. **FIXED (2026-07-10):** `create_stake` now debits the staker via `payments::debit_sap`
   (lock, fails if underfunded so a stake can't exist unbacked); `withdraw_stake` credits the full
   amount back (unlock); `slash_stake` returns the un-slashed remainder and burns the slashed portion,
   so nothing is stuck. Stakes are now backed by real SAP. (Routing slashed SAP to a commons fund vs.
   burning is a future policy choice.)
9. ⭐ **Forgeable multisig escrow** — `add_escrow_signature` (`staking:663-724`) checks only
   `signer_did ∈ signers`, never binds to author or verifies signature bytes → one caller
   impersonates all N signers.
10. ⭐ **Credit score fully forgeable** — self-reported `collateral_ratio`, unlinked
    `appraised_value` saturating to 1.0, fabricated `OnTime` payment records; `update_matl_score` has
    no caller check (`credit_scoring/coordinator:53-476`). *(lending + credit_scoring are
    `#[deprecated]` for v6.0 removal but still compiled/callable — decide: delete now or fix.)*
11. ⭐ **lending state machine is `query()`-local-only + self-mutable** — `repay_loan`
    (`lending:239-267`) flips `status=Repaid` with no amount check, no lender consent, no transfer;
    self-issue loan → self-mark repaid → perfect payment history.
12. ⭐ **Mint/vault caps use `query()`** (local source-chain only) — globally unenforceable
    (`payments` annual cap ~L779, `bridge` vault L2315).
13. ⭐ **price-oracle single-reporter capture** — reports entered un-deduped; one agent flooding the
    1-week window owned the weighted median while counting as one unique reporter.
    **Flood FIXED (2026-07-10):** `get_consensus_price` now keeps only the *latest report per
    reporter* before trimming/median, so one reporter contributes exactly one price (and
    `reporter_count` now reflects distinct reporters). **Still open (follow-up):** `MIN_REPORTERS=2`
    colluder capture, no reporter stake/slashing, newcomer accuracy 1.0 — harden with a higher
    minimum + reporter staking. This feeds mint-rate and TEND limits.
14. ⭐ **Governance-auth bootstrap-open** — until the first governance agent registers, any agent can
    slash stakes, dissolve MYCEL, forgive balances, drive the oracle (`shared:603-620`, `tend:35-64`,
    `bridge:962-985`).

### Class B — Architectural: the SAP ledger can't enforce conservation on a DHT

Root cause behind half of Class A: **all economic safety lives in bypassable coordinator code; the
integrity layer (the only DHT-binding validation) checks format/finiteness/state-transitions but
almost never authorization or conservation.** The SAP balance model is a mutable shared entry +
optimistic lock + lowest-ActionHash fork-discard → double-spend / lost-credit under concurrency
(`RACE_CONDITION_AUDIT.md`'s "FIXED" is overstated). Treasury's ledger is also disjoint from the SAP
supply (`treasury:88-122`) — pools track their own numbers with no conservation link to payments.
**TEND is the exception that proves it's solvable** — it already does conserved value correctly.

### Class C — Design coherence & the inert lever

- **The headline anti-accumulation lever is both empirically inert AND code-evadable.** Our own ABM
  (`simulations/SIMULATION_REPORT.md`) found: within the constitutional 1–5% band the **demurrage
  rate has no effect on SAP velocity** (the unchartered 1,000-SAP exempt floor is the binding
  constraint); **jubilee frequency has zero effect on MYCEL Gini**; **SAP Gini persists at ≈0.70
  despite demurrage + decay**; and consciousness-gating is **not superior** to flat/random exclusion
  below ~40% defectors. Worse, in code demurrage is **trivially evaded**: any exchange resets
  `last_activity` (`currency-mint/helpers.rs:214`) and sub-1 deductions round to 0, so a periodic
  micro-exchange zeroes decay forever. The economy's marketing rests on a lever the model says
  doesn't move and the code lets you switch off.
- **Three docs tell incompatible stories about what the tokens even are.** Constitution/finance-README
  (canonical): MYCEL=soulbound reputation, SAP=demurrage medium, TEND=mutual-credit hours. The
  tend-economics paper: "SAP=local mutual credit, MYCEL=inter-bioregion bridge currency" (a soulbound
  score can't be a bridge currency). The Observatory still says "SAP=Sentient Action Points, CIV,
  HEARTH, KREDIT." A parallel `ECONOMIC_FRAMEWORK.md` defines a *four*-currency model with UBI/QF that
  no zome implements.
- **What backs SAP and who may mint it is named but never specified** — "mintable against energy
  certificates / agricultural production / fiat bridges," no oracle design, no reserve ratio, no
  redemption, no mint-authority chain. This vacuum is *exactly* where Class-A bugs #1–3 live.
- **TEND↔SAP settlement has no rate rule.** "Quarterly SAP settlement of bilateral clearing" with no
  answer to *how many SAP per TEND-hour* — quietly reintroduces labor pricing and breaks "1 hour = 1
  hour."
- **"Anti-reflexivity guards" are named as a core principle with zero specification**, while the
  system is deeply reflexive (MYCEL sets fees + TEND limits; TEND quality feeds MYCEL; SAP stakes are
  MYCEL-weighted).
- **Dead-code / stub economic paths**: `RewardDistribution`/`pending_rewards` never created or paid;
  the apprentice ±10 TEND limit keys off a `tier:apprentice:{did}` link no code creates (everyone gets
  the standard limit); `MAX_RECOGNITIONS_PER_CYCLE` enforced coordinator-side only. The **`hearth`
  zome is a non-functional stub** — `get_pool` always returns `None`, and pool-accounting, vote-tally,
  and `finalize_request` disbursement are all empty `Ok(())` / "Not yet implemented" no-ops despite
  `contribute()`/`vote()` claiming to work (`hearth/coordinator:246-699`). The `currency-mint` **confirm
  path never re-checks credit limits**, so N pending exchanges each passing the record-time check all
  confirm and blow past the ±200 limit (`exchanges.rs:213-405`).

---

### Class D — The economy is an island: cross-cluster rails are mostly broken or theater

A dedicated cross-cluster value-flow audit (22 edges, `OtherRole("finance")` call sites +
`routing_registry.rs`) found that **registered economic routes exceed actual working call sites by
roughly 2:1**, and the flagship integrations don't move value:

- ⭐ **Governance cannot spend money — via five independent breakages, mostly silent.** Automatic
  budget allocation sends a malformed payload to `dispatch_finance_call` and `let _ =`-swallows the
  deserialization error (`governance/budgeting:453`); milestone "disbursement" writes a local
  `Disbursement` entry with no finance call (`:468`); `execution::TransferCredits`, despite a
  "fail-closed, no phantom transactions" comment, only emits a governance *event*
  (`bridge/query.rs:514`); the manual treasury externs call `treasury::execute_governance_transfer` /
  `get_current_balance`, **neither of which exists** in finance; and finance's own
  `execute_dkg_allocation` calls `threshold_signing::verify_threshold_signature`, **which also doesn't
  exist** (that zome has one extern). **The democratic-budget → funds-move loop is 0% functional.**
- ⭐ **Two whole hApps were written against an imaginary finance API.** epistemic-markets calls a
  `kredit` zome and a `matl` zome and `treasury::receive_protocol_fee`; knowledge's claim-staking calls
  `finance_bridge::lock_funds_in_escrow` — **none of these exist** in finance. Meanwhile finance's
  *real* escrow (`staking::create_escrow`) has **zero external consumers**.
- **Silent-degradation reads**: identity's reputation aggregator does `io.decode::<f64>()` on a
  `MemberMycelState` struct (always fails → MYCEL dimension silently dropped); identity's economic-
  velocity read sends a bare `String` where finance expects a struct (always 0.0). Same "decorative"
  family as the `gate_civic` P0.
- **Theater**: attribution `reciprocity` and craft apprenticeship "stakes" have registered
  finance routes with intent comments ("reciprocity payments", "SAP escrow") but the source clusters
  **never call finance** — craft stores `stake_sap: u64` as a plain field, no escrow. music runs its own
  pre-SAP eth-address ledger and is *deliberately* firewalled from finance in the routing registry.
- **What genuinely works**: `energy/grid::settle_trade → payments::send_payment` (sweettest-proven),
  `supplychain::process_payment`, and `commons care-circles/boundary-contracts → tend::record_exchange`
  (best-effort, failure swallowed). marketplace is code-correct but **undeployable** — it isn't a role in
  `happs/mycelix-unified-happ.yaml`, so its `OtherRole` call can never resolve.
- **The tier gate compounds all of it**: even the working rails require `verify_participant_tier()`,
  which fails closed for any agent without seeded cross-cluster reputation — so the true happy path (real
  SAP moving cross-cluster) has, by the energy sweettest's own admission, **never been exercised
  end-to-end**.
- Doc phantom: the `VoteWeightConfig` "budget/constitutional/emergency presets" cited in CLAUDE.md
  **do not exist in code** — governance voting uses `PhiWeight`/`compute_vote_weight`.

## The plan (phased, each phase independently shippable)

### Phase 0 — Stop the bleeding (security), 1–2 weeks
Fold into the P0 author-binding pass, but **add the non-binding items it won't catch**:
- Bind every mint/debit/mint-adjust/stake/allocation/recognition to `action.author()` in **integrity**
  (mirror `staking:448`); close all wide-open `RegisterUpdate`/`RegisterDelete`.
- **DKG replay** (#6): add a nonce/allocation-id to the signed message + a consumed-signature record
  checked in integrity.
- **request_allocation** (#7): require manager/DKG authorization + emit a real allocation receipt.
- **Oracle** (#13): dedupe reports per reporter per window; raise `MIN_REPORTERS`; require a stake to
  report; slash on divergence-from-consensus (real penalty, not a 0.05 EMA floor).
- Decide lending/credit_scoring: **delete the deprecated crates now** (they're the source of #10–11
  and add nothing TEND doesn't) or gate them behind a feature flag off by default.
- Close the **bootstrap-open** window (#14): default-deny before governance registration.

### Phase 1 — Rebuild the SAP ledger on the TEND pattern (the root fix), 2–4 weeks
Generalize TEND's conserved-value stack to SAP: DHT link-read + `follow_update_chain`, 2-phase
`PendingBalanceAdjustment`, integrity-level conservation checks. **Move authorization AND conservation
into the integrity layer** so they can't be bypassed by calling a coordinator directly. Make treasury
pool balances conservation-linked to the SAP supply (close `treasury:88-122` disjointness). This
single phase dissolves Class B and Class-A #3, #8, #12.

**Started (2026-07-10):**
- `debit_sap` authorization-guarded (owner-only) — drain half of Class-A #3 closed.
- `transfer_sap(from, to, amount)` added — the sanctioned conservation-preserving move (verify
  caller==from, debit+credit); bridge `process_payment` migrated onto it, removing one raw-`credit_sap`
  cross-zome call site.

**Remaining for the mint half of #3 (the honest hard part):** the credit callers split into *transfers*
(now on `transfer_sap`) and *issuance* — bridge `deposit_collateral` (498), bridge `verify_fiat_deposit`
(2252), `currency-mint` (146). These mint SAP against collateral/fiat/community-currency, called
cross-zome by a *non-governance* user, so no coordinator-level caller guard is correct, and a
coordinator-only `mint_sap` would just relocate the hole. Note a key Holochain fact discovered here:
**within one cell all zomes share one agent, so capability grants cannot distinguish an internal
zome-to-zome call from an external client call** — the capability-grant idea does *not* work intra-cell.
The correct close is the **integrity-conservation model**: every issuance creates an author-bound
`SapMintRecord` (the entry already exists), and integrity validates each `SapBalance` *increase* against
a valid, unconsumed mint record or a matching debit — then raw `credit_sap` becomes private. That is the
real Phase-1 rebuild and should be its own carefully-verified unit (payments + bridge + currency-mint +
integrity, must compile together).

### Phase 2 — Reconnect the severed mechanisms, 1 week
- **Wire recognition → MYCEL** (#4): compute `mycel_score` by *aggregating received, author-bound,
  recognizer-weighted `RecognitionEvent`s* — never from caller-supplied fields. This is what makes the
  entire reputation layer real; it's currently theater.
  **Status: Recognition component landed (2026-07-10).** `update_mycel_score`
  (`recognition/coordinator`) now computes the Recognition component (20%) via
  `compute_recognition_component` → `aggregate_recognition` over the member's immutable
  `RecognitionEvent`s: each *distinct* recognizer counts once at their strongest weight, saturating at
  `RECOGNITION_SATURATION_WEIGHT` (5.0) — so a single spammer or a low-MYCEL Sybil ring can't
  manufacture it. `input.recognition` is now ignored. 5 pure unit tests. **Effect: pure
  self-declaration now tops out at 0.6 (participation 0.4 + longevity 0.2), *below* the Steward
  threshold (0.7)** — so Amber's Steward-gate can no longer be reached by self-report alone.
  **Longevity landed too (2026-07-10):** `MemberMycelState` now carries a `created_at`, and Longevity
  (20%) is derived from real account age via `longevity_from_age_secs` (capped at 24 months) instead of
  self-reported `active_months`. 2 more unit tests. **The self-declaration ceiling is now 0.4**
  (participation only) — below both Steward (0.7) and Citizen thresholds.
  **Remaining (follow-up):** Participation (40%) is the last `input`-supplied component — derive it from
  governance activity (cross-cluster, Phase 2.5) to fully close the loop.
- Implement or delete the dead reward/apprentice-limit paths; enforce `MAX_RECOGNITIONS_PER_CYCLE` in
  integrity.
- Fix the hibernation TEND-limit/integrity-cap mismatch (`tend/integrity:930`).

### Phase 2.5 — Make the cross-cluster rails real (Class D), 2–3 weeks
The economy is currently an island; this is where "the economy actually works for the ecosystem"
lives. Sequence by leverage:
- **Fix the governance→funds loop** (the biggest single gap): correct the `dispatch_finance_call`
  payload shape in `budgeting`, implement the missing finance externs
  (`treasury::execute_governance_transfer`, `get_current_balance`) or repoint callers at the real ones,
  make `TransferCredits`/milestone disbursement actually call `payments`, and implement
  `threshold_signing::verify_threshold_signature` (or drop the DKG path). A passed budget proposal must
  move SAP.
- **Stop swallowing cross-cluster errors**: replace `let _ =` / silent fallback on economic calls with
  surfaced failures (this is the same silent-degradation class as `gate_civic`). Fix identity's two
  decode/payload mismatches so reputation includes the economic dimension.
- **Reconcile the imaginary APIs**: either implement `kredit`/`matl`/`lock_funds_in_escrow`/
  `receive_protocol_fee` or migrate epistemic-markets + knowledge onto the real `staking::create_escrow`
  / `payments` / `treasury` surface. Give finance's real escrow at least one consumer.
- **Deployment truth**: add marketplace to the unified hApp (or mark it explicitly out), and prune the
  ~2:1 registered-but-unused routes (cafe, praxis, attribution, craft) so the routing registry reflects
  reality.
- Seed a **cross-cluster reputation fixture** so the tier-gated happy path can be exercised end-to-end in
  a sweettest — today no test moves real SAP across clusters through the gate.

### Phase 3 — Redesign the anti-accumulation levers the sim proved inert, 2–3 weeks (design + sim)
The model already tells us the current levers don't bind. Options to test in the ABM *before*
committing:
- **Charter the exempt floor** (it's the real lever) and make it progressive, or replace flat
  demurrage with **progressive demurrage above the floor** so the rate actually bites on large idle
  balances → attack the persistent SAP Gini ≈0.70.
- Re-examine whether the 4-year jubilee should exist at all (zero measured effect) or be replaced by a
  mechanism that targets the SAP distribution (where inequality actually lives) rather than MYCEL
  (already near-equal by growth caps).
- Specify the threat model where consciousness-gating beats flat/random exclusion (sim says it only
  wins above ~40% defectors) — or accept simpler gating for the common case.

#### Phase 3.A — **Amber**: a protected, demurrage-exempt class of SAP (new feature)

**Motivation.** Demurrage is a circulation tax: correct for idle wealth, wrong for people who
*can't* circulate (children, elders) and for savings that are *supposed* to sit still (a
multi-year project's runway, a child's education fund, an elder's reserve). Amber is a
demurrage-exempt (or demurrage-*limited*) tranche of SAP for exactly these holders/purposes. It is
the same instinct the Constitution already encodes as the commons-pool "waqf" exemption
(`treasury/integrity:524`), extended to protected individuals and project-scoped pools.

**Status: mechanism landed (v1), grant path Steward-gated; full identity-VC verification deferred.**
As of 2026-07-10 the core is implemented and unit-tested (see "Implementation status" below).
Previously `SapBalance` had only `member_did` / `balance` / `last_demurrage_at`; the only exemptions
were the flat 200-SAP `DEMURRAGE_EXEMPT_FLOOR` (`types/src/lib.rs:249`, applied identically to
*everyone*) and the commons-pool boolean — neither targeted at protected people or long-horizon
projects.

**The core risk (why "properly" is the operative word).** Any exemption is a demurrage-dodge
arbitrage vector: the moment Amber is tax-free, every large holder wants to reclassify normal SAP as
Amber. Amber is only safe if eligibility is *attested* (not self-declared), *capped*, and
*transfer-restricted*. Self-declaration here would recreate the exact MYCEL self-report bug (Class A
#4) with real money.

**Design ("done properly").**
- **Two Amber sub-types**, sharing one mechanism:
  - *Custodial Amber* — for a person, keyed to a **child/elder verifiable credential from
    `mycelix-identity`** (issuer + expiry on the credential). Demurrage-exempt (or reduced-rate)
    **up to a per-person cap**; balance above the cap decays normally.
  - *Project Amber* — for a governance-approved multi-year project pool. Exempt **while the project
    is active and milestone-current**; demurrage **re-arms** if the project stalls or the grant
    lapses. This is the waqf pool mechanism with a time-box + milestone gate, not a new primitive.
- **Eligibility is issued, never self-set.** The exemption carries an `issuer` (identity DID or a
  governance/DKG authority for projects) and an `expires_at`. Integrity validation binds the issuer
  and rejects self-issued exemptions.
- **Transfer-restricted.** Custodial Amber is spend-down/non-transferable; on any outbound transfer
  to a normal account it **converts to normal SAP and re-arms demurrage** (so Amber can't be a
  pass-through to launder idle wealth). Project Amber moves only via the project's approved
  disbursement path.
- **Bounded and expiring.** Per-person cap (governance param); credential expiry forces
  re-attestation (an elder credential shouldn't outlive the elder's membership). No permanent,
  uncapped exemptions.
- **Enforced in integrity, honored in the demurrage math.** Both the coordinator's demurrage
  application *and* the integrity validator must read the exemption; if only the coordinator honors
  it, it's bypassable like the rest of the cluster.

**Interaction with Phase 3.** Amber and the exempt floor are the same lever aimed at opposite goals
(shelter the vulnerable vs. tax the idle). Model them together in the ABM: does a capped Amber class
meaningfully change the SAP Gini or the compost-pool inflows the floor already dominates? Charter the
floor and Amber in the same amendment.

**ABM result (2026-07-10).** Ran the A/B/C in `simulations/macro-economy` (500 agents, 365 days, 5
seeds, 15% Amber cohort, 20,000-SAP cap; `run_amber_experiment.sh`):

| Scenario | SAP Gini | Total commons compost |
|---|---|---|
| Baseline (no Amber) | 0.6914 | 24,814 |
| Amber → low balances (children/elders proxy) | 0.6909 | 22,433 (−10%) |
| Amber → high balances (whale-abuse case) | 0.6931 | 11,615 (−53%) |

Conclusions, all confirmed:
1. **Amber for its intended targets is safe — it barely moves inequality** (Gini 0.6914→0.6909, within
   noise). The universal 200-SAP floor already shelters most low-balance holders, so extending Amber to
   them changes almost nothing about the distribution. This *validates shipping it*: it doesn't distort
   the economy.
2. **The binding cost of Amber is commons revenue, not inequality.** Even intended use forgoes ~10% of
   compost; the abuse case (whales sheltered) **halves it (−53%)** while nudging Gini *up*. Demurrage
   revenue is heavily concentrated in a few large balances, so sheltering 15% of agents — if they're the
   big ones — guts the compost stream.
3. **Therefore the anti-abuse guards protect the commons compost flow, not the Gini**, and the
   **per-person cap is the key knob.** A 20,000-SAP cap fully shelters even fairly large holders; a
   **tighter cap (≈2,000–5,000 SAP) would protect a genuine child/elder fund while sharply limiting
   compost leakage.** Recommend lowering the default `AMBER_CUSTODIAL_CAP_MICRO_SAP` accordingly and
   keeping Amber strictly off large balances (the Steward-gate + issuer≠member guards already shipped do
   this; the cap bounds the damage if one leaks).
4. Consistent with the earlier finding that SAP Gini (~0.69) is structurally sticky and demurrage barely
   touches it — Amber is not a lever on inequality either way, which is exactly what you want from a
   *protective* exemption.

**Scope — implementation breakdown (est. ~1.5–2.5 weeks, best done *after* Phase 1's ledger rebuild).**

| # | Task | Files / anchors | Notes |
|---|------|-----------------|-------|
| 1 | **Decide the policy inputs** (blocking, user-owned) | — | Per-person Amber cap; exempt vs. reduced-rate; which identity credential types count as child/elder; project-approval authority (governance vote? DKG?). These are the only non-mechanical decisions. |
| 2 | **Schema**: add exemption to the balance entry | `payments/integrity/src/lib.rs:113` (`SapBalance`) + `types/src/lib.rs` | `pub exemption: Option<AmberExemption>` where `AmberExemption { class: AmberClass /*Custodial/Project*/, issuer: String, cap_micro_sap: u64, expires_at: Timestamp }`. Additive/optional → backward-compatible with existing balances. |
| 3 | **Demurrage math**: honor the cap | `types/src/lib.rs:920` (`compute_demurrage_deduction`) + call sites `payments/coordinator/src/lib.rs:72,96,379,464,…` | New helper `compute_demurrage_with_exemption(balance, exemption, floor, rate, elapsed, now)`: if exemption valid & unexpired, exempt `min(balance, cap)`; decay only the excess; expired exemption falls back to the normal floor path. Extend the existing fuzz/proptests (`types/fuzz/…`, `types/src/lib.rs:2028`) to assert "capped amount never decays; excess decays normally; expired ⇒ normal." |
| 4 | **Integrity validation**: bind the issuer | `payments/integrity/src/lib.rs` (validate create/update of `SapBalance`) | Reject self-issued exemptions (`issuer == member_did` for custodial); require issuer ∈ {identity credential authority, project authority}; enforce `cap ≤ governance max`; this is the anti-arbitrage gate. Mirrors the correct author-binding template at `staking:448`. |
| 5 | **Grant/verify path**: issue Custodial Amber from an identity credential | new coordinator fn in `payments` + `call(OtherRole("identity"), reputation-aggregator/credential-verify)` | Verify the child/elder VC via the identity cluster (fixing, or reusing the fix for, the decode mismatch noted in Class D). Set `exemption` on the recipient's balance. |
| 6 | **Transfer rule**: convert-and-re-arm | `payments/coordinator/src/lib.rs` `send_payment`/`debit_sap` paths (~996, 333–523) | On outbound transfer from Custodial Amber to a normal account, strip the exemption and stamp `last_demurrage_at = now`. |
| 7 | **Project Amber**: milestone-gated pool exemption | `treasury` (reuse `demurrage_exempt` bool + add `exempt_until` / milestone link) | Extend the existing commons-pool exemption with an expiry + a governance/milestone re-check; re-arm on lapse. Smaller than the custodial path — it's mostly a time-box on an existing flag. |
| 8 | **Tests + sim** | `mycelix-finance/tests/`, `simulations/macro-economy/` | Sweettest: grant Amber → verify no decay up to cap → transfer → verify re-armed. ABM: add an Amber-holder cohort, measure effect on SAP Gini and compost inflow vs. the floor alone. |
| 9 | **Charter it** | `THE ECONOMIC CHARTER`, Constitution §VIII.5 | Amber + the (currently unchartered) exempt floor in one amendment; state the cap, eligibility, and transfer rule so it's not another undocumented lever. |

**Dependencies / sequencing.** Task 5 depends on the identity-cluster decode fix (Class D / Phase
2.5). All of it is cleaner *after* Phase 1 moves the SAP ledger to the TEND conservation pattern and
authorization into integrity — otherwise Amber's integrity checks sit next to a ledger that's still
bypassable. Recommend: land Phase 1, then Amber tasks 2–4 (the mechanism), then 5–7 (the grant/
transfer paths), then 8–9.

**Implementation status (2026-07-10).** Tasks 2, 3, 4, and a Steward-gated slice of 5 are **landed**:
- `types`: `AmberClass`, `AmberExemption` (`is_active`), `AMBER_CUSTODIAL_CAP_MICRO_SAP` /
  `AMBER_MAX_CAP_MICRO_SAP`, and the pure `compute_demurrage_with_exemption` — 5 unit tests
  (shelters-up-to-cap, below-cap-zero, expired-falls-back, none==floor, sub-floor-cap-uses-floor);
  `cargo test -p mycelix_finance_types` = 160 passing.
- `payments/integrity`: `exemption: Option<AmberExemption>` on `SapBalance` (`#[serde(default)]` →
  backward-compatible) + structural validation (issuer valid DID, **no self-issue**, cap ≤ ceiling,
  **must expire**) + 5 validation tests.
- `payments/coordinator`: all four individual-balance demurrage sites honor the exemption;
  `grant_amber_exemption` / `revoke_amber_exemption` externs, **Steward-tier-gated**, issuer bound to
  caller and ≠ holder, cap ceilinged, expiry-in-future required.

**Still deferred** (explicit, not silently skipped): full child/elder **identity-VC verification** at
grant time (task 5 currently trusts Steward judgement, not a credential — needs the Phase 2.5 identity
decode fix); **transfer-lock / convert-and-re-arm** (task 6 — not required for the v1 anti-abuse story,
which rests on gated issuance + cap + issuer≠member, but worth adding); **Project Amber** milestone
pools (task 7 — the four `HearthSapPool` demurrage sites still use the plain floor); **ABM cohort** and
**charter amendment** (tasks 8–9). Note the grant path does a single optimistic update (no retry loop)
— acceptable for a rare admin op, but it inherits the Class-B ledger-race caveat until Phase 1 lands.

### Phase 4 — Specify the unspecified, 2–3 weeks (charter/design)
- **SAP backing & mint authority**: pick one model (fully-reserved bridge / asset-certificate /
  mutual-credit), specify oracle, reserve ratio, redemption, mint-authority chain. This is the design
  prerequisite for Phase 0 #1–2 being *correct*, not just *authenticated*.
- **TEND↔SAP settlement rate rule** (fixed peg? governance-set? none-and-keep-them-non-convertible?).
- **Exchange/transferability boundary**: is the orphaned Polygon-DEX design (`HOLOCHAIN_CURRENCY_
  EXCHANGE_ARCHITECTURE.md`) in or out? Reconcile with "SAP is not an investment contract."
- **Anti-reflexivity guards**: write the actual damper rules (e.g. cap TEND-quality→MYCEL flow, cap
  MYCEL→fee advantage) the docs promise.
- Abuse-analyze the **exit protocol** (TEND forgiveness on exit at the ±120 emergency limit = free
  defection).

### Phase 5 — Coherence & evidence, ongoing
- **One canonical taxonomy**: make Constitution §VIII.5 authoritative; fix the tend-economics
  role-swap, retire the Observatory's stale SAP/CIV/HEARTH/KREDIT vocabulary, subordinate or delete
  `ECONOMIC_FRAMEWORK.md`'s four-currency model; settle TEND's core-vs-optional status.
- **Extend the ABM to test what's untested**: sybil-swarm earning MYCEL via mutual recognition (now
  that #4 is fixed); exchange/peg-attack and speculation; TEND default risk at the ±120 emergency
  limit (limits loosen exactly when default risk peaks — WIR pattern is asserted, never stress-tested);
  treasury compost flow volumes; progressive-demurrage vs flat.

---

## Recommended first move

Phases 0 → 1 → 2 → 2.5 in that order: they convert the economy from "paper guarantees" to "real
guarantees" and from "island" to "wired," and are pure engineering with clear file:line targets. The
**highest-leverage single change for internal integrity** is **Phase 2's recognition→MYCEL wiring (#4)**
combined with **Phase 1's ledger rebuild** — together they make reputation and money *actually conserved
and actually earned*, which every other mechanism (fees, staking, gating, mediator eligibility) silently
depends on. The **highest-leverage change for the ecosystem** is **Phase 2.5's governance→funds loop** —
right now a passed budget proposal moves zero SAP, which arguably makes the whole finance cluster
non-load-bearing for the rest of Mycelix. Phase 3's demurrage redesign is the highest-leverage *economic*
change and is cheap to explore because the ABM already exists. Phases 4–5 are design/charter work that
can run in parallel with a separate owner.

## The two headline sentences, if you read nothing else

1. **Internally, the economy's guarantees are unenforced**: reputation is self-declared, SAP has
   several unauthenticated mint paths, and only TEND actually conserves value on the DHT.
2. **Externally, the economy is an island**: the governance→funds loop is 0% functional, two hApps call
   a finance API that doesn't exist, and the real happy path has never moved SAP across clusters
   end-to-end.
