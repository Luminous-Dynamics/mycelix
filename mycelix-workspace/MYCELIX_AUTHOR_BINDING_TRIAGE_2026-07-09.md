# Mycelix author-binding triage — turning "567 candidates" into a real worklist

**Created 2026-07-09.** The integrity-provenance audit (`scripts/audit-integrity-provenance.sh`)
reports **567 integrity validators without a per-function `action.author` check** across the
Mycelix clusters. That number is a *heuristic candidate list*, not a bug count — the script
itself says so. This doc triages the candidates into the classes that actually matter, so
fix-sessions can go straight to the real mechanical work and the roadmap can stop treating "567"
as "567 vulnerabilities."

> **Status 2026-07-09: 5 high-value clusters triaged** (identity, governance, finance, knowledge,
> civic — every domain where forgery = money/votes/justice/claims/identity). Result: **76 real
> mechanical Class-A binds + 97 Class-D authority gaps**, not 567 vulns. The audit both *over*counts
> (most flags are false positives) and *under*counts (real bugs hide in validators it can't see). See
> the scoreboard and per-cluster sections below; the Class-D mechanism is designed in
> `MYCELIX_CAPABILITY_GRANT_DESIGN_2026-07-09.md`.

> **Threat model** (Holochain 0.6): a malicious agent can run a *modified coordinator* that
> commits any entry with any field values. The only defense is the integrity zome's `validate_*`
> callbacks, which run on every validating authority. An entry with a self-declared owner field
> (a DID / AgentPubKey the committer *claims to be*) that isn't checked against `action.author`
> is forgeable → vote forgery, self-mint, DID takeover.

---

## The four classes (why 567 ≠ 567 bugs)

Every flagged validator falls into one of these. Only **Class A** is a mechanical author-binding fix.

### Class A — REAL, mechanically fixable (`validate_create_*` with a forgeable owner field)
The entry has an owner field, and the coordinator sets that field to the *caller's own* key. Fix =
bind the field to `action.author` with a small pure helper + a rejection test. This is the
established pattern already applied to voting, staking, recognition, did_registry,
verifiable_credential, attribution (pledge/dependency/usage), praxis/dao, commons/boundary-contracts.
**These are the ones worth finding and fixing.**

### Class B — FALSE POSITIVE: central-dispatcher update binding (the audit can't see it)
Most crates bind author **once**, in the `validate(op)` dispatcher's `FlatOp::RegisterUpdate`
(and `RegisterDelete`) arm:
```rust
FlatOp::RegisterUpdate(update) => {
    let original = must_get_valid_record(update.original_action_hash)?;
    if *original.action().author() != action.author {
        return Ok(ValidateCallbackResult::Invalid("Only the original author can update".into()));
    }
    ...
}
```
This single check covers **every** `validate_update_*` in that crate — but the audit's
per-function regex only scans individual function bodies, so it flags all of them as "unbound."
**Any `validate_update_*` in a crate whose `validate()` has this arm is a false positive.**
Confirmed in e.g. `mycelix-identity/zomes/recovery` (lines 268/286/295) and `.../bridge` (260/269).
*Triage rule: before touching any `validate_update_*`, grep the crate's `validate()` for a
`RegisterUpdate` author comparison. If present → false positive, move on.*

### Class C — FALSE POSITIVE: no forgeable owner field
The entry legitimately has nothing to bind to:
- **Encrypted payloads** (`EncryptedEntry`): `recipient_key_id` names the addressee, not the committer.
- **Announcements / registrations** (`HappRegistration`): anyone may publish; no owner field.
- **Lookup / audit-trail records** (`IdentityQuery`): `did` is the *subject looked up*, a third party.
- **On-behalf-of flows** (payments, mints, endorsements of others): the committer legitimately
  isn't the subject — binding to author would *break* the feature. These need an *authority* check
  (Class D), not author-binding.

### Class D — REAL but NOT author-binding-fixable: call-provenance / capability-grant gap
The dangerous residual. The entry's authority claim is a **free-text label or cross-DNA call
origin**, not an agent-comparable key — so `action.author` can't distinguish legitimate from
forged. Examples:
- `IdentityReputation.source_happ` — any agent can `report_reputation` claiming to *be*
  "mycelix-finance" and inject an arbitrary score for any victim DID; it folds straight into MATL.
- `IdentityVerification` — cached cross-zome computation a malicious coordinator can fabricate
  wholesale (`is_valid:true, matl_score:1.0`); validate() can't re-derive it.
- `mycelix-justice` Arbitration/Judgment — needs `must_get` juror-authorization.

Because `CallTargetCell::OtherRole` cross-DNA calls within one hApp share the **same agent key**,
author-binding is structurally insufficient here. **These need a capability-grant / call-provenance
mechanism** (this is the deeper form of MASTER_ROADMAP P0 #1, and the real hard work). Field-binding
is the wrong tool; do not "fix" these by binding a label to author.

---

## Cluster: mycelix-identity — FULLY TRIAGED (0 Class A remaining)

The 2026-07-08 P0 pass already reviewed and documented every one of identity's 18 hits inline.
Result:

| Zome / validator | Class | Note |
|---|---|---|
| recovery: `validate_update_recovery_config`, `validate_update_recovery_request`, `validate_update_self_recovery_request` | B | central `RegisterUpdate` binding (lines 268/286/295) |
| recovery: `validate_create_self_recovery_request` | C (by design) | committer is *necessarily* a new key (original lost); secured by anchor-possession, not author |
| bridge: all `validate_update_*` (reputation, happ_registration) | B | central binding (lines 260/269) |
| bridge: `validate_create_happ_registration` | C | announcement, no owner field |
| bridge: `validate_create_identity_query` | C | `did` is the subject looked up |
| bridge: `validate_create_bridge_event` | C | event-stream record, `subject` free-text |
| bridge: `validate_create_identity_reputation` | **D** | `source_happ` free-text label — forgeable MATL injection; needs capability-grant |
| bridge: `validate_create_identity_verification` | **D** | cached cross-zome computation; validate() can't re-derive |
| mfa / vc / trust_credential: `validate_create_encrypted_entry` | C | `recipient_key_id` = addressee, not committer |
| revocation / education: `validate_update_*` | B | central binding |

**Identity actionable Class A count: 0.** Residual real risk = 2 Class-D gaps (reputation,
verification) — tracked under the capability-grant workstream, not author-binding.

---

## Cluster: mycelix-finance — TRIAGED (25 Class-A, 41 Class-D, 20 false-positive)

**Same cluster-wide exposure as governance, and higher stakes (real money):** no finance zome
inspects `action.author()`; every crate's `validate()` has `FlatOp::RegisterUpdate(_) => Ok(Valid)`
(no central updater-binding), so **25 `validate_update_*` functions can rewrite owner/amount fields
on anyone's entry** — a parallel attack surface as serious as the create side. Only 5 of 30 updates
even have a `must_get` immutability check. Several create validators take **no `action` param at all** —
any fix must first thread `EntryCreationAction`/`action.author()` into the signature.
Paths: `mycelix-workspace/mycelix-finance/zomes/<zome>/integrity/src/lib.rs`.

### ⚠️ TOP FINDING — Class D, most dangerous in the cluster:
`payments/validate_create_sap_mint_record` (**payments:672**) — **unauthenticated SAP self-mint**,
zero authority gating (only checks DID format + amount>0). Needs a mint-authority `must_get`
(governance/treasury authorization), NOT author-binding (recipient≠minter). Followed by tend's 5
unbound money-balance validators (create/update balance 899/940, hearth_balance 1312/1338) and
`currency-mint/validate_create_pending_minted_adjustment` (586, forgeable balance vector).

### Class A — REAL, clean, fix first (25). Highest-value money-in paths at top:
| zome | validator | line | owner field | note |
|---|---|---|---|---|
| bridge | validate_create_collateral_bridge_deposit | 601 | `depositor_did` | **mints SAP** |
| payments | validate_create_payment | 430 | `from_did` | debit forgery |
| treasury | validate_create_contribution | 350 | `contributor_did` | money-in |
| hearth | validate_create_contribution | 470 | `contributor_did` | money-in |
| credit_scoring | validate_create_collateral_record | 194 | `owner_did` | forges collateral ratio |
| bridge | validate_create_collateral_registration / _multi_collateral_position / _energy_certificate / _agricultural_asset | 490 / 997 / 870 / 937 | `owner_did`/`holder_did`/`producer_did` | collateral value forgery |
| bridge | validate_create_cross_happ_payment | 443 | `from_did` | cross-hApp debit forgery |
| payments | validate_create_exit_record | 696 | `member_did` | forges others' TEND forgiveness |
| cgc | validate_create_gift / _allocation | 360 / 418 | `giver_did`/`member_did` | spend anyone's allocation |
| lending | validate_create_loan_offer | 234 | `lender_did` | |
| credit_scoring | validate_create_credit_profile | 131 | `did` | self-score to 1000 |
| hearth | validate_create_vote | 539 | `voter_did` | allocation vote forgery |
| recognition | validate_create_allocation | 482 | `recognizer_did` | reuse in-zome `require_recognizer_is_author` |
| tend | validate_create_quality_rating / _exchange / _listing / _request / _dispute_case | 1059 / 774 / 960 / 1013 / 1122 | `rater_did`/`provider_did`/`requester_did`/`complainant_did` | |
| bridge | validate_create_covenant | 715 | `beneficiary_did` | |

### Class D — need `must_get`/authority/immutability, NOT naive author-binding (41):
The mint/balance authority set (sap_mint 672, tend balances 899/940/1312/1338, pending_minted_adjustment
586, fiat_bridge_deposit 800), all 25 unmitigated `validate_update_*` (fund-moving: payment_channel
update 567, lending update_loan 224, hearth update_request 526, treasury allocation/pool updates…),
two-party/on-behalf flows (payment_channel create 538, loan create 192, disbursement 563, escrow 689),
DAO-DID authority (treasury commons_pool/savings_pool, currency-mint create_currency 327, cgc alias 545),
and reputation/credit-history attestation (recognition mycel_state 331/414, staking update_stake 514,
credit_scoring payment_record 168/181). See full agent worklist for per-function `must_get` design.

### Class B/C — false positives (20): update-transition/immutability-only validators, no-owner-field
structural/reference entries, and DAO/subject-DID fields whose residual gap is a *separate* authority
model (not author-binding). No author-binding fix needed.

## Cluster: mycelix-governance — TRIAGED (26 Class-A, 21 Class-D, 25 false-positive)

**Critical cluster-wide finding: governance is the OPPOSITE of identity.** None of its 8 integrity
crates does *any* author-binding — every `validate_*` discards `_action`, and every `validate()`
dispatcher no-ops the update path (`FlatOp::RegisterUpdate(_) => Ok(Valid)`). So there is **no
central check to lean on (no Class B here)** and **zero author provenance anywhere in the cluster** —
every update path also needs per-function binding. This is the most exposed cluster found.
Paths below are `mycelix-workspace/mycelix-governance/zomes/<zome>/integrity/src/lib.rs`.
All Class-A fixes reuse voting's existing helper `require_voter_is_author` (lib.rs:1485).

> **LANDED 2026-07-10:** the 3 vote-forgery binds are done + verified — `validate_create_vote`
> (shipped `c33715a17d`), `validate_create_phi_vote` + `validate_create_quadratic_vote` (this
> session, verified `cargo test -p voting_integrity --lib` = 71/0). All reuse `require_voter_is_author`
> (`voter == did:mycelix:{action.author}`). Remaining governance Class-A (delegation, council
> membership, threshold-signing member/share, bridge consciousness snapshots, etc.) still to do.

### Class A — REAL, fix first (26). Highest-severity (vote/proposal forgery) at top:
| zome | validator | line | owner field | severity |
|---|---|---|---|---|
| voting | validate_create_phi_vote | 1635 | `voter` | ✅ DONE (b57519626e) |
| voting | validate_create_quadratic_vote | 1702 | `voter` | ✅ DONE (b57519626e) |
| bridge | validate_create_weighted_vote | 2166 | `voter_did` | **VOTE FORGERY** |
| execution | validate_create_veto | 592 | `guardian` | HIGH (forged veto freezes any timelock) |
| execution | validate_create_override_vote | 603 | `voter_did` | HIGH (swings 67% override) |
| proposals | validate_create_proposal | 472 | `author` | PROPOSAL SPOOF |
| councils | validate_create_membership | 746 | `member_did` | HIGH (council impersonation) |
| voting | validate_create_delegation / validate_update_delegation | 1533 / 1569 | `delegator` | HIGH (steal voting power) |
| threshold-signing | validate_create_member | 555 | `agent` | HIGH (occupy DKG slot) |
| proposals | validate_create_amendment | 504 | `proposer` | HIGH |
| constitution | validate_create_amendment | 476 | `proposer` | HIGH |
| proposals | validate_create_contribution | 528 | `contributor` | MED |
| execution | validate_create_execution | 581 | `executor` | MED |
| threshold-signing | validate_create_share / _violation_report / _pq_attestor | 696 / 769 / 853 | `signer` / `reporter` / `agent` | MED |
| voting | validate_create_eligibility_proof | 1896 | `voter_did` | MED |
| bridge | 8× consciousness_snapshot/_attestation/_gate/_history/_value_alignment/_k_vector/_federated_reputation/_consensus_participant | 1736–2130 | `agent_did` | MED (impersonation) |

### Class D — needs `must_get` authority or signature check, NOT naive author-binding (21):
voting `voice_credits` create/update (1732/1761 — self-mint: needs grantor authority), `verified_vote`
(1966 — relay≠voter), `proof_attestation` (2021 — oracle Ed25519 verify vs allowlist); proposals
`validate_update_proposal` (483 — tally-driven transitions), `update_amendment` (515); councils
`update_membership` (757 — steward remove); threshold-signing `update_member` (566), `create_signature`
(685 — aggregator), `hash_commitment`/`hash_reveal` (814/842 — dealer-slot spoof); execution
`update_timelock`/`override_result`/`fund_allocation` create+update (559/614/625/636 — treasury
authority); constitution `update_amendment` (487); jurisdiction create/update (150/161 — authority
claim); bridge `matl_score` (2055 — self-asserted score, `report_reputation` class), `consensus_round`
(2214 — leader election), `slashing_record` (2257 — needs evidence/council authority).

### Class B/C — false positives (25): tallies/reflections (aggregates, no owner), councils Council/decision
records, threshold committee group-record, execution/constitution charter/parameter/timelock, bridge
query/ref/event/execution_req (free-text `source_happ` labels). No fix needed (some have a *separate*
"no authority model on who may create" gap, distinct from author-binding).

---

## ⚠️ Audit blind spot (found via knowledge cluster, 2026-07-09)

The audit regex only matches free-function validators named `validate_(create|update)_*`. It is
**blind** to two real patterns that also gate entries:
- **struct-method validators** (`impl Entry { fn validate(&self, ..) }`, routed from `validate(op)`),
- **differently-named free functions** (`validate_fact_check_result`, `validate_market_evidence`).

These fall through to permissive `_ => Ok(Valid)` update arms and are **never counted in the 567**.
Knowledge's 30 flagged hits were all already-hardened, but 3 genuinely exploitable **Class-A** bugs +
2 **Class-D** oracle gaps lived in unflagged zomes (factcheck, markets_integration, dkg_integrity).
**Implication: the real bug population is larger than 567 and partly *outside* the audit's list.** A
second-generation audit should also scan for `fn validate(&self` and any `validate_*(.. ) ->
ExternResult<ValidateCallbackResult>` regardless of name. Until then, per-cluster human triage (like
this doc) is the only way to catch them.

## Taxonomy validation against an already-fixed cluster (property)

Cross-checked against `mycelix-property` — the roadmap's "worst found," now fixed in the 2026-07-08
pass. Its 11 residual audit hits are **0 Class A**, exactly as the taxonomy predicts for a correctly-
fixed cluster: the residue is Class C (`CommonResource` is group-owned from creation; `usage_right`'s
`holder_did` is the grant *recipient*, not the committer) and Class D (disputes'
`validate_update_property_dispute`/`_ownership_claim` are *deliberately* unbound with an inline comment
calling for "a dedicated authorization-model pass" — i.e. the capability-grant design). Create-path
claimant fields *are* bound (`did:mycelix:{action.author()}`), with rejection tests. This confirms:
(a) the taxonomy is accurate, (b) "fixed" clusters correctly leave only C+D residue, and (c) the
Class-D capability model is the documented next step, not an invention. (Water, also fixed, matches.)

## Scoreboard — 5 high-value clusters triaged (all money/vote/justice/claim/identity domains)

| Cluster | Class A (fix) | Class D (authority) | False positive | Central update-binding? |
|---|---|---|---|---|
| identity | **0** (already bound) | 2 | 16 | ✅ yes |
| governance | **26** incl. live vote forgery | 21 | 25 | ❌ no |
| finance | **25** incl. SAP self-mint | 41 | 20 | ❌ no |
| knowledge | **3** (off audit-list) | 4 | 25 | mixed (crates hardened Jul-9) |
| civic | **22** incl. forged evac orders | 29 (justice verdict surface) | 44 | mixed |
| **TOTAL** | **76 real Class-A binds** | **97 Class-D authority gaps** | ~130 | |

**The honest headline: the real work is 76 mechanical create-path bindings + 97 authority gaps that
one capability-grant mechanism closes — NOT "567 vulnerabilities."** These 5 clusters cover every
high-forgery-stakes domain (money, votes, justice, claims, identity). Two findings materially change
the picture vs the raw audit: (1) the audit **undercounts** — real Class-A bugs live in zomes it can't
see (knowledge's factcheck/markets/dkg, all of justice); (2) the audit **overcounts** — most flagged
hits are Class B/C false positives.

Remaining untriaged clusters are lower-stakes resource/coordination domains (commons 139, housing 25,
care 17, energy 19, mutualaid 8, health 8, water 6 [done], property 11 [done], plus emergency/media
already covered under civic). Expect a similar A/B/C/D split; grep each crate's `validate()` for
`RegisterUpdate` author-binding first, and don't trust the audit list to be complete — scan for
struct-method and oddly-named validators too (see blind-spot note).

## Cluster: mycelix-knowledge — TRIAGED (audit hits all already-fixed; 3 NEW Class-A off-list)

The 30 audit-flagged knowledge validators are **all in crates already hardened on 2026-07-09** — no
action. The real, unaddressed forgeries are in **3 zomes the audit never flagged** (blind-spot above):

### Class A — REAL, off-audit-list, fix first:
| zome | validator | file:line | owner field | note |
|---|---|---|---|---|
| factcheck | validate_fact_check_request | factcheck/integrity/src/lib.rs:431 (field :74) | `requester: AgentPubKey` | thread Create action into StoreEntry arm (:508), assert `requester == action.author` |
| markets_integration | validate_verification_market_request | markets_integration/integrity/src/lib.rs:455 (field :83) | `requester: AgentPubKey` | bind at :575; also `Cancelled.cancelled_by` (:143) forgeable |
| dkg_integrity | DisputeEntry::validate (routed :325) | dkg_integrity/src/lib.rs:246 (field :214) | `challenger: AgentPubKey` | add `if dispute.challenger != action.author { Invalid }` in create arm |

### Class D — REAL, oracle authority (needs capability grant, highest priority):
| zome | validator | file:line | entry | why |
|---|---|---|---|---|
| factcheck | validate_fact_check_result + permissive update | factcheck/integrity/src/lib.rs:472, :505-533 | FactCheckResult | anyone can commit AND overwrite an authoritative verdict (cross-author update via `_ => Valid`); mutates claim credibility. Needs oracle-authority check + RegisterUpdate author arm |
| markets_integration | validate_market_evidence + permissive update | markets_integration/integrity/src/lib.rs:501, :569-601 | MarketEvidence | fabricate "market resolved" with no authorized-resolver proof; mutates `matl_weighted_confidence`. Oracle-authority `must_get` needed |
| inference | validate_update_author_reputation | inference/integrity/src/lib.rs:764 | AuthorReputation | already in-code flagged; forged `author_did` inflates any author's rep |
| invention | validate_create_ledger_entry | invention/integrity/src/lib.rs:753 | RoyaltyLedgerEntry | already in-code flagged; forge a royalty debt against a victim |

Counts: **3 Class-A (off-list)**, 8 Class-B, 17 Class-C, 4 Class-D. The two oracle-output entries
(FactCheckResult, MarketEvidence) are the highest priority — epistemic-authority records a modified
coordinator can forge or overwrite today.

## Cluster: mycelix-civic — TRIAGED (22 Class-A, 29 Class-D; justice is the worst Class-D hole)

Counts: **A=22, B=13, C=22 (+9 civic-bridge doc-comment noise), D=29.** Two structural findings:

1. **Justice = the largest single Class-D hole in the repo, and the audit couldn't see it.** Every
   justice `StoreEntry` create arm dispatches only `&entry` to `validate_*` — the **action is never
   passed in**, so no create validator can bind *anything* to the committer. The whole verdict surface
   is forgeable: any agent can commit a finalized `Decision`, self-appoint an `Arbitration` panel, or
   issue an `Enforcement` order (FundsTransfer / AssetFreeze / AccessRevocation). Fix = change the
   create-arm dispatch to pass the action, THEN add `must_get` authority checks (Class D). Same 4
   validators repeat identically across all 4 justice zomes (arbitration/cases/enforcement/restorative).
2. **DID ≠ AgentPubKey** in media + justice. Owner fields are `did:holo:…`/`did:mycelix:…` strings, so
   the Class-A fix is `field == format!("did:holo:{}", action.author)` (the derivation
   `create_gallery`/`create_exhibition` already use), **not** raw `!= action.author`.

> **STATUS 2026-07-10 — emergency Class-A COMPLETE (10/10), justice in hand, media RE-CLASSIFIED:**
> - ✅ **emergency: all 10 landed + verified.** This session: comms (broadcast/message/channel),
>   incidents (disaster/incident-update), resources (resource/request), triage — 8 fixes, per-crate
>   `cargo test` green (97/96/87/82, 0 failed). Another session (`ce9bd2f09a`): coordination
>   (assignment/checkpoint). (My earlier "coordination deferred — fixture mismatch" note was a
>   FALSE ALARM: `fake_create` author and the `agent()` fixture are BOTH `[1u8;36]`; I'd compared
>   against the wrong line. It was clean, and the other session landed it.)
> - 🔄 **justice: handled by `ce9bd2f09a`** (arbitration/cases/enforcement/evidence/restorative —
>   coordinators changed to pass `action`, integrity binding added). Don't duplicate.
> - ⚠️ **media publication/factcheck/curation: NOT clean Class-A — RE-CLASSIFY to Class D.**
>   Verified 2026-07-10: their coordinators COPY a caller-supplied DID (`create_collection`
>   coord:78 `input.curator_did`, `feature_content` coord:162, `create_fact_check` coord:37,
>   `dispute_fact_check` coord:218, `create_publication` coord:37) — and existing tests use
>   arbitrary `did:example:*` strings, NOT `did:holo:{agent}`. Only gallery/exhibition derive
>   `did:holo:{agent}` (coord 643/719) and are already correct. A naive
>   `field == format!("did:holo:{}", action.author)` bind would REJECT every existing test and any
>   real client using another DID scheme. Correct fix = DID→AgentPubKey resolution via the identity
>   cluster (a `must_get`/cross-zome authority check), i.e. Class D — OR change the coordinators to
>   derive `did:holo:{agent}` (a behavioral change that needs client sign-off). **Do NOT apply a
>   plain integrity bind here.** So civic's real clean Class-A count is the 10 emergency fixes
>   (done) + justice case/appeal (via `ce9bd2f09a`), not the media rows.

### Class A — REAL, fix first. Emergency one-liners are turnkey (real-world coercion):
| zome | validator | file:line | owner field | severity |
|---|---|---|---|---|
| emergency-comms | validate_create_broadcast | integrity:296 | `issued_by` | **forged evac/all-clear order** |
| emergency-incidents | validate_create_disaster | integrity:299 | `declared_by` | HIGH (false disaster declaration) |
| emergency-coordination | validate_create_assignment | integrity:325 | `assigned_by` | HIGH (forged dispatch) |
| emergency-comms | validate_create_message / _channel | integrity:230 / 274 | `sender` / `created_by` | plain AgentPubKey, one-liner |
| emergency-incidents | validate_create_update | integrity:524 | `author` | one-liner |
| emergency-coordination | validate_create_checkpoint | integrity:383 | `agent` | one-liner |
| emergency-resources | validate_create_resource / _request | integrity:234 / 310 | `owner` / `requesting_team` | one-liner |
| emergency-triage | validate_create_triage | integrity:222 | `triaged_by` | one-liner (leave `patient_id`) |
| justice ×4 | validate_case / validate_appeal | 876/934/876/945, 1319/1377/1319/1388 | `complainant` / `appellant` (DID) | DID-resolved bind |
| media-publication | validate_create_publication | integrity:241 | `author_did` | DID-derived bind |
| media-factcheck | validate_create_fact_check / _dispute | integrity:226 / 354 | `checker_did` / `disputer_did` | DID-derived |
| media-curation | validate_create_collection / _featured_content | integrity:294 / 415 | `curator_did` / `featured_by` | DID-derived (`did:holo:{author}`) |

### Class D — justice authority surface (highest) + media oracle/consent gates (29):
justice ×4 `validate_decision` (verdict), `validate_enforcement` (asset seizure), `validate_arbitration`
(panel self-appointment), `validate_mediation`, `validate_restorative`, `validate_evidence` (verifier
flag), plus `validate_restitution`; media `validate_create_attribution` (credit others), `_royalty_rule`
(forge terms for any publication), `_endorsement` (endorse in another's name), `_quality_score` (forge
score=1.0). All need `must_get` authority/consent, not author-binding — the capability-grant design.

## Recommended sequencing

1. **Fix Class A first** (mechanical, per-function, testable) — finance & governance are highest
   value (money/votes). Each: pure helper + `validate_create_*` binding + rejection test, mirroring
   the shipped examples. Verify with a single-zome wasm compile in a low-session window.
2. **Improve the audit** to recognize Class B (central-dispatcher binding) so the 567 number
   becomes honest and the ratchet meaningful. (Deferred — the script is a hot shared file; do it in
   a quiet window with coordination.)
3. **Design the capability-grant / call-provenance mechanism** for Class D — the genuinely hard
   P0. This is one design that closes reputation, cross-DNA `source_happ`, juror-authorization, and
   cached-computation forgeries together. Belongs in `MYCELIX_IMPROVEMENT_PLAN.md` as the P0 #1
   follow-through.

> **Do not** report "567 unbound validators" as "567 vulnerabilities." The honest statement:
> most are Class B/C (already-bound-centrally or nothing-to-bind); the real work is a bounded set
> of Class-A create-path bindings plus one Class-D capability-grant design.
