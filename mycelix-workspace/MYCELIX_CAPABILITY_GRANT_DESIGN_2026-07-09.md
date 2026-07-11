# Mycelix authority model — closing the Class-D gaps (capability grants)

**Created 2026-07-09.** Companion to `MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md`. Author-binding
(Class A) fixes "an agent forged an entry claiming to be *someone else*." It does **nothing** for
**Class D**: operations whose security is an *authority claim* — "I am allowed to mint / report
reputation / adjudicate / lead consensus" — where the claim is a free-text label or a self-declared
scalar that `action.author()` can't adjudicate. This doc designs the one mechanism that closes the
whole Class-D set at once.

Triaged Class-D so far: **64** (finance 41, governance 21, identity 2), e.g. unauthenticated SAP
self-mint (`payments:672`), `report_reputation` `source_happ` spoofing (identity/bridge), juror
authorization (justice), leader-election (governance/bridge), treasury allocation authority.

---

## Why the obvious fixes don't work

**Why not author-binding?** The committer legitimately is *not* the subject. A mint's recipient ≠
the minter; a reputation report's subject ≠ the reporter; a judgment's defendant ≠ the juror.
Binding `owner == action.author` would break the feature.

**Why not Holochain's native `CapGrant`/`CapClaim`?** Native capabilities authorize *who may call a
zome function*. They do **not** constrain what a malicious coordinator writes to its own chain, and a
cap grant is **not visible to a remote validating authority** during entry validation. Native caps
answer "who can call"; they cannot answer "does this committed entry validate." (Confirmed: the only
`CapGrant`/`CapClaim` references in the tree are HDK boilerplate in generic `validate()` arms — no
real capability infra exists to extend.)

**Why not `call_info().provenance` / the calling cell's identity?** Cross-DNA
`CallTargetCell::OtherRole` calls **within one hApp share a single agent key**. So provenance is
identical whether the call came from `mycelix-finance`'s real bridge zome or from an attacker calling
`report_reputation` directly. Provenance cannot distinguish them. (This is the exact reasoning already
documented inline on `validate_create_identity_reputation`.)

**The only robust primitive:** make the authority claim reference **on-chain data validatable via
`must_get`, chained to a root of trust the integrity zome can read** — grounding authority in the
DHT, not in a coordinator's assertion. Two Holochain facts make this work:
1. `must_get_valid_record` / `must_get_entry` — deterministically fetch a referenced entry *during
   validation* (already used across the codebase). An entry's validity can depend on another entry.
2. A **root of trust the integrity zome can read** — `dna_info()?.modifiers.properties` (DNA
   properties pinned at hApp-pack time). **VERIFIED available in this stack** (see R1): `hdi` 0.7.1
   re-exports `dna_info()` as a real integrity host function, and `DnaInfo` (=`DnaInfoV2`,
   `holochain_integrity_types` 0.6.1) carries `modifiers: DnaModifiers` with
   `properties: SerializedBytes`. So integrity validators can deserialize a `DnaProps` from
   `dna_info()?.modifiers.properties`.

`must_get_agent_activity` (currently unused) is a third primitive, useful for the reputation grounding
layer (prove a referenced receipt is really on the reporter's chain).

---

## The design: on-chain `AuthorityGrant`, anchored in DNA properties

### Layer 0 — Root of trust (per DNA)
Each cluster DNA pins its root authorities in `properties` at pack time:
```yaml
# in dna.yaml modifiers.properties
root_authorities:
  - <governance_agent_pubkey_b64>     # who may issue grants
governance_did: "did:mycelix:<...>"
```
Integrity reads this once per validation via `dna_info()?.modifiers.properties` (deserialize to a
`DnaProps` struct). This is the anchor: no grant is valid unless it traces to a root here.

### Layer 1 — `AuthorityGrant` entry (new, in `crates/mycelix-bridge-entry-types`)
```rust
pub enum Capability {
    Mint        { currency: String, max_amount: u64 },
    ReportReputation { source_happ: String },   // binds the label to a grant
    Adjudicate  { court: String },
    LeadConsensus { round_scope: String },
    AllocateTreasury { treasury: ActionHash },
    // ... one variant per Class-D operation class
}
pub struct AuthorityGrant {
    pub grantee: AgentPubKey,     // who may exercise it (the committer of the gated entry)
    pub capability: Capability,
    pub issued_at: Timestamp,
    pub expires_at: Option<Timestamp>,
    pub parent: Option<ActionHash>, // for delegation chains (see Layer 1b)
}
```
**`validate_create_authority_grant`:** the grant is valid iff its **author** is either
(a) a `root_authority` from DNA properties, or (b) the grantee of a `parent` grant (fetched via
`must_get`) that itself carries a delegatable super-capability. This is the whole trust root — a small,
auditable validator that everything else leans on.

### Layer 1b — Delegation (optional, bounded)
`parent: Some(hash)` lets a root authority delegate a *narrower* capability. The validator walks the
chain via `must_get` (cap depth, e.g. ≤ 4) and requires each step to be a strict narrowing
(mint max_amount non-increasing, same currency, etc.). Skip in v1 if not needed.

### Layer 2 — Gate the Class-D entries on a grant
Each Class-D entry gains an `authority: ActionHash` field pointing at the grant it exercises. Its
`validate_create_*`:
```rust
let grant_rec = must_get_valid_record(entry.authority)?;
let grant: AuthorityGrant = grant_rec.entry().to_app_option()?.ok_or(...)?;
// 1. the committer must BE the grantee (author-binding RE-ENTERS here, correctly)
if grant.grantee != *action.author() { return Invalid("not the grantee") }
// 2. not expired
if let Some(exp) = grant.expires_at { if entry_timestamp > exp { return Invalid("grant expired") } }
// 3. capability matches this operation AND its scope
match grant.capability {
    Capability::Mint { currency, max_amount }
        if currency == mint.currency && mint.amount <= max_amount => {}
    _ => return Invalid("grant does not authorize this mint"),
}
```
This closes the same-agent-key hole: the authority is now a `must_get`-validatable entry chained to a
DNA-pinned root, and the *committer* is bound to the grantee — so a bare agent calling
`report_reputation` has no grant to point at, and can't forge one (grant creation requires a root key).

### Layer 3 — Reputation needs one more layer (derive, don't trust)
`IdentityReputation`/`MatlTrustScore` are worse than mint: even a correctly-authorized reporter
shouldn't be *trusted* on a raw scalar. Two-part fix:
- **Authority** (Layer 2): only a `ReportReputation{source_happ}` grantee may write, and the entry's
  `source_happ` must equal the grant's — killing label spoofing.
- **Grounding**: the report references (via `must_get` / `must_get_agent_activity`) the underlying
  interaction receipt(s) on the reporter's chain, and the validator recomputes the score bound from
  them. Better still where feasible: **don't store the scalar — compute reputation at read-time** from
  validatable primitives (receipts, PoGQ, consciousness snapshots). "Don't store trust, derive it."

---

## What this closes (mapping to the triage)

| Class-D gap | Grant capability | Extra |
|---|---|---|
| SAP self-mint (`payments:672`), tend balances, pending_minted_adjustment | `Mint{currency,max}` | amount ≤ cap |
| `report_reputation` source_happ spoof (identity/bridge) | `ReportReputation{source_happ}` | + Layer 3 grounding |
| governance `matl_score`, `consensus_round` leader, `slashing_record` | `LeadConsensus`/`ReportReputation` | leader-election grant |
| justice Arbitration/Judgment juror auth | `Adjudicate{court}` | grant issued on jury assignment |
| treasury allocation / commons_pool / currency create | `AllocateTreasury`/`Mint` | manager-set as grantees |
| identity `identity_verification` cached computation | (separate — content integrity) | see R2 |

One entry type + one validator helper (`require_authorized(grant_hash, expected_cap, committer)` in
bridge-common) + a per-DNA properties schema retrofits the entire Class-D set.

---

## Risks / open questions (verify before building)

- **R1 — `dna_info()` in HDI 0.7.1. ✅ RESOLVED (verified against the registry source 2026-07-09).**
  `hdi-0.7.1/src/prelude.rs` re-exports `crate::info::dna_info`, a real host function
  (`info.rs:7 pub fn dna_info() -> ExternResult<DnaInfo>`); `DnaInfo = DnaInfoV2` with
  `modifiers: DnaModifiers { properties: SerializedBytes, .. }`
  (`holochain_integrity_types-0.6.1/src/{info.rs,dna_modifiers.rs}`). The properties-anchor is viable;
  the `const ROOT_AUTHORITIES` fallback is NOT needed. (Caveat: no in-tree integrity zome calls it yet,
  so the R1 spike still proves it end-to-end through a real validation pass, but the API exists.)
- **R2 — cached-computation entries** (`identity_verification`) are a *different* class: the fix is
  content-integrity (re-derive or don't store), not an authority grant. Track separately.
- **R3 — bootstrapping the first grant.** Genesis: root authorities self-grant, validated by the
  DNA-properties check (author ∈ root_authorities). No chicken-and-egg.
- **R4 — grant revocation** = delete the AuthorityGrant; gated entries should `must_get_valid_record`
  (not `must_get_entry`) so a deleted grant fails validation of *new* exercises. Already-committed
  entries stay valid (correct — authority was real at the time).
- **R5 — cross-DNA grants.** A grant issued in governance-DNA but exercised in finance-DNA can't be
  `must_get`'d across DNAs. Options: mirror the grant into each consuming DNA (a bridge-replicated
  entry), or host all Class-D grants in a shared identity/governance cell that others read via
  `must_get` if same-conductor. **This is the deepest open question** — resolve before finance uses
  governance-issued mint authority.

---

## Recommended build sequence (all gated on a quiet window for wasm verify)

1. **Spike R1** — a throwaway integrity zome that calls `dna_info()?.modifiers.properties`, packed &
   validated on a sweettest. Decides properties-anchor vs const-anchor. *Everything downstream depends
   on this one fact.*
2. **`AuthorityGrant` + `Capability`** in `mycelix-bridge-entry-types`; `require_authorized` helper +
   `DnaProps` reader in `mycelix-bridge-common`. Unit-test the validator logic in isolation.
3. **One vertical slice end-to-end**: gate `payments/validate_create_sap_mint_record` (the worst gap)
   on a `Mint` grant. Prove forge-fails / authorized-succeeds in a sweettest. This validates the whole
   pattern before scaling.
4. **Resolve R5** (cross-DNA grant visibility) using the finance-mint slice as the forcing case.
5. Roll the pattern across the remaining Class-D entries, cluster by cluster.

> This is the real content of MASTER_ROADMAP P0 #1 beyond the mechanical binds. Author-binding
> (Class A) is a bounded ~51-item mechanical pass; this authority model is the *design*-level P0 —
> one mechanism, ~1 entry type + 1 helper + a per-DNA properties schema, that closes 64+ gaps.
