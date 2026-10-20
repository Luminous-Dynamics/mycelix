# Polygon Settlement Rail Boundary v1

**Status:** Design boundary / pre-production qualification  
**Scope:** Polygon PoS, Polygon AggLayer, Ethereum settlement interaction  
**Related:** #4444, #4445

## Decision

Mycelix should treat Polygon as a **secondary public settlement and payments rail**, not as the sole economic or security root.

The preferred initial topology remains:

```text
Mycelix / Holochain
    |
    v
Ethereum L1
(public commitment / settlement anchor)
    |
    +--> Arbitrum One (initial general-purpose L2 rail)
    |
    +--> Polygon PoS (secondary payment / settlement rail)
    |
    +--> future Polygon AggLayer-connected chains
```

Polygon may therefore complement Ethereum L1 and another Ethereum L2 without replacing Holochain.

## Why Polygon is attractive

Polygon PoS is EVM-compatible, has substantial existing transaction/payment activity, and is explicitly positioned by Polygon for low-cost payments and high-volume use cases.

For Mycelix this makes Polygon useful for:

- small-value SAP-denominated payments where users need inexpensive settlement;
- merchant and marketplace flows;
- recurring payments and machine-to-machine settlement;
- public settlement receipts for otherwise-private Mycelix events;
- liquidity access to established stablecoin and EVM infrastructure;
- future interoperability through AggLayer.

The economics of a rail are not by themselves a security qualification. Cheap execution must remain separate from settlement assurance.

## Security classification

### Polygon PoS

Current independent infrastructure assessment classifies Polygon PoS as an EVM-compatible proof-of-stake sidechain rather than an Ethereum rollup. Its bridge/state model currently relies on Polygon's validator security assumptions, while Polygon describes a roadmap toward a validity-proof-based validium architecture.

Therefore:

```text
Polygon PoS checkpoint on Ethereum
!= Ethereum-validity proof of Polygon state
!= Ethereum-equivalent security
```

A Mycelix bridge policy must preserve this distinction.

### Polygon AggLayer

AggLayer is more strategically interesting. It is intended to provide cross-chain token/message interoperability across heterogeneous chains and is moving toward chain-agnostic interoperability.

Mycelix should treat AggLayer as an **interop adapter surface**, not as an implicit replacement for its own provenance and authority model.

An AggLayer receipt must still identify:

- source chain;
- destination chain;
- bridge / interoperability version;
- contract identity;
- state or message commitment;
- proof / attestation class;
- finality state;
- replay protection;
- emergency status;
- correction/supersession lineage.

### Polygon zkEVM Mainnet Beta

Do **not** target the retired Polygon zkEVM Mainnet Beta as a production settlement rail.

Polygon states that its Mainnet Beta sequencer was sunset on July 3, 2026. Withdrawal through the AggLayer bridge is no longer available for that retired network; EOA-held assets are handled through a dedicated Ethereum recovery path.

Any legacy zkEVM integration must therefore be classified as historical / migration-only unless a separate current Polygon network is explicitly qualified.

## Architectural boundary

The Mycelix layers retain distinct responsibilities:

```text
Holochain
  identity + provenance + evidence + local authority + economic state

Ethereum L1
  durable public commitments + high-value settlement + canonical custody boundary

Polygon PoS
  low-cost public execution / payments / settlement sidecar

AggLayer
  optional cross-chain interoperability fabric

Symthaea
  anomaly detection + scenario analysis + supervision
  (never monetary issuer, bridge authority, or oracle source)
```

A Polygon transaction is therefore an external settlement event, not a Mycelix truth source.

## Token boundary

Do not expose an internal currency merely by deploying an ERC-20 and calling it SAP.

Any public representation of SAP must be a typed bridge instrument with at minimum:

- source issuance domain;
- source issuance event / journal reference;
- exact bridge version;
- source and destination chain IDs;
- token contract identity;
- custody / lock / burn semantics;
- mint authorization class;
- redemption authorization class;
- finality state;
- total-supply correspondence proof;
- replay domain;
- emergency / pause state;
- correction and supersession lineage.

Invariant:

```text
bridged_supply
  <= provably escrowed_or_burned_source_supply
```

No caller-supplied balance, oracle rate, bridge status, or destination-chain observation may create monetary authority merely because it arrived from Polygon.

## Oracle boundary

Polygon does **not** solve the current Mycelix oracle problem.

Before exposing SAP or collateral value to any public chain, Mycelix must remove fail-open behavior where:

- oracle consensus is unavailable but a caller-supplied rate is accepted;
- oracle vitality is unavailable but a default vitality value is substituted;
- collateral valuation is unavailable but a permissive fallback is used.

External-chain settlement must consume a qualified, versioned observation with freshness, source identity, independence metadata, and an explicit fail-closed state.

## Bridge threat model

The Polygon adapter must be qualified against at least:

1. forged Mycelix DID/address binding;
2. address rotation and revoked bindings;
3. replay across source/destination chain IDs;
4. replay across bridge versions;
5. duplicate settlement claims;
6. amount mutation;
7. token-contract substitution;
8. stale or superseded Mycelix commitments;
9. stale Polygon state;
10. validator / checkpoint equivocation;
11. Polygon bridge contract upgrade;
12. multisig threshold or signer-set change;
13. compromised relayer;
14. sequencer / RPC outage where applicable;
15. destination censorship;
16. one-sided mint/burn/redemption;
17. bridge supply conservation failure;
18. emergency pause abuse;
19. recovery-path ambiguity;
20. stablecoin freeze or depeg;
21. liquidity exhaustion;
22. partial failure between Holochain and Polygon;
23. reorg/finality assumption mismatch;
24. incorrect chain ID / contract address / deployment version;
25. cross-AggLayer message confusion if AggLayer is enabled later.

## Finality model

Never represent "transaction submitted" as "settled".

The adapter should expose explicit states such as:

```text
Observed
Submitted
Included
ProbabilisticallyFinal
BridgeFinal
EconomicallyFinal
Reconciled
Disputed
Superseded
```

The exact state machine must be tied to one chain configuration and one configured finality policy.

## Multi-rail strategy

The strongest architecture is not "choose the one best chain."

It is:

```text
one Mycelix economic event
        |
        +--> zero or more external settlement claims
        |
        +--> each claim has its own trust/finality profile
        |
        +--> one common conservation + provenance journal
```

This lets Mycelix use Polygon for inexpensive payments while retaining an Ethereum-rooted security anchor and an independent second L2.

Rail diversity is only meaningful when trust assumptions are actually distinct. Two RPC providers for the same chain do not constitute independent settlement evidence.

## Recommended deployment sequence

### Phase 0 — design only

- define `SettlementClaim`;
- define chain/contract registry;
- define identity-binding records;
- define finality states;
- define supply-conservation receipts;
- qualify the current oracle path as fail-closed.

### Phase 1 — Ethereum + Arbitrum

Use one production-grade Ethereum settlement path first.

### Phase 2 — Polygon PoS secondary rail

Add Polygon PoS for low-value/high-volume payment settlement after the bridge threat corpus passes.

Risk limits should be lower than for Ethereum-rooted settlement until empirical evidence justifies raising them.

### Phase 3 — Polygon AggLayer adapter

Add AggLayer only after its exact proof, bridge, recovery, upgrade, and finality semantics are version-pinned and independently qualified for the connected chain pair.

### Phase 4 — dedicated Mycelix chain

Only after measurable economic need, mature bridge evidence, real transaction volume, and a demonstrated reason that an application-specific chain is better than composing existing Ethereum settlement rails.

## Qualification receipts

Every successful external settlement should be reproducible from a receipt containing:

- Mycelix event ID;
- settlement claim ID;
- source state digest;
- destination chain ID;
- contract address;
- transaction hash;
- block/reference point;
- observed finality state;
- bridge proof / attestation reference;
- exact adapter version;
- configuration digest;
- signer / relayer identity where relevant;
- timestamp and known-at time;
- reconciliation result.

Missing receipt fields must fail closed for high-value claims.

## Nonclaims

This document does not establish:

- that Polygon PoS has Ethereum-equivalent security;
- that Polygon's bridge is trustless under all external threat models;
- that AggLayer eliminates bridge risk;
- that Polygon is the correct sole settlement chain;
- that a token bridge creates monetary authority;
- that a Polygon transaction is automatically a Mycelix truth event.

The purpose is to keep Polygon useful without allowing external-chain convenience to weaken Mycelix's authority, provenance, privacy, or monetary-conservation boundaries.


## Current Polygon PoS operational evidence

The rail profile should encode operational and consensus assumptions separately from application-layer RPC health.

Polygon's 2026 release history includes multiple security/liveness fixes across Bor and Heimdall. In February 2026, Bor v2.5.9 addressed peer-to-peer, header-validation, Heimdall state-sync, and block-number correctness issues, including bugs with potential consensus/correctness impact. Polygon subsequently published further Bor/Heimdall security and liveness fixes in June and August 2026.

Polygon's September 2026 chaos-engineering report also demonstrates an important property for the adapter: under injected network/CPU faults, healthy validators continued consensus while a degraded validator fell behind; the report explicitly ties safety progress to an honest supermajority of voting power.

The current Polygon validator dashboard shows 105 active validators and about 3.52B POL staked. The five largest displayed validator positions account for roughly 42% of that displayed stake; this number must **not** be interpreted as 42% independent control because displayed validator entities may share operators or economic relationships.

Therefore the Mycelix adapter must not infer settlement assurance from:

```text
RPC availability
  or
block inclusion
  or
validator count
  or
nominal stake totals
```

Instead, the qualified Polygon rail profile should explicitly bind:

- chain ID 137;
- exact Bor/Heimdall protocol-version assumptions relevant to the observation;
- checkpoint/finality evidence class;
- bridge contract/version identity;
- required validator/checkpoint condition;
- node/provider failure-domain assumptions;
- observed block/reference point;
- adapter configuration digest;
- configured exposure ceiling.

### Polygon-specific chaos corpus

Before allowing material external value, exercise at minimum:

1. Bor peer isolation;
2. Heimdall peer isolation;
3. Bor/Heimdall desynchronization;
4. RPC provider stall;
5. stale RPC response;
6. competing RPC observations;
7. validator outage below the configured safety threshold;
8. proposer/checkpoint delay;
9. malformed block/header;
10. invalid or stale checkpoint;
11. bridge contract pause/upgrade;
12. duplicate bridge event;
13. replay across destination chain or bridge version;
14. relayer compromise;
15. interrupted Holochain -> Polygon settlement;
16. interrupted Polygon -> Holochain reconciliation;
17. claim submitted before required finality;
18. finality observed, then configuration superseded;
19. stablecoin freeze/depeg on the external rail;
20. exposure limit exhaustion.

A passing runtime observation must be recorded as a receipt tied to an exact software/configuration/protocol version. A static test result alone does not establish live Polygon settlement safety.
