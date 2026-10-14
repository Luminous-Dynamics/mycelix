# Web3 and Ethereum L2 Integration Boundary v1

Status: architecture contract; no blockchain is the canonical source of all Mycelix economic state.

## Thesis

Mycelix should add Web3 as an interoperability, settlement, liquidity and public-verifiability layer.

It should NOT replace Holochain with a blockchain.

Target architecture:

~~~text
Holochain / Mycelix
identity + provenance + evidence + local governance
                    |
                    | typed bridge
                    v
Ethereum L1
public settlement anchor + high-value commitments + reserve custody
                    |
                    v
Ethereum L2
high-volume settlement + EVM interoperability + consumer liquidity
~~~

The L2 is a settlement sidecar, not the semantic root of Mycelix.

## What Web3 adds

- permissionless public settlement;
- globally inspectable asset state;
- programmable custody;
- wallet and DeFi composability;
- external liquidity;
- cryptographic settlement receipts;
- public commitments that survive one Mycelix deployment.

Ethereum's current L2 model is specifically intended to move execution away from L1 while anchoring transaction data and/or validity to Ethereum. Ethereum's current documentation distinguishes optimistic rollups, ZK rollups, sidechains and validiums by their security and data-availability assumptions.

## What stays in Holochain

Mycelix / Holochain should own:

- DIDs and local identity relationships;
- credentials and revocation;
- governance;
- economic events;
- stewardship and contribution claims;
- evidence and provenance;
- local currency policy;
- mutual credit;
- source/model airlocks;
- business and social relationships;
- semantic state.

Ethereum L1 should own:

- high-value public settlement commitments;
- reserve custody when a reserve is actually held onchain;
- canonical public token contracts;
- bridge escrow and exit contracts;
- emergency settlement anchors;
- hashes/commitments for critical Mycelix state.

Ethereum L2 should own:

- routine token transfers;
- high-volume settlement;
- marketplace payments;
- liquidity routing;
- low-cost Web3 transactions;
- EVM-native composability.

## Do not turn SAP into a generic token

An Ethereum representation of SAP should be a typed bridged instrument, not an unlabeled clone.

It should preserve:

- originating currency definition;
- bridge version;
- source issuance domain;
- contract and chain identity;
- supply correspondence;
- custody model;
- redemption path;
- finality model;
- emergency behavior;
- correction lineage.

The bridge must explicitly enforce the correspondence between source SAP and its Web3 representation. A token that merely has a one-SAP name does not prove one-for-one backing.

## Bridge rule

Preferred lifecycle:

~~~text
Mycelix issuance state
 -> exact settlement evidence
 -> authenticated bridge message
 -> L2 representation
 -> transfer
 -> burn/lock
 -> verified return message
 -> Mycelix redemption
~~~

Never use a privileged mint key as the implicit source of truth.

If a multisig, security council or upgrade authority exists, it is part of the bridge trust model and must be recorded explicitly.

## Identity

Users should be able to bind:

~~~text
Mycelix DID <-> Ethereum address <-> L2 address
~~~

through an explicit signed identity-binding credential.

An Ethereum address is not itself a Mycelix identity.

## Privacy

Most Mycelix personal and economic evidence should stay off public chains.

Prefer:

~~~text
private / peer-local evidence
        -> commitment or selective-disclosure proof
        -> public settlement claim
~~~

This makes Web3 the public verification and settlement layer without making every social or financial relationship permanently public.

## Oracle boundary

The current Finance bridge contains collateral-to-SAP paths and price-oracle dependencies.

A future Web3 bridge should require:

~~~text
source observations
 -> normalized observations
 -> aggregation method
 -> valuation snapshot
 -> validity interval
 -> freshness evidence
 -> bridge decision
~~~

A caller-provided price must never become authoritative just because an oracle is unavailable.

## L2 recommendation

Yes: use an Ethereum L2.

Do not build a dedicated L2 first.

The correct initial role is:

~~~text
Holochain = semantic / provenance root
Ethereum L1 = high-value settlement anchor
Ethereum L2 = scalable public settlement rail
~~~

## Which L2?

As of October 7, 2026, the current public L2 landscape is mature enough to use an existing rollup, but the leading general-purpose rollups remain at different trust and governance stages.

Arbitrum One is currently listed by L2BEAT as Stage 1 and an optimistic rollup. Its ordinary upgrade path has a delay, with a Security Council exception. This is attractive for sovereignty-oriented infrastructure because upgrade authority is itself an explicit security assumption.

Arbitrum also has a first-class Rust/WASM smart-contract path through Stylus. However, Arbitrum's current documentation says new Stylus activations on Arbitrum One and Nova are temporarily paused as a security measure. Therefore Mycelix should not make immediate production deployment depend on Stylus.

That suggests an initial deployment strategy of ordinary EVM contracts on Arbitrum One, with Rust retained for offchain tooling, verification, SDKs and shared business logic. Re-evaluate Stylus when activations reopen and the then-current security state is requalified.

OP Mainnet and Base are also Stage 1 optimistic rollups according to L2BEAT and publish their required data to Ethereum. Base currently has substantial ecosystem liquidity but its assessed upgrade path includes privileged upgrade authority with no ordinary delay, so it should not be treated as an independent Mycelix security root.

ZKsync Era is currently listed as a Stage 0 ZK rollup. Its validity-proof model is interesting for future proof-heavy Mycelix functions, but its current maturity should prevent us from treating it as a singular trust anchor.

This is not a claim that Arbitrum is universally safest. It is a statement about the current documented trust-model fit for an initial settlement adapter.

## Multi-rail strategy

Do not permanently bind Mycelix to one L2.

Phase 1:

~~~text
Holochain -> Ethereum -> Arbitrum One EVM
~~~

Phase 2:

Add another independent Ethereum settlement rail.

Independence must be evaluated by actual trust assumptions, not merely different RPC providers or different chain names.

Phase 3:

Only after substantial transaction volume and security evidence, consider a dedicated Mycelix rollup.

## Dedicated Mycelix L2

A dedicated L2 becomes justified when the ecosystem needs:

- very high settlement volume;
- predictable transaction pricing;
- custom execution;
- sovereign gas economics;
- specialized privacy;
- application-specific sequencing;
- deeper protocol integration.

Even then, the chain should be rooted in Ethereum rather than positioned as a replacement for Ethereum settlement.

## Smart-contract surface

Keep the onchain contracts deliberately small.

Initial contract roles:

- BridgeRegistry: chain, contract, version and verifier bindings;
- SettlementInbox: authenticated Mycelix-to-EVM settlement messages;
- SettlementOutbox: EVM-to-Mycelix settlement claims;
- TokenWrapper: optional public representation of a Mycelix asset;
- ReserveVault: only for explicitly governed reserve assets;
- IdentityBinding: DID/address relationships;
- StateCommitment: durable commitments to critical Mycelix state.

## DeFi

DeFi should be treated as an external composability environment.

Mycelix can expose selected assets to DEXs, lending markets, liquidity pools and payment rails, but an external market price must not silently become canonical Mycelix valuation.

Token ownership also must not automatically confer Mycelix citizenship, reputation, treasury authority or monetary-policy authority.

~~~text
token ownership
!= identity
!= reputation
!= citizenship
!= governance authority
~~~

## Risk graph

Each Web3 integration should carry explicit risk dimensions:

~~~text
smart-contract risk
+ bridge risk
+ oracle risk
+ sequencer risk
+ upgrade-authority risk
+ liquidity risk
+ stablecoin issuer risk
+ custody risk
+ governance risk
+ legal/regulatory risk
~~~

Symthaea should model these risks, simulate scenarios and surface contradictions, but should not acquire autonomous authority to seize or freeze assets.

## Web3 qualification corpus

At minimum test:

1. forged DID/address binding;
2. replayed bridge message;
3. duplicated message with changed amount;
4. wrong chain ID;
5. wrong contract identity;
6. wrong bridge version;
7. stale source commitment;
8. conflicting source commitments;
9. revoked identity binding;
10. revoked bridge authorization;
11. malicious contract upgrade;
12. compromised relayer;
13. L2 censorship;
14. sequencer outage;
15. invalid state proof;
16. valid proof for the wrong state;
17. oracle outage;
18. oracle disagreement;
19. stale valuation snapshot;
20. collateral double-use;
21. source burn without L2 release;
22. L2 burn without source redemption;
23. bridge supply-conservation failure;
24. emergency-pause abuse;
25. wallet rotation;
26. stablecoin issuer freeze/depeg;
27. liquidity exhaustion;
28. recovery after partial failure.

## Final rule

> Holochain provides identity, provenance, evidence and local coordination; Ethereum provides public settlement and durable anchoring; L2s provide scalable execution and liquidity; Symthaea provides analysis and supervision.

No layer should silently redefine another layer's semantics.

## Current references

- Ethereum Foundation, L1/L2 relationship, March 23, 2026: https://blog.ethereum.org/2026/03/23/l1-l2-ethereum
- Ethereum.org, L2 network maturity, updated October 6, 2026: https://ethereum.org/layer-2/networks/
- Ethereum.org, data availability: https://ethereum.org/developers/docs/data-availability/
- Ethereum.org, ZK rollups: https://ethereum.org/developers/docs/scaling/zk-rollups/
- Ethereum.org, optimistic rollups: https://ethereum.org/developers/docs/scaling/optimistic-rollups/
- L2BEAT, Arbitrum One: https://l2beat.com/layer2s/projects/arbitrum
- L2BEAT, Base: https://l2beat.com/layer2s/projects/base
- L2BEAT, OP Mainnet: https://l2beat.com/layer2s/projects/op-mainnet
- L2BEAT, ZKsync Era: https://l2beat.com/layer2s/projects/zksync-era
- Arbitrum Docs, Stylus: https://docs.arbitrum.io/stylus/gentle-introduction
- Arbitrum Docs, Stylus activation pause: https://docs.arbitrum.io/notices/stylus-activation-pause-notice