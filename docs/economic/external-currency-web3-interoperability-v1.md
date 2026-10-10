# External Currency + Web3 Interoperability V1

## Purpose

This contract extends the Economic Fabric across fiat, bank/deposit money, cryptoassets, stablecoins, mutual-credit instruments, Integral ITC, Holo Fuel-style systems, Valueflows and simulation-origin economic events.

The layer is an interoperability fabric, not a universal currency, wallet, exchange, or financial authority.

## Canonical pipeline

```
ExternalObservation
 -> Evidence
 -> InstrumentProfile
 -> EconomicEvent / Position / Obligation
 -> Policy / Valuation
 -> Authorization
 -> Settlement / Effect
 -> Outcome / Reconciliation
```

No ticker, symbol, chain reachability, balance observation, or provider status may bypass these stages.

## Instrument families

| Family | Example | Mandatory separation |
|---|---|---|
| Fiat unit | USD, EUR, ZAR | unit of account != deposit/cash/instrument |
| Bank/deposit | bank USD | deposit claim != cash |
| Native crypto | BTC, ETH | chain identity + asset identity are bound |
| Token | ERC-20 | contract + chain + issuer/profile matter |
| Stablecoin | issuer/network token | token != fiat unit |
| Wrapped/bridged | wrapped BTC | representation != native asset |
| Mutual credit | TEND, Holo Fuel-style | instrument policy != generic currency |
| Integral | ITC | contribution/access instrument != generic currency |
| Valueflows | EconomicEvent/Commitment/Intent | ontology roles remain distinct |
| Symtropy | simulated currency | simulation fact != physical economic fact |

## ExternalInstrumentProfileV1

A registered profile may bind:

- profile ID and revision;
- namespace/scheme;
- instrument identity;
- display symbol/name;
- issuer;
- network/ledger;
- chain ID;
- contract/address;
- token standard;
- atomic-unit and scale;
- fungibility/equivalence domain;
- economic/legal form;
- redemption/reference mechanism;
- validity interval;
- source evidence;
- provenance;
- claim ceiling.

Display metadata never becomes authoritative identity by itself.

Illustrative identifiers such as `iso4217:USD`, `mycelix:TEND`, or `caip19:eip155:1/erc20:<contract>` are examples only; the exact namespace grammar requires its own profile.

## Web3 airlock

Web3 observation is decomposed into:

1. chain/network identity;
2. block/transaction/log observation;
3. inclusion/confirmation evidence;
4. finality profile;
5. instrument mapping;
6. wallet/address observation;
7. key-control evidence, if separately proven;
8. economic interpretation;
9. authorization;
10. settlement/effect.

The following are never inferred from one another:

```
RPC reachability
!= contract legitimacy
!= wallet control
!= legal ownership
!= settlement finality
```

An ERC-20 transfer is not automatically a market fill. A market fill is not automatically final settlement. A token balance is not automatically legal title.

Reorgs, reversals and chain corrections append lineage; they do not mutate the historical event.

## Stablecoin boundary

A stablecoin profile must keep distinct:

- token identity;
- chain/contract;
- issuer;
- reference asset;
- reserve/redemption evidence;
- current status;
- market price;
- reference/oracle value;
- settlement finality.

Therefore:

```
USDC-like token
!= USD
!= bank deposit
!= cash
```

No universal stability, solvency, reserve or redemption claim is made by the generic adapter.

## Fiat/payment boundary

The existing ISO 20022 airlock and future bank/payment profiles should map external messages into evidence-bearing candidates.

```
message
!= instruction
!= acceptance
!= booking
!= settlement
!= finality
!= legal discharge
```

External provider/ruleset semantics remain profile-specific.

## FX and valuation

Keep these fields separate:

```
AssetIdentity
UnitProfile
Quantity
ValuationObservation
FXRate
Quote
ExecutionPrice
SettlementAmount
```

FX is not unit conversion. An FX rate must be source-bound, versioned and validity-bounded.

```
USD -> EUR
```

requires explicit valuation/exchange policy.

No stale quote may authorize settlement.

## Mutual credit and Holo Fuel-style profiles

Holo Fuel's public model is useful as a reference profile for mutual credit, productive-capacity-linked credit limits, reserves and external exchange. Mycelix must not turn that reference into a universal monetary assumption.

For any mutual-credit profile, preserve:

- issuer/credit authority;
- credit limit;
- obligation direction;
- backing/capacity evidence where the profile requires it;
- issuance/retirement policy;
- instrument identity;
- settlement rules.

```
credit limit
!= asset ownership
!= collateral
!= settlement receipt
```

## Integral and Valueflows

Integral ITC remains source-owned and instrument-specific.

```
COS observation != ITC entitlement
ITC != generic currency
```

Valueflows roles remain distinct:

```
Intent != Commitment != Claim != EconomicEvent
```

The Economic Fabric may bridge them but must not flatten their meanings.

## Wallet and custody boundary

Portable evidence must never contain reusable private keys, seed phrases, OAuth/API secrets or bearer credentials.

A public address is an observation. Key control requires a separate cryptographic proof/profile.

## Cross-adapter conservation vector

Every admitted adapter should preserve:

1. source identity;
2. origin;
3. instrument identity;
4. issuer/network/contract identity where required;
5. quantity;
6. unit/scale;
7. evidence;
8. validity;
9. causality;
10. authorization;
11. settlement identity;
12. correction lineage;
13. claim ceiling.

No implicit unit conversion is allowed. Unresolved validity prevents a clean authoritative projection.

## Profile monotonicity

```
profile_transition subset_of kernel_transition
```

A profile may restrict a generic Economic Fabric transition but cannot widen the kernel's authority or claim ceiling.

## Nonclaims

This contract does not establish legal tender status, asset legitimacy, issuer solvency, reserve adequacy, investment value, regulatory compliance, custody, key ownership, external settlement finality, bridge security, tax treatment, or autonomous trading authority.
