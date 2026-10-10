# Mycelix Economic Fabric — Finance Source Mapping v1

**Status:** implementation/conformance baseline  
**Scope:** MYC-ECO-005 / #3346  
**Source revision:** main at 43110125e73b3c2eb51c29d302a98435f5c279de

## Purpose

This document establishes a source-to-Economic-Fabric mapping for the existing Mycelix Finance surface without replacing Finance-owned semantics.

The mapping is deliberately conservative: an adapter may expose a narrower qualified meaning, but must not promote an observation into authorization, settlement, ownership, or legal/economic outcome without explicit evidence and profile qualification.

## Current code inventory

| Existing surface | Current location | Fabric interpretation | Boundary |
|---|---|---|---|
| SAP balance | `mycelix-finance/zomes/payments/coordinator/src/lib.rs` | Instrument position / balance observation | A balance is not, by itself, issuance authority or settlement finality |
| `initialize_sap_balance` | payments coordinator | Position initialization event | Initialization creates the domain record; it does not establish external asset provenance |
| `get_sap_balance` / `SapBalanceResponse` | payments coordinator / finance wire types | Position observation | Effective balance includes domain-specific demurrage semantics; preserve raw vs effective values |
| `credit_sap` | payments coordinator | Financial effect / position mutation candidate | Must remain distinct from authorized issuance and transfer semantics |
| Currency mint | `mycelix-finance/zomes/currency-mint/coordinator/src/` | Instrument issuance lifecycle | Issuance must remain bound to the instrument profile and issuer policy |
| Balance/demurrage | `balances.rs`, `demurrage.rs` | Position projection + lifecycle adjustment | Demurrage is not an FX conversion or external settlement event |
| Exchanges | `exchanges.rs` | Offer/quote/execution candidate | Quote, execution price and settlement amount remain distinct |
| Payments | `mycelix-finance/zomes/payments/` | Payment intent/execution/effect pipeline | `send_payment`-style APIs must not collapse instruction, execution, settlement and finality |
| TEND | `mycelix-finance/zomes/tend/` | MutualCreditProfile + obligations/positions | Preserve issuer, limits, obligation direction and settlement semantics |
| Bridge | `mycelix-finance/zomes/bridge/` | External recognition / bridge observation | External evidence cannot silently become native Mycelix issuance or ownership |
| Price oracle | `mycelix-finance/zomes/price-oracle/` | Valuation observation | Oracle rate is not unit conversion, execution price or settlement finality |
| Lending / credit scoring | `mycelix-finance/zomes/lending/`, `credit_scoring/` | Credit exposure / policy input | Credit capacity is not asset ownership or transferable entitlement |
| Treasury | `mycelix-finance/zomes/treasury/` | Finance-owned scarce state / settlement projection | Treasury state is not a universal economic outcome |
| Recognition | `mycelix-finance/zomes/recognition/` | Recognition/attestation input | Recognition must retain source, provenance and qualification |
| Simulation types | `mycelix-finance/types/src/simulation.rs` | Simulation instrument profile | Simulation currency must not be promoted to physical-world currency |

## Canonical separation

The adapter layer MUST preserve these distinctions:

```text
instrument identity
    ≠ position/balance
    ≠ accounting projection
    ≠ entitlement
    ≠ authorization
    ≠ execution
    ≠ settlement
    ≠ finality
    ≠ economic outcome
```

For the principal Mycelix instruments:

```text
MYCEL reputation/trust
    ≠ SAP instrument
    ≠ TEND mutual-credit instrument
    ≠ external fiat/Web3 instrument
```

A reputation or trust observation may be supplied as policy evidence where explicitly authorized; it must not mint or transfer a monetary instrument merely by being present.

## Payment lifecycle mapping

Existing payment-facing APIs should map through an explicit lifecycle:

```text
request
  -> authorization binding
  -> instrument/quantity validation
  -> execution attempt
  -> rail observation
  -> confirmation/finality qualification
  -> reconciliation receipt
  -> economic outcome projection
```

A legacy one-shot API may remain as a compatibility facade, but its internal/result model should retain each stage and expose the strongest qualified state rather than asserting every downstream state.

## SAP-specific conservation

SAP handling must retain:

- instrument identity/profile version;
- member/subject identity;
- raw balance;
- effective balance after demurrage;
- issuance source and authorization when newly created;
- quantity scale;
- provenance;
- transfer/bridge origin where applicable;
- correction lineage;
- settlement rail and finality profile where a settlement occurs.

The existing code contains a documented hole around raw `credit_sap`: it can serve legitimate inbound credit use cases but is also a broad mutation surface. The conformance target is therefore to make authorized issuance and conservation-preserving transfer explicit, rather than assuming caller identity alone can solve both cases.

## TEND-specific conservation

TEND should map to a mutual-credit profile rather than a generic currency abstraction. The adapter must retain:

- issuer/steward identity;
- participant/obligor identities;
- credit limits;
- obligation direction;
- issuance/retirement semantics;
- settlement rail;
- expiry/validity where applicable;
- evidence and correction lineage.

## Finance/accounting boundary

Accounting projections are evidence-bearing projections of Finance state. They do not rewrite source events and do not become physical/economic evidence merely because an accounting entry exists.

Corrections append lineage:

```text
source event -> observation -> correction/reversal -> reconciled projection
```

rather than mutating historical source meaning.

## Conformance target

The first executable conformance tranche should assert:

1. profile identity survives translation;
2. quantity/unit scale survives translation;
3. issuer/network/contract identity is retained when applicable;
4. recognition cannot mint an unrelated local instrument;
5. reputation cannot become money without an explicit instrument/profile;
6. credit limits cannot become asset ownership;
7. entitlement cannot become authorization;
8. authorization cannot imply settlement;
9. provider acknowledgement cannot imply finality without a matching finality profile;
10. correction/reversal preserves original observation lineage;
11. simulation instruments cannot become physical-world instruments by adapter default;
12. FX/valuation observations remain distinct from quantity conversion.

## Nonclaims

This mapping establishes software interoperability and semantic conformance only. It does not establish legal ownership, legal tender status, issuer solvency, reserve adequacy, tax treatment, regulatory compliance, financial performance, custody, or external settlement finality.
