# External Instrument Profile V1

## Design

An external economic instrument is admitted only through an explicit, versioned profile.

```
ExternalInstrumentProfile
  -> identity
  -> issuer/network binding
  -> unit/quantity semantics
  -> evidence + validity
  -> economic form
  -> adapter policy
  -> claim ceiling
```

A display ticker is metadata, not authoritative identity.

## Profile classes

### FiatUnit

Uses an explicit currency namespace such as ISO 4217 where applicable. ISO 4217 defines three-letter and three-digit currency codes and, for currencies with minor units, their decimal relationship. citeturn0search2turn0search3

A FiatUnit identifies a currency/unit, not a bank account, cash note, deposit claim, or tokenized representation.

### BankDeposit

Binds the deposit instrument to the relevant institution/account namespace and validity evidence. Currency code alone is insufficient.

### NativeChainAsset

Binds the asset to an exact chain/network identity and native-asset profile. For EVM systems, chain identity must remain explicit; EIP-155 uses chain IDs as part of transaction signing/replay protection. citeturn0search1

### TokenAsset

Binds chain + token contract/mint + token standard + unit scale. ERC-20 defines an interface for fungible tokens and exposes optional name/symbol/decimals metadata; therefore symbol and even decimals must not be treated as sufficient identity. citeturn0search0

Chain-aware identifiers such as CAIP-19 are useful interoperability references because they encode a blockchain identifier plus an asset namespace/reference. citeturn0search7turn0search4

### Stablecoin

A token profile plus issuer/reference/redemption/status evidence. Stablecoin status does not imply that the token is literally the underlying fiat unit.

### WrappedOrBridgedAsset

Binds representation identity, bridge/issuer and underlying-reference claim separately. The representation cannot silently become the underlying asset.

### MutualCredit

Binds issuer/steward, credit limits, obligation direction, issuance/retirement policy and any required backing/capacity evidence.

### IntegralITC

Preserves Integral-specific contribution/access semantics. It is not flattened into a generic currency.

### ValueflowsRole

Preserves the role distinction between Intent, Commitment, Claim, EconomicEvent and EconomicResource.

### SymtropySimulationAsset

Binds world, episode, simulation build and deterministic replay context. A simulation asset can be recognized by another system without becoming a physical-world asset.

## Mandatory identity vector

When applicable:

- profile ID + revision;
- namespace/scheme;
- canonical instrument identity;
- issuer/steward;
- network/ledger;
- chain ID;
- contract/mint/address;
- token standard;
- atomic unit + decimal scale;
- fungibility/equivalence domain;
- validity;
- evidence;
- provenance;
- claim ceiling.

## Adapter algebra

A source adapter is valid only if:

```
TargetMeaning ⊆ SourceClaimCeiling + ExplicitAdapterPolicy
```

and never:

```
TargetMeaning ⊃ SourceClaimCeiling
```

The adapter must preserve the conservation vector from the external-currency contract.

## Identity rules

1. same ticker may identify many instruments;
2. same contract address on different chains is not the same instrument;
3. decimal metadata cannot silently alter an authoritative quantity;
4. chain/network is part of identity where the source semantics require it;
5. issuer identity cannot be discarded where issuance matters;
6. recognition creates a new target identity rather than rewriting source identity;
7. corrections append lineage.

## Claim ceiling

A valid profile says what its evidence can support. It does not certify solvency, legality, ownership, custody, price, finality, regulatory compliance or investment value.
