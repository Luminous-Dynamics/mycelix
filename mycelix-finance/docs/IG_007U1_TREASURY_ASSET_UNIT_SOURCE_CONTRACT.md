# IG-007U1 Treasury Asset/Unit Source-Contract Corpus

Status: **historical evidence only / non-activating**

Parent evidence: IG-007U0 / PR #1225, qualified by slim run `35123955189`.
Tracking: #1241, #1222.

## Purpose

U1 turns U0's prose/profile observations into eight deterministic, content-addressed source-contract receipts bound to the same frozen Treasury/Finance source. It adds no Finance runtime behavior and does not redefine SAP, TEND, MYCEL, Treasury balances, or authorization.

Each receipt preserves five separate things: an identifier, observed source predicates, the bounded property being evaluated, the qualification result, and explicit forbidden conclusions. Positive controls remain beside negative findings so `binding not established` cannot be misread as `no canonical currency exists` or `an exploit occurred`.

## Receipt families

1. `CanonicalFinanceCurrencyEnumExists`
2. `TreasuryDoesNotConsumeCanonicalFinanceCurrencyType`
3. `CanonicalFinanceQuantityTypeNotEstablished`
4. `TreasuryAssetIdentityImmutabilityNotEstablished`
5. `ContributionToTreasuryAssetEqualityNotEstablished`
6. `AllocationToTreasuryAssetEqualityNotEstablished`
7. `DkgThresholdAssetBindingNotEstablished`
8. `DkgSignedSubjectAssetUnitPolicyBindingNotEstablished`

## Semantics

`PropertyEstablishedWithinBoundSource` means only that the receipt's exact required property is established by the frozen source predicates. Some required properties are intentionally negative statements such as “binding is not established.” It does not convert absence of a proof into evidence of a live exploit.

The validator independently re-derives the principal U0 observations from the bound source and checks the corpus schema, receipt identities, common non-claims, U0 profile commitment, and source binding. Running it twice must produce identical output.

## Claim ceiling

A qualified U1 receipt may support successor design and regression tests. It may not claim:

- a live cross-currency transfer;
- stolen funds;
- a deployed exploit;
- deployment currentness;
- overall Treasury safety or unsafety;
- correctness of a future `SapAmount`, Treasury V2, or authorization implementation.

U1 is therefore evidence for the **problem statement**, not evidence that a successor implementation is correct.
