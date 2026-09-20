# CI-GOV-001K-E — aggregate qualification evidence root v0.1

This tranche defines a pure offline verifier for the **actual A/B/C/D qualification receipts** emitted by the frozen CI-GOV-001K source qualifiers.

It does not execute those qualifiers. It accepts their retained JSON receipts only after they exist.

## Canonical components

```text
A source      bf07e3fa9701e947cb7ac5c03e43052661e8a155
A qualifier   2b0d268ad40ab46357a4d256c727cd3ae432cf76
A qtree       eb7d914c9df2a3217b28fd70d008e454d1c7585d

B source      d3de3a4d24c7459b80f939fee5c3c2bb4843ad71
B qualifier   936633584dc99bc917d341788dbc9b7aabd01722
B qtree       26f071f4922b6a592bf277e19a0e02ea138b8488

C source      8ee69ad386e4189fb4b1d549dc95ea649c282c1d
C qualifier   ba1e01ab3bb51f3b40aa2d8c88399f4d227eedec
C qtree       0e38c46cd331adc81beeb67fa979864b782699ce

D source      55b6de9f8e39ea9c639b890730f4f48b9889267c
D qualifier   498fa8d487b9a7b8b65be4dcff9ab76cb24e2910
D qtree       bbcbd63187c3926c43208aabd73d6f9343268329
```

## Frozen source-lock SHA-256 values

```text
A 55b83c77e91bbab9cdbcf7ee4089bbec0116c1a4cf7f37a3fec40bb6599f23e9
B 7c29e73c525947d91025d80b9b1c37bc062b8f2bb1e236323c39ac5563d40bfc
C 52f8498909aa02ed7f27d123b6854ee0b3597232496f7f77d002df8450f7ab22
D dea22ebd5e38002adba1da3ffbb25dfb0559b9ab8adc55d2d19d6d6bad86ec7d
```

## Verification contract

The verifier requires exactly four receipts, one for every registered schema. It:

1. rejects oversized (>1 MiB), non-UTF-8, malformed, duplicate-key, or non-finite JSON;
2. rejects unknown, duplicate, missing, or extra component schemas;
3. requires exact top-level field sets for each receipt;
4. recomputes every qualifier receipt commitment canonically;
5. requires the exact registered source commit/tree and qualifier commit/tree;
6. requires the exact source-lock SHA-256, proposition, nonclaims and test field set;
7. requires each registered isolated test result to have `returncode = 0` with 64-hex stdout/stderr digests;
8. requires every registered authority/grant field to remain exactly `false`;
9. re-checks B→A, C→B, C→A-policy and D→A lineage explicitly;
10. binds both canonical component commitments and exact receipt-file SHA-256 values into one A/B/C/D-ordered aggregate root.

Input file order does not affect the resulting evidence root; schema determines canonical A/B/C/D order.

## Aggregate authority ceiling

The aggregate receipt always records `false` for:

- workflow activation;
- label mutation;
- Actions mutation;
- live scheduler qualification;
- cancellation authority;
- merge authority;
- product PASS;
- scientific PASS.

## Critical nonclaim

A structurally valid aggregate receipt is **not proof that A/B/C/D really executed** unless the four supplied receipts are the retained outputs of the actual qualifier executions. Synthetic receipts are used only in the unit-test corpus and must never be promoted as canonical qualification evidence.

A later exact-source qualifier should bind this verifier's source identity and committed adversarial corpus. No workflow is added by this tranche.

Refs #2117 #1964 #2002 #2057 #2064 #2075 #2067 #697 #969.
