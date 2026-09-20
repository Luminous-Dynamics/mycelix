# CI-GOV-001K-E exact evidence-chain source qualification v0.1

This qualifier proves only the exact source identity and offline adversarial execution of the CI-GOV-001K-E aggregate receipt-chain verifier.

## Exact subject

- evidence-chain source: `1b0d00342faea04fa09bae05931af7ced21f69dd`
- source tree: `36d962547c61056600dba1e2e3f0a76b8d5a2a41`
- parent policy source: `bf07e3fa9701e947cb7ac5c03e43052661e8a155`
- parent tree: `e486d2d74d696b92577c52ab5d4e534ed2c36890`

## Gates

1. reject Git redirect environment, grafts and replace refs;
2. disable Git replacement objects for every Git read;
3. require a clean one-commit/three-file qualifier checkout directly above the E source;
4. verify exact E source commit/tree/parent/path set/blob OIDs;
5. verify the complete E source lock contract;
6. statically reject network or GitHub mutation clients in the E verifier;
7. reconstruct only the committed verifier + test script from exact Git objects;
8. run the committed **34-case** E verifier corpus under isolated Python (`-E -s -S -B`);
9. re-check qualifier bytes and clean checkout post-execution;
10. require retained receipts outside the repository checkout.

## Proposition

A PASS proves only that the exact CI-GOV-001K-E v0.1 source identity is bound and its committed 34-case fail-closed aggregate-evidence verifier corpus executes successfully.

It does **not** execute or qualify A/B/C/D themselves, does not turn synthetic unit-test receipts into canonical evidence, and grants no live scheduler, workflow, label, Actions, cancellation, merge, product or scientific authority.
