# CI-GOV-001K source qualification v0.2

This qualifier proves only the exact source capsule for CI-GOV-001K-A. It does not exercise GitHub Actions concurrency or establish a capacity observer.

## Exact subject

- subject commit: `bf07e3fa9701e947cb7ac5c03e43052661e8a155`
- subject tree: `e486d2d74d696b92577c52ab5d4e534ed2c36890`
- qualified base: `884a14e14758a91d3c1d370d49648946dc8b89ef`
- qualified base tree: `1a3190fef7ce9fc4ea9fba05aa20f3c277ec030f`

The subject must contain exactly four additive review paths with the frozen blob OIDs recorded by the qualifier.

## Qualification gates

1. reject Git redirect environment, grafts, and replace refs;
2. disable replacement objects for every Git subprocess;
3. require this qualifier to be a clean one-commit direct child of the subject with exactly three qualifier paths;
4. verify subject commit/tree/direct parent/base tree;
5. verify exact source path set and exact blob OIDs;
6. verify `CI_GOV_001K.lock.json` equals the frozen v0.2 lock contract;
7. reconstruct only the committed oracle and oracle-test script from exact subject Git objects;
8. run the exact committed 28-case oracle suite with isolated Python startup (`-E -s -S -B`);
9. re-check qualifier checkout cleanliness/bytes after execution;
10. require any retained receipt path to be outside the checkout.

## Receipt proposition

A PASS proves only that the exact CI-GOV-001K v0.2 source capsule identity is bound and its committed 28-case explicit-token/pending-budget oracle suite executes successfully.

It does **not** prove live GitHub `queue: max` behavior, capacity-observer completeness/freshness, runner availability, cancellation authority, fairness/FIFO, product correctness, or any scientific claim.
