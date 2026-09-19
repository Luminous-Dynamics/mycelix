# CI-GOV-001K-B exact observer source qualification v0.1

This qualifier proves only the exact source identity and offline test execution of the read-only capacity observer. It performs no GitHub API request itself.

## Exact observer subject

- observer commit: `d3de3a4d24c7459b80f939fee5c3c2bb4843ad71`
- observer tree: `3ef12424861acf5a1ee63d216b71805561bed977`
- parent policy source: `bf07e3fa9701e947cb7ac5c03e43052661e8a155`
- parent tree: `e486d2d74d696b92577c52ab5d4e534ed2c36890`

The observer subject must contain exactly four additive review paths with the blob identities frozen in the qualifier.

## Qualification gates

1. reject Git redirect environment, grafts, and replace refs;
2. disable replacement objects for every Git subprocess;
3. require the qualifier itself to be a clean one-commit direct child with exactly three qualifier paths;
4. verify observer commit/tree/direct parent/parent tree;
5. verify exact observer path set and blob OIDs;
6. verify the exact observer lock contract;
7. reconstruct only the committed observer + test script from exact Git objects;
8. run the exact committed 26-case observer suite under isolated Python (`-E -s -S -B`);
9. re-check qualifier working bytes and cleanliness post-execution;
10. require retained receipts outside the checkout.

## Proposition

A PASS proves only that the exact CI-GOV-001K-B v0.1 observer source identity is bound and its committed 26-case offline corpus executes successfully.

It does not prove a live GitHub API observation, `queue: max` behavior, observer freshness at a later decision time, queue admission, runner availability, cancellation authority, merge authority, product correctness, or any scientific claim.
