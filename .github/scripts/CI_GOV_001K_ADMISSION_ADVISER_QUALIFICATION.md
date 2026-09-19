# CI-GOV-001K-C exact admission-adviser source qualification v0.1

This qualifier proves only the exact source identity and offline test execution of the read-only CI-GOV-001K-C admission adviser.

It performs no GitHub API request and grants no mutation authority.

## Exact subject

- adviser commit: `8ee69ad386e4189fb4b1d549dc95ea649c282c1d`
- adviser tree: `d44049d174976b2450e090a702f036a090acdb48`
- parent observer: `d3de3a4d24c7459b80f939fee5c3c2bb4843ad71`
- parent observer tree: `3ef12424861acf5a1ee63d216b71805561bed977`
- inherited policy source: `bf07e3fa9701e947cb7ac5c03e43052661e8a155`
- inherited policy tree: `e486d2d74d696b92577c52ab5d4e534ed2c36890`

## Gates

1. reject Git redirect environment, grafts, and replace refs;
2. disable replacement objects for every Git subprocess;
3. require the qualifier itself to be a clean one-commit direct child with exactly three qualifier paths;
4. verify adviser commit/tree/direct parent and inherited policy tree;
5. verify all four adviser-added blob OIDs;
6. independently verify the inherited exact oracle/observer source and lock blob OIDs;
7. verify the complete adviser lock contract;
8. reconstruct only the canonical oracle, observer, adviser, and adviser-test files from exact adviser Git objects;
9. execute the exact committed **28-case** adviser corpus with isolated Python (`-E -s -S -B`);
10. re-check qualifier working bytes and cleanliness after execution;
11. require retained receipts outside the checkout.

## Proposition

A PASS proves only:

> The exact CI-GOV-001K-C v0.1 read-only admission-adviser identity is bound and its committed 28-case corpus passes against the canonical inherited CI-GOV-001K-A v0.2 oracle and hardened CI-GOV-001K-B observer source bytes.

## Nonclaims

This qualification does not perform a live API observation, does not establish live `queue:max` scheduler behavior, does not mutate a PR or workflow, does not grant label/cancellation/merge authority, and establishes no product or scientific claim.
