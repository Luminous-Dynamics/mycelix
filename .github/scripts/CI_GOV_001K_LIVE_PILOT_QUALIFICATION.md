# CI-GOV-001K-D-A exact inert pilot-fixture qualification v0.1

This qualifier proves only the exact source identity and offline static-verifier execution of the inert CI-GOV-001K-D-A live-pilot fixture.

It cannot activate a GitHub Actions workflow and performs no GitHub network request.

## Exact subject

- source commit: `55b6de9f8e39ea9c639b890730f4f48b9889267c`
- source tree: `d8fc9c0f8f03840a1114d02c24cd3881820b2d6e`
- parent policy: `bf07e3fa9701e947cb7ac5c03e43052661e8a155`
- parent tree: `e486d2d74d696b92577c52ab5d4e534ed2c36890`
- fixture SHA-256: `c1479745b168901f67931ff392b7c3e70ec4959a3841328943380719e9117cfe`

## Gates

1. reject Git redirects, grafts and replacement refs;
2. disable replacement objects on every Git subprocess;
3. require the qualifier to be a clean one-commit direct child with exactly three qualifier paths;
4. verify exact source commit/tree/direct parent/parent tree;
5. verify all five source blob OIDs;
6. independently hash the exact YAML fixture and compare the frozen SHA-256;
7. verify the complete source lock contract;
8. reconstruct only the fixture, static verifier and verifier-test files from canonical source Git objects;
9. execute the exact committed **25-case** corpus under isolated Python (`-E -s -S -B`);
10. re-check qualifier working bytes/cleanliness after execution;
11. retain any receipt only outside the checkout.

## Proposition

A PASS proves only:

> Exact CI-GOV-001K-D-A v0.1 inert pilot-fixture identity and committed 25-case static-verifier corpus passed.

It does not install a workflow, create a pilot run, prove live `queue:max` behavior, grant label/Actions/cancellation/merge authority, or establish any product/scientific claim.
