# Status and claims policy

This document defines how Mycelix Marketplace communicates engineering maturity.

## Evidence ladder

| Level | Permitted wording | Required evidence |
| --- | --- | --- |
| 0 | Proposed | Design or issue only |
| 1 | Implemented | Code exists and is reviewable |
| 2 | Unit verified | Relevant deterministic tests pass |
| 3 | Build verified | Clean documented toolchain builds all claimed artifacts |
| 4 | Integration verified | Real zomes interact in a conductor test |
| 5 | End-to-end verified | Multiple agents complete the user journey from a clean state |
| 6 | Adversarially evaluated | Threat model and negative/attack cases are reproducible |
| 7 | Formally established | Stated property is proven under explicit assumptions |

A higher level must not be implied from a lower one. A unit test of a weighting formula does not establish marketplace-level Byzantine tolerance. A packed DNA does not establish authorization correctness. A UI preview does not establish a live backend flow.

## Current claim boundary

The project may currently claim:

- a substantial Holochain and SvelteKit marketplace prototype;
- implemented listing, transaction, reputation, arbitration, messaging, search, and security-log zomes;
- experimental reputation-weighted trust and arbitration mechanisms;
- an evidence-aware marketplace interface;
- active work toward a reproducible multi-agent alpha.

The project should not currently claim without new evidence:

- production readiness;
- independent academic validation;
- a measured percentage of Byzantine tolerance for the deployed marketplace;
- complete escrow or payment settlement;
- complete frontend/backend integration;
- complete E2E coverage;
- a security audit.

## Historical documents

Root-level files named `*_COMPLETE.md`, `*_SUCCESS.md`, `DEPLOYMENT_*`, session summaries, and milestone reports are historical work notes. They may describe the state or expectations at the time they were written. They are not release attestations and are superseded for current claims by `README.md`, `STATUS.md`, and this document.

## Release evidence bundle

A future alpha release should publish:

- exact source revision;
- Nix flake or equivalent toolchain lock;
- dependency lock files;
- DNA and hApp hashes;
- clean build logs;
- unit and integration test results;
- two-agent happy-path transcript;
- dispute-path transcript;
- negative authorization tests;
- known limitations and excluded claims.
