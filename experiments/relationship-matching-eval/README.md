# Relationship Matching Evaluation Harness

**Status:** exploratory research tool, not product or production matching code.

This isolated, dependency-free Rust crate implements an initial offline slice proposed in the public Mycelix RFC at PR #5127. The current source has not yet been compiled or tested on its exact branch head. It generates seeded synthetic adult-only participants, applies hard eligibility constraints before ranking, compares four policies—feasibility-only pair ordering, one-sided utility, pairwise mutual utility, and an exposure-balanced mutual-utility allocator—and emits a deterministic JSON report.

## Run

Use a Rust 2021 toolchain. From this directory:

- **Tests:** cargo test
- **Example report:** cargo run -- 20261017 200 5

Arguments are seed, even participant count (minimum 4), and top-k recommendations per participant. Defaults are seed 20261017, 200 participants, and top-k 5.

## What the harness is designed to test

- All policies use the same eligibility predicate before ranking or allocation.
- `discovery_opt_in` is modeled separately from `active`. Opt-in authorizes consideration for discovery only; it is not consent to a specific introduction.
- The feasibility-only policy deliberately ignores interest utility after constraints pass; its stable pair-priority function is pseudo-random-looking but not cryptographic randomness.
- The exposure-balanced policy allocates one candidate at a time to each opted-in viewer and uses incoming exposure counts as its primary selection objective; it is a simple benchmark baseline, not a globally optimal matching algorithm.
- Inactive users, mismatched connection goals, incompatible age ranges, mutual blocks, insufficient shared interests, and insufficient declared utility are filtered before ranking.
- Output ordering is deterministic for fixed input and policy.
- Exposure coverage and Gini are measured over the set of participants who have at least one feasible counterpart.
- The report includes a canonical synthetic-corpus FNV-1a 64-bit fingerprint so same-input regressions are visible.

## What it does not prove

- It does not run a stable-marriage, maximum-cardinality, or globally optimal exposure-fair matching algorithm.
- It does not model pair-specific consent. A later product protocol must record an explicit two-sided acceptance/decline/withdraw state before revealing mutual interest or enabling communication.
- A mutual shortlist is not a human acceptance, date, successful relationship, or compatibility verdict.
- Synthetic interest weights are not psychological traits and are not suitable for production profiles.
- The FNV-1a fingerprint is not cryptographic evidence or a provenance/security receipt.
- It does not test real-user fairness, safety classifier quality, privacy controls, Holochain integration, messaging, or UI behavior.

This harness deliberately includes no personal data, network access, model dependency, image ranking, public reputation score, or external crate dependency. Do not connect it to live profile or message data. A later qualification step should replace the non-cryptographic fingerprint with the repository's approved content-digest format and bind all results to exact code/environment evidence.
