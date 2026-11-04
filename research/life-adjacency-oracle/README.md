# LIFE-ADJACENCY-001 — Exact privacy-unit adjacency oracle

## Question

What happens to a differential-privacy bound when the protected adjacency unit changes from one record to one person, a group, or a relational graph? And what if the mechanism randomizes record values but publishes the very relationship edge that defines the graph adjacency in clear?

This is an exact finite-distribution reference model for binary randomized response. It is not a production DP library or an audit of any live Mycelix protocol.

## Exact mechanism

For each binary fact, randomized response outputs the input bit with probability 3/4 and the opposite bit with probability 1/4. One fact therefore has exact likelihood ratio 3 and pure-DP parameter epsilon = ln(3), delta = 0.

For two adjacent datasets that differ in m binary facts, if each differing fact is independently randomized, the ratio for every possible output vector is exactly 3 raised to the number of output bits agreeing with dataset A minus the number agreeing with dataset B. Exhaustive enumeration of all 2^m output vectors verifies the maximum exponent m and minimum exponent -m. Therefore this release has epsilon = m * ln(3), delta = 0 for the declared m-fact adjacency.

That is a bound for this concrete toy mechanism only, not a universal claim for different mechanisms.

## Adjacency profiles

- Record adjacency: one binary record changes.
- Person adjacency: all configured records belonging to one person change together; report 1 and 3 records/person.
- Group adjacency: all records for 1, 2 or 4 people change together.
- Relational-graph adjacency: group records change and one relation edge per group member also changes.

Each row reports the changed-record count and changed-edge count separately. The exact likelihood-ratio enumerator is limited to 16 changed binary facts (65,536 output states), frozen in the manifest.

## Deliberately unsafe comparison

`randomized_records_publish_relation_edges_raw` randomizes the record bits but publishes relation edges deterministically. If a relational-graph adjacency changes a published edge, there is an output event with probability 1 on one dataset and 0 on the adjacent dataset. No finite epsilon can satisfy pure DP for that release. For approximate DP, the event forces delta >= 1; the CSV reports a 1,000,000 ppm lower bound.

This does not mean all public metadata always violates DP. It means the edge must be included in the protected dataset and output model if a graph-level guarantee is intended. If the adjacency definition excludes edges, the toy guarantee does not protect those edges.

## Paradigm shift

Do not compare privacy parameters without comparing:
1. the neighboring-dataset definition;
2. which record/relationship facts change together;
3. which facts the mechanism actually randomizes versus publishes in clear;
4. the composition and accounting model;
5. the real mechanism and implementation revision.

The empirical relational attacker benchmark (#5475 / #5482) complements this oracle, but attack accuracy and formal DP are different evidence classes.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-adjacency-oracle/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-adjacency-oracle/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-adjacency-oracle/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-adjacency-oracle/Cargo.toml
~~~

The exact-head workflow checks out and asserts the PR-head SHA, runs format/tests/strict Clippy, executes the report twice and compares bytes, then uploads the report, manifest, digests and qualification receipt.

## Proposed acceptance criteria (not achieved results)

- Exhaustive likelihood-ratio exponent equals the changed-fact count for every randomized-response scenario.
- Record/person/group/relational-graph profiles differ only according to the frozen adjacency definitions.
- Clear changed relation edges mark the full mechanism as having unbounded epsilon and delta lower bound of 1.
- Randomized relation edges compose into the declared record+edge epsilon.
- Every output row distinguishes overall privacy bound from the conditional randomized-record channel.
- Fixed code + manifest produces a byte-identical report.

## Nonclaims

This oracle does not prove the correctness of a production DP implementation, a calibrated privacy parameter for Mycelix data, group privacy beyond these declared adjacency definitions, anonymity, legal compliance, or production protocol safety. It is a mathematical reference model for binary randomized response and an executable counterexample to treating a record-level epsilon as automatically person/group/graph-level epsilon.

