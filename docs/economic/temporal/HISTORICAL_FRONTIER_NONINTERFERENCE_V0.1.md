# Historical Frontier Noninterference — V0.1

**Status:** research/verification draft. The TLA+ model is authored but has not yet been executed by TLC. No model-checking result is claimed.

## 1. The property

A temporal query certificate is a statement about one verifier's accepted causal frontier, not about everything visible in a database today.

Let two executions have identical authenticated per-owner prefixes at a fixed frontier F0 and identical query/policy/profile inputs. They may differ in any suffix beyond F0. A historical replay at F0 must be identical:

\[
\operatorname{Prefix}(E_1,F_0)=\operatorname{Prefix}(E_2,F_0)
\Rightarrow
\operatorname{Replay}(E_1,F_0,Q,V,P)=\operatorname{Replay}(E_2,F_0,Q,V,P)
\]

This is **historical-frontier noninterference** (the no-hindsight property). It is a hyperproperty because it compares two executions, not just one execution against a local invariant.

The key adversarial case is a correction whose business-valid time is before the query instant V, but whose receipt is accepted only after F0. It can change a query at a later frontier F1. It must not change what F0 justified.

## 2. Why pointwise proof checks are insufficient

An interval proof can be complete for its root while a caller accidentally supplies a newer root, a latest-state projection, or a correction head selected using evidence beyond the requested frontier. A Merkle proof can still verify. The failure is semantic: the verifier answered a historical question using future knowledge.

The current forest work correctly separates:
- completeness relative to declared owner-index roots;
- receipt-inventory/log-root reconstruction;
- semantic mapping from assertion payload bytes to validity intervals;
- authentication of checkpoint/key/witness roots.

This model adds a separate test: even if a later suffix is visible in one replica but delayed in another, does the same F0 replay remain the same?

## 3. Bounded TLA+ model

`TemporalFrontierNoninterference.tla` models:
- two owner histories with independent sequence cuts;
- two replicas that may observe later suffix receipts at different times;
- one base receipt per owner at sequence 1;
- a backdated correction per owner at sequence 2;
- fixed historical frontier F0 = (A:1, B:1);
- independently advancing current cuts;
- historical replay outputs and certificates bound to F0.

The correction abstracts a later accepted VALID_TO assertion that ends the matching interval at or before V. It changes the current result only after the relevant owner-local cut advances to sequence 2.

The model intentionally has no global scalar sequence. Owner A and owner B can advance independently, yielding incomparable current cuts.

### Correct specification

Run the base model using the TLA+ tools:

```sh
java -cp tla2tools.jar tlc2.TLC -config TemporalFrontierNoninterference.cfg TemporalFrontierNoninterference.tla
```

The configured invariants are:
- `TypeOK`;
- equality of accepted prefixes at F0;
- historical replay is evaluated at F0, not at the current cut;
- paired historical replay equality;
- no-hindsight equivalence;
- exact certificate binding to F0, policy epoch, query instant, accepted prefix and result.

### Negative control

`TemporalFrontierNoninterference-Mutant.cfg` enables two deliberately incorrect actions:
1. replay a historical query using the latest/current frontier;
2. relabel an F0 certificate with a later current frontier.

The mutant model is expected to produce a counterexample after a correction is observed and accepted. A failing mutant is a successful negative control; it must not be counted as a passing production invariant or CI qualification.

## 4. Required implementation semantics

A production query certificate should bind at least:
- verifier identity and exact accepted frontier digest;
- canonical owner set and stable owner-history identities;
- each owner-local cut, log root/tree size, gaps and consistency lineage;
- exact query scope and valid-time instant/interval;
- temporal schema and interval profile;
- semantic mapper and evaluator build digests;
- policy identity and immutable epoch;
- canonical result/provenance commitment;
- claim ceiling and every unevaluated trust boundary.

Historical replay must resolve correction/supersession heads against the accepted frontier, not against a latest global view. A later correction produces a linked current reassessment with new evidence and a new certificate. It does not mutate the earlier certificate.

This property does not permit a missing owner to be ignored: both executions must share the exact required owner set, and incomplete coverage still yields Indeterminate.

## 5. Adversarial and metamorphic benchmark

Generate paired traces with the same F0 prefixes and vary only evidence outside F0:

| Mutation | F0 replay | Later/current replay |
|---|---|---|
| Delayed receipt visible in one replica only | identical | may differ only after accepted frontier advances |
| Backdated revocation accepted at F1 | unchanged | must reflect applicable revocation |
| Corrections delivered in opposite interleavings across owners | identical for same owner cuts/dependencies | same result for equivalent merged frontier |
| A advances while B remains behind | unchanged | preserve a partial-order cut |
| Same sequence, conflicting owner-history roots | Indeterminate | Indeterminate until conflict is resolved |
| Policy epoch or mapper build changes | historical certificate remains bound to old identity | new evaluation must name new identity |
| Missing owner or unresolved gap | Indeterminate, never Complete | Indeterminate until repaired |
| Irrelevant correction | unchanged | no false invalidation of unrelated claims |

Hard correctness targets (proposed, not achieved):
- zero historical result drift under suffix-only mutations;
- zero acceptance of stale or relabelled certificates;
- zero hindsight leaks from post-frontier backdated corrections;
- zero result dependence on topological delivery order where causal edges are preserved;
- 100% exact replay parity against an independent oracle;
- 100% detection of applicable corrections at a later frontier;
- zero false invalidation from irrelevant corrections;
- zero Complete outcomes with missing required owners, gaps or unresolved forks.

For each run, publish the raw trace and corpus digest, exact source SHA, toolchain, hardware, model state-space size, and all failed cases. For performance, collect at least 30 repetitions after warm-up and report p50/p95/p99 for historical replay, current reassessment, full recomputation and incremental correction processing. Also report certificate bytes and peak memory. Keep semantic correctness, proof verification, index construction and query latency as separate measurements.

## 6. Fair industry and research baselines

- **VeriTS** (Yao et al., 28 Aug 2026): authenticated aggregate interval trees, minimum-covering-set completeness and sliding-window delta verification are mandatory interval-proof baselines. This model does not claim to outperform VeriTS's interval data structure; it tests a different property—knowledge-frontier noninterference.
  - https://arxiv.org/abs/2608.28318
- **XTDB:** compare valid-time/system-time reconstruction and retrospective corrections under the same frozen corpus. XTDB documents separate system-time and valid-time axes, including backdated valid-time changes; the additional Mycelix variable is the verifier's accepted owner frontier.
  - https://docs.xtdb.com/about/time-in-xtdb.html
  - https://docs.xtdb.com/concepts/key-concepts.html
- **SQL Server temporal tables:** compare system-versioned AS OF and range history behavior. Its temporal query semantics are a mature baseline, not a distributed verifier-frontier proof.
  - https://learn.microsoft.com/en-us/sql/relational-databases/tables/querying-data-in-a-system-versioned-temporal-table
- **Event sourcing/workflow replay:** compare deterministic reconstruction and correction processing over the same frozen event streams. Do not compare an unverified latest-state scan against a proof-carrying path as though their guarantees were equivalent.

### VTRQ and temporal/spatial range-query baselines

A second 2026 comparator is **VTRQ** (Yao et al., 21 August 2026). It uses separate authenticated spatial and temporal structures, including a temporal interval tree, and a spatio-temporal edge aggregation phase. Its evaluation spans three trajectory datasets; the Chengdu corpus includes 284,608 trajectories and roughly 220 million trajectory points. The authors report lower query latency than their three baselines and roughly an order of magnitude lower verification time in most cases; the temporal structure is only one part of the full spatial workload. Its construction evaluation also shows that index-build cost and the spatial structure can dominate total cost. Treat it as an adjacent authenticated temporal-index and maintenance baseline—not as a direct comparator for verifier-relative causal history. [VTRQ paper](https://arxiv.org/abs/2608.21314).

This suggests a specific stress test for Mycelix: owner-count and interval-count scaling must be crossed, not varied separately. Benchmark matrices should include sparse/high-owner workloads (many small histories), dense/low-owner workloads (few large histories), and skewed owners (one hot owner plus many quiet owners). A global average can conceal a single owner's index or replay becoming the latency and availability bottleneck.

### VeriTS results that should shape our benchmark

The published VeriTS evaluation is a useful caution against optimizing only for asymptotic node counts or bytes:
- It evaluates five on-chain stream types with about 27,000 to 168,499 records, and time windows covering 100–2,000 records.
- Its reported workload averages five repetitions over 20–40 random windows.
- On its exact path, the authenticated tree touches a near-constant number of objects as the query window grows; at the reported comparison point, object touches differ by up to 130x versus scanning.
- Yet the paper reports that the scan implementations are several times faster in wall-clock query latency in that prototype, because a vectorized scan can beat interpreted tree-object traversal.
- For a 2,000-record window, the authors report KB-scale verification objects, approximately 18x less data than re-download/recompute and 544x less than per-record Merkle membership proofs.

Therefore, our comparison must separate **query computation**, **proof generation**, **proof bytes**, **independent verification**, **index build/update**, **memory**, and **end-to-end latency**. We should reproduce the published workload shape before extending it to 1k / 10k / 100k / 1m records, 1 / 10 / 100 / 1k / 10k owners, and delayed/backdated correction schedules. A lower node count is not evidence of a faster user-visible query; a smaller proof is not evidence of a complete accepted-frontier result.

VeriTS also makes an approximate model-segment path available with certified value/time error bounds. That is a plausible later **analytics-only** research branch: approximate aggregate dashboards may use a certified interval result with an explicit error envelope, while financial eligibility, revocation, settlement and mutation gates continue to require exact evidence. Any such extension must bind segments to the same owner-frontier and correction lineage and fall back to exact or Indeterminate when a later correction invalidates a segment. It must not silently turn approximate summaries into authoritative individual facts.

## 7. Independent executable oracle and local qualification record

The branch now includes a separate Go reference package at `tools/economic/temporal-hindsight/reference/go/`. It does not import the candidate temporal index or forest code. It implements a simple owner-prefix scan as an independently readable expected-result oracle.

The oracle binds stable owner-history identity and owner-local sequence cut, isolates endpoint pairing by namespace/subject/claim/`validity_interval_ref`, requires explicit OPEN, resolves supersession only inside the accepted prefix, and ignores post-cut suffix content. Its positive disposition is deliberately named `CompleteForSuppliedOwnerPrefixes`; it does not authenticate the owner set, frontier checkpoint or source truth.

Local execution record (Go 1.23.2, linux/amd64, Intel Xeon Platinum 8573C):

| Command | Local result |
|---|---|
| `go test ./...` | Pass |
| `go test -race ./...` | Pass |
| `go vet ./...` | Pass |
| Core suffix/backdated/mutant tests, `-count=30` | Pass |
| Owner interleaving permutations | 100 deterministic permutations pass |
| Suffix-only adversarial mutations | 10,000 deterministic mutations pass |
| Fixed result-certificate SHA-256 vector | Pass |

A separate bounded benchmark was run with 30 iterations per sub-benchmark. The scan oracle reported 31.3 µs at one owner / ten intervals, 27.6 ms at 1,000 owners / ten intervals each, and 43.7 ms at one owner / 10,000 intervals. Full raw samples and allocation counts are in `reference/go/BENCHMARK-OBSERVATIONS.md`. These are oracle-cost observations, not index results, p50/p95/p99, or SOTA claims.

A dedicated qualification workflow bootstrap is proposed separately in [PR #6793](https://github.com/Luminous-Dynamics/mycelix/pull/6793). It pins TLA+ tools v1.7.4 by SHA-256, checks out the exact PR head, runs the Go oracle gates and both TLC configurations, and requires the faulty mutant to produce the expected counterexample. Until that workflow exists on `main` and has a successful exact-head run, hosted qualification remains pending.

## 8. Boundaries

This bounded model abstracts cryptographic verification, owner-set discovery, schema parsing, interval pairing and policy execution. It does not prove source truth, trusted clocks, global absence, signed-checkpoint authenticity, legal effect, financial authorization, settlement or permission to mutate external state.

TLC execution is required before the model is described as model-checked. Cross-language implementation parity and independent source/checkpoint authentication remain separate gates.
