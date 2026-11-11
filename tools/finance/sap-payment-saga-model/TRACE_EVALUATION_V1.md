# Deterministic Trace V1 evaluator

Status: executable effects-disabled interpreter and independent Rust/Go disposition tables. Synthetic model qualification only.

## Components

- src/trace.rs: frozen Trace V1 generator and canonical byte codec. Do not change its opcode values, fixed-width encoding, six-draw SplitMix64 schedule or scenario templates. Changes require Trace V2 and fresh KATs.
- src/trace_eval.rs: Rust interpreter that creates the PaymentJournal at CreateIntent and executes commands in order. It uses the effects-disabled saga model transitions.
- sap-payment-saga-go-verifier/trace_eval.go: independent Go interpreter against the Go saga model.
- Rust and Go trace evaluator tests: run a fixed 4,096-trace seed corpus and verify scenario coverage plus invariant/oracle agreement.
- src/bin/trace-eval.rs: deterministic scale runner and timing/digest reporter.

## Oracle separation

The generator produces input, not expected answers. The independent disposition table maps opcodes/faults to expected outcomes. The interpreter calls saga model APIs and records observed dispositions. Post-command invariants independently recompute the relevant constraints.

The initial invariant set covers:
- operation and effect identity stability;
- local debit = principal + fee and local credit = principal;
- Treasury fee exact amount and dependency on the committed local payment;
- no Completed status while a required effect is unresolved;
- indeterminate-state preservation under uncertain lookup;
- changed-intent rejection without retroactive mutation of the committed original;
- wrong-owner, stale-frontier and fork-checkpoint rejection;
- compensation reported as a separately authorized effect, never an in-place rollback.

The V1 authoritative-absence command is a synthetic fixture observation. It does not prove a real provider's query semantics or authorize production retries.

## Run the corpus

From the repository root:

cargo run --release --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml --bin trace-eval -- 1592598566 10000 0

Arguments are seed, count and start_index. The runner exits nonzero on a model/oracle error and prints trace and command counts, the distribution across all eight scenarios, elapsed time, traces/second, commands/second, p50/p95/p99 per-trace evaluation latency and a SHA-256 over the ordered trace-digest sequence.

Scale the same seed and contiguous index range to 100,000 and 1,000,000. Preserve exact source SHA, rustc version, hardware, OS, command line, corpus digest and raw output when publishing results. Do not compare throughput across different machines or build modes as though it were a matched baseline.

Go tests independently evaluate the same seed=0x5eed2026, indexes 0..4095 corpus. The next differential step is to emit a shared per-trace disposition digest and compare Rust/Go outputs byte-for-byte, then add the counterexample shrinker.

## Performance targets (research targets, not achieved results)

- 10k: qualification smoke run.
- 100k: sustained determinism and memory run.
- 1M: scale qualification once the 100k run is stable.
- Zero invariant/oracle failures.
- Zero cross-run or cross-language disposition divergence.
- 100% coverage of the eight frozen scenario templates.
- p50/p95/p99, traces/sec and commands/sec reported for every run.
- 100% reproducible failure replay and minimized-trace digest after the shrinker is delivered.

Passing synthetic traces do not establish Holochain multi-agent atomicity, partial-DHT convergence, provider receipt authenticity, bank liquidity, legal finality or regulatory compliance.
