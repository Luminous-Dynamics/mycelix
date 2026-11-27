# Historical frontier noninterference — TLC checks

Run the bounded paired-execution model and its two negative controls with a
pinned TLA+ tools JAR:

```sh
TLA2TOOLS_JAR=/path/to/tla2tools.jar \
  bash tools/economic/temporal-range-index/formal/verify-frontier-noninterference.sh
```

The qualification runner performs three independent TLC invocations:

1. The correct `Spec` must complete without an invariant violation.
2. The latest-frontier replay mutant must violate
   `HistoricalReplayBound`.
3. The certificate-relabel mutant must violate
   `ExactHistoricalCertificateBinding`.

The mutant checks are intentionally separate. A counterexample for one defect
cannot stand in for detection of the other. A TLC parser/tool failure also
cannot be mistaken for the expected counterexample: the runner matches the
specific invariant name in the TLC output.

The GitHub Actions workflow downloads TLA+ tools v1.7.4 and verifies the
release JAR against the SHA-1 checksum published by the upstream release
before running the script. The workflow pins its GitHub Actions by commit SHA.

## Scope

A successful run checks this finite TLA+ abstraction only. It is not a proof
of the production Go/Rust projection implementations, authenticated checkpoint
or witness trust, real-world owner discovery, source truth, legal effect,
authorization, settlement, or permission to mutate external state.

The model currently has two owner histories, one base receipt and one
post-frontier backdated correction per owner, two replica views, and a fixed
historical frontier. The finite state-space and abstraction limits are part of
the claim; the result must not be described as an unbounded proof.
