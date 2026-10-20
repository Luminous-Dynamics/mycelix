# Capability-plan compiler benchmark protocol

Status: protocol and harness authored; measurements not yet collected.

## Question being measured

How does the deterministic Rust planner's runtime vary with catalogue size, dependency shape, option count, and one known structural rejection path?

This is a software-cost benchmark only. It does not establish that the selected tasks teach better, that estimated minutes are accurate, that the catalogue is correct, or that the planner is optimal.

## Run and preserve the evidence

Run from `mycelix-workspace/mycelix-praxis` on the exact source revision under review:

```sh
rustc -Vv
cargo -V
uname -a
cargo bench -p praxis-core --bench capability_plan > capability-plan.csv
```

Capture the exact full commit SHA, repository clean/dirty state, CPU model/core count, memory, OS/kernel, Rust/Cargo versions, Cargo profile, benchmark stdout and stderr, and any relevant power/thermal constraints beside the CSV. Do not compare results from different source SHAs, machines, profiles, or toolchains as if they were controlled.

The harness executes one excluded warm-up before each valid workload, then 30 timed samples. It reports min, nearest-rank p50 and p95, and max in nanoseconds. Thirty samples give a coarse p95 estimate; the raw output and environment record are required. No performance threshold is treated as passed merely because a benchmark row exists.

## Workload matrix

Valid workloads cover 10, 100, 512 and 4,096 catalogue nodes. Option counts are 1, 8 and 64 at up to 512 nodes, and 1 and 8 at 4,096 nodes.

- `chain`: a long prerequisite chain; stresses deep prerequisite closure and ordered planning.
- `wide_fan_in`: one target depends on many preceding nodes; stresses broad target closure.
- `balanced_tree`: parent-linked tree; contrasts catalogue size with a logarithmic target closure.
- `shared_prerequisites`: many options/nodes reuse a small prerequisite set; stresses shared dependency structure and irrelevant catalog content.
- `isolated_components`: small prerequisite components alongside unrelated nodes; tests catalogue-wide work versus target-local work.

A negative-control row per size/option count introduces one missing prerequisite. The harness first confirms the expected `UnknownPrerequisite` failure, then times repeated attempts and fails the run if the failure class changes or malformed input is accepted. It is one error-path sample, not a comprehensive rejection benchmark.

## Output interpretation

CSV columns are `case,shape,nodes,options_per_node,iterations,steps,outcome,min_ns,p50_ns,p95_ns,max_ns,input_digest`.

For successful cases, `input_digest` comes from the planner's canonical normalized graph/request inputs. It binds the declared input representation; it is not a signature, proof of catalogue truth, or proof of plan quality. A negative-control case is rejected before the planner emits a plan digest, so its digest is deliberately `not_applicable`.

Check output invariants before comparing performance:
- all valid rows succeed with nonzero expected plan length;
- the chain/wide-fan-in cases at these sizes produce the expected target closure;
- the negative-control cases reject specifically as `UnknownPrerequisite`;
- equivalent normalized inputs retain deterministic plan/digest behavior in the unit tests;
- no workload or failed sample is silently dropped.

## Qualification roadmap

The first run establishes a baseline, not superiority. Before claiming improvement, preserve a baseline CSV for the same exact workload matrix and machine. A later optimization must compare like-for-like source inputs and report per-shape regressions as well as aggregate changes. Avoid one scalar “planner score.”

Next qualification increments should add, with their own correctness fixtures:
1. bounded cycle-rejection and budget-infeasibility timing;
2. peak memory/allocation measurements using an appropriate stable measurement environment;
3. a frozen, versioned randomized-DAG fixture generator and a small independent reference/oracle for route validity;
4. repeated-process and reordered-input determinism checks.

The benchmark does not measure human learning. Educational qualification remains separate: randomized or otherwise defensible comparison against a fixed path, existing Praxis baselines and a reproducible external knowledge-tracing baseline; independently graded unseen-task transfer, 30/90-day retention, time-to-competence, calibration and false-mastery, assistance-off results, attrition and accessibility outcomes. Do not claim SOTA until a pre-registered, fair evaluation has actually run.
