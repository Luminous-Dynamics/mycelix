# LISE Invalidation Dependency-Closure Evaluator

Dependency-free Go reference for the **benchmark-only** dependency-closure relation in `docs/research/assurance-invalidation/closure/`.

## Run

From the repository root:

```sh
cd tools/lise-invalidation-closure
go test -race ./...
go vet ./...
go run . -json
```

## Contract

Edges are directed from an upstream dependency node to a dependent node or claim-dimension node. A `QUALIFIED` change event seeds breadth-first traversal; reachable claim dimensions are candidate-affected. An `UNKNOWN` event, or an incomplete/unknown change inventory with explicit unknown-scope claim nodes, yields unknown impact. An `UNSUPPORTED` event is reported as rejected and never treated as a qualified change.

The evaluator reports three disjoint sets:
- affected claim-dimension nodes;
- unknown claim-dimension nodes;
- unaffected-by-this-declared-change-set claim-dimension nodes.

Unknown coverage takes precedence over a partially known affected result for the same claim. Graph cycles, dangling edges, invalid event targets, duplicate nodes/edges/events, invalid A/B pairs and scenario/oracle mismatches fail closed.

## What it does not do

This evaluator does not decide `EvidenceApplicability`, `CurrentnessAssessment`, `EvidenceLease`, legal/constitutional authority or effect permission. Those remain owner-specific semantics (#1087, #2631, #181, #4753, #6219). It calculates only a benchmark impact-candidate closure over the explicitly supplied fixture graph. It is not a production dependency graph, evidence store or continuous monitoring service.

The fixture reuses concepts from [#1243 LEX-NET-030](https://github.com/Luminous-Dynamics/mycelix/issues/1243) and [#5762](https://github.com/Luminous-Dynamics/mycelix/issues/5762); it does not replace either owner.

The public corpus and oracle are calibration data, not a blinded holdout. A successful run establishes only exact replay of the declared finite graph fixtures.
