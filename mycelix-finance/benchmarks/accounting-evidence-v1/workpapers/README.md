# FIN-ACC workpaper DAG v0.1

This directory defines a first, versioned shape contract for durable accounting close workpapers.

## Why

Accounting agents must not keep critical intermediate amounts, filters, denominators, join assumptions and policy context only in conversational memory. A workpaper node binds each result to its input frontier, computation recipe, exact units, parent nodes, verification state and review obligation.

This directly targets multi-step propagation failures while preserving Mycelix's existing source-of-truth and authority boundaries.

## Files

- `WORKPAPER_DAG_V0.1.schema.json` — Draft 2020-12 structural schema.
- `examples/acc-syn-0001-minimal-dag-v0.1.json` — synthetic three-case smoke-suite example: PO/GRN/invoice evidence and a separate bank debit observation in ACC-SYN-0001, plus the wrong-entity quarantine negative control from ACC-SYN-0003. The reporting context is co-001; the co-002 invoice is retained only as a conflicting out-of-scope source.

## Ownership

- Mycelix owns source/scope, evidence qualification, accounting policy, journals and statement facts.
- Symthaea may produce candidate assertion/reconciliation workpapers, but it cannot grant posting authority.
- Symtropy may provide simulated/observed operational inputs with their evidence state; simulation outputs remain non-authoritative.
- Human reviewers/authorized finance owners retain the review and approval responsibilities specified by the applicable action/policy contract.

This schema composes existing Mycelix Business Fabric #566–#571. It does not define a second workflow engine, Action Contract, Decision Capsule, authorization model, reservation, execution receipt, ledger or benchmark-run receipt.

## Structural vs semantic validation

JSON Schema validates field types, required properties, enumerations, formatting and some result-state combinations. It **cannot** prove:
- node IDs are unique across the DAG;
- dependencies refer to existing nodes;
- the dependency graph is acyclic;
- parent payload digests match the exact parent workpaper bytes;
- a stale frontier invalidates the full transitive descendant set;
- exactly one terminal disposition exists for every scheduled node;
- terminal state is consistent with its evidence and verifier results;
- a candidate workpaper has not been elevated into financial authority;
- an accounting-close result is complete when the source universe itself is not qualified.

Those conditions require the independent semantic graph verifier in #5805 and adversarial tests. Do not claim DAG correctness from schema-valid JSON alone.

## Exact calculation contract

A calculation recipe retains named numerator and denominator references, source inputs, filters, join keys/cardinality, group-by fields, reporting cutoff, missing/null semantics, unit profile, precision and rounding stage. In particular:
- unknown is not zero;
- one-to-many and many-to-many joins require explicit aggregation semantics;
- ratio operations require a named denominator and a zero-denominator disposition;
- currency/unit conversion requires a versioned profile and separate evidence;
- monetary minor-unit results use canonical signed integer strings bound to a versioned unit profile;
- ratios/weighted averages can use exact numerator/denominator strings without premature rounding;
- fixed-decimal results declare an explicit scale and unit profile, with exact decimal parsing (no exponent/floating-point coercion);
- the calculation recipe still binds the source population, filters, denominator, joins and rounding stage.

## Claim ceiling

A schema-valid example establishes only that the payload matches this structural contract. It does not prove correct accounting treatment, source authenticity/completeness, audit sufficiency, tax/IFRS compliance, or production-ready orchestration.


## Required-stage manifest

The DAG manifest includes `required_node_kinds` for the selected close profile. This makes required stages explicit rather than inferring them from whichever nodes happen to be present. A structurally valid graph that omits a required node kind is not a complete close. The semantic verifier in #5805 must check that each required kind has a qualifying node and that all required nodes are terminal and valid before a close can be considered ready. The source-scope manifest also carries an explicit qualification state; a close cannot be `ready_for_authorized_signoff` while the source universe is unknown or unqualified.

Dependencies are represented twice for different purposes: `depends_on` defines the graph edge, while an `inputs` entry with purpose `prior_workpaper` binds the consumed parent workpaper identity/version/digest. The semantic verifier must prove these sets agree and verify the exact digest when the parent is completed. A non-terminal parent must not be treated as a completed result.


## Node-scoped frontier and commitment semantics

Each node now binds the exact source-scope, evidence, policy, taxonomy, authority, knowledge and validator frontiers it actually consumed. The DAG-level `frontiers` object records the close's overall frontier; `frontier_bindings` narrows that to each node. A changed frontier invalidates only nodes that consumed it and their transitive descendants, not unrelated workpapers.

Two commitments are distinct:
- `payload_sha256` commits to the exact derived result payload;
- `workpaper_commitment_sha256` commits to the complete canonical workpaper (node identity/version, inputs, parent commitments, calculation recipe, frontier bindings, review requirement, result payload and verifier state).

A downstream `prior_workpaper` input must bind the parent's `workpaper_commitment_sha256` and exact node/version identity. It must not bind only a rendered number or narrative. Completed workpapers require both digests. In-progress examples intentionally have neither.


## Exact numeric result representations

`numeric_value` is a tagged union: integer minor units, fixed-scale decimal, or exact rational. Use integer minor units for ZAR/SAP/etc only when the matching unit profile explicitly defines that atomic unit. Use exact rational for rates/ratios/weighted averages until a profile-defined rounding step is reached. A semantic verifier must reject a zero denominator, a decimal whose textual scale disagrees with its declared scale, non-canonical negative zero, unsupported units, overflow and any attempt to compare/add values with different profiles. Do not convert these values through binary floating point.
