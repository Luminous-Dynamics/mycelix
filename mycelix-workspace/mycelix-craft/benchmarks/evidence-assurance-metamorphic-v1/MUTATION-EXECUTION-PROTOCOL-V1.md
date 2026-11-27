# Metamorphic Mutation Execution Protocol V1

**Status:** declarative execution contract only. No mutation runner or owner adapters are implemented by this artifact.

## Immutable inputs and projections

1. Fetch the exact source fixture revision and verify its file digest.
2. Load the matching row in owner-inputs.json. The owner projection is a separately versioned synthetic supplemental view; it does not mutate or replace the source fixture.
3. Compute exact canonical byte digests for both source fixture and owner projection. Shape-only digests in synthetic source records are placeholders and must never be used as commitments.
4. Reject mismatched seed IDs, source fixture revisions, context digests or schema versions.

## One mutation per execution

1. Run the baseline owner evaluator and independent oracle on the exact source + projection.
2. Resolve the selector. Zero matches, more matches than the specification permits, or a missing field produces invalid_mutation/not_applicable; it is not counted as a survived mutant.
3. Check all preconditions before transformation.
4. Apply exactly one registered operation. No hidden cleanup, normalisation or other edits are permitted.
5. Verify the declared postconditions and compute changed-field digests.
6. Verify that every non-target field is byte-identical after canonicalisation. If it is not, the mutation is invalid.
7. Where natural-language semantics, consent, materiality, translation or dissent is involved, qualified original-language reviewers must verify the intended semantic change. Model self-attestation is insufficient.
8. Freeze the mutated fixture and record all transformation and adjudication receipts before calling the owner.

## Oracle execution

1. Invoke the relevant owner evaluator with the exact baseline or mutated bytes and pinned owner version.
2. Invoke an independently implemented reference oracle. Critical and high-risk mutations require both owner and independent-reference oracle classes.
3. Record canonical traces and replay where ordering or nondeterminism is material.
4. Check the metamorphic relation, not a generic score or an LLM-generated narrative.
5. Run the registered negative control in a separate immutable execution. A control must demonstrate that legitimate progress is still possible.
6. Oracle disagreement is reported as oracle_disagreement. It is neither killed nor survived until adjudicated.

## Outcomes and denominators

Report killed, survived, invalid_mutation, not_applicable, equivalent, timeout, oracle_disagreement, owner_execution_error and independent_oracle_error separately.

Only validated, applicable, non-equivalent mutants enter the mutation-detection denominator. Publish the excluded count and reason codes. Never drop difficult mutants silently and never count an ambiguous selector as a system success.

## Receipt requirements

Every execution receipt binds:
- benchmark/relation/mutation/seed IDs;
- source fixture and owner projection digests;
- exact baseline and mutated input digests;
- selector matches and precondition outcomes;
- transformation ID and exact changed-field paths/digests;
- proof that non-target fields were preserved;
- semantic-validity outcome and human adjudication references;
- owner evaluator version and output digest;
- independent oracle version and output digest;
- canonical trace and replay digest;
- final outcome and reason codes.

## Claims boundary

The current files describe how future executions must be performed. They do not call owner evaluators, run mutations, establish oracle independence, or demonstrate benchmark scores. Passing structural validation means only that the execution contract is complete and cross-referenced.
