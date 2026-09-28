# Integral five-system adversarial corpus v1

Status: **ReferenceModelOnly**

This artifact joins the existing COS conformance model (#3334) with the twelve-seam interface discipline (#3209) into one oracle-hidden semantic workload.

It is deliberately runtime-neutral. It does not implement or ratify Integral's architecture.

## Canonical workload

```text
OAD design evidence
  -> COS plan / requirements / observations
  -> ITC projection
  -> FRS projection / finding / recommendation
  -> CDS decision
  -> explicit authorization
  -> effect receipt
  -> outcome / recalibration evidence
```

The workload is useful only if every transition preserves source ownership, provenance, temporal validity, and authority boundaries.

## Mutation corpus

| Mutation | Required result |
|---|---|
| stale OAD design accepted as current production basis | reject as stale |
| planned labor treated as observed labor | reject without binding |
| planned material treated as consumed material | reject without binding |
| duplicate labor implicitly creates ITC effect | reject without explicit projection |
| FRS finding becomes source observation | reject without source-observation transition |
| FRS recommendation executes directly | reject without authorization/effect transition |
| timeout becomes definite failure | remain unknown/indeterminate |
| foreign decision becomes local authority | preserve foreign origin |
| FRS-derived summary overwrites source fact | reject semantic collapse |
| historical failure disappears after later success | preserve failure history |
| qualification silently becomes physical execution | require explicit physical-work binding |

## Four seam refinements inherited from #3209

1. FRS recalibration proposal != OAD admission != applied design mutation.
2. FRS packet receipt != governance acceptance != decision.
3. CDS dispatch acknowledgement != local semantic admission != authorization != effect.
4. OAD certification status != production authorization.

These are mutation targets, not assumptions that an implementation is already correct.

## Qualification ladder

1. Reference model defined.
2. Executable adversarial tests present.
3. Tests pass in repository CI.
4. Each transition is mapped to source-owned production paths.
5. Bounded implementation evidence exists.
6. Formal refinement witnesses link implementation to obligations.
7. Independent runtime profile reproduces the same semantic results.
8. Pilot observations establish real-world behavior.
9. External validation, if any, is separately sourced.

A passing synthetic corpus does not establish production correctness, safety, economic/social outcomes, ecological performance, or Integral validation.

## Interoperability target

The same semantic workload should eventually run unchanged against PostgreSQL, Holochain/Mycelix, hybrid, and independent conformer profiles. Runtime-specific adapters may differ; semantic acceptance and claim ceilings must not.

## Relationship to Symthaea

Symthaea may provide bounded analytical evidence or recommendations to FRS/OAD. It does not become the source-of-truth merely because it produced an analysis, and an analytical recommendation does not become authorization without the explicit governance transition.

## External contribution posture

This is designed as a replaceable reference artifact: an Integral implementer should be able to reuse the semantic workload and replace Mycelix runtime components entirely.
