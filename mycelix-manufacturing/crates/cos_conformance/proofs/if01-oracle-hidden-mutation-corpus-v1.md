# IF01 oracle-hidden mutation corpus v1

These cases are deliberately adversarial. A conforming implementation must preserve the
semantic distinctions represented by the candidate IF01 reference model.

| Case | Mutation | Required outcome | Boundary |
|---|---|---|---|
| IF01-M-001 | Valid auth + stale schema | Reject | Schema currentness |
| IF01-M-002 | Valid auth + wrong authorization reference | Reject | Explicit authority binding |
| IF01-M-003 | Semantic admission + mutated retry payload | Reject | Idempotency / replay |
| IF01-M-004 | Duplicate admission with changed attempt ID, same logical delivery/payload | Accept as idempotent duplicate | Retry identity |
| IF01-M-005 | Known recipient rejection followed by retry | Preserve rejection; retry is a new attempt | Delivery outcome |
| IF01-M-006 | Timeout after possible server-side admission | Indeterminate | No invented success/failure |
| IF01-M-007 | Foreign-origin admission followed by local projection | Preserve foreign origin | Provenance |
| IF01-M-008 | Certification changes after authorization | Re-evaluate certification; do not equate authorization with certification | Authority/qualification |
| IF01-M-009 | Authorization valid for generation N, envelope generation N+1 | Reject | Generation binding |
| IF01-M-010 | IF01 receipt presented as COS execution evidence | Reject / remain unbound | Plan != execution |
| IF01-M-011 | IF01 admission presented as ProductiveLoop observed work | Reject | Admission != observation |
| IF01-M-012 | IF01 admission used to mint ITC/FRS/production authority | Reject | Projection/authority separation |

## Oracle-hiding rule

The implementation under test must not read the expected outcome from this corpus at runtime.
Expected outcomes belong to the test oracle only.

## Claim ceiling

Passing this corpus establishes adversarial semantic conformance of the candidate reference
model. It does not establish cryptographic security, network reliability, production
authorization, physical execution, manufacturing qualification, economic outcomes, or Integral
ratification.
