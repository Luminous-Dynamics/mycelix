# D2 — OAD → CDS → COS vertical slice

## What this demonstrates

A concrete reference-node path now composes the existing OAD/COS admission model with the demo's human-boundary domain seam:

1. OAD supplies a versioned design.
2. CDS records an explicit human/community decision for that exact design generation.
3. COS evaluates the design against the expected generation/profile and source maturity.
4. A matching production authorization is required before the path can become an execution intent.
5. The model stops at the COS semantic boundary; it does not claim physical execution.

## Fail-closed cases

The executable witnesses cover:

- no CDS decision;
- non-human/assistive decision presented as human authorization;
- decision for an older design generation;
- stale or mismatched design generation;
- non-ratified source status;
- missing authorization;
- authorization for the wrong production profile.

## Why this matters for the eventual demo

The UI should be able to show these as actual state transitions instead of merely describing them. A participant should be able to see:

**What design? → What did the community decide? → What authorization exists? → What did COS admit? → What has actually happened?**

The final question remains unanswered by this slice because execution/observation is deliberately downstream. That distinction is important: the demo should never turn a successful semantic admission into a fabricated claim that production occurred.

## Assurance ceiling

`ReferenceModelOnly`.

This PR does not establish Integral ratification, production correctness, manufacturing performance, economic validity, safety, or human outcomes.
