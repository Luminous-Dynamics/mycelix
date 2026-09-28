# D4 — ITC projection → FRS feedback

D4 composes the bounded ITC projection into an FRS feedback seam while preserving the distinction between source observation, assessment, recommendation, and CDS decision.

## Model

The reference seam:

1. consumes an existing source-bound `ItcProjection`;
2. creates an FRS `Assessment` with the original observation binding;
3. preserves source origin and uncertainty;
4. allows a separate `Recommendation` to reference the finding;
5. rejects a mutated observation binding;
6. keeps CDS decisions as separate human/community artifacts.

FRS therefore receives feedback about evidence rather than rewriting the underlying evidence.

## Human boundary

The eventual cockpit can show:

**What was observed? → What accounting projection followed? → What finding was produced? → What remains uncertain or disputed? → What can humans review/change/appeal?**

Symthaea-assisted explanation may make this chain easier to understand, but explanation or recommendation does not become legitimacy or authority.

## Claim ceiling

`ReferenceModelOnly`.

No claim of Integral ratification, real-world economic validity, worker compensation, or human outcomes.