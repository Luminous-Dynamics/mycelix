# Integral Interop 1 — Numeric Boundary

Status: **ReferenceModelOnly**

## Finding

The public Integral OAD model includes numeric ecological, material, lifecycle, and labor fields represented as floating-point values in its published pseudocode. The public OAD valuation profile describes bill-of-materials quantities in kg and exposes fields such as material intensity, ecological score, embodied energy/carbon, lifespan, production labor, and maintenance labor as float values. citeturn1search1turn1search0

The current Mycelix D6S-CANON-1 reference model intentionally accepts only integral JSON numbers and rejects non-integral numbers. That is a fail-closed boundary: an Integral value such as 0.25 kg must not silently acquire an implementation-dependent floating-point identity.

## Conformance rule

For integral-interop-1:

- integral selected numeric values are canonicalizable under D6S-CANON-1;
- fractional selected numeric values are rejected;
- no implicit rounding, binary-float normalization, decimal-to-float conversion, or stringification is permitted to make a fractional value pass;
- production-step array order remains semantic.

The regression test integral_interop_1_numeric_boundary.rs freezes these behaviors.

## Why this matters

RFC 8785/JCS has its own ECMAScript-compatible number serialization rules and therefore cannot be substituted for the current D6S numeric contract merely because both schemes provide deterministic JSON canonicalization. citeturn0search0

Integral's published design model makes this boundary concrete: future ecological/lifecycle semantics are likely to contain fractional quantities. We should therefore **not** silently broaden D6S-CANON-1.

## Future extension

If fractional Integral semantics become selected D6X dependencies, use an explicit versioned numeric contract, for example:

1. define a new canonicalization/profile version;
2. specify decimal semantics independently of host-language floating point;
3. choose either exact decimal text or a fixed-point integer representation with an explicit scale;
4. publish cross-language vectors covering positive/negative zero, exponent forms, scale preservation rules, rounding rejection, and boundary magnitudes;
5. keep D6S-CANON-1 unchanged for existing commitments.

A future D6S-CANON-2 should never reinterpret an existing D6S-CANON-1 byte sequence.

## Claim ceiling

This document does not establish an Integral wire schema or formal interoperability agreement. It records a conservative interoperability boundary against the public Integral reference material and the current Mycelix reference model.
