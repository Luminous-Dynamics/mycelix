# Integral Interop 1 — Canonicalization boundary

The current D6S reference model uses `D6S-CANON-1`; it is **not claimed to be RFC 8785 JSON Canonicalization Scheme (JCS)**.

## Current rules that matter for independent implementations

- JSON object properties are recursively sorted by UTF-16 code-unit order.
- JSON array element order is preserved.
- Strings are emitted using the D6S canonical string writer.
- Numeric values are intentionally restricted to integral JSON numbers. Non-integral numbers are rejected rather than normalized through a floating-point algorithm.
- SHA-256 input includes the D6S hash-domain prefix, the caller-supplied domain, a zero separator, and the canonical bytes.
- `D6S-CANON-1` is therefore a versioned Mycelix reference-model canonicalization contract, not a generic assertion of JCS compatibility.

RFC 8785 also sorts object properties recursively and preserves array order, but its number serialization rules are based on ECMAScript-compatible JSON number serialization and its I-JSON constraints. The distinction matters for cross-language implementations: an implementation must reproduce `D6S-CANON-1`, not merely call itself JCS-compatible.

## Interop rule

Do not freeze a new Integral cross-language golden vector merely because two implementations produce the same result for the current integer-only fixture. The conformance corpus should include:

1. Unicode property names exercising UTF-16 ordering.
2. Nested objects and arrays.
3. Integer boundary values supported by the D6S model.
4. An explicit rejection case for non-integral numbers.
5. Domain-separator mutation.
6. Projection-version mutation.

Only after those behaviors agree independently should the vector be treated as a cross-runtime conformance artifact.
