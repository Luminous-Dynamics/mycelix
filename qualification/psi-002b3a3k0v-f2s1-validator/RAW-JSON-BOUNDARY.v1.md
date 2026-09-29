# Raw JSON input boundary v1

Status: **CONTRACT FREEZE / NOT EXECUTED / NOT A PASS**

## Purpose

This boundary prevents ambiguous JSON objects from entering later structural-schema validation. It is a syntax and evidence-identity gate only; it is not a schema validator, issuer-directory parser, currentness oracle, or cryptographic verifier.

## Normative rules

1. The input is the exact byte sequence supplied by the caller. Compute SHA-256 over those bytes before decoding; retain that digest as raw-input evidence.
2. Decode as strict UTF-8. Reject invalid UTF-8 and a leading UTF-8 BOM. Do not repair, replace, or normalize bytes.
3. Parse exactly one complete JSON value. Reject malformed JSON and non-whitespace trailing data.
4. Reject an object if a member name repeats within that same object. Compare names after JSON escape decoding, so `"a"` and `"\\u0061"` collide. The check applies recursively at every object depth. Equal and unequal duplicate values are both rejected.
5. Preserve array order. Preserve distinctions represented by the parsed JSON model, including absent member versus explicit `null`, and empty string, array, and object.
6. This boundary does not canonicalize JSON, reorder members, normalize whitespace, or equate numeric spellings. Raw-byte identity is the SHA-256 of the original bytes. Any future semantic canonicalization must be specified and independently qualified as a separate gate.
7. A parse/duplicate failure blocks schema evaluation. Downstream status must be `NOT_EVALUATED`; it must not be represented as a schema rejection or a currentness decision.

## Reference implementation and vectors

- `tools/check_raw_json.py` implements the duplicate-aware parse boundary using Python's `object_pairs_hook`, which observes object member pairs before conversion to a dictionary.
- `tools/test_raw_json_boundary.py` covers top-level and nested duplicates, equal and unequal values, escaped-name collision, valid empty/null forms, numeric spellings, whitespace/member-order raw identity, malformed/trailing input, BOM, and invalid UTF-8.

The implementation is a reference boundary for this qualification package. A later Rust validator must independently enforce the same raw-input semantics before any deserialization path that could discard duplicate names.

## Claim ceiling

Acceptance means only that the supplied bytes decode as one duplicate-free JSON value under this parser boundary and that their raw SHA-256 is recorded. It does not establish RFC 9578 conformance, schema validity, authenticity, freshness, key admission, currentness, token validity, replay authority, PSI qualification, or contact-discovery security.

**NOT EXECUTED / NOT A PASS.**
