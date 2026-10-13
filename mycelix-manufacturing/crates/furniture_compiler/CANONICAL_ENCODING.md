# Furniture Compiler Canonical Encoding Specification

**Profile:** `canonical-binary-v2`  
**Packet schema:** `furniture-compiler-packet-v2`  
**Status:** Reference-protocol specification; Rust tests authored but not yet executed in the current environment.  
**Purpose:** Define the exact bytes that the reference compiler hashes so a future independent verifier does not infer encoding rules from JSON presentation.

This is a versioned application protocol, not RFC 8785 canonical JSON and not a safety or qualification standard. JSON is the transport/display form. Hashes cover the byte encodings described here.

## 1. Primitive encoding

All integers are unsigned, big-endian:
- `u32`: exactly 4 bytes.
- string: `u64` UTF-8 byte length, then exactly that many UTF-8 bytes.
- sequence/list: `u64` element count, then elements in defined order.
- boolean: exactly one byte, `00` for false and `01` for true.
- enums: explicit one-byte discriminants in Section 2.

There is no alignment, padding, terminator, or implicit host-endian data. Strings are length-prefixed; embedded zero bytes are permitted by the primitive encoding, although identifiers in this prototype use known UTF-8 values.

## 2. Enum discriminants

### MaterialId
- `01` = `birch_plywood`
- `02` = `melamine_mdf`

### FinishId
- `01` = `clear_coat`
- `02` = `white_melamine`
- `03` = `black_melamine`

### GrainAxis
- `01` = `length`

Any new enum value requires a defined discriminant and a reviewed protocol/schema version change. Never use a Rust enum's implicit memory representation or ordinal without a dedicated encoder.

## 3. Configuration encoding

Fields are concatenated in exactly this order:
1. `schema_version` (string)
2. `width_mm` (u32)
3. `depth_mm` (u32)
4. `height_mm` (u32)
5. `panel_thickness_mm` (u32)
6. `material_id` (MaterialId discriminant)
7. `finish_id` (FinishId discriminant)

No JSON keys, commas, whitespace, field tags, or object delimiters appear in this encoding.

### Configuration golden vector

For the sample configuration `furniture-desk-input-v1, 1200, 600, 750, 18, birch_plywood, clear_coat`, the expected configuration bytes are:

- `00 00 00 00 00 00 00 17` — UTF-8 string length 23
- UTF-8 bytes of `furniture-desk-input-v1`
- `00 00 04 B0` — width 1200
- `00 00 02 58` — depth 600
- `00 00 02 EE` — height 750
- `00 00 00 12` — panel thickness 18
- `01 01` — birch plywood, clear coat

The unit test `canonical_configuration_encoding_has_a_fixed_golden_vector` asserts this byte sequence.

## 4. Family rules encoding

The current reference compiler binds the following fixed rules into the design identity, in order:

1. `MIN_WIDTH_MM` (u32) = 600
2. `MAX_WIDTH_MM` (u32) = 1600
3. `MIN_DEPTH_MM` (u32) = 400
4. `MAX_DEPTH_MM` (u32) = 900
5. `MIN_HEIGHT_MM` (u32) = 650
6. `MAX_HEIGHT_MM` (u32) = 850
7. `PANEL_THICKNESS_MM` (u32) = 18
8. `REAR_APRON_HEIGHT_MM` (u32) = 100
9. material/finish allowlist count (u64) = 3
10. allowlist entries in this exact order: (MaterialId 01, FinishId 01), (MaterialId 02, FinishId 02), (MaterialId 02, FinishId 03).

The compiler uses this same allowlist constant for configuration validation and rules encoding. The bounds are schema gates, not ergonomic recommendations or safety/load limits.

The ruleset identity strings are currently:
- `family_ruleset_version = modular-desk-rules-v1`
- `geometry_ruleset_id = rectangular-panel-envelopes-v1`

If geometric semantics change, increment the geometry-rule identifier. If constraints/material logic change, review and increment the family ruleset identifier and ensure the encoded values match the changed rules. If fields, primitive representation, or field order change, version the packet/encoding schema as well. Code review should explicitly inspect changes to these constants.

## 5. Cut-list encoding

The list begins with a u64 part count. Each part then encodes, in order:
1. `part_id` (string)
2. `quantity` (u32)
3. `length_mm` (u32)
4. `width_mm` (u32)
5. `thickness_mm` (u32)
6. `material_id` (MaterialId discriminant)
7. `finish_id` (FinishId discriminant)
8. `grain_axis` (GrainAxis discriminant)

Parts are sorted lexicographically by `part_id` before output and before hashing.

For this prototype only, the generated rectangular panel envelopes use these formulae:
- `desk_top`: quantity 1; length = configured width; width = configured depth.
- `side_panel`: quantity 2; length = configured height − panel thickness; width = configured depth.
- `rear_apron`: quantity 1; length = configured width − 2 × panel thickness; width = fixed rear-apron height of 100 mm.
- Every part's thickness/material/finish are copied from the configuration; grain axis is `length`.

These are not a complete desk design: they omit joinery, fasteners, drilling, support details, tolerance strategy, structural verification, and approved manufacturing processes. The output is not eligible for production.

Cut-list hash domain: `cut-list-v1`.

## 6. Candidate process-step encoding

The list begins with a u64 count. Each step encodes:
1. `sequence` (u32)
2. `operation_code` (string)
3. `requires_partner_mapping` (boolean byte)

The current fixed candidate sequence is:
1. `panel_cutting`
2. `edge_finishing`
3. `joinery_and_hardware_definition`
4. `inspection_and_packaging`

Every step has `requires_partner_mapping = true`; these are proposed operation labels, not a qualified routing.

## 7. Release-gate encoding

Fields in order:
1. `configuration_bounds_passed` (boolean byte)
2. `release_eligible` (boolean byte)
3. blocker count (u64)
4. blocker strings in list order (each encoded as a string).

The current compiler must always emit `release_eligible = false` and non-empty blockers. Recomputing a packet hash must not override the intended gate: verification recompiles the expected packet from its configuration and compares the complete packet, rather than trusting the supplied outer digest alone.

## 8. Hash input and domains

Each digest uses the BLAKE3 derive-key API with context string:

`Luminous Dynamics Furniture Compiler v1`

Then hash, in order:
1. UTF-8 bytes of the domain string,
2. one zero byte,
3. u64 big-endian length of the canonical encoded payload,
4. canonical encoded payload bytes.

Current domains:
- `design-spec-v2`: family ID string, family revision u32, ruleset-version string, geometry-rule ID string, family rules encoding (Section 4), configuration encoding (Section 3).
- `cut-list-v1`: cut-list encoding (Section 5).
- `manufacturing-packet-v2`: packet-core encoding in Section 9.

Output is lowercase hexadecimal BLAKE3 digest text. Do not hash JSON presentation output as a substitute for these encodings.

## 9. Packet-core encoding

Fields are concatenated in exactly this order:
1. `packet_schema_version` (string; `furniture-compiler-packet-v2`)
2. `serialization_profile` (string; `canonical-binary-v2`)
3. `family_id` (string)
4. `family_revision` (u32)
5. `family_ruleset_version` (string)
6. `geometry_ruleset_id` (string)
7. configuration encoding (Section 3)
8. cut-list encoding (Section 5)
9. candidate process-step encoding (Section 6)
10. release-gate encoding (Section 7)
11. `design_digest` (string containing lowercase hex)
12. `cut_list_digest` (string containing lowercase hex)

The `packet_digest` field is not included within the packet-core payload; it is the resulting digest over that payload using the `manufacturing-packet-v2` domain.

## 10. Verification and claims

An independent verifier must:
- implement the encoding above without depending on this Rust crate's serialization behavior;
- reproduce the configuration golden vector;
- compare encoded outputs and hashes against test vectors generated by a verified compiler run on the exact commit;
- reject unknown schema/profile/ruleset/geometry IDs until explicitly supported;
- recompute expected parts, steps, blockers, subordinate digests, and packet digest from the bound configuration;
- reject mutation of any generated field, especially the production-release gate.

Passing these checks establishes conformance to this reference software protocol only. It does not establish who authored an input, whether supplier evidence is true, whether a design is safe, whether a test lab is competent, or whether a product complies with law or a product standard.
