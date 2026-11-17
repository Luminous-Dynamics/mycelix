# Mycelix VOEvent 2.1 root-header parser

This isolated Rust package is the first executable astronomy adapter slice under [ASTRO-VOEVENT-001](https://github.com/Luminous-Dynamics/mycelix/issues/6147).

## What it does

- Accepts a bounded UTF-8 XML packet (2 MiB maximum).
- Requires one unprefixed VOEvent root element.
- Requires ivorn and version 2.1.
- Applies the VOEvent 2.1 role default (observation) when role is absent while preserving that omission as raw_role = null and role_was_implicit = true.
- Preserves observation, prediction, utility and test as distinct role values.
- Preserves all root attributes for later loss accounting.
- Rejects duplicate root attributes, mismatched element names, invalid XML, multiple roots, DOCTYPE declarations, unsupported role values, wrong versions and non-IVOID root identifiers.
- Binds the parsed header to the SHA-256 digest and byte length of the exact packet bytes.

## Run

From the repository root:

    cargo test --manifest-path research/voevent-header/Cargo.toml
    cargo run --manifest-path research/voevent-header/Cargo.toml --bin voevent-header -- path/to/packet.xml

The CLI emits a JSON header projection. It does not rewrite the input packet.

## Deliberate scope limit

This is a root-header parser, not a complete VOEvent 2.1 XSD validator or a source trust service. In particular it does not yet parse or qualify:
- Who, What, WhereWhen, How, Why, Citations, References, follow-up and external content;
- full STC coordinate/time/frame semantics;
- IVOID registry resolution or publisher authentication;
- XML signatures;
- source rights or artifact retrieval permissions;
- scientific event reality or astrophysical hypotheses.

The original packet bytes and digest remain authoritative. Parsed fields are a derived projection tied to this parser version. A valid header is not proof that the remainder of the packet is schema-valid or that the physical event occurred.

## Frozen unit tests

The library tests cover explicit and implicit observation roles, prediction/utility/test separation, unknown role, wrong/missing version, missing/invalid IVOID, duplicate attributes, mismatched nested tags, DOCTYPE, multiple roots, malformed/incomplete XML, text outside the document root, oversized/non-UTF-8 inputs and retention of uninterpreted root attributes.

Synthetic unit fixtures are not real astronomical events and are not an IVOA conformance certification.
