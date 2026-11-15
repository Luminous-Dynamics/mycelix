# Mycelix VOEvent 2.1 bounded packet extractor

This isolated Rust package is the first executable astronomy adapter slice under [ASTRO-VOEVENT-001](https://github.com/Luminous-Dynamics/mycelix/issues/6147). It extracts a bounded root header and a typed lifecycle citation set from one exact VOEvent 2.1 XML packet.

## What it does

- Accepts a bounded UTF-8 XML packet (2 MiB maximum).
- Requires one unprefixed VOEvent root element, an IVOID in the ivorn attribute, and exact version 2.1.
- Applies the VOEvent 2.1 role default (observation) when role is absent while preserving that omission separately from an explicit value.
- Preserves observation, prediction, utility and test as distinct role values.
- Preserves all root attributes for later loss accounting.
- Extracts the optional Who curation block's direct AuthorIVORN and Date values, preserving source lexical values and absence.
- Treats AuthorIVORN as a source assertion, not a verified publisher identity; Date is retained as source packet-creation time and is not used as fabric receipt time.
- Extracts the optional root Citations block and every direct EventIVORN child.
- Requires the cite attribute and accepts only followup, supersedes or retraction; there is no implicit relationship default.
- Preserves each citation's decoded IVOID target and all EventIVORN attributes.
- Preserves the optional Citations/Description as text.
- Resolves XML predefined and numeric character references in citation text; unsupported references fail closed.
- Rejects duplicate root attributes, mismatched element names, invalid XML, multiple roots, DOCTYPE declarations, unsupported role/citation values, duplicate Citations blocks, empty Citations blocks, nested markup inside EventIVORN/Description, wrong versions and non-IVOID identifiers.
- Binds the complete parsed projection to the SHA-256 digest and byte length of the exact packet bytes.

## Run

From the repository root:

    cargo fmt --manifest-path research/voevent-header/Cargo.toml --all -- --check
    cargo test --manifest-path research/voevent-header/Cargo.toml --all-targets
    cargo clippy --manifest-path research/voevent-header/Cargo.toml --all-targets -- -D warnings
    cargo run --manifest-path research/voevent-header/Cargo.toml --bin voevent-header -- path/to/packet.xml

The CLI emits a JSON packet projection with its header, citation set, and optional citation description. It does not rewrite, upload or task an instrument with the input packet.

## Lifecycle semantics

A packet with no Citations element is retained as a packet without lifecycle edges; this is not synthesized into a fake root citation. Each EventIVORN is emitted as a distinct typed edge. The extractor does not decide whether the publisher's assertion is logically consistent, whether the cited packet exists, or whether a cited event should be treated as scientifically true.

The separate [VOEvent lifecycle replay package](../voevent-lifecycle/README.md) reconstructs the received event graph at an explicit receipt-time cutoff. It does not collapse source assertions into a single canonical truth.

## Deliberate scope limit

This is a bounded extraction profile, not a complete VOEvent 2.1 XSD validator or a source trust service. In particular it does not yet parse or qualify:

- Who, What, WhereWhen, How, Why, References, full follow-up content or external content;
- full STC coordinate/time/frame semantics;
- IVOID registry resolution or publisher authentication;
- XML signatures;
- source rights or artifact retrieval permissions;
- source independence, calibration applicability or instrument authority;
- scientific event reality or astrophysical hypotheses.

The original packet bytes and digest remain authoritative. Parsed fields are a derived projection tied to this parser version. A valid header/citation extraction is not proof that the remainder of the packet is schema-valid or that the physical event occurred.

## Frozen synthetic tests

The library tests cover explicit and implicit packet roles, prediction/utility/test separation, unknown role, wrong/missing version, missing/invalid root, author and citation IVOIDs, optional/partial/duplicate Who curation fields, citation cardinality, missing/unknown cite relation, multiple citations, descriptions, XML character references, duplicate attributes, nested citation markup, DOCTYPE, multiple roots, malformed/incomplete XML, text outside the document root, oversized/non-UTF-8 input and retention of uninterpreted attributes.

Synthetic unit cases are not real astronomical events and are not an IVOA conformance certification.
