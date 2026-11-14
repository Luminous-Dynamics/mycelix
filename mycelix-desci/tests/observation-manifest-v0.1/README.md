# Mycelix Observation Manifest v0.1

This directory contains the first cross-domain fixture corpus for the source-preserving observation-manifest envelope. The normative schema is:

\`mycelix-desci/schemas/observation-manifest-v0.1.schema.json\`

## Why the envelope is deliberately small

The manifest gives an observation a portable identity, source assertion, temporal context, instrument/calibration context, immutable source-artifact reference, domain-profile reference, quality state, rights disposition and correction lineage.

It does **not** flatten domain-specific measurement semantics into one universal geometry or scalar:

- Astronomy retains VOEvent packet identity/role, celestial frame, time scale, STC coordinates and publisher-hypothesis semantics in the astronomy profile.
- Ocean observations retain ERDDAP/OOI variable, timebase, depth/pressure/vertical datum, units, deployment identity and source QC semantics in the ocean profile.
- Earth-observation imagery retains STAC Collection/Item/Asset identity, geometry, projection, platform/instrument, acquisition and processing baseline, band and quality-mask semantics in the satellite profile.

The three times are deliberately distinct:

1. \`phenomenon.event_time\`: when the physical event or measurement occurred;
2. \`source.published_at\`: source-reported record publication time, when supplied;
3. \`received_at\`: time observed at the Mycelix/fabric boundary.

No conversion between source time scales, coordinate frames or units is implied by the common envelope. Such conversions belong to versioned domain transforms with explicit uncertainty and loss accounting.

## Synthetic fixtures

The positive fixtures are intentionally synthetic:
- \`fixtures/astronomy-voevent-v2-1.synthetic.json\`
- \`fixtures/ocean-ooi-erddap.synthetic.json\`
- \`fixtures/earth-observation-stac-1-1.synthetic.json\`

Each references an adjacent synthetic payload. The fixture media type is \`text/plain\` deliberately: these are byte-integrity/conformance fixtures, not real VOEvent XML, ERDDAP CSV or STAC Item documents.

## Validate

Development-only Python validation uses JSON Schema Draft 2020-12:

\`\`\`bash
python3 -m pip install jsonschema==4.26.0
python3 mycelix-desci/tests/observation-manifest-v0.1/validate_manifests.py
\`\`\`

The validator:
- validates the schema itself;
- validates all three positive manifests;
- verifies the exact payload byte length, SHA-256 digest and \`urn:sha256\` locator;
- rejects five negative controls (wrong profile, unknown field, malformed digest, empty permitted purpose and missing fabric receipt time);
- verifies observation IDs are unique across the frozen fixture set.

Expected result:

\`\`\`text
SUMMARY positive=3 negative=5 schema=draft2020-12 payload-digests=verified
\`\`\`

The validator is a test utility, not a production ingestion path, signature verifier, rights engine or scientific evaluator.

## Claim ceiling

These fixtures establish only that the proposed JSON contract and its sample byte references are structurally consistent. They do not establish source authenticity, real dataset rights, calibration validity, sensor qualification, physical observation, scientific truth, instrument authority or production readiness.