# mycelix-hardware-interop

Loss-aware interoperability adapters for the Mycelix open-hardware semantic core.

The internal Mycelix model remains canonical. External schemas are treated as adapters, not authorities. Every conversion produces an `InteropReport` describing what was preserved, approximated, omitted, unsupported, or left unknown.

Initial targets:

- Open Know-How 2.4 project metadata;
- CycloneDX 1.7 hardware-component subset;
- SPDX 3.1-dev hardware work only behind the explicit `experimental-spdx-3-1-dev` feature.

Normative rules:

- adapters never invent factual values merely to satisfy a target schema;
- unknown source values stay unknown;
- caller-supplied external context is explicit and separate from canonical Mycelix fields;
- unsupported source data is preserved opaquely where practical;
- round-tripping must never strengthen engineering, safety, openness, or certification claims.
