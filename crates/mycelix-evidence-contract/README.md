# mycelix-evidence-contract

Pure `#![no_std]` AMSAP-002B companion crate for axis-bound empirical evidence contracts.

This crate preserves the already-qualified AMSAP C/V status and welfare-evidence crates unchanged. It layers a stricter claim envelope above them so evidence cannot be laundered across scientific axes merely because two records share similar strength or confidence descriptions.

The core distinction is:

```text
same evidence shape != same scientific meaning
```

`EmpiricalAxis::{Consciousness, Valence, Responsibility}` is therefore part of immutable claim meaning. One artifact may legitimately support several axis-specific claims, but those claims remain distinct and share a common `DependenceRootId`; three labels over one artifact are not three independent replications.

The contract also separates:

- measurement modality from replication provenance;
- causal intervention claims from observational association;
- preregistered from exploratory/post-hoc work;
- informative evidence from inconclusive tests;
- complete result sets from partial/withheld result sets;
- scientific measurement from known proxy/context variables;
- claim identifiers from opaque content commitments.

Known proxies such as model size, generic benchmarks, reward signals, refusal rates, policy/legal recognition, prior protections, economic value, or copy count may be recorded only as context. They cannot be relabelled as axis-bearing evidence.

Likewise, self-report, external report, and lineage facts can be retained as corroborative/contextual records but cannot independently become axis-bearing evidence in this tranche.

A `QualifiedAxisEvidenceClaim` means only that the claim is structurally coherent under this provenance contract. `SynthesisAdmissibility` indicates whether the record is suitable input to a later synthesis layer; it does not establish consciousness, valence, responsibility, moral patienthood, welfare protection, legal responsibility, liability, legal standing, deployment authority, currentness, governance authority, or external-effect authority.

This tranche intentionally treats `claim_payload_commitment`, experiment manifests, and result-set commitments as opaque commitment references. AMSAP-005C remains responsible for canonical serialization, domain separation, algorithm identifiers, and cryptographic agility.
