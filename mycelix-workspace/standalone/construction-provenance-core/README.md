# Construction Provenance Core v0.1

A dependency-free Rust reference verifier for Mycelix's narrow responsibility in
the infrastructure lifecycle: participant authority/commitments, evidence-use
assertions, material certification references, batch custody, and manufacturing
and inspection attestations.

It composes with—not replaces—the source-owned engineering and asset graphs.

## Run

\`\`\`sh
cargo fmt --manifest-path mycelix-workspace/standalone/construction-provenance-core/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-workspace/standalone/construction-provenance-core/Cargo.toml
cargo clippy --locked --manifest-path mycelix-workspace/standalone/construction-provenance-core/Cargo.toml --all-targets -- -D warnings
python3 mycelix-workspace/standalone/construction-provenance-core/tests/validate_fixture.py
\`\`\`

## Ownership boundaries

- **Sol-Atlas** owns site context, portfolio alternatives, the built-environment
  artifact/revision graph, engineering disciplines and engineering approval
  validation. Existing owner: PR #138.
- **Symthaea** owns design-intent identity and source-side fabrication artifacts.
  Existing owners include ROB-DESIGN-001A (#4806) and the Fabrication Kernel.
- **Symtropy** owns physical/resource simulation runs and their technical
  qualification; Mycelix records opaque run references, not solver conclusions.
- **Mycelix** owns identity/authority resolution, participant commitments,
  evidence/provenance governance, correction/revocation/dispute state, material
  batch custody, and cross-party manufacturing/inspection attestations.

The Mycelix verifier checks only supplied owner/role/revision/digest references
and exact subject links between them. It does not fetch source bytes, recompute
source artifact hashes, verify signatures, parse IFC, replay simulations, or
validate the engineering adequacy of a Sol-Atlas receipt.

## Executable lifecycle trace

The synthetic pavilion fixture composes:

\`Sol-Atlas design revision\`
→ \`Symthaea design/IFC/drawing references\`
→ \`Symtropy run + Sol-Atlas engineering receipt\`
→ \`Mycelix participant authority/commitment\`
→ \`material batch + scoped certificate\`
→ \`predecessor-linked custody transfers\`
→ \`manufacturer attestation + fabricated component\`
→ \`independent inspection + installation + as-built references\`.

Every opaque artifact pointer carries a source owner, role, revision, digest and
optional exact subject artifact. Supporting engineering receipts must point to
the design named by the participant commitment. Manufacturing records must
point to the exact design and component; inspection, installation and as-built
records must all point to the same manufactured component.

Evidence and authority assertions carry both domain-valid time and
\`available_at\`. Evidence first available after a decision cutoff cannot justify
that earlier decision. Revoked evidence is rejected, disputed evidence remains
disputed, and unknown, stale or unavailable evidence remains indeterminate.
Supplier claims are distinct from independently resolved certification evidence.

The verifier emits per-record Accepted / Rejected / Indeterminate / Disputed /
Superseded findings. It does not collapse these into a project-level
\`ready\`, \`safe\`, \`compliant\` or \`approved\` boolean.

## Claim ceiling

A valid Mycelix receipt proves only that the narrow participant, authority,
evidence-use and custody contract passed with the supplied external references.
It does not prove that the source artifacts are authentic or technically sound,
that an issuer is competent, that a permit exists, or that a component/asset is
safe or fit for service. A claimed conforming inspection remains an attestation,
not a professional certification or statutory acceptance.

## Wire format

The versioned JSON Schema and synthetic fixture are under \`fixtures/\`. The
standard-library Python checker validates a deliberately limited JSON Schema
keyword subset and adversarial cross-record relationships. It is test tooling,
not a production Python dependency or a complete JSON Schema implementation.
