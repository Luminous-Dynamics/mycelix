# Prism Runtime: Rust-Native Engine Architecture

Status: research architecture, not a security qualification.

Issue: #3738 (PRISM-ENGINE-001)

## 1. Decision

Prism should develop toward a Rust-native browser/runtime stack while retaining a mature-web compatibility path during the transition.

This is not a proposal to reimplement Chromium or Gecko immediately. It is a proposal to make the Prism security boundary independent of the host webview and progressively move rendering authority into Prism-owned processes.

The current repository already contains the beginnings of this path:

- `prism-dom`
- `prism-layout`
- `prism-shell`
- `prism-net`
- `prism-privacy`
- `prism-reflex`
- `prism-bridge`

The architectural objective is therefore **convergence**, not a second browser project beside Prism.

## 2. Threat model

A web page is hostile computation.

The renderer must be assumed capable of:

- malformed HTML/CSS/media/font input;
- script-level abuse when scripting is enabled;
- resource exhaustion;
- navigation confusion;
- origin confusion;
- attempts to reach privileged APIs;
- attempts to exploit parser/layout/decoder bugs;
- attempts to exploit IPC;
- attempts to exploit GPU or platform integration;
- attempts to induce privacy leakage.

Rust provides memory-safety and data-race guarantees, but it does not establish the browser security boundary by itself.

The security boundary is therefore:

**untrusted web content -> isolated renderer -> typed IPC -> privileged broker -> explicitly authorized capability**

## 3. Process architecture

The target architecture is:

    prism-browser
        |
        +-- prism-security-broker
        |     +-- origin policy
        |     +-- capability policy
        |     +-- navigation policy
        |     +-- permission policy
        |     +-- evidence authority
        |
        +-- prism-renderer-N
        |     +-- DOM
        |     +-- style
        |     +-- layout
        |     +-- script/WASM (later)
        |
        +-- prism-network
        |     +-- WEB target admission
        |     +-- DNS observation
        |     +-- HTTP/TLS
        |     +-- capture/evidence
        |
        +-- prism-gpu
        |     +-- compositor
        |     +-- rasterization
        |
        +-- prism-utility
              +-- image/font/media/data decoding

The browser broker is the authority. Renderer processes are not authorities.

Process boundaries are security boundaries, not merely performance boundaries.

## 4. Capability model

Renderer code must not receive ambient access to:

- arbitrary sockets;
- filesystem paths;
- environment variables;
- process creation;
- native devices;
- unrestricted IPC;
- credential stores;
- browser profile databases.

Operations cross a typed broker interface.

Conceptually:

    RendererRequest
      -> capability check
      -> origin check
      -> user/privacy policy
      -> deterministic security policy
      -> operation
      -> evidence/receipt where applicable
      -> RendererResponse

A capability should encode the narrow operation it authorizes. A generic "browser privilege" is deliberately avoided.

## 5. Navigation is a security transition

Navigation is not simply loading another URL.

It changes the security context.

The broker must model at least:

    supplied locator
      -> parsed locator
      -> admitted target
      -> network observation
      -> retrieved artifact
      -> origin
      -> document
      -> renderer assignment

A navigation cannot silently carry privileged state across an origin transition.

This aligns with the existing WEB-LOCATOR / WEB-DNS / WEB-CAPTURE direction.

## 6. Origin isolation

Before JavaScript is introduced, Prism should establish explicit identities for:

- scheme;
- host;
- effective port;
- origin;
- site;
- agent cluster;
- renderer assignment.

The renderer process assignment must be derived from these identities, not from UI tab identity alone.

A compromised renderer must not gain access to another origin merely because both pages share a browser tab, profile, or UI process.

## 7. Network authority

The renderer never owns the network.

The intended path is:

    Renderer
      -> NetworkCapability
      -> security broker
      -> qualified WEB target admission
      -> qualified WEB-DNS observation
      -> transport
      -> WEB-CAPTURE
      -> RetrievalEvidence
      -> renderer projection

`prism-net::SafeFetchClient` remains a containment bridge until the qualified WEB primitives are available.

It must not become a second permanent network/provenance ontology.

## 8. Evidence authority

Network success is not source truth.

The system should distinguish:

- transport observation;
- retrieved bytes;
- artifact identity;
- document identity;
- extracted claims;
- external intelligence;
- epistemic assessment;
- security assessment.

The renderer may consume projections of evidence but must not manufacture evidence authority.

Security UI should eventually be able to explain a decision from an inspectable receipt.

## 9. OSINT, OPSEC, and Symthaea

These systems sit above the renderer.

### WEB

Answers:

> What happened on the network?

### OPSEC

Answers:

> What information is the user exposing by performing this operation?

### OSINT

Answers:

> What independently observable external information exists about this infrastructure or artifact?

### EPISTEMICS

Answers:

> What claims are justified by the available observations?

### Symthaea

Answers:

> What patterns or anomalies may deserve attention?

Symthaea remains advisory. It cannot grant a capability, override a deterministic deny, or create evidence.

## 10. Rendering architecture

The first native renderer should remain deliberately small.

Initial qualification target:

1. byte input;
2. HTML tokenization;
3. DOM construction;
4. deterministic style parsing;
5. layout tree;
6. text/image display;
7. compositor output.

JavaScript is explicitly later.

This allows the project to establish parser, layout, resource-limit, origin, and IPC invariants before introducing a general-purpose scripting runtime.

## 11. Script and WASM boundary

When scripting is eventually added:

- script execution belongs in a renderer process;
- JavaScript engine memory is renderer-owned;
- host capabilities are broker-mediated;
- Web APIs are capability surfaces;
- WASM does not gain additional authority merely because it is native-like;
- JIT is an explicit security decision.

"No JIT" can remain a legitimate hardened profile, but it must not be confused with the universal architecture.

## 12. Utility-process isolation

Complex decoders should not be assumed safe merely because they are written in Rust.

Images, fonts, media, compression, archives, and other complex formats should have resource limits and, where practical, dedicated utility processes.

A decoder exploit should not automatically become browser-broker authority.

## 13. Qualification

Prism engine compatibility must be measured, not asserted.

The qualification stack should eventually include:

- unit/property tests;
- parser fuzzing;
- differential tests;
- WPT-derived conformance;
- hostile-document corpus;
- process-isolation tests;
- IPC abuse tests;
- capability-confusion tests;
- origin-isolation tests;
- resource-exhaustion tests;
- security regression corpus.

WPT is the interoperability reference; local examples are not sufficient evidence of web compatibility.

## 14. Compatibility strategy

The project should maintain two explicit tracks:

### Prism Native

The security-first Rust runtime described here.

### Prism Compatibility

A mature engine/webview integration used for applications that require broader web compatibility before Prism Native reaches equivalent coverage.

The compatibility track must not silently become the permanent trusted architecture.

## 15. Migration invariant

At no point should replacing a system webview with Prism Native reduce an existing security property.

For each migrated capability:

    old property
      -> explicit Prism invariant
      -> implementation
      -> adversarial test
      -> qualification receipt
      -> migration

This makes migration evidence-driven rather than aspirational.

## 16. Near-term implementation order

1. Define typed browser/renderer capability contracts.
2. Define origin/site/agent-cluster identity types.
3. Separate renderer state from broker state.
4. Harden `prism-dom` parser/resource limits.
5. Make `prism-layout` explicitly renderer-owned.
6. Move network operations behind the broker interface.
7. Add a minimal renderer process boundary.
8. Add hostile-document and IPC security tests.
9. Integrate qualified WEB primitives.
10. Begin WPT-derived qualification.
11. Only then introduce scripting/WASM.

## 17. Relationship to Servo

Servo is an important external reference and potential upstream ecosystem, not an architectural dependency.

Prism should reuse mature Rust ecosystem components where doing so preserves its security invariants.

The decision to reuse a component should be based on:

- security boundary clarity;
- auditability;
- maintenance health;
- licensing;
- standards conformance;
- ability to impose Prism capability policy.

Prism should not fork an engine merely for the sake of owning code.

## 18. Long-term property

The goal is not:

> a browser written in Rust.

The goal is:

> a browser runtime where hostile computation, network authority, user capabilities, privacy exposure, evidence, and security decisions are explicit architectural objects.

That is the property that makes a Prism-native engine worth building.
