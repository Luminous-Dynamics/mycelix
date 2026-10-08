# Cross-observer gossip and split-view evidence research v1

Status: research-only.

This layer addresses a security property that cannot be established by comparing one client's tree heads in isolation: whether independent observers were shown compatible VDS views.

The pipeline is:

    signed witness tree heads
        ->
    independently signed observer observations
        ->
    subject-head authentication
        ->
    cross-observer comparison
        ->
    consistency proof where sizes differ
        ->
    split-view / rollback / consistency decision

Each observer-gossip observation binds:

- observer/monitor identity and key;
- gossip registry/version;
- VDS identity;
- the exact signed subject tree-head digest;
- subject observer identity;
- observed tree size;
- observed root hash;
- observation sequence.

The monitor signs the complete observation. The verifier then resolves the referenced subject head by digest and independently verifies the subject head under the witness registry.

## Decision semantics

    same size + same root
        -> compatible observations

    same size + different root
        -> split-view / equivocation evidence

    older size + newer size + valid consistency proof
        -> compatible append-only views

    newer view followed by smaller view
        -> rollback / unresolved

    larger view without valid consistency proof
        -> unresolved

The two sides of a cross-observer comparison must come from different monitor identities. A monitor cannot manufacture independent evidence by repeating its own observation.

## Attack corpus

The 16-case campaign covers:

- valid 4 -> 7 cross-observer consistency;
- same-size split view using a cryptographically valid fork head;
- same-size identical heads seen by different monitors;
- rollback;
- mutated and missing consistency proofs;
- mutated root;
- domain and key substitution;
- monitor identity substitution;
- extra-field/signature-wrapping mutation;
- subject observer substitution;
- subject-head digest substitution;
- VDS substitution;
- same-monitor and same-observation replay.

## Important boundary

Certificate Transparency describes gossip as a way for clients and other entities to share signed tree-head observations; RFC 9162 explicitly notes that checking consistency of the view presented to all entities is harder because it requires sharing log responses. This research layer turns that conceptual boundary into a deterministic, signed transcript that can be replayed and independently classified.

The layer still does not claim a complete gossip network. It does not prove that all observers receive the same data, that all clients participate, that a malicious monitor cannot omit observations, or that the witness set is organizationally independent.

## Claim ceiling

Research-only.

Demonstrated:

    authenticated subject head
        +
    authenticated observation of that head
        +
    independent monitor comparison
        +
    consistency/split-view classification

Not demonstrated:

    network-wide gossip completeness
    reliable message delivery
    anti-censorship availability
    production monitor discovery
    independent organizational governance
    threshold compromise resistance
    COSE/CBOR wire interoperability

The next protocol-oriented boundary is to bind these transcripts to interoperable COSE receipts/tree-head representations and then test delayed, reordered, partitioned, and eventually convergent observer views.
