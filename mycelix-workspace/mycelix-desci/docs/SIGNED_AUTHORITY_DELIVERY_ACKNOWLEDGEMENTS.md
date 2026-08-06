# Signed Authority Delivery Acknowledgements

A signed outbox envelope proves that Mycelix-DeSci emitted a publication. A
delivery acknowledgement is separate evidence from an independently governed
witness that the immutable publication was durably retained after it was
published.

## Bound fields

The witness signs:

- acknowledgement ID;
- delivery ID and signed delivery hash;
- exact topic;
- witness actor and organization;
- acknowledgement time; and
- immutable durable HTTPS reference.

The durable reference must contain the lower-case delivery hash in its path and
must not contain user credentials, a query string, or fragment. A mutable
`latest` pointer does not qualify.

## Acceptance checks

The authority service verifies:

- the witness signature;
- active governed witness authority at acknowledgement time;
- organization binding;
- exact outbox topic and signed delivery hash;
- the outbox envelope and all indexed delivery metadata;
- that publication completed before acknowledgement; and
- uniqueness per delivery and witness actor.

Identical retries return the original acknowledgement record. Reusing an
acknowledgement ID or witness slot for different signed bytes fails closed.

## Quorum semantics

Readiness counts distinct currently valid organizations, not accounts or raw
signatures. A governed witness-compromise interval can invalidate only evidence
signed within the affected historical interval.

Acknowledgements prove retention claims by witnesses; they do not prove that a
remote object remains reachable forever. Independent auditors should retrieve
the referenced bytes, verify the signed delivery envelope, and recompute the
delivery hash.
