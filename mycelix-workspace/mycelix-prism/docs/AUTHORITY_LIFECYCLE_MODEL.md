# Prism authority lifecycle reference model v0.1

**Status:** dependency-free, test-only Rust state-machine model. It is not a production capability broker, HTTP connection pool, TLS verifier, BPF policy, or runtime enforcement claim.

## 1. Paradigm: authorize actions, not clients

A normal HTTP client has a reusable client object and often a reusable connection pool. A restrictive URL filter adds policy checks around requests, but does not by itself explain whether a new socket, a reused HTTP/2 stream, a retry, or a redirected request has fresh authority.

Prism should model three separate authorities:

1. **ConnectCapability** — one new socket attempt to one authorized peer in a bounded address set, for one workload, origin, TLS identity and policy generation.
2. **ConnectionLease** — a verified connection whose reuse is bounded by origin, workload, parent scope, address-set digest, TLS identity, policy generation, network generation, DNS/parent expiry, total exchange budget and maximum in-flight exchanges.
3. **ExchangeCapability** — one exact HTTP request attempt on one exact connection lease, with its own opaque request digest, expiry and one-use lifecycle.

A redirect or retry always requires a new ExchangeCapability. If it needs a different origin, peer set, policy generation, parent scope, TLS identity or network-enforcement generation, it additionally requires a new ConnectCapability and connection. This is intentionally stricter than assuming that possession of a pooled socket grants authority for every future HTTP request.

## 2. Lifecycle contracts

### ConnectCapability

- Expires before a socket attempt: deny.
- Can be consumed once only.
- A foreign workload cannot consume or burn another workload's capability.
- After the owning workload attempts a connection, the capability is consumed before peer evaluation; a denied peer attempt cannot be replayed against another peer.
- The selected peer must be a member of the authorized IP:port set.
- TLS identity must match before a ConnectionLease is created.
- Establishment must complete before the bounded connect deadline.
- A failed TLS handshake produces no reusable lease.

### ConnectionLease

A lease binds these dimensions:

- workload identity;
- exact scheme/host/port origin (no implicit HTTP/2 cross-origin coalescing);
- owner-authorized parent address-scope digest;
- exact DNS address-set digest;
- verified TLS identity digest;
- application policy generation;
- network enforcement generation.

Lease expiry is clamped to the earliest of the requested connection lifetime, DNS snapshot validity and parent-scope validity. A DNS address-set change, TLS identity change, parent-scope change, policy generation change or network generation change invalidates reuse. The model does not silently rebind an existing connection to new DNS answers.

The lease has independent total-exchange and concurrent-in-flight budgets. Releasing an in-flight slot does not replenish the total exchange budget.

### ExchangeCapability

- Binds to one connection ID and one exact ConnectionScope.
- Binds an opaque digest of the exact request representation (method, target, authority, headers and body should be canonicalized by the issuer).
- Is consumed once. A failed or cancelled attempt does not make it reusable.
- A retry must receive a new capability and new attempt identity.
- Every redirect target gets a fresh exchange authorization. A changed connection scope requires a new connection capability as well.
- A foreign workload cannot burn another workload's exchange capability. Same-workload misuse, such as using it on the wrong connection or under a stale policy, consumes it and denies the attempt.

## 3. Expiry, revocation and in-flight work

The model separates four states:

- **Active:** new exchanges may be admitted inside every current scope and budget.
- **Draining:** no new exchanges; already admitted work may finish before both its request deadline and the drain deadline.
- **Expired:** no new exchanges; already admitted work may finish only within its own request deadline.
- **Revoked:** no new exchanges, and outstanding application exchange permits fail qualification and release their in-flight accounting slot.

Revoking an application exchange permit is not proof that packets on an established socket have stopped. Packet-level enforcement and descriptor-passed socket behavior belong to Nixward's separate cgroup/packet-egress qualification. The model intentionally makes no claim about kernel termination of an existing TCP or UDP flow.

## 4. Executable tests

The dependency-free integration test `prism-egress/tests/authority_lifecycle_model.rs` contains 30 deterministic tests. It covers:

- single-use connect authority, expired grants, cross-workload attempts and denied-peer token consumption;
- TLS identity mismatch and DNS/parent expiry;
- one-use exchange authority, connection binding and cross-connection replay;
- DNS address-set, TLS identity, policy-generation and network-generation invalidation;
- implicit cross-origin HTTP/2 coalescing rejection;
- fresh authority for retries;
- maximum concurrent and total exchange budgets;
- explicit expiry, drain, revoke and exact deadline transitions;
- generated replay sequences across 128 distinct capability IDs.

These cases are model-level source coverage only. They do not establish that the current Reqwest integration has these semantics or that a production connection pool exists.

## 5. Implementation path

Do not immediately add a production pool. Prism currently creates a fresh DNS-pinned Reqwest client per consumed EgressCapability, which is conservative and avoids cross-request pool reuse. The reference model establishes the authority contract to preserve before any pooling optimization.

Next, build a transport adapter experiment with:
1. A fake connector/pool that accepts only an exact ConnectionScope key.
2. A loopback HTTP/1 fixture for sequential exchanges.
3. An HTTP/2 fixture for concurrent streams and cross-origin coalescing attempts.
4. DNS answer refresh where the address-set digest is unchanged, then changed.
5. Policy and Nixward network-generation rollover.
6. Retries, redirects, cancellation and drain under load.
7. Independent socket identity observation for every new connection.

Only after semantic equivalence is established should PEAB measure the cost and reuse benefit versus fresh-client Reqwest, a safely pooled Reqwest reference, Envoy DFP, and Envoy+Cilium.

## 6. Falsifiable research hypotheses

- **H1 — authority-scoped reuse:** exact-scope leases can preserve safe connection reuse without allowing request authority to leak across origins, workloads, policy generations or DNS address sets.
- **H2 — amortized overhead:** a qualified pool can reduce TLS handshakes and warm p95 latency while adding bounded admission and reconciliation cost.
- **H3 — bounded revocation:** expiry can stop new exchange admission immediately while recording bounded in-flight drain separately from kernel packet termination.
- **H4 — explainable retries:** each retry and redirect can be attributed to a distinct one-use exchange capability, while transport retries remain disabled.
- **H5 — no confused deputy:** proxy reachability does not grant an untrusted caller authority to use that reachability; caller identity and destination authority are evaluated independently.

All are hypotheses. No comparative measurements or production superiority claims are present in this artifact.

## 7. Industry comparison extension: confused-deputy resistance

Envoy's v1.40.0 Dynamic Forward Proxy documentation explicitly warns that an untrusted client can use the proxy's network reachability to attempt access to localhost, link-local, cloud metadata and private-network destinations. It recommends firewalls, default-deny RBAC and container/kernel networking restrictions. PEAB adds ENVOY-004 to test caller identity and target authorization separately from the proxy's global ability to reach an address. Stock DFP and hardened DFP profiles must be measured separately; this is not a claim that every Envoy deployment is vulnerable.

Cilium's latest FQDN debugging documentation distinguishes per-endpoint DNS cache from global policy aggregation. PEAB's endpoint-crossing tests must keep principal/workload identity and DNS-observation provenance separate, rather than treating an IP's presence in a node-wide policy cache as proof that every endpoint independently authorized that hostname.
