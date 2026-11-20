//! Test-only reference model separating one-use connection authority,
//! one-use HTTP exchange authority, and bounded connection reuse.
//! This model does not implement a production broker, transport pool, TLS
//! verifier, or packet-level enforcement.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Peer {
    ip_last_octet: u8,
    port: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ConnectionScope {
    workload_id: u64,
    origin: String,
    parent_scope_digest: u64,
    address_set_digest: u64,
    tls_identity_digest: u64,
    policy_generation: u64,
    network_generation: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AuthorityError {
    AlreadyConsumed,
    Expired,
    WorkloadMismatch,
    PeerOutsideAuthorizedSet,
    ConnectTimedOut,
    TlsIdentityMismatch,
    DnsSnapshotExpired,
    ParentScopeExpired,
    ScopeMismatch,
    ConnectionMismatch,
    ConnectionNotActive,
    PolicyGenerationMismatch,
    AddressSetChanged,
    ExchangeBudgetExhausted,
    TooManyInFlightExchanges,
    ExchangeTimedOut,
    DrainDeadlineExceeded,
    WrongConnection,
    NoInFlightExchange,
}

#[derive(Debug)]
struct ConnectCapability {
    id: u64,
    scope: ConnectionScope,
    authorized_peers: Vec<Peer>,
    expires_at_ns: u64,
    connect_timeout_ns: u64,
    consumed: bool,
}

#[derive(Debug)]
struct ConnectGrant {
    capability_id: u64,
    scope: ConnectionScope,
    selected_peer: Peer,
    establishment_deadline_ns: u64,
}

impl ConnectCapability {
    fn consume(
        &mut self,
        now_ns: u64,
        workload_id: u64,
        peer: Peer,
    ) -> Result<ConnectGrant, AuthorityError> {
        if self.consumed {
            return Err(AuthorityError::AlreadyConsumed);
        }
        if now_ns >= self.expires_at_ns {
            return Err(AuthorityError::Expired);
        }

        // The first eligible attempt consumes the capability even when a
        // later peer check fails. A denied attempt cannot be retried with the
        // same token against a different socket.
        self.consumed = true;

        if workload_id != self.scope.workload_id {
            return Err(AuthorityError::WorkloadMismatch);
        }
        if !self.authorized_peers.contains(&peer) {
            return Err(AuthorityError::PeerOutsideAuthorizedSet);
        }

        let establishment_deadline_ns = now_ns
            .saturating_add(self.connect_timeout_ns)
            .min(self.expires_at_ns);

        Ok(ConnectGrant {
            capability_id: self.id,
            scope: self.scope.clone(),
            selected_peer: peer,
            establishment_deadline_ns,
        })
    }
}

impl ConnectGrant {
    fn establish(
        self,
        now_ns: u64,
        observed_tls_identity_digest: u64,
        connection_id: u64,
        dns_valid_until_ns: u64,
        parent_scope_valid_until_ns: u64,
        requested_lease_ns: u64,
        max_exchanges: u32,
        max_in_flight: u32,
    ) -> Result<ConnectionLease, AuthorityError> {
        if now_ns > self.establishment_deadline_ns {
            return Err(AuthorityError::ConnectTimedOut);
        }
        if observed_tls_identity_digest != self.scope.tls_identity_digest {
            return Err(AuthorityError::TlsIdentityMismatch);
        }
        if now_ns >= dns_valid_until_ns {
            return Err(AuthorityError::DnsSnapshotExpired);
        }
        if now_ns >= parent_scope_valid_until_ns {
            return Err(AuthorityError::ParentScopeExpired);
        }

        let expires_at_ns = now_ns
            .saturating_add(requested_lease_ns)
            .min(dns_valid_until_ns)
            .min(parent_scope_valid_until_ns);

        Ok(ConnectionLease {
            id: connection_id,
            source_capability_id: self.capability_id,
            scope: self.scope,
            selected_peer: self.selected_peer,
            expires_at_ns,
            state: LeaseState::Active,
            max_exchanges,
            max_in_flight,
            total_exchanges: 0,
            in_flight: 0,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LeaseState {
    Active,
    Draining { deadline_ns: u64 },
    Revoked,
    Expired,
}

#[derive(Debug)]
struct ConnectionLease {
    id: u64,
    source_capability_id: u64,
    scope: ConnectionScope,
    selected_peer: Peer,
    expires_at_ns: u64,
    state: LeaseState,
    max_exchanges: u32,
    max_in_flight: u32,
    total_exchanges: u32,
    in_flight: u32,
}

#[derive(Debug)]
struct ExchangeCapability {
    id: u64,
    connection_id: u64,
    workload_id: u64,
    request_digest: u64,
    scope: ConnectionScope,
    expires_at_ns: u64,
    consumed: bool,
}

#[derive(Debug)]
struct ExchangePermit {
    connection_id: u64,
    capability_id: u64,
    request_digest: u64,
    deadline_ns: u64,
}

impl ExchangeCapability {
    fn new(
        id: u64,
        connection_id: u64,
        workload_id: u64,
        request_digest: u64,
        scope: ConnectionScope,
        expires_at_ns: u64,
    ) -> Self {
        Self {
            id,
            connection_id,
            workload_id,
            request_digest,
            scope,
            expires_at_ns,
            consumed: false,
        }
    }
}

impl ConnectionLease {
    fn admit_exchange(
        &mut self,
        capability: &mut ExchangeCapability,
        now_ns: u64,
        current_scope: &ConnectionScope,
        current_dns_valid_until_ns: u64,
        current_parent_scope_valid_until_ns: u64,
        exchange_timeout_ns: u64,
    ) -> Result<ExchangePermit, AuthorityError> {
        if capability.consumed {
            return Err(AuthorityError::AlreadyConsumed);
        }
        if now_ns >= capability.expires_at_ns {
            return Err(AuthorityError::Expired);
        }

        // Any attempt to consume a live, connection-bound exchange capability
        // is one-use, including attempts rejected by the lease or scope checks.
        capability.consumed = true;

        if capability.connection_id != self.id {
            return Err(AuthorityError::ConnectionMismatch);
        }
        if capability.workload_id != self.scope.workload_id
            || current_scope.workload_id != self.scope.workload_id
        {
            return Err(AuthorityError::WorkloadMismatch);
        }
        if current_scope.policy_generation != self.scope.policy_generation {
            return Err(AuthorityError::PolicyGenerationMismatch);
        }
        if current_scope.address_set_digest != self.scope.address_set_digest {
            return Err(AuthorityError::AddressSetChanged);
        }
        if &capability.scope != &self.scope || current_scope != &self.scope {
            return Err(AuthorityError::ScopeMismatch);
        }
        if now_ns >= current_dns_valid_until_ns {
            return Err(AuthorityError::DnsSnapshotExpired);
        }
        if now_ns >= current_parent_scope_valid_until_ns {
            return Err(AuthorityError::ParentScopeExpired);
        }

        match self.state {
            LeaseState::Active => {}
            LeaseState::Draining { .. } | LeaseState::Revoked => {
                return Err(AuthorityError::ConnectionNotActive);
            }
            LeaseState::Expired => return Err(AuthorityError::Expired),
        }
        if now_ns >= self.expires_at_ns {
            self.state = LeaseState::Expired;
            return Err(AuthorityError::Expired);
        }
        if self.total_exchanges >= self.max_exchanges {
            return Err(AuthorityError::ExchangeBudgetExhausted);
        }
        if self.in_flight >= self.max_in_flight {
            return Err(AuthorityError::TooManyInFlightExchanges);
        }

        self.total_exchanges += 1;
        self.in_flight += 1;
        Ok(ExchangePermit {
            connection_id: self.id,
            capability_id: capability.id,
            request_digest: capability.request_digest,
            deadline_ns: now_ns.saturating_add(exchange_timeout_ns),
        })
    }

    fn begin_drain(&mut self, deadline_ns: u64) {
        if self.state == LeaseState::Active {
            self.state = LeaseState::Draining { deadline_ns };
        }
    }

    fn revoke(&mut self) {
        self.state = LeaseState::Revoked;
    }

    fn expire_if_due(&mut self, now_ns: u64) {
        if now_ns >= self.expires_at_ns {
            self.state = LeaseState::Expired;
        }
        if let LeaseState::Draining { deadline_ns } = self.state {
            if now_ns >= deadline_ns {
                self.state = LeaseState::Revoked;
            }
        }
    }

    fn finish_exchange(
        &mut self,
        permit: ExchangePermit,
        now_ns: u64,
    ) -> Result<(), AuthorityError> {
        if permit.connection_id != self.id {
            return Err(AuthorityError::WrongConnection);
        }
        if self.in_flight == 0 {
            return Err(AuthorityError::NoInFlightExchange);
        }
        self.in_flight -= 1;

        if now_ns > permit.deadline_ns {
            return Err(AuthorityError::ExchangeTimedOut);
        }
        if let LeaseState::Draining { deadline_ns } = self.state {
            if now_ns > deadline_ns {
                return Err(AuthorityError::DrainDeadlineExceeded);
            }
        }
        Ok(())
    }
}

fn scope() -> ConnectionScope {
    ConnectionScope {
        workload_id: 17,
        origin: "https://api.example.test:443".to_owned(),
        parent_scope_digest: 101,
        address_set_digest: 202,
        tls_identity_digest: 303,
        policy_generation: 4,
        network_generation: 9,
    }
}

fn connect_capability(id: u64, expires_at_ns: u64) -> ConnectCapability {
    ConnectCapability {
        id,
        scope: scope(),
        authorized_peers: vec![
            Peer { ip_last_octet: 10, port: 443 },
            Peer { ip_last_octet: 11, port: 443 },
        ],
        expires_at_ns,
        connect_timeout_ns: 40,
        consumed: false,
    }
}

fn established_lease(max_exchanges: u32, max_in_flight: u32) -> ConnectionLease {
    let mut cap = connect_capability(1, 200);
    let grant = cap.consume(100, 17, Peer { ip_last_octet: 10, port: 443 }).unwrap();
    grant.establish(110, 303, 55, 180, 190, 500, max_exchanges, max_in_flight).unwrap()
}

fn exchange_capability(
    id: u64,
    connection_id: u64,
    request_digest: u64,
    expires_at_ns: u64,
) -> ExchangeCapability {
    ExchangeCapability::new(
        id, connection_id, 17, request_digest, scope(), expires_at_ns,
    )
}

fn admit(
    lease: &mut ConnectionLease,
    capability: &mut ExchangeCapability,
    now_ns: u64,
) -> Result<ExchangePermit, AuthorityError> {
    lease.admit_exchange(capability, now_ns, &scope(), 175, 185, 30)
}

#[test]
fn connect_capability_authorizes_exactly_one_socket_attempt() {
    let mut cap = connect_capability(1, 200);
    let peer = Peer { ip_last_octet: 10, port: 443 };
    assert!(cap.consume(120, 17, peer).is_ok());
    assert_eq!(cap.consume(121, 17, peer).err(), Some(AuthorityError::AlreadyConsumed));
}

#[test]
fn expired_connect_capability_cannot_create_a_socket() {
    let mut cap = connect_capability(1, 200);
    let peer = Peer { ip_last_octet: 10, port: 443 };
    assert_eq!(cap.consume(200, 17, peer).err(), Some(AuthorityError::Expired));
}

#[test]
fn denied_peer_attempt_consumes_capability_and_cannot_be_replayed() {
    let mut cap = connect_capability(1, 200);
    let denied = Peer { ip_last_octet: 99, port: 443 };
    assert_eq!(cap.consume(120, 17, denied).err(), Some(AuthorityError::PeerOutsideAuthorizedSet));
    let allowed = Peer { ip_last_octet: 10, port: 443 };
    assert_eq!(cap.consume(121, 17, allowed).err(), Some(AuthorityError::AlreadyConsumed));
}

#[test]
fn connect_capability_cannot_be_used_by_another_workload() {
    let mut cap = connect_capability(1, 200);
    let peer = Peer { ip_last_octet: 10, port: 443 };
    assert_eq!(cap.consume(120, 18, peer).err(), Some(AuthorityError::WorkloadMismatch));
    assert_eq!(cap.consume(121, 17, peer).err(), Some(AuthorityError::AlreadyConsumed));
}

#[test]
fn tls_identity_mismatch_prevents_connection_lease_issuance() {
    let mut cap = connect_capability(1, 200);
    let grant = cap.consume(120, 17, Peer { ip_last_octet: 10, port: 443 }).unwrap();
    assert_eq!(
        grant.establish(130, 999, 55, 180, 190, 50, 4, 2).err(),
        Some(AuthorityError::TlsIdentityMismatch)
    );
}

#[test]
fn connection_lease_is_clamped_by_dns_and_parent_scope_expiry() {
    let mut cap = connect_capability(1, 200);
    let grant = cap.consume(120, 17, Peer { ip_last_octet: 10, port: 443 }).unwrap();
    let lease = grant.establish(130, 303, 55, 160, 170, 500, 4, 2).unwrap();
    assert_eq!(lease.expires_at_ns, 160);
}

#[test]
fn exchange_capability_is_bound_to_exact_connection_and_is_one_use() {
    let mut lease = established_lease(4, 2);
    let mut cap = exchange_capability(8, 55, 777, 170);
    let permit = admit(&mut lease, &mut cap, 120).unwrap();
    assert_eq!(permit.request_digest, 777);
    assert_eq!(permit.capability_id, 8);
    assert_eq!(lease.finish_exchange(permit, 125), Ok(()));
    assert_eq!(admit(&mut lease, &mut cap, 126).err(), Some(AuthorityError::AlreadyConsumed));
}

#[test]
fn exchange_capability_cannot_be_replayed_on_another_connection() {
    let mut lease = established_lease(4, 2);
    let mut cap = exchange_capability(8, 56, 777, 170);
    assert_eq!(admit(&mut lease, &mut cap, 120).err(), Some(AuthorityError::ConnectionMismatch));
    assert_eq!(admit(&mut lease, &mut cap, 121).err(), Some(AuthorityError::AlreadyConsumed));
}

#[test]
fn changed_dns_address_set_invalidates_connection_reuse() {
    let mut lease = established_lease(4, 2);
    let mut current = scope();
    current.address_set_digest = 999;
    let mut cap = exchange_capability(8, 55, 777, 170);
    assert_eq!(
        lease.admit_exchange(&mut cap, 120, &current, 175, 185, 30).err(),
        Some(AuthorityError::AddressSetChanged)
    );
}

#[test]
fn changed_policy_generation_invalidates_connection_reuse() {
    let mut lease = established_lease(4, 2);
    let mut current = scope();
    current.policy_generation += 1;
    let mut cap = exchange_capability(8, 55, 777, 170);
    assert_eq!(
        lease.admit_exchange(&mut cap, 120, &current, 175, 185, 30).err(),
        Some(AuthorityError::PolicyGenerationMismatch)
    );
}

#[test]
fn cross_origin_http2_coalescing_is_not_implicitly_authorized() {
    let mut lease = established_lease(4, 2);
    let mut current = scope();
    current.origin = "https://other.example.test:443".to_owned();
    let mut cap = ExchangeCapability::new(8, 55, 17, 777, current.clone(), 170);
    assert_eq!(
        lease.admit_exchange(&mut cap, 120, &current, 175, 185, 30).err(),
        Some(AuthorityError::ScopeMismatch)
    );
}

#[test]
fn retry_uses_a_fresh_exchange_capability_after_first_attempt_is_consumed() {
    let mut lease = established_lease(4, 2);
    let mut first = exchange_capability(8, 55, 777, 170);
    let first_permit = admit(&mut lease, &mut first, 120).unwrap();
    // A failed HTTP attempt still consumes its capability; completion releases
    // only the in-flight slot. The retry receives a distinct request capability.
    assert_eq!(lease.finish_exchange(first_permit, 121), Ok(()));
    assert_eq!(admit(&mut lease, &mut first, 122).err(), Some(AuthorityError::AlreadyConsumed));

    let mut retry = exchange_capability(9, 55, 778, 170);
    let retry_permit = admit(&mut lease, &mut retry, 122).unwrap();
    assert_eq!(retry_permit.capability_id, 9);
    assert_eq!(lease.finish_exchange(retry_permit, 123), Ok(()));
}

#[test]
fn concurrent_exchange_budget_is_enforced() {
    let mut lease = established_lease(10, 1);
    let mut first = exchange_capability(8, 55, 777, 170);
    let mut second = exchange_capability(9, 55, 778, 170);
    let first_permit = admit(&mut lease, &mut first, 120).unwrap();
    assert_eq!(
        admit(&mut lease, &mut second, 121).err(),
        Some(AuthorityError::TooManyInFlightExchanges)
    );
    assert_eq!(lease.in_flight, 1);
    assert_eq!(lease.finish_exchange(first_permit, 122), Ok(()));
    let mut third = exchange_capability(10, 55, 779, 170);
    let third_permit = admit(&mut lease, &mut third, 123).unwrap();
    assert_eq!(lease.finish_exchange(third_permit, 124), Ok(()));
}

#[test]
fn exchange_budget_is_bounded_even_when_in_flight_slots_are_released() {
    let mut lease = established_lease(1, 2);
    let mut first = exchange_capability(8, 55, 777, 170);
    let permit = admit(&mut lease, &mut first, 120).unwrap();
    assert_eq!(lease.finish_exchange(permit, 121), Ok(()));
    let mut second = exchange_capability(9, 55, 778, 170);
    assert_eq!(admit(&mut lease, &mut second, 122).err(), Some(AuthorityError::ExchangeBudgetExhausted));
}

#[test]
fn expired_connection_lease_blocks_new_exchange_admission() {
    let mut lease = established_lease(4, 2);
    let mut cap = exchange_capability(8, 55, 777, 170);
    assert_eq!(admit(&mut lease, &mut cap, 180).err(), Some(AuthorityError::Expired));
    assert_eq!(lease.state, LeaseState::Expired);
}

#[test]
fn draining_connection_lease_rejects_new_exchanges_but_allows_bounded_in_flight_completion() {
    let mut lease = established_lease(4, 2);
    let mut first = exchange_capability(8, 55, 777, 170);
    let permit = admit(&mut lease, &mut first, 120).unwrap();
    lease.begin_drain(140);

    let mut second = exchange_capability(9, 55, 778, 170);
    assert_eq!(admit(&mut lease, &mut second, 121).err(), Some(AuthorityError::ConnectionNotActive));
    assert_eq!(lease.finish_exchange(permit, 130), Ok(()));
    assert_eq!(lease.in_flight, 0);
}

#[test]
fn drain_deadline_bounds_existing_exchange_completion() {
    let mut lease = established_lease(4, 2);
    let mut first = exchange_capability(8, 55, 777, 170);
    let permit = admit(&mut lease, &mut first, 120).unwrap();
    lease.begin_drain(125);
    assert_eq!(lease.finish_exchange(permit, 126).err(), Some(AuthorityError::DrainDeadlineExceeded));
    assert_eq!(lease.in_flight, 0);
}

#[test]
fn revocation_blocks_new_exchange_authority() {
    let mut lease = established_lease(4, 2);
    lease.revoke();
    let mut cap = exchange_capability(8, 55, 777, 170);
    assert_eq!(admit(&mut lease, &mut cap, 120).err(), Some(AuthorityError::ConnectionNotActive));
}

#[test]
fn exchange_completion_after_deadline_releases_slot_but_fails_qualification() {
    let mut lease = established_lease(4, 2);
    let mut cap = exchange_capability(8, 55, 777, 170);
    let permit = admit(&mut lease, &mut cap, 120).unwrap();
    assert_eq!(lease.finish_exchange(permit, 151).err(), Some(AuthorityError::ExchangeTimedOut));
    assert_eq!(lease.in_flight, 0);
}
