//! Test-local, deliberately separate reserve oracle.
//!
//! This model tracks only the state needed for the small generated cases. It
//! does not call ReservationBook helpers; production decisions are compared with
//! the oracle's independently calculated admissibility, revision and capacity.

use std::collections::BTreeMap;

use economic_coordination_reference::reservation_cas::{
    CapacityPoolSpec, ReservationBook, ReserveCommand,
};
use economic_coordination_reference::{EvidenceOrigin, EvidenceStatus};

const POOL_ID: &str = "oracle-pool";
const SNAPSHOT: &str = "oracle-snapshot-v1";

struct Oracle {
    capacity: u64,
    occupied: u64,
    revision: u64,
    fencing_token: u64,
    last_mutation_at: u64,
    reserve_by_key: BTreeMap<String, ReserveCommand>,
    command_ids: BTreeMap<String, ReserveCommand>,
    reservation_ids: BTreeMap<String, u64>,
}

impl Oracle {
    fn new(capacity: u64, snapshot_observed_at: u64) -> Self {
        Self {
            capacity,
            occupied: 0,
            revision: 1,
            fencing_token: 1,
            last_mutation_at: snapshot_observed_at,
            reserve_by_key: BTreeMap::new(),
            command_ids: BTreeMap::new(),
            reservation_ids: BTreeMap::new(),
        }
    }

    fn reserve(&mut self, command: &ReserveCommand, snapshot_valid_until: u64) -> bool {
        if let Some(previous) = self.reserve_by_key.get(&command.idempotency_key) {
            return previous == command;
        }
        if self.command_ids.contains_key(&command.command_id)
            || self.reservation_ids.contains_key(&command.reservation_id)
        {
            return false;
        }
        if command.quantity == 0
            || command.expires_at <= command.issued_at
            || command.expires_at > snapshot_valid_until
            || command.pool_id != POOL_ID
            || command.expected_snapshot_digest != SNAPSHOT
            || command.expected_revision != self.revision
            || command.expected_fencing_token != self.fencing_token
            || command.issued_at < self.last_mutation_at
        {
            return false;
        }
        let Some(next_occupied) = self.occupied.checked_add(command.quantity) else {
            return false;
        };
        if next_occupied > self.capacity {
            return false;
        }
        let Some(next_revision) = self.revision.checked_add(1) else {
            return false;
        };
        let Some(next_fence) = self.fencing_token.checked_add(1) else {
            return false;
        };

        self.occupied = next_occupied;
        self.revision = next_revision;
        self.fencing_token = next_fence;
        self.last_mutation_at = command.issued_at;
        self.reserve_by_key
            .insert(command.idempotency_key.clone(), command.clone());
        self.command_ids
            .insert(command.command_id.clone(), command.clone());
        self.reservation_ids
            .insert(command.reservation_id.clone(), command.quantity);
        true
    }
}

fn pool_spec(capacity: u64) -> CapacityPoolSpec {
    CapacityPoolSpec {
        pool_id: POOL_ID.to_owned(),
        unit: "kit".to_owned(),
        time_bucket: "week-44".to_owned(),
        capacity_quantity: capacity,
        snapshot_digest: SNAPSHOT.to_owned(),
        evidence_status: EvidenceStatus::Verified,
        evidence_origin: EvidenceOrigin::LiveDomain,
        observed_at: 5,
        valid_until: 1_000,
    }
}

fn command(index: usize, quantity: u64, revision: u64, fence: u64) -> ReserveCommand {
    ReserveCommand {
        command_id: format!("command-{index}"),
        idempotency_key: format!("key-{index}"),
        reservation_id: format!("reservation-{index}"),
        request_id: format!("request-{index}"),
        pool_id: POOL_ID.to_owned(),
        expected_revision: revision,
        expected_fencing_token: fence,
        expected_snapshot_digest: SNAPSHOT.to_owned(),
        quantity,
        issued_at: 10,
        expires_at: 50,
        policy_digest: "policy-v1".to_owned(),
        input_manifest_digest: "manifest-v1".to_owned(),
    }
}

#[test]
fn small_generated_reservation_sequences_match_independent_capacity_oracle() {
    // Enumerate 4 * 3^3 = 108 short sequences. For every attempt, the command
    // observes the model's current revision/fence before submission. Quantity
    // zero, exact capacity and over-capacity attempts naturally exercise both
    // non-mutating rejection and successful state transitions.
    let mut case_count = 0usize;
    for capacity in 1..=4 {
        for q0 in 0..=2 {
            for q1 in 0..=2 {
                for q2 in 0..=2 {
                    let quantities = [q0, q1, q2];
                    let mut oracle = Oracle::new(capacity, 5);
                    let mut book = ReservationBook::new();
                    book.register_pool(pool_spec(capacity)).unwrap();

                    for (index, quantity) in quantities.into_iter().enumerate() {
                        let expected = command(
                            index,
                            quantity,
                            oracle.revision,
                            oracle.fencing_token,
                        );
                        let expected_acceptance = oracle.reserve(&expected, 1_000);
                        let actual_acceptance = book.reserve(expected).is_ok();
                        assert_eq!(
                            actual_acceptance, expected_acceptance,
                            "capacity={capacity}, quantities={quantities:?}, step={index}"
                        );

                        let report = book.pool_capacity(POOL_ID).unwrap();
                        assert_eq!(report.capacity_quantity, capacity);
                        assert_eq!(report.reserved_quantity, oracle.occupied);
                        assert_eq!(report.consumed_quantity, 0);
                        assert_eq!(report.remaining_quantity, capacity - oracle.occupied);
                        assert_eq!(report.revision, oracle.revision);
                        assert_eq!(report.fencing_token, oracle.fencing_token);
                    }
                    case_count += 1;
                }
            }
        }
    }
    assert_eq!(case_count, 108);
}
