use finance_route_clearing_optimizer::batch::{
    select_atomic_batch, BatchAccount, BatchContext, BatchPayment, BatchRoute,
};
use finance_route_clearing_optimizer::AuthorityQualification;
use std::collections::BTreeMap;

fn context() -> BatchContext {
    BatchContext {
        rail_id: "oracle-rail".into(),
        asset_id: "oracle-asset".into(),
        netting_set_id: "oracle-set".into(),
        policy_version: "batch-oracle-v1".into(),
        tick: 0,
        authority: AuthorityQualification::ExternallyQualified {
            evidence_ref: "fixture-only://not-legal-evidence".into(),
        },
    }
}

fn account(id: &str, balance_minor: u64) -> BatchAccount {
    BatchAccount {
        participant_id: id.into(),
        balance_minor,
    }
}

fn route(
    id: &str,
    capacity_minor: u64,
    fixed_fee_units: u64,
    fee_units_per_minor: u64,
) -> BatchRoute {
    BatchRoute {
        route_id: id.into(),
        rail_id: "oracle-rail".into(),
        asset_id: "oracle-asset".into(),
        netting_set_id: "oracle-set".into(),
        policy_version: "batch-oracle-v1".into(),
        enabled: true,
        remaining_capacity_minor: capacity_minor,
        available_now: true,
        fixed_fee_units,
        fee_units_per_minor,
    }
}

fn payment(id: &str, route_id: &str, from: &str, to: &str, amount_minor: u64) -> BatchPayment {
    BatchPayment {
        payment_id: id.into(),
        rail_id: "oracle-rail".into(),
        asset_id: "oracle-asset".into(),
        netting_set_id: "oracle-set".into(),
        policy_version: "batch-oracle-v1".into(),
        route_id: route_id.into(),
        from: from.into(),
        to: to.into(),
        amount_minor,
        arrival_tick: 0,
        deadline_tick: None,
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct OraclePlan {
    gross_minor: u128,
    fee_units: u128,
    count: usize,
    ids: Vec<String>,
}

/// Independent tiny-set oracle. It enumerates selected payment sets, recomputes net
/// post-batch balances and route use without calling the production selector, then
/// compares the declared lexicographic objective: gross value, fee, count, sorted IDs.
fn oracle(
    accounts: &[BatchAccount],
    routes: &[BatchRoute],
    payments: &[BatchPayment],
) -> OraclePlan {
    let account_balances = accounts
        .iter()
        .map(|a| (a.participant_id.as_str(), a.balance_minor as u128))
        .collect::<BTreeMap<_, _>>();
    let route_by_id = routes
        .iter()
        .map(|r| (r.route_id.as_str(), r))
        .collect::<BTreeMap<_, _>>();
    let mut best = OraclePlan {
        gross_minor: 0,
        fee_units: 0,
        count: 0,
        ids: Vec::new(),
    };

    for mask in 0_u64..(1_u64 << payments.len()) {
        let mut outgoing = BTreeMap::<&str, u128>::new();
        let mut incoming = BTreeMap::<&str, u128>::new();
        let mut route_used = BTreeMap::<&str, u128>::new();
        let mut gross_minor = 0_u128;
        let mut fee_units = 0_u128;
        let mut ids = Vec::new();

        for (index, payment) in payments.iter().enumerate() {
            if mask & (1_u64 << index) == 0 {
                continue;
            }
            let route = route_by_id[payment.route_id.as_str()];
            let amount = payment.amount_minor as u128;
            *outgoing.entry(payment.from.as_str()).or_default() += amount;
            *incoming.entry(payment.to.as_str()).or_default() += amount;
            *route_used.entry(payment.route_id.as_str()).or_default() += amount;
            gross_minor += amount;
            fee_units += route.fixed_fee_units as u128 + amount * route.fee_units_per_minor as u128;
            ids.push(payment.payment_id.clone());
        }
        ids.sort();

        let route_capacity_ok = route_used
            .iter()
            .all(|(id, amount)| *amount <= route_by_id[id].remaining_capacity_minor as u128);
        if !route_capacity_ok {
            continue;
        }

        let balances_ok = account_balances.iter().all(|(participant, starting)| {
            let received = incoming.get(participant).copied().unwrap_or(0);
            let sent = outgoing.get(participant).copied().unwrap_or(0);
            sent <= *starting + received
        });
        if !balances_ok {
            continue;
        }

        let candidate = OraclePlan {
            gross_minor,
            fee_units,
            count: ids.len(),
            ids,
        };
        let better = candidate.gross_minor > best.gross_minor
            || (candidate.gross_minor == best.gross_minor && candidate.fee_units < best.fee_units)
            || (candidate.gross_minor == best.gross_minor
                && candidate.fee_units == best.fee_units
                && candidate.count > best.count)
            || (candidate.gross_minor == best.gross_minor
                && candidate.fee_units == best.fee_units
                && candidate.count == best.count
                && candidate.ids < best.ids);
        if better {
            best = candidate;
        }
    }
    best
}

#[test]
fn atomic_batch_selector_matches_exhaustive_oracle_on_64_small_graphs() {
    for seed in 0..64_usize {
        let accounts = [
            account("A", ((seed * 3) % 5) as u64),
            account("B", ((seed * 5 + 1) % 5) as u64),
            account("C", ((seed * 7 + 2) % 5) as u64),
        ];
        let amounts = [
            ((seed * 3 + 1) % 3 + 1) as u64,
            ((seed * 5 + 2) % 3 + 1) as u64,
            ((seed * 7 + 1) % 3 + 1) as u64,
            ((seed * 11 + 2) % 3 + 1) as u64,
        ];
        let routes = [
            route("r-ab", amounts[0], ((seed * 3 + 1) % 7) as u64, 1),
            route("r-bc", amounts[1], ((seed * 5 + 2) % 7) as u64, 2),
            route("r-ca", amounts[2], ((seed * 7 + 3) % 7) as u64, 3),
            route("r-ac", amounts[3], ((seed * 11 + 4) % 7) as u64, 1),
        ];
        let payments = [
            payment("p-ab", "r-ab", "A", "B", amounts[0]),
            payment("p-bc", "r-bc", "B", "C", amounts[1]),
            payment("p-ca", "r-ca", "C", "A", amounts[2]),
            payment("p-ac", "r-ac", "A", "C", amounts[3]),
        ];

        let expected = oracle(&accounts, &routes, &payments);
        let actual = select_atomic_batch(&context(), &accounts, &routes, &payments)
            .unwrap_or_else(|error| panic!("seed={seed}: {error}"));

        assert_eq!(
            actual.selected_gross_minor, expected.gross_minor,
            "gross-value mismatch, seed={seed}"
        );
        assert_eq!(
            actual.modeled_fee_units, expected.fee_units,
            "fee mismatch, seed={seed}"
        );
        assert_eq!(
            actual.selected_payment_count, expected.count,
            "payment-count mismatch, seed={seed}"
        );
        assert_eq!(
            actual.selected_payment_ids, expected.ids,
            "selected-set mismatch, seed={seed}"
        );
    }
}
