use finance_liquidity_reordering::{
    bounded_insertion_search, exact_pareto_frontier, exact_permutation_oracle,
    heuristic_portfolio, input_order, receiver_ready_greedy, simulated_annealing, Payment, Problem,
    ReorderError, SearchResult,
};
use std::collections::BTreeSet;
use std::error::Error;
use std::time::Instant;

const WARMUPS: usize = 2;
const REPETITIONS: usize = 21;
const ANNEALING_SEEDS: [u64; 3] = [17, 101, 997];

#[derive(Clone, Copy)]
enum Scenario {
    ReversedEqualChain,
    ReversedHeterogeneousChain,
    ReversedCycle,
    HubRoundTrips,
    PrecedenceConstrainedChain,
}

impl Scenario {
    const ALL: [Self; 5] = [
        Self::ReversedEqualChain,
        Self::ReversedHeterogeneousChain,
        Self::ReversedCycle,
        Self::HubRoundTrips,
        Self::PrecedenceConstrainedChain,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::ReversedEqualChain => "reversed_equal_chain",
            Self::ReversedHeterogeneousChain => "reversed_heterogeneous_chain",
            Self::ReversedCycle => "reversed_cycle",
            Self::HubRoundTrips => "hub_round_trips",
            Self::PrecedenceConstrainedChain => "precedence_constrained_chain",
        }
    }
}

fn make_problem(length: usize, scenario: Scenario) -> Result<Problem, ReorderError> {
    let (participant_count, payments) = match scenario {
        Scenario::ReversedEqualChain | Scenario::PrecedenceConstrainedChain => {
            let payments = (0..length)
                .rev()
                .map(|i| Payment {
                    id: format!("p{i:04}"),
                    from: i,
                    to: i + 1,
                    amount_minor: 10,
                })
                .collect();
            (length + 1, payments)
        }
        Scenario::ReversedHeterogeneousChain => {
            let payments = (0..length)
                .rev()
                .map(|i| Payment {
                    id: format!("p{i:04}"),
                    from: i,
                    to: i + 1,
                    amount_minor: (length - i + 10) as u64,
                })
                .collect();
            (length + 1, payments)
        }
        Scenario::ReversedCycle => {
            let payments = (0..length)
                .rev()
                .map(|i| Payment {
                    id: format!("p{i:04}"),
                    from: i,
                    to: (i + 1) % length,
                    amount_minor: 10,
                })
                .collect();
            (length, payments)
        }
        Scenario::HubRoundTrips => {
            // All benchmark sizes are even. Return legs precede outbound legs in the
            // intentionally hostile input order, making fan-in/fan-out liquidity visible.
            debug_assert_eq!(length % 2, 0);
            let pairs = length / 2;
            let mut payments = Vec::with_capacity(length);
            for participant in (1..=pairs).rev() {
                payments.push(Payment {
                    id: format!("r{participant:04}"),
                    from: participant,
                    to: 0,
                    amount_minor: 10,
                });
            }
            for participant in (1..=pairs).rev() {
                payments.push(Payment {
                    id: format!("o{participant:04}"),
                    from: 0,
                    to: participant,
                    amount_minor: 10,
                });
            }
            (pairs + 1, payments)
        }
    };
    let problem = Problem::new(participant_count, payments)?;
    if matches!(scenario, Scenario::PrecedenceConstrainedChain) {
        let precedence = (0..length.saturating_sub(1))
            .map(|index| (length - 1 - index, length - 2 - index))
            .collect();
        problem.with_precedence_constraints(precedence)
    } else {
        Ok(problem)
    }
}

#[derive(Clone, Copy, Debug)]
struct TopologyFeatures {
    participants_both_send_receive: usize,
    unique_receivers: usize,
    unique_directed_edges: usize,
}

/// Static workload features motivated by the 2026 TARGET2 paper's finding that
/// optimizability varies with sender/receiver overlap and the diversity of receivers.
fn topology_features(problem: &Problem) -> TopologyFeatures {
    let mut sends = vec![false; problem.participant_count()];
    let mut receives = vec![false; problem.participant_count()];
    let mut unique_directed_edges = BTreeSet::new();
    for payment in problem.payments() {
        sends[payment.from] = true;
        receives[payment.to] = true;
        unique_directed_edges.insert((payment.from, payment.to));
    }
    TopologyFeatures {
        participants_both_send_receive: sends
            .iter()
            .zip(&receives)
            .filter(|(send, receive)| **send && **receive)
            .count(),
        unique_receivers: receives.iter().filter(|&&receives| receives).count(),
        unique_directed_edges: unique_directed_edges.len(),
    }
}

fn signed_saving(reference: u128, observed: u128) -> String {
    if reference >= observed {
        (reference - observed).to_string()
    } else {
        format!("-{}", observed - reference)
    }
}

fn percentile(samples: &[u128], percentile: usize) -> u128 {
    let rank = (samples.len() * percentile).div_ceil(100).max(1);
    samples[rank.min(samples.len()) - 1]
}

fn measure<F>(mut run: F) -> Result<(SearchResult, Vec<u128>), ReorderError>
where
    F: FnMut() -> Result<SearchResult, ReorderError>,
{
    for _ in 0..WARMUPS {
        std::hint::black_box(run()?);
    }

    let mut samples = Vec::with_capacity(REPETITIONS);
    let mut last = None;
    for _ in 0..REPETITIONS {
        let start = Instant::now();
        let result = run()?;
        let elapsed = start.elapsed().as_nanos();
        samples.push(elapsed);
        last = Some(result);
    }
    samples.sort_unstable();
    Ok((last.ok_or(ReorderError::ArithmeticOverflow)?, samples))
}

fn emit<F>(
    problem: &Problem,
    scenario: Scenario,
    algorithm: &str,
    seed: Option<u64>,
    exact: Option<&SearchResult>,
    pareto_frontier_size: Option<usize>,
    run: F,
) -> Result<(), Box<dyn Error>>
where
    F: FnMut() -> Result<SearchResult, ReorderError>,
{
    let (result, samples) = measure(run)?;
    let features = topology_features(problem);
    let baseline = input_order(problem)?;
    let aggregate_saving = signed_saving(
        baseline.evaluation.aggregate_required_initial_liquidity_minor,
        result.evaluation.aggregate_required_initial_liquidity_minor,
    );
    let max_participant_reduction = signed_saving(
        baseline.evaluation.max_participant_required_initial_liquidity_minor,
        result.evaluation.max_participant_required_initial_liquidity_minor,
    );
    let measured = finance_liquidity_reordering::evaluate_order(problem, &result.order)?;
    if measured != result.evaluation {
        return Err(format!("{} returned inconsistent metrics", algorithm).into());
    }

    let (aggregate_gap, max_participant_gap) = if let Some(optimum) = exact {
        let score = &result.evaluation;
        let best = &optimum.evaluation;
        if score.aggregate_required_initial_liquidity_minor
            < best.aggregate_required_initial_liquidity_minor
            || (score.aggregate_required_initial_liquidity_minor
                == best.aggregate_required_initial_liquidity_minor
                && score.max_participant_required_initial_liquidity_minor
                    < best.max_participant_required_initial_liquidity_minor)
        {
            return Err(format!("{} beat exact oracle unexpectedly", algorithm).into());
        }
        let aggregate_gap = score
            .aggregate_required_initial_liquidity_minor
            .checked_sub(best.aggregate_required_initial_liquidity_minor)
            .ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::InvalidData, "aggregate gap underflow")
            })?;
        let max_gap = (score.max_participant_required_initial_liquidity_minor as i128)
            - (best.max_participant_required_initial_liquidity_minor as i128);
        (aggregate_gap.to_string(), max_gap.to_string())
    } else {
        (String::new(), String::new())
    };

    println!(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        problem.payments().len(),
        problem.participant_count(),
        scenario.label(),
        features.participants_both_send_receive,
        features.unique_receivers,
        features.unique_directed_edges,
        pareto_frontier_size
            .map(|size| size.to_string())
            .unwrap_or_default(),
        algorithm,
        seed.map(|value| value.to_string())
            .unwrap_or_else(|| "-".into()),
        result.candidate_evaluations,
        result.evaluation.aggregate_required_initial_liquidity_minor,
        result
            .evaluation
            .max_participant_required_initial_liquidity_minor,
        aggregate_saving,
        max_participant_reduction,
        aggregate_gap,
        max_participant_gap,
        percentile(&samples, 50),
        percentile(&samples, 95),
        percentile(&samples, 99)
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("payments,participants,scenario,participants_both_send_receive,unique_receivers,unique_directed_edges,exact_pareto_frontier_size,algorithm,seed,candidate_evaluations,aggregate_required_initial_liquidity_minor,max_participant_required_initial_liquidity_minor,aggregate_saving_vs_input_minor,max_participant_reduction_vs_input_minor,aggregate_liquidity_gap_minor,max_participant_gap_minor,p50_ns,p95_ns,p99_ns");

    for payment_count in [4, 6, 8, 10, 30, 70, 140, 700] {
        for scenario in Scenario::ALL {
            let problem = make_problem(payment_count, scenario)?;
            let fifo = input_order(&problem)?;
            let exact = if payment_count <= 8 {
                Some(exact_permutation_oracle(&problem)?)
            } else {
                None
            };
            let pareto_frontier_size = if payment_count <= 8 {
                Some(exact_pareto_frontier(&problem)?.points.len())
            } else {
                None
            };
            let budget = (payment_count * 4).clamp(40, 1_000);

            emit(
                &problem,
                scenario,
                "input_order",
                None,
                exact.as_ref(),
                pareto_frontier_size,
                || input_order(&problem),
            )?;
            emit(
                &problem,
                scenario,
                "receiver_ready_greedy",
                None,
                exact.as_ref(),
                pareto_frontier_size,
                || receiver_ready_greedy(&problem),
            )?;
            emit(
                &problem,
                scenario,
                "bounded_insertion_search",
                None,
                exact.as_ref(),
                pareto_frontier_size,
                || bounded_insertion_search(&problem, &fifo.order, budget),
            )?;

            for seed in ANNEALING_SEEDS {
                emit(
                    &problem,
                    scenario,
                    "simulated_annealing",
                    Some(seed),
                    exact.as_ref(),
                    pareto_frontier_size,
                    || simulated_annealing(&problem, &fifo.order, seed, budget),
                )?;
            }

            emit(
                &problem,
                scenario,
                "heuristic_portfolio",
                Some(41),
                exact.as_ref(),
                pareto_frontier_size,
                || heuristic_portfolio(&problem, budget, 41, budget),
            )?;

            if exact.is_some() {
                emit(
                    &problem,
                    scenario,
                    "exact_permutation_oracle",
                    None,
                    exact.as_ref(),
                    pareto_frontier_size,
                    || exact_permutation_oracle(&problem),
                )?;
            }
        }
    }
    Ok(())
}
