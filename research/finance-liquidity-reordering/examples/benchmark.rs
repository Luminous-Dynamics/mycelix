use finance_liquidity_reordering::{
    bounded_insertion_search, exact_permutation_oracle, input_order, receiver_ready_greedy,
    simulated_annealing, Evaluation, Payment, Problem, ReorderError, SearchResult,
};
use std::error::Error;
use std::time::Instant;

const WARMUPS: usize = 3;
const REPETITIONS: usize = 31;
const ANNEALING_SEEDS: [u64; 3] = [17, 101, 997];

fn make_reversed_chain(length: usize) -> Result<Problem, ReorderError> {
    let payments = (0..length)
        .rev()
        .map(|i| Payment {
            id: format!("p{i:04}"),
            from: i,
            to: i + 1,
            amount_minor: 10,
        })
        .collect();
    Problem::new(length + 1, payments)
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
    algorithm: &str,
    seed: Option<u64>,
    exact: Option<&SearchResult>,
    run: F,
) -> Result<(), Box<dyn Error>>
where
    F: FnMut() -> Result<SearchResult, ReorderError>,
{
    let (result, samples) = measure(run)?;
    let measured = finance_liquidity_reordering::evaluate_order(problem, &result.order)?;
    if measured != result.evaluation {
        return Err(format!("{} returned inconsistent metrics", algorithm).into());
    }

    let gap = if let Some(optimum) = exact {
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
        score
            .aggregate_required_initial_liquidity_minor
            .checked_sub(best.aggregate_required_initial_liquidity_minor)
            .ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::InvalidData, "objective gap underflow")
            })?
            .to_string()
    } else {
        String::new()
    };

    println!(
        "{},{},reversed_equal_chain,{},{},{},{},{},{},{},{},{}",
        problem.payments().len(),
        problem.participant_count(),
        algorithm,
        seed.map(|value| value.to_string())
            .unwrap_or_else(|| "-".into()),
        result.candidate_evaluations,
        result.evaluation.aggregate_required_initial_liquidity_minor,
        result
            .evaluation
            .max_participant_required_initial_liquidity_minor,
        gap,
        percentile(&samples, 50),
        percentile(&samples, 95),
        percentile(&samples, 99)
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("payments,participants,scenario,algorithm,seed,candidate_evaluations,aggregate_required_initial_liquidity_minor,max_participant_required_initial_liquidity_minor,optimality_gap_minor,p50_ns,p95_ns,p99_ns");

    for payment_count in [4, 6, 8, 10, 30, 70, 140, 700] {
        let problem = make_reversed_chain(payment_count)?;
        let fifo = input_order(&problem)?;
        let exact = if payment_count <= 8 {
            Some(exact_permutation_oracle(&problem)?)
        } else {
            None
        };
        let budget = (payment_count * 8).clamp(40, 2_000);

        emit(&problem, "input_order", None, exact.as_ref(), || {
            input_order(&problem)
        })?;
        emit(
            &problem,
            "receiver_ready_greedy",
            None,
            exact.as_ref(),
            || receiver_ready_greedy(&problem),
        )?;
        emit(
            &problem,
            "bounded_insertion_search",
            None,
            exact.as_ref(),
            || bounded_insertion_search(&problem, &fifo.order, budget),
        )?;

        for seed in ANNEALING_SEEDS {
            emit(
                &problem,
                "simulated_annealing",
                Some(seed),
                exact.as_ref(),
                || simulated_annealing(&problem, &fifo.order, seed, budget),
            )?;
        }

        if exact.is_some() {
            emit(
                &problem,
                "exact_permutation_oracle",
                None,
                exact.as_ref(),
                || exact_permutation_oracle(&problem),
            )?;
        }
    }
    Ok(())
}
