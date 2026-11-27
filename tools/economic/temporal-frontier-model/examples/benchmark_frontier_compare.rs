use std::hint::black_box;
use std::time::{Duration, Instant};

use mycelix_temporal_frontier_model::{
    CheckpointInclusionDeclarationV1, FrontierRelationV1, OwnerConsistencyLinkV1,
    OwnerCutV1, SequenceRangeV1, VerifierFrontierV1,
};

const VERIFIER: &str = "did:mycelix:verifier:benchmark-v1";

fn digest(value: u64) -> String {
    format!("sha256:{value:064x}")
}

fn aggregate_root(id: &str) -> String {
    let value = id
        .bytes()
        .fold(0_u64, |acc, byte| acc.wrapping_mul(31).wrapping_add(u64::from(byte)));
    digest(value)
}

fn owner_root(owner_index: usize, sequence: u64) -> String {
    digest((owner_index as u64).wrapping_mul(1_000_003).wrapping_add(sequence))
}

fn build_frontier(id: &str, owner_count: usize, sequence: u64) -> VerifierFrontierV1 {
    let checkpoint_root_digest = aggregate_root(id);
    let owner_cuts = (0..owner_count)
        .map(|owner_index| {
            let owner_ref = format!("owner:{owner_index:08}");
            let history_ref = format!("history:{owner_index:08}");
            let root_digest = owner_root(owner_index, sequence);
            OwnerCutV1 {
                owner_ref,
                history_ref,
                sequence,
                root_digest: root_digest.clone(),
                checkpoint_inclusion: CheckpointInclusionDeclarationV1 {
                    checkpoint_root_digest: checkpoint_root_digest.clone(),
                    owner_root_digest: root_digest,
                    proof_ref: format!("proof:owner-checkpoint:{owner_index:08}"),
                },
                gaps: Vec::<SequenceRangeV1>::new(),
            }
        })
        .collect();

    VerifierFrontierV1 {
        frontier_id: id.into(),
        verifier_ref: VERIFIER.into(),
        policy_epoch_ref: "policy-epoch:benchmark-v1".into(),
        checkpoint_root_digest,
        parent_frontier_ids: Vec::new(),
        owner_cuts,
    }
}

fn build_links(owner_count: usize) -> Vec<OwnerConsistencyLinkV1> {
    (0..owner_count)
        .map(|owner_index| OwnerConsistencyLinkV1 {
            owner_ref: format!("owner:{owner_index:08}"),
            history_ref: format!("history:{owner_index:08}"),
            ancestor_root_digest: owner_root(owner_index, 10),
            descendant_root_digest: owner_root(owner_index, 20),
            consistency_proof_ref: format!("proof:consistency:{owner_index:08}"),
        })
        .collect()
}

fn percentile(samples: &[Duration], percentile: usize) -> Duration {
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    let index = ((sorted.len() - 1) * percentile).div_ceil(100);
    sorted[index]
}

fn main() {
    println!("owners,links,build_us,compare_p50_us,compare_p95_us,compare_p99_us");
    for owner_count in [16_usize, 64, 256, 1_024] {
        let build_start = Instant::now();
        let earlier = build_frontier("frontier:earlier", owner_count, 10);
        let later = build_frontier("frontier:later", owner_count, 20);
        let links = build_links(owner_count);
        earlier.validate().expect("earlier frontier should validate");
        later.validate().expect("later frontier should validate");
        let build_elapsed = build_start.elapsed();

        let mut samples = Vec::with_capacity(101);
        for _ in 0..101 {
            let start = Instant::now();
            let relation = earlier
                .compare(&later, black_box(&links))
                .expect("frontier comparison should be structurally valid");
            assert_eq!(relation, FrontierRelationV1::Before);
            black_box(relation);
            samples.push(start.elapsed());
        }

        let p50 = percentile(&samples, 50);
        let p95 = percentile(&samples, 95);
        let p99 = percentile(&samples, 99);
        println!(
            "{owner_count},{},{},{},{},{}",
            links.len(),
            build_elapsed.as_micros(),
            p50.as_micros(),
            p95.as_micros(),
            p99.as_micros()
        );
    }
}
