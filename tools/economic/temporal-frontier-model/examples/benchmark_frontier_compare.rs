use std::hint::black_box;
use std::time::{Duration, Instant};

use mycelix_rfc9162_proof_verifier::{
    consistency_proof, inclusion_proof, tree_hash,
};
use mycelix_temporal_frontier_model::{
    canonical_owner_cut_entry_v1, CheckpointInclusionDeclarationV1,
    FrontierRelationV1, OwnerConsistencyLinkV1, OwnerCutV1,
    VerifierFrontierV1,
};

const VERIFIER: &str = "did:mycelix:verifier:benchmark-v1";
const POLICY_EPOCH: &str = "policy-epoch:benchmark-v1";

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn owner_entries(owner_ref: &str, sequence: u64) -> Vec<Vec<u8>> {
    (1..=sequence)
        .map(|position| format!("benchmark-owner-log:{owner_ref}:{position}").into_bytes())
        .collect()
}

fn build_frontier(id: &str, owner_count: usize, sequence: u64) -> VerifierFrontierV1 {
    let mut owner_cuts: Vec<OwnerCutV1> = (0..owner_count)
        .map(|owner_index| {
            let owner_ref = format!("owner:{owner_index:08}");
            let history_ref = format!("history:{owner_index:08}");
            let entries = owner_entries(&owner_ref, sequence);
            let root_digest = format!("sha256:{}", hex(&tree_hash(&entries).unwrap()));
            OwnerCutV1 {
                owner_ref,
                history_ref,
                sequence,
                tree_size: entries.len() as u64,
                root_digest: root_digest.clone(),
                checkpoint_inclusion: CheckpointInclusionDeclarationV1 {
                    checkpoint_root_digest: String::new(),
                    owner_root_digest: root_digest,
                    leaf_index: 0,
                    tree_size: 1,
                    inclusion_path: Vec::new(),
                    proof_ref: format!("proof:owner-cut:{owner_index:08}"),
                },
                gaps: Vec::new(),
            }
        })
        .collect();

    owner_cuts.sort_by(|left, right| left.owner_ref.cmp(&right.owner_ref));
    let checkpoint_entries: Vec<Vec<u8>> = owner_cuts
        .iter()
        .map(|cut| canonical_owner_cut_entry_v1(cut, VERIFIER, POLICY_EPOCH).unwrap())
        .collect();
    let checkpoint_root = tree_hash(&checkpoint_entries).unwrap();
    let checkpoint_root_digest = format!("sha256:{}", hex(&checkpoint_root));

    for (index, cut) in owner_cuts.iter_mut().enumerate() {
        cut.checkpoint_inclusion = CheckpointInclusionDeclarationV1 {
            checkpoint_root_digest: checkpoint_root_digest.clone(),
            owner_root_digest: cut.root_digest.clone(),
            leaf_index: index as u64,
            tree_size: checkpoint_entries.len() as u64,
            inclusion_path: inclusion_proof(&checkpoint_entries, index as u64).unwrap(),
            proof_ref: format!("proof:checkpoint-inclusion:{index:08}"),
        };
    }

    VerifierFrontierV1 {
        frontier_id: id.into(),
        verifier_ref: VERIFIER.into(),
        policy_epoch_ref: POLICY_EPOCH.into(),
        checkpoint_root_digest,
        parent_frontier_ids: Vec::new(),
        owner_cuts,
    }
}

fn build_links(owner_count: usize) -> Vec<OwnerConsistencyLinkV1> {
    (0..owner_count)
        .map(|owner_index| {
            let owner_ref = format!("owner:{owner_index:08}");
            let entries = owner_entries(&owner_ref, 20);
            let first_root = tree_hash(&entries[..10]).unwrap();
            let second_root = tree_hash(&entries).unwrap();
            OwnerConsistencyLinkV1 {
                owner_ref: owner_ref.clone(),
                history_ref: format!("history:{owner_index:08}"),
                ancestor_root_digest: format!("sha256:{}", hex(&first_root)),
                descendant_root_digest: format!("sha256:{}", hex(&second_root)),
                first_tree_size: 10,
                second_tree_size: 20,
                consistency_path: consistency_proof(&entries, 10).unwrap(),
                consistency_proof_ref: format!("proof:consistency:{owner_index:08}"),
            }
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
    println!(
        "owners,checkpoint_proof_nodes,consistency_proof_nodes,build_us,compare_p50_us,compare_p95_us,compare_p99_us"
    );
    for owner_count in [16_usize, 64, 256, 1_024] {
        let build_start = Instant::now();
        let earlier = build_frontier("frontier:earlier", owner_count, 10);
        let later = build_frontier("frontier:later", owner_count, 20);
        let links = build_links(owner_count);
        earlier.validate().expect("earlier checkpoint inclusion proofs validate");
        later.validate().expect("later checkpoint inclusion proofs validate");
        let build_elapsed = build_start.elapsed();

        let checkpoint_nodes: usize = earlier
            .owner_cuts
            .iter()
            .chain(later.owner_cuts.iter())
            .map(|cut| cut.checkpoint_inclusion.inclusion_path.len())
            .sum();
        let consistency_nodes: usize = links
            .iter()
            .map(|link| link.consistency_path.len())
            .sum();

        let mut samples = Vec::with_capacity(101);
        for _ in 0..101 {
            let start = Instant::now();
            let relation = earlier
                .compare(&later, black_box(&links))
                .expect("frontier consistency proofs should validate");
            assert_eq!(relation, FrontierRelationV1::Before);
            black_box(relation);
            samples.push(start.elapsed());
        }

        let p50 = percentile(&samples, 50);
        let p95 = percentile(&samples, 95);
        let p99 = percentile(&samples, 99);
        println!(
            "{owner_count},{checkpoint_nodes},{consistency_nodes},{},{},{},{}",
            build_elapsed.as_micros(),
            p50.as_micros(),
            p95.as_micros(),
            p99.as_micros()
        );
    }
}
