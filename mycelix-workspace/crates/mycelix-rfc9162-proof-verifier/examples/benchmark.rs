use mycelix_rfc9162_proof_verifier::{
    Hash, leaf_hash, merkle_tree_hash_from_leaf_hashes, node_hash, verify_consistency,
    verify_leaf_inclusion,
};
use std::hint::black_box;
use std::time::Instant;

const TRIALS: usize = 30;
const SIZES: [usize; 3] = [1_000, 10_000, 100_000];

fn largest_power_of_two_less_than(n: usize) -> usize {
    let mut k = 1usize;
    while k <= (n - 1) / 2 {
        k <<= 1;
    }
    k
}

fn make_entries(count: usize) -> Vec<Vec<u8>> {
    (0..count)
        .map(|i| format!("rfc9162-benchmark-leaf-{i:08}").into_bytes())
        .collect()
}

fn make_inclusion_path(leaves: &[Hash], index: usize) -> Vec<Hash> {
    if leaves.len() == 1 {
        return Vec::new();
    }
    let k = largest_power_of_two_less_than(leaves.len());
    if index < k {
        let mut path = make_inclusion_path(&leaves[..k], index);
        path.push(merkle_tree_hash_from_leaf_hashes(&leaves[k..]));
        path
    } else {
        let mut path = make_inclusion_path(&leaves[k..], index - k);
        path.push(merkle_tree_hash_from_leaf_hashes(&leaves[..k]));
        path
    }
}

fn make_consistency_subproof(old_size: usize, leaves: &[Hash], complete: bool) -> Vec<Hash> {
    if old_size == leaves.len() {
        return if complete {
            Vec::new()
        } else {
            vec![merkle_tree_hash_from_leaf_hashes(leaves)]
        };
    }
    let k = largest_power_of_two_less_than(leaves.len());
    let mut proof = if old_size <= k {
        make_consistency_subproof(old_size, &leaves[..k], complete)
    } else {
        make_consistency_subproof(old_size - k, &leaves[k..], false)
    };
    if old_size <= k {
        proof.push(merkle_tree_hash_from_leaf_hashes(&leaves[k..]));
    } else {
        proof.push(merkle_tree_hash_from_leaf_hashes(&leaves[..k]));
    }
    proof
}

fn make_consistency_path(old_size: usize, leaves: &[Hash]) -> Vec<Hash> {
    make_consistency_subproof(old_size, leaves, true)
}

fn percentile(sorted: &[u128], p: usize) -> u128 {
    let index = ((sorted.len() - 1) * p).div_ceil(100);
    sorted[index.min(sorted.len() - 1)]
}

fn measure(operation: &str, size: usize, mut run: impl FnMut()) {
    run(); // warm-up
    let mut samples = Vec::with_capacity(TRIALS);
    for trial in 0..TRIALS {
        let started = Instant::now();
        run();
        let elapsed = started.elapsed().as_nanos();
        samples.push(elapsed);
        println!("sample,{operation},{size},{trial},{elapsed}");
    }
    samples.sort_unstable();
    println!(
        "summary,{operation},{size},p50_ns={},p95_ns={},p99_ns={}",
        percentile(&samples, 50),
        percentile(&samples, 95),
        percentile(&samples, 99)
    );
}

fn main() {
    println!("record_type,operation,leaf_count,trial,duration_ns_or_percentiles");
    for size in SIZES {
        let entries = make_entries(size);
        let leaves: Vec<Hash> = entries.iter().map(|entry| leaf_hash(entry)).collect();
        let root = merkle_tree_hash_from_leaf_hashes(&leaves);
        let old_size = size / 2;
        let old_root = merkle_tree_hash_from_leaf_hashes(&leaves[..old_size]);
        let index = size / 2;
        let inclusion = make_inclusion_path(&leaves, index);
        let consistency = make_consistency_path(old_size, &leaves);
        let payload = &entries[index];

        measure("mth_full", size, || {
            black_box(merkle_tree_hash_from_leaf_hashes(black_box(&leaves)));
        });
        measure("inclusion_verify", size, || {
            let result = verify_leaf_inclusion(
                black_box(&leaves[index]),
                index as u64,
                size as u64,
                black_box(&root),
                black_box(&inclusion),
            );
            assert!(result.is_ok());
        });
        measure("consistency_verify", size, || {
            let result = verify_consistency(
                old_size as u64,
                black_box(&old_root),
                size as u64,
                black_box(&root),
                black_box(&consistency),
            );
            assert!(result.is_ok());
        });
        // Exercise the payload API as a separate sanity check, not a timed row.
        assert!(
            verify_leaf_inclusion(
                &leaf_hash(payload),
                index as u64,
                size as u64,
                &root,
                &inclusion
            )
            .is_ok()
        );
    }
    let _ = node_hash(&leaf_hash(b"left"), &leaf_hash(b"right"));
}
