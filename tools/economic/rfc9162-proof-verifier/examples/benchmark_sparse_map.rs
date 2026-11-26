use std::hint::black_box;
use std::time::{Duration, Instant};

use mycelix_rfc9162_proof_verifier::{
    SparseKeyV1, SparseMerkleTreeV1, SparseValueV1, assertion_index_value_v1, sha256,
    verify_sparse_membership_v1, verify_sparse_non_membership_v1,
};

fn percentile(samples: &[Duration], percentile: usize) -> Duration {
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    sorted[((sorted.len() - 1) * percentile).div_ceil(100)]
}

fn indexed_key(index: u64) -> SparseKeyV1 {
    let mut bytes = b"MYCELIX-SPARSE-BENCHMARK-KEY-V1\0".to_vec();
    bytes.extend_from_slice(&index.to_be_bytes());
    sha256(&bytes)
}

fn indexed_value(index: u64) -> SparseValueV1 {
    let receipt_digest = format!("sha256:{:064x}", index + 1);
    assertion_index_value_v1(&receipt_digest, "owner:benchmark-receipts", index + 1)
        .expect("well-formed receipt index value")
}

fn measure_verify(
    mut verify: impl FnMut() -> Result<(), mycelix_rfc9162_proof_verifier::SparseMerkleErrorV1>,
) -> Vec<Duration> {
    let mut samples = Vec::with_capacity(101);
    for _ in 0..101 {
        let start = Instant::now();
        verify().expect("proof should verify");
        samples.push(start.elapsed());
    }
    samples
}

fn main() {
    println!(
        "entries,build_us,root_us,membership_prove_us,nonmembership_prove_us,proof_bytes,membership_verify_p50_ns,membership_verify_p95_ns,membership_verify_p99_ns,nonmembership_verify_p50_ns,nonmembership_verify_p95_ns,nonmembership_verify_p99_ns,linear_scan_absent_p50_ns"
    );

    for count in [64_u64, 256, 1_024, 4_096] {
        let build_start = Instant::now();
        let mut tree = SparseMerkleTreeV1::new();
        let mut linear_entries = Vec::with_capacity(count as usize);
        for index in 0..count {
            let key = indexed_key(index);
            let value = indexed_value(index);
            tree.insert(key, value).expect("unique deterministic key");
            linear_entries.push((key, value));
        }
        let build_elapsed = build_start.elapsed();

        let root_start = Instant::now();
        let root = tree.root_hash();
        let root_elapsed = root_start.elapsed();

        let present_key = indexed_key(count / 2);
        let present_value = indexed_value(count / 2);
        let membership_start = Instant::now();
        let membership_proof = tree.prove(present_key);
        let membership_prove_elapsed = membership_start.elapsed();

        let mut absent_key = indexed_key(count + 10_000);
        absent_key[0] ^= 0x80;
        assert!(
            linear_entries
                .iter()
                .all(|(candidate, _)| candidate != &absent_key),
            "benchmark negative-control key unexpectedly collides"
        );
        let nonmembership_start = Instant::now();
        let nonmembership_proof = tree.prove(absent_key);
        let nonmembership_prove_elapsed = nonmembership_start.elapsed();

        let membership_samples = measure_verify(|| {
            verify_sparse_membership_v1(
                &present_key,
                &present_value,
                black_box(&membership_proof),
                &root,
            )
        });
        let nonmembership_samples = measure_verify(|| {
            verify_sparse_non_membership_v1(&absent_key, black_box(&nonmembership_proof), &root)
        });

        // This is only an in-memory lookup cost baseline. It does not provide
        // cryptographic assurance and is not semantically equivalent to
        // independently verifying a sparse non-membership proof.
        let mut scan_samples = Vec::with_capacity(101);
        for _ in 0..101 {
            let start = Instant::now();
            let found = linear_entries
                .iter()
                .any(|(candidate, _)| candidate == &absent_key);
            assert!(!black_box(found));
            scan_samples.push(start.elapsed());
        }

        let mp50 = percentile(&membership_samples, 50);
        let mp95 = percentile(&membership_samples, 95);
        let mp99 = percentile(&membership_samples, 99);
        let np50 = percentile(&nonmembership_samples, 50);
        let np95 = percentile(&nonmembership_samples, 95);
        let np99 = percentile(&nonmembership_samples, 99);
        let sp50 = percentile(&scan_samples, 50);
        let proof_bytes = membership_proof
            .siblings_leaf_to_root
            .len()
            .max(nonmembership_proof.siblings_leaf_to_root.len())
            * 32;

        println!(
            "{count},{},{},{},{},{},{},{},{},{},{},{},{}",
            build_elapsed.as_micros(),
            root_elapsed.as_micros(),
            membership_prove_elapsed.as_micros(),
            nonmembership_prove_elapsed.as_micros(),
            proof_bytes,
            mp50.as_nanos(),
            mp95.as_nanos(),
            mp99.as_nanos(),
            np50.as_nanos(),
            np95.as_nanos(),
            np99.as_nanos(),
            sp50.as_nanos(),
        );
    }
}
