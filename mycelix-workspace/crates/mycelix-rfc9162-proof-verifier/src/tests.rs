use super::*;

fn entries(count: usize) -> Vec<Vec<u8>> {
    (0..count)
        .map(|index| format!("mycelix-rfc9162-entry-{index}").into_bytes())
        .collect()
}

fn leaf_hashes(values: &[Vec<u8>]) -> Vec<Hash> {
    values.iter().map(|value| leaf_hash(value)).collect()
}

fn inclusion_path(leaves: &[Hash], index: usize) -> Vec<Hash> {
    assert!(!leaves.is_empty());
    assert!(index < leaves.len());
    if leaves.len() == 1 {
        return Vec::new();
    }
    let k = largest_power_of_two_less_than(leaves.len());
    if index < k {
        let mut path = inclusion_path(&leaves[..k], index);
        path.push(merkle_tree_hash_from_leaf_hashes(&leaves[k..]));
        path
    } else {
        let mut path = inclusion_path(&leaves[k..], index - k);
        path.push(merkle_tree_hash_from_leaf_hashes(&leaves[..k]));
        path
    }
}

fn consistency_subproof(old_size: usize, leaves: &[Hash], complete: bool) -> Vec<Hash> {
    assert!(old_size > 0 && old_size <= leaves.len());
    if old_size == leaves.len() {
        return if complete {
            Vec::new()
        } else {
            vec![merkle_tree_hash_from_leaf_hashes(leaves)]
        };
    }

    let k = largest_power_of_two_less_than(leaves.len());
    let mut proof = if old_size <= k {
        consistency_subproof(old_size, &leaves[..k], complete)
    } else {
        consistency_subproof(old_size - k, &leaves[k..], false)
    };

    if old_size <= k {
        proof.push(merkle_tree_hash_from_leaf_hashes(&leaves[k..]));
    } else {
        proof.push(merkle_tree_hash_from_leaf_hashes(&leaves[..k]));
    }
    proof
}

fn consistency_path(old_size: usize, leaves: &[Hash]) -> Vec<Hash> {
    consistency_subproof(old_size, leaves, true)
}

fn hex(value: &Hash) -> String {
    value.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[test]
fn stable_rfc9162_sha256_vectors() {
    assert_eq!(
        hex(&empty_tree_hash()),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    assert_eq!(
        hex(&leaf_hash(b"a")),
        "022a6979e6dab7aa5ae4c3e5e45f7e977112a7e63593820dbec1ec738a24f93c"
    );
    assert_eq!(
        hex(&leaf_hash(b"b")),
        "57eb35615d47f34ec714cacdf5fd74608a5e8e102724e80b24b287c0c27b6a31"
    );
    let a = leaf_hash(b"a");
    let b = leaf_hash(b"b");
    assert_eq!(
        hex(&node_hash(&a, &b)),
        "b137985ff484fb600db93107c77b0365c80d78f5b429ded0fd97361d077999eb"
    );
}

#[test]
fn all_leaf_inclusions_verify_for_tree_sizes_one_through_sixty_four() {
    for size in 1..=64usize {
        let values = entries(size);
        let leaves = leaf_hashes(&values);
        let root = merkle_tree_hash_from_leaf_hashes(&leaves);
        for (index, value) in values.iter().enumerate() {
            let path = inclusion_path(&leaves, index);
            let result = verify_entry_inclusion(value, index as u64, size as u64, &root, &path);
            assert_eq!(
                result,
                Ok(InclusionDisposition::InclusionValidRelativeToSuppliedRoot),
                "tree_size={size}, leaf_index={index}, path_len={}",
                path.len()
            );
        }
    }
}

#[test]
fn all_prefix_consistency_pairs_verify_for_tree_sizes_one_through_sixty_four() {
    for new_size in 2..=64usize {
        let values = entries(new_size);
        let leaves = leaf_hashes(&values);
        let new_root = merkle_tree_hash_from_leaf_hashes(&leaves);
        for old_size in 1..new_size {
            let old_root = merkle_tree_hash_from_leaf_hashes(&leaves[..old_size]);
            let path = consistency_path(old_size, &leaves);
            let result = verify_consistency(
                old_size as u64,
                &old_root,
                new_size as u64,
                &new_root,
                &path,
            );
            assert_eq!(
                result,
                Ok(ConsistencyDisposition::ConsistencyValidRelativeToSuppliedRoots),
                "old_size={old_size}, new_size={new_size}, path_len={}",
                path.len()
            );
        }
    }
}

#[test]
fn inclusion_rejects_empty_tree_and_out_of_range_leaf() {
    let hash = leaf_hash(b"entry");
    assert_eq!(
        verify_leaf_inclusion(&hash, 0, 0, &hash, &[]),
        Err(ProofError::EmptyTree)
    );
    assert_eq!(
        verify_leaf_inclusion(&hash, 4, 4, &hash, &[]),
        Err(ProofError::LeafIndexOutOfRange)
    );
}

#[test]
fn inclusion_rejects_wrong_root_and_wrong_leaf() {
    let values = entries(7);
    let leaves = leaf_hashes(&values);
    let root = merkle_tree_hash_from_leaf_hashes(&leaves);
    let path = inclusion_path(&leaves, 3);
    let wrong_root = leaf_hash(b"wrong-root");
    assert_eq!(
        verify_entry_inclusion(&values[3], 3, 7, &wrong_root, &path),
        Err(ProofError::RootMismatch)
    );
    assert_eq!(
        verify_entry_inclusion(b"wrong-entry", 3, 7, &root, &path),
        Err(ProofError::RootMismatch)
    );
}

#[test]
fn inclusion_rejects_truncated_and_extra_paths() {
    let values = entries(9);
    let leaves = leaf_hashes(&values);
    let root = merkle_tree_hash_from_leaf_hashes(&leaves);
    let path = inclusion_path(&leaves, 5);
    assert!(!path.is_empty());

    assert_eq!(
        verify_entry_inclusion(&values[5], 5, 9, &root, &path[..path.len() - 1]),
        Err(ProofError::MalformedInclusionPath)
    );

    let mut extra = path.clone();
    extra.push(leaf_hash(b"extra"));
    assert_eq!(
        verify_entry_inclusion(&values[5], 5, 9, &root, &extra),
        Err(ProofError::MalformedInclusionPath)
    );
}

#[test]
fn inclusion_rejects_excessive_path_length() {
    let hash = leaf_hash(b"entry");
    let path = vec![hash; MAX_INCLUSION_PATH_NODES + 1];
    assert_eq!(
        verify_leaf_inclusion(&hash, 0, u64::MAX, &hash, &path),
        Err(ProofError::ProofPathTooLong)
    );
}

#[test]
fn consistency_rejects_zero_same_and_reversed_tree_sizes() {
    let hash = leaf_hash(b"root");
    assert_eq!(
        verify_consistency(0, &hash, 1, &hash, &[hash]),
        Err(ProofError::InvalidTreeSizeOrder)
    );
    assert_eq!(
        verify_consistency(4, &hash, 4, &hash, &[hash]),
        Err(ProofError::InvalidTreeSizeOrder)
    );
    assert_eq!(
        verify_consistency(5, &hash, 4, &hash, &[hash]),
        Err(ProofError::InvalidTreeSizeOrder)
    );
}

#[test]
fn consistency_rejects_empty_truncated_and_extra_paths() {
    let values = entries(11);
    let leaves = leaf_hashes(&values);
    let old_root = merkle_tree_hash_from_leaf_hashes(&leaves[..5]);
    let new_root = merkle_tree_hash_from_leaf_hashes(&leaves);
    let path = consistency_path(5, &leaves);
    assert!(!path.is_empty());

    assert_eq!(
        verify_consistency(5, &old_root, 11, &new_root, &[]),
        Err(ProofError::EmptyConsistencyPath)
    );
    assert_eq!(
        verify_consistency(5, &old_root, 11, &new_root, &path[..path.len() - 1]),
        Err(ProofError::MalformedConsistencyPath)
    );
    let mut extra = path.clone();
    extra.push(leaf_hash(b"extra"));
    assert_eq!(
        verify_consistency(5, &old_root, 11, &new_root, &extra),
        Err(ProofError::MalformedConsistencyPath)
    );
}

#[test]
fn consistency_rejects_wrong_old_or_new_roots() {
    let values = entries(13);
    let leaves = leaf_hashes(&values);
    let old_root = merkle_tree_hash_from_leaf_hashes(&leaves[..6]);
    let new_root = merkle_tree_hash_from_leaf_hashes(&leaves);
    let path = consistency_path(6, &leaves);
    let wrong = leaf_hash(b"wrong-root");

    assert_eq!(
        verify_consistency(6, &wrong, 13, &new_root, &path),
        Err(ProofError::RootMismatch)
    );
    assert_eq!(
        verify_consistency(6, &old_root, 13, &wrong, &path),
        Err(ProofError::RootMismatch)
    );
}

#[test]
fn consistency_detects_reordered_and_mutated_path_nodes() {
    let values = entries(15);
    let leaves = leaf_hashes(&values);
    let old_root = merkle_tree_hash_from_leaf_hashes(&leaves[..7]);
    let new_root = merkle_tree_hash_from_leaf_hashes(&leaves);
    let path = consistency_path(7, &leaves);
    assert!(path.len() >= 2);

    let mut reordered = path.clone();
    reordered.swap(0, 1);
    assert_eq!(
        verify_consistency(7, &old_root, 15, &new_root, &reordered),
        Err(ProofError::RootMismatch)
    );

    let mut mutated = path.clone();
    mutated[0] = leaf_hash(b"mutated");
    assert_eq!(
        verify_consistency(7, &old_root, 15, &new_root, &mutated),
        Err(ProofError::RootMismatch)
    );
}

#[test]
fn node_and_leaf_domains_are_separated() {
    let payload = b"same payload";
    let leaf = leaf_hash(payload);
    let raw: Hash = Sha256::digest(payload).into();
    assert_ne!(leaf, raw);
    assert_ne!(node_hash(&leaf, &raw), node_hash(&raw, &leaf));
}

#[test]
fn consumes_shared_go_rust_interoperability_vectors() {
    const VECTORS: &str =
        include_str!("../../../../tools/economic/temporal-frontier-model/RFC9162_VECTORS_V1.txt");
    let mut leaves = std::collections::BTreeMap::<String, Hash>::new();
    let mut mth = std::collections::BTreeMap::<usize, Hash>::new();
    let mut inclusions = Vec::new();

    for (line_number, raw_line) in VECTORS.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("MYCELIX-") {
            continue;
        }
        let fields: Vec<&str> = line.split('|').collect();
        match fields.first().copied() {
            Some("leaf") if fields.len() == 4 => {
                let payload = hex::decode(fields[2]).expect("valid shared vector payload hex");
                let expected: Hash = hex::decode(fields[3])
                    .expect("valid shared leaf hash hex")
                    .try_into()
                    .expect("32-byte shared leaf hash");
                assert_eq!(
                    leaf_hash(&payload),
                    expected,
                    "shared leaf vector at line {}",
                    line_number + 1
                );
                leaves.insert(fields[1].to_owned(), expected);
            }
            Some("mth") if fields.len() == 3 => {
                let count: usize = fields[1].parse().expect("valid tree size");
                let expected: Hash = hex::decode(fields[2])
                    .expect("valid shared MTH hex")
                    .try_into()
                    .expect("32-byte shared MTH");
                mth.insert(count, expected);
            }
            Some("include") if fields.len() == 6 => {
                let index: u64 = fields[2].parse().expect("valid leaf index");
                let size: u64 = fields[3].parse().expect("valid tree size");
                let root: Hash = hex::decode(fields[4])
                    .expect("valid shared inclusion root hex")
                    .try_into()
                    .expect("32-byte inclusion root");
                let path = if fields[5].is_empty() {
                    Vec::new()
                } else {
                    fields[5]
                        .split(',')
                        .map(|part| {
                            hex::decode(part)
                                .expect("valid inclusion path node hex")
                                .try_into()
                                .expect("32-byte inclusion path node")
                        })
                        .collect()
                };
                inclusions.push((fields[1].to_owned(), index, size, root, path));
            }
            other => panic!(
                "invalid shared vector at line {}: {:?}",
                line_number + 1,
                other
            ),
        }
    }

    let ordered = [
        *leaves.get("a").expect("shared leaf a"),
        *leaves.get("b").expect("shared leaf b"),
        *leaves.get("c").expect("shared leaf c"),
    ];
    assert_eq!(leaves.len(), 3);
    for (size, expected) in mth {
        assert_eq!(
            merkle_tree_hash_from_leaf_hashes(&ordered[..size]),
            expected,
            "shared MTH({size}) vector"
        );
    }
    for (name, index, size, root, path) in inclusions {
        let leaf = *leaves.get(&name).expect("shared inclusion leaf");
        assert_eq!(
            verify_leaf_inclusion(&leaf, index, size, &root, &path),
            Ok(InclusionDisposition::InclusionValidRelativeToSuppliedRoot),
            "shared inclusion vector {name}"
        );
    }

    #[test]
    fn xenia_checkpoint_v1_payload_has_the_same_rfc9162_leaf_hash() {
        const VECTOR: &str = include_str!(
            "../../../../tools/economic/temporal-frontier-model/XENIA_CHECKPOINT_V1_KAT.txt"
        );
        let mut payload_len = None;
        let mut payload_hex = None;
        let mut expected_leaf_hex = None;
        for line in VECTOR.lines().map(str::trim) {
            if let Some(value) = line.strip_prefix("payload_len=") {
                payload_len = Some(value.parse::<usize>().expect("valid Xenia payload length"));
            } else if let Some(value) = line.strip_prefix("payload_hex=") {
                payload_hex = Some(value);
            } else if let Some(value) = line.strip_prefix("leaf_hash_sha256=") {
                expected_leaf_hex = Some(value);
            }
        }
        let payload = hex::decode(payload_hex.unwrap_or_else(|| panic!("Xenia payload missing")))
            .expect("valid Xenia payload hex");
        assert_eq!(
            payload.len(),
            payload_len.expect("Xenia payload length missing")
        );
        let expected: Hash =
            hex::decode(expected_leaf_hex.unwrap_or_else(|| panic!("Xenia leaf hash missing")))
                .expect("valid Xenia leaf hash hex")
                .try_into()
                .expect("32-byte Xenia leaf hash");
        assert_eq!(leaf_hash(&payload), expected);
    }

}
