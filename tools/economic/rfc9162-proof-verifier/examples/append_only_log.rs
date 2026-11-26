use mycelix_rfc9162_proof_verifier::{
    consistency_proof, inclusion_proof, tree_hash, verify_consistency_proof,
    verify_entry_inclusion_proof,
};

fn main() {
    let version_one = vec![
        b"grant-created".to_vec(),
        b"grant-approved".to_vec(),
        b"payment-authorized".to_vec(),
    ];
    let version_two = vec![
        b"grant-created".to_vec(),
        b"grant-approved".to_vec(),
        b"payment-authorized".to_vec(),
        b"grant-revoked".to_vec(),
        b"revocation-received".to_vec(),
    ];

    let old_root = tree_hash(&version_one).expect("old tree hash");
    let new_root = tree_hash(&version_two).expect("new tree hash");

    let receipt_index = 4_u64;
    let receipt_proof = inclusion_proof(&version_two, receipt_index).expect("inclusion proof");
    verify_entry_inclusion_proof(
        &version_two[receipt_index as usize],
        receipt_index,
        version_two.len() as u64,
        &receipt_proof,
        &new_root,
    )
    .expect("receipt inclusion verifies");

    let extension_proof =
        consistency_proof(&version_two, version_one.len() as u64).expect("consistency proof");
    verify_consistency_proof(
        version_one.len() as u64,
        &old_root,
        version_two.len() as u64,
        &new_root,
        &extension_proof,
    )
    .expect("new tree is an append-only extension");

    println!("receipt inclusion: verified against supplied root");
    println!("tree consistency: verified against supplied roots");
    println!("checkpoint signatures and log identity: NOT VERIFIED");
}
