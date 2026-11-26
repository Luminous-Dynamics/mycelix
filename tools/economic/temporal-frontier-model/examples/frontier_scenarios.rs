use mycelix_rfc9162_proof_verifier::{inclusion_proof, tree_hash};
use mycelix_temporal_frontier_model::{
    canonical_knowledge_receipt_entry_v1, canonical_owner_cut_entry_v1,
    CheckpointInclusionDeclarationV1, FrontierRelationV1, KnowledgeReceiptV1,
    OwnerConsistencyLinkV1, OwnerCutV1, OwnerPositionV1,
    ReceiptInclusionDeclarationV1, ReceiptVisibilityV1, SequenceRangeV1,
    VerifierFrontierV1,
};

const VERIFIER: &str = "did:mycelix:verifier:finance-v1";
const POLICY: &str = "policy-epoch:1";

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn owner_entries_with_gaps(
    owner: &str,
    sequence: u64,
    gaps: &[SequenceRangeV1],
) -> Vec<Vec<u8>> {
    (1..=sequence)
        .filter(|position| {
            !gaps.iter().any(|gap| gap.first <= *position && *position <= gap.last)
        })
        .map(|position| format!("owner-log:{owner}:{position}").into_bytes())
        .collect()
}

fn owner_entries(owner: &str, sequence: u64) -> Vec<Vec<u8>> {
    owner_entries_with_gaps(owner, sequence, &[])
}

fn owner_cut(owner: &str, sequence: u64) -> OwnerCutV1 {
    let root_digest = format!("sha256:{}", hex(&tree_hash(&owner_entries(owner, sequence)).unwrap()));
    OwnerCutV1 {
        owner_ref: owner.into(),
        history_ref: format!("history:{owner}"),
        sequence,
        root_digest: root_digest.clone(),
        checkpoint_inclusion: CheckpointInclusionDeclarationV1 {
            checkpoint_root_digest: String::new(),
            owner_root_digest: root_digest,
            leaf_index: 0,
            tree_size: 1,
            inclusion_path: Vec::new(),
            proof_ref: format!("proof:owner-checkpoint:{owner}"),
        },
        gaps: Vec::new(),
    }
}

fn frontier(id: &str, mut cuts: Vec<OwnerCutV1>) -> VerifierFrontierV1 {
    cuts.sort_by(|left, right| left.owner_ref.cmp(&right.owner_ref));
    let entries: Vec<Vec<u8>> = cuts
        .iter()
        .map(|cut| canonical_owner_cut_entry_v1(cut, VERIFIER, POLICY).unwrap())
        .collect();
    let checkpoint_root = tree_hash(&entries).unwrap();
    let checkpoint_root_digest = format!("sha256:{}", hex(&checkpoint_root));
    for (index, cut) in cuts.iter_mut().enumerate() {
        cut.checkpoint_inclusion = CheckpointInclusionDeclarationV1 {
            checkpoint_root_digest: checkpoint_root_digest.clone(),
            owner_root_digest: cut.root_digest.clone(),
            leaf_index: index as u64,
            tree_size: entries.len() as u64,
            inclusion_path: inclusion_proof(&entries, index as u64).unwrap(),
            proof_ref: format!("proof:checkpoint-inclusion:{index}"),
        };
    }
    VerifierFrontierV1 {
        frontier_id: id.into(),
        verifier_ref: VERIFIER.into(),
        policy_epoch_ref: POLICY.into(),
        checkpoint_root_digest,
        parent_frontier_ids: Vec::new(),
        owner_cuts: cuts,
    }
}

fn consistency_link(
    owner: &str,
    first_tree_size: u64,
    second_tree_size: u64,
) -> OwnerConsistencyLinkV1 {
    let entries = owner_entries(owner, second_tree_size);
    let first_root = tree_hash(&entries[..first_tree_size as usize]).unwrap();
    let second_root = tree_hash(&entries).unwrap();
    mycelix_rfc9162_proof_verifier::consistency_proof(
        &entries,
        first_tree_size,
    )
    .map(|consistency_path| OwnerConsistencyLinkV1 {
        owner_ref: owner.into(),
        history_ref: format!("history:{owner}"),
        ancestor_root_digest: format!("sha256:{}", hex(&first_root)),
        descendant_root_digest: format!("sha256:{}", hex(&second_root)),
        first_tree_size,
        second_tree_size,
        consistency_path,
        consistency_proof_ref: format!(
            "proof:consistency:{owner}:{first_tree_size}:{second_tree_size}"
        ),
    })
    .expect("owner consistency proof")
}

fn main() {
    let grants_10 = owner_cut("owner:grants", 10);
    let mut grants_20 = owner_cut("owner:grants", 20);
    let payments_20 = owner_cut("owner:payments", 20);
    let payments_10 = owner_cut("owner:payments", 10);

    let mut receipt = KnowledgeReceiptV1 {
        receipt_id: "receipt:grant-revocation:5".into(),
        verifier_ref: VERIFIER.into(),
        assertion_id: "assertion:grant-revocation".into(),
        assertion_digest: format!("sha256:{}", "b".repeat(64)),
        owner_ref: "owner:grants".into(),
        owner_sequence: 5,
        causal_dependencies: vec![
            OwnerPositionV1 { owner_ref: "owner:governance".into(), sequence: 3 },
        ],
        receipt_digest: String::new(),
        inclusion: None,
    };
    let receipt_payload = canonical_knowledge_receipt_entry_v1(&receipt)
        .expect("canonical receipt encoding");
    receipt.receipt_digest = format!("sha256:{}", hex(&sha2_digest(&receipt_payload)));

    let mut grant_log = owner_entries("owner:grants", 20);
    grant_log[4] = receipt_payload;
    let grant_root = tree_hash(&grant_log).unwrap();
    grants_20.root_digest = format!("sha256:{}", hex(&grant_root));
    receipt.inclusion = Some(ReceiptInclusionDeclarationV1 {
        owner_root_digest: grants_20.root_digest.clone(),
        leaf_index: 4,
        tree_size: 20,
        inclusion_path: inclusion_proof(&grant_log, 4).unwrap(),
        proof_ref: "proof:grant-revocation-receipt".into(),
    });

    let early = frontier("frontier:early", vec![grants_10.clone(), payments_10.clone()]);
    let crossed = frontier("frontier:crossed", vec![grants_20.clone(), payments_10.clone()]);
    let later = frontier("frontier:later", vec![grants_20.clone(), payments_20.clone()]);

    let grant_link = consistency_link("owner:grants", 10, 20);
    let payment_link = consistency_link("owner:payments", 10, 20);

    println!(
        "early_vs_later={:?}",
        early.compare(&later, &[grant_link.clone(), payment_link.clone()])
    );
    println!(
        "early_vs_crossed={:?}",
        early.compare(&crossed, &[grant_link])
    );

    let governance_3 = owner_cut("owner:governance", 3);
    let receipt_frontier = frontier(
        "frontier:receipt",
        vec![grants_20.clone(), governance_3.clone()],
    );
    println!(
        "receipt_visibility={:?}",
        receipt_frontier.classify_receipt(&receipt)
    );

    let gaps = vec![SequenceRangeV1 { first: 3, last: 4 }];
    let mut incomplete_entries = owner_entries_with_gaps("owner:grants", 20, &gaps);
    let incomplete_leaf_index = 2_u64; // owner sequences 1, 2, 5, ...
    incomplete_entries[incomplete_leaf_index as usize] =
        canonical_knowledge_receipt_entry_v1(&receipt).unwrap();
    let incomplete_owner_root = tree_hash(&incomplete_entries).unwrap();
    let mut incomplete_cut = grants_20;
    incomplete_cut.sequence = 20;
    incomplete_cut.tree_size = incomplete_entries.len() as u64;
    incomplete_cut.root_digest = format!("sha256:{}", hex(&incomplete_owner_root));
    incomplete_cut.gaps = gaps;
    let mut incomplete_receipt = receipt.clone();
    incomplete_receipt.inclusion = Some(ReceiptInclusionDeclarationV1 {
        owner_root_digest: incomplete_cut.root_digest.clone(),
        leaf_index: incomplete_leaf_index,
        tree_size: incomplete_cut.tree_size,
        inclusion_path: inclusion_proof(
            &incomplete_entries, incomplete_leaf_index,
        ).unwrap(),
        proof_ref: "proof:receipt-with-gap".into(),
    });
    let incomplete = frontier(
        "frontier:incomplete",
        vec![incomplete_cut, governance_3],
    );
    println!(
        "receipt_with_gap={:?}",
        incomplete.classify_receipt(&incomplete_receipt)
    );

    assert_eq!(
        early.compare(
            &later,
            &[consistency_link("owner:grants", 10, 20), payment_link]
        ),
        Ok(FrontierRelationV1::Before)
    );
    assert_eq!(
        early.compare(
            &crossed,
            &[
                consistency_link("owner:grants", 10, 20),
                consistency_link("owner:payments", 10, 20),
            ]
        ),
        Ok(FrontierRelationV1::Concurrent)
    );
    assert_eq!(
        receipt_frontier.classify_receipt(&receipt),
        Ok(ReceiptVisibilityV1::MerkleInclusionVerified)
    );
    assert_eq!(
        incomplete.classify_receipt(&receipt),
        Ok(ReceiptVisibilityV1::Indeterminate)
    );
}

fn sha2_digest(bytes: &[u8]) -> [u8; 32] {
    mycelix_rfc9162_proof_verifier::sha256(bytes)
}
