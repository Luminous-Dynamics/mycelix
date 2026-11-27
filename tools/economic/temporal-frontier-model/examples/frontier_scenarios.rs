use mycelix_temporal_frontier_model::{
    CheckpointInclusionDeclarationV1, FrontierRelationV1, KnowledgeReceiptV1,
    OwnerConsistencyLinkV1, OwnerCutV1, ReceiptInclusionDeclarationV1,
    ReceiptVisibilityV1, VerifierFrontierV1,
};

const VERIFIER: &str = "did:mycelix:verifier:finance-v1";

fn digest(byte: char) -> String {
    format!("sha256:{}", byte.to_string().repeat(64))
}

fn owner_cut(owner: &str, sequence: u64, root: char) -> OwnerCutV1 {
    OwnerCutV1 {
        owner_ref: owner.into(),
        history_ref: format!("history:{owner}"),
        sequence,
        root_digest: digest(root),
        checkpoint_inclusion: CheckpointInclusionDeclarationV1 {
            checkpoint_root_digest: digest('f'),
            owner_root_digest: digest(root),
            proof_ref: "proof:declared-owner-checkpoint-binding".into(),
        },
        gaps: Vec::new(),
    }
}

fn frontier(id: &str, mut cuts: Vec<OwnerCutV1>) -> VerifierFrontierV1 {
    // Synthetic fixture value only; not a cryptographic aggregate-root hash.
    let aggregate_value = id
        .bytes()
        .fold(0_u64, |value, byte| value.wrapping_mul(31).wrapping_add(u64::from(byte)));
    let checkpoint_root_digest = format!("sha256:{aggregate_value:064x}");
    for cut in &mut cuts {
        cut.checkpoint_inclusion.checkpoint_root_digest = checkpoint_root_digest.clone();
    }
    VerifierFrontierV1 {
        frontier_id: id.into(),
        verifier_ref: VERIFIER.into(),
        policy_epoch_ref: "policy-epoch:1".into(),
        checkpoint_root_digest,
        parent_frontier_ids: Vec::new(),
        owner_cuts: cuts,
    }
}

fn main() {
    let grants_10 = owner_cut("owner:grants", 10, 'a');
    let grants_20 = owner_cut("owner:grants", 20, 'b');
    let payments_20 = owner_cut("owner:payments", 20, 'c');
    let payments_10 = owner_cut("owner:payments", 10, 'd');

    let early = frontier("frontier:early", vec![grants_10.clone(), payments_10.clone()]);
    let crossed = frontier("frontier:crossed", vec![grants_20.clone(), payments_10.clone()]);
    let later = frontier("frontier:later", vec![grants_20.clone(), payments_20.clone()]);

    let grants_link = OwnerConsistencyLinkV1 {
        owner_ref: "owner:grants".into(),
        history_ref: "history:owner:grants".into(),
        ancestor_root_digest: grants_10.root_digest.clone(),
        descendant_root_digest: grants_20.root_digest.clone(),
        consistency_proof_ref: "proof:declared-grants-extension".into(),
    };
    let payments_link = OwnerConsistencyLinkV1 {
        owner_ref: "owner:payments".into(),
        history_ref: "history:owner:payments".into(),
        ancestor_root_digest: payments_10.root_digest.clone(),
        descendant_root_digest: payments_20.root_digest.clone(),
        consistency_proof_ref: "proof:declared-payments-extension".into(),
    };

    println!(
        "early_vs_later={:?}",
        early.compare(&later, &[grants_link.clone(), payments_link.clone()])
    );
    println!(
        "early_vs_crossed={:?}",
        early.compare(&crossed, &[grants_link])
    );

    let receipt_digest = digest('e');
    let receipt = KnowledgeReceiptV1 {
        receipt_id: "receipt:grant-revocation:5".into(),
        verifier_ref: VERIFIER.into(),
        assertion_id: "assertion:grant-revocation".into(),
        assertion_digest: digest('b'),
        owner_ref: "owner:grants".into(),
        owner_sequence: 5,
        causal_dependencies: Vec::new(),
        receipt_digest: receipt_digest.clone(),
        inclusion: Some(ReceiptInclusionDeclarationV1 {
            owner_root_digest: grants_20.root_digest.clone(),
            receipt_leaf_digest: receipt_digest,
            leaf_index: 4,
            tree_size: 20,
            proof_ref: "proof:declared-receipt-inclusion".into(),
        }),
    };
    let receipt_frontier = frontier("frontier:receipt", vec![grants_20.clone()]);
    println!(
        "receipt_visibility={:?}",
        receipt_frontier.classify_receipt(&receipt)
    );

    let mut incomplete_cut = grants_20;
    incomplete_cut.gaps.push(
        mycelix_temporal_frontier_model::SequenceRangeV1 { first: 3, last: 4 }
    );
    let incomplete = frontier("frontier:incomplete", vec![incomplete_cut]);
    println!(
        "receipt_with_gap={:?}",
        incomplete.classify_receipt(&receipt)
    );

    assert_eq!(
        early.compare(
            &later,
            &[
                OwnerConsistencyLinkV1 {
                    owner_ref: "owner:grants".into(),
                    history_ref: "history:owner:grants".into(),
                    ancestor_root_digest: digest('a'),
                    descendant_root_digest: digest('b'),
                    consistency_proof_ref: "proof:grants".into(),
                },
                payments_link,
            ]
        ),
        Ok(FrontierRelationV1::Before)
    );
    assert_eq!(
        early.compare(&crossed, &[]),
        Ok(FrontierRelationV1::Unknown)
    );
    assert_eq!(
        receipt_frontier.classify_receipt(&receipt),
        Ok(ReceiptVisibilityV1::DeclaredIncluded)
    );
    assert_eq!(
        incomplete.classify_receipt(&receipt),
        Ok(ReceiptVisibilityV1::Indeterminate)
    );
}