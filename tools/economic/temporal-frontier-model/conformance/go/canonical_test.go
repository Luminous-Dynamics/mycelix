package conformance

import (
	"bytes"
	"encoding/hex"
	"testing"
)

func TestOwnerCutCanonicalGolden(t *testing.T) {
	cut := OwnerCut{
		VerifierRef:    "did:mycelix:verifier:finance-v1",
		PolicyEpochRef: "policy-epoch:1",
		OwnerRef:       "owner:grants",
		HistoryRef:     "history:owner:grants",
		Sequence:       10,
		TreeSize:       8,
		RootDigest:     "sha256:" + string(bytes.Repeat([]byte{'a'}, 64)),
		Gaps:           []SequenceRange{{First: 3, Last: 4}},
	}
	encoded, err := CanonicalOwnerCutEntryV1(cut)
	if err != nil {
		t.Fatal(err)
	}
	if err := VerifyGoldenDigest(encoded, "sha256:8415d807e0dbe3b5bc4b39914b0a408fbf88ea65edf7ce6994b66bd6f303acec"); err != nil {
		t.Fatal(err)
	}
}

func TestKnowledgeReceiptCanonicalGoldenAndDependencyOrder(t *testing.T) {
	receipt := KnowledgeReceipt{
		ReceiptID:       "receipt:5",
		VerifierRef:     "did:mycelix:verifier:finance-v1",
		PolicyEpochRef:  "policy-epoch:1",
		AssertionID:     "assertion:revocation",
		AssertionDigest: "sha256:" + string(bytes.Repeat([]byte{'b'}, 64)),
		OwnerRef:        "owner:grants",
		OwnerSequence:   5,
		CausalDependency: []OwnerPosition{
			{OwnerRef: "owner:payments", Sequence: 2},
			{OwnerRef: "owner:governance", Sequence: 3},
		},
	}
	encoded, err := CanonicalKnowledgeReceiptEntryV1(receipt)
	if err != nil {
		t.Fatal(err)
	}
	if err := VerifyGoldenDigest(encoded, "sha256:66ec5ae10377119451008e8b6a9b782477f06cc71152a805188990828176885e"); err != nil {
		t.Fatal(err)
	}
	reversed := receipt
	reversed.CausalDependency = []OwnerPosition{
		{OwnerRef: "owner:governance", Sequence: 3},
		{OwnerRef: "owner:payments", Sequence: 2},
	}
	reversedEncoded, err := CanonicalKnowledgeReceiptEntryV1(reversed)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(encoded, reversedEncoded) {
		t.Fatal("dependency permutation changed canonical receipt bytes")
	}
}

func TestRFC9162SevenLeafProofsAndCrossLanguageRoots(t *testing.T) {
	entries := make([][]byte, 7)
	for i := range entries {
		entries[i] = []byte{byte('a' + i)}
	}
	root := TreeHash(entries)
	if got := "sha256:" + hex.EncodeToString(root[:]); got != "sha256:4ae191939f548d9934740b88dea2c5cb89bb8870fc4505cd79dec6bbfaaee9cb" {
		t.Fatalf("root mismatch: %s", got)
	}

	proof, err := InclusionProof(entries, 6)
	if err != nil {
		t.Fatal(err)
	}
	if len(proof) != 2 {
		t.Fatalf("RFC 9162 PATH(6,D7) should contain 2 nodes; got %d", len(proof))
	}
	if err := VerifyEntryInclusion(entries[6], 6, 7, proof, root); err != nil {
		t.Fatal(err)
	}

	oldRoot := TreeHash(entries[:3])
	consistency, err := ConsistencyProof(entries, 3)
	if err != nil {
		t.Fatal(err)
	}
	if len(consistency) != 4 {
		t.Fatalf("RFC 9162 PROOF(3,D7) should contain 4 nodes; got %d", len(consistency))
	}
	if err := VerifyConsistency(3, oldRoot, 7, root, consistency); err != nil {
		t.Fatal(err)
	}
}

func TestExhaustiveBoundedInclusionAndConsistency(t *testing.T) {
	for size := 1; size <= 64; size++ {
		entries := make([][]byte, size)
		for i := range entries {
			entries[i] = []byte{byte(i >> 8), byte(i)}
		}
		root := TreeHash(entries)
		for index := range entries {
			path, err := InclusionProof(entries, uint64(index))
			if err != nil {
				t.Fatal(err)
			}
			if err := VerifyEntryInclusion(entries[index], uint64(index), uint64(size), path, root); err != nil {
				t.Fatalf("inclusion size=%d index=%d: %v", size, index, err)
			}
		}
		for oldSize := 0; oldSize <= size; oldSize++ {
			oldRoot := TreeHash(entries[:oldSize])
			path, err := ConsistencyProof(entries, uint64(oldSize))
			if err != nil {
				t.Fatal(err)
			}
			if err := VerifyConsistency(uint64(oldSize), oldRoot, uint64(size), root, path); err != nil {
				t.Fatalf("consistency %d->%d: %v", oldSize, size, err)
			}
		}
	}
}

func TestTamperedInclusionAndConsistencyProofsFail(t *testing.T) {
	entries := [][]byte{[]byte("a"), []byte("b"), []byte("c"), []byte("d"), []byte("e")}
	root := TreeHash(entries)
	path, err := InclusionProof(entries, 3)
	if err != nil {
		t.Fatal(err)
	}
	path[0] = SHA256([]byte("tampered"))
	if err := VerifyEntryInclusion(entries[3], 3, 5, path, root); err == nil {
		t.Fatal("tampered inclusion proof unexpectedly passed")
	}

	consistency, err := ConsistencyProof(entries, 2)
	if err != nil {
		t.Fatal(err)
	}
	consistency[0] = SHA256([]byte("tampered"))
	if err := VerifyConsistency(2, TreeHash(entries[:2]), 5, root, consistency); err == nil {
		t.Fatal("tampered consistency proof unexpectedly passed")
	}
}
