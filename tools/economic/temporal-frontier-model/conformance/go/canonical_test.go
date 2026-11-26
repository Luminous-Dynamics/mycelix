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
	if got := hex.EncodeToString(encoded); got != "4d5943454c49582d4f574e45522d4355542d5631000000001f6469643a6d7963656c69783a76657269666965723a66696e616e63652d76310000000e706f6c6963792d65706f63683a310000000c6f776e65723a6772616e747300000014686973746f72793a6f776e65723a6772616e7473000000000000000a0000000000000008000000477368613235363a616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161616161610000000100000000000000030000000000000004" {
		t.Fatalf("owner-cut bytes mismatch: %s", got)
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
	if got := hex.EncodeToString(encoded); got != "4d5943454c49582d4b4e4f574c454447452d524543454950542d56310000000009726563656970743a350000001f6469643a6d7963656c69783a76657269666965723a66696e616e63652d76310000000e706f6c6963792d65706f63683a3100000014617373657274696f6e3a7265766f636174696f6e000000477368613235363a626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262626262620000000c6f776e65723a6772616e7473000000000000000500000002000000106f776e65723a676f7665726e616e636500000000000000030000000e6f776e65723a7061796d656e74730000000000000002" {
		t.Fatalf("receipt bytes mismatch: %s", got)
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
	wantInclusion := []string{
		"918566184c9d5be235ad2b6dd60828f5cec14fc409f02f7db8647009ec6da588",
		"33376a3bd63e9993708a84ddfe6c28ae58b83505dd1fed711bd924ec5a6239f0",
	}
	if len(proof) != len(wantInclusion) {
		t.Fatalf("RFC 9162 PATH(6,D7) should contain 2 nodes; got %d", len(proof))
	}
	for i, node := range proof {
		if got := hex.EncodeToString(node[:]); got != wantInclusion[i] {
			t.Fatalf("PATH(6,D7) node %d mismatch: %s", i, got)
		}
	}
	if err := VerifyEntryInclusion(entries[6], 6, 7, proof, root); err != nil {
		t.Fatal(err)
	}

	oldRoot := TreeHash(entries[:3])
	if got := hex.EncodeToString(oldRoot[:]); got != "36642e73c2540ab121e3a6bf9545b0a24982cd830eb13d3cd19de3ce6c021ec1" {
		t.Fatalf("three-leaf root mismatch: %s", got)
	}
	consistency, err := ConsistencyProof(entries, 3)
	if err != nil {
		t.Fatal(err)
	}
	wantConsistency := []string{
		"597fcb31282d34654c200d3418fca5705c648ebf326ec73d8ddef11841f876d8",
		"d070dc5b8da9aea7dc0f5ad4c29d89965200059c9a0ceca3abd5da2492dcb71d",
		"b137985ff484fb600db93107c77b0365c80d78f5b429ded0fd97361d077999eb",
		"e286d3390665a7cdc759453bed0b00cded1842d757e3e6cfe87df53db177e725",
	}
	if len(consistency) != len(wantConsistency) {
		t.Fatalf("RFC 9162 PROOF(3,D7) should contain 4 nodes; got %d", len(consistency))
	}
	for i, node := range consistency {
		if got := hex.EncodeToString(node[:]); got != wantConsistency[i] {
			t.Fatalf("PROOF(3,D7) node %d mismatch: %s", i, got)
		}
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
