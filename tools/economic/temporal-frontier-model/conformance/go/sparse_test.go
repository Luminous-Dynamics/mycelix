package conformance

import (
	"crypto/sha256"
	"encoding/hex"
	"strings"
	"testing"
)

func sparseKeyWithFirstByte(value byte) SparseKey {
	var key SparseKey
	key[0] = value
	return key
}

func sparseValueWithFirstByte(value byte) SparseValue {
	var out SparseValue
	out[0] = value
	return out
}

func TestSparseMerkleKnownAnswerRootAndNonMembershipProof(t *testing.T) {
	tree := NewSparseTree()
	fixtures := []struct {
		key   SparseKey
		value SparseValue
	}{
		{sparseKeyWithFirstByte(0x00), sparseValueWithFirstByte(0xA1)},
		{sparseKeyWithFirstByte(0x80), sparseValueWithFirstByte(0xA2)},
		{sparseKeyWithFirstByte(0xFF), sparseValueWithFirstByte(0xA3)},
	}
	for _, fixture := range fixtures {
		if err := tree.Insert(fixture.key, fixture.value); err != nil {
			t.Fatal(err)
		}
	}
	root := tree.RootHash()
	if got := hex.EncodeToString(root[:]); got != "ca399e0f53e5b7607402225d68a14e607cc7e7e488e44ccf8bde194e6386718f" {
		t.Fatalf("sparse root mismatch: %s", got)
	}

	for index, fixture := range fixtures {
		proof := tree.Prove(fixture.key)
		if len(proof.SiblingsLeafToRoot) != SparseMerkleDepthV1 {
			t.Fatalf("present proof %d has %d siblings", index, len(proof.SiblingsLeafToRoot))
		}
		if err := VerifySparseMembership(fixture.key, fixture.value, proof, root); err != nil {
			t.Fatalf("present proof %d: %v", index, err)
		}
	}

	absentKey := sparseKeyWithFirstByte(0x40)
	proof := tree.Prove(absentKey)
	if len(proof.SiblingsLeafToRoot)*32 != 8192 {
		t.Fatalf("uncompressed sparse proof bytes mismatch: %d", len(proof.SiblingsLeafToRoot)*32)
	}
	if err := VerifySparseNonMembership(absentKey, proof, root); err != nil {
		t.Fatalf("non-membership proof: %v", err)
	}
	encoded := make([]byte, 0, len(proof.SiblingsLeafToRoot)*32)
	for _, sibling := range proof.SiblingsLeafToRoot {
		encoded = append(encoded, sibling[:]...)
	}
	proofDigest := sha256.Sum256(encoded)
	if got := hex.EncodeToString(proofDigest[:]); got != "dc05c01529111d01b12e06f0ddb2a9b22d78822352ba8425c6768e3e3548766f" {
		t.Fatalf("non-membership proof vector mismatch: %s", got)
	}
}

func TestSparseMerkleEmptyRootVector(t *testing.T) {
	tree := NewSparseTree()
	root := tree.RootHash()
	if got := hex.EncodeToString(root[:]); got != "de3054ab8d693834a8b74a5f07765258eb95da1c61faaf16ce437659142ee250" {
		t.Fatalf("empty sparse root mismatch: %s", got)
	}
	key := sparseKeyWithFirstByte(0x81)
	if err := VerifySparseNonMembership(key, tree.Prove(key), root); err != nil {
		t.Fatal(err)
	}
}

func TestSparseMerkleExhaustiveSmallMaps(t *testing.T) {
	for count := 0; count <= 64; count++ {
		tree := NewSparseTree()
		for index := 0; index < count; index++ {
			var key SparseKey
			key[0] = byte(index >> 8)
			key[1] = byte(index)
			value := sparseValueWithFirstByte(byte(index + 1))
			if err := tree.Insert(key, value); err != nil {
				t.Fatal(err)
			}
		}
		root := tree.RootHash()
		for index := 0; index < count; index++ {
			var key SparseKey
			key[0] = byte(index >> 8)
			key[1] = byte(index)
			value := sparseValueWithFirstByte(byte(index + 1))
			if err := VerifySparseMembership(key, value, tree.Prove(key), root); err != nil {
				t.Fatalf("count=%d member=%d: %v", count, index, err)
			}
		}
		for index := 128; index < 136; index++ {
			var key SparseKey
			key[0] = byte(index >> 8)
			key[1] = byte(index)
			if err := VerifySparseNonMembership(key, tree.Prove(key), root); err != nil {
				t.Fatalf("count=%d nonmember=%d: %v", count, index, err)
			}
		}
	}
}

func TestSparseMerkleRejectsWrongKeyValueRootAndTamperedProof(t *testing.T) {
	tree := NewSparseTree()
	key := sparseKeyWithFirstByte(0x81)
	value := sparseValueWithFirstByte(0xCA)
	if err := tree.Insert(key, value); err != nil {
		t.Fatal(err)
	}
	root := tree.RootHash()
	proof := tree.Prove(key)

	if err := VerifySparseMembership(key, sparseValueWithFirstByte(0xCB), proof, root); err == nil {
		t.Fatal("wrong value passed")
	}
	if err := VerifySparseMembership(sparseKeyWithFirstByte(0x82), value, proof, root); err == nil {
		t.Fatal("wrong key passed")
	}
	if err := VerifySparseMembership(key, value, proof, sparseValueWithFirstByte(0x11)); err == nil {
		t.Fatal("wrong root passed")
	}
	proof.SiblingsLeafToRoot[37][0] ^= 0x80
	if err := VerifySparseMembership(key, value, proof, root); err == nil {
		t.Fatal("tampered path passed")
	}
	if err := VerifySparseNonMembership(key, proof, root); err == nil {
		t.Fatal("present key passed as absent")
	}
}

func TestSparseQueryKeyAndValueKnownAnswers(t *testing.T) {
	key, err := AssertionIndexKeyV1(
		"did:mycelix:verifier:finance-v1",
		"policy-epoch:1",
		"subject:grant:42",
		"claim:revoked",
		"VALID_AT",
		"assertion:42",
	)
	if err != nil {
		t.Fatal(err)
	}
	if got := hex.EncodeToString(key[:]); got != "6f02bb0fe2b2c6bd9b0c2efac3f74fdc0db676eb21ac5d5f42648428b3dc1817" {
		t.Fatalf("sparse query key mismatch: %s", got)
	}

	value, err := AssertionIndexValueV1(
		"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
		"owner:verifier-log",
		7,
	)
	if err != nil {
		t.Fatal(err)
	}
	if got := hex.EncodeToString(value[:]); got != "d641a80447381fdce99b19eb3e9f4fc58000699bff05c81a3410cf174c5c2d7c" {
		t.Fatalf("sparse index value mismatch: %s", got)
	}
}

func TestSparseKeyAndValueAreDomainScoped(t *testing.T) {
	base, err := AssertionIndexKeyV1("verifier:A", "policy:1", "subject:1", "claim:valid", "VALID_AT", "assertion:1")
	if err != nil {
		t.Fatal(err)
	}
	otherVerifier, _ := AssertionIndexKeyV1("verifier:B", "policy:1", "subject:1", "claim:valid", "VALID_AT", "assertion:1")
	otherPolicy, _ := AssertionIndexKeyV1("verifier:A", "policy:2", "subject:1", "claim:valid", "VALID_AT", "assertion:1")
	otherSubject, _ := AssertionIndexKeyV1("verifier:A", "policy:1", "subject:2", "claim:valid", "VALID_AT", "assertion:1")
	otherClaim, _ := AssertionIndexKeyV1("verifier:A", "policy:1", "subject:1", "claim:revoked", "VALID_AT", "assertion:1")
	otherDimension, _ := AssertionIndexKeyV1("verifier:A", "policy:1", "subject:1", "claim:valid", "OCCURRED_AT", "assertion:1")
	otherAssertion, _ := AssertionIndexKeyV1("verifier:A", "policy:1", "subject:1", "claim:valid", "VALID_AT", "assertion:2")
	for _, got := range []SparseKey{otherVerifier, otherPolicy, otherSubject, otherClaim, otherDimension, otherAssertion} {
		if got == base {
			t.Fatal("query key omitted a required scope dimension")
		}
	}

	valueA, err := AssertionIndexValueV1("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", "owner:verifier-log", 7)
	if err != nil {
		t.Fatal(err)
	}
	valueB, err := AssertionIndexValueV1("sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", "owner:verifier-log", 7)
	if err != nil {
		t.Fatal(err)
	}
	if valueA == valueB {
		t.Fatal("receipt value digest not bound")
	}
	if _, err := AssertionIndexValueV1("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", "owner:verifier-log", 0); err == nil {
		t.Fatal("zero owner sequence passed")
	}
	if _, err := AssertionIndexValueV1("sha256:invalid", "owner:verifier-log", 7); err == nil {
		t.Fatal("malformed receipt digest passed")
	}
}

func TestSparseIndexIdentifiersAreBounded(t *testing.T) {
	tooLong := strings.Repeat("x", 1_025)
	if _, err := AssertionIndexKeyV1("verifier", "policy", tooLong, "claim", "VALID_AT", "assertion"); err == nil {
		t.Fatal("oversize sparse key identifier passed")
	}
	if _, err := AssertionIndexValueV1("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", tooLong, 1); err == nil {
		t.Fatal("oversize sparse owner identifier passed")
	}
}

func TestSparseMerkleInsertRejectsDuplicateAndReplaceIsExplicit(t *testing.T) {
	tree := NewSparseTree()
	key := sparseKeyWithFirstByte(0x33)
	first := sparseValueWithFirstByte(0x01)
	second := sparseValueWithFirstByte(0x02)
	if err := tree.Insert(key, first); err != nil {
		t.Fatal(err)
	}
	if err := tree.Insert(key, second); err == nil {
		t.Fatal("duplicate insert passed")
	}
	previous, existed := tree.Replace(key, second)
	if !existed || previous != first {
		t.Fatal("replace did not return previous mapping")
	}
	if _, existed := tree.Replace(sparseKeyWithFirstByte(0x34), first); existed {
		t.Fatal("replace inserted a missing key")
	}
	if tree.Len() != 1 {
		t.Fatal("replace changed map cardinality")
	}
	if err := VerifySparseMembership(key, second, tree.Prove(key), tree.RootHash()); err != nil {
		t.Fatal(err)
	}
}
