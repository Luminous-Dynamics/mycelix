package temporalrange

import (
	"bytes"
	"encoding/binary"
	"testing"
)

func proofWireFixtureV1(t *testing.T) (IntervalScopeV1, IntervalTreeV1, IntervalPointProofV1, IntervalInstantV1) {
	t.Helper()
	scope := intervalScopeFixtureV1()
	records := []IntervalRecordV1{
		intervalRecordFixtureV1("a", 1, 20),
		{AssertionRef: "b-open", ValidFrom: intervalPointV1(2, 125), ValidTo: intervalOpenV1(), ReceiptDigest: intervalDigestV1(10)},
		intervalRecordFixtureV1("c-expired", 0, 4),
		intervalRecordFixtureV1("d-future", 30, 50),
		intervalRecordFixtureV1("e-overlap", 3, 40),
	}
	tree, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(10, 0)
	proof, err := tree.ProvePointQueryV1(query)
	if err != nil {
		t.Fatal(err)
	}
	return scope, *tree, proof, query
}

func TestIntervalProofWireRoundTripAndCanonicalBytes(t *testing.T) {
	scope, tree, proof, query := proofWireFixtureV1(t)
	encoded, err := MarshalIntervalPointProofV1(proof)
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := UnmarshalIntervalPointProofV1(encoded)
	if err != nil {
		t.Fatal(err)
	}
	reencoded, err := MarshalIntervalPointProofV1(decoded)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(encoded, reencoded) {
		t.Fatal("canonical wire encoding changed after decode/re-encode")
	}
	original, err := VerifyIntervalPointProofV1(scope, query, proof, tree.RootHash())
	if err != nil {
		t.Fatal(err)
	}
	roundTrip, err := VerifyIntervalPointProofV1(scope, query, decoded, tree.RootHash())
	if err != nil {
		t.Fatal(err)
	}
	if len(original.Records) != len(roundTrip.Records) {
		t.Fatalf("round-trip result size mismatch: %d != %d", len(original.Records), len(roundTrip.Records))
	}
	for i := range original.Records {
		if original.Records[i] != roundTrip.Records[i] {
			t.Fatalf("round-trip record mismatch at %d", i)
		}
	}
}

func TestIntervalProofWireEmptyResultRoundTrip(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("future", 20, 30)})
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(10, 0)
	proof, err := tree.ProvePointQueryV1(query)
	if err != nil {
		t.Fatal(err)
	}
	encoded, err := MarshalIntervalPointProofV1(proof)
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := UnmarshalIntervalPointProofV1(encoded)
	if err != nil {
		t.Fatal(err)
	}
	result, err := VerifyIntervalPointProofV1(scope, query, decoded, tree.RootHash())
	if err != nil {
		t.Fatal(err)
	}
	if result.Disposition != CompleteEmptyIntervalResultForRoot || len(result.Records) != 0 {
		t.Fatalf("wrong empty query result: %+v", result)
	}
}

func TestIntervalProofWireRejectsMalformedFramingAndBounds(t *testing.T) {
	_, _, proof, _ := proofWireFixtureV1(t)
	encoded, err := MarshalIntervalPointProofV1(proof)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := UnmarshalIntervalPointProofV1(encoded[:len(encoded)-1]); err == nil {
		t.Fatal("truncated proof passed")
	}
	withTrailing := append(append([]byte(nil), encoded...), 0)
	if _, err := UnmarshalIntervalPointProofV1(withTrailing); err == nil {
		t.Fatal("trailing byte passed")
	}
	wrongMagic := append([]byte(nil), encoded...)
	wrongMagic[0] ^= 0x80
	if _, err := UnmarshalIntervalPointProofV1(wrongMagic); err == nil {
		t.Fatal("wrong proof magic passed")
	}
	oversizedString := append([]byte(nil), encoded...)
	binary.BigEndian.PutUint32(oversizedString[8:12], 1_025)
	if _, err := UnmarshalIntervalPointProofV1(oversizedString); err == nil {
		t.Fatal("over-limit scope string passed")
	}

	offset := len(intervalProofWireMagicV1)
	for _, value := range []string{
		proof.Scope.VerifierRef, proof.Scope.PolicyEpochRef, proof.Scope.NamespaceRef,
		proof.Scope.SubjectRef, proof.Scope.ClaimID, proof.Scope.SourceFrontierDigest,
	} {
		offset += 4 + len(value)
	}
	oversizedCount := append([]byte(nil), encoded...)
	binary.BigEndian.PutUint64(oversizedCount[offset+12:offset+20], maxIntervalProofEntriesV1+1)
	if _, err := UnmarshalIntervalPointProofV1(oversizedCount); err == nil {
		t.Fatal("over-limit proof record count passed")
	}
}

func TestIntervalProofWireRejectsMalformedEndTagAndNanoseconds(t *testing.T) {
	var endBytes bytes.Buffer
	writeWireEndV1(&endBytes, intervalOpenV1())
	if got, err := readWireEndV1(bytes.NewReader(endBytes.Bytes())); err != nil || !got.Open {
		t.Fatalf("canonical open end failed: %+v %v", got, err)
	}
	if _, err := readWireEndV1(bytes.NewReader([]byte{2})); err == nil {
		t.Fatal("unknown end-bound tag passed")
	}
	var instantBytes bytes.Buffer
	writeWireInstantV1(&instantBytes, intervalPointV1(5, 1_000_000_000))
	if _, err := readWireInstantV1(bytes.NewReader(instantBytes.Bytes())); err == nil {
		t.Fatal("nanosecond overflow passed")
	}
}

func TestIntervalProofWireDoesNotEmbedOrAuthenticateExpectedRoot(t *testing.T) {
	scope, tree, proof, query := proofWireFixtureV1(t)
	encoded, err := MarshalIntervalPointProofV1(proof)
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := UnmarshalIntervalPointProofV1(encoded)
	if err != nil {
		t.Fatal(err)
	}
	wrongRoot := [32]byte{0x5A}
	if _, err := VerifyIntervalPointProofV1(scope, query, decoded, wrongRoot); err != ErrIntervalRootMismatch {
		t.Fatalf("untrusted external root substitution: %v", err)
	}
	if _, err := VerifyIntervalPointProofV1(scope, query, decoded, tree.RootHash()); err != nil {
		t.Fatal(err)
	}
}
