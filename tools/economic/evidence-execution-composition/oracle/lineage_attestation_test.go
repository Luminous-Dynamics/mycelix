package oracle

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"os"
	"strconv"
	"strings"
	"testing"
)

func decodeLineageDigest(value string) ([32]byte, error) {
	var digest [32]byte
	raw, err := hex.DecodeString(value)
	if err != nil {
		return digest, err
	}
	if len(raw) != len(digest) {
		return digest, strconv.ErrSyntax
	}
	copy(digest[:], raw)
	return digest, nil
}

func parseLineageAttestationVector(fields []string) (LineageAttestationV1, []byte, error) {
	if len(fields) != 22 {
		return LineageAttestationV1{}, nil, strconv.ErrSyntax
	}
	parseUint := func(value string) (uint64, error) {
		return strconv.ParseUint(value, 10, 64)
	}
	epoch, err := parseUint(fields[4])
	if err != nil { return LineageAttestationV1{}, nil, err }
	issued, err := parseUint(fields[16])
	if err != nil { return LineageAttestationV1{}, nil, err }
	validFrom, err := parseUint(fields[17])
	if err != nil { return LineageAttestationV1{}, nil, err }
	validUntil, err := parseUint(fields[18])
	if err != nil { return LineageAttestationV1{}, nil, err }

	digests := make([][32]byte, 7)
	for i, field := range []int{6, 7, 9, 10, 12, 13, 15} {
		digests[i], err = decodeLineageDigest(fields[field])
		if err != nil { return LineageAttestationV1{}, nil, err }
	}
	attestation := LineageAttestationV1{
		AttestationRef: fields[1], IssuerRef: fields[2], IssuerKeyRef: fields[3],
		IssuerKeyEpoch: epoch, EvidenceRef: fields[5],
		ScopeDigest: digests[0], GraphDigest: digests[1], ProfileRef: fields[8],
		ProfileDigest: digests[2], VerifierFrontierDigest: digests[3],
		TrustPolicyRef: fields[11], TrustPolicyDigest: digests[4],
		IdentityRegistryDigest: digests[5], AssuranceBasisRef: fields[14],
		StatementDigest: digests[6], IssuedAtUnixSeconds: issued,
		ValidFromUnixSeconds: validFrom, ValidUntilUnixSeconds: validUntil,
		DisclosureStatus: LineageDisclosureStatus(fields[19]),
		AssuranceBasis: LineageAttestationBasis(fields[20]),
	}
	expected, err := hex.DecodeString(fields[21])
	return attestation, expected, err
}

func TestSharedGoRustLineageAttestationCanonicalizationVector(t *testing.T) {
	file, err := os.Open("../LINEAGE_ATTESTATION_V1_KAT.tsv")
	if err != nil { t.Fatal(err) }
	defer file.Close()

	scanner := bufio.NewScanner(file)
	seen := 0
	for line := 1; scanner.Scan(); line++ {
		raw := scanner.Text()
		if raw == "" || strings.HasPrefix(raw, "#") { continue }
		fields := strings.Split(raw, "\t")
		attestation, expected, err := parseLineageAttestationVector(fields)
		if err != nil { t.Fatalf("line %d: %v", line, err) }
		actual, err := CanonicalLineageAttestationBytesV1(attestation)
		if err != nil { t.Fatalf("line %d (%s): %v", line, fields[0], err) }
		if !bytes.Equal(actual, expected) {
			t.Errorf("line %d (%s): preimage mismatch\n got %s\nwant %s",
				line, fields[0], hex.EncodeToString(actual), hex.EncodeToString(expected))
		}
		seen++
	}
	if err := scanner.Err(); err != nil { t.Fatal(err) }
	if seen != 1 { t.Fatalf("expected one attestation vector, evaluated %d", seen) }
}

func TestLineageAttestationCanonicalizerRejectsInvalidStatesAndValidity(t *testing.T) {
	valid := LineageAttestationV1{
		AttestationRef: "attestation:test", IssuerRef: "issuer:test",
		IssuerKeyRef: "key:test", EvidenceRef: "evidence:test",
		ProfileRef: "profile:test", TrustPolicyRef: "policy:test",
		AssuranceBasisRef: "audit:test",
		ValidFromUnixSeconds: 10, ValidUntilUnixSeconds: 20,
		DisclosureStatus: LineageDisclosureComplete,
		AssuranceBasis: LineageBasisIndependentlyAudited,
	}
	valid.DisclosureStatus = LineageDisclosureStatus("unknown")
	if _, err := CanonicalLineageAttestationBytesV1(valid); err == nil {
		t.Fatal("unknown disclosure status accepted")
	}
	valid.DisclosureStatus = LineageDisclosureComplete
	valid.AssuranceBasis = LineageAttestationBasis("unknown")
	if _, err := CanonicalLineageAttestationBytesV1(valid); err == nil {
		t.Fatal("unknown assurance basis accepted")
	}
	valid.AssuranceBasis = LineageBasisIndependentlyAudited
	valid.ValidUntilUnixSeconds = valid.ValidFromUnixSeconds
	if _, err := CanonicalLineageAttestationBytesV1(valid); err == nil {
		t.Fatal("empty validity interval accepted")
	}
}
