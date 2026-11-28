package oracle

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"unicode/utf8"
)

type LineageDisclosureStatus string

type LineageAttestationBasis string

const (
	LineageDisclosureComplete  LineageDisclosureStatus = "complete"
	LineageDisclosureIncomplete LineageDisclosureStatus = "incomplete"
	LineageDisclosureWithheld   LineageDisclosureStatus = "withheld"

	LineageBasisIssuerDeclared LineageAttestationBasis = "issuer-declared"
	LineageBasisIndependentlyAudited LineageAttestationBasis = "independently-audited"
)

type LineageAttestationV1 struct {
	AttestationRef          string
	IssuerRef               string
	IssuerKeyRef            string
	IssuerKeyEpoch          uint64
	EvidenceRef             string
	ScopeDigest             [32]byte
	GraphDigest             [32]byte
	ProfileRef              string
	ProfileDigest           [32]byte
	VerifierFrontierDigest  [32]byte
	TrustPolicyRef          string
	TrustPolicyDigest       [32]byte
	IdentityRegistryDigest  [32]byte
	AssuranceBasisRef       string
	StatementDigest         [32]byte
	IssuedAtUnixSeconds     uint64
	ValidFromUnixSeconds    uint64
	ValidUntilUnixSeconds   uint64
	DisclosureStatus        LineageDisclosureStatus
	AssuranceBasis          LineageAttestationBasis
}

func writeLineageStringV1(buffer *bytes.Buffer, value string) error {
	if !utf8.ValidString(value) {
		return errors.New("lineage attestation reference is not valid UTF-8")
	}
	if stringsTrimmedEmpty(value) || len([]byte(value)) > 4096 {
		return errors.New("lineage attestation reference is empty or oversized")
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len([]byte(value))))
	_, _ = buffer.Write(length[:])
	_, _ = buffer.WriteString(value)
	return nil
}

func stringsTrimmedEmpty(value string) bool {
	for _, r := range value {
		if r != ' ' && r != '\t' && r != '\n' && r != '\r' {
			return false
		}
	}
	return true
}

// CanonicalLineageAttestationBytesV1 returns the signing preimage for DJCC's
// lineage attestation. It does not hash the preimage, sign it, or verify COSE.
func CanonicalLineageAttestationBytesV1(attestation LineageAttestationV1) ([]byte, error) {
	if attestation.ValidFromUnixSeconds >= attestation.ValidUntilUnixSeconds {
		return nil, errors.New("lineage attestation validity interval is empty or reversed")
	}
	switch attestation.DisclosureStatus {
	case LineageDisclosureComplete, LineageDisclosureIncomplete, LineageDisclosureWithheld:
	default:
		return nil, fmt.Errorf("unknown lineage disclosure status %q", attestation.DisclosureStatus)
	}
	switch attestation.AssuranceBasis {
	case LineageBasisIssuerDeclared, LineageBasisIndependentlyAudited:
	default:
		return nil, fmt.Errorf("unknown lineage assurance basis %q", attestation.AssuranceBasis)
	}

	var output bytes.Buffer
	if err := writeLineageStringV1(&output, "MYCELIX-DJCC-LINEAGE-ATTESTATION-V1"); err != nil {
		return nil, err
	}
	for _, value := range []string{
		attestation.AttestationRef,
		attestation.IssuerRef,
		attestation.IssuerKeyRef,
		attestation.EvidenceRef,
		attestation.ProfileRef,
		attestation.TrustPolicyRef,
		attestation.AssuranceBasisRef,
	} {
		if err := writeLineageStringV1(&output, value); err != nil {
			return nil, err
		}
	}
	var u64 [8]byte
	binary.BigEndian.PutUint64(u64[:], attestation.IssuerKeyEpoch)
	_, _ = output.Write(u64[:])
	for _, digest := range [][32]byte{
		attestation.ScopeDigest,
		attestation.GraphDigest,
		attestation.ProfileDigest,
		attestation.VerifierFrontierDigest,
		attestation.TrustPolicyDigest,
		attestation.IdentityRegistryDigest,
		attestation.StatementDigest,
	} {
		_, _ = output.Write(digest[:])
	}
	for _, timestamp := range []uint64{
		attestation.IssuedAtUnixSeconds,
		attestation.ValidFromUnixSeconds,
		attestation.ValidUntilUnixSeconds,
	} {
		binary.BigEndian.PutUint64(u64[:], timestamp)
		_, _ = output.Write(u64[:])
	}
	disclosureTag := byte(0)
	switch attestation.DisclosureStatus {
	case LineageDisclosureComplete:
		disclosureTag = 1
	case LineageDisclosureIncomplete:
		disclosureTag = 2
	case LineageDisclosureWithheld:
		disclosureTag = 3
	}
	assuranceTag := byte(0)
	switch attestation.AssuranceBasis {
	case LineageBasisIssuerDeclared:
		assuranceTag = 1
	case LineageBasisIndependentlyAudited:
		assuranceTag = 2
	}
	_ = output.WriteByte(disclosureTag)
	_ = output.WriteByte(assuranceTag)
	return output.Bytes(), nil
}
