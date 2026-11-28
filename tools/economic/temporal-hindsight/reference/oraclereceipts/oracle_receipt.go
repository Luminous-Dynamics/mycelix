package oraclereceipts

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sort"
	"strings"
)

const (
	MaxIdentifierBytes = 4096
	MaxEvidenceReferences = 4096
	MaxAnswerBytes = 1 << 20
	MaxSignatureBytes = 16 << 10
	MaxReceiptCount = 1_000_000
	MaxReceiptEncodingBytes uint64 = 67_108_864

	DispositionAffirm AnswerDispositionV1 = "Affirm"
	DispositionReject AnswerDispositionV1 = "Reject"
	DispositionAbstain AnswerDispositionV1 = "Abstain"
	DispositionUnableToAssess AnswerDispositionV1 = "UnableToAssess"
	DispositionNoResponse AnswerDispositionV1 = "NoResponse"

	ChainStructurallyValid ChainDispositionV1 = "OracleReceiptChainStructurallyValid"
	ChainIndeterminate ChainDispositionV1 = "Indeterminate"

	NotEvaluated = "NotEvaluated"
)

type AnswerDispositionV1 string
type ChainDispositionV1 string

// OracleScopeV1 binds an oracle answer to one precise query input and
// verifier-selected historical evidence frontier.
type OracleScopeV1 struct {
	SourceRef               string
	SnapshotRef             string
	RegistryRef             string
	RegistryEpoch           string
	QueryProfileRef         string
	QueryDigest             string
	InputCommitmentDigest   string
	EvidenceFrontierDigest  string
}

// ValidTimeV1 describes when the oracle's proposition is asserted to hold.
// RecordedAtUnixNanos is separate: a backdated correction changes valid time
// without rewriting when the system received the correction.
type ValidTimeV1 struct {
	FromUnixNanos int64
	ToUnixNanos   *int64
	OpenEnded     bool
}

type EvidenceReferenceV1 struct {
	Reference string
	Kind      string
	Digest    string
}

// OracleReceiptV1 is immutable. Corrections append a new receipt that points
// to the prior receipt; they never overwrite its bytes or digest.
type OracleReceiptV1 struct {
	Sequence                uint64
	ReceiptID               string
	PreviousReceiptDigest   string
	OracleRef               string
	KeyEpoch                string
	AuthorityGrantRef       string
	AuthorityScopeDigest    string
	QueryDigest             string
	InputCommitmentDigest   string
	EvidenceFrontierDigest  string
	AnswerSchemaRef         string
	AnswerDisposition       AnswerDispositionV1
	AnswerPayload           []byte
	AnswerDigest             string
	EvidenceReferences      []EvidenceReferenceV1
	RecordedAtUnixNanos     int64
	ValidTime               ValidTimeV1
	SupersedesReceiptID     string
	SignatureScheme         string
	SignatureBytes          []byte
	SignedMessageDigest     string
	ReceiptDigest            string
}

type ChainResultV1 struct {
	Disposition                    ChainDispositionV1
	Reason                         string
	ScopeDigest                    string
	ExpectedReceiptCount           uint64
	ObservedReceiptCount           uint64
	ExpectedTailDigest             string
	SubmittedReceiptsDigest        string
	GenesisDigest                  string
	ObservedTailDigest             string
	ReceiptChainDigest             string
	OracleSignatureAuthentication  string
	OracleKeyEpochAuthentication   string
	AuthorityGrantAuthentication   string
	SourceUniverseCompleteness     string
	AnswerTruth                    string
	CertificateDigest              string
}

func writeUint32(b *bytes.Buffer, value uint32) {
	var encoded [4]byte
	binary.BigEndian.PutUint32(encoded[:], value)
	_, _ = b.Write(encoded[:])
}

func writeUint64(b *bytes.Buffer, value uint64) {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	_, _ = b.Write(encoded[:])
}

func writeString(b *bytes.Buffer, value string) {
	writeUint32(b, uint32(len(value)))
	_, _ = b.WriteString(value)
}

func digestBytes(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}

func validDigest(value string) bool {
	if len(value) != 64 || strings.ToLower(value) != value {
		return false
	}
	_, err := hex.DecodeString(value)
	return err == nil
}

func requireIdentifier(label, value string) error {
	if strings.TrimSpace(value) == "" {
		return fmt.Errorf("%s is empty", label)
	}
	if len(value) > MaxIdentifierBytes {
		return fmt.Errorf("%s exceeds %d bytes", label, MaxIdentifierBytes)
	}
	return nil
}

func ScopeDigestV1(scope OracleScopeV1) (string, error) {
	for _, field := range []struct{ label, value string }{
		{"source reference", scope.SourceRef},
		{"snapshot reference", scope.SnapshotRef},
		{"registry reference", scope.RegistryRef},
		{"registry epoch", scope.RegistryEpoch},
		{"query profile reference", scope.QueryProfileRef},
	} {
		if err := requireIdentifier(field.label, field.value); err != nil {
			return "", err
		}
	}
	if !validDigest(scope.QueryDigest) ||
		!validDigest(scope.InputCommitmentDigest) ||
		!validDigest(scope.EvidenceFrontierDigest) {
		return "", errors.New("query, input-commitment or evidence-frontier digest is malformed")
	}
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-SCOPE-V1")
	for _, value := range []string{
		scope.SourceRef, scope.SnapshotRef, scope.RegistryRef,
		scope.RegistryEpoch, scope.QueryProfileRef, scope.QueryDigest,
		scope.InputCommitmentDigest, scope.EvidenceFrontierDigest,
	} {
		writeString(&b, value)
	}
	return digestBytes(b.Bytes()), nil
}

func AnswerDigestV1(payload []byte) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-ANSWER-PAYLOAD-V1")
	writeUint64(&b, uint64(len(payload)))
	_, _ = b.Write(payload)
	return digestBytes(b.Bytes())
}

func canonicalEvidenceReferences(refs []EvidenceReferenceV1) ([]EvidenceReferenceV1, error) {
	if len(refs) > MaxEvidenceReferences {
		return nil, fmt.Errorf("evidence reference count exceeds %d", MaxEvidenceReferences)
	}
	canonical := append([]EvidenceReferenceV1(nil), refs...)
	for i, ref := range canonical {
		if err := requireIdentifier("evidence reference", ref.Reference); err != nil {
			return nil, err
		}
		if err := requireIdentifier("evidence kind", ref.Kind); err != nil {
			return nil, err
		}
		if !validDigest(ref.Digest) {
			return nil, errors.New("evidence reference digest is malformed")
		}
		canonical[i] = ref
	}
	sort.Slice(canonical, func(i, j int) bool {
		if canonical[i].Reference != canonical[j].Reference {
			return canonical[i].Reference < canonical[j].Reference
		}
		if canonical[i].Kind != canonical[j].Kind {
			return canonical[i].Kind < canonical[j].Kind
		}
		return canonical[i].Digest < canonical[j].Digest
	})
	for i := 1; i < len(canonical); i++ {
		if canonical[i] == canonical[i-1] {
			return nil, errors.New("duplicate evidence reference")
		}
	}
	return canonical, nil
}

func validateAnswerDisposition(disposition AnswerDispositionV1, payload []byte) error {
	switch disposition {
	case DispositionAffirm, DispositionReject, DispositionAbstain, DispositionUnableToAssess:
		if len(payload) == 0 {
			return errors.New("answer payload is required for this disposition")
		}
	case DispositionNoResponse:
		if len(payload) != 0 {
			return errors.New("NoResponse must not contain an answer payload")
		}
	default:
		return errors.New("unknown answer disposition")
	}
	return nil
}

func validateValidTime(value ValidTimeV1) error {
	if value.OpenEnded {
		if value.ToUnixNanos != nil {
			return errors.New("open-ended valid time must not include an end value")
		}
		return nil
	}
	if value.ToUnixNanos == nil {
		return errors.New("closed valid-time interval requires an end value")
	}
	if *value.ToUnixNanos <= value.FromUnixNanos {
		return errors.New("valid-time interval must be non-empty and half-open")
	}
	return nil
}

func SignableDigestV1(scope OracleScopeV1, receipt OracleReceiptV1) (string, error) {
	scopeDigest, err := ScopeDigestV1(scope)
	if err != nil {
		return "", fmt.Errorf("invalid oracle scope: %w", err)
	}
	if err := validateReceiptFields(scopeDigest, scope, receipt, false); err != nil {
		return "", err
	}
	refs, err := canonicalEvidenceReferences(receipt.EvidenceReferences)
	if err != nil {
		return "", err
	}
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-SIGNABLE-RECEIPT-V1")
	writeString(&b, scopeDigest)
	writeUint64(&b, receipt.Sequence)
	for _, value := range []string{
		receipt.ReceiptID, receipt.PreviousReceiptDigest,
		receipt.OracleRef, receipt.KeyEpoch, receipt.AuthorityGrantRef,
		receipt.AuthorityScopeDigest, receipt.QueryDigest,
		receipt.InputCommitmentDigest, receipt.EvidenceFrontierDigest,
		receipt.AnswerSchemaRef, string(receipt.AnswerDisposition),
		receipt.AnswerDigest,
	} {
		writeString(&b, value)
	}
	writeUint64(&b, uint64(receipt.RecordedAtUnixNanos))
	writeUint64(&b, uint64(receipt.ValidTime.FromUnixNanos))
	if receipt.ValidTime.OpenEnded {
		b.WriteByte(1)
	} else {
		b.WriteByte(0)
		writeUint64(&b, uint64(*receipt.ValidTime.ToUnixNanos))
	}
	writeString(&b, receipt.SupersedesReceiptID)
	writeUint32(&b, uint32(len(refs)))
	for _, ref := range refs {
		writeString(&b, ref.Reference)
		writeString(&b, ref.Kind)
		writeString(&b, ref.Digest)
	}
	return digestBytes(b.Bytes()), nil
}

func validateReceiptFields(scopeDigest string, scope OracleScopeV1, receipt OracleReceiptV1, requireSignature bool) error {
	if receipt.Sequence == 0 {
		return errors.New("receipt sequence must start at one")
	}
	for _, field := range []struct{ label, value string }{
		{"receipt ID", receipt.ReceiptID},
		{"oracle reference", receipt.OracleRef},
		{"key epoch", receipt.KeyEpoch},
		{"authority grant reference", receipt.AuthorityGrantRef},
		{"answer schema reference", receipt.AnswerSchemaRef},
	} {
		if err := requireIdentifier(field.label, field.value); err != nil {
			return err
		}
	}
	if !validDigest(receipt.PreviousReceiptDigest) ||
		!validDigest(receipt.AuthorityScopeDigest) {
		return errors.New("previous receipt or authority-scope digest is malformed")
	}
	if receipt.QueryDigest != scope.QueryDigest ||
		receipt.InputCommitmentDigest != scope.InputCommitmentDigest ||
		receipt.EvidenceFrontierDigest != scope.EvidenceFrontierDigest {
		return errors.New("oracle receipt does not bind the exact query, input commitment and evidence frontier")
	}
	if receipt.RecordedAtUnixNanos <= 0 {
		return errors.New("recorded-at timestamp must be positive")
	}
	if err := validateValidTime(receipt.ValidTime); err != nil {
		return err
	}
	if len(receipt.AnswerPayload) > MaxAnswerBytes {
		return fmt.Errorf("answer payload exceeds %d bytes", MaxAnswerBytes)
	}
	if err := validateAnswerDisposition(receipt.AnswerDisposition, receipt.AnswerPayload); err != nil {
		return err
	}
	if receipt.AnswerDigest != AnswerDigestV1(receipt.AnswerPayload) {
		return errors.New("answer payload digest mismatch")
	}
	if receipt.SupersedesReceiptID != "" {
		if err := requireIdentifier("superseded receipt ID", receipt.SupersedesReceiptID); err != nil {
			return err
		}
		if receipt.SupersedesReceiptID == receipt.ReceiptID {
			return errors.New("receipt cannot supersede itself")
		}
	}
	if requireSignature {
		if err := requireIdentifier("signature scheme", receipt.SignatureScheme); err != nil {
			return err
		}
		if len(receipt.SignatureBytes) == 0 || len(receipt.SignatureBytes) > MaxSignatureBytes {
			return errors.New("signature bytes are empty or exceed configured limit")
		}
		if !validDigest(receipt.SignedMessageDigest) {
			return errors.New("signed-message digest is malformed")
		}
	}
	_ = scopeDigest
	return nil
}

// ReceiptDigestV1 commits to the signable content and signature envelope.
// It does not perform signature, key, role, authority or truth verification.
func ReceiptDigestV1(scope OracleScopeV1, receipt OracleReceiptV1) (string, error) {
	signableDigest, err := SignableDigestV1(scope, receipt)
	if err != nil {
		return "", err
	}
	if err := requireIdentifier("signature scheme", receipt.SignatureScheme); err != nil {
		return "", err
	}
	if len(receipt.SignatureBytes) == 0 || len(receipt.SignatureBytes) > MaxSignatureBytes {
		return "", errors.New("signature bytes are empty or exceed configured limit")
	}
	if !validDigest(receipt.SignedMessageDigest) {
		return "", errors.New("signed-message digest is malformed")
	}
	if receipt.SignedMessageDigest != signableDigest {
		return "", errors.New("signature envelope is bound to different receipt content")
	}
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-RECEIPT-V1")
	writeString(&b, signableDigest)
	writeString(&b, receipt.SignatureScheme)
	writeUint32(&b, uint32(len(receipt.SignatureBytes)))
	_, _ = b.Write(receipt.SignatureBytes)
	writeString(&b, receipt.SignedMessageDigest)
	return digestBytes(b.Bytes()), nil
}

func rawOracleReceiptSubmissionDigest(receipts []OracleReceiptV1) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-RECEIPT-SUBMISSION-V1")
	writeUint64(&b, uint64(len(receipts)))
	for _, receipt := range receipts {
		writeUint64(&b, receipt.Sequence)
		for _, value := range []string{
			receipt.ReceiptID, receipt.PreviousReceiptDigest,
			receipt.OracleRef, receipt.KeyEpoch, receipt.AuthorityGrantRef,
			receipt.AuthorityScopeDigest, receipt.QueryDigest,
			receipt.InputCommitmentDigest, receipt.EvidenceFrontierDigest,
			receipt.AnswerSchemaRef, string(receipt.AnswerDisposition),
			receipt.AnswerDigest, receipt.SupersedesReceiptID,
			receipt.SignatureScheme, receipt.SignedMessageDigest,
			receipt.ReceiptDigest,
		} {
			writeString(&b, value)
		}
		writeUint64(&b, uint64(len(receipt.AnswerPayload)))
		_, _ = b.Write(receipt.AnswerPayload)
		writeUint64(&b, uint64(len(receipt.SignatureBytes)))
		_, _ = b.Write(receipt.SignatureBytes)
		writeUint64(&b, uint64(receipt.RecordedAtUnixNanos))
		writeUint64(&b, uint64(receipt.ValidTime.FromUnixNanos))
		if receipt.ValidTime.OpenEnded {
			b.WriteByte(1)
		} else {
			b.WriteByte(0)
		}
		if receipt.ValidTime.ToUnixNanos == nil {
			b.WriteByte(0)
		} else {
			b.WriteByte(1)
			writeUint64(&b, uint64(*receipt.ValidTime.ToUnixNanos))
		}
		writeUint32(&b, uint32(len(receipt.EvidenceReferences)))
		for _, ref := range receipt.EvidenceReferences {
			writeString(&b, ref.Reference)
			writeString(&b, ref.Kind)
			writeString(&b, ref.Digest)
		}
	}
	return digestBytes(b.Bytes())
}

func oversizedOracleReceiptSubmissionDigest(receiptCount uint64, index uint64, field string, value string) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-RECEIPT-OVERSIZE-SUBMISSION-V1")
	writeUint64(&b, receiptCount)
	writeUint64(&b, index)
	writeString(&b, field)
	writeUint64(&b, uint64(len(value)))
	if len(value) <= 128 {
		writeString(&b, value)
	} else {
		writeString(&b, value[:64])
		writeString(&b, value[len(value)-64:])
	}
	return digestBytes(b.Bytes())
}

func oversizedOracleReceiptBytesDigest(receiptCount, index uint64, field string, value []byte) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-RECEIPT-OVERSIZE-BYTES-V1")
	writeUint64(&b, receiptCount)
	writeUint64(&b, index)
	writeString(&b, field)
	writeUint64(&b, uint64(len(value)))
	if len(value) <= 128 {
		writeString(&b, string(value))
	} else {
		writeString(&b, string(value[:64]))
		writeString(&b, string(value[len(value)-64:]))
	}
	return digestBytes(b.Bytes())
}

func preflightOracleReceiptSubmissions(receipts []OracleReceiptV1) (string, string) {
	if uint64(len(receipts)) > MaxReceiptCount {
		return "", digestBytes([]byte(fmt.Sprintf(
			"MYCELIX-ORACLE-RECEIPT-COUNT-OVERLIMIT|%d", len(receipts),
		)))
	}
	var totalBytes uint64
	addString := func(index uint64, label, value string) (string, bool) {
		if len(value) > MaxIdentifierBytes || uint64(len(value)) > MaxReceiptEncodingBytes-totalBytes {
			return oversizedOracleReceiptSubmissionDigest(uint64(len(receipts)), index, label, value), false
		}
		totalBytes += uint64(len(value))
		return "", true
	}
	for index, receipt := range receipts {
		for _, field := range []struct{ label, value string }{
			{"receipt_id", receipt.ReceiptID},
			{"previous_digest", receipt.PreviousReceiptDigest},
			{"oracle_ref", receipt.OracleRef},
			{"key_epoch", receipt.KeyEpoch},
			{"authority_grant_ref", receipt.AuthorityGrantRef},
			{"authority_scope_digest", receipt.AuthorityScopeDigest},
			{"query_digest", receipt.QueryDigest},
			{"input_commitment_digest", receipt.InputCommitmentDigest},
			{"evidence_frontier_digest", receipt.EvidenceFrontierDigest},
			{"answer_schema_ref", receipt.AnswerSchemaRef},
			{"answer_disposition", string(receipt.AnswerDisposition)},
			{"answer_digest", receipt.AnswerDigest},
			{"supersedes_receipt_id", receipt.SupersedesReceiptID},
			{"signature_scheme", receipt.SignatureScheme},
			{"signed_message_digest", receipt.SignedMessageDigest},
			{"receipt_digest", receipt.ReceiptDigest},
		} {
			if digest, ok := addString(uint64(index), field.label, field.value); !ok {
				return fmt.Sprintf("receipt %d %s exceeds the configured encoding budget", index, field.label), digest
			}
		}
		for _, field := range []struct{ label string; value []byte; limit uint64 }{
			{"answer_payload", receipt.AnswerPayload, MaxAnswerBytes},
			{"signature_bytes", receipt.SignatureBytes, MaxSignatureBytes},
		} {
			if uint64(len(field.value)) > field.limit || uint64(len(field.value)) > MaxReceiptEncodingBytes-totalBytes {
				return fmt.Sprintf("receipt %d %s exceeds the configured byte budget", index, field.label),
					oversizedOracleReceiptBytesDigest(uint64(len(receipts)), uint64(index), field.label, field.value)
			}
			totalBytes += uint64(len(field.value))
		}
		if len(receipt.EvidenceReferences) > MaxEvidenceReferences {
			return fmt.Sprintf("receipt %d evidence reference count exceeds configured limit", index),
				digestBytes([]byte(fmt.Sprintf("MYCELIX-ORACLE-RECEIPT-OVERSIZE-REFS|%d|%d", len(receipts), index)))
		}
		for refIndex, ref := range receipt.EvidenceReferences {
			for _, field := range []struct{ label, value string }{
				{"evidence_reference", ref.Reference},
				{"evidence_kind", ref.Kind},
				{"evidence_digest", ref.Digest},
			} {
				label := fmt.Sprintf("%s[%d]", field.label, refIndex)
				if digest, ok := addString(uint64(index), label, field.value); !ok {
					return fmt.Sprintf("receipt %d %s exceeds the configured encoding budget", index, label), digest
				}
			}
		}
	}
	return "", rawOracleReceiptSubmissionDigest(receipts)
}

func genesisDigest(scopeDigest string) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-RECEIPT-GENESIS-V1")
	writeString(&b, scopeDigest)
	return digestBytes(b.Bytes())
}

func finalizeChainResult(result ChainResultV1) ChainResultV1 {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-CHAIN-RESULT-V1")
	writeString(&b, string(result.Disposition))
	writeString(&b, result.Reason)
	writeString(&b, result.ScopeDigest)
	writeUint64(&b, result.ExpectedReceiptCount)
	writeUint64(&b, result.ObservedReceiptCount)
	writeString(&b, result.ExpectedTailDigest)
	writeString(&b, result.SubmittedReceiptsDigest)
	writeString(&b, result.GenesisDigest)
	writeString(&b, result.ObservedTailDigest)
	writeString(&b, result.ReceiptChainDigest)
	writeString(&b, result.OracleSignatureAuthentication)
	writeString(&b, result.OracleKeyEpochAuthentication)
	writeString(&b, result.AuthorityGrantAuthentication)
	writeString(&b, result.SourceUniverseCompleteness)
	writeString(&b, result.AnswerTruth)
	result.CertificateDigest = digestBytes(b.Bytes())
	return result
}

// VerifyOracleReceiptChainV1 verifies canonical structure, query/input/frontier
// binding, signature-envelope binding, append-only sequence links and
// supersession references. It deliberately does not verify the signature,
// issuer-key status, authority grant, source completeness or answer truth.
func VerifyOracleReceiptChainV1(scope OracleScopeV1, receipts []OracleReceiptV1, expectedCount uint64, expectedTail string) ChainResultV1 {
	result := ChainResultV1{
		Disposition: ChainIndeterminate,
		Reason: "oracle receipt chain is indeterminate",
		ExpectedReceiptCount: expectedCount,
		ObservedReceiptCount: uint64(len(receipts)),
		OracleSignatureAuthentication: NotEvaluated,
		OracleKeyEpochAuthentication: NotEvaluated,
		AuthorityGrantAuthentication: NotEvaluated,
		SourceUniverseCompleteness: NotEvaluated,
		AnswerTruth: NotEvaluated,
	}
	result.ExpectedTailDigest = expectedTail
	if reason, submissionDigest := preflightOracleReceiptSubmissions(receipts); reason != "" {
		result.Reason = reason
		result.SubmittedReceiptsDigest = submissionDigest
		return finalizeChainResult(result)
	}
	result.SubmittedReceiptsDigest = rawOracleReceiptSubmissionDigest(receipts)
	scopeDigest, err := ScopeDigestV1(scope)
	if err != nil {
		result.Reason = "invalid oracle scope: " + err.Error()
		return finalizeChainResult(result)
	}
	result.ScopeDigest = scopeDigest
	result.GenesisDigest = genesisDigest(scopeDigest)
	result.ObservedTailDigest = result.GenesisDigest
	if expectedCount > MaxReceiptCount || uint64(len(receipts)) > MaxReceiptCount {
		result.Reason = "oracle receipt count exceeds configured limit"
		return finalizeChainResult(result)
	}
	if !validDigest(expectedTail) {
		result.Reason = "expected receipt tail digest is malformed"
		return finalizeChainResult(result)
	}
	if uint64(len(receipts)) != expectedCount {
		result.Reason = "observed receipt count differs from expected count"
		return finalizeChainResult(result)
	}
	seenIDs := make(map[string]uint64, len(receipts))
	digests := make([]string, 0, len(receipts))
	running := result.GenesisDigest
	for i, receipt := range receipts {
		expectedSequence := uint64(i) + 1
		if receipt.Sequence != expectedSequence {
			result.Reason = fmt.Sprintf("receipt sequence discontinuity at index %d", i)
			return finalizeChainResult(result)
		}
		if _, exists := seenIDs[receipt.ReceiptID]; exists {
			result.Reason = "duplicate oracle receipt ID"
			return finalizeChainResult(result)
		}
		if receipt.PreviousReceiptDigest != running {
			result.Reason = fmt.Sprintf("previous receipt digest mismatch at sequence %d", expectedSequence)
			return finalizeChainResult(result)
		}
		digest, err := ReceiptDigestV1(scope, receipt)
		if err != nil {
			result.Reason = fmt.Sprintf("receipt %d invalid: %v", expectedSequence, err)
			return finalizeChainResult(result)
		}
		if receipt.ReceiptDigest != digest {
			result.Reason = fmt.Sprintf("receipt digest mismatch at sequence %d", expectedSequence)
			return finalizeChainResult(result)
		}
		if receipt.SupersedesReceiptID != "" {
			priorSequence, exists := seenIDs[receipt.SupersedesReceiptID]
			if !exists || priorSequence >= expectedSequence {
				result.Reason = "supersession must reference an earlier receipt in this chain"
				return finalizeChainResult(result)
			}
		}
		seenIDs[receipt.ReceiptID] = expectedSequence
		digests = append(digests, digest)
		running = digest
	}
	result.ObservedTailDigest = running
	if running != expectedTail {
		result.Reason = "observed receipt tail differs from expected anchor"
		return finalizeChainResult(result)
	}
	var chain bytes.Buffer
	writeString(&chain, "MYCELIX-ORACLE-RECEIPT-CHAIN-V1")
	writeString(&chain, scopeDigest)
	for _, digest := range digests {
		writeString(&chain, digest)
	}
	result.ReceiptChainDigest = digestBytes(chain.Bytes())
	result.Disposition = ChainStructurallyValid
	result.Reason = "oracle receipt chain is structurally valid; signature, authority, source completeness and answer truth remain NotEvaluated"
	return finalizeChainResult(result)
}
