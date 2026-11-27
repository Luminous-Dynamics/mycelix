package temporalrange

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"strings"
	"unicode/utf8"
)

const projectionAuditDomainV1 = "MYCELIX-OWNER-TEMPORAL-PROJECTION-AUDIT-V0.1\x00"
const projectionInventoryDomainV1 = "MYCELIX-OWNER-ACCEPTED-RECEIPT-INVENTORY-V0.1\x00"
const canonicalKnowledgeReceiptMagicV1 = "MYCELIX-KNOWLEDGE-RECEIPT-V1\x00"

const (
	maxProjectionReceiptsV1 = 100_000
	maxProjectionReceiptBytesV1 = 64 * 1024 * 1024
	maxProjectionDependenciesV1 = 10_000
)

type ReceiptProjectionClassV1 uint8

const (
	ReceiptProjectsIntervalAssertionV1 ReceiptProjectionClassV1 = 1
	ReceiptIsNonTemporalV1              ReceiptProjectionClassV1 = 2
)

// AcceptedReceiptProjectionV1 is the claimed semantic projection of one canonical knowledge
// receipt leaf. Receipt bytes and the full owner-log RFC 9162 root are recomputed; derivation
// of interval fields from the underlying assertion payload remains an external obligation.
type AcceptedReceiptProjectionV1 struct {
	ReceiptLogLeafIndex       uint64
	OwnerSequence             uint64
	ReceiptDigest             string
	CanonicalReceiptEntryBytes []byte
	AssertionID               string
	Class                     ReceiptProjectionClassV1
	AssertionRef              string
	ValidFrom                 IntervalInstantV1
	ValidTo                   IntervalEndV1
}

// AcceptedReceiptInventoryV1 carries every leaf in a declared owner-log prefix, including
// non-receipt entries. This package recomputes the RFC 9162 root against the declared root
// digest, but does not authenticate the checkpoint that makes that root authoritative.
type OwnerLogLeafV1 struct {
	LeafIndex    uint64
	OwnerSequence uint64
	EntryBytes   []byte
}

type AcceptedReceiptInventoryV1 struct {
	OwnerRef           string
	OwnerHistoryRef    string
	OwnerSequenceCut   uint64
	TreeSize           uint64
	OwnerLogRootDigest string
	LogLeaves          []OwnerLogLeafV1
	Receipts           []AcceptedReceiptProjectionV1
}

type ProjectionReplayDispositionV1 string

type ReceiptInventoryAuthenticationV1 string

const (
	ProjectionMatchesSuppliedAcceptedInventoryV1 ProjectionReplayDispositionV1 = "ProjectionMatchesSuppliedAcceptedInventory"
	ReceiptInventoryAuthenticationNotEvaluatedV1 ReceiptInventoryAuthenticationV1 = "NotEvaluated"
	ReceiptLogRootRecomputedButTrustExternalV1 ReceiptInventoryAuthenticationV1 = "LogRootRecomputedButTrustExternal"
)

type ProjectionMappingAuthenticationV1 string

const ProjectionMappingAuthenticationNotEvaluatedV1 ProjectionMappingAuthenticationV1 = "NotEvaluated"

type OwnerProjectionReplayAuditV1 struct {
	Disposition                  ProjectionReplayDispositionV1
	ReceiptInventoryAuthentication ReceiptInventoryAuthenticationV1
	ProjectionMappingAuthentication ProjectionMappingAuthenticationV1
	OwnerRef                     string
	OwnerHistoryRef              string
	OwnerSequenceCut             uint64
	TreeSize                     uint64
	ReceiptCount                 uint64
	NonReceiptLogLeafCount       uint64
	IntervalAssertionCount       uint64
	NonTemporalReceiptCount      uint64
	InventoryDigest              [32]byte
	DerivedIntervalIndexRoot     [32]byte
	AuditDigest                  [32]byte
	RecomputedOwnerLogRoot       [32]byte
	OwnerLogRootMatches          bool
	HasOwnerGaps                 bool
}

var (
	ErrProjectionInventoryMismatch = errors.New("accepted receipt inventory does not match owner binding")
	ErrProjectionLeafCoverage       = errors.New("receipt log leaf positions are incomplete or non-canonical")
	ErrProjectionSequenceOrder      = errors.New("owner receipt sequences are not strictly increasing within the cut")
	ErrProjectionReceiptClass       = errors.New("unsupported or malformed receipt projection class")
	ErrProjectionIndexRootMismatch  = errors.New("replayed temporal index root does not match owner binding")
	ErrProjectionAuditMismatch      = errors.New("projection audit digest does not match owner binding")
	ErrProjectionReceiptOmitted     = errors.New("canonical knowledge receipt leaf has no temporal projection disposition")
)

// ComputeOwnerProjectionReplayAuditV1 independently rebuilds the interval index from the
// supplied canonical owner-log inventory and computes its deterministic projection audit digest.
//
// This function recomputes each canonical receipt digest and the RFC 9162 Merkle Tree Hash
// over all declared log leaves. It therefore proves that the supplied ordered receipt bytes
// match the claimed owner-log root. It does NOT authenticate that root's checkpoint/signature,
// nor does it independently derive the supplied temporal projection from the assertion payload.
// The checkpoint/frontier layer must authenticate the root; a separately qualified mapper must
// establish the interval projection semantics before source-complete reliance is permitted.
func ComputeOwnerProjectionReplayAuditV1(
	scope IntervalScopeV1,
	binding OwnerIntervalIndexBindingV1,
	inventory AcceptedReceiptInventoryV1,
) (OwnerProjectionReplayAuditV1, error) {
	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: intervalProfileV1,
		QueryScope: scope, Owners: []OwnerIntervalIndexBindingV1{binding},
	}
	if validateTemporalFrontierManifestV1(manifest) != nil {
		return OwnerProjectionReplayAuditV1{}, ErrForestMalformed
	}
	if inventory.OwnerRef != binding.OwnerRef ||
		inventory.OwnerHistoryRef != binding.OwnerHistoryRef ||
		inventory.OwnerSequenceCut != binding.OwnerSequence ||
		inventory.TreeSize != binding.TreeSize ||
		inventory.OwnerLogRootDigest != binding.OwnerLogRootDigest {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionInventoryMismatch
	}
	if inventory.TreeSize > maxProjectionReceiptsV1 ||
		uint64(len(inventory.LogLeaves)) != inventory.TreeSize ||
		uint64(len(inventory.Receipts)) > inventory.TreeSize {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
	}

	projectionsByLeaf := make(map[uint64]AcceptedReceiptProjectionV1, len(inventory.Receipts))
	for _, projection := range inventory.Receipts {
		if projection.ReceiptLogLeafIndex >= inventory.TreeSize ||
			len(projection.CanonicalReceiptEntryBytes) == 0 ||
			validateIntervalDigestV1(projection.ReceiptDigest) != nil {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
		}
		if _, exists := projectionsByLeaf[projection.ReceiptLogLeafIndex]; exists {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
		}
		projectionsByLeaf[projection.ReceiptLogLeafIndex] = projection
	}

	records := make([]IntervalRecordV1, 0, len(inventory.Receipts))
	logEntries := make([][]byte, 0, len(inventory.LogLeaves))
	var totalLogBytes uint64
	var nonTemporal uint64
	var nonReceiptLeaves uint64
	var previousSequence uint64
	for i, leaf := range inventory.LogLeaves {
		if leaf.LeafIndex != uint64(i) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
		}
		if leaf.OwnerSequence == 0 || leaf.OwnerSequence > binding.OwnerSequence ||
			(i > 0 && leaf.OwnerSequence <= previousSequence) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionSequenceOrder
		}
		expectedLeafIndex, ok := ownerLeafIndexForSequenceV1(binding, leaf.OwnerSequence)
		if !ok || expectedLeafIndex != uint64(i) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionSequenceOrder
		}
		totalLogBytes += uint64(len(leaf.EntryBytes))
		if totalLogBytes > maxProjectionReceiptBytesV1 {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
		}
		logEntries = append(logEntries, leaf.EntryBytes)
		previousSequence = leaf.OwnerSequence

		projection, hasProjection := projectionsByLeaf[leaf.LeafIndex]
		isKnowledgeReceipt := bytes.HasPrefix(leaf.EntryBytes, []byte(canonicalKnowledgeReceiptMagicV1))
		looksLikeKnowledgeReceipt := bytes.HasPrefix(leaf.EntryBytes, []byte(canonicalKnowledgeReceiptStemV1))
		if looksLikeKnowledgeReceipt && !isKnowledgeReceipt {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
		}
		if !isKnowledgeReceipt {
			if hasProjection {
				return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
			}
			nonReceiptLeaves++
			continue
		}
		if !hasProjection {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptOmitted
		}
		if projection.ReceiptLogLeafIndex != leaf.LeafIndex ||
			projection.OwnerSequence != leaf.OwnerSequence ||
			!bytes.Equal(projection.CanonicalReceiptEntryBytes, leaf.EntryBytes) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionInventoryMismatch
		}
		canonical, err := decodeCanonicalKnowledgeReceiptEntryV1(leaf.EntryBytes)
		if err != nil {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
		}
		if canonical.VerifierRef != scope.VerifierRef ||
			canonical.PolicyEpochRef != scope.PolicyEpochRef ||
			canonical.OwnerRef != binding.OwnerRef ||
			canonical.OwnerSequence != leaf.OwnerSequence ||
			canonical.AssertionID != projection.AssertionID {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionInventoryMismatch
		}
		if canonical.ReceiptDigest != projection.ReceiptDigest {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
		}
		if projection.Class == ReceiptProjectsIntervalAssertionV1 {
			record := IntervalRecordV1{
				AssertionRef: projection.AssertionRef,
				ValidFrom: projection.ValidFrom,
				ValidTo: projection.ValidTo,
				ReceiptDigest: projection.ReceiptDigest,
			}
			if projection.AssertionRef == "" || projection.AssertionRef != canonical.AssertionID ||
				validateIntervalRecordV1(record) != nil {
				return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
			}
			records = append(records, record)
		} else if projection.Class == ReceiptIsNonTemporalV1 {
			if projection.AssertionRef != "" ||
				projection.ValidFrom != (IntervalInstantV1{}) ||
				projection.ValidTo != (IntervalEndV1{}) {
				return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
			}
			nonTemporal++
		} else {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
		}
		delete(projectionsByLeaf, leaf.LeafIndex)
	}
	if len(projectionsByLeaf) != 0 {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionInventoryMismatch
	}

	recomputedLogRoot := rfc9162TreeHashV1(logEntries)
	if fmt.Sprintf("sha256:%x", recomputedLogRoot) != binding.OwnerLogRootDigest {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionInventoryMismatch
	}

	tree, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		return OwnerProjectionReplayAuditV1{}, err
	}
	if tree.RootHash() != binding.IntervalIndexRoot {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionIndexRootMismatch
	}

	inventoryDigest := hashAcceptedReceiptInventoryV1(scope, binding, inventory)
	auditDigest := hashOwnerProjectionAuditV1(
		scope, binding, inventoryDigest, tree.RootHash(),
		uint64(len(inventory.Receipts)), nonReceiptLeaves, uint64(len(records)), nonTemporal,
	)
	hasGaps := len(binding.Gaps) > 0
	return OwnerProjectionReplayAuditV1{
		Disposition: ProjectionMatchesSuppliedAcceptedInventoryV1,
		ReceiptInventoryAuthentication: ReceiptLogRootRecomputedButTrustExternalV1,
		ProjectionMappingAuthentication: ProjectionMappingAuthenticationNotEvaluatedV1,
		OwnerRef: binding.OwnerRef,
		OwnerHistoryRef: binding.OwnerHistoryRef,
		OwnerSequenceCut: binding.OwnerSequence,
		TreeSize: binding.TreeSize,
		ReceiptCount: uint64(len(inventory.Receipts)),
		NonReceiptLogLeafCount: nonReceiptLeaves,
		IntervalAssertionCount: uint64(len(records)),
		NonTemporalReceiptCount: nonTemporal,
		InventoryDigest: inventoryDigest,
		DerivedIntervalIndexRoot: tree.RootHash(),
		AuditDigest: auditDigest,
		RecomputedOwnerLogRoot: recomputedLogRoot,
		OwnerLogRootMatches: true,
		HasOwnerGaps: hasGaps,
	}, nil
}

// VerifyOwnerProjectionReplayAuditV1 also checks that the computed audit digest is the exact
// digest committed by the owner binding / verifier checkpoint.
func VerifyOwnerProjectionReplayAuditV1(
	scope IntervalScopeV1,
	binding OwnerIntervalIndexBindingV1,
	inventory AcceptedReceiptInventoryV1,
) (OwnerProjectionReplayAuditV1, error) {
	audit, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		return OwnerProjectionReplayAuditV1{}, err
	}
	if binding.ProjectionAuditDigest == "" ||
		binding.ProjectionAuditDigest != fmt.Sprintf("sha256:%x", audit.AuditDigest) {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionAuditMismatch
	}
	return audit, nil
}

func ownerSequenceIsGapV1(gaps []OwnerGapRangeV1, sequence uint64) bool {
	for _, gap := range gaps {
		if sequence < gap.First {
			return false
		}
		if sequence <= gap.Last {
			return true
		}
	}
	return false
}

func hashAcceptedReceiptInventoryV1(
	scope IntervalScopeV1,
	binding OwnerIntervalIndexBindingV1,
	inventory AcceptedReceiptInventoryV1,
) [32]byte {
	var preimage bytes.Buffer
	preimage.WriteByte(0x50)
	preimage.WriteString(projectionInventoryDomainV1)
	appendIntervalScopeBytesV1(&preimage, scope)
	appendForestStringV1(&preimage, binding.OwnerRef)
	appendForestStringV1(&preimage, binding.OwnerHistoryRef)
	_ = binary.Write(&preimage, binary.BigEndian, binding.OwnerSequence)
	_ = binary.Write(&preimage, binary.BigEndian, binding.TreeSize)
	appendForestStringV1(&preimage, binding.OwnerLogRootDigest)

	// Commit the complete owner-log leaf sequence, including non-receipt entries.
	_ = binary.Write(&preimage, binary.BigEndian, uint64(len(inventory.LogLeaves)))
	for _, leaf := range inventory.LogLeaves {
		_ = binary.Write(&preimage, binary.BigEndian, leaf.LeafIndex)
		_ = binary.Write(&preimage, binary.BigEndian, leaf.OwnerSequence)
		_ = binary.Write(&preimage, binary.BigEndian, uint32(len(leaf.EntryBytes)))
		preimage.Write(leaf.EntryBytes)
	}

	// Then commit the explicit semantic projection assigned to receipt leaves.
	_ = binary.Write(&preimage, binary.BigEndian, uint64(len(inventory.Receipts)))
	for _, receipt := range inventory.Receipts {
		_ = binary.Write(&preimage, binary.BigEndian, receipt.ReceiptLogLeafIndex)
		_ = binary.Write(&preimage, binary.BigEndian, receipt.OwnerSequence)
		appendForestStringV1(&preimage, receipt.ReceiptDigest)
		_ = binary.Write(&preimage, binary.BigEndian, uint32(len(receipt.CanonicalReceiptEntryBytes)))
		preimage.Write(receipt.CanonicalReceiptEntryBytes)
		appendForestStringV1(&preimage, receipt.AssertionID)
		preimage.WriteByte(byte(receipt.Class))
		if receipt.Class == ReceiptProjectsIntervalAssertionV1 {
			record := IntervalRecordV1{
				AssertionRef: receipt.AssertionRef,
				ValidFrom: receipt.ValidFrom,
				ValidTo: receipt.ValidTo,
				ReceiptDigest: receipt.ReceiptDigest,
			}
			appendIntervalRecordBytesV1(&preimage, record)
		}
	}
	return sha256.Sum256(preimage.Bytes())
}

func hashOwnerProjectionAuditV1(
	scope IntervalScopeV1,
	binding OwnerIntervalIndexBindingV1,
	inventoryDigest [32]byte,
	indexRoot [32]byte,
	receiptCount uint64,
	nonReceiptLeafCount uint64,
	intervalCount uint64,
	nonTemporalCount uint64,
) [32]byte {
	var preimage bytes.Buffer
	preimage.WriteByte(0x51)
	preimage.WriteString(projectionAuditDomainV1)
	appendIntervalScopeBytesV1(&preimage, scope)
	appendForestStringV1(&preimage, binding.OwnerRef)
	appendForestStringV1(&preimage, binding.OwnerHistoryRef)
	_ = binary.Write(&preimage, binary.BigEndian, binding.OwnerSequence)
	_ = binary.Write(&preimage, binary.BigEndian, binding.TreeSize)
	appendForestStringV1(&preimage, binding.OwnerLogRootDigest)
	preimage.Write(inventoryDigest[:])
	preimage.Write(indexRoot[:])
	_ = binary.Write(&preimage, binary.BigEndian, receiptCount)
	_ = binary.Write(&preimage, binary.BigEndian, nonReceiptLeafCount)
	_ = binary.Write(&preimage, binary.BigEndian, intervalCount)
	_ = binary.Write(&preimage, binary.BigEndian, nonTemporalCount)
	_ = binary.Write(&preimage, binary.BigEndian, uint32(len(binding.Gaps)))
	for _, gap := range binding.Gaps {
		_ = binary.Write(&preimage, binary.BigEndian, gap.First)
		_ = binary.Write(&preimage, binary.BigEndian, gap.Last)
	}
	return sha256.Sum256(preimage.Bytes())
}

func appendIntervalScopeBytesV1(dst *bytes.Buffer, scope IntervalScopeV1) {
	appendForestStringV1(dst, scope.VerifierRef)
	appendForestStringV1(dst, scope.PolicyEpochRef)
	appendForestStringV1(dst, scope.NamespaceRef)
	appendForestStringV1(dst, scope.SubjectRef)
	appendForestStringV1(dst, scope.ClaimID)
	appendForestStringV1(dst, scope.SourceFrontierDigest)
}

func appendIntervalRecordBytesV1(dst *bytes.Buffer, record IntervalRecordV1) {
	appendForestStringV1(dst, record.AssertionRef)
	_ = binary.Write(dst, binary.BigEndian, record.ValidFrom.Seconds)
	_ = binary.Write(dst, binary.BigEndian, record.ValidFrom.Nanos)
	if record.ValidTo.Open {
		dst.WriteByte(1)
		return
	}
	dst.WriteByte(0)
	_ = binary.Write(dst, binary.BigEndian, record.ValidTo.At.Seconds)
	_ = binary.Write(dst, binary.BigEndian, record.ValidTo.At.Nanos)
}


type canonicalKnowledgeReceiptFieldsV1 struct {
	ReceiptID      string
	VerifierRef    string
	PolicyEpochRef string
	AssertionID    string
	AssertionDigest string
	OwnerRef       string
	OwnerSequence  uint64
	ReceiptDigest  string
}

func decodeCanonicalKnowledgeReceiptEntryV1(entry []byte) (canonicalKnowledgeReceiptFieldsV1, error) {
	var fields canonicalKnowledgeReceiptFieldsV1
	if len(entry) == 0 || len(entry) > 64*1024 {
		return fields, ErrProjectionReceiptClass
	}
	reader := bytes.NewReader(entry)
	magic := make([]byte, len(canonicalKnowledgeReceiptMagicV1))
	if _, err := io.ReadFull(reader, magic); err != nil || string(magic) != canonicalKnowledgeReceiptMagicV1 {
		return fields, ErrProjectionReceiptClass
	}
	stringsOut := []*string{
		&fields.ReceiptID, &fields.VerifierRef, &fields.PolicyEpochRef,
		&fields.AssertionID, &fields.AssertionDigest, &fields.OwnerRef,
	}
	for _, field := range stringsOut {
		value, err := readCanonicalReceiptStringV1(reader)
		if err != nil {
			return fields, err
		}
		*field = value
	}
	if validateIntervalDigestV1(fields.AssertionDigest) != nil {
		return fields, ErrProjectionReceiptClass
	}
	if binary.Read(reader, binary.BigEndian, &fields.OwnerSequence) != nil || fields.OwnerSequence == 0 {
		return fields, ErrProjectionReceiptClass
	}
	var dependencyCount uint32
	if binary.Read(reader, binary.BigEndian, &dependencyCount) != nil || dependencyCount > maxProjectionDependenciesV1 {
		return fields, ErrProjectionReceiptClass
	}
	previousOwner := ""
	var previousSequence uint64
	for i := uint32(0); i < dependencyCount; i++ {
		owner, err := readCanonicalReceiptStringV1(reader)
		if err != nil {
			return fields, err
		}
		var sequence uint64
		if binary.Read(reader, binary.BigEndian, &sequence) != nil || sequence == 0 {
			return fields, ErrProjectionReceiptClass
		}
		if owner == fields.OwnerRef && sequence >= fields.OwnerSequence {
			return fields, ErrProjectionReceiptClass
		}
		if i > 0 && (owner < previousOwner || (owner == previousOwner && sequence <= previousSequence)) {
			return fields, ErrProjectionReceiptClass
		}
		previousOwner, previousSequence = owner, sequence
	}
	if reader.Len() != 0 {
		return fields, ErrProjectionReceiptClass
	}
	sum := sha256.Sum256(entry)
	fields.ReceiptDigest = fmt.Sprintf("sha256:%x", sum)
	return fields, nil
}

func readCanonicalReceiptStringV1(reader *bytes.Reader) (string, error) {
	var length uint32
	if binary.Read(reader, binary.BigEndian, &length) != nil ||
		length == 0 || length > 1_024 || uint64(length) > uint64(reader.Len()) {
		return "", ErrProjectionReceiptClass
	}
	data := make([]byte, int(length))
	if _, err := io.ReadFull(reader, data); err != nil || !utf8.Valid(data) {
		return "", ErrProjectionReceiptClass
	}
	value := string(data)
	if strings.TrimSpace(value) == "" {
		return "", ErrProjectionReceiptClass
	}
	return value, nil
}

func ownerLeafIndexForSequenceV1(binding OwnerIntervalIndexBindingV1, sequence uint64) (uint64, bool) {
	if sequence == 0 || sequence > binding.OwnerSequence || ownerSequenceIsGapV1(binding.Gaps, sequence) {
		return 0, false
	}
	var missingBefore uint64
	for _, gap := range binding.Gaps {
		if gap.Last >= sequence {
			break
		}
		width := gap.Last - gap.First + 1
		if ^uint64(0)-missingBefore < width {
			return 0, false
		}
		missingBefore += width
	}
	if sequence-1 < missingBefore {
		return 0, false
	}
	return sequence - 1 - missingBefore, true
}

func rfc9162LeafHashV1(entry []byte) [32]byte {
	preimage := make([]byte, 1+len(entry))
	preimage[0] = 0x00
	copy(preimage[1:], entry)
	return sha256.Sum256(preimage)
}

func rfc9162NodeHashV1(left, right [32]byte) [32]byte {
	var preimage [65]byte
	preimage[0] = 0x01
	copy(preimage[1:33], left[:])
	copy(preimage[33:], right[:])
	return sha256.Sum256(preimage[:])
}

func rfc9162LargestPowerOfTwoLessThanV1(value int) int {
	power := 1
	for power<<1 < value {
		power <<= 1
	}
	return power
}

func rfc9162TreeHashV1(entries [][]byte) [32]byte {
	if len(entries) == 0 {
		return sha256.Sum256(nil)
	}
	leaves := make([][32]byte, len(entries))
	for i, entry := range entries {
		leaves[i] = rfc9162LeafHashV1(entry)
	}
	var treeHash func([][32]byte) [32]byte
	treeHash = func(values [][32]byte) [32]byte {
		switch len(values) {
		case 0:
			return sha256.Sum256(nil)
		case 1:
			return values[0]
		default:
			split := rfc9162LargestPowerOfTwoLessThanV1(len(values))
			left := treeHash(values[:split])
			right := treeHash(values[split:])
			return rfc9162NodeHashV1(left, right)
		}
	}
	return treeHash(leaves)
}
