package temporalrange

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"fmt"
)

const projectionAuditDomainV1 = "MYCELIX-OWNER-TEMPORAL-PROJECTION-AUDIT-V0.1\x00"
const projectionInventoryDomainV1 = "MYCELIX-OWNER-ACCEPTED-RECEIPT-INVENTORY-V0.1\x00"

type ReceiptProjectionClassV1 uint8

const (
	ReceiptProjectsIntervalAssertionV1 ReceiptProjectionClassV1 = 1
	ReceiptIsNonTemporalV1              ReceiptProjectionClassV1 = 2
)

// AcceptedReceiptProjectionV1 is the decoded semantic projection of one accepted receipt.
// Receipt authenticity/admission is an upstream precondition and is not established here.
type AcceptedReceiptProjectionV1 struct {
	ReceiptLogLeafIndex uint64
	OwnerSequence       uint64
	ReceiptDigest       string
	Class               ReceiptProjectionClassV1
	AssertionRef        string
	ValidFrom           IntervalInstantV1
	ValidTo             IntervalEndV1
}

// AcceptedReceiptInventoryV1 is a claimed complete owner-local receipt-log prefix.
// This package verifies its internal structure and projection, not its log proof or completeness.
type AcceptedReceiptInventoryV1 struct {
	OwnerRef           string
	OwnerHistoryRef    string
	OwnerSequenceCut   uint64
	TreeSize           uint64
	OwnerLogRootDigest string
	Receipts           []AcceptedReceiptProjectionV1
}

type ProjectionReplayDispositionV1 string

type ReceiptInventoryAuthenticationV1 string

const (
	ProjectionMatchesSuppliedAcceptedInventoryV1 ProjectionReplayDispositionV1 = "ProjectionMatchesSuppliedAcceptedInventory"
	ReceiptInventoryAuthenticationNotEvaluatedV1  ReceiptInventoryAuthenticationV1 = "NotEvaluated"
)

type OwnerProjectionReplayAuditV1 struct {
	Disposition                  ProjectionReplayDispositionV1
	ReceiptInventoryAuthentication ReceiptInventoryAuthenticationV1
	OwnerRef                     string
	OwnerHistoryRef              string
	OwnerSequenceCut             uint64
	TreeSize                     uint64
	ReceiptCount                 uint64
	IntervalAssertionCount       uint64
	NonTemporalReceiptCount      uint64
	InventoryDigest              [32]byte
	DerivedIntervalIndexRoot     [32]byte
	AuditDigest                  [32]byte
	HasOwnerGaps                 bool
}

var (
	ErrProjectionInventoryMismatch = errors.New("accepted receipt inventory does not match owner binding")
	ErrProjectionLeafCoverage       = errors.New("receipt log leaf positions are incomplete or non-canonical")
	ErrProjectionSequenceOrder      = errors.New("owner receipt sequences are not strictly increasing within the cut")
	ErrProjectionReceiptClass       = errors.New("unsupported or malformed receipt projection class")
	ErrProjectionIndexRootMismatch  = errors.New("replayed temporal index root does not match owner binding")
	ErrProjectionAuditMismatch      = errors.New("projection audit digest does not match owner binding")
)

// ComputeOwnerProjectionReplayAuditV1 independently rebuilds the interval index from the
// supplied decoded receipt inventory and computes its deterministic projection audit digest.
//
// This function does NOT authenticate receipt bytes, signatures, the owner log root, the
// inventory's prefix completeness, the checkpoint, or the verifier frontier. In particular,
// a caller cannot promote this result to source-complete merely because this replay succeeds.
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
	if inventory.TreeSize > maxProjectionReceiptsV1 || uint64(len(inventory.Receipts)) != inventory.TreeSize {
		return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
	}

	records := make([]IntervalRecordV1, 0, len(inventory.Receipts))
	var nonTemporal uint64
	var previousSequence uint64
	for i, receipt := range inventory.Receipts {
		if receipt.ReceiptLogLeafIndex != uint64(i) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionLeafCoverage
		}
		if receipt.OwnerSequence == 0 || receipt.OwnerSequence > binding.OwnerSequence ||
			(i > 0 && receipt.OwnerSequence <= previousSequence) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionSequenceOrder
		}
		if ownerSequenceIsGapV1(binding.Gaps, receipt.OwnerSequence) {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionSequenceOrder
		}
		if validateIntervalDigestV1(receipt.ReceiptDigest) != nil {
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
		}
		previousSequence = receipt.OwnerSequence

		switch receipt.Class {
		case ReceiptProjectsIntervalAssertionV1:
			record := IntervalRecordV1{
				AssertionRef: receipt.AssertionRef,
				ValidFrom: receipt.ValidFrom,
				ValidTo: receipt.ValidTo,
				ReceiptDigest: receipt.ReceiptDigest,
			}
			if validateIntervalRecordV1(record) != nil {
				return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
			}
			records = append(records, record)
		case ReceiptIsNonTemporalV1:
			if receipt.AssertionRef != "" ||
				receipt.ValidFrom != (IntervalInstantV1{}) ||
				receipt.ValidTo != (IntervalEndV1{}) {
				return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
			}
			nonTemporal++
		default:
			return OwnerProjectionReplayAuditV1{}, ErrProjectionReceiptClass
		}
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
		uint64(len(inventory.Receipts)), uint64(len(records)), nonTemporal,
	)
	hasGaps := len(binding.Gaps) > 0
	return OwnerProjectionReplayAuditV1{
		Disposition: ProjectionMatchesSuppliedAcceptedInventoryV1,
		ReceiptInventoryAuthentication: ReceiptInventoryAuthenticationNotEvaluatedV1,
		OwnerRef: binding.OwnerRef,
		OwnerHistoryRef: binding.OwnerHistoryRef,
		OwnerSequenceCut: binding.OwnerSequence,
		TreeSize: binding.TreeSize,
		ReceiptCount: uint64(len(inventory.Receipts)),
		IntervalAssertionCount: uint64(len(records)),
		NonTemporalReceiptCount: nonTemporal,
		InventoryDigest: inventoryDigest,
		DerivedIntervalIndexRoot: tree.RootHash(),
		AuditDigest: auditDigest,
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
	_ = binary.Write(&preimage, binary.BigEndian, uint64(len(inventory.Receipts)))
	for _, receipt := range inventory.Receipts {
		_ = binary.Write(&preimage, binary.BigEndian, receipt.ReceiptLogLeafIndex)
		_ = binary.Write(&preimage, binary.BigEndian, receipt.OwnerSequence)
		appendForestStringV1(&preimage, receipt.ReceiptDigest)
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
