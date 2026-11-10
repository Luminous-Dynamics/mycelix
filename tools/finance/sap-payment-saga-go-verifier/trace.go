package sagaverifier

import (
	"crypto/sha256"
	"encoding/hex"
	"errors"
)

const (
	traceDomain       = "MXSAP-SAGA-TRACE-V1\x00"
	traceSeedMultiply = uint64(0xD6E8FEB86659FD93)
	splitMixGamma     = uint64(0x9E3779B97F4A7C15)
	splitMixMul1      = uint64(0xBF58476D1CE4E5B9)
	splitMixMul2      = uint64(0x94D049BB133111EB)
	MaxTraceCommands  = 64
)

type TraceOpcode uint8

const (
	TraceCreateIntent TraceOpcode = iota + 1
	TraceAuthorize
	TracePrepareLocal
	TraceCommitLocal
	TracePrepareTreasury
	TraceDispatchTreasury
	TraceMarkIndeterminate
	TraceReconcileTreasury
	TraceRetrySameEffect
	TraceCommitTreasury
	TraceCheckpointPayments
	TraceCheckpointTreasury
	TraceMutateIntent
	TraceDuplicateDelivery
	TraceMutateEvent
	TraceStaleCheckpoint
	TraceForkCheckpoint
	TraceCompensate
)

type TraceOwner uint8

const (
	TraceSender TraceOwner = iota + 1
	TracePayments
	TraceTreasury
	TraceAdversary
)

type TraceFault uint8

const (
	TraceNoFault TraceFault = iota
	TraceResponseLostBeforeEffect
	TraceResponseLostAfterEffect
	TraceLookupUncertain
	TraceReceiptMismatch
	TraceStaleFrontier
	TraceWrongOwner
	TracePayloadMutation
	TraceDuplicateDeliveryFault
	TraceForkedCheckpoint
	TraceMissingDependency
	TraceAuthoritativeAbsence
)

type TraceCommandV1 struct {
	Opcode  TraceOpcode
	Owner   TraceOwner
	Fault   TraceFault
	Subject uint64
	Value   uint64
}

type TraceV1 struct {
	Seed          uint64
	TraceIndex    uint64
	Scenario      uint8
	OperationNonce uint64
	Commands      []TraceCommandV1
}

var (
	ErrTraceInvalidScenario = errors.New("trace scenario must be in 1..=8")
	ErrTraceEmptyCommands   = errors.New("trace must contain at least one command")
	ErrTraceTooManyCommands = errors.New("trace exceeds the 64-command bound")
)

func (trace TraceV1) CanonicalBytes() ([]byte, error) {
	if trace.Scenario < 1 || trace.Scenario > 8 {
		return nil, ErrTraceInvalidScenario
	}
	if len(trace.Commands) == 0 {
		return nil, ErrTraceEmptyCommands
	}
	if len(trace.Commands) > MaxTraceCommands {
		return nil, ErrTraceTooManyCommands
	}

	output := make([]byte, 0, 37+len(trace.Commands)*20)
	output = append(output, []byte(traceDomain)...)
	output = appendTraceU64(output, trace.Seed)
	output = appendTraceU64(output, trace.TraceIndex)
	output = append(output, trace.Scenario)
	output = appendTraceU64(output, trace.OperationNonce)
	count := uint32(len(trace.Commands))
	output = append(output,
		byte(count>>24), byte(count>>16), byte(count>>8), byte(count),
	)
	for _, command := range trace.Commands {
		output = append(output, byte(command.Opcode), byte(command.Owner), byte(command.Fault), 0)
		output = appendTraceU64(output, command.Subject)
		output = appendTraceU64(output, command.Value)
	}
	return output, nil
}

func (trace TraceV1) CanonicalDigest() (string, error) {
	encoded, err := trace.CanonicalBytes()
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func appendTraceU64(output []byte, value uint64) []byte {
	return append(output,
		byte(value>>56), byte(value>>48), byte(value>>40), byte(value>>32),
		byte(value>>24), byte(value>>16), byte(value>>8), byte(value),
	)
}

func makeTraceCommand(opcode TraceOpcode, owner TraceOwner, fault TraceFault, subject, value uint64) TraceCommandV1 {
	return TraceCommandV1{
		Opcode: opcode, Owner: owner, Fault: fault, Subject: subject, Value: value,
	}
}

type splitMix64 struct {
	state uint64
}

func newSplitMix64(seed, traceIndex uint64) splitMix64 {
	return splitMix64{state: seed ^ traceIndex*traceSeedMultiply}
}

func (random *splitMix64) nextUint64() uint64 {
	random.state += splitMixGamma
	value := random.state
	value = (value ^ (value >> 30)) * splitMixMul1
	value = (value ^ (value >> 27)) * splitMixMul2
	return value ^ (value >> 31)
}

// GenerateTraceV1 consumes exactly six SplitMix64 draws in this fixed order:
// scenario, operation nonce, effect nonce, checkpoint nonce, amount, fee.
func GenerateTraceV1(seed, traceIndex uint64) TraceV1 {
	random := newSplitMix64(seed, traceIndex)
	scenario := uint8(random.nextUint64()%8 + 1)
	operationNonce := random.nextUint64()
	effectNonce := random.nextUint64()
	checkpointNonce := random.nextUint64()
	amount := random.nextUint64()%1_000_000 + 1
	fee := random.nextUint64() % 1_001
	total := amount + fee

	commands := []TraceCommandV1{
		makeTraceCommand(TraceCreateIntent, TraceSender, TraceNoFault, operationNonce, amount),
		makeTraceCommand(TraceAuthorize, TraceSender, TraceNoFault, operationNonce, fee),
		makeTraceCommand(TracePrepareLocal, TracePayments, TraceNoFault, operationNonce, total),
		makeTraceCommand(TraceCommitLocal, TracePayments, TraceNoFault, operationNonce, total),
	}

	switch scenario {
	case 1:
		commands = append(commands,
			makeTraceCommand(TracePrepareTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceCommitTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceCheckpointPayments, TracePayments, TraceNoFault, operationNonce, checkpointNonce),
			makeTraceCommand(TraceCheckpointTreasury, TraceTreasury, TraceNoFault, effectNonce, checkpointNonce),
		)
	case 2:
		commands = append(commands,
			makeTraceCommand(TracePrepareTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceMarkIndeterminate, TraceTreasury, TraceResponseLostBeforeEffect, effectNonce, fee),
			makeTraceCommand(TraceReconcileTreasury, TraceTreasury, TraceLookupUncertain, effectNonce, fee),
			makeTraceCommand(TraceRetrySameEffect, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceCommitTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
		)
	case 3:
		commands = append(commands,
			makeTraceCommand(TracePrepareTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceMarkIndeterminate, TraceTreasury, TraceResponseLostAfterEffect, effectNonce, fee),
			makeTraceCommand(TraceReconcileTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceCommitTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
		)
	case 4:
		commands = append(commands,
			makeTraceCommand(TracePrepareTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceMarkIndeterminate, TraceTreasury, TraceResponseLostAfterEffect, effectNonce, fee),
			makeTraceCommand(TraceReconcileTreasury, TraceTreasury, TraceLookupUncertain, effectNonce, fee),
			makeTraceCommand(TraceCheckpointPayments, TracePayments, TraceMissingDependency, operationNonce, checkpointNonce),
		)
	case 5:
		commands = append(commands,
			makeTraceCommand(TraceMutateIntent, TraceAdversary, TracePayloadMutation, operationNonce, amount+1),
		)
	case 6:
		commands = append(commands,
			makeTraceCommand(TracePrepareTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDuplicateDelivery, TraceAdversary, TraceDuplicateDeliveryFault, effectNonce, fee),
			makeTraceCommand(TraceCommitTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceMutateEvent, TraceAdversary, TracePayloadMutation, operationNonce, checkpointNonce),
		)
	case 7:
		commands = append(commands,
			makeTraceCommand(TraceCheckpointPayments, TracePayments, TraceNoFault, operationNonce, checkpointNonce),
			makeTraceCommand(TraceCheckpointTreasury, TraceTreasury, TraceStaleFrontier, effectNonce, checkpointNonce),
			makeTraceCommand(TraceForkCheckpoint, TraceAdversary, TraceForkedCheckpoint, effectNonce, checkpointNonce),
		)
	case 8:
		commands = append(commands,
			makeTraceCommand(TracePrepareTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceDispatchTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceCommitTreasury, TraceTreasury, TraceNoFault, effectNonce, fee),
			makeTraceCommand(TraceCompensate, TraceSender, TraceNoFault, checkpointNonce, total),
		)
	}

	return TraceV1{
		Seed: seed, TraceIndex: traceIndex, Scenario: scenario,
		OperationNonce: operationNonce, Commands: commands,
	}
}
