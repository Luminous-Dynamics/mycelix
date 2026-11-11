package sagaverifier

import (
	"errors"
)

const maxTraceShrinkAttemptsV1 uint32 = 100_000

type TraceShrinkResultV1 struct {
	Trace                TraceV1
	OriginalDigest       string
	MinimizedDigest      string
	Attempts             uint32
	RemovedCommands      uint8
	ChangedNumericFields uint16
}

var (
	ErrTraceShrinkInvalidInput  = errors.New("input trace is not canonical Trace V1")
	ErrTraceShrinkTargetMissing = errors.New("target failure signature is absent from input")
	ErrTraceShrinkAttemptLimit  = errors.New("trace shrink attempt limit exceeded")
)

type traceNumericFieldV1 uint8

const (
	shrinkOperationNonce traceNumericFieldV1 = iota + 1
	shrinkSubject
	shrinkValue
)

// MinimizeTraceV1 uses deterministic delta debugging while preserving exactly
// targetFailure. The callback owns failure classification; this function does
// not decide whether a trace is safe.
func MinimizeTraceV1(
	input TraceV1,
	targetFailure string,
	failureSignature func(TraceV1) string,
) (TraceShrinkResultV1, error) {
	originalDigest, err := input.CanonicalDigest()
	if err != nil {
		return TraceShrinkResultV1{}, ErrTraceShrinkInvalidInput
	}
	if failureSignature == nil || failureSignature(cloneTraceV1(input)) != targetFailure {
		return TraceShrinkResultV1{}, ErrTraceShrinkTargetMissing
	}
	current := cloneTraceV1(input)
	var attempts uint32
	var removed uint8
	var changed uint16

	granularity := 2
	for {
		length := len(current.Commands)
		if length <= 1 {
			break
		}
		chunkSize := (length + granularity - 1) / granularity
		reduced := false
		for start := 0; start < length; start += chunkSize {
			end := start + chunkSize
			if end > length {
				end = length
			}
			if end-start == length {
				continue
			}
			candidate := cloneTraceV1(current)
			candidate.Commands = append(candidate.Commands[:start], candidate.Commands[end:]...)
			attempts, err = nextTraceShrinkAttemptV1(attempts)
			if err != nil {
				return TraceShrinkResultV1{}, err
			}
			if preservesTraceFailureV1(candidate, targetFailure, failureSignature) {
				removed += uint8(end - start)
				current = candidate
				if granularity > 2 {
					granularity--
				}
				reduced = true
				break
			}
		}
		if reduced {
			continue
		}
		if granularity >= length {
			break
		}
		granularity *= 2
		if granularity > length {
			granularity = length
		}
	}

	for {
		changedThisPass := false
		nonce := current.OperationNonce
		ok, nextErr := tryTraceNumericReductionV1(
			&current, shrinkOperationNonce, 0, nonce,
			targetFailure, failureSignature, &attempts,
		)
		if nextErr != nil {
			return TraceShrinkResultV1{}, nextErr
		}
		if ok {
			changed++
			changedThisPass = true
		}
		for index := range current.Commands {
			subject := current.Commands[index].Subject
			ok, nextErr = tryTraceNumericReductionV1(
				&current, shrinkSubject, index, subject,
				targetFailure, failureSignature, &attempts,
			)
			if nextErr != nil {
				return TraceShrinkResultV1{}, nextErr
			}
			if ok {
				changed++
				changedThisPass = true
			}
			value := current.Commands[index].Value
			ok, nextErr = tryTraceNumericReductionV1(
				&current, shrinkValue, index, value,
				targetFailure, failureSignature, &attempts,
			)
			if nextErr != nil {
				return TraceShrinkResultV1{}, nextErr
			}
			if ok {
				changed++
				changedThisPass = true
			}
		}
		if !changedThisPass {
			break
		}
	}

	minimizedDigest, err := current.CanonicalDigest()
	if err != nil {
		return TraceShrinkResultV1{}, ErrTraceShrinkInvalidInput
	}
	return TraceShrinkResultV1{
		Trace: current, OriginalDigest: originalDigest, MinimizedDigest: minimizedDigest,
		Attempts: attempts, RemovedCommands: removed, ChangedNumericFields: changed,
	}, nil
}

func cloneTraceV1(input TraceV1) TraceV1 {
	clone := input
	clone.Commands = append([]TraceCommandV1(nil), input.Commands...)
	return clone
}

func preservesTraceFailureV1(
	trace TraceV1,
	target string,
	signature func(TraceV1) string,
) bool {
	if _, err := trace.CanonicalBytes(); err != nil {
		return false
	}
	return signature(cloneTraceV1(trace)) == target
}

func nextTraceShrinkAttemptV1(attempts uint32) (uint32, error) {
	next := attempts + 1
	if next > maxTraceShrinkAttemptsV1 {
		return attempts, ErrTraceShrinkAttemptLimit
	}
	return next, nil
}

func tryTraceNumericReductionV1(
	trace *TraceV1,
	field traceNumericFieldV1,
	commandIndex int,
	original uint64,
	target string,
	signature func(TraceV1) string,
	attempts *uint32,
) (bool, error) {
	candidates := []uint64{0, 1, original / 2, original - boolToUint64(original > 0)}
	seen := make(map[uint64]struct{}, len(candidates))
	for _, value := range candidates {
		if value >= original {
			continue
		}
		if _, ok := seen[value]; ok {
			continue
		}
		seen[value] = struct{}{}
		candidate := cloneTraceV1(*trace)
		switch field {
		case shrinkOperationNonce:
			candidate.OperationNonce = value
		case shrinkSubject:
			candidate.Commands[commandIndex].Subject = value
		case shrinkValue:
			candidate.Commands[commandIndex].Value = value
		default:
			return false, nil
		}
		next, err := nextTraceShrinkAttemptV1(*attempts)
		if err != nil {
			return false, err
		}
		*attempts = next
		if preservesTraceFailureV1(candidate, target, signature) {
			*trace = candidate
			return true, nil
		}
	}
	return false, nil
}

func boolToUint64(value bool) uint64 {
	if value {
		return 1
	}
	return 0
}
