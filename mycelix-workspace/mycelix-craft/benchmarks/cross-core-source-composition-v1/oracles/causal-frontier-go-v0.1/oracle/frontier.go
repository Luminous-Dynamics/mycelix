// Package oracle validates a bounded, single-epoch-per-stream causal frontier.
// It is a test-only independent reference, not a production log or trust service.
package oracle

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"sort"
	"strings"
)

type Status string

const (
	StatusValid      Status = "Valid"
	StatusIncomplete Status = "Incomplete"
	StatusInvalid    Status = "Invalid"
)

type Ordering string

const (
	OrderBefore       Ordering = "Before"
	OrderAfter        Ordering = "After"
	OrderEqual        Ordering = "Equal"
	OrderIncomparable Ordering = "Incomparable"
	OrderUnknown      Ordering = "Unknown"
)

type Entry struct {
	ID         string   `json:"id"`
	StreamID   string   `json:"stream_id"`
	Epoch      uint64   `json:"epoch"`
	Offset     uint64   `json:"offset"`
	Digest     string   `json:"digest"`
	Parents    []string `json:"parents"`
	ObservedAt string   `json:"observed_at,omitempty"`
}

type StreamCut struct {
	StreamID   string `json:"stream_id"`
	Epoch      uint64 `json:"epoch"`
	Offset     uint64 `json:"offset"`
	HeadDigest string `json:"head_digest"`
}

type Frontier struct {
	Cuts []StreamCut `json:"cuts"`
}

type Failure struct {
	Code   string `json:"code"`
	Target string `json:"target,omitempty"`
}

type StreamHead struct {
	StreamID string `json:"stream_id"`
	Epoch    uint64 `json:"epoch"`
	Offset   uint64 `json:"offset"`
	Digest   string `json:"digest"`
}

type Receipt struct {
	SchemaID                   string       `json:"schema_id"`
	Status                     Status       `json:"status"`
	SourceUniverseCompleteness string       `json:"source_universe_completeness"`
	IncludedEventIDs           []string     `json:"included_event_ids"`
	IncludedEntriesSHA256      string       `json:"included_entries_sha256"`
	Heads                      []StreamHead `json:"heads"`
	Failures                   []Failure    `json:"failures"`
	ReceiptSHA256              string       `json:"receipt_sha256"`
}

type Projection struct {
	Receipt Receipt
	entries map[string]Entry
	parents map[string][]string
}

func isDigest(value string) bool {
	if len(value) != sha256.Size*2 || strings.ToLower(value) != value {
		return false
	}
	decoded, err := hex.DecodeString(value)
	return err == nil && len(decoded) == sha256.Size
}

func sortedUnique(values []string) bool {
	for i := 1; i < len(values); i++ {
		if values[i-1] >= values[i] {
			return false
		}
	}
	return true
}

func canonicalFailures(values []Failure) []Failure {
	out := append([]Failure(nil), values...)
	sort.Slice(out, func(i, j int) bool {
		if out[i].Code != out[j].Code {
			return out[i].Code < out[j].Code
		}
		return out[i].Target < out[j].Target
	})
	return out
}

// Validate applies a frontier over one declared epoch per stream. Entries
// beyond a selected stream cut are deliberately not semantically validated:
// malformed future payloads cannot poison an earlier historical projection.
// The result never claims source-universe completeness or authenticates log
// heads; those are separate trust-owner responsibilities.
func Validate(entries []Entry, frontier Frontier) Projection {
	const maxStreamsV1 = 4096
	const maxIncludedEntriesV1 = 1_000_000
	const maxOffsetV1 = uint64(1_000_000)
	included := make([]Entry, 0)
	cutByStream := make(map[string]StreamCut, len(frontier.Cuts))
	failures := make([]Failure, 0)

	if len(frontier.Cuts) == 0 {
		failures = append(failures, Failure{Code: "EMPTY_FRONTIER"})
	}
	if len(frontier.Cuts) > maxStreamsV1 {
		failures = append(failures, Failure{Code: "STREAM_LIMIT_EXCEEDED"})
	}
	for i, cut := range frontier.Cuts {
		if strings.TrimSpace(cut.StreamID) == "" || cut.StreamID != strings.TrimSpace(cut.StreamID) || cut.Epoch == 0 || cut.Offset == 0 || cut.Offset > maxOffsetV1 || !isDigest(cut.HeadDigest) {
			failures = append(failures, Failure{Code: "INVALID_CUT", Target: cut.StreamID})
			continue
		}
		if i > 0 && frontier.Cuts[i-1].StreamID >= cut.StreamID {
			failures = append(failures, Failure{Code: "NON_CANONICAL_CUT_ORDER", Target: cut.StreamID})
		}
		if _, exists := cutByStream[cut.StreamID]; exists {
			failures = append(failures, Failure{Code: "DUPLICATE_STREAM_CUT", Target: cut.StreamID})
			continue
		}
		cutByStream[cut.StreamID] = cut
	}

	// Select by the immutable admission coordinate first. Payload-level checks
	// happen only after selection, so future records are outside this query.
	for _, entry := range entries {
		cut, ok := cutByStream[entry.StreamID]
		if !ok || entry.Epoch != cut.Epoch || entry.Offset > cut.Offset {
			continue
		}
		// Canonicalize the semantically empty parent set so omitted/null and
		// explicit empty arrays produce the same entry-set commitment.
		if entry.Parents == nil {
			entry.Parents = []string{}
		}
		included = append(included, entry)
	}
	if len(included) > maxIncludedEntriesV1 {
		failures = append(failures, Failure{Code: "ENTRY_LIMIT_EXCEEDED"})
	}
	sort.Slice(included, func(i, j int) bool {
		if included[i].StreamID != included[j].StreamID {
			return included[i].StreamID < included[j].StreamID
		}
		if included[i].Epoch != included[j].Epoch {
			return included[i].Epoch < included[j].Epoch
		}
		if included[i].Offset != included[j].Offset {
			return included[i].Offset < included[j].Offset
		}
		return included[i].ID < included[j].ID
	})

	byID := make(map[string]Entry, len(included))
	byCoordinate := make(map[string]Entry, len(included))
	byStreamOffset := make(map[string]map[uint64]Entry, len(cutByStream))
	for _, entry := range included {
		if strings.TrimSpace(entry.ID) == "" || entry.ID != strings.TrimSpace(entry.ID) || strings.TrimSpace(entry.StreamID) == "" || entry.Epoch == 0 || entry.Offset == 0 {
			failures = append(failures, Failure{Code: "INVALID_ENTRY_ENVELOPE", Target: entry.ID})
			continue
		}
		if !isDigest(entry.Digest) {
			failures = append(failures, Failure{Code: "INVALID_ENTRY_DIGEST", Target: entry.ID})
		}
		if !sortedUnique(entry.Parents) && len(entry.Parents) > 0 {
			failures = append(failures, Failure{Code: "NON_CANONICAL_PARENT_SET", Target: entry.ID})
		}
		if _, exists := byID[entry.ID]; exists {
			failures = append(failures, Failure{Code: "DUPLICATE_EVENT_ID", Target: entry.ID})
		} else {
			byID[entry.ID] = entry
		}
		coordinate := fmt.Sprintf("%s/%d/%d", entry.StreamID, entry.Epoch, entry.Offset)
		if previous, exists := byCoordinate[coordinate]; exists {
			code := "DUPLICATE_STREAM_OFFSET"
			if previous.Digest != entry.Digest || previous.ID != entry.ID {
				code = "STREAM_FORK"
			}
			failures = append(failures, Failure{Code: code, Target: coordinate})
		} else {
			byCoordinate[coordinate] = entry
		}
		if byStreamOffset[entry.StreamID] == nil {
			byStreamOffset[entry.StreamID] = make(map[uint64]Entry)
		}
		byStreamOffset[entry.StreamID][entry.Offset] = entry
	}

	// Each cut must be backed by a contiguous prefix in its declared epoch.
	heads := make([]StreamHead, 0, len(cutByStream))
	for streamID, cut := range cutByStream {
		streamEntries := byStreamOffset[streamID]
		if uint64(len(streamEntries)) != cut.Offset {
			failures = append(failures, Failure{Code: "MISSING_PREFIX_OFFSET", Target: fmt.Sprintf("%s/%d/1..%d", streamID, cut.Epoch, cut.Offset)})
		}
		for offset := uint64(1); offset <= uint64(len(streamEntries)); offset++ {
			if _, exists := streamEntries[offset]; !exists {
				failures = append(failures, Failure{Code: "MISSING_PREFIX_OFFSET", Target: fmt.Sprintf("%s/%d/%d", streamID, cut.Epoch, offset)})
			}
		}
		head, exists := streamEntries[cut.Offset]
		if !exists {
			failures = append(failures, Failure{Code: "MISSING_STREAM_HEAD", Target: streamID})
			continue
		}
		if head.Digest != cut.HeadDigest {
			failures = append(failures, Failure{Code: "HEAD_DIGEST_MISMATCH", Target: streamID})
		}
		heads = append(heads, StreamHead{StreamID: streamID, Epoch: cut.Epoch, Offset: cut.Offset, Digest: cut.HeadDigest})
	}
	sort.Slice(heads, func(i, j int) bool { return heads[i].StreamID < heads[j].StreamID })

	// Verify downward closure: every included event must name parents that are
	// also included. Parent references establish causality across streams; clocks
	// and observed-at timestamps do not.
	parentMap := make(map[string][]string, len(byID))
	for id, entry := range byID {
		for _, parentID := range entry.Parents {
			parent, exists := byID[parentID]
			if !exists {
				failures = append(failures, Failure{Code: "MISSING_CAUSAL_PARENT", Target: id + "->" + parentID})
				continue
			}
			if parent.StreamID == entry.StreamID && parent.Epoch == entry.Epoch && parent.Offset >= entry.Offset {
				failures = append(failures, Failure{Code: "NON_CAUSAL_SAME_STREAM_PARENT", Target: id + "->" + parentID})
			}
		}
		parentMap[id] = append([]string(nil), entry.Parents...)
	}
	if hasCycle(parentMap) {
		failures = append(failures, Failure{Code: "CAUSAL_CYCLE"})
	}

	status := StatusValid
	for _, failure := range failures {
		switch failure.Code {
		case "MISSING_PREFIX_OFFSET", "MISSING_STREAM_HEAD", "MISSING_CAUSAL_PARENT":
			if status != StatusInvalid {
				status = StatusIncomplete
			}
		default:
			status = StatusInvalid
		}
	}

	ids := make([]string, 0, len(byID))
	for id := range byID {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	failures = canonicalFailures(failures)
	entryBytes, _ := json.Marshal(included)
	entryDigest := sha256.Sum256(append([]byte("MYCELIX-CRAFT-CAUSAL-FRONTIER-ENTRIES-V0.1\x00"), entryBytes...))
	entrySetSHA256 := hex.EncodeToString(entryDigest[:])
	canonical := struct {
		SchemaID                   string       `json:"schema_id"`
		Status                     Status       `json:"status"`
		SourceUniverseCompleteness string       `json:"source_universe_completeness"`
		Cuts                       []StreamHead `json:"cuts"`
		IDs                        []string     `json:"included_event_ids"`
		EntrySetSHA256             string       `json:"included_entries_sha256"`
		Failures                   []Failure    `json:"failures"`
	}{
		SchemaID:                   "mycelix.craft-causal-frontier-receipt.v0.1",
		Status:                     status,
		SourceUniverseCompleteness: "NotEvaluated",
		Cuts:                       heads,
		IDs:                        ids,
		EntrySetSHA256:             entrySetSHA256,
		Failures:                   failures,
	}
	bytes, _ := json.Marshal(canonical)
	digest := sha256.Sum256(append([]byte("MYCELIX-CRAFT-CAUSAL-FRONTIER-V0.1\x00"), bytes...))

	return Projection{
		Receipt: Receipt{
			SchemaID:                   canonical.SchemaID,
			Status:                     status,
			SourceUniverseCompleteness: "NotEvaluated",
			IncludedEventIDs:           ids,
			IncludedEntriesSHA256:      entrySetSHA256,
			Heads:                      heads,
			Failures:                   failures,
			ReceiptSHA256:              hex.EncodeToString(digest[:]),
		},
		entries: byID,
		parents: parentMap,
	}
}

func hasCycle(parents map[string][]string) bool {
	type frame struct {
		id   string
		next int
	}
	state := make(map[string]uint8, len(parents))
	for root := range parents {
		if state[root] != 0 {
			continue
		}
		state[root] = 1
		stack := []frame{{id: root}}
		for len(stack) > 0 {
			top := &stack[len(stack)-1]
			children := parents[top.id]
			if top.next >= len(children) {
				state[top.id] = 2
				stack = stack[:len(stack)-1]
				continue
			}
			child := children[top.next]
			top.next++
			if _, exists := parents[child]; !exists {
				continue
			}
			switch state[child] {
			case 1:
				return true
			case 2:
				continue
			default:
				state[child] = 1
				stack = append(stack, frame{id: child})
			}
		}
	}
	return false
}

// Compare returns a causal relation only when a verified parent path connects
// the two selected events. Timestamp equality or lexical order does not create
// a causal edge.
func (p Projection) Compare(a, b string) Ordering {
	if _, ok := p.entries[a]; !ok {
		return OrderUnknown
	}
	if _, ok := p.entries[b]; !ok {
		return OrderUnknown
	}
	if a == b {
		return OrderEqual
	}
	if p.isAncestor(a, b) {
		return OrderBefore
	}
	if p.isAncestor(b, a) {
		return OrderAfter
	}
	return OrderIncomparable
}

func (p Projection) isAncestor(ancestor, descendant string) bool {
	seen := make(map[string]bool)
	stack := append([]string(nil), p.parents[descendant]...)
	for len(stack) > 0 {
		current := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if current == ancestor {
			return true
		}
		if seen[current] {
			continue
		}
		seen[current] = true
		stack = append(stack, p.parents[current]...)
	}
	return false
}
