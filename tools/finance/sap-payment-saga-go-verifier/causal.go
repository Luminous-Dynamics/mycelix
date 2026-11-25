// Package sagaverifier contains an independent owner-scoped causal graph
// verifier. It intentionally does not assign a global clock or sort events
// into a total order when no explicit causal relationship exists.
package sagaverifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sort"
	"unicode/utf8"
)

const (
	CausalGraphSchemaV1 uint16 = 1
	maxCausalEventsV1 = 100000
	maxCausalRefsPerEventV1 = 64
	maxCausalComponentBytesV1 = 1024
)

var causalGraphDomainV1 = []byte("mycelix:sap-causal-graph:v1\x00")

var (
	ErrCausalEmptyGraph = errors.New("causal graph is empty")
	ErrCausalTooManyEvents = errors.New("causal graph exceeds event bound")
	ErrCausalTooManyReferences = errors.New("event exceeds causal reference bound")
	ErrCausalInvalidComponent = errors.New("causal graph component is empty, oversized, or invalid UTF-8")
	ErrCausalDuplicateEventReference = errors.New("duplicate owner-scoped event reference")
	ErrCausalDuplicateOwnerSequence = errors.New("duplicate owner-local sequence")
	ErrCausalOwnerSequenceGap = errors.New("owner-local sequence is not a complete prefix")
	ErrCausalUnexpectedGenesisPredecessor = errors.New("genesis event has an unexpected predecessor")
	ErrCausalMissingOwnerPredecessor = errors.New("non-genesis event is missing its owner predecessor")
	ErrCausalWrongOwnerPredecessor = errors.New("owner predecessor is not the exact previous owner event")
	ErrCausalDuplicateReference = errors.New("event repeats a causal reference")
	ErrCausalSelfReference = errors.New("event cannot causally depend on itself")
	ErrCausalMissingReference = errors.New("causal reference does not resolve in supplied graph")
	ErrCausalCycle = errors.New("causal graph contains a cycle")
	ErrCausalCanonicalLengthOverflow = errors.New("canonical graph length exceeds u32")
)

// CausalEventRefV1 is scoped by owner. Event IDs are not assumed to be
// globally unique across independent owner journals.
type CausalEventRefV1 struct {
	OwnerID string
	EventID string
}

// CausalEventV1 carries only explicit order information. Timestamps and
// arrival order are deliberately absent.
type CausalEventV1 struct {
	OwnerID          string
	EventID          string
	OwnerSequence    uint64
	OwnerPredecessor *CausalEventRefV1
	CausedBy         []CausalEventRefV1
}

func (e CausalEventV1) reference() CausalEventRefV1 {
	return CausalEventRefV1{OwnerID: e.OwnerID, EventID: e.EventID}
}

type CausalOwnerFrontierV1 struct {
	OwnerID   string
	Sequence  uint64
	EventID   string
}

type CausalProjectionV1 struct {
	// GraphDigest is a domain-separated SHA-256 of the canonical graph
	// subject. It does not authenticate its producer or authorize any effect.
	GraphDigest string
	EventCount  int
	EdgeCount   int
	// OwnerFrontiers are independent; no global sequence is synthesized.
	OwnerFrontiers []CausalOwnerFrontierV1
	// TopologicalLayers are deterministic ready-sets. Members of a layer are
	// incomparable under this graph, not necessarily simultaneous in reality.
	TopologicalLayers [][]CausalEventRefV1
}

type causalOwnerNodeV1 struct {
	sequence uint64
	ref      CausalEventRefV1
	index    int
}

type causalEdgeV1 struct {
	from int
	to   int
}

func causalRefLessV1(a, b CausalEventRefV1) bool {
	if a.OwnerID != b.OwnerID {
		return a.OwnerID < b.OwnerID
	}
	return a.EventID < b.EventID
}

func validateCausalComponentV1(value string) error {
	if value == "" || len(value) > maxCausalComponentBytesV1 || !utf8.ValidString(value) {
		return ErrCausalInvalidComponent
	}
	return nil
}

func appendCausalU32V1(buffer *bytes.Buffer, value int) error {
	if uint64(value) > uint64(^uint32(0)) {
		return ErrCausalCanonicalLengthOverflow
	}
	return binary.Write(buffer, binary.BigEndian, uint32(value))
}

func appendCausalTextV1(buffer *bytes.Buffer, value string) error {
	if err := validateCausalComponentV1(value); err != nil {
		return err
	}
	if err := appendCausalU32V1(buffer, len([]byte(value))); err != nil {
		return err
	}
	_, err := buffer.WriteString(value)
	return err
}

func appendCausalRefV1(buffer *bytes.Buffer, ref CausalEventRefV1) error {
	if err := appendCausalTextV1(buffer, ref.OwnerID); err != nil {
		return err
	}
	return appendCausalTextV1(buffer, ref.EventID)
}

func canonicalCausalGraphBytesV1(events []CausalEventV1) ([]byte, error) {
	ordered := append([]CausalEventV1(nil), events...)
	sort.Slice(ordered, func(i, j int) bool {
		if ordered[i].OwnerID != ordered[j].OwnerID {
			return ordered[i].OwnerID < ordered[j].OwnerID
		}
		if ordered[i].OwnerSequence != ordered[j].OwnerSequence {
			return ordered[i].OwnerSequence < ordered[j].OwnerSequence
		}
		return ordered[i].EventID < ordered[j].EventID
	})

	var buffer bytes.Buffer
	if _, err := buffer.Write(causalGraphDomainV1); err != nil {
		return nil, err
	}
	if err := binary.Write(&buffer, binary.BigEndian, CausalGraphSchemaV1); err != nil {
		return nil, err
	}
	if err := appendCausalU32V1(&buffer, len(ordered)); err != nil {
		return nil, err
	}

	for _, event := range ordered {
		if err := appendCausalTextV1(&buffer, event.OwnerID); err != nil {
			return nil, err
		}
		if err := appendCausalTextV1(&buffer, event.EventID); err != nil {
			return nil, err
		}
		if err := binary.Write(&buffer, binary.BigEndian, event.OwnerSequence); err != nil {
			return nil, err
		}
		if event.OwnerPredecessor == nil {
			if err := buffer.WriteByte(0); err != nil {
				return nil, err
			}
		} else {
			if err := buffer.WriteByte(1); err != nil {
				return nil, err
			}
			if err := appendCausalRefV1(&buffer, *event.OwnerPredecessor); err != nil {
				return nil, err
			}
		}
		causes := append([]CausalEventRefV1(nil), event.CausedBy...)
		sort.Slice(causes, func(i, j int) bool {
			return causalRefLessV1(causes[i], causes[j])
		})
		if err := appendCausalU32V1(&buffer, len(causes)); err != nil {
			return nil, err
		}
		for _, cause := range causes {
			if err := appendCausalRefV1(&buffer, cause); err != nil {
				return nil, err
			}
		}
	}
	return buffer.Bytes(), nil
}

// ProjectCausalGraphV1 validates owner-local prefixes, explicit causal edges,
// and acyclicity, then returns deterministic antichain layers and frontiers.
// The input must contain each owner's complete prefix from sequence zero.
func ProjectCausalGraphV1(events []CausalEventV1) (CausalProjectionV1, error) {
	if len(events) == 0 {
		return CausalProjectionV1{}, ErrCausalEmptyGraph
	}
	if len(events) > maxCausalEventsV1 {
		return CausalProjectionV1{}, ErrCausalTooManyEvents
	}

	byRef := make(map[CausalEventRefV1]int, len(events))
	byOwner := make(map[string][]causalOwnerNodeV1)
	for index, event := range events {
		if err := validateCausalComponentV1(event.OwnerID); err != nil {
			return CausalProjectionV1{}, err
		}
		if err := validateCausalComponentV1(event.EventID); err != nil {
			return CausalProjectionV1{}, err
		}
		ref := event.reference()
		if _, exists := byRef[ref]; exists {
			return CausalProjectionV1{}, ErrCausalDuplicateEventReference
		}
		byRef[ref] = index
		byOwner[event.OwnerID] = append(byOwner[event.OwnerID], causalOwnerNodeV1{
			sequence: event.OwnerSequence,
			ref:      ref,
			index:    index,
		})
		if len(event.CausedBy) > maxCausalRefsPerEventV1 {
			return CausalProjectionV1{}, ErrCausalTooManyReferences
		}
	}

	edgeSet := make(map[causalEdgeV1]struct{})
	frontiers := make([]CausalOwnerFrontierV1, 0, len(byOwner))
	owners := make([]string, 0, len(byOwner))
	for owner := range byOwner {
		owners = append(owners, owner)
	}
	sort.Strings(owners)

	for _, owner := range owners {
		nodes := byOwner[owner]
		sort.Slice(nodes, func(i, j int) bool {
			return nodes[i].sequence < nodes[j].sequence
		})
		var previous *causalOwnerNodeV1
		for expected, node := range nodes {
			if node.sequence != uint64(expected) {
				if previous != nil && previous.sequence == node.sequence {
					return CausalProjectionV1{}, ErrCausalDuplicateOwnerSequence
				}
				return CausalProjectionV1{}, ErrCausalOwnerSequenceGap
			}
			event := events[node.index]
			switch {
			case node.sequence == 0 && event.OwnerPredecessor == nil:
				// Genesis of this complete owner prefix.
			case node.sequence == 0 && event.OwnerPredecessor != nil:
				return CausalProjectionV1{}, ErrCausalUnexpectedGenesisPredecessor
			case node.sequence > 0 && event.OwnerPredecessor == nil:
				return CausalProjectionV1{}, ErrCausalMissingOwnerPredecessor
			case node.sequence > 0 && previous == nil:
				return CausalProjectionV1{}, ErrCausalOwnerSequenceGap
			case node.sequence > 0 && *event.OwnerPredecessor != previous.ref:
				return CausalProjectionV1{}, ErrCausalWrongOwnerPredecessor
			case node.sequence > 0:
				edgeSet[causalEdgeV1{from: previous.index, to: node.index}] = struct{}{}
			}
			copyNode := node
			previous = &copyNode
		}
		if previous != nil {
			frontiers = append(frontiers, CausalOwnerFrontierV1{
				OwnerID:  owner,
				Sequence: previous.sequence,
				EventID:  previous.ref.EventID,
			})
		}
	}

	for index, event := range events {
		current := event.reference()
		seen := make(map[CausalEventRefV1]struct{}, len(event.CausedBy))
		for _, parent := range event.CausedBy {
			if _, exists := seen[parent]; exists {
				return CausalProjectionV1{}, ErrCausalDuplicateReference
			}
			seen[parent] = struct{}{}
			if parent == current {
				return CausalProjectionV1{}, ErrCausalSelfReference
			}
			parentIndex, exists := byRef[parent]
			if !exists {
				return CausalProjectionV1{}, ErrCausalMissingReference
			}
			edgeSet[causalEdgeV1{from: parentIndex, to: index}] = struct{}{}
		}
	}

	adjacency := make([][]int, len(events))
	indegree := make([]int, len(events))
	for edge := range edgeSet {
		adjacency[edge.from] = append(adjacency[edge.from], edge.to)
		indegree[edge.to]++
	}

	references := make([]CausalEventRefV1, len(events))
	ready := make([]int, 0, len(events))
	for index, event := range events {
		references[index] = event.reference()
		if indegree[index] == 0 {
			ready = append(ready, index)
		}
	}

	layers := make([][]CausalEventRefV1, 0)
	processed := 0
	for len(ready) > 0 {
		sort.Slice(ready, func(i, j int) bool {
			return causalRefLessV1(references[ready[i]], references[ready[j]])
		})
		layerIndices := ready
		ready = make([]int, 0)
		layer := make([]CausalEventRefV1, 0, len(layerIndices))
		for _, source := range layerIndices {
			processed++
			layer = append(layer, references[source])
			for _, target := range adjacency[source] {
				indegree[target]--
				if indegree[target] == 0 {
					ready = append(ready, target)
				}
			}
		}
		layers = append(layers, layer)
	}
	if processed != len(events) {
		return CausalProjectionV1{}, ErrCausalCycle
	}

	canonical, err := canonicalCausalGraphBytesV1(events)
	if err != nil {
		return CausalProjectionV1{}, err
	}
	digest := sha256.Sum256(canonical)

	return CausalProjectionV1{
		GraphDigest:       "sha256:" + hex.EncodeToString(digest[:]),
		EventCount:        len(events),
		EdgeCount:         len(edgeSet),
		OwnerFrontiers:    frontiers,
		TopologicalLayers: layers,
	}, nil
}

// FormatCausalGraphError gives stable, non-localized error labels for reports.
func FormatCausalGraphError(err error) string {
	if err == nil {
		return "ok"
	}
	return fmt.Sprintf("%v", err)
}
