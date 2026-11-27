// Package hindsight implements a deliberately small, independent oracle for
// verifier-relative historical temporal replay. It has no dependency on the
// candidate temporal-index implementation.
package hindsight

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sort"
)

type Dimension uint8

const (
	NonTemporal Dimension = iota
	ValidFrom
	ValidTo
)

type Event struct {
	Owner           string
	OwnerHistoryRef string
	Sequence        uint64
	ID              string
	Namespace       string
	Subject         string
	Claim           string
	IntervalRef     string
	Dimension       Dimension
	Value           int64
	ExplicitOpen    bool
	Supersedes      string
}

type OwnerCut struct {
	OwnerHistoryRef string
	Sequence        uint64
}

type Cut map[string]OwnerCut

type Scope struct {
	Verifier  string
	Policy    string
	Namespace string
	Subject   string
	Claim     string
}

type Match struct {
	Owner       string
	IntervalRef string
	Start       int64
	Open        bool
	End         int64
	StartID     string
	EndID       string
}

type Result struct {
	Disposition string
	Reason      string
	Matches     []Match
	Frontier    Cut
	Certificate string
}

var ErrIndeterminate = errors.New("temporal replay is indeterminate")

// Evaluate applies an exact owner-vector cut. It intentionally scans the
// prefix and resolves endpoint supersession there; it does not use timestamps
// as an ordering mechanism and does not inspect post-cut suffix events.
func Evaluate(requiredOwners []string, histories map[string][]Event, cut Cut, scope Scope, at int64) Result {
	fail := func(reason string) Result {
		r := Result{Disposition: "Indeterminate", Reason: reason, Frontier: cloneCut(cut)}
		r.Certificate = certificate(scope, at, cut, r)
		return r
	}
	if scope.Verifier == "" || scope.Policy == "" || scope.Namespace == "" || scope.Subject == "" || scope.Claim == "" {
		return fail("incomplete query scope")
	}
	owners := append([]string(nil), requiredOwners...)
	sort.Strings(owners)
	if len(owners) == 0 {
		return fail("empty required owner set")
	}
	for i, owner := range owners {
		if owner == "" || (i > 0 && owners[i-1] == owner) {
			return fail("malformed or duplicate required owner")
		}
	}
	if len(histories) != len(owners) || len(cut) != len(owners) {
		return fail("owner set or frontier cardinality mismatch")
	}
	for _, owner := range owners {
		if _, ok := histories[owner]; !ok {
			return fail("required owner history missing: " + owner)
		}
		ownerCut, ok := cut[owner]
		if !ok || ownerCut.OwnerHistoryRef == "" {
			return fail("required owner cut/history identity missing: " + owner)
		}
	}
	for owner := range histories {
		if !contains(owners, owner) {
			return fail("unexpected owner history: " + owner)
		}
	}

	matches := make([]Match, 0)
	for _, owner := range owners {
		ownerCut := cut[owner]
		prefix := make([]Event, 0)
		for _, event := range histories[owner] {
			// The historical verifier must not inspect suffix contents at all.
			// Even malformed or not-yet-delivered records beyond this owner's
			// accepted cut cannot change the result at the earlier frontier.
			if event.Sequence > ownerCut.Sequence {
				continue
			}
			if event.Owner != owner || event.OwnerHistoryRef != ownerCut.OwnerHistoryRef || event.Sequence == 0 {
				return fail("owner/history identity mismatch or zero sequence: " + owner)
			}
			prefix = append(prefix, event)
		}
		sort.Slice(prefix, func(i, j int) bool { return prefix[i].Sequence < prefix[j].Sequence })
		if uint64(len(prefix)) != ownerCut.Sequence {
			return fail("owner prefix has missing or duplicate sequence positions: " + owner)
		}
		for i := range prefix {
			if prefix[i].Sequence != uint64(i+1) {
				return fail("owner prefix is not contiguous: " + owner)
			}
		}

		type intervalKey struct{ namespace, subject, claim, ref string }
		type endpointKey struct {
			intervalKey
			dimension Dimension
		}
		groups := make(map[intervalKey]struct{})
		byEndpoint := make(map[endpointKey][]Event)
		ids := make(map[string]Event)
		for _, event := range prefix {
			if event.ID == "" {
				return fail("empty event identifier: " + owner)
			}
			if _, exists := ids[event.ID]; exists {
				return fail("duplicate event identifier: " + event.ID)
			}
			ids[event.ID] = event
			if event.Dimension == NonTemporal {
				if event.IntervalRef != "" || event.ExplicitOpen || event.Supersedes != "" {
					return fail("non-temporal event carries interval metadata: " + event.ID)
				}
				continue
			}
			if event.Dimension != ValidFrom && event.Dimension != ValidTo {
				return fail("unknown temporal dimension: " + event.ID)
			}
			if event.Namespace == "" || event.Subject == "" || event.Claim == "" || event.IntervalRef == "" {
				return fail("temporal endpoint missing namespace/subject/claim/interval identity: " + event.ID)
			}
			if event.ExplicitOpen && event.Dimension != ValidTo {
				return fail("explicit OPEN is permitted only for VALID_TO: " + event.ID)
			}
			// Endpoint pairing is query-scoped. Other subjects/claims/namespaces
			// remain in the owner's log but cannot contaminate this query's
			// interval interpretation.
			if event.Namespace != scope.Namespace || event.Subject != scope.Subject || event.Claim != scope.Claim {
				continue
			}
			key := intervalKey{event.Namespace, event.Subject, event.Claim, event.IntervalRef}
			groups[key] = struct{}{}
			byEndpoint[endpointKey{key, event.Dimension}] = append(byEndpoint[endpointKey{key, event.Dimension}], event)
		}

		for key := range groups {
			start, startOK, startErr := resolveHead(byEndpoint[endpointKey{key, ValidFrom}], ids)
			if startErr != "" {
				return fail(startErr)
			}
			end, endOK, endErr := resolveHead(byEndpoint[endpointKey{key, ValidTo}], ids)
			if endErr != "" {
				return fail(endErr)
			}
			if !startOK || !endOK {
				return fail("unpaired validity endpoint; missing endpoint is not OPEN: " + key.ref)
			}
			if !end.ExplicitOpen && end.Value < start.Value {
				return fail("finite validity end precedes start: " + key.ref)
			}
			if start.Value <= at && (end.ExplicitOpen || at < end.Value) {
				matches = append(matches, Match{Owner: owner, IntervalRef: key.ref,
					Start: start.Value, Open: end.ExplicitOpen, End: end.Value,
					StartID: start.ID, EndID: end.ID})
			}
		}
	}
	sort.Slice(matches, func(i, j int) bool {
		if matches[i].Owner != matches[j].Owner {
			return matches[i].Owner < matches[j].Owner
		}
		return matches[i].IntervalRef < matches[j].IntervalRef
	})
	r := Result{Disposition: "CompleteForSuppliedOwnerPrefixes", Matches: matches, Frontier: cloneCut(cut)}
	r.Certificate = certificate(scope, at, cut, r)
	return r
}

func resolveHead(events []Event, ids map[string]Event) (Event, bool, string) {
	if len(events) == 0 {
		return Event{}, false, ""
	}
	superseded := make(map[string]bool)
	for _, event := range events {
		if event.Supersedes == "" {
			continue
		}
		prior, ok := ids[event.Supersedes]
		if !ok {
			return Event{}, false, "supersession target is not in this accepted prefix: " + event.ID
		}
		if prior.Owner != event.Owner || prior.OwnerHistoryRef != event.OwnerHistoryRef ||
			prior.Namespace != event.Namespace || prior.Subject != event.Subject || prior.Claim != event.Claim ||
			prior.IntervalRef != event.IntervalRef || prior.Dimension != event.Dimension {
			return Event{}, false, "supersession crosses owner, claim, interval or endpoint dimension: " + event.ID
		}
		if superseded[event.Supersedes] {
			return Event{}, false, "endpoint has multiple superseders: " + event.Supersedes
		}
		superseded[event.Supersedes] = true
	}
	heads := make([]Event, 0, len(events))
	for _, event := range events {
		if !superseded[event.ID] {
			heads = append(heads, event)
		}
	}
	if len(heads) != 1 {
		return Event{}, false, "conflicting endpoint heads at accepted frontier"
	}
	return heads[0], true, ""
}

func certificate(scope Scope, at int64, cut Cut, result Result) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-HISTORICAL-TEMPORAL-REPLAY-V1")
	writeString(&b, scope.Verifier)
	writeString(&b, scope.Policy)
	writeString(&b, scope.Namespace)
	writeString(&b, scope.Subject)
	writeString(&b, scope.Claim)
	_ = binary.Write(&b, binary.BigEndian, at)
	owners := make([]string, 0, len(cut))
	for owner := range cut {
		owners = append(owners, owner)
	}
	sort.Strings(owners)
	_ = binary.Write(&b, binary.BigEndian, uint32(len(owners)))
	for _, owner := range owners {
		writeString(&b, owner)
		writeString(&b, cut[owner].OwnerHistoryRef)
		_ = binary.Write(&b, binary.BigEndian, cut[owner].Sequence)
	}
	writeString(&b, result.Disposition)
	writeString(&b, result.Reason)
	_ = binary.Write(&b, binary.BigEndian, uint32(len(result.Matches)))
	for _, m := range result.Matches {
		writeString(&b, m.Owner)
		writeString(&b, m.IntervalRef)
		_ = binary.Write(&b, binary.BigEndian, m.Start)
		if m.Open {
			b.WriteByte(1)
		} else {
			b.WriteByte(0)
			_ = binary.Write(&b, binary.BigEndian, m.End)
		}
		writeString(&b, m.StartID)
		writeString(&b, m.EndID)
	}
	d := sha256.Sum256(b.Bytes())
	return hex.EncodeToString(d[:])
}

func writeString(b *bytes.Buffer, s string) {
	_ = binary.Write(b, binary.BigEndian, uint32(len(s)))
	_, _ = b.WriteString(s)
}

func cloneCut(c Cut) Cut {
	out := make(Cut, len(c))
	for k, v := range c {
		out[k] = v
	}
	return out
}

func contains(xs []string, s string) bool {
	for _, x := range xs {
		if x == s {
			return true
		}
	}
	return false
}

// LatestStateMutant intentionally ignores the requested cut and resolves over
// every visible event. Tests use it as a negative control; production code must not.
func LatestStateMutant(requiredOwners []string, histories map[string][]Event, cut Cut, scope Scope, at int64) Result {
	latest := make(Cut, len(cut))
	for owner, events := range histories {
		var max uint64
		var historyRef string
		for _, event := range events {
			if event.Sequence > max {
				max = event.Sequence
				historyRef = event.OwnerHistoryRef
			}
		}
		latest[owner] = OwnerCut{OwnerHistoryRef: historyRef, Sequence: max}
	}
	return Evaluate(requiredOwners, histories, latest, scope, at)
}

func ValidateResult(r Result) error {
	if r.Disposition != "CompleteForSuppliedOwnerPrefixes" && r.Disposition != "Indeterminate" {
		return fmt.Errorf("unknown disposition: %s", r.Disposition)
	}
	if r.Certificate == "" {
		return ErrIndeterminate
	}
	return nil
}
