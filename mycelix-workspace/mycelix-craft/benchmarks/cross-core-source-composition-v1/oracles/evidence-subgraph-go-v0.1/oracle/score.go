package oracle

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"math"
	"sort"
	"time"
)

type ClaimState string

const (
	ClaimEstablished   ClaimState = "EstablishedWithinProfile"
	ClaimNotEvaluated  ClaimState = "NotEvaluated"
	ClaimIndeterminate ClaimState = "Indeterminate"
	ClaimRejected      ClaimState = "Rejected"
)

type Query struct {
	ID                           string            `json:"id"`
	Scope                        string            `json:"scope"`
	ValidAt                      string            `json:"valid_at"`
	AcceptedFrontier             map[string]uint64 `json:"accepted_frontier"`
	SourceUniverseCompleteness   ClaimState        `json:"source_universe_completeness"`
	ProgressUniverseCompleteness ClaimState        `json:"progress_universe_completeness"`
}

type Node struct {
	ID   string `json:"id"`
	Type string `json:"type"`
}

type Edge struct {
	ID             string `json:"id"`
	From           string `json:"from"`
	Predicate      string `json:"predicate"`
	To             string `json:"to"`
	SourceClaimID  string `json:"source_claim_id"`
	SourceDigest   string `json:"source_digest"`
	StreamID       string `json:"stream_id"`
	AcceptedOffset uint64 `json:"accepted_offset"`
	ValidFrom      string `json:"valid_from"`
	ValidTo        string `json:"valid_to,omitempty"`
	Scope          string `json:"scope"`
	Revoked        bool   `json:"revoked"`
}

type Graph struct {
	Nodes                        []Node               `json:"nodes"`
	Edges                        []Edge               `json:"edges"`
	RequiredPaths                 [][]string           `json:"required_paths,omitempty"`
	Claims                        map[string]ClaimState `json:"claims"`
	SourceUniverseCompleteness   ClaimState           `json:"source_universe_completeness"`
	ProgressUniverseCompleteness ClaimState           `json:"progress_universe_completeness"`
}

type Metric struct {
	Numerator   int      `json:"numerator"`
	Denominator int      `json:"denominator"`
	Value       *float64 `json:"value"`
}

type Score struct {
	QueryID                 string            `json:"query_id"`
	NodePrecision           Metric            `json:"node_precision"`
	NodeRecall              Metric            `json:"node_recall"`
	EdgePrecision           Metric            `json:"edge_precision"`
	EdgeRecall              Metric            `json:"edge_recall"`
	SourceAttributionPrec   Metric            `json:"source_attribution_precision"`
	SourceAttributionRecall Metric            `json:"source_attribution_recall"`
	TemporalValidity        Metric            `json:"temporal_validity"`
	ChainConsistency        Metric            `json:"chain_consistency"`
	FalseSupportCount       int               `json:"false_support_count"`
	ClaimCeilingPromotions  []string          `json:"claim_ceiling_promotions"`
	FalseCompleteness       bool              `json:"false_completeness"`
	GoldNodeIDs             []string          `json:"gold_node_ids"`
	CandidateNodeIDs        []string          `json:"candidate_node_ids"`
	GoldEdgeIDs             []string          `json:"gold_edge_ids"`
	CandidateEdgeIDs        []string          `json:"candidate_edge_ids"`
	ReceiptSHA256           string            `json:"receipt_sha256"`
}

func metric(numerator, denominator int) Metric {
	m := Metric{Numerator: numerator, Denominator: denominator}
	if denominator > 0 {
		v := float64(numerator) / float64(denominator)
		m.Value = &v
	}
	return m
}

func validDigest(s string) bool {
	if len(s) != 64 {
		return false
	}
	b, err := hex.DecodeString(s)
	return err == nil && len(b) == sha256.Size && s == stringLower(s)
}

func stringLower(s string) string {
	out := make([]byte, len(s))
	for i := range s {
		b := s[i]
		if b >= 'A' && b <= 'Z' {
			b += 'a' - 'A'
		}
		out[i] = b
	}
	return string(out)
}

func validateGraph(g Graph, label string) error {
	nodes := make(map[string]bool, len(g.Nodes))
	for _, n := range g.Nodes {
		if n.ID == "" || n.Type == "" || nodes[n.ID] {
			return errors.New(label + ": empty/duplicate node identity or missing type")
		}
		nodes[n.ID] = true
	}
	edges := make(map[string]bool, len(g.Edges))
	for _, e := range g.Edges {
		if e.ID == "" || e.From == "" || e.Predicate == "" || e.To == "" || e.SourceClaimID == "" ||
			!validDigest(e.SourceDigest) || e.StreamID == "" || e.AcceptedOffset == 0 || e.Scope == "" || edges[e.ID] {
			return errors.New(label + ": malformed/duplicate edge or source reference")
		}
		if !nodes[e.From] || !nodes[e.To] {
			return errors.New(label + ": edge endpoint missing from graph")
		}
		if _, err := time.Parse(time.RFC3339, e.ValidFrom); err != nil {
			return errors.New(label + ": invalid valid_from")
		}
		if e.ValidTo != "" {
			to, err := time.Parse(time.RFC3339, e.ValidTo)
			if err != nil {
				return errors.New(label + ": invalid valid_to")
			}
			from, _ := time.Parse(time.RFC3339, e.ValidFrom)
			if !to.After(from) {
				return errors.New(label + ": invalid half-open validity interval")
			}
		}
		edges[e.ID] = true
	}
	for _, path := range g.RequiredPaths {
		if len(path) == 0 {
			return errors.New(label + ": empty required path")
		}
		for _, edgeID := range path {
			if !edges[edgeID] {
				return errors.New(label + ": required path references unknown edge")
			}
		}
	}
	return nil
}

func exactRelation(a, b Edge) bool {
	return a.ID == b.ID && a.From == b.From && a.Predicate == b.Predicate && a.To == b.To
}

func sourceAttribution(a, b Edge) bool {
	return a.SourceClaimID == b.SourceClaimID && a.SourceDigest == b.SourceDigest
}

func temporalAndAdmissible(q Query, e Edge) bool {
	queryTime, err := time.Parse(time.RFC3339, q.ValidAt)
	if err != nil {
		return false
	}
	from, err := time.Parse(time.RFC3339, e.ValidFrom)
	if err != nil || queryTime.Before(from) {
		return false
	}
	if e.ValidTo != "" {
		to, err := time.Parse(time.RFC3339, e.ValidTo)
		if err != nil || !queryTime.Before(to) {
			return false
		}
	}
	frontier, ok := q.AcceptedFrontier[e.StreamID]
	return ok && e.AcceptedOffset <= frontier && e.Scope == q.Scope && !e.Revoked
}

func Evaluate(q Query, gold, candidate Graph) (Score, error) {
	if q.ID == "" || q.Scope == "" {
		return Score{}, errors.New("query identity and scope are required")
	}
	if _, err := time.Parse(time.RFC3339, q.ValidAt); err != nil {
		return Score{}, errors.New("valid_at must be RFC3339")
	}
	if len(q.AcceptedFrontier) == 0 {
		return Score{}, errors.New("accepted frontier is required")
	}
	if err := validateGraph(gold, "gold"); err != nil {
		return Score{}, err
	}
	if err := validateGraph(candidate, "candidate"); err != nil {
		return Score{}, err
	}
	goldNodes := map[string]Node{}
	candidateNodes := map[string]Node{}
	for _, n := range gold.Nodes {
		goldNodes[n.ID] = n
	}
	for _, n := range candidate.Nodes {
		candidateNodes[n.ID] = n
	}
	nodeTP := 0
	for id, n := range candidateNodes {
		if expected, ok := goldNodes[id]; ok && expected.Type == n.Type {
			nodeTP++
		}
	}
	goldEdges := map[string]Edge{}
	candidateEdges := map[string]Edge{}
	for _, e := range gold.Edges {
		goldEdges[e.ID] = e
	}
	for _, e := range candidate.Edges {
		candidateEdges[e.ID] = e
	}
	edgeTP, attributionTP, temporalTP, falseSupport := 0, 0, 0, 0
	correctEdge := map[string]bool{}
	for id, e := range candidateEdges {
		expected, exists := goldEdges[id]
		relationOK := exists && exactRelation(e, expected)
		attributionOK := relationOK && sourceAttribution(e, expected)
		timeOK := temporalAndAdmissible(q, e)
		if timeOK {
			temporalTP++
		}
		if relationOK && attributionOK && timeOK {
			edgeTP++
			attributionTP++
			correctEdge[id] = true
		} else {
			falseSupport++
		}
	}
	pathTP := 0
	for _, path := range gold.RequiredPaths {
		ok := true
		for i, id := range path {
			if !correctEdge[id] {
				ok = false
				break
			}
			if i > 0 {
				prev := candidateEdges[path[i-1]]
				next := candidateEdges[id]
				if prev.To != next.From {
					ok = false
					break
				}
			}
		}
		if ok {
			pathTP++
		}
	}
	promotions := make([]string, 0)
	for dimension, state := range candidate.Claims {
		if state == ClaimEstablished && gold.Claims[dimension] != ClaimEstablished {
			promotions = append(promotions, dimension)
		}
	}
	sort.Strings(promotions)
	falseCompleteness := candidate.SourceUniverseCompleteness == ClaimEstablished &&
		(q.SourceUniverseCompleteness != ClaimEstablished || candidate.ProgressUniverseCompleteness == ClaimEstablished ||
			q.ProgressUniverseCompleteness != ClaimEstablished)
	goldNodeIDs, candidateNodeIDs := make([]string, 0, len(goldNodes)), make([]string, 0, len(candidateNodes))
	goldEdgeIDs, candidateEdgeIDs := make([]string, 0, len(goldEdges)), make([]string, 0, len(candidateEdges))
	for id := range goldNodes {
		goldNodeIDs = append(goldNodeIDs, id)
	}
	for id := range candidateNodes {
		candidateNodeIDs = append(candidateNodeIDs, id)
	}
	for id := range goldEdges {
		goldEdgeIDs = append(goldEdgeIDs, id)
	}
	for id := range candidateEdges {
		candidateEdgeIDs = append(candidateEdgeIDs, id)
	}
	sort.Strings(goldNodeIDs)
	sort.Strings(candidateNodeIDs)
	sort.Strings(goldEdgeIDs)
	sort.Strings(candidateEdgeIDs)
	score := Score{
		QueryID: q.ID,
		NodePrecision: metric(nodeTP, len(candidateNodes)),
		NodeRecall: metric(nodeTP, len(goldNodes)),
		EdgePrecision: metric(edgeTP, len(candidateEdges)),
		EdgeRecall: metric(edgeTP, len(goldEdges)),
		SourceAttributionPrec: metric(attributionTP, len(candidateEdges)),
		SourceAttributionRecall: metric(attributionTP, len(goldEdges)),
		TemporalValidity: metric(temporalTP, len(candidateEdges)),
		ChainConsistency: metric(pathTP, len(gold.RequiredPaths)),
		FalseSupportCount: falseSupport,
		ClaimCeilingPromotions: promotions,
		FalseCompleteness: falseCompleteness,
		GoldNodeIDs: goldNodeIDs,
		CandidateNodeIDs: candidateNodeIDs,
		GoldEdgeIDs: goldEdgeIDs,
		CandidateEdgeIDs: candidateEdgeIDs,
	}
	canonical, err := json.Marshal(score)
	if err != nil {
		return Score{}, err
	}
	digest := sha256.Sum256(append([]byte("MYCELIX-CRAFT-EVIDENCE-SUBGRAPH-SCORE-V0.1\x00"), canonical...))
	score.ReceiptSHA256 = hex.EncodeToString(digest[:])
	return score, nil
}

func IsPerfect(v Metric) bool { return v.Value != nil && !math.IsNaN(*v.Value) && *v.Value == 1 }
