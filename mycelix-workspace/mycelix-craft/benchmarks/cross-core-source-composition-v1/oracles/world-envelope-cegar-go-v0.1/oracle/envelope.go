// Package oracle checks finite-domain uncertainty abstractions and performs
// coverage-preserving, constraint-guided refinement. It is a research oracle.
package oracle

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"reflect"
	"sort"
)

const (
	SchemaID        = "mycelix.craft.world-envelope-cegar.v0.1"
	MaxAssignments  = 100000
)

var frozenDomainDigests = map[string]string{
	"CWE-001-spurious-decision-flip": "651c083107e4502e9627beef94f45b4b26d4cd5ab90c614ad9df20c434c649c7",
	"CWE-002-genuine-decision-flip": "9c49b79ec74079b4b10cbb5e86dbb219de9f18ae3cb302ab517d73c580a83fb4",
}

type ScopeBasis string

const (
	ScopeFixtureExact       ScopeBasis = "fixture_exact"
	ScopeDeclaredAssumptions ScopeBasis = "declared_assumptions"
	ScopeNotEvaluated       ScopeBasis = "not_evaluated"
)

type Dimension struct {
	ID     string   "json:\"id\""
	Values []string "json:\"values\""
}

type ForbiddenConstraint struct {
	ID           string            "json:\"id\""
	Forbidden    map[string]string "json:\"forbidden\""
	EvidenceRefs []string          "json:\"evidence_refs\""
}

type WorldEvaluation struct {
	Assignment     map[string]string "json:\"assignment\""
	Losses         map[string]int64  "json:\"losses\""
	HardViolations []string          "json:\"hard_violations,omitempty\""
}

type Cell struct {
	ID       string              "json:\"id\""
	Fixed    map[string]string   "json:\"fixed,omitempty\""
	Excluded []map[string]string "json:\"excluded,omitempty\""
}

type Problem struct {
	SchemaID        string               "json:\"schema_id\""
	CaseID          string               "json:\"case_id\""
	ScopeBasis      ScopeBasis            "json:\"scope_basis\""
	FixtureDigest   string               "json:\"fixture_digest,omitempty\""
	Dimensions      []Dimension           "json:\"dimensions\""
	Constraints    []ForbiddenConstraint "json:\"constraints\""
	Actions         []string              "json:\"actions\""
	Worlds          []WorldEvaluation     "json:\"worlds\""
	Cells           []Cell                "json:\"cells\""
	UnresolvedItems []string              "json:\"unresolved_items,omitempty\""
}

type DecisionSummary struct {
	WorldCount              int      "json:\"world_count\""
	RobustSafeActions       []string "json:\"robust_safe_actions\""
	OptimalInEveryWorld     []string "json:\"optimal_in_every_world\""
	MinimaxLossActions      []string "json:\"minimax_loss_actions\""
	MinimaxRegretActions    []string "json:\"minimax_regret_actions\""
}

type SpuriousDecisionFlipWitness struct {
	SpuriousAssignment      map[string]string "json:\"spurious_assignment\""
	SpuriousOptimalActions  []string          "json:\"spurious_optimal_actions\""
	ConcreteAssignment      map[string]string "json:\"concrete_assignment\""
	ConcreteOptimalActions  []string          "json:\"concrete_optimal_actions\""
	DifferingDimensions     []string          "json:\"differing_dimensions\""
	ViolatedConstraintIDs   []string          "json:\"violated_constraint_ids\""
}

type Result struct {
	CaseID                     string                       "json:\"case_id\""
	ScopeBasis                 ScopeBasis                    "json:\"scope_basis\""
	ScopeStatus                string                       "json:\"scope_status\""
	DomainAssignmentCount      int                          "json:\"domain_assignment_count\""
	ConcreteWorldCount         int                          "json:\"concrete_world_count\""
	AbstractWorldCount         int                          "json:\"abstract_world_count\""
	SpuriousWorldCount         int                          "json:\"spurious_world_count\""
	AbstractCoverageSound      bool                         "json:\"abstract_coverage_sound\""
	ConcreteSummary            DecisionSummary              "json:\"concrete_summary\""
	AbstractSummary            DecisionSummary              "json:\"abstract_summary\""
	SpuriousDecisionFlip       *SpuriousDecisionFlipWitness "json:\"spurious_decision_flip,omitempty\""
	OmittedConcreteAssignments []map[string]string          "json:\"omitted_concrete_assignments,omitempty\""
}

type RefinementReceipt struct {
	CellID                    string   "json:\"cell_id\""
	ConstraintID              string   "json:\"constraint_id\""
	DomainWorldDigest         string   "json:\"domain_world_digest\""
	BeforeStateDigest         string   "json:\"before_state_digest\""
	AfterStateDigest          string   "json:\"after_state_digest\""
	ConcreteWorldCount        int      "json:\"concrete_world_count\""
	AbstractWorldsBefore      int      "json:\"abstract_worlds_before\""
	AbstractWorldsAfter       int      "json:\"abstract_worlds_after\""
	SpuriousWorldsBefore      int      "json:\"spurious_worlds_before\""
	SpuriousWorldsAfter       int      "json:\"spurious_worlds_after\""
	ConcreteSemanticsPreserved bool    "json:\"concrete_semantics_preserved\""
	DecisionAmbiguityReduced  bool     "json:\"decision_ambiguity_reduced\""
	AbstractOptimaBefore      []string "json:\"abstract_optima_before\""
	AbstractOptimaAfter       []string "json:\"abstract_optima_after\""
}

type domainBinding struct {
	SchemaID        string               "json:\"schema_id\""
	CaseID          string               "json:\"case_id\""
	ScopeBasis      ScopeBasis            "json:\"scope_basis\""
	Dimensions      []Dimension           "json:\"dimensions\""
	Constraints    []ForbiddenConstraint "json:\"constraints\""
	Actions         []string              "json:\"actions\""
	Worlds          []WorldEvaluation     "json:\"worlds\""
	UnresolvedItems []string              "json:\"unresolved_items,omitempty\""
}

func DecodeProblem(raw []byte) (Problem, error) {
	if err := rejectDuplicateKeys(raw); err != nil {
		return Problem{}, err
	}
	var p Problem
	dec := json.NewDecoder(bytes.NewReader(raw))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&p); err != nil {
		return Problem{}, err
	}
	if err := ensureNoTrailingJSON(dec); err != nil {
		return Problem{}, err
	}
	if err := Validate(p); err != nil {
		return Problem{}, err
	}
	return p, nil
}

func Validate(p Problem) error {
	if p.SchemaID != SchemaID {
		return fmt.Errorf("schema_id must equal %q", SchemaID)
	}
	if p.CaseID == "" {
		return errors.New("case_id is required")
	}
	switch p.ScopeBasis {
	case ScopeFixtureExact:
		expected, ok := frozenDomainDigests[p.CaseID]
		if !ok {
			return fmt.Errorf("fixture %q is not in the frozen domain allowlist", p.CaseID)
		}
		actual, err := DomainWorldDigest(p)
		if err != nil {
			return err
		}
		if p.FixtureDigest == "" || p.FixtureDigest != expected || actual != expected {
			return errors.New("fixture digest does not match the frozen domain/world registry")
		}
	case ScopeDeclaredAssumptions, ScopeNotEvaluated:
		if p.FixtureDigest != "" {
			return errors.New("fixture_digest is only valid for a pinned exact fixture")
		}
	default:
		return fmt.Errorf("unsupported scope_basis %q", p.ScopeBasis)
	}
	if len(p.UnresolvedItems) > 0 && p.ScopeBasis != ScopeNotEvaluated {
		return errors.New("unresolved_items require scope_basis=not_evaluated")
	}
	if len(p.Dimensions) == 0 {
		return errors.New("at least one finite dimension is required")
	}
	if len(p.Actions) == 0 {
		return errors.New("at least one action is required")
	}
	dimensions := make(map[string]map[string]struct{}, len(p.Dimensions))
	productSize := 1
	for _, dimension := range p.Dimensions {
		if dimension.ID == "" {
			return errors.New("dimension IDs must be non-empty")
		}
		if _, exists := dimensions[dimension.ID]; exists {
			return fmt.Errorf("duplicate dimension ID %q", dimension.ID)
		}
		if len(dimension.Values) == 0 {
			return fmt.Errorf("dimension %q has an empty value domain", dimension.ID)
		}
		values := make(map[string]struct{}, len(dimension.Values))
		for _, value := range dimension.Values {
			if value == "" {
				return fmt.Errorf("dimension %q contains an empty value", dimension.ID)
			}
			if _, exists := values[value]; exists {
				return fmt.Errorf("dimension %q has duplicate value %q", dimension.ID, value)
			}
			values[value] = struct{}{}
		}
		if productSize > MaxAssignments/len(values) {
			return fmt.Errorf("finite domain exceeds maximum assignment count %d", MaxAssignments)
		}
		productSize *= len(values)
		dimensions[dimension.ID] = values
	}

	actions := make(map[string]struct{}, len(p.Actions))
	for _, action := range p.Actions {
		if action == "" {
			return errors.New("action IDs must be non-empty")
		}
		if _, exists := actions[action]; exists {
			return fmt.Errorf("duplicate action ID %q", action)
		}
		actions[action] = struct{}{}
	}

	constraintIDs := make(map[string]struct{}, len(p.Constraints))
	for _, constraint := range p.Constraints {
		if constraint.ID == "" {
			return errors.New("constraint IDs must be non-empty")
		}
		if _, exists := constraintIDs[constraint.ID]; exists {
			return fmt.Errorf("duplicate constraint ID %q", constraint.ID)
		}
		constraintIDs[constraint.ID] = struct{}{}
		if len(constraint.Forbidden) == 0 {
			return fmt.Errorf("constraint %q has an empty forbidden pattern", constraint.ID)
		}
		if len(constraint.EvidenceRefs) == 0 {
			return fmt.Errorf("constraint %q lacks evidence/assumption references", constraint.ID)
		}
		if err := validatePattern(constraint.Forbidden, dimensions); err != nil {
			return fmt.Errorf("constraint %q: %w", constraint.ID, err)
		}
		for _, ref := range constraint.EvidenceRefs {
			if ref == "" {
				return fmt.Errorf("constraint %q has an empty evidence reference", constraint.ID)
			}
		}
	}

	if len(p.Worlds) != productSize {
		return fmt.Errorf("world evaluation count %d does not equal full domain product %d", len(p.Worlds), productSize)
	}
	worlds := make(map[string]WorldEvaluation, len(p.Worlds))
	for _, world := range p.Worlds {
		if err := validateCompleteAssignment(world.Assignment, dimensions); err != nil {
			return fmt.Errorf("world assignment: %w", err)
		}
		key, err := AssignmentKey(world.Assignment)
		if err != nil {
			return err
		}
		if _, exists := worlds[key]; exists {
			return fmt.Errorf("duplicate world evaluation for assignment %s", key)
		}
		if len(world.Losses) != len(actions) {
			return fmt.Errorf("world %s must declare exactly one loss per action", key)
		}
		for action := range actions {
			loss, ok := world.Losses[action]
			if !ok {
				return fmt.Errorf("world %s is missing loss for action %q", key, action)
			}
			if loss < 0 {
				return fmt.Errorf("world %s has negative loss for action %q", key, action)
			}
		}
		for action := range world.Losses {
			if _, ok := actions[action]; !ok {
				return fmt.Errorf("world %s has unknown action %q", key, action)
			}
		}
		seen := make(map[string]struct{}, len(world.HardViolations))
		for _, action := range world.HardViolations {
			if _, ok := actions[action]; !ok {
				return fmt.Errorf("world %s has an unknown hard-violation action %q", key, action)
			}
			if _, exists := seen[action]; exists {
				return fmt.Errorf("world %s repeats hard violation %q", key, action)
			}
			seen[action] = struct{}{}
		}
		worlds[key] = world
	}

	assignments, err := EnumerateAssignments(p.Dimensions)
	if err != nil {
		return err
	}
	for _, assignment := range assignments {
		key, _ := AssignmentKey(assignment)
		if _, ok := worlds[key]; !ok {
			return fmt.Errorf("missing world evaluation for full-domain assignment %s", key)
		}
	}

	if len(p.Cells) == 0 {
		return errors.New("at least one abstract cell is required")
	}
	cellIDs := make(map[string]struct{}, len(p.Cells))
	for _, cell := range p.Cells {
		if cell.ID == "" {
			return errors.New("abstract cell IDs must be non-empty")
		}
		if _, exists := cellIDs[cell.ID]; exists {
			return fmt.Errorf("duplicate abstract cell ID %q", cell.ID)
		}
		cellIDs[cell.ID] = struct{}{}
		if err := validatePattern(cell.Fixed, dimensions); err != nil {
			return fmt.Errorf("cell %q fixed pattern: %w", cell.ID, err)
		}
		for i, excluded := range cell.Excluded {
			if len(excluded) == 0 {
				return fmt.Errorf("cell %q exclusion %d is empty", cell.ID, i)
			}
			if err := validatePattern(excluded, dimensions); err != nil {
				return fmt.Errorf("cell %q exclusion %d: %w", cell.ID, i, err)
			}
		}
	}
	for _, unresolved := range p.UnresolvedItems {
		if unresolved == "" {
			return errors.New("unresolved item IDs must be non-empty")
		}
	}
	return nil
}

func Evaluate(p Problem) (Result, error) {
	result := Result{CaseID: p.CaseID, ScopeBasis: p.ScopeBasis, ScopeStatus: scopeStatus(p.ScopeBasis)}
	if err := Validate(p); err != nil {
		return result, err
	}
	assignments, err := EnumerateAssignments(p.Dimensions)
	if err != nil {
		return result, err
	}
	worldsByKey := make(map[string]WorldEvaluation, len(p.Worlds))
	for _, world := range p.Worlds {
		key, _ := AssignmentKey(world.Assignment)
		worldsByKey[key] = world
	}
	constraintsByID := make(map[string]ForbiddenConstraint, len(p.Constraints))
	for _, constraint := range p.Constraints {
		constraintsByID[constraint.ID] = constraint
	}

	concrete := make([]WorldEvaluation, 0, len(assignments))
	abstract := make([]WorldEvaluation, 0, len(assignments))
	spurious := make([]WorldEvaluation, 0)
	omitted := make([]map[string]string, 0)
	for _, assignment := range assignments {
		world := worldsByKey[mustAssignmentKey(assignment)]
		possible := true
		for _, constraint := range p.Constraints {
			if Matches(assignment, constraint.Forbidden) {
				possible = false
				break
			}
		}
		covered := false
		for _, cell := range p.Cells {
			if CellContains(cell, assignment) {
				covered = true
				break
			}
		}
		if possible {
			concrete = append(concrete, world)
			if !covered {
				omitted = append(omitted, cloneMap(assignment))
			}
		}
		if covered {
			abstract = append(abstract, world)
			if !possible {
				spurious = append(spurious, world)
			}
		}
	}
	result.DomainAssignmentCount = len(assignments)
	result.ConcreteWorldCount = len(concrete)
	result.AbstractWorldCount = len(abstract)
	result.SpuriousWorldCount = len(spurious)
	if len(concrete) == 0 {
		return result, errors.New("declared constraints leave no concrete compatible worlds")
	}
	if len(omitted) > 0 {
		result.AbstractCoverageSound = false
		result.OmittedConcreteAssignments = omitted
		return result, fmt.Errorf("abstract cells under-approximate the declared concrete world set: %d assignments omitted", len(omitted))
	}
	result.AbstractCoverageSound = true
	result.ConcreteSummary = Summarize(p.Actions, concrete)
	result.AbstractSummary = Summarize(p.Actions, abstract)
	result.SpuriousDecisionFlip = findSpuriousDecisionFlip(spurious, concrete, p.Constraints)
	_ = constraintsByID
	return result, nil
}

func Refine(p Problem, cellID, constraintID string) (Problem, RefinementReceipt, error) {
	var receipt RefinementReceipt
	before, err := Evaluate(p)
	if err != nil {
		return p, receipt, fmt.Errorf("pre-refinement envelope is not sound: %w", err)
	}
	constraintIndex := -1
	for i, constraint := range p.Constraints {
		if constraint.ID == constraintID {
			constraintIndex = i
			break
		}
	}
	if constraintIndex < 0 {
		return p, receipt, fmt.Errorf("constraint %q not found", constraintID)
	}
	cellIndex := -1
	for i, cell := range p.Cells {
		if cell.ID == cellID {
			cellIndex = i
			break
		}
	}
	if cellIndex < 0 {
		return p, receipt, fmt.Errorf("cell %q not found", cellID)
	}
	constraint := p.Constraints[constraintIndex]
	updated := CloneProblem(p)
	for _, existing := range updated.Cells[cellIndex].Excluded {
		if reflect.DeepEqual(existing, constraint.Forbidden) {
			return p, receipt, errors.New("constraint already applied to this abstract cell")
		}
	}
	hasCandidate := false
	allAssignments, err := EnumerateAssignments(p.Dimensions)
	if err != nil {
		return p, receipt, err
	}
	for _, assignment := range allAssignments {
		if CellContains(updated.Cells[cellIndex], assignment) && Matches(assignment, constraint.Forbidden) {
			hasCandidate = true
			break
		}
	}
	if !hasCandidate {
		return p, receipt, errors.New("constraint does not narrow the selected abstract cell")
	}
	updated.Cells[cellIndex].Excluded = append(updated.Cells[cellIndex].Excluded, cloneMap(constraint.Forbidden))
	after, err := Evaluate(updated)
	if err != nil {
		return p, receipt, fmt.Errorf("refinement would violate concrete coverage: %w", err)
	}
	if before.ConcreteWorldCount != after.ConcreteWorldCount ||
		!reflect.DeepEqual(before.ConcreteSummary, after.ConcreteSummary) {
		return p, receipt, errors.New("refinement changed the concrete world set or its decision semantics")
	}
	beforeDigest, err := StateDigest(p)
	if err != nil {
		return p, receipt, err
	}
	afterDigest, err := StateDigest(updated)
	if err != nil {
		return p, receipt, err
	}
	receipt = RefinementReceipt{
		CellID: cellID,
		ConstraintID: constraintID,
		DomainWorldDigest: mustDomainWorldDigest(p),
		BeforeStateDigest: beforeDigest,
		AfterStateDigest: afterDigest,
		ConcreteWorldCount: after.ConcreteWorldCount,
		AbstractWorldsBefore: before.AbstractWorldCount,
		AbstractWorldsAfter: after.AbstractWorldCount,
		SpuriousWorldsBefore: before.SpuriousWorldCount,
		SpuriousWorldsAfter: after.SpuriousWorldCount,
		ConcreteSemanticsPreserved: true,
		DecisionAmbiguityReduced: len(after.AbstractSummary.OptimalInEveryWorld) > len(before.AbstractSummary.OptimalInEveryWorld) ||
			after.SpuriousWorldCount < before.SpuriousWorldCount,
		AbstractOptimaBefore: append([]string(nil), before.AbstractSummary.OptimalInEveryWorld...),
		AbstractOptimaAfter: append([]string(nil), after.AbstractSummary.OptimalInEveryWorld...),
	}
	return updated, receipt, nil
}

func Summarize(actions []string, worlds []WorldEvaluation) DecisionSummary {
	result := DecisionSummary{WorldCount: len(worlds)}
	if len(worlds) == 0 {
		result.RobustSafeActions = []string{}
		result.OptimalInEveryWorld = []string{}
		result.MinimaxLossActions = []string{}
		result.MinimaxRegretActions = []string{}
		return result
	}
	ordered := append([]string(nil), actions...)
	sort.Strings(ordered)
	robustSafe := make(map[string]struct{}, len(ordered))
	identified := make(map[string]struct{}, len(ordered))
	for _, action := range ordered {
		robustSafe[action] = struct{}{}
		identified[action] = struct{}{}
	}
	bestLoss := make([]int64, len(worlds))
	worldOptima := make([]map[string]struct{}, len(worlds))
	for i, world := range worlds {
		minimum := int64(int64(^uint64(0) >> 1))
		safe := make([]string, 0, len(ordered))
		for _, action := range ordered {
			if contains(world.HardViolations, action) {
				delete(robustSafe, action)
				continue
			}
			safe = append(safe, action)
			if world.Losses[action] < minimum {
				minimum = world.Losses[action]
			}
		}
		if len(safe) == 0 {
			for action := range identified {
				delete(identified, action)
			}
			worldOptima[i] = map[string]struct{}{}
			bestLoss[i] = int64(^uint64(0) >> 1)
			continue
		}
		bestLoss[i] = minimum
		optimum := make(map[string]struct{})
		for _, action := range safe {
			if world.Losses[action] == minimum {
				optimum[action] = struct{}{}
			}
		}
		worldOptima[i] = optimum
		for action := range identified {
			if _, ok := optimum[action]; !ok {
				delete(identified, action)
			}
		}
	}
	result.RobustSafeActions = sortedKeys(robustSafe)
	result.OptimalInEveryWorld = sortedKeys(identified)
	worstLoss := make(map[string]int64)
	worstRegret := make(map[string]int64)
	for action := range robustSafe {
		maxLoss := int64(0)
		maxRegret := int64(0)
		for i, world := range worlds {
			loss := world.Losses[action]
			if loss > maxLoss {
				maxLoss = loss
			}
			if bestLoss[i] != int64(^uint64(0) >> 1) {
				regret := loss - bestLoss[i]
				if regret > maxRegret {
					maxRegret = regret
				}
			}
		}
		worstLoss[action] = maxLoss
		worstRegret[action] = maxRegret
	}
	result.MinimaxLossActions = minimize(robustSafe, worstLoss)
	result.MinimaxRegretActions = minimize(robustSafe, worstRegret)
	return result
}

func findSpuriousDecisionFlip(spurious, concrete []WorldEvaluation, constraints []ForbiddenConstraint) *SpuriousDecisionFlipWitness {
	var best *SpuriousDecisionFlipWitness
	for _, abstractWorld := range spurious {
		abstractOptimum := SummarizeWorld(abstractWorld)
		for _, realWorld := range concrete {
			realOptimum := SummarizeWorld(realWorld)
			if intersects(abstractOptimum, realOptimum) {
				continue
			}
			dimensions := differingDimensions(abstractWorld.Assignment, realWorld.Assignment)
			violated := matchingConstraintIDs(abstractWorld.Assignment, constraints)
			candidate := &SpuriousDecisionFlipWitness{
				SpuriousAssignment: cloneMap(abstractWorld.Assignment),
				SpuriousOptimalActions: abstractOptimum,
				ConcreteAssignment: cloneMap(realWorld.Assignment),
				ConcreteOptimalActions: realOptimum,
				DifferingDimensions: dimensions,
				ViolatedConstraintIDs: violated,
			}
			if best == nil ||
				len(candidate.DifferingDimensions) < len(best.DifferingDimensions) ||
				(len(candidate.DifferingDimensions) == len(best.DifferingDimensions) &&
					mustAssignmentKey(candidate.SpuriousAssignment) < mustAssignmentKey(best.SpuriousAssignment)) {
				best = candidate
			}
		}
	}
	return best
}

func SummarizeWorld(world WorldEvaluation) []string {
	best := int64(int64(^uint64(0) >> 1))
	for action, loss := range world.Losses {
		if contains(world.HardViolations, action) {
			continue
		}
		if loss < best {
			best = loss
		}
	}
	if best == int64(^uint64(0) >> 1) {
		return []string{}
	}
	result := make([]string, 0)
	for action, loss := range world.Losses {
		if !contains(world.HardViolations, action) && loss == best {
			result = append(result, action)
		}
	}
	sort.Strings(result)
	return result
}

func EnumerateAssignments(dimensions []Dimension) ([]map[string]string, error) {
	ordered := append([]Dimension(nil), dimensions...)
	sort.Slice(ordered, func(i, j int) bool { return ordered[i].ID < ordered[j].ID })
	result := make([]map[string]string, 0)
	current := make(map[string]string, len(ordered))
	var walk func(int)
	walk = func(index int) {
		if index == len(ordered) {
			result = append(result, cloneMap(current))
			return
		}
		values := append([]string(nil), ordered[index].Values...)
		sort.Strings(values)
		for _, value := range values {
			current[ordered[index].ID] = value
			walk(index + 1)
		}
		delete(current, ordered[index].ID)
	}
	walk(0)
	if len(result) > MaxAssignments {
		return nil, fmt.Errorf("assignment count %d exceeds maximum %d", len(result), MaxAssignments)
	}
	return result, nil
}

func CellContains(cell Cell, assignment map[string]string) bool {
	if !Matches(assignment, cell.Fixed) {
		return false
	}
	for _, exclusion := range cell.Excluded {
		if Matches(assignment, exclusion) {
			return false
		}
	}
	return true
}

func Matches(assignment, pattern map[string]string) bool {
	for dimension, value := range pattern {
		if assignment[dimension] != value {
			return false
		}
	}
	return true
}

func DomainWorldDigest(p Problem) (string, error) {
	binding := domainBinding{
		SchemaID: p.SchemaID,
		CaseID: p.CaseID,
		ScopeBasis: p.ScopeBasis,
		Dimensions: p.Dimensions,
		Constraints: p.Constraints,
		Actions: p.Actions,
		Worlds: p.Worlds,
		UnresolvedItems: p.UnresolvedItems,
	}
	encoded, err := json.Marshal(binding)
	if err != nil {
		return "", err
	}
	sum := sha256.Sum256(encoded)
	return hex.EncodeToString(sum[:]), nil
}

func StateDigest(p Problem) (string, error) {
	p.FixtureDigest = ""
	encoded, err := json.Marshal(p)
	if err != nil {
		return "", err
	}
	sum := sha256.Sum256(encoded)
	return hex.EncodeToString(sum[:]), nil
}

func validatePattern(pattern map[string]string, dimensions map[string]map[string]struct{}) error {
	for dimension, value := range pattern {
		values, ok := dimensions[dimension]
		if !ok {
			return fmt.Errorf("unknown dimension %q", dimension)
		}
		if _, ok := values[value]; !ok {
			return fmt.Errorf("unknown value %q for dimension %q", value, dimension)
		}
	}
	return nil
}

func validateCompleteAssignment(assignment map[string]string, dimensions map[string]map[string]struct{}) error {
	if len(assignment) != len(dimensions) {
		return errors.New("assignment must contain every declared dimension exactly once")
	}
	return validatePattern(assignment, dimensions)
}

func AssignmentKey(assignment map[string]string) (string, error) {
	encoded, err := json.Marshal(assignment)
	if err != nil {
		return "", err
	}
	return string(encoded), nil
}

func mustAssignmentKey(assignment map[string]string) string {
	key, _ := AssignmentKey(assignment)
	return key
}

func matchingConstraintIDs(assignment map[string]string, constraints []ForbiddenConstraint) []string {
	result := make([]string, 0)
	for _, constraint := range constraints {
		if Matches(assignment, constraint.Forbidden) {
			result = append(result, constraint.ID)
		}
	}
	sort.Strings(result)
	return result
}

func differingDimensions(a, b map[string]string) []string {
	ids := make([]string, 0)
	for dimension, value := range a {
		if b[dimension] != value {
			ids = append(ids, dimension)
		}
	}
	sort.Strings(ids)
	return ids
}

func sortedKeys(set map[string]struct{}) []string {
	result := make([]string, 0, len(set))
	for key := range set {
		result = append(result, key)
	}
	sort.Strings(result)
	return result
}

func minimize(candidates map[string]struct{}, metric map[string]int64) []string {
	if len(candidates) == 0 {
		return []string{}
	}
	best := int64(int64(^uint64(0) >> 1))
	for action := range candidates {
		if metric[action] < best {
			best = metric[action]
		}
	}
	result := make([]string, 0)
	for action := range candidates {
		if metric[action] == best {
			result = append(result, action)
		}
	}
	sort.Strings(result)
	return result
}

func contains(items []string, target string) bool {
	for _, item := range items {
		if item == target {
			return true
		}
	}
	return false
}

func intersects(a, b []string) bool {
	for _, item := range a {
		if contains(b, item) {
			return true
		}
	}
	return false
}

func differingDimensionsCount(a, b map[string]string) int {
	return len(differingDimensions(a, b))
}

func cloneMap(input map[string]string) map[string]string {
	if input == nil {
		return map[string]string{}
	}
	result := make(map[string]string, len(input))
	for key, value := range input {
		result[key] = value
	}
	return result
}

func CloneProblem(input Problem) Problem {
	result := input
	result.Dimensions = append([]Dimension(nil), input.Dimensions...)
	for i := range result.Dimensions {
		result.Dimensions[i].Values = append([]string(nil), input.Dimensions[i].Values...)
	}
	result.Constraints = append([]ForbiddenConstraint(nil), input.Constraints...)
	for i := range result.Constraints {
		result.Constraints[i].Forbidden = cloneMap(input.Constraints[i].Forbidden)
		result.Constraints[i].EvidenceRefs = append([]string(nil), input.Constraints[i].EvidenceRefs...)
	}
	result.Actions = append([]string(nil), input.Actions...)
	result.Worlds = append([]WorldEvaluation(nil), input.Worlds...)
	for i := range result.Worlds {
		result.Worlds[i].Assignment = cloneMap(input.Worlds[i].Assignment)
		result.Worlds[i].Losses = make(map[string]int64, len(input.Worlds[i].Losses))
		for action, loss := range input.Worlds[i].Losses {
			result.Worlds[i].Losses[action] = loss
		}
		result.Worlds[i].HardViolations = append([]string(nil), input.Worlds[i].HardViolations...)
	}
	result.Cells = append([]Cell(nil), input.Cells...)
	for i := range result.Cells {
		result.Cells[i].Fixed = cloneMap(input.Cells[i].Fixed)
		result.Cells[i].Excluded = append([]map[string]string(nil), input.Cells[i].Excluded...)
		for j := range result.Cells[i].Excluded {
			result.Cells[i].Excluded[j] = cloneMap(input.Cells[i].Excluded[j])
		}
	}
	result.UnresolvedItems = append([]string(nil), input.UnresolvedItems...)
	return result
}

func scopeStatus(basis ScopeBasis) string {
	switch basis {
	case ScopeFixtureExact:
		return "FROZEN_SYNTHETIC_FIXTURE_ONLY"
	case ScopeDeclaredAssumptions:
		return "CONDITIONAL_ON_DECLARED_DOMAIN_AND_CONSTRAINTS"
	default:
		return "NOT_EVALUATED"
	}
}

func mustDomainWorldDigest(p Problem) string {
	digest, _ := DomainWorldDigest(p)
	return digest
}

// JSON helpers reject duplicate object keys and trailing values before typed decoding.
func rejectDuplicateKeys(raw []byte) error {
	dec := json.NewDecoder(bytes.NewReader(raw))
	if err := scanJSONValue(dec); err != nil {
		return err
	}
	return ensureNoTrailingJSON(dec)
}

func scanJSONValue(dec *json.Decoder) error {
	token, err := dec.Token()
	if err != nil {
		return err
	}
	delimiter, ok := token.(json.Delim)
	if !ok {
		return nil
	}
	switch delimiter {
	case '{':
		seen := make(map[string]struct{})
		for dec.More() {
			keyToken, err := dec.Token()
			if err != nil {
				return err
			}
			key, ok := keyToken.(string)
			if !ok {
				return errors.New("JSON object key is not a string")
			}
			if _, exists := seen[key]; exists {
				return fmt.Errorf("duplicate JSON object key %q", key)
			}
			seen[key] = struct{}{}
			if err := scanJSONValue(dec); err != nil {
				return err
			}
		}
		closeToken, err := dec.Token()
		if err != nil {
			return err
		}
		if closeToken != json.Delim('}') {
			return errors.New("malformed JSON object")
		}
	case '[':
		for dec.More() {
			if err := scanJSONValue(dec); err != nil {
				return err
			}
		}
		closeToken, err := dec.Token()
		if err != nil {
			return err
		}
		if closeToken != json.Delim(']') {
			return errors.New("malformed JSON array")
		}
	default:
		return errors.New("unexpected JSON delimiter")
	}
	return nil
}

func ensureNoTrailingJSON(dec *json.Decoder) error {
	var extra any
	if err := dec.Decode(&extra); err != io.EOF {
		if err == nil {
			return errors.New("trailing JSON value")
		}
		return fmt.Errorf("trailing JSON or malformed bytes: %w", err)
	}
	return nil
}
