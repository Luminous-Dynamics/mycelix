package oracle

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"sort"
)

type wireRef struct {
	ID       string `json:"id"`
	SchemaID string `json:"schema_id"`
	Digest   string `json:"digest"`
}

type wireInput struct {
	Slot              string   `json:"slot"`
	ProducerStage     string   `json:"producer_stage"`
	ProducerReceiptID string   `json:"producer_receipt_id"`
	ProducerSchemaID  string   `json:"producer_schema_id"`
	ProducerDigest    string   `json:"producer_digest"`
	RequiredClaims    []string `json:"required_claims"`
}

type wireStage struct {
	ID            string           `json:"id"`
	Stage         string           `json:"stage"`
	SchemaID      string           `json:"schema_id"`
	OutputSchema  string           `json:"output_schema_id"`
	ProfileDigest string           `json:"profile_digest"`
	ScopeDigest   string           `json:"scope_digest"`
	SourceCommit  string           `json:"source_commit"`
	BuildDigest   string           `json:"build_digest"`
	OutputDigest  string           `json:"output_digest"`
	Claims        map[string]State `json:"claims"`
	Inputs        []wireInput      `json:"inputs"`
}

type wireExternalBinding struct {
	ConsumerStage string `json:"consumer_stage"`
	Slot          string `json:"slot"`
	ReceiptID     string `json:"receipt_id"`
	SchemaID      string `json:"schema_id"`
	Digest        string `json:"digest"`
}

type wireFrontier struct {
	Kind          string   `json:"kind"`
	Receipt       wireRef  `json:"receipt"`
	Status        string   `json:"status"`
	Prefix        State    `json:"accepted_prefix_completeness"`
}

type wireProgress struct {
	Kind           string   `json:"kind"`
	Receipt        wireRef  `json:"receipt"`
	Status         string   `json:"status"`
	Universe       State    `json:"source_universe_completeness"`
	OwnerDigest    string   `json:"owner_inventory_digest"`
}

type wireOwner struct {
	Receipt         wireRef `json:"receipt"`
	Authentication  State   `json:"authentication"`
	Completeness    State   `json:"completeness"`
	FrontierDigest  string  `json:"frontier_digest"`
}

type wireTemporal struct {
	OCELProfile       wireRef `json:"ocel_profile"`
	SourceCorpus      wireRef `json:"source_corpus"`
	Projection        wireRef `json:"projection_receipt"`
	Replay            wireRef `json:"replay_receipt"`
	FrontierDigest    string  `json:"frontier_digest"`
	SourceCorpusDigest string `json:"source_corpus_digest"`
}

type wireEvidence struct {
	FailureDomainGraph *wireRef `json:"failure_domain_graph"`
	LineageAttestation *wireRef `json:"lineage_attestation"`
	IndependenceStatus State    `json:"independence_status"`
}

type wireAuthority struct {
	Receipt   wireRef `json:"receipt"`
	Principal string  `json:"principal_id"`
	Scope     string  `json:"scope"`
	Validity  State   `json:"validity_status"`
}

type wireActionPlan struct {
	Receipt           wireRef `json:"receipt"`
	Digest            string  `json:"plan_digest"`
	Revision          string  `json:"revision"`
	Count             uint64  `json:"action_count"`
	Covered           uint64  `json:"covered_action_count"`
	CoverageDigest    string  `json:"coverage_plan_digest"`
	CoverageRevision  string  `json:"coverage_revision"`
	Coverage          wireRef `json:"coverage_receipt"`
}

type wireOutcome struct {
	Receipt                 wireRef `json:"receipt"`
	Status                  string  `json:"status"`
	Reconciliation          string  `json:"reconciliation"`
	ExecutionDigest         string  `json:"execution_digest"`
	ActionPlanDigest        string  `json:"plan_digest"`
	MutationScope           string  `json:"mutation_scope"`
	SameScopeMutationAttempt bool    `json:"same_scope_mutation_attempt"`
}

type wireCapsule struct {
	SchemaID      string                `json:"schema_id"`
	ProfileRef    string                `json:"profile_ref"`
	ProfileDigest string                `json:"profile_digest"`
	DecisionID    string                `json:"decision_id"`
	QueryID       string                `json:"query_id"`
	QueryDigest   string                `json:"query_digest"`
	QuerySchema   string                `json:"query_schema_id"`
	ScopeDigest   string                `json:"scope"`
	ValidAt       string                `json:"valid_at"`
	Policy        struct { Receipt wireRef `json:"receipt"` } `json:"policy"`
	Frontier      wireFrontier          `json:"causal_event_frontier"`
	Progress      wireProgress          `json:"progress_antichain"`
	Owner         wireOwner             `json:"owner_universe"`
	Temporal      wireTemporal          `json:"temporal"`
	Evidence      wireEvidence          `json:"evidence"`
	Authority     wireAuthority         `json:"authority"`
	Action        wireActionPlan        `json:"action_plan"`
	Execution     wireRef               `json:"execution"`
	Outcome       wireOutcome           `json:"outcome"`
	External      []wireExternalBinding `json:"external_bindings"`
	Stages        []wireStage           `json:"stages"`
	Claims        map[string]State      `json:"claims"`
	Disposition   Disposition           `json:"disposition"`
	Unknowns      []string              `json:"unknowns"`
	Conflicts     []string              `json:"conflicts"`
	Abstention    string                `json:"abstention_reason,omitempty"`
}

func refFromWire(r wireRef) Ref {
	return Ref{ID: r.ID, Schema: r.SchemaID, Digest: r.Digest}
}

func refToWire(r Ref) wireRef {
	return wireRef{ID: r.ID, SchemaID: r.Schema, Digest: r.Digest}
}

func parseStage(value string) (Stage, error) {
	for _, stage := range stageOrder {
		if string(stage) == value {
			return stage, nil
		}
	}
	return "", fmt.Errorf("unknown DJCC stage %q", value)
}

func rejectDuplicateJSONKeys(data []byte) error {
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.UseNumber()
	var readValue func() error
	readValue = func() error {
		token, err := decoder.Token()
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
			for decoder.More() {
				keyToken, err := decoder.Token()
				if err != nil {
					return err
				}
				key, ok := keyToken.(string)
				if !ok {
					return fmt.Errorf("JSON object key is not a string")
				}
				if _, exists := seen[key]; exists {
					return fmt.Errorf("duplicate JSON object key %q", key)
				}
				seen[key] = struct{}{}
				if err := readValue(); err != nil {
					return err
				}
			}
			end, err := decoder.Token()
			if err != nil {
				return err
			}
			if end != json.Delim('}') {
				return fmt.Errorf("malformed JSON object terminator")
			}
		case '[':
			for decoder.More() {
				if err := readValue(); err != nil {
					return err
				}
			}
			end, err := decoder.Token()
			if err != nil {
				return err
			}
			if end != json.Delim(']') {
				return fmt.Errorf("malformed JSON array terminator")
			}
		default:
			return fmt.Errorf("unexpected JSON delimiter %q", delimiter)
		}
		return nil
	}

	if err := readValue(); err != nil {
		return fmt.Errorf("scan DBC JSON keys: %w", err)
	}
	var trailing any
	if err := decoder.Decode(&trailing); err != io.EOF {
		if err == nil {
			return fmt.Errorf("multiple JSON values are not permitted")
		}
		return fmt.Errorf("trailing JSON data: %w", err)
	}
	return nil
}

func DecodeJSON(data []byte) (Capsule, error) {
	if err := rejectDuplicateJSONKeys(data); err != nil {
		return Capsule{}, err
	}
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	var wire wireCapsule
	if err := decoder.Decode(&wire); err != nil {
		return Capsule{}, fmt.Errorf("decode DBC wire capsule: %w", err)
	}
	if wire.External == nil || wire.Stages == nil || wire.Unknowns == nil || wire.Conflicts == nil {
		return Capsule{}, fmt.Errorf("DBC wire external_bindings, stages, unknowns and conflicts must be JSON arrays")
	}
	for _, stage := range wire.Stages {
		if stage.Inputs == nil || stage.Claims == nil {
			return Capsule{}, fmt.Errorf("DBC stage %q inputs and claims must be present", stage.Stage)
		}
		for _, input := range stage.Inputs {
			if input.RequiredClaims == nil {
				return Capsule{}, fmt.Errorf("DBC stage %q input %q required_claims must be a JSON array", stage.Stage, input.Slot)
			}
		}
	}
	var trailing any
	if err := decoder.Decode(&trailing); err != io.EOF {
		if err == nil {
			return Capsule{}, fmt.Errorf("multiple JSON values are not permitted")
		}
		return Capsule{}, fmt.Errorf("trailing JSON data: %w", err)
	}

	capsule := Capsule{
		SchemaID: wire.SchemaID,
		ProfileRef: wire.ProfileRef,
		ProfileDigest: wire.ProfileDigest,
		DecisionID: wire.DecisionID,
		QueryID: wire.QueryID,
		QueryDigest: wire.QueryDigest,
		QuerySchema: wire.QuerySchema,
		ScopeDigest: wire.ScopeDigest,
		ValidAt: wire.ValidAt,
		Policy: refFromWire(wire.Policy.Receipt),
		Frontier: FrontierBinding{Kind: wire.Frontier.Kind, Receipt: refFromWire(wire.Frontier.Receipt), Status: wire.Frontier.Status, Prefix: wire.Frontier.Prefix},
		Progress: ProgressBinding{Kind: wire.Progress.Kind, Receipt: refFromWire(wire.Progress.Receipt), Status: wire.Progress.Status, Universe: wire.Progress.Universe, OwnerInventoryDigest: wire.Progress.OwnerDigest},
		Owner: OwnerBinding{Receipt: refFromWire(wire.Owner.Receipt), Authentication: wire.Owner.Authentication, Completeness: wire.Owner.Completeness, FrontierDigest: wire.Owner.FrontierDigest},
		Temporal: struct {
			OCELProfile, SourceCorpus, Projection, Replay Ref
			FrontierDigest, SourceCorpusDigest string
		}{
			OCELProfile: refFromWire(wire.Temporal.OCELProfile),
			SourceCorpus: refFromWire(wire.Temporal.SourceCorpus),
			Projection: refFromWire(wire.Temporal.Projection),
			Replay: refFromWire(wire.Temporal.Replay),
			FrontierDigest: wire.Temporal.FrontierDigest,
			SourceCorpusDigest: wire.Temporal.SourceCorpusDigest,
		},
		Evidence: EvidenceBinding{IndependenceStatus: wire.Evidence.IndependenceStatus},
		Authority: AuthorityBinding{Receipt: refFromWire(wire.Authority.Receipt), Principal: wire.Authority.Principal, Scope: wire.Authority.Scope, Validity: wire.Authority.Validity},
		Action: ActionBinding{Receipt: refFromWire(wire.Action.Receipt), Digest: wire.Action.Digest, Revision: wire.Action.Revision, CoverageRevision: wire.Action.CoverageRevision, Count: wire.Action.Count, Covered: wire.Action.Covered, CoverageDigest: wire.Action.CoverageDigest, Coverage: refFromWire(wire.Action.Coverage)},
		Execution: refFromWire(wire.Execution),
		Outcome: OutcomeBinding{Receipt: refFromWire(wire.Outcome.Receipt), Status: wire.Outcome.Status, Reconciliation: wire.Outcome.Reconciliation, ExecutionDigest: wire.Outcome.ExecutionDigest, ActionPlanDigest: wire.Outcome.ActionPlanDigest, MutationScope: wire.Outcome.MutationScope, SameScopeMutationAttempt: wire.Outcome.SameScopeMutationAttempt},
		Claims: wire.Claims,
		Disposition: wire.Disposition,
		Unknowns: wire.Unknowns,
		Conflicts: wire.Conflicts,
		Abstention: wire.Abstention,
	}
	if wire.Evidence.FailureDomainGraph != nil {
		value := refFromWire(*wire.Evidence.FailureDomainGraph)
		capsule.Evidence.FailureDomainGraph = &value
	}
	if wire.Evidence.LineageAttestation != nil {
		value := refFromWire(*wire.Evidence.LineageAttestation)
		capsule.Evidence.LineageAttestation = &value
	}
	for _, item := range wire.Stages {
		stage, err := parseStage(item.Stage)
		if err != nil { return Capsule{}, err }
		receipt := StageReceipt{
			ID: item.ID, Stage: stage, Schema: item.SchemaID, OutputSchema: item.OutputSchema,
			ProfileDigest: item.ProfileDigest, ScopeDigest: item.ScopeDigest, SourceCommit: item.SourceCommit,
			BuildDigest: item.BuildDigest, OutputDigest: item.OutputDigest, Claims: item.Claims,
		}
		for _, input := range item.Inputs {
			producer, err := parseStage(input.ProducerStage)
			if err != nil { return Capsule{}, err }
			receipt.Inputs = append(receipt.Inputs, Input{
				Slot: input.Slot, Producer: producer, ReceiptID: input.ProducerReceiptID,
				Schema: input.ProducerSchemaID, Digest: input.ProducerDigest, Required: input.RequiredClaims,
			})
		}
		capsule.Stages = append(capsule.Stages, receipt)
	}
	for _, item := range wire.External {
		consumer, err := parseStage(item.ConsumerStage)
		if err != nil { return Capsule{}, err }
		capsule.External = append(capsule.External, ExternalBinding{
			Consumer: consumer, Slot: item.Slot, ReceiptID: item.ReceiptID, Schema: item.SchemaID, Digest: item.Digest,
		})
	}
	return capsule, nil
}

func EncodeJSON(c Capsule) ([]byte, error) {
	wire := wireCapsule{
		SchemaID: c.SchemaID, ProfileRef: c.ProfileRef, ProfileDigest: c.ProfileDigest,
		DecisionID: c.DecisionID, QueryID: c.QueryID, QueryDigest: c.QueryDigest,
		QuerySchema: c.QuerySchema, ScopeDigest: c.ScopeDigest, ValidAt: c.ValidAt,
		Policy: struct { Receipt wireRef `json:"receipt"` }{Receipt: refToWire(c.Policy)},
		Frontier: wireFrontier{Kind: c.Frontier.Kind, Receipt: refToWire(c.Frontier.Receipt), Status: c.Frontier.Status, Prefix: c.Frontier.Prefix},
		Progress: wireProgress{Kind: c.Progress.Kind, Receipt: refToWire(c.Progress.Receipt), Status: c.Progress.Status, Universe: c.Progress.Universe, OwnerDigest: c.Progress.OwnerInventoryDigest},
		Owner: wireOwner{Receipt: refToWire(c.Owner.Receipt), Authentication: c.Owner.Authentication, Completeness: c.Owner.Completeness, FrontierDigest: c.Owner.FrontierDigest},
		Temporal: wireTemporal{
			OCELProfile: refToWire(c.Temporal.OCELProfile), SourceCorpus: refToWire(c.Temporal.SourceCorpus),
			Projection: refToWire(c.Temporal.Projection), Replay: refToWire(c.Temporal.Replay),
			FrontierDigest: c.Temporal.FrontierDigest, SourceCorpusDigest: c.Temporal.SourceCorpusDigest,
		},
		Evidence: wireEvidence{IndependenceStatus: c.Evidence.IndependenceStatus},
		Authority: wireAuthority{Receipt: refToWire(c.Authority.Receipt), Principal: c.Authority.Principal, Scope: c.Authority.Scope, Validity: c.Authority.Validity},
		Action: wireActionPlan{Receipt: refToWire(c.Action.Receipt), Digest: c.Action.Digest, Revision: c.Action.Revision,
			Count: c.Action.Count, Covered: c.Action.Covered, CoverageDigest: c.Action.CoverageDigest,
			CoverageRevision: c.Action.CoverageRevision, Coverage: refToWire(c.Action.Coverage)},
		Execution: refToWire(c.Execution),
		Outcome: wireOutcome{Receipt: refToWire(c.Outcome.Receipt), Status: c.Outcome.Status, Reconciliation: c.Outcome.Reconciliation,
			ExecutionDigest: c.Outcome.ExecutionDigest, ActionPlanDigest: c.Outcome.ActionPlanDigest, MutationScope: c.Outcome.MutationScope,
			SameScopeMutationAttempt: c.Outcome.SameScopeMutationAttempt},
		Claims: c.Claims, Disposition: c.Disposition, Unknowns: c.Unknowns, Conflicts: c.Conflicts, Abstention: c.Abstention,
	}
	if c.Evidence.FailureDomainGraph != nil {
		value := refToWire(*c.Evidence.FailureDomainGraph)
		wire.Evidence.FailureDomainGraph = &value
	}
	if c.Evidence.LineageAttestation != nil {
		value := refToWire(*c.Evidence.LineageAttestation)
		wire.Evidence.LineageAttestation = &value
	}
	for _, stage := range c.Stages {
		item := wireStage{ID: stage.ID, Stage: string(stage.Stage), SchemaID: stage.Schema, OutputSchema: stage.OutputSchema,
			ProfileDigest: stage.ProfileDigest, ScopeDigest: stage.ScopeDigest, SourceCommit: stage.SourceCommit,
			BuildDigest: stage.BuildDigest, OutputDigest: stage.OutputDigest, Claims: stage.Claims}
		for _, input := range stage.Inputs {
			item.Inputs = append(item.Inputs, wireInput{Slot: input.Slot, ProducerStage: string(input.Producer),
				ProducerReceiptID: input.ReceiptID, ProducerSchemaID: input.Schema, ProducerDigest: input.Digest, RequiredClaims: input.Required})
		}
		if item.Inputs == nil { item.Inputs = []wireInput{} }
		sort.Slice(item.Inputs, func(i, j int) bool { return item.Inputs[i].Slot < item.Inputs[j].Slot })
		wire.Stages = append(wire.Stages, item)
	}
	sort.Slice(wire.Stages, func(i, j int) bool {
		return stageRank(wire.Stages[i].Stage) < stageRank(wire.Stages[j].Stage)
	})
	if wire.Unknowns == nil { wire.Unknowns = []string{} }
	if wire.Conflicts == nil { wire.Conflicts = []string{} }
	for _, binding := range c.External {
		wire.External = append(wire.External, wireExternalBinding{
			ConsumerStage: string(binding.Consumer), Slot: binding.Slot, ReceiptID: binding.ReceiptID,
			SchemaID: binding.Schema, Digest: binding.Digest,
		})
	}
	sort.Slice(wire.External, func(i, j int) bool {
		if wire.External[i].ConsumerStage != wire.External[j].ConsumerStage {
			return stageRank(wire.External[i].ConsumerStage) < stageRank(wire.External[j].ConsumerStage)
		}
		return wire.External[i].Slot < wire.External[j].Slot
	})
	sort.Strings(wire.Unknowns)
	sort.Strings(wire.Conflicts)
	return json.Marshal(wire)
}

func stageRank(value string) int {
	for i, stage := range stageOrder {
		if string(stage) == value { return i }
	}
	return len(stageOrder)
}

func ValidateJSON(data []byte) (Capsule, error) {
	capsule, err := DecodeJSON(data)
	if err != nil { return Capsule{}, err }
	if err := Validate(capsule); err != nil { return Capsule{}, err }
	return capsule, nil
}
