// Package oracle deterministically materializes the frozen Craft synthetic
// decision-episode mutation registry. It is a test-only fixture generator.
package oracle

import (
 "bytes"
 "crypto/sha256"
 "encoding/hex"
 "encoding/json"
 "errors"
 "fmt"
 "sort"
)

const EpisodeDomain = "MYCELIX-CRAFT-SYNTHETIC-EPISODE-V0.1\x00"

type MutationCase struct {
 ID string `json:"id"`
 Mutation string `json:"mutation"`
 Missingness string `json:"missingness,omitempty"`
 Expected map[string]any `json:"expected"`
 Purpose string `json:"purpose,omitempty"`
 Event map[string]any `json:"event,omitempty"`
 SelectionPolicy map[string]any `json:"selection_policy,omitempty"`
}
type Registry struct {
 SchemaID string `json:"schema_id"`
 FixtureID string `json:"fixture_id"`
 Seed uint64 `json:"seed"`
 Mutations []MutationCase `json:"mutations"`
}
type Bundle struct {
 SchemaID string `json:"schema_id"`
 GeneratorVersion string `json:"generator_version"`
 Seed uint64 `json:"seed"`
 ScenarioID string `json:"scenario_id"`
 Mutation string `json:"mutation"`
 Missingness string `json:"missingness,omitempty"`
 BaseFixtureDigest string `json:"base_fixture_digest"`
 ScenarioRegistryDigest string `json:"scenario_registry_digest"`
 TransformedEpisodeDigest string `json:"transformed_episode_digest"`
 Expected map[string]any `json:"expected"`
 Purpose string `json:"purpose,omitempty"`
 Episode map[string]any `json:"episode"`
}

func decodeObject(raw []byte) (map[string]any,error) {
 d:=json.NewDecoder(bytes.NewReader(raw)); d.UseNumber(); var v map[string]any
 if err:=d.Decode(&v); err!=nil{return nil,err}; if v==nil{return nil,errors.New("root must be object")}
 var extra any; if err:=d.Decode(&extra); err==nil{return nil,errors.New("trailing JSON")}
 return v,nil
}
func canonical(v any)([]byte,error){return json.Marshal(v)}
func Digest(domain string,v any)(string,error){b,e:=canonical(v);if e!=nil{return "",e};h:=sha256.Sum256(append([]byte(domain),b...));return hex.EncodeToString(h[:]),nil}
func RawDigest(domain string,b []byte)string{h:=sha256.Sum256(append([]byte(domain),b...));return hex.EncodeToString(h[:])}
func clone(v map[string]any)(map[string]any,error){b,e:=canonical(v);if e!=nil{return nil,e};return decodeObject(b)}
func list(v any,name string)([]any,error){x,ok:=v.([]any);if !ok{return nil,fmt.Errorf("%s must be array",name)};return x,nil}
func object(v any,name string)(map[string]any,error){x,ok:=v.(map[string]any);if !ok{return nil,fmt.Errorf("%s must be object",name)};return x,nil}
func find(items []any,key,want string)(map[string]any,error){for _,v:=range items{m,e:=object(v,key+" item");if e!=nil{return nil,e};if m[key]==want{return m,nil}};return nil,fmt.Errorf("%s=%s missing",key,want)}
func remove(items []any,key,want string)([]any,error){out:=make([]any,0,len(items));found:=false;for _,v:=range items{m,e:=object(v,key+" item");if e!=nil{return nil,e};if m[key]==want{found=true;continue};out=append(out,v)};if !found{return nil,fmt.Errorf("%s=%s missing",key,want)};return out,nil}

func apply(ep map[string]any,c MutationCase)error{
 switch c.ID {
 case "SYN-001":
  if c.Mutation!="identity"{return errors.New("identity contract mismatch")}
 case "SYN-002":
  x,e:=list(ep["source_universe"],"source_universe");if e!=nil{return e};ep["source_universe"],e=remove(x,"source_id","calibration-registry");if e!=nil{return e};ep["source_universe_coverage"]="Indeterminate";ep["source_omission"]=map[string]any{"source_id":"calibration-registry","missingness":c.Missingness}
 case "SYN-003":
  if c.Event==nil{return errors.New("late event missing")};x,e:=list(ep["events"],"events");if e!=nil{return e};for _,v:=range x{m,_:=object(v,"event");if m["event_id"]==c.Event["event_id"]{return errors.New("late event ID exists")}};ep["events"]=append(x,c.Event);ep["late_arrival"]=map[string]any{"event_id":"EV-010","historical_frontier_mutates":false,"new_frontier_requires_recomputation":true}
 case "SYN-004":
  x,e:=list(ep["events"],"events");if e!=nil{return e};a,e:=find(x,"event_id","EV-003");if e!=nil{return e};b,e:=find(x,"event_id","EV-005");if e!=nil{return e};b["event_time"]=a["event_time"];b["parents"]=[]any{};ep["causal_comparison"]=map[string]any{"left":"EV-003","right":"EV-005","expected":"IncomparableWithoutCausalPath"}
 case "SYN-005":
  x,e:=list(ep["events"],"events");if e!=nil{return e};m,e:=find(x,"event_id","EV-003");if e!=nil{return e};fork,e:=clone(m);if e!=nil{return e};fork["event_id"]="EV-011";fork["activity"]="forked_pressure_alarm";ep["events"]=append(x,fork)
 case "SYN-006":
  deps,e:=list(ep["dependencies"],"dependencies");if e!=nil{return e};if len(deps)<2{return errors.New("need two dependency entries")};ep["independence_claim"]=map[string]any{"components":[]any{"utility_pressure_sensor","lab_measurement_chain"},"claimed":"Independent","expected":"NotEvaluated","shared_root":"CAL-4"}
 case "SYN-007":
  if c.SelectionPolicy==nil{return errors.New("selection policy missing")};ep["inspection_policy"]=c.SelectionPolicy;ep["off_policy_status"]="NotEvaluated"
 case "SYN-008":
  ep["execution_reconciliation"]=map[string]any{"outcome":"Unknown","same_scope_mutation_attempted":true,"expected":"RejectedByMutationBarrier"}
 case "SYN-009":
  x,e:=list(ep["object_relationships"],"object_relationships");if e!=nil{return e};out:=make([]any,0,len(x));for _,v:=range x{m,e:=object(v,"relationship");if e!=nil{return e};if m["source"]=="CAL-4"&&m["target"]=="LB-17"{continue};out=append(out,v)};if len(out)==len(x){return errors.New("target relationship missing")};ep["object_relationships"]=out
 case "SYN-010":
  ep["source_feed_attestation"]=map[string]any{"signature_status":"fixture_valid","selective_omission":true};ep["source_universe_coverage"]="Indeterminate"
 default:return fmt.Errorf("unsupported scenario %q",c.ID)
 }
 return nil
}

func Generate(baseRaw,registryRaw []byte)([]Bundle,error){
 base,e:=decodeObject(baseRaw);if e!=nil{return nil,fmt.Errorf("base episode: %w",e)}
 d:=json.NewDecoder(bytes.NewReader(registryRaw));d.UseNumber();var reg Registry;if e=d.Decode(&reg);e!=nil{return nil,e}
 if reg.SchemaID!="mycelix.craft.synthetic-scenario-mutations.v0.1"||reg.FixtureID==""||len(reg.Mutations)==0{return nil,errors.New("invalid registry header")}
 baseDigest:=RawDigest(EpisodeDomain,baseRaw);regDigest:=RawDigest(EpisodeDomain,registryRaw)
 cases:=append([]MutationCase(nil),reg.Mutations...);sort.Slice(cases,func(i,j int)bool{return cases[i].ID<cases[j].ID})
 seen:=map[string]bool{};out:=make([]Bundle,0,len(cases))
 for _,c:=range cases{if c.ID==""||seen[c.ID]{return nil,fmt.Errorf("duplicate or empty scenario ID %q",c.ID)};seen[c.ID]=true;ep,e:=clone(base);if e!=nil{return nil,e};if e=apply(ep,c);e!=nil{return nil,fmt.Errorf("%s: %w",c.ID,e)};digest,e:=Digest(EpisodeDomain,ep);if e!=nil{return nil,e};out=append(out,Bundle{SchemaID:"mycelix.craft.synthetic-scenario-bundle.v0.1",GeneratorVersion:"0.1.0",Seed:reg.Seed,ScenarioID:c.ID,Mutation:c.Mutation,Missingness:c.Missingness,BaseFixtureDigest:baseDigest,ScenarioRegistryDigest:regDigest,TransformedEpisodeDigest:digest,Expected:c.Expected,Purpose:c.Purpose,Episode:ep})}
 return out,nil
}
func GenerateJSON(base,registry []byte)([]byte,error){v,e:=Generate(base,registry);if e!=nil{return nil,e};return json.MarshalIndent(v,"","  ")}
