package oracle

import (
 "encoding/json"
 "math"
 "os"
 "testing"
)

func fixture()Case{
 actions:=[]Action{{"protect",map[string]float64{"unsafe":2,"safe":2}},{"release",map[string]float64{"unsafe":100,"safe":0}}}
 perfect:=ModelScenario{"calibrated-perfect",[]State{{"unsafe",.05},{"safe",.95}},map[string][]Likelihood{"unsafe":{{"unsafe-result",1}},"safe":{{"safe-result",1}}}}
 uninformative:=ModelScenario{"misspecified-uninformative",[]State{{"unsafe",.05},{"safe",.95}},map[string][]Likelihood{"unsafe":{{"unsafe-result",.5},{"safe-result",.5}},"safe":{{"unsafe-result",.5},{"safe-result",.5}}}}
 return Case{"calibration-ambiguity","two-models-v0.1",true,true,[]string{"protect","release"},actions,[]ModelScenario{perfect,uninformative},.2,true,true,true,false}
}
func near(t *testing.T,name string,g,w float64){t.Helper();if math.Abs(g-w)>1e-9{t.Fatalf("%s=%v want %v",name,g,w)}}
func TestRobustModelSetCanReverseNominalDecision(t *testing.T){
 r,e:=Evaluate(fixture());if e!=nil{t.Fatal(e)}
 if r.Status!=DoNotAcquire{t.Fatalf("status %s",r.Status)}
 near(t,"baseline",r.BaselineRobustRisk,2);near(t,"terminal",r.TerminalWorstCaseRisk,2);near(t,"acquisition",r.AcquisitionRobustRisk,2.2);near(t,"gross",r.RobustGrossEVSI,0);near(t,"net",r.RobustNetValue,-.2)
 if r.SelectedPolicy["unsafe-result"]!="protect"||r.SelectedPolicy["safe-result"]!="protect"{t.Fatalf("policy %#v",r.SelectedPolicy)}
 near(t,"nominal perfect gross",r.NominalPerModel[0].GrossEVSI,1.9);near(t,"nominal perfect net",r.NominalPerModel[0].NetEVSI,1.7)
 if r.RealWorldCalibration!="NotEvaluated"||r.ModelSetCoverageClaim!="EstablishedWithinDeclaredFiniteSet"{t.Fatalf("claim limits %#v",r)}
}
func TestSinglePerfectModelRecommends(t *testing.T){c:=fixture();c.Models=c.Models[:1];r,e:=Evaluate(c);if e!=nil{t.Fatal(e)};if r.Status!=Recommend{t.Fatalf("status %s",r.Status)};near(t,"terminal",r.TerminalWorstCaseRisk,.1);near(t,"net",r.RobustNetValue,1.7)}
func TestMissingCoverageAndInvalidModelsAbstain(t *testing.T){cases:=[]func(*Case){func(c *Case){c.CoverageEstablished=false},func(c *Case){c.ModelSetAdmissible=false},func(c *Case){c.ObservationInDomain=false},func(c *Case){c.ObservationAdmissible=false}};for _,mut:=range cases{c:=fixture();mut(&c);r,e:=Evaluate(c);if e!=nil{t.Fatal(e)};if r.Status!=Abstain||r.ModelSetCoverageClaim!="NotEvaluated"{t.Fatalf("did not abstain %#v",r)}}}
func TestMandatoryEvidencePreempts(t *testing.T){c:=fixture();c.MandatoryEvidenceOpen=true;c.ModelSetAdmissible=false;r,e:=Evaluate(c);if e!=nil{t.Fatal(e)};if r.Status!=AcquireMandatory{t.Fatalf("status %s",r.Status)}}
func TestMalformedLikelihoodAndDuplicateModelRejected(t *testing.T){c:=fixture();c.Models[0].LikelihoodByState["unsafe"][0].Given=.3;if _,e:=Evaluate(c);e==nil{t.Fatal("malformed likelihood accepted")};c=fixture();c.Models=append(c.Models,c.Models[0]);if _,e:=Evaluate(c);e==nil{t.Fatal("duplicate model accepted")}}
func TestPolicySpaceBounded(t *testing.T){c:=fixture();c.SafeActions=[]string{"a","b","c","d","e","f"};c.Actions=[]Action{};for _,id:=range c.SafeActions{c.Actions=append(c.Actions,Action{id,map[string]float64{"unsafe":1,"safe":1}})};for i:=0;i<9;i++{for _,m:=range c.Models{for state:=range m.LikelihoodByState{m.LikelihoodByState[state]=append(m.LikelihoodByState[state],Likelihood{ResultID:string(rune('a'+i)),Given:0})}}};if _,e:=Evaluate(c);e==nil{t.Fatal("oversized policy space accepted")}}
func TestFrozenFixture(t *testing.T){
 b,e:=os.ReadFile("../fixtures/robust-evsi-model-ambiguity-v0.1.json");if e!=nil{t.Fatal(e)}
 var f struct{Schema string `json:"schema_id"`;Case Case `json:"case"`;Expected struct{Status Status `json:"status"`;Baseline float64 `json:"baseline_robust_risk"`;Terminal float64 `json:"terminal_worst_case_risk"`;Acquisition float64 `json:"acquisition_robust_risk"`;Gross float64 `json:"robust_gross_evsi"`;Net float64 `json:"robust_net_value"`} `json:"expected"`}
 if e=json.Unmarshal(b,&f);e!=nil{t.Fatal(e)};if f.Schema!="mycelix.craft.robust-evsi-model-ambiguity.v0.1"{t.Fatal(f.Schema)}
 r,e:=Evaluate(f.Case);if e!=nil{t.Fatal(e)};if r.Status!=f.Expected.Status{t.Fatalf("%s want %s",r.Status,f.Expected.Status)};near(t,"baseline",r.BaselineRobustRisk,f.Expected.Baseline);near(t,"terminal",r.TerminalWorstCaseRisk,f.Expected.Terminal);near(t,"acquisition",r.AcquisitionRobustRisk,f.Expected.Acquisition);near(t,"gross",r.RobustGrossEVSI,f.Expected.Gross);near(t,"net",r.RobustNetValue,f.Expected.Net)
}
