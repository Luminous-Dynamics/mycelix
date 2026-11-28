
package oracle
import("encoding/json";"math";"os";"path/filepath";"testing")
func xorCase()Case{
 states:=[]State{{"00",.25},{"01",.25},{"10",.25},{"11",.25}}
 actions:=[]Action{{"A",map[string]float64{"00":0,"01":10,"10":10,"11":0}},{"B",map[string]float64{"00":10,"01":0,"10":0,"11":10}}}
 x:=Observation{ID:"observe-X",Cost:.3,Available:true,InDomain:true,Admissible:true,ConditionedOnLatentState:true,LikelihoodByState:map[string][]Likelihood{"00":{{"x0",1}},"01":{{"x0",1}},"10":{{"x1",1}},"11":{{"x1",1}}}}
 y:=Observation{ID:"observe-Y",Cost:.3,Available:true,InDomain:true,Admissible:true,ConditionedOnLatentState:true,LikelihoodByState:map[string][]Likelihood{"00":{{"y0",1}},"01":{{"y1",1}},"10":{{"y0",1}},"11":{{"y1",1}}}}
 return Case{ID:"XOR-complementarity",FactorizationProfile:"conditional_independence_given_complete_latent_state_v0.1",States:states,Actions:actions,SafeActions:[]string{"A","B"},Observations:[]Observation{x,y},Budget:.6,Horizon:2}
}
func near(t *testing.T,n string,v,w float64){t.Helper();if math.Abs(v-w)>1e-9{t.Fatalf("%s=%v want %v",n,v,w)}}
func TestComplementaryObservationsBeatGreedy(t *testing.T){c:=xorCase();id,net,e:=GreedyOneStep(c);if e!=nil{t.Fatal(e)};if id!=""||net!=0{t.Fatalf("greedy should stop, got %q %v",id,net)};r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=Recommend{t.Fatalf("status %s",r.Status)};near(t,"baseline",r.BaselineExpectedLoss,5);near(t,"terminal",r.ExpectedTerminalLoss,0);near(t,"cost",r.ExpectedAcquisitionCost,.6);near(t,"gross EVSI",r.GrossEVSI,5);near(t,"net",r.NetDecisionValue,4.4);if len(r.OneStepScores)!=2{t.Fatalf("one-step scores %#v",r.OneStepScores)};for _,s:=range r.OneStepScores{near(t,"one-step EVSI",s.EVSI,0)};if r.Plan.Stop||len(r.Plan.Branches)!=2{t.Fatalf("expected policy tree %#v",r.Plan)};for _,child:=range r.Plan.Branches{if child.Stop||child.ObservationID==""{t.Fatalf("second stage absent %#v",child)}}}
func TestUninformativeObservationNotAcquired(t *testing.T){c:=xorCase();for i:=range c.Observations{c.Observations[i].LikelihoodByState=map[string][]Likelihood{"00":{{"same",1}},"01":{{"same",1}},"10":{{"same",1}},"11":{{"same",1}}}};r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=DoNotAcquire{t.Fatalf("status %s",r.Status)};near(t,"EVSI",r.GrossEVSI,0);if !r.Plan.Stop{t.Fatal("must stop")}}
func TestBudgetInfeasible(t *testing.T){c:=xorCase();c.Budget=.5;r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=DoNotAcquire||!r.Plan.Stop{t.Fatalf("budget gate failed %#v",r)};near(t,"net",r.NetDecisionValue,0)}
func TestMandatoryPreempts(t *testing.T){c:=xorCase();c.MandatoryEvidenceOpen=true;r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=AcquireMandatory{t.Fatalf("status %s",r.Status)};c.MandatoryEvidenceOpen=false;c.Observations[0].Mandatory=true;r,e=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=AcquireMandatory{t.Fatalf("status %s",r.Status)}}
func TestUnknownDependenceAbstains(t *testing.T){c:=xorCase();c.FactorizationProfile="";r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=Abstain{t.Fatalf("status %s",r.Status)};near(t,"baseline",r.BaselineExpectedLoss,5);if r.Plan==nil||!r.Plan.Stop{t.Fatal("abstention must preserve stop baseline")}}
func TestInvalidObservationModelsAbstain(t *testing.T){c:=xorCase();c.Observations[0].ConditionedOnLatentState=false;c.Observations[1].InDomain=false;r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=Abstain{t.Fatalf("status %s",r.Status)}}
func TestBadLikelihoodRejected(t *testing.T){c:=xorCase();c.Observations[0].LikelihoodByState["00"]=[]Likelihood{{"x0",.4}};if _,e:=Solve(c);e==nil{t.Fatal("bad likelihood accepted")}}

func TestFrozenComplementarityFixture(t *testing.T) {
 data,err:=os.ReadFile(filepath.Join("..","fixtures","sequential-evsi-complementarity-v0.1.json"));if err!=nil{t.Fatal(err)}
 var fixture struct {
  SchemaID string `json:"schema_id"`
  Case Case `json:"case"`
  Expected struct {
   Baseline float64 `json:"baseline_expected_loss"`
   XEVSI float64 `json:"observe_x_one_step_evsi"`
   YEVSI float64 `json:"observe_y_one_step_evsi"`
   GreedyStops bool `json:"greedy_one_step_stops"`
   Terminal float64 `json:"expected_terminal_loss"`
   Cost float64 `json:"expected_acquisition_cost"`
   Gross float64 `json:"gross_evsi"`
   Net float64 `json:"net_decision_value"`
  } `json:"expected"`
 }
 if err:=json.Unmarshal(data,&fixture);err!=nil{t.Fatal(err)}
 if fixture.SchemaID!="mycelix.craft.sequential-evsi-complementarity.v0.1"{t.Fatalf("schema %q",fixture.SchemaID)}
 report,err:=Solve(fixture.Case);if err!=nil{t.Fatal(err)}
 near(t,"fixture baseline",report.BaselineExpectedLoss,fixture.Expected.Baseline)
 near(t,"fixture terminal",report.ExpectedTerminalLoss,fixture.Expected.Terminal)
 near(t,"fixture cost",report.ExpectedAcquisitionCost,fixture.Expected.Cost)
 near(t,"fixture gross EVSI",report.GrossEVSI,fixture.Expected.Gross)
 near(t,"fixture net",report.NetDecisionValue,fixture.Expected.Net)
 id,_,err:=GreedyOneStep(fixture.Case);if err!=nil{t.Fatal(err)}
 if (id=="")!=fixture.Expected.GreedyStops{t.Fatalf("greedy selected %q",id)}
 if len(report.OneStepScores)!=2{t.Fatalf("one-step scores %#v",report.OneStepScores)}
 near(t,"X one-step EVSI",report.OneStepScores[0].EVSI,fixture.Expected.XEVSI)
 near(t,"Y one-step EVSI",report.OneStepScores[1].EVSI,fixture.Expected.YEVSI)
}
func bruteForceTotal(c Case,belief []float64,available []string,budget float64,horizon int)float64{
 best:=math.Inf(1)
 for _,actionID:=range c.SafeActions{loss:=0.0;for i,state:=range c.States{for _,action:=range c.Actions{if action.ID==actionID{loss+=belief[i]*action.LossByState[state.ID];break}}};if loss<best{best=loss}}
 if horizon==0||len(available)==0{return best}
 for _,id:=range available{
  var observation Observation;found:=false
  for _,candidate:=range c.Observations{if candidate.ID==id{observation=candidate;found=true;break}}
  if !found||!observation.Available||!observation.InDomain||!observation.Admissible||!observation.ConditionedOnLatentState||observation.Mandatory||observation.Cost>budget+eps{continue}
  masses:=map[string]float64{};joint:=map[string][]float64{}
  for i,state:=range c.States{for _,likelihood:=range observation.LikelihoodByState[state.ID]{masses[likelihood.ResultID]+=belief[i]*likelihood.Probability;if joint[likelihood.ResultID]==nil{joint[likelihood.ResultID]=make([]float64,len(belief))};joint[likelihood.ResultID][i]=likelihood.Probability}}
  remaining:=make([]string,0,len(available)-1);for _,other:=range available{if other!=id{remaining=append(remaining,other)}}
  value:=observation.Cost
  for resultID,mass:=range masses{if mass<=eps{continue};posterior:=make([]float64,len(belief));for i:=range belief{posterior[i]=belief[i]*joint[resultID][i]/mass};value+=mass*bruteForceTotal(c,posterior,remaining,budget-observation.Cost,horizon-1)}
  if value<best{best=value}
 }
 return best
}
func TestDynamicProgramMatchesUnmemoizedPolicyTreeEnumeration(t *testing.T){
 c:=xorCase();report,err:=Solve(c);if err!=nil{t.Fatal(err)}
 brute:=bruteForceTotal(c,[]float64{.25,.25,.25,.25},[]string{"observe-X","observe-Y"},c.Budget,c.Horizon)
 near(t,"unmemoized exhaustive policy tree",report.ExpectedTotalLoss,brute)
}
