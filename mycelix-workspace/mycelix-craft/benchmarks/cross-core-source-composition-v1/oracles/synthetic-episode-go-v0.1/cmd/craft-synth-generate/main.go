package main

import (
 "flag"
 "fmt"
 "os"
 "example.com/mycelix/craft-synthetic-episode-oracle/oracle"
)
func main(){
 episode:=flag.String("episode","","immutable episode JSON")
 scenarios:=flag.String("scenarios","","frozen scenario registry JSON")
 out:=flag.String("out","","output JSON file")
 flag.Parse()
 if *episode==""||*scenarios==""||*out==""{fmt.Fprintln(os.Stderr,"usage: craft-synth-generate -episode FILE -scenarios FILE -out FILE");os.Exit(2)}
 base,e:=os.ReadFile(*episode);fatal(e)
 registry,e:=os.ReadFile(*scenarios);fatal(e)
 b,e:=oracle.GenerateJSON(base,registry);fatal(e)
 fatal(os.WriteFile(*out,append(b,'\n'),0644))
}
func fatal(e error){if e!=nil{fmt.Fprintln(os.Stderr,e);os.Exit(1)}}
