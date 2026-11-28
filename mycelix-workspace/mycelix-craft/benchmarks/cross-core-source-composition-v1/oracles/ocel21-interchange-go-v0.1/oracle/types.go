package oracle

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"sort"
	"strconv"
	"strings"
	"time"
)

const epoch = "1970-01-01T00:00:00Z"

type Event struct {
	ID         string      "json:\"id\""
	Type       string      "json:\"type\""
	Time       string      "json:\"time\""
	Attributes []Attribute "json:\"attributes\""
}
type Object struct {
	ID   string "json:\"id\""
	Type string "json:\"type\""
}
type Attribute struct {
	ObjectID string "json:\"object_id,omitempty\""
	Name     string "json:\"name\""
	Time     string "json:\"time,omitempty\""
	Type     string "json:\"type\""
	Value    string "json:\"value\""
}
type Relation struct {
	Source    string "json:\"source\""
	Target    string "json:\"target\""
	Qualifier string "json:\"qualifier\""
}
type Projection struct {
	Events      []Event     "json:\"events\""
	Objects     []Object    "json:\"objects\""
	Assignments []Attribute "json:\"assignments\""
	E2O         []Relation  "json:\"e2o\""
	O2O         []Relation  "json:\"o2o\""
}

type attrDef struct {
	Name string "json:\"name\""
	Type string "json:\"type\""
}
type typeDef struct {
	Name       string    "json:\"name\""
	Attributes []attrDef "json:\"attributes\""
}
type jAttr struct {
	Name  string "json:\"name\""
	Time  string "json:\"time\""
	Value any    "json:\"value\""
}
type jRel struct {
	ObjectID  string "json:\"objectId\""
	Qualifier string "json:\"qualifier\""
}
type jEvent struct {
	ID            string  "json:\"id\""
	Type          string  "json:\"type\""
	Time          string  "json:\"time\""
	Attributes    []jAttr "json:\"attributes\""
	Relationships []jRel  "json:\"relationships\""
}
type jObject struct {
	ID            string  "json:\"id\""
	Type          string  "json:\"type\""
	Attributes    []jAttr "json:\"attributes\""
	Relationships []jRel  "json:\"relationships\""
}
type jLog struct {
	EventTypes  []typeDef "json:\"eventTypes\""
	ObjectTypes []typeDef "json:\"objectTypes\""
	Events      []jEvent  "json:\"events\""
	Objects     []jObject "json:\"objects\""
}
type bundleType struct {
	File        string    "json:\"file\""
	ChangesFile string    "json:\"changesFile\""
	Attributes  []attrDef "json:\"attributes\""
}
type bundleMeta struct {
	OCELVersion         string                "json:\"ocelVersion\""
	BundleFormatVersion string                "json:\"bundleFormatVersion\""
	StorageFormat       string                "json:\"storageFormat\""
	EventTypes          map[string]bundleType "json:\"eventTypes\""
	ObjectTypes         map[string]bundleType "json:\"objectTypes\""
	Relations           map[string]string     "json:\"relations\""
}

type objectRef struct {
	ID, Qualifier string
	Attributes    map[string]any
}

func value(v any, declared string) (string, string, error) {
	switch x := v.(type) {
	case string:
		if declared != "" && declared != "string" && declared != "time" {
			return "", "", fmt.Errorf("string incompatible with %s", declared)
		}
		if declared == "" {
			declared = "string"
		}
		return declared, x, nil
	case json.Number:
		raw := string(x)
		if declared == "" {
			if strings.ContainsAny(raw, ".eE") {
				declared = "float"
			} else {
				declared = "integer"
			}
		}
		if declared == "integer" {
			n, e := strconv.ParseInt(raw, 10, 64)
			if e != nil {
				return "", "", e
			}
			return declared, strconv.FormatInt(n, 10), nil
		}
		n, e := strconv.ParseFloat(raw, 64)
		if e != nil {
			return "", "", e
		}
		return declared, strconv.FormatFloat(n, 'f', -1, 64), nil
	case float64:
		if declared == "" {
			if x == float64(int64(x)) {
				declared = "integer"
			} else {
				declared = "float"
			}
		}
		if declared == "integer" {
			return declared, strconv.FormatInt(int64(x), 10), nil
		}
		return declared, strconv.FormatFloat(x, 'f', -1, 64), nil
	case bool:
		if declared == "" {
			declared = "boolean"
		}
		if declared != "boolean" {
			return "", "", fmt.Errorf("boolean incompatible with %s", declared)
		}
		return declared, strconv.FormatBool(x), nil
	case nil:
		return "null", "null", nil
	default:
		return "", "", fmt.Errorf("unsupported primitive %T", v)
	}
}
func parseOCELTime(raw string) (time.Time, error) {
	layouts := []string{time.RFC3339Nano, "2006-01-02T15:04:05-0700", "2006-01-02T15:04:05.999999999-0700"}
	for _, layout := range layouts {
		if t, err := time.Parse(layout, raw); err == nil {
			return t.UTC(), nil
		}
	}
	return time.Time{}, fmt.Errorf("timestamp %q lacks valid ISO 8601 timezone information", raw)
}

func csvValue(s, declared string) (string, string, error) {
	if declared != "" {
		return parseTypedCSV(s, declared)
	}
	if n, e := strconv.ParseInt(s, 10, 64); e == nil && strconv.FormatInt(n, 10) == s {
		return "integer", strconv.FormatInt(n, 10), nil
	}
	if f, e := strconv.ParseFloat(s, 64); e == nil && strconv.FormatFloat(f, 'g', -1, 64) == s {
		return "float", strconv.FormatFloat(f, 'g', -1, 64), nil
	}
	if strings.EqualFold(s, "true") || strings.EqualFold(s, "false") {
		b, _ := strconv.ParseBool(strings.ToLower(s))
		return "boolean", strconv.FormatBool(b), nil
	}
	if t, e := parseOCELTime(s); e == nil {
		return "time", t.Format(time.RFC3339Nano), nil
	}
	return "string", s, nil
}

func addObject(p *Projection, seen map[string]string, id, typ string) error {
	if id == "" || typ == "" {
		return errors.New("object ID and type required")
	}
	if old, ok := seen[id]; ok {
		if old != typ {
			return fmt.Errorf("object %s appears as both %s and %s", id, old, typ)
		}
		return nil
	}
	seen[id] = typ
	p.Objects = append(p.Objects, Object{id, typ})
	return nil
}
func normalize(p *Projection) {
	sort.Slice(p.Events, func(i, j int) bool { return p.Events[i].ID < p.Events[j].ID })
	for i := range p.Events {
		sort.Slice(p.Events[i].Attributes, func(a, b int) bool { return p.Events[i].Attributes[a].Name < p.Events[i].Attributes[b].Name })
	}
	sort.Slice(p.Objects, func(i, j int) bool { return p.Objects[i].ID < p.Objects[j].ID })
	sort.Slice(p.Assignments, func(i, j int) bool {
		a, b := p.Assignments[i], p.Assignments[j]
		if a.ObjectID != b.ObjectID {
			return a.ObjectID < b.ObjectID
		}
		if a.Name != b.Name {
			return a.Name < b.Name
		}
		if a.Time != b.Time {
			return a.Time < b.Time
		}
		return a.Value < b.Value
	})
	sort.Slice(p.E2O, func(i, j int) bool {
		a, b := p.E2O[i], p.E2O[j]
		if a.Source != b.Source {
			return a.Source < b.Source
		}
		if a.Target != b.Target {
			return a.Target < b.Target
		}
		return a.Qualifier < b.Qualifier
	})
	sort.Slice(p.O2O, func(i, j int) bool {
		a, b := p.O2O[i], p.O2O[j]
		if a.Source != b.Source {
			return a.Source < b.Source
		}
		if a.Target != b.Target {
			return a.Target < b.Target
		}
		return a.Qualifier < b.Qualifier
	})
}
func validate(p Projection) error {
	events, objects := map[string]bool{}, map[string]bool{}
	for _, e := range p.Events {
		if e.ID == "" || e.Type == "" || events[e.ID] {
			return fmt.Errorf("invalid or duplicate event %q", e.ID)
		}
		if _, err := parseOCELTime(e.Time); err != nil {
			return err
		}
		events[e.ID] = true
	}
	for _, o := range p.Objects {
		if o.ID == "" || o.Type == "" || objects[o.ID] {
			return fmt.Errorf("invalid or duplicate object %q", o.ID)
		}
		objects[o.ID] = true
	}
	assignmentValues := map[string]string{}
	for _, a := range p.Assignments {
		if !objects[a.ObjectID] || a.Name == "" || a.Type == "" {
			return fmt.Errorf("invalid attribute assignment %s.%s", a.ObjectID, a.Name)
		}
		if _, err := parseOCELTime(a.Time); err != nil {
			return err
		}
		key := a.ObjectID + "\x00" + a.Name + "\x00" + a.Time
		if old, ok := assignmentValues[key]; ok && old != a.Type+"\x00"+a.Value {
			return fmt.Errorf("conflicting object attribute assignment %s.%s at %s", a.ObjectID, a.Name, a.Time)
		}
		assignmentValues[key] = a.Type + "\x00" + a.Value
	}
	for _, r := range p.E2O {
		if !events[r.Source] || !objects[r.Target] {
			return fmt.Errorf("invalid E2O endpoint %s -> %s", r.Source, r.Target)
		}
	}
	for _, r := range p.O2O {
		if !objects[r.Source] || !objects[r.Target] {
			return fmt.Errorf("invalid O2O endpoint %s -> %s", r.Source, r.Target)
		}
	}
	return nil
}
func finish(p Projection) (Projection, error) {
	normalize(&p)
	if e := validate(p); e != nil {
		return p, e
	}
	return p, nil
}

func ParseJSON(data []byte) (Projection, error) {
	dec := json.NewDecoder(strings.NewReader(string(data)))
	dec.UseNumber()
	var log jLog
	if e := dec.Decode(&log); e != nil {
		return Projection{}, e
	}
	var extra any
	if e := dec.Decode(&extra); e != io.EOF {
		return Projection{}, errors.New("trailing JSON")
	}
	eventTypes, objectTypes, objectIDs := map[string]map[string]string{}, map[string]map[string]string{}, map[string]string{}
	for _, t := range log.EventTypes {
		m := map[string]string{}
		for _, a := range t.Attributes {
			m[a.Name] = a.Type
		}
		eventTypes[t.Name] = m
	}
	for _, t := range log.ObjectTypes {
		m := map[string]string{}
		for _, a := range t.Attributes {
			m[a.Name] = a.Type
		}
		objectTypes[t.Name] = m
	}
	p := Projection{}
	for _, o := range log.Objects {
		if o.ID == "" || o.Type == "" || objectTypes[o.Type] == nil || objectIDs[o.ID] != "" {
			return p, fmt.Errorf("invalid or duplicate object %s", o.ID)
		}
		objectIDs[o.ID] = o.Type
		p.Objects = append(p.Objects, Object{o.ID, o.Type})
	}
	for _, e := range log.Events {
		if e.ID == "" || e.Type == "" || eventTypes[e.Type] == nil || e.Time == "" {
			return p, errors.New("event requires id, type and time")
		}
		ev := Event{ID: e.ID, Type: e.Type, Time: e.Time}
		for _, a := range e.Attributes {
			typ, ok := eventTypes[e.Type][a.Name]
			if !ok {
				return p, fmt.Errorf("undeclared event attribute %s.%s", e.Type, a.Name)
			}
			t, v, x := value(a.Value, typ)
			if x != nil {
				return p, x
			}
			ev.Attributes = append(ev.Attributes, Attribute{Name: a.Name, Type: t, Value: v})
		}
		p.Events = append(p.Events, ev)
		for _, r := range e.Relationships {
			if objectIDs[r.ObjectID] == "" {
				return p, fmt.Errorf("unknown E2O object %s", r.ObjectID)
			}
			p.E2O = append(p.E2O, Relation{e.ID, r.ObjectID, r.Qualifier})
		}
	}
	for _, o := range log.Objects {
		for _, a := range o.Attributes {
			typ, ok := objectTypes[o.Type][a.Name]
			if !ok {
				return p, fmt.Errorf("undeclared object attribute %s.%s", o.Type, a.Name)
			}
			t, v, x := value(a.Value, typ)
			if x != nil {
				return p, x
			}
			p.Assignments = append(p.Assignments, Attribute{ObjectID: o.ID, Name: a.Name, Time: a.Time, Type: t, Value: v})
		}
		for _, r := range o.Relationships {
			if objectIDs[r.ObjectID] == "" {
				return p, fmt.Errorf("unknown O2O object %s", r.ObjectID)
			}
			p.O2O = append(p.O2O, Relation{o.ID, r.ObjectID, r.Qualifier})
		}
	}
	return finish(p)
}
