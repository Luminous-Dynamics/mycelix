package oracle

import (
	"encoding/csv"
	"encoding/json"
	"errors"
	"fmt"
	"strconv"
	"strings"
)

func parseRefs(s string) ([]objectRef, error) {
	var out []objectRef
	for i := 0; i < len(s); {
		r := objectRef{}
		readPart := func(stopHash bool) (string, error) {
			var b strings.Builder
			for i < len(s) && s[i] != '/' && s[i] != '{' && !(stopHash && s[i] == '#') {
				if s[i] == '\\' {
					i++
					if i >= len(s) || !strings.ContainsRune("/#\\{", rune(s[i])) {
						return "", errors.New("invalid object-reference escape")
					}
					b.WriteByte(s[i])
					i++
					continue
				}
				if !stopHash && s[i] == '#' {
					return "", errors.New("unescaped # in object qualifier")
				}
				b.WriteByte(s[i])
				i++
			}
			return b.String(), nil
		}
		id, err := readPart(true)
		if err != nil {
			return nil, err
		}
		if id == "" {
			return nil, errors.New("empty object ID")
		}
		r.ID = strings.TrimSpace(id)
		if r.ID == "" {
			return nil, errors.New("empty object ID after whitespace normalization")
		}
		if i < len(s) && s[i] == '#' {
			i++
			q, err := readPart(false)
			if err != nil {
				return nil, err
			}
			if q == "" {
				return nil, errors.New("empty object qualifier")
			}
			r.Qualifier = strings.TrimSpace(q)
		}
		if i < len(s) && s[i] == '{' {
			start, depth := i, 0
			quote, escape := false, false
			for i < len(s) {
				c := s[i]
				if quote {
					if escape {
						escape = false
					} else if c == '\\' {
						escape = true
					} else if c == '"' {
						quote = false
					}
				} else {
					switch c {
					case '"':
						quote = true
					case '{':
						depth++
					case '}':
						depth--
						if depth == 0 {
							i++
							goto parsedJSON
						}
					}
				}
				i++
			}
			return nil, errors.New("unterminated object attributes")
		parsedJSON:
			dec := json.NewDecoder(strings.NewReader(s[start:i]))
			dec.UseNumber()
			if err := dec.Decode(&r.Attributes); err != nil {
				return nil, err
			}
			if r.Attributes == nil {
				return nil, errors.New("object attributes must be a JSON object")
			}
			for _, v := range r.Attributes {
				switch v.(type) {
				case nil, string, bool, json.Number:
				default:
					return nil, errors.New("object-reference attributes must be primitive JSON values")
				}
			}
		}
		out = append(out, r)
		if i < len(s) {
			if s[i] != '/' {
				return nil, errors.New("invalid object-reference delimiter")
			}
			i++
			if i == len(s) {
				return nil, errors.New("trailing object-reference delimiter")
			}
		}
	}
	return out, nil
}

func unifiedType(values []Attribute) string {
	if len(values) == 0 {
		return "string"
	}
	allInt, allNumeric, allBool, allTime := true, true, true, true
	for _, a := range values {
		allInt = allInt && a.Type == "integer"
		allNumeric = allNumeric && (a.Type == "integer" || a.Type == "float")
		allBool = allBool && a.Type == "boolean"
		allTime = allTime && a.Type == "time"
	}
	if allInt {
		return "integer"
	}
	if allNumeric {
		return "float"
	}
	if allBool {
		return "boolean"
	}
	if allTime {
		return "time"
	}
	return "string"
}

func unifySingleCSVScopes(p *Projection) {
	eventGroups := map[string][]Attribute{}
	for _, e := range p.Events {
		for _, a := range e.Attributes {
			eventGroups[a.Name] = append(eventGroups[a.Name], a)
		}
	}
	for i := range p.Events {
		for j := range p.Events[i].Attributes {
			a := &p.Events[i].Attributes[j]
			a.Type = unifiedType(eventGroups[a.Name])
			if a.Type == "float" && a.Value != "" {
				if n, err := strconv.ParseFloat(a.Value, 64); err == nil {
					a.Value = strconv.FormatFloat(n, 'g', -1, 64)
				}
			}
		}
	}
	objectTypes := map[string]string{}
	for _, o := range p.Objects {
		objectTypes[o.ID] = o.Type
	}
	objectGroups := map[string][]Attribute{}
	for _, a := range p.Assignments {
		scope := objectTypes[a.ObjectID] + "\\x00" + a.Name
		objectGroups[scope] = append(objectGroups[scope], a)
	}
	for i := range p.Assignments {
		a := &p.Assignments[i]
		scope := objectTypes[a.ObjectID] + "\\x00" + a.Name
		a.Type = unifiedType(objectGroups[scope])
		if a.Type == "float" && a.Value != "" {
			if n, err := strconv.ParseFloat(a.Value, 64); err == nil {
				a.Value = strconv.FormatFloat(n, 'g', -1, 64)
			}
		}
	}
}

func ParseSingleCSV(data []byte) (Projection, error) {
	r := csv.NewReader(strings.NewReader(string(data)))
	records, e := r.ReadAll()
	if e != nil {
		return Projection{}, e
	}
	if len(records) < 2 {
		return Projection{}, errors.New("CSV needs header and rows")
	}
	h := records[0]
	cols, objectCols := map[string]int{}, map[int]string{}
	for i, x := range h {
		if x == "" {
			return Projection{}, errors.New("empty CSV column name")
		}
		if _, ok := cols[x]; ok {
			return Projection{}, fmt.Errorf("duplicate CSV column %s", x)
		}
		cols[x] = i
		if strings.HasPrefix(x, "ot:") {
			typ := strings.TrimPrefix(x, "ot:")
			if typ == "" {
				return Projection{}, errors.New("empty object type column suffix")
			}
			objectCols[i] = typ
		}
	}
	idx := func(k string) int {
		if i, ok := cols[k]; ok {
			return i
		}
		return -1
	}
	get := func(row []string, i int) string {
		if i < 0 || i >= len(row) {
			return ""
		}
		return row[i]
	}
	for i, row := range records[1:] {
		if len(row) != len(h) {
			return Projection{}, fmt.Errorf("CSV line %d has %d fields; want %d", i+2, len(row), len(h))
		}
	}
	idI, actI, timeI := idx("id"), idx("activity"), idx("timestamp")
	if idI < 0 || actI < 0 || timeI < 0 {
		return Projection{}, errors.New("single-file CSV requires id, activity and timestamp columns")
	}
	p := Projection{}
	seen := map[string]string{}
	for n, row := range records[1:] {
		line := n + 2
		id, act, tm := strings.TrimSpace(get(row, idI)), strings.TrimSpace(get(row, actI)), strings.TrimSpace(get(row, timeI))
		if id != "" && strings.EqualFold(act, "o2o") {
			if seen[id] == "" {
				return p, fmt.Errorf("O2O source %s has no earlier type declaration", id)
			}
			for ci, typ := range objectCols {
				refs, x := parseRefs(get(row, ci))
				if x != nil {
					return p, x
				}
				for _, q := range refs {
					if x = addObject(&p, seen, q.ID, typ); x != nil {
						return p, x
					}
					p.O2O = append(p.O2O, Relation{id, q.ID, q.Qualifier})
					if q.Attributes != nil && tm == "" {
						return p, errors.New("O2O row with JSON attributes requires timestamp")
					}
					if tm != "" {
						for k, v := range q.Attributes {
							t, val, x := value(v, "")
							if x != nil {
								return p, x
							}
							p.Assignments = append(p.Assignments, Attribute{ObjectID: q.ID, Name: k, Time: tm, Type: t, Value: val})
						}
					}
				}
			}
			continue
		}
		if id != "" && act != "" {
			if tm == "" {
				return p, fmt.Errorf("event line %d missing timestamp", line)
			}
			if _, x := parseOCELTime(tm); x != nil {
				return p, x
			}
			ev := Event{ID: id, Type: act, Time: tm}
			for ci, name := range h {
				if _, ok := objectCols[ci]; ok || ci == idI || ci == actI || ci == timeI {
					continue
				}
				raw := get(row, ci)
				if raw != "" {
					t, v, x := csvValue(raw, "")
					if x != nil {
						return p, x
					}
					ev.Attributes = append(ev.Attributes, Attribute{Name: name, Type: t, Value: v})
				}
			}
			p.Events = append(p.Events, ev)
			for ci, typ := range objectCols {
				refs, x := parseRefs(get(row, ci))
				if x != nil {
					return p, x
				}
				for _, q := range refs {
					if x = addObject(&p, seen, q.ID, typ); x != nil {
						return p, x
					}
					p.E2O = append(p.E2O, Relation{id, q.ID, q.Qualifier})
					for k, v := range q.Attributes {
						t, val, x := value(v, "")
						if x != nil {
							return p, x
						}
						p.Assignments = append(p.Assignments, Attribute{ObjectID: q.ID, Name: k, Time: tm, Type: t, Value: val})
					}
				}
			}
			continue
		}
		if id == "" && act == "" {
			when := tm
			if when == "" {
				when = epoch
			} else if _, x := parseOCELTime(when); x != nil {
				return p, x
			}
			for ci, typ := range objectCols {
				refs, x := parseRefs(get(row, ci))
				if x != nil {
					return p, x
				}
				for _, q := range refs {
					if q.Qualifier != "" {
						return p, errors.New("qualifier is invalid in an object declaration row")
					}
					if x = addObject(&p, seen, q.ID, typ); x != nil {
						return p, x
					}
					for k, v := range q.Attributes {
						t, val, x := value(v, "")
						if x != nil {
							return p, x
						}
						p.Assignments = append(p.Assignments, Attribute{ObjectID: q.ID, Name: k, Time: when, Type: t, Value: val})
					}
				}
			}
			continue
		}
		return p, fmt.Errorf("invalid CSV row discriminator at line %d", line)
	}
	if _, err := finish(p); err != nil {
		return p, err
	}
	unifySingleCSVScopes(&p)
	return finish(p)
}
