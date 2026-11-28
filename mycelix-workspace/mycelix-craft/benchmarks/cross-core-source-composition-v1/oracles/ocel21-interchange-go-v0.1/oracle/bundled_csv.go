package oracle

import (
	"encoding/csv"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

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

func readCSV(path string) ([][]string, error) {
	b, e := os.ReadFile(path)
	if e != nil {
		return nil, e
	}
	r := csv.NewReader(strings.NewReader(string(b)))
	return r.ReadAll()
}
func parseTypedCSV(raw, declared string) (string, string, error) {
	switch declared {
	case "string":
		return "string", raw, nil
	case "time":
		t, err := parseOCELTime(raw)
		if err != nil {
			return "", "", err
		}
		return "time", t.Format(time.RFC3339Nano), nil
	case "integer":
		n, e := strconv.ParseInt(raw, 10, 64)
		if e != nil {
			return "", "", e
		}
		return "integer", strconv.FormatInt(n, 10), nil
	case "float":
		n, e := strconv.ParseFloat(raw, 64)
		if e != nil {
			return "", "", e
		}
		return "float", strconv.FormatFloat(n, 'f', -1, 64), nil
	case "boolean":
		b, e := strconv.ParseBool(strings.ToLower(raw))
		if e != nil {
			return "", "", e
		}
		return "boolean", strconv.FormatBool(b), nil
	default:
		return "", "", fmt.Errorf("unsupported or missing declared primitive type %q", declared)
	}
}

func safeBundlePath(root, name string) (string, error) {
	if name == "" || strings.Contains(name, "\\") || filepath.IsAbs(name) {
		return "", fmt.Errorf("invalid bundle-relative path %q", name)
	}
	for _, part := range strings.Split(name, "/") {
		if part == "" || part == "." || part == ".." {
			return "", fmt.Errorf("invalid bundle-relative path %q", name)
		}
	}
	return filepath.Join(root, filepath.FromSlash(name)), nil
}

func readTable(root, rel string, required []string) ([][]string, map[string]int, error) {
	path, err := safeBundlePath(root, rel)
	if err != nil {
		return nil, nil, err
	}
	rows, err := readCSV(path)
	if err != nil {
		return nil, nil, err
	}
	if len(rows) == 0 {
		return nil, nil, fmt.Errorf("table %s is empty", rel)
	}
	cols := map[string]int{}
	for i, name := range rows[0] {
		if name == "" {
			return nil, nil, fmt.Errorf("table %s has an empty column name", rel)
		}
		if _, ok := cols[name]; ok {
			return nil, nil, fmt.Errorf("table %s has duplicate column %s", rel, name)
		}
		cols[name] = i
	}
	for _, name := range required {
		if _, ok := cols[name]; !ok {
			return nil, nil, fmt.Errorf("table %s lacks required column %s", rel, name)
		}
	}
	for i, row := range rows[1:] {
		if len(row) != len(rows[0]) {
			return nil, nil, fmt.Errorf("table %s row %d has %d fields, want %d", rel, i+2, len(row), len(rows[0]))
		}
	}
	return rows, cols, nil
}

func declaredTypes(defs []attrDef) (map[string]string, error) {
	out := map[string]string{}
	for _, d := range defs {
		if d.Name == "" {
			return nil, errors.New("empty attribute name in metadata")
		}
		if _, ok := out[d.Name]; ok {
			return nil, fmt.Errorf("duplicate metadata attribute %s", d.Name)
		}
		switch d.Type {
		case "string", "time", "integer", "float", "boolean":
		default:
			return nil, fmt.Errorf("unsupported metadata type %s", d.Type)
		}
		out[d.Name] = d.Type
	}
	return out, nil
}

func ParseBundledCSV(root string) (Projection, error) {
	metaPath, err := safeBundlePath(root, "ocel-meta.json")
	if err != nil {
		return Projection{}, err
	}
	b, err := os.ReadFile(metaPath)
	if err != nil {
		return Projection{}, err
	}
	var m bundleMeta
	if err = json.Unmarshal(b, &m); err != nil {
		return Projection{}, err
	}
	if m.StorageFormat != "csv" || m.OCELVersion != "2.0" || m.BundleFormatVersion != "1.0" {
		return Projection{}, errors.New("unsupported bundled metadata profile")
	}
	p := Projection{}
	seen := map[string]string{}

	for typ, d := range m.EventTypes {
		if typ == "" {
			return p, errors.New("empty event type in metadata")
		}
		attrs, e := declaredTypes(d.Attributes)
		if e != nil {
			return p, e
		}
		rows, cols, e := readTable(root, d.File, []string{"ocel_id", "ocel_time"})
		if e != nil {
			return p, e
		}
		for name := range attrs {
			if _, ok := cols[name]; !ok {
				return p, fmt.Errorf("event table %s lacks declared attribute %s", d.File, name)
			}
		}
		for _, row := range rows[1:] {
			ev := Event{ID: row[cols["ocel_id"]], Type: typ, Time: row[cols["ocel_time"]]}
			if ev.ID == "" || ev.Time == "" {
				return p, errors.New("event row requires ID and time")
			}
			for name, typName := range attrs {
				raw := row[cols[name]]
				if raw == "" {
					continue
				}
				t, v, e := parseTypedCSV(raw, typName)
				if e != nil {
					return p, fmt.Errorf("%s.%s: %w", typ, name, e)
				}
				ev.Attributes = append(ev.Attributes, Attribute{Name: name, Type: t, Value: v})
			}
			p.Events = append(p.Events, ev)
		}
	}

	for typ, d := range m.ObjectTypes {
		if typ == "" || d.ChangesFile == "" {
			return p, errors.New("every object type requires an object and object-change table")
		}
		attrs, e := declaredTypes(d.Attributes)
		if e != nil {
			return p, e
		}
		rows, cols, e := readTable(root, d.File, []string{"ocel_id"})
		if e != nil {
			return p, e
		}
		for name := range attrs {
			if _, ok := cols[name]; !ok {
				return p, fmt.Errorf("object table %s lacks declared attribute %s", d.File, name)
			}
		}
		for _, row := range rows[1:] {
			id := row[cols["ocel_id"]]
			if e = addObject(&p, seen, id, typ); e != nil {
				return p, e
			}
			for name, typName := range attrs {
				raw := row[cols[name]]
				if raw == "" {
					continue
				}
				t, v, e := parseTypedCSV(raw, typName)
				if e != nil {
					return p, fmt.Errorf("%s.%s: %w", typ, name, e)
				}
				p.Assignments = append(p.Assignments, Attribute{ObjectID: id, Name: name, Time: epoch, Type: t, Value: v})
			}
		}
		changes, changeCols, e := readTable(root, d.ChangesFile, []string{"ocel_id", "ocel_time", "ocel_changed_field"})
		if e != nil {
			return p, e
		}
		for name := range attrs {
			if _, ok := changeCols[name]; !ok {
				return p, fmt.Errorf("object-change table %s lacks declared attribute %s", d.ChangesFile, name)
			}
		}
		for _, row := range changes[1:] {
			id, tm, name := row[changeCols["ocel_id"]], row[changeCols["ocel_time"]], row[changeCols["ocel_changed_field"]]
			if seen[id] != typ || tm == "" {
				return p, errors.New("invalid object-change row or object type")
			}
			typName, ok := attrs[name]
			if !ok {
				return p, fmt.Errorf("undeclared object attribute %s.%s", typ, name)
			}
			raw := row[changeCols[name]]
			if raw == "" {
				return p, errors.New("changed attribute value is missing")
			}
			t, v, e := parseTypedCSV(raw, typName)
			if e != nil {
				return p, fmt.Errorf("%s.%s: %w", typ, name, e)
			}
			p.Assignments = append(p.Assignments, Attribute{ObjectID: id, Name: name, Time: tm, Type: t, Value: v})
		}
	}

	for kind, required := range map[string][]string{
		"e2o": {"ocel_event_id", "ocel_object_id", "ocel_qualifier"},
		"o2o": {"ocel_source_id", "ocel_target_id", "ocel_qualifier"},
	} {
		rel := m.Relations[kind]
		if rel == "" {
			return p, fmt.Errorf("missing %s relation table", kind)
		}
		rows, cols, e := readTable(root, rel, required)
		if e != nil {
			return p, e
		}
		for _, row := range rows[1:] {
			if kind == "e2o" {
				p.E2O = append(p.E2O, Relation{Source: row[cols["ocel_event_id"]], Target: row[cols["ocel_object_id"]], Qualifier: row[cols["ocel_qualifier"]]})
			} else {
				p.O2O = append(p.O2O, Relation{Source: row[cols["ocel_source_id"]], Target: row[cols["ocel_target_id"]], Qualifier: row[cols["ocel_qualifier"]]})
			}
		}
	}
	return finish(p)
}
