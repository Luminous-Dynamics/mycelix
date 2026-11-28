package oracle

import (
	"encoding/csv"
	"encoding/json"
	"io/fs"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

func root(t *testing.T) string {
	t.Helper()
	if p := os.Getenv("CRAFT_OCEL_FIXTURE_ROOT"); p != "" {
		return p
	}
	return filepath.Join("..", "..", "..", "fixtures", "ocel21-interchange")
}
func read(t *testing.T, n string) []byte {
	t.Helper()
	b, e := os.ReadFile(filepath.Join(root(t), n))
	if e != nil {
		t.Fatal(e)
	}
	return b
}
func counts(t *testing.T, p Projection) {
	t.Helper()
	if len(p.Events) != 2 || len(p.Objects) != 5 || len(p.Assignments) != 6 || len(p.E2O) != 5 || len(p.O2O) != 2 {
		t.Fatalf("wrong counts: %#v", p)
	}
}
func TestFormatParity(t *testing.T) {
	j, e := ParseJSON(read(t, "ocel21-mini.json"))
	if e != nil { t.Fatal(e) }
	s, e := ParseSingleCSV(read(t, "ocel21-mini.ocel.csv"))
	if e != nil { t.Fatal(e) }
	b, e := ParseBundledCSV(filepath.Join(root(t), "bundled-csv"))
	if e != nil { t.Fatal(e) }
	counts(t, j)
	counts(t, s)
	counts(t, b)
	if !reflect.DeepEqual(j, s) { t.Fatalf("single CSV differs from JSON\n%#v\n%#v", j, s) }
	if !reflect.DeepEqual(j, b) { t.Fatalf("bundled CSV differs from JSON\n%#v\n%#v", j, b) }
}
func TestQualifiersAndHistory(t *testing.T) {
	p, e := ParseJSON(read(t, "ocel21-mini.json"))
	if e != nil { t.Fatal(e) }
	if p.E2O[0].Qualifier == "" || p.O2O[0].Qualifier == "" { t.Fatal("relationship qualifier lost") }
	var attrs []Attribute
	for _, a := range p.Assignments {
		if a.ObjectID == "P-17" && a.Name == "status" { attrs = append(attrs, a) }
	}
	if len(attrs) != 2 || attrs[0].Time != epoch || attrs[1].Time != "2026-11-28T08:10:00Z" {
		t.Fatalf("attribute history changed: %#v", attrs)
	}
}
func TestMutationSensitivity(t *testing.T) {
	base, e := ParseJSON(read(t, "ocel21-mini.json"))
	if e != nil { t.Fatal(e) }
	cases := map[string]func(*Projection){
		"drop_e2o": func(p *Projection) { p.E2O = p.E2O[1:] },
		"drop_o2o": func(p *Projection) { p.O2O = p.O2O[1:] },
		"collapse_history": func(p *Projection) {
			out := p.Assignments[:0]
			for _, a := range p.Assignments {
				if !(a.ObjectID == "P-17" && a.Name == "status" && a.Time == epoch) { out = append(out, a) }
			}
			p.Assignments = out
		},
	}
	for n, mut := range cases {
		t.Run(n, func(t *testing.T) {
			p := base
			p.Events = append([]Event(nil), base.Events...)
			p.Objects = append([]Object(nil), base.Objects...)
			p.Assignments = append([]Attribute(nil), base.Assignments...)
			p.E2O = append([]Relation(nil), base.E2O...)
			p.O2O = append([]Relation(nil), base.O2O...)
			mut(&p)
			if reflect.DeepEqual(base, p) { t.Fatal("mutation not detected") }
		})
	}
}
func TestRejectCrossTypeIDReuse(t *testing.T) {
	s := strings.Replace(string(read(t, "ocel21-mini.ocel.csv")), "Z-3#sampled_zone", "P-17#sampled_zone", 1)
	if _, e := ParseSingleCSV([]byte(s)); e == nil { t.Fatal("cross-type object ID accepted") }
}
func copyFixtureTree(t *testing.T) string {
	t.Helper()
	dst := t.TempDir()
	if err := filepath.WalkDir(root(t), func(path string, d fs.DirEntry, err error) error {
		if err != nil { return err }
		rel, err := filepath.Rel(root(t), path)
		if err != nil { return err }
		out := filepath.Join(dst, rel)
		if d.IsDir() { return os.MkdirAll(out, 0o755) }
		b, err := os.ReadFile(path)
		if err != nil { return err }
		return os.WriteFile(out, b, 0o644)
	}); err != nil { t.Fatal(err) }
	return dst
}

func TestBundledCSVColumnOrderIndependent(t *testing.T) {
	dst := copyFixtureTree(t)
	path := filepath.Join(dst, "bundled-csv", "events", "event_pressure_observed.csv")
	b, err := os.ReadFile(path)
	if err != nil { t.Fatal(err) }
	r := csv.NewReader(strings.NewReader(string(b)))
	rows, err := r.ReadAll()
	if err != nil { t.Fatal(err) }
	for i := range rows { rows[i][0], rows[i][1], rows[i][2] = rows[i][2], rows[i][0], rows[i][1] }
	var out strings.Builder
	w := csv.NewWriter(&out)
	if err := w.WriteAll(rows); err != nil { t.Fatal(err) }
	w.Flush()
	if err := os.WriteFile(path, []byte(out.String()), 0o644); err != nil { t.Fatal(err) }
	got, err := ParseBundledCSV(filepath.Join(dst, "bundled-csv"))
	if err != nil { t.Fatal(err) }
	want, err := ParseJSON(read(t, "ocel21-mini.json"))
	if err != nil { t.Fatal(err) }
	if !reflect.DeepEqual(got, want) { t.Fatalf("column reordering changed semantics\n got: %#v\nwant: %#v", got, want) }
}

func TestRejectBundlePathTraversal(t *testing.T) {
	dst := copyFixtureTree(t)
	path := filepath.Join(dst, "bundled-csv", "ocel-meta.json")
	b, err := os.ReadFile(path)
	if err != nil { t.Fatal(err) }
	var meta map[string]any
	if err := json.Unmarshal(b, &meta); err != nil { t.Fatal(err) }
	events := meta["eventTypes"].(map[string]any)
	def := events["pressure_observed"].(map[string]any)
	def["file"] = "../../outside.csv"
	b, err = json.Marshal(meta)
	if err != nil { t.Fatal(err) }
	if err := os.WriteFile(path, b, 0o644); err != nil { t.Fatal(err) }
	if _, err := ParseBundledCSV(filepath.Join(dst, "bundled-csv")); err == nil { t.Fatal("path traversal in metadata accepted") }
}

func TestRejectMapShapedBundleAttributeMetadata(t *testing.T) {
	dst := copyFixtureTree(t)
	path := filepath.Join(dst, "bundled-csv", "ocel-meta.json")
	b, err := os.ReadFile(path)
	if err != nil { t.Fatal(err) }
	var meta map[string]any
	if err := json.Unmarshal(b, &meta); err != nil { t.Fatal(err) }
	events := meta["eventTypes"].(map[string]any)
	def := events["pressure_observed"].(map[string]any)
	def["attributes"] = map[string]string{"pressure_kpa": "integer"}
	b, err = json.Marshal(meta)
	if err != nil { t.Fatal(err) }
	if err := os.WriteFile(path, b, 0o644); err != nil { t.Fatal(err) }
	if _, err := ParseBundledCSV(filepath.Join(dst, "bundled-csv")); err == nil { t.Fatal("non-standard map-shaped attribute metadata accepted") }
}

func TestSingleCSVReservedCharacterEscapes(t *testing.T) {
	r, err := parseRefs("a\\/b#in\\#box/c\\{d#literal\\/slash")
	if err != nil { t.Fatal(err) }
	if len(r) != 2 || r[0].ID != "a/b" || r[0].Qualifier != "in#box" || r[1].ID != "c{d" || r[1].Qualifier != "literal/slash" {
		t.Fatalf("unexpected escaped refs: %#v", r)
	}
}

func TestSingleCSVRejectsInvalidDeclarationAndO2OAttributeRows(t *testing.T) {
	base := string(read(t, "ocel21-mini.ocel.csv"))
	badDeclaration := strings.Replace(base, "\"P-17{\"\"status\"\":\"\"active\"\"}", "\"P-17#bad{\"\"status\"\":\"\"active\"\"}", 1)
	if _, err := ParseSingleCSV([]byte(badDeclaration)); err == nil { t.Fatal("qualifier in object declaration accepted") }
	badO2O := strings.Replace(base, "CAL-4,o2o,,P-17#calibrates", "CAL-4,o2o,,P-17#calibrates{\"\"status\"\":\"\"active\"\"}", 1)
	if _, err := ParseSingleCSV([]byte(badO2O)); err == nil { t.Fatal("O2O JSON attributes without timestamp accepted") }
}

func TestSingleCSVCanonicalNumericLexicalForm(t *testing.T) {
	for _, raw := range []string{"007", "+5", "1.50"} {
		typ, value, err := csvValue(raw, "")
		if err != nil { t.Fatal(err) }
		if typ != "string" || value != raw { t.Fatalf("non-canonical numeric %q retyped as %s:%q", raw, typ, value) }
	}
}

func TestSingleCSVRejectsConflictingSameTimeAssignments(t *testing.T) {
	base := string(read(t, "ocel21-mini.ocel.csv"))
	line := ",,2026-11-28T08:10:00Z,\"P-17{\"\"status\"\":\"\"conflicting\"\"}\",,,,,,\n"
	mutated := strings.Replace(base, "CAL-4,o2o", line+"CAL-4,o2o", 1)
	if _, err := ParseSingleCSV([]byte(mutated)); err == nil { t.Fatal("conflicting same-time object assignments accepted") }
}

func TestObjectReferenceDelimiterHandling(t *testing.T) {
	r, e := parseRefs("A#uses{\"path\":\"north/south\"}/B#target")
	if e != nil { t.Fatal(e) }
	if len(r) != 2 || r[0].Attributes["path"] != "north/south" || r[1].ID != "B" { t.Fatalf("bad refs: %#v", r) }
}
func TestRejectRaggedCSV(t *testing.T) {
	s := string(read(t, "ocel21-mini.ocel.csv"))
	s = strings.Replace(s, "OCEL-E1,pressure_observed,2026-11-28T08:00:00Z,P-17#measured_asset,Z-3#served_zone,,CAL-4#calibration_basis,,410,\n", "OCEL-E1,pressure_observed,2026-11-28T08:00:00Z,P-17#measured_asset\n", 1)
	if _, e := ParseSingleCSV([]byte(s)); e == nil { t.Fatal("ragged CSV accepted") }
}

func TestSingleCSVUnifiesAttributeTypeAcrossScope(t *testing.T) {
	base := string(read(t, "ocel21-mini.ocel.csv"))
	row := "E3,pressure_observed,2026-11-28T08:11:00Z,P-17#measured_asset,,,,,007,\n"
	parsed, err := ParseSingleCSV([]byte(base + row))
	if err != nil { t.Fatal(err) }
	var got []Attribute
	for _, event := range parsed.Events {
		for _, attr := range event.Attributes { if attr.Name == "pressure_kpa" { got = append(got, attr) } }
	}
	if len(got) != 2 { t.Fatalf("expected two values in same attribute scope: %#v", got) }
	for _, attr := range got { if attr.Type != "string" { t.Fatalf("scope should unify to string, got %#v", got) } }
}
