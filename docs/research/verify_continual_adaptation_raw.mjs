#!/usr/bin/env node
import fs from "node:fs";

function skip(s, i) {
  while (i < s.length && /\s/.test(s[i])) i++;
  return i;
}

function stringEnd(s, i) {
  if (s[i] !== '"') throw new Error("string expected");
  i++;
  while (i < s.length) {
    if (s[i] === '"') return i + 1;
    if (s[i] === '\\') {
      i++;
      if (i >= s.length) throw new Error("bad escape");
      if (s[i] === 'u') {
        if (!/^[0-9A-Fa-f]{4}$/.test(s.slice(i + 1, i + 5))) throw new Error("bad unicode escape");
        i += 5;
      } else if ('"\\/bfnrt'.includes(s[i])) {
        i++;
      } else {
        throw new Error("bad escape");
      }
    } else {
      if (s.charCodeAt(i) < 0x20) throw new Error("control char");
      i++;
    }
  }
  throw new Error("unterminated string");
}

function parseString(s, i) {
  const j = stringEnd(s, i);
  return [JSON.parse(s.slice(i, j)), j];
}

function value(s, i) {
  i = skip(s, i);
  if (i >= s.length) throw new Error("value expected");
  if (s[i] === '"') return stringEnd(s, i);
  if (s[i] === '{') return object(s, i);
  if (s[i] === '[') return array(s, i);
  if (s.startsWith("true", i)) return i + 4;
  if (s.startsWith("false", i)) return i + 5;
  if (s.startsWith("null", i)) return i + 4;
  const match = s.slice(i).match(/^-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?/);
  if (match) {
    if (match[0] === "-0") throw new Error("negative zero");
    return i + match[0].length;
  }
  throw new Error("invalid value");
}

function object(s, i) {
  i = skip(s, i + 1);
  const keys = new Set();
  if (s[i] === '}') return i + 1;
  while (true) {
    i = skip(s, i);
    const [key, j] = parseString(s, i);
    i = skip(s, j);
    if (keys.has(key)) throw new Error("duplicate key");
    keys.add(key);
    if (s[i] !== ':') throw new Error("colon expected");
    i = skip(s, i + 1);
    i = value(s, i);
    i = skip(s, i);
    if (s[i] === '}') return i + 1;
    if (s[i] !== ',') throw new Error("comma expected");
    i = skip(s, i + 1);
  }
}

function array(s, i) {
  i = skip(s, i + 1);
  if (s[i] === ']') return i + 1;
  while (true) {
    i = value(s, i);
    i = skip(s, i);
    if (s[i] === ']') return i + 1;
    if (s[i] !== ',') throw new Error("comma expected");
    i = skip(s, i + 1);
  }
}

function accept(raw) {
  let i = value(raw, 0);
  i = skip(raw, i);
  if (i !== raw.length) throw new Error("trailing input");
  JSON.parse(raw);
}

const path = process.argv[2];
if (!path) {
  console.error("usage: verify_continual_adaptation_raw.mjs CORPUS.json");
  process.exit(2);
}

const corpus = JSON.parse(fs.readFileSync(path, "utf8"));
const failures = [];

for (const testCase of corpus.cases) {
  let accepted = true;
  try {
    accept(testCase.raw);
  } catch {
    accepted = false;
  }
  const actual = accepted ? "accept" : "reject";
  if (actual !== testCase.expected) {
    failures.push([testCase.case_id, testCase.expected, actual]);
  }
}

console.log("cases=" + corpus.cases.length + " failures=" + failures.length);
for (const failure of failures) console.log("FAIL", failure);
process.exit(failures.length ? 1 : 0);
