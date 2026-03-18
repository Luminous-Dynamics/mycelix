import { o as onMount } from "./ssr2.js";
import "./exports.js";
import "@sveltejs/kit/internal/server";
const is_legacy = onMount.toString().includes("$$") || /function \w+\(\) \{\}/.test(onMount.toString());
const placeholder_url = "a:";
if (is_legacy) {
  ({
    data: {},
    form: null,
    error: null,
    params: {},
    route: { id: null },
    state: {},
    status: -1,
    url: new URL(placeholder_url)
  });
}
