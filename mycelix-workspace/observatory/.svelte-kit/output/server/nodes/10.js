

export const index = 10;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/network/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/10.ofXYaX5F.js","_app/immutable/chunks/DMvb8rdj.js","_app/immutable/chunks/Ct2uKrlJ.js","_app/immutable/chunks/Bwzdl04Z.js","_app/immutable/chunks/CbCG2bqt.js"];
export const stylesheets = ["_app/immutable/assets/10.5gpOXjRl.css"];
export const fonts = [];
