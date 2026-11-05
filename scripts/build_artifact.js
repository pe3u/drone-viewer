// Build a self-contained single-file viewer for the claude.ai Artifact host:
// strips the document wrapper, inlines app.js and embeds both LOD GLBs (uncompressed) as base64.
// usage: node build_artifact.js
const fs = require("fs"), path = require("path");
const ROOT = path.join(__dirname, "..");
const web = path.join(ROOT, "web"), out = path.join(ROOT, "artifact");

let html = fs.readFileSync(path.join(web, "index.html"), "utf8");
const app = fs.readFileSync(path.join(web, "app.js"), "utf8");
const airflow = fs.readFileSync(path.join(web, "airflow.js"), "utf8");
const b64 = (f) => fs.readFileSync(path.join(out, "raw", f)).toString("base64");

html = html
  .replace(/<!doctype html>\s*/i, "")
  .replace(/<html[^>]*>\s*/i, "")
  .replace(/<\/html>\s*/i, "")
  .replace(/<head>\s*/i, "")
  .replace(/<\/head>\s*/i, "")
  .replace(/<body>\s*/i, "")
  .replace(/<\/body>\s*/i, "")
  .replace(/<meta charset="utf-8">\s*/i, "")
  .replace(/<meta name="viewport"[^>]*>\s*/i, "");

const embed = `<script>window.DRONE_EMBED = { hd: "${b64("drone.glb")}", lite: "${b64("drone_lite.glb")}" };</script>\n`;
html = html.replace('<script src="./airflow.js"></script>', () => "<script>" + airflow + "</script>");
html = html.replace('<script type="module" src="./app.js"></script>', () => embed + "<script type=\"module\">\n" + app + "\n</script>");
if (!html.includes("window.DRONE_EMBED")) throw new Error("app.js script tag not found");
fs.writeFileSync(path.join(out, "survey-drone.html"), html);
console.log("artifact page", (Buffer.byteLength(html) / 1048576).toFixed(2), "MB; title in first 8KB:", html.slice(0, 8192).includes("<title>"));
