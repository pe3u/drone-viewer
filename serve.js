// Tiny static server for the drone viewer (ES modules and the GLB need http, not file://).
const http = require("http"), fs = require("fs"), path = require("path");
const root = __dirname, port = +process.env.PORT || 8370;
const types = { ".html": "text/html; charset=utf-8", ".js": "text/javascript", ".css": "text/css", ".json": "application/json", ".png": "image/png", ".jpg": "image/jpeg", ".glb": "model/gltf-binary", ".svg": "image/svg+xml" };
http.createServer((req, res) => {
  let p = decodeURIComponent(req.url.split("?")[0]); if (p === "/") p = "/index.html";
  const f = path.join(root, path.normalize(p));
  if (!f.startsWith(root)) { res.writeHead(403); return res.end(); }
  fs.stat(f, (e, st) => {
    if (e || !st.isFile()) { res.writeHead(404); return res.end("not found"); }
    res.writeHead(200, { "Content-Type": types[path.extname(f).toLowerCase()] || "application/octet-stream", "Content-Length": st.size, "Cache-Control": "no-cache" });
    fs.createReadStream(f).pipe(res);
  });
}).listen(port, "127.0.0.1", () => console.log("Survey drone viewer: http://127.0.0.1:" + port + "/"));
