#!/usr/bin/env python3
"""Dev server for Waypoint Studio that disables browser caching.

`python3 -m http.server` sends no Cache-Control/Expires/ETag headers, so browsers
apply heuristic caching and can keep serving a STALE `waypoint_studio.html`
after edits — the page then behaves like the old version (e.g. exports ignore the
Body-height Z offset). This wrapper adds `no-store` headers so a normal reload
always fetches the latest file.

Usage (from anywhere):
    python3 tools/serve_nocache.py [port]      # default port 8765
Then open:
    http://127.0.0.1:8765/tools/waypoint_studio.html

Serves the repository root (the parent of this file's directory).
"""
import functools
import http.server
import json
import socketserver
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8765


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler that forbids caching of every response."""

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def _send_json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        """Accept {path, content} and write it into the repo (yaml/yml/csv only).

        Lets the browser tool save a waypoints file straight to disk instead of
        using a Blob download, which hangs on a native save dialog in embedded
        preview browsers. Confined to ROOT; rejects traversal and other types.
        """
        if self.path.split("?", 1)[0].rstrip("/") != "/__save":
            self._send_json(404, {"ok": False, "error": "unknown endpoint"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            rel = str(payload.get("path", "")).strip().lstrip("/")
            content = payload.get("content", "")
            if not rel or not isinstance(content, str):
                raise ValueError("both 'path' and string 'content' are required")
            if not rel.lower().endswith((".yaml", ".yml", ".csv")):
                raise ValueError("only .yaml/.yml/.csv paths are allowed")
            target = (ROOT / rel).resolve()
            if target == ROOT or ROOT not in target.parents:
                raise ValueError("path escapes the repo root")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content, encoding="utf-8")
            nbytes = len(content.encode("utf-8"))
            print(f"saved {rel} ({nbytes} bytes)", flush=True)
            self._send_json(200, {"ok": True, "path": str(target), "bytes": nbytes})
        except Exception as exc:  # noqa: BLE001 - report any failure to the client
            self._send_json(400, {"ok": False, "error": str(exc)})


class NoCacheServer(socketserver.ThreadingTCPServer):
    """Threaded so a big .pcd fetch never blocks other concurrent requests."""

    allow_reuse_address = True
    daemon_threads = True


def main():
    handler = functools.partial(NoCacheHandler, directory=str(ROOT))
    with NoCacheServer(("127.0.0.1", PORT), handler) as httpd:
        print(f"Serving {ROOT} at http://127.0.0.1:{PORT}/ (no-cache, threaded)", flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nstopped", flush=True)


if __name__ == "__main__":
    main()
