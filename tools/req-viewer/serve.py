#!/usr/bin/env python3
"""
serve.py — local web server for the requirement viewer.

Builds the trace index (see build_index.py), then serves four things from one origin so the
app can reach into the spec iframe directly:

    /                   the app
    /api/index.json     the trace index
    /api/rebuild        rebuild the index in place (returns the new stats)
    /docs/<name>.html   instrumented article bodies, one per spec page
    /sphinx/...         the Sphinx build tree, for _static and _images

Nothing is written except the index under --out. Binds to localhost unless told otherwise.

Usage:
    serve.py [--port 8765] [--host 127.0.0.1] [--no-build] [--open]
"""

from __future__ import print_function

import argparse
import io
import json
import os
import posixpath
import sys
import threading

try:
    from http.server import SimpleHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn
    from urllib.parse import urlparse, unquote
except ImportError:  # pragma: no cover - py2
    from SimpleHTTPServer import SimpleHTTPRequestHandler
    from BaseHTTPServer import HTTPServer
    from SocketServer import ThreadingMixIn
    from urlparse import urlparse
    from urllib import unquote

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build_index  # noqa: E402


MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
    ".ttf": "font/ttf",
    ".eot": "application/vnd.ms-fontobject",
    ".txt": "text/plain; charset=utf-8",
    ".map": "application/json; charset=utf-8",
    ".inv": "application/octet-stream",
}


class Config(object):
    def __init__(self, repo_root, html_dir, reqs_dir, rtl_dir, out_dir):
        self.repo_root = repo_root
        self.html_dir = html_dir
        self.reqs_dir = reqs_dir
        self.rtl_dir = rtl_dir
        self.out_dir = out_dir
        self.sphinx_root = os.path.dirname(html_dir.rstrip(os.sep))
        self.app_dir = os.path.join(HERE, "app")
        self.lock = threading.Lock()

    def rebuild(self, quiet=True):
        with self.lock:
            index, _ = build_index.build(self.repo_root, self.html_dir, self.reqs_dir,
                                         self.rtl_dir, self.out_dir, quiet=quiet)
            return index["stats"]


class Handler(SimpleHTTPRequestHandler):
    config = None
    server_version = "req-viewer"

    # -- routing -------------------------------------------------------------
    def do_GET(self):
        path = urlparse(self.path).path
        try:
            self.route(path)
        except BrokenPipeError:  # pragma: no cover - browser navigated away
            pass

    do_POST = do_GET

    def route(self, path):
        cfg = self.config
        if path in ("/", "/index.html"):
            return self.send_file(os.path.join(cfg.app_dir, "index.html"))
        if path == "/api/index.json":
            return self.send_file(os.path.join(cfg.out_dir, "index.json"))
        if path == "/api/rebuild":
            try:
                stats = cfg.rebuild()
            except SystemExit as exc:
                return self.send_json({"error": "index build failed (%s)" % exc}, 500)
            except Exception as exc:  # surfaced in the UI, not the terminal
                return self.send_json({"error": "%s: %s" % (type(exc).__name__, exc)}, 500)
            return self.send_json({"stats": stats})

        for prefix, root in (("/app/", cfg.app_dir),
                            ("/docs/", os.path.join(cfg.out_dir, "docs")),
                            ("/sphinx/", cfg.sphinx_root)):
            if path.startswith(prefix):
                target = self.resolve(root, path[len(prefix):])
                if target is None:
                    return self.send_error(403, "path escapes the served root")
                return self.send_file(target)
        return self.send_error(404, "no route for %s" % path)

    # -- plumbing ------------------------------------------------------------
    @staticmethod
    def resolve(root, rel):
        rel = unquote(rel)
        target = os.path.normpath(os.path.join(root, rel.lstrip("/")))
        root_abs = os.path.abspath(root)
        if not os.path.abspath(target).startswith(root_abs + os.sep):
            return None
        return target

    def send_file(self, target):
        if not os.path.isfile(target):
            return self.send_error(404, "not found: %s" % posixpath.basename(target))
        ctype = MIME.get(os.path.splitext(target)[1].lower(), "application/octet-stream")
        with open(target, "rb") as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        if os.environ.get("REQ_VIEWER_VERBOSE"):
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    default_root = os.path.abspath(os.path.join(HERE, "..", ".."))
    ap = argparse.ArgumentParser(description="Serve the requirement viewer.")
    ap.add_argument("--repo-root", default=default_root)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--html-dir", default=None)
    ap.add_argument("--reqs-dir", default=None)
    ap.add_argument("--rtl-dir", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-build", action="store_true",
                    help="serve the existing index instead of rebuilding at startup")
    args = ap.parse_args()

    root = os.path.abspath(args.repo_root)
    cfg = Config(
        root,
        args.html_dir or os.path.join(root, "docs_caracal", "_build", "html", "src"),
        args.reqs_dir or os.path.join(root, "src", "main", "nlhdl", "reqs"),
        args.rtl_dir or os.path.join(root, "src", "main", "nlhdl"),
        args.out or os.path.join(root, "docs_caracal", "_build", "reqviewer"),
    )

    if args.no_build:
        if not os.path.isfile(os.path.join(cfg.out_dir, "index.json")):
            sys.stderr.write("error: no index at %s — drop --no-build\n" % cfg.out_dir)
            return 2
    else:
        cfg.rebuild(quiet=False)

    Handler.config = cfg
    httpd = Server((args.host, args.port), Handler)
    url = "http://%s:%d/" % ("localhost" if args.host == "127.0.0.1" else args.host,
                             args.port)
    print("\nrequirement viewer: %s" % url)
    print("  spec  <- %s" % os.path.relpath(cfg.html_dir, root))
    print("  reqs  <- %s" % os.path.relpath(cfg.reqs_dir, root))
    print("  rtl   <- %s (scanned for //@req- tags)" % os.path.relpath(cfg.rtl_dir, root))
    print("\nCtrl-C to stop. After 'make html' in docs_caracal, hit Rebuild in the toolbar.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
