#!/usr/bin/env python3
"""Receives telemetry lines from the home controllers and appends them to /var/www/html/home/<name>.txt.

POST /home/api/ingest/<name> with "Authorization: Bearer <token>" and a text/plain body of lines,
each either a '#' comment or a data line starting with YYYY-MM-DD.  This replaces the gobble scripts
on homecontrol, which appended the same lines over ssh.

Lines containing "alert" are sent to Pushover, as gobble did, and a controller that has been heard
from but goes quiet for SILENT_MINUTES is reported once.  Pushover is skipped if its credentials
file is absent.

Files (passed by systemd as credentials):
  tokens:   one "<name> <token>" per line
  pushover: "app_token <token>" and "user_token <token>" lines (optional)
"""

import fcntl
import hmac
import json
import os
import re
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DATA_DIR = "/var/www/html/home"
STATE_DIR = os.environ.get("STATE_DIRECTORY", "/var/lib/home-ingest")
PORT = int(os.environ.get("INGEST_PORT", "5002"))
SILENT_MINUTES = 30
MAX_BODY = 256 * 1024
NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,40}$")
LINE_RE = re.compile(r"^(#.*|\d{4}-\d{2}-\d{2} .*)$")


def read_pairs(env):
    path = os.environ.get(env)
    pairs = {}
    if path and os.path.exists(path):
        for line in open(path):
            parts = line.split()
            if len(parts) == 2 and not parts[0].startswith("#"):
                pairs[parts[0]] = parts[1]
    return pairs


TOKENS = read_pairs("INGEST_TOKENS")
PUSHOVER = read_pairs("INGEST_PUSHOVER")


def pushover(message):
    if "app_token" not in PUSHOVER or "user_token" not in PUSHOVER:
        return
    data = urllib.parse.urlencode({"token": PUSHOVER["app_token"], "user": PUSHOVER["user_token"],
                                   "message": message[:1000]}).encode()
    try:
        urllib.request.urlopen("https://api.pushover.net/1/messages.json", data, timeout=10).read()
    except OSError as e:
        print(f"pushover failed: {e}", flush=True)


def seen_path(name):
    return os.path.join(STATE_DIR, f"{name}.json")


def mark_seen(name):
    with open(seen_path(name), "w") as f:
        json.dump({"last": time.time(), "silent_reported": False}, f)


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, text):
        body = (text + "\n").encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        prefix = "/home/api/ingest/"
        if not self.path.startswith(prefix):
            return self.reply(404, "not found")
        name = self.path[len(prefix):]
        token = TOKENS.get(name)
        auth = self.headers.get("Authorization", "")
        if not NAME_RE.match(name) or token is None or \
                not hmac.compare_digest(auth.encode(), f"Bearer {token}".encode()):
            return self.reply(403, "forbidden")
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > MAX_BODY:
            return self.reply(413, "bad length")
        text = self.rfile.read(length).decode("utf-8", errors="replace")
        lines = [l.rstrip("\r") for l in text.split("\n") if l.strip()]
        good = [l for l in lines if LINE_RE.match(l) and len(l) < 400]
        if good:
            with open(os.path.join(DATA_DIR, f"{name}.txt"), "a") as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                f.write("\n".join(good) + "\n")
        mark_seen(name)
        for line in good:
            if "alert" in line.lower():
                threading.Thread(target=pushover, args=(f"{name}: {line}",), daemon=True).start()
        self.reply(200, f"ok {len(good)} of {len(lines)}")

    def log_message(self, fmt, *args):
        pass


def watch_silence():
    while True:
        time.sleep(60)
        for name in TOKENS:
            try:
                with open(seen_path(name)) as f:
                    state = json.load(f)
            except (OSError, ValueError):
                continue
            quiet = time.time() - state["last"]
            if quiet > SILENT_MINUTES * 60 and not state["silent_reported"]:
                pushover(f"{name}: alert no data for {int(quiet // 60)} minutes")
                state["silent_reported"] = True
                with open(seen_path(name), "w") as f:
                    json.dump(state, f)


if __name__ == "__main__":
    threading.Thread(target=watch_silence, daemon=True).start()
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
