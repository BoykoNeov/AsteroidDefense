#!/usr/bin/env python3
"""Bake the keyhole map JSON into the interactive HTML page.

The template (`tools/keyhole_map_template.html`) is a self-contained page with
one placeholder; this inlines `docs/keyhole_map.json` so the result opens from a
file, a static host, or an Artifact with no fetch.

Usage::

    python tools/keyhole_map_html.py docs/keyhole_map.json docs/keyhole_map.html
"""

from __future__ import annotations

import json
import pathlib
import sys


def main() -> None:
    src = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "docs/keyhole_map.json")
    dst = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else "docs/keyhole_map.html")
    template = pathlib.Path(__file__).with_name("keyhole_map_template.html")
    data = json.loads(src.read_text(encoding="utf-8"))
    # Compact, and `</` escaped so the JSON can never close the script element.
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    html = template.read_text(encoding="utf-8").replace("__KEYHOLE_JSON__", payload, 1)
    dst.write_text(html, encoding="utf-8")
    print(f"wrote {dst} ({len(html)} bytes)")


if __name__ == "__main__":
    main()
