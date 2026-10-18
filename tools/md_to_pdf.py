#!/usr/bin/env python3
"""
md_to_pdf.py -- Markdown → PDF for the docs/A1_x documents, styled like the A1.3 PDF.

Supports the Markdown used in these documents: headings, paragraphs, **bold**, *italic*,
`code`, fenced code blocks, lists, tables, images, links, block quotes, horizontal rules.
The HTML is printed to PDF by Firefox through its WebDriver (geckodriver); no other software
is needed.

    python3 tools/md_to_pdf.py docs/A1_1/A1_1_mapa_a_planovanie.md [more.md ...]
"""

import base64
import html
import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.request

PORT = 4455
CSS = """
@page { size: A4; margin: 18mm 17mm; }
body { font-family: "Ubuntu", "DejaVu Sans", sans-serif; font-size: 10.5pt; line-height: 1.55; color: #24292e; }
h1 { font-weight: 400; font-size: 21pt; border-bottom: 1px solid #d0d7de; padding-bottom: 6px; margin: 0 0 14px; }
h2 { font-weight: 400; font-size: 15.5pt; margin: 22px 0 8px; }
h3 { font-weight: 400; font-size: 12.5pt; margin: 18px 0 6px; }
h4 { font-weight: 600; font-size: 11pt; margin: 14px 0 4px; }
p { margin: 6px 0 9px; }
code { font-family: "DejaVu Sans Mono", monospace; font-size: 9pt; color: #e7663a; }
pre { background: #f6f8fa; border: 1px solid #e1e4e8; border-radius: 4px; padding: 8px 10px; font-size: 8.8pt;
      line-height: 1.4; white-space: pre-wrap; break-inside: avoid; }
pre code { color: #24292e; }
table { border-collapse: collapse; margin: 8px 0 12px; font-size: 9.5pt; break-inside: avoid; }
th { text-align: left; font-weight: 600; border-bottom: 2px solid #d0d7de; padding: 5px 10px; }
td { border-bottom: 1px solid #e1e4e8; padding: 5px 10px; vertical-align: top; }
img { max-width: 100%; max-height: 23cm; display: block; margin: 0 auto; }
figure { margin: 8px 0 12px; break-inside: avoid; page-break-inside: avoid; }
ul, ol { margin: 4px 0 10px; padding-left: 22px; }
li { margin: 2px 0; }
blockquote { margin: 8px 0; padding: 2px 12px; border-left: 4px solid #d0d7de; color: #57606a; }
hr { border: 0; border-top: 1px solid #d0d7de; margin: 16px 0; }
a { color: #0969da; text-decoration: none; }
"""


def inline(text):
    """Inline Markdown → HTML (code spans are protected from the other rules)."""
    codes = []

    def keep(m):
        codes.append(html.escape(m.group(1)))
        return f"\x00{len(codes) - 1}\x00"
    text = re.sub(r"`([^`]+)`", keep, text)
    text = html.escape(text, quote=False)
    text = re.sub(r"!\[([^\]]*)\]\(([^)]+)\)", r'<img alt="\1" src="\2">', text)
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', text)
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
    text = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<i>\1</i>", text)
    return re.sub(r"\x00(\d+)\x00", lambda m: f"<code>{codes[int(m.group(1))]}</code>", text)


def to_html(md):
    lines = md.split("\n")
    out, i = [], 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("```"):
            j = i + 1
            while j < len(lines) and not lines[j].startswith("```"):
                j += 1
            out.append("<pre><code>" + html.escape("\n".join(lines[i + 1:j])) + "</code></pre>")
            i = j + 1
            continue
        m = re.match(r"(#{1,4})\s+(.*)", line)
        if m:
            n = len(m.group(1))
            out.append(f"<h{n}>{inline(m.group(2))}</h{n}>")
            i += 1
            continue
        if re.match(r"^\s*(---+|\*\*\*+)\s*$", line):
            out.append("<hr>")
            i += 1
            continue
        if line.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                rows.append(lines[i])
                i += 1
            cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
            aligns = ["right" if re.match(r"^-+:$", c) else "left" for c in cells[1]] if len(cells) > 1 else []
            t = ["<table><thead><tr>" + "".join(f"<th>{inline(c)}</th>" for c in cells[0]) + "</tr></thead><tbody>"]
            for r in cells[2:]:
                t.append("<tr>" + "".join(
                    f'<td style="text-align:{aligns[k] if k < len(aligns) else "left"}">{inline(c)}</td>'
                    for k, c in enumerate(r)) + "</tr>")
            out.append("".join(t) + "</tbody></table>")
            continue
        if re.match(r"^\s*([-*]|\d+\.)\s+", line):
            ordered = bool(re.match(r"^\s*\d+\.", line))
            items = []
            while i < len(lines) and (re.match(r"^\s*([-*]|\d+\.)\s+", lines[i]) or
                                      (lines[i].startswith("  ") and items)):
                if re.match(r"^\s*([-*]|\d+\.)\s+", lines[i]):
                    items.append(re.sub(r"^\s*([-*]|\d+\.)\s+", "", lines[i]))
                else:
                    items[-1] += " " + lines[i].strip()
                i += 1
            tag = "ol" if ordered else "ul"
            out.append(f"<{tag}>" + "".join(f"<li>{inline(x)}</li>" for x in items) + f"</{tag}>")
            continue
        if line.startswith(">"):
            quote = []
            while i < len(lines) and lines[i].startswith(">"):
                quote.append(lines[i][1:].strip())
                i += 1
            out.append(f"<blockquote><p>{inline(' '.join(quote))}</p></blockquote>")
            continue
        if not line.strip():
            i += 1
            continue
        para = [line]
        i += 1
        while i < len(lines) and lines[i].strip() and not re.match(r"^(#|```|\||>|\s*([-*]|\d+\.)\s)", lines[i]):
            para.append(lines[i])
            i += 1
        content = inline(" ".join(para))
        if re.fullmatch(r'<img [^>]*>', content.strip()):
            out.append(f"<figure>{content}</figure>")      # an image never splits across pages
        else:
            out.append(f"<p>{content}</p>")
    return "\n".join(out)


def webdriver(method, path, body=None):
    req = urllib.request.Request(f"http://127.0.0.1:{PORT}{path}", method=method,
                                 data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=120))


def ensure_driver():
    try:
        webdriver("GET", "/status")
        return None                           # one is already running: reuse it
    except Exception:
        pass
    exe = shutil.which("geckodriver") or shutil.which("firefox.geckodriver") or "/snap/bin/geckodriver"
    proc = subprocess.Popen([exe, "--port", str(PORT)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(50):
        try:
            webdriver("GET", "/status")
            return proc
        except Exception:
            time.sleep(0.2)
    sys.exit("geckodriver did not start")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    proc = ensure_driver()
    sid = webdriver("POST", "/session", {"capabilities": {"alwaysMatch": {
        "moz:firefoxOptions": {"args": ["-headless"]}}}})["value"]["sessionId"]
    try:
        for md_path in sys.argv[1:]:
            md_path = os.path.abspath(md_path)
            body = to_html(open(md_path, encoding="utf-8").read())
            title = os.path.basename(md_path)
            page = (f'<!doctype html><html lang="sk"><head><meta charset="utf-8"><title>{title}</title>'
                    f"<style>{CSS}</style></head><body>{body}</body></html>")
            html_path = md_path[:-3] + ".print.html"
            with open(html_path, "w", encoding="utf-8") as f:
                f.write(page)
            webdriver("POST", f"/session/{sid}/url", {"url": "file://" + html_path})
            time.sleep(1.0)                   # let the images load
            pdf = webdriver("POST", f"/session/{sid}/print", {
                "background": True, "page": {"width": 21.0, "height": 29.7},
                "margin": {"top": 1.6, "bottom": 1.6, "left": 1.6, "right": 1.6}, "shrinkToFit": True})["value"]
            pdf_path = md_path[:-3] + ".pdf"
            with open(pdf_path, "wb") as f:
                f.write(base64.b64decode(pdf))
            os.remove(html_path)
            print(f"Wrote {pdf_path}")
    finally:
        webdriver("DELETE", f"/session/{sid}")
        if proc:
            proc.terminate()


if __name__ == "__main__":
    main()
