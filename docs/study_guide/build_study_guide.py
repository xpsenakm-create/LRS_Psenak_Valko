#!/usr/bin/env python3
"""
Builds docs/A1_2_study_guide.html: a self-contained, offline study guide that shows the real
source code of the project block by block, each block with an explanation next to it.

The code is read from the repository at build time, so line numbers are always current.
Explanations are attached to blocks by an *anchor*: a piece of text that the first line of the
block contains.  A block runs until the next anchor, so every line of a file belongs to exactly
one explained block.  An anchor that cannot be found stops the build with an error, so the guide
can never silently drift away from the code.

    python3 docs/study_guide/build_study_guide.py
"""

import html
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "docs", "A1_2_study_guide.html")

# Chapters with code, in reading order: (notes module, language)
CODE_CHAPTERS = [
    ("notes_executor", "cpp"),
    ("notes_mission_model_h", "cpp"),
    ("notes_mission_model_cpp", "cpp"),
    ("notes_run_script", "bash"),
    ("notes_plot", "python"),
    ("notes_view3d", "python"),
    ("notes_planner_yaml", "bash"),
    ("notes_astar_h", "cpp"),
    ("notes_astar", "cpp"),
    ("notes_planner_node", "cpp"),
    ("notes_voxel_grid", "cpp"),
    ("notes_build_files", "cmake"),
    ("notes_package_xml", "text"),
]


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------------ syntax highlighting
CPP_KEYWORDS = set("""
alignas auto bool break case catch char class const constexpr continue decltype default delete do
double else enum explicit false final float for if inline int int8_t int32_t int64_t uint8_t uint16_t
uint32_t uint64_t size_t long namespace new nullptr operator override private protected public return
short signed static static_cast struct switch template this throw true try typedef typename union
unsigned using virtual void volatile while std
""".split())
BASH_KEYWORDS = set("""
if then else elif fi for in do done while until case esac function return local export set
source trap exit break continue printf echo read wait kill shift true false
""".split())
PY_KEYWORDS = set("""
and as assert break class continue def del elif else except False finally for from global if import
in is lambda None nonlocal not or pass raise return True try while with yield print
""".split())
CMAKE_KEYWORDS = set("""
cmake_minimum_required project set if endif find_package add_executable add_library
target_include_directories target_compile_definitions target_link_directories target_link_libraries
ament_target_dependencies install ament_package
""".split())


def highlight_line(line, lang, state):
    """Return HTML for one source line.  `state` carries multi-line string state for Python."""
    out = []
    i = 0
    n = len(line)

    def emit(cls, text):
        out.append(f'<span class="{cls}">{html.escape(text)}</span>' if cls else html.escape(text))

    if lang == "python" and state.get("triple"):
        q = state["triple"]
        end = line.find(q)
        if end < 0:
            emit("s", line)
            return "".join(out)
        emit("s", line[:end + 3])
        i = end + 3
        state["triple"] = None

    if lang == "cpp" and re.match(r"\s*#\s*\w+", line):
        emit("pp", line)
        return "".join(out)

    kw = {"cpp": CPP_KEYWORDS, "bash": BASH_KEYWORDS, "python": PY_KEYWORDS,
          "cmake": CMAKE_KEYWORDS}.get(lang, set())
    while i < n:
        rest = line[i:]
        if lang == "cpp" and rest.startswith("//"):
            emit("c", rest)
            break
        if lang in ("bash", "python", "cmake") and rest.startswith("#") and (
                lang != "bash" or i == 0 or line[i - 1] in " \t;(") and not (
                lang == "bash" and i > 0 and line[i - 1] in "${"):
            emit("c", rest)
            break
        if lang == "python" and (rest.startswith('"""') or rest.startswith("'''")):
            q = rest[:3]
            end = rest.find(q, 3)
            if end < 0:
                emit("s", rest)
                state["triple"] = q
                break
            emit("s", rest[:end + 3])
            i += end + 3
            continue
        if rest[0] == '"' or (rest[0] == "'" and (lang != "cpp" or re.match(r"'(\\.|[^'\\])'", rest))):
            q = rest[0]
            j = 1
            while j < len(rest):
                if rest[j] == "\\":
                    j += 2
                    continue
                if rest[j] == q:
                    break
                j += 1
            emit("s", rest[:j + 1])
            i += j + 1
            continue
        m = re.match(r"\d+(\.\d+)?([eE][-+]?\d+)?f?", rest)
        if m and (i == 0 or not (line[i - 1].isalnum() or line[i - 1] == "_")):
            emit("n", m.group(0))
            i += len(m.group(0))
            continue
        m = re.match(r"[A-Za-z_][A-Za-z0-9_]*", rest)
        if m:
            word = m.group(0)
            emit("k" if word in kw else "", word)
            i += len(word)
            continue
        emit("", rest[0])
        i += 1
    return "".join(out)


# ------------------------------------------------------------------ blocks
def split_blocks(lines, notes, path):
    """Find each note's anchor (searching forward) and return blocks covering every line."""
    starts = []
    pos = 0
    for anchor, title, text in notes:
        for k in range(pos, len(lines)):
            if anchor in lines[k]:
                starts.append((k, title, text))
                pos = k + 1
                break
        else:
            sys.exit(f"{path}: anchor not found after line {pos + 1}: {anchor!r}")
    if not starts or starts[0][0] != 0:
        sys.exit(f"{path}: the first note must be anchored on line 1 (found line {starts[0][0] + 1 if starts else '-'})")
    blocks = []
    for idx, (k, title, text) in enumerate(starts):
        end = starts[idx + 1][0] if idx + 1 < len(starts) else len(lines)
        blocks.append((k, end, title, text))
    return blocks


def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def render_code_chapter(module, lang, chapter_no):
    path = os.path.join(ROOT, module.FILE)
    with open(path, encoding="utf-8") as f:
        lines = f.read().split("\n")
    if lines and lines[-1] == "":
        lines = lines[:-1]
    blocks = split_blocks(lines, module.NOTES, module.FILE)
    cid = f"ch{chapter_no}"
    state = {}
    parts = [f'<section class="chapter" id="{cid}">',
             f'<h2><span class="num">{chapter_no}</span>{html.escape(module.TITLE)}</h2>',
             f'<p class="filepath">{html.escape(module.FILE)} &middot; {len(lines)} lines &middot; '
             f'{len(blocks)} explained blocks</p>',
             f'<div class="intro">{module.INTRO}</div>']
    toc = []
    for b_no, (start, end, title, text) in enumerate(blocks, start=1):
        bid = f"{cid}-b{b_no}"
        code_html = []
        for ln in range(start, end):
            code_html.append(f'<span class="ln">{ln + 1}</span>{highlight_line(lines[ln], lang, state)}')
        rng = f"{start + 1}" if end - start == 1 else f"{start + 1}–{end}"
        parts.append(
            f'<article class="block" id="{bid}">'
            f'<div class="code"><pre><code>{chr(10).join(code_html)}</code></pre></div>'
            f'<div class="note"><div class="note-head"><span class="lines">L{rng}</span>'
            f'<h3>{html.escape(title)}</h3></div><div class="note-body">{text}</div></div>'
            f'</article>')
        toc.append((bid, title))
    parts.append("</section>")
    return "\n".join(parts), (cid, module.TITLE, toc)


def render_text_chapter(cid, number, title, body):
    return (f'<section class="chapter prose" id="{cid}"><h2><span class="num">{number}</span>'
            f'{html.escape(title)}</h2>{body}</section>')


def main():
    prose = load_module("prose")
    questions = load_module("questions")

    chapters_html, toc = [], []
    number = 0
    for cid, title, body in prose.BEFORE_CODE:
        number += 1
        chapters_html.append(render_text_chapter(cid, number, title, body))
        toc.append((cid, title, []))
    for name, lang in CODE_CHAPTERS:
        number += 1
        h, t = render_code_chapter(load_module(name), lang, number)
        chapters_html.append(h)
        toc.append(t)
    number += 1
    chapters_html.append(render_text_chapter("questions", number, "Examiner questions (flashcards)",
                                             questions.render()))
    toc.append(("questions", "Examiner questions (flashcards)", []))
    for cid, title, body in prose.AFTER_CODE:
        number += 1
        chapters_html.append(render_text_chapter(cid, number, title, body))
        toc.append((cid, title, []))

    nav = []
    for cid, title, sub in toc:
        if sub:
            items = "".join(f'<li><a href="#{bid}">{html.escape(t)}</a></li>' for bid, t in sub)
            nav.append(f'<li><details><summary><a href="#{cid}">{html.escape(title)}</a></summary>'
                       f'<ol class="sub">{items}</ol></details></li>')
        else:
            nav.append(f'<li><a href="#{cid}">{html.escape(title)}</a></li>')

    with open(os.path.join(HERE, "template.html"), encoding="utf-8") as f:
        template = f.read()
    page = (template.replace("{{NAV}}", "\n".join(nav))
                    .replace("{{CONTENT}}", "\n".join(chapters_html)))
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(page)
    n_blocks = sum(len(t[2]) for t in toc)
    print(f"Wrote {OUT} ({len(page) // 1024} KB, {len(toc)} chapters, {n_blocks} code blocks)")


if __name__ == "__main__":
    main()
