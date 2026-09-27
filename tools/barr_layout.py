#!/usr/bin/env python3
"""
BARR-C:2018 layout rules that clang-format cannot express.

Run after clang-format:   python3 tools/barr_layout.py app_program/*.[ch]

  8.3.a  'break;' ending a case is aligned with its 'case' label.
  3.3.b  a blank line before and after each natural block of code
         (if/else, loops, switch, do/while).
  3.3.c  every file ends with an end-of-file comment and a blank line.

The script is idempotent: running it twice changes nothing more.
"""

import re
import sys

EOF_MARKER = "/*** end of file ***/"
BLOCK_START = re.compile(r"^\s*(if|for|while|switch|do)\b")
CASE_LABEL = re.compile(r"^\s*(case\b.*|default\s*):\s*$")


def align_breaks(lines):
    """Dedent 'break;' by one level when it closes a case body."""
    out = []
    case_indent = []                       # stack of case label indents
    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))
        if CASE_LABEL.match(line):
            while case_indent and case_indent[-1] > indent:
                case_indent.pop()
            if case_indent and case_indent[-1] == indent:
                case_indent.pop()
            case_indent.append(indent)
        elif stripped.startswith("}") and case_indent and \
                indent < case_indent[-1]:
            case_indent.pop()              # end of the switch
        if stripped == "break;" and case_indent and \
                indent == case_indent[-1] + 4:
            line = " " * case_indent[-1] + "break;"
        out.append(line)
    return out


def is_blank(line):
    return line.strip() == ""


def is_comment_line(line):
    s = line.strip()
    return s.startswith(("/*", "*", "//")) or s.endswith("*/")


def block_end(lines, idx):
    """Index of the line that closes the block starting at lines[idx]."""
    depth = 0
    seen = False
    j = idx
    while j < len(lines):
        code = re.sub(r"/\*.*?\*/|//.*$|\"(\\.|[^\"])*\"|'(\\.|[^'])*'", "",
                      lines[j])
        depth += code.count("{") - code.count("}")
        if "{" in code:
            seen = True
        if seen and depth == 0:
            nxt = j + 1
            while nxt < len(lines) and is_blank(lines[nxt]):
                nxt += 1
            # if/else chains and do/while are one natural block
            if nxt < len(lines) and re.match(r"^\s*(else\b|while\b.*;\s*$)",
                                             lines[nxt]):
                j = nxt
                seen = False
                depth = 0
                continue
            return j
        j += 1
    return len(lines) - 1


def blank_lines(lines):
    """Insert blank lines around natural blocks inside function bodies."""
    out = list(lines)
    i = 0
    while i < len(out):
        line = out[i]
        if BLOCK_START.match(line) and not re.match(r"^\s*while\b.*;\s*$",
                                                    line):
            # --- blank line before (above any comment describing it) ---
            k = i - 1
            while k >= 0 and is_comment_line(out[k]) and \
                    not out[k].strip().startswith("{"):
                k -= 1
            prev = out[k].strip() if k >= 0 else ""
            if k >= 0 and prev and not prev.endswith(("{", ":")) and \
                    not prev.startswith("#") and not is_blank(out[k]):
                out.insert(k + 1, "")
                i += 1
            # --- blank line after ---
            end = block_end(out, i)
            nxt = end + 1
            if nxt < len(out) and not is_blank(out[nxt]):
                s = out[nxt].strip()
                if not (s.startswith("}") or s == "break;" or
                        CASE_LABEL.match(out[nxt]) or s.startswith("#")):
                    out.insert(nxt, "")
        i += 1
    return out


def ensure_eof(lines):
    while lines and is_blank(lines[-1]):
        lines.pop()
    if not lines or lines[-1].strip() != EOF_MARKER:
        lines += ["", EOF_MARKER]
    return lines + [""]


def main():
    for path in sys.argv[1:]:
        src = open(path).read()
        lines = src.split("\n")
        lines = align_breaks(lines)
        if path.endswith(".c"):
            lines = blank_lines(lines)
        lines = ensure_eof(lines)
        new = "\n".join(lines)
        new = re.sub(r"\n{3,}", "\n\n", new)
        if new != src:
            open(path, "w").write(new)
            print("layout:", path)


if __name__ == "__main__":
    main()
