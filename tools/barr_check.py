#!/usr/bin/env python3
"""
Heuristic BARR-C:2018 checker for the PicoCar application code.

    python3 tools/barr_check.py app_program/*.[ch]          summary
    python3 tools/barr_check.py -v app_program/*.[ch]       every finding

It tokenises each file (comments, strings, preprocessor lines and code are
kept apart) and checks the rules that can be decided mechanically.  It is a
review aid, not a proof: rules that need judgement (comment quality,
naming that "describes the purpose", volatile use, ...) were reviewed by
hand.  The compiler flags in the template's
build_make/mtkernel_3/app_program/subdir.mk, app_program/.clang-format and
tools/barr_layout.py cover the rest.

Documented project deviations are reported under their own "deviation"
headings so that they stay visible without hiding real findings.
"""

import re
import sys
from collections import defaultdict

KEYWORDS = {
    "if", "else", "for", "while", "do", "switch", "case", "default",
    "return", "break", "continue", "goto", "sizeof", "typedef", "struct",
    "union", "enum", "static", "const", "volatile", "extern", "inline",
    "register", "auto", "void", "char", "short", "int", "long", "float",
    "double", "signed", "unsigned", "_Bool", "bool",
}
TYPE_NAMES = re.compile(
    r"^(void|char|bool|float|double|int|u?int(8|16|32|64)_t|float(32|64)_t|"
    r"u(8|16|32)_t|err_t|[a-z_][a-z0-9_]*_t|UB|B|H|UH|W|UW|INT|UINT|ID|ER|"
    r"FP|TMO|RELTIM|SZ|SYSTIM)$")
# Names fixed by the kernel, the template or the C standard library.
EXTERNAL_NAMES = {"usermain", "tm_usb_rx_byte", "cyw43_utk_app_poll",
                  "blink_task", "main"}
# Files copied unchanged from the template: not checked (report section 8.2).
TEMPLATE_OWNED = {"demo_tasks.c", "demo_tasks.h", "usb_console_compat.h"}
TASK_SIGNATURE = re.compile(r"\(\s*INT\s+\w+\s*,\s*void\s*\*\s*\w+\s*\)")

findings = defaultdict(list)


def add(rule, path, line, text):
    findings[rule].append((path, line, text.strip()[:110]))


class Tok:
    __slots__ = ("kind", "text", "line", "pos")

    def __init__(self, kind, text, line, pos):
        self.kind, self.text, self.line, self.pos = kind, text, line, pos

    def __repr__(self):
        return "%s:%r@%d" % (self.kind, self.text, self.line)


PUNCT = sorted(["<<=", ">>=", "...", "->", "++", "--", "<<", ">>", "<=",
                ">=", "==", "!=", "&&", "||", "+=", "-=", "*=", "/=", "%=",
                "&=", "|=", "^=", "{", "}", "(", ")", "[", "]", ";", ",",
                ":", "?", "=", "<", ">", "+", "-", "*", "/", "%", "&", "|",
                "^", "!", "~", ".", "#"], key=len, reverse=True)


def tokenize(src):
    """Return (code tokens, comment tokens, preprocessor tokens)."""
    toks, comments, pps = [], [], []
    i, n, line = 0, len(src), 1
    at_line_start = True
    while i < n:
        c = src[i]
        if c == "\n":
            line += 1
            i += 1
            at_line_start = True
            continue
        if c in " \t\f\r":
            i += 1
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            comments.append(Tok("comment", src[i:j], line, i))
            line += src.count("\n", i, j)
            i = j
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            comments.append(Tok("comment", src[i:j], line, i))
            i = j
            continue
        if c == "#" and at_line_start:
            j = i
            while True:
                k = src.find("\n", j)
                k = n if k < 0 else k
                if src[k - 1:k] == "\\":
                    j = k + 1
                    continue
                break
            pps.append(Tok("pp", src[i:k], line, i))
            line += src.count("\n", i, k)
            i = k
            continue
        at_line_start = False
        if c in "\"'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            toks.append(Tok("string" if c == '"' else "char", src[i:j + 1],
                            line, i))
            i = j + 1
            continue
        m = re.match(r"[A-Za-z_]\w*", src[i:])
        if m:
            toks.append(Tok("ident", m.group(0), line, i))
            i += len(m.group(0))
            continue
        m = re.match(r"(0[xX][0-9A-Fa-f]+|\d+\.?\d*([eE][+-]?\d+)?|\.\d+)"
                     r"[uUlLfF]*", src[i:])
        if m:
            toks.append(Tok("number", m.group(0), line, i))
            i += len(m.group(0))
            continue
        for p in PUNCT:
            if src.startswith(p, i):
                toks.append(Tok("punct", p, line, i))
                i += len(p)
                break
        else:
            toks.append(Tok("punct", c, line, i))
            i += 1
    return toks, comments, pps


def match_paren(toks, k):
    """Index of the bracket matching toks[k] ('(' '[' or '{')."""
    open_t = toks[k].text
    close_t = {"(": ")", "[": "]", "{": "}"}[open_t]
    depth = 0
    for j in range(k, len(toks)):
        if toks[j].text == open_t:
            depth += 1
        elif toks[j].text == close_t:
            depth -= 1
            if depth == 0:
                return j
    return len(toks) - 1


def is_cast(toks, k):
    """toks[k] is '(' : does it open a cast?"""
    j = k + 1
    words = []
    while j < len(toks) and toks[j].kind == "ident":
        words.append(toks[j].text)
        j += 1
    stars = 0
    while j < len(toks) and toks[j].text == "*":
        stars += 1
        j += 1
    while j < len(toks) and toks[j].text in ("const", "volatile"):
        j += 1
    if not words or j >= len(toks) or toks[j].text != ")":
        return None
    core = [w for w in words if w not in ("const", "volatile", "unsigned",
                                          "signed", "struct")]
    if len(core) != 1 or not TYPE_NAMES.match(core[0]):
        return None
    if k > 0 and (toks[k - 1].kind == "ident"
                  and toks[k - 1].text not in KEYWORDS
                  or toks[k - 1].text in (")", "]")):
        return None                    # function call or sizeof(type)
    if k > 0 and toks[k - 1].text == "sizeof":
        return None
    nxt = toks[j + 1] if j + 1 < len(toks) else None
    if nxt is None or not (nxt.kind in ("ident", "number", "string", "char")
                           or nxt.text in ("(", "&", "*", "-", "!", "~",
                                           "+")):
        return None
    return (" ".join(words) + " " + "*" * stars).strip(), j


def is_zero_or_one(text):
    """0, 0u, 0.0f, 1, 1u, 1.0f ... are not magic numbers."""
    core = text.rstrip("uUlLfF")
    try:
        return float(int(core, 0) if re.match(r"^0[xX]", core)
                     else core) in (0.0, 1.0)
    except ValueError:
        return False


def comment_lines(comments):
    lines = {}
    for c in comments:
        for off, _ in enumerate(c.text.split("\n")):
            lines[c.line + off] = c.text
    return lines


def functions(toks):
    """Yield (name, start_index_of_name, body_open, body_close)."""
    depth = 0
    k = 0
    while k < len(toks):
        t = toks[k]
        if t.text == "{":
            if depth == 0 and k > 0 and toks[k - 1].text == ")":
                # find the '(' matching this ')' and the name before it
                d = 0
                j = k - 1
                while j >= 0:
                    if toks[j].text == ")":
                        d += 1
                    elif toks[j].text == "(":
                        d -= 1
                        if d == 0:
                            break
                    j -= 1
                name_i = j - 1
                if name_i >= 0 and toks[name_i].kind == "ident":
                    end = match_paren(toks, k)
                    yield toks[name_i].text, name_i, k, end
                    k = end + 1
                    continue
            depth += 1
        elif t.text == "}":
            depth -= 1
        k += 1


def operand_bounds(toks, k, direction):
    """Tokens of the operand of the && / || at toks[k]."""
    stop = {"&&", "||", "?", ":", "=", "+=", "-=", "*=", "/=", "%=", "&=",
            "|=", "^=", "<<=", ">>=", ",", ";", "{", "}", "return"}
    out = []
    j = k + direction
    depth = 0
    while 0 <= j < len(toks):
        t = toks[j].text
        if direction > 0:
            if t in ("(", "["):
                depth += 1
            elif t in (")", "]"):
                if depth == 0:
                    break
                depth -= 1
            elif depth == 0 and t in stop:
                break
        else:
            if t in (")", "]"):
                depth += 1
            elif t in ("(", "["):
                if depth == 0:
                    break
                depth -= 1
            elif depth == 0 and t in stop:
                break
        out.append(toks[j])
        j += direction
    return out if direction > 0 else out[::-1]


def operand_ok(ops):
    if len(ops) == 1 and ops[0].kind in ("ident", "number", "char"):
        return True
    if ops and ops[0].text == "(":
        # fully parenthesised?
        depth = 0
        for idx, t in enumerate(ops):
            if t.text == "(":
                depth += 1
            elif t.text == ")":
                depth -= 1
                if depth == 0:
                    return idx == len(ops) - 1
    return False


def check(path):
    src = open(path, encoding="utf-8").read()
    lines = src.split("\n")
    is_c = path.endswith(".c")
    name = path.split("/")[-1]
    toks, comments, pps = tokenize(src)
    clines = comment_lines(comments)

    # ---- whole-file text rules ----
    for n, l in enumerate(lines, 1):
        if len(l) > 80:
            add("1.2.a line longer than 80 characters", name, n, l)
        if "\t" in l:
            add("3.5.a tab character", name, n, l)
        if l.endswith("\r"):
            add("3.6.a CR-LF line ending", name, n, l)
    body = src.rstrip("\n").split("\n")
    if not body or body[-1].strip() != "/*** end of file ***/" or \
            not src.endswith("\n"):
        add("3.3.c no end-of-file comment and blank line", name, len(lines),
            lines[-1] if lines else "")
    if not src.startswith("/** @file " + name):
        add("2.2.h no Doxygen @file header", name, 1, lines[0])
    for c in comments:
        inner = c.text[2:-2] if c.text.startswith("/*") else c.text[2:]
        if "/*" in inner or "//" in inner or "\\" in inner:
            add("2.1.b comment contains /* or // or backslash", name, c.line,
                c.text.split("\n")[0])

    # ---- token rules ----
    for k, t in enumerate(toks):
        if t.kind == "ident" and t.text in ("short", "long"):
            add("5.2.b short or long keyword", name, t.line, lines[t.line - 1])
        if t.kind == "ident" and t.text == "float" and \
                name != "car_types.h":
            add("5.4.b.i float instead of float32_t", name, t.line,
                lines[t.line - 1])
        if t.kind == "ident" and t.text in ("auto", "register"):
            add("1.7.a/b auto or register", name, t.line, lines[t.line - 1])
        if t.kind == "ident" and t.text == "goto":
            add("1.7.c goto", name, t.line, lines[t.line - 1])
        if t.kind == "ident" and t.text == "continue":
            add("1.7.d continue", name, t.line, lines[t.line - 1])
        if t.kind == "number" and re.match(r"^\d*\.\d*([eE][+-]?\d+)?$",
                                           t.text):
            add("5.4.b.ii floating constant without f", name, t.line,
                lines[t.line - 1])
        if t.text in ("==", "!=") and k + 1 < len(toks):
            nb = [toks[k - 1], toks[k + 1]]
            if any(x.kind == "number" and re.search(r"[.fF]$|\.", x.text)
                   for x in nb):
                add("5.4.b.iv floating-point equality test", name, t.line,
                    lines[t.line - 1])
        if t.text == "==" and k + 1 < len(toks):
            right, left = toks[k + 1], toks[k - 1]
            right_const = right.kind in ("number", "char") or \
                re.match(r"^[A-Z][A-Z0-9_]+$", right.text) or \
                right.text in ("NULL", "true", "false")
            left_const = left.kind in ("number", "char") or \
                re.match(r"^[A-Z][A-Z0-9_]+$", left.text) or \
                left.text in ("NULL", "true", "false", ")")
            if right_const and not left_const and \
                    (k + 2 >= len(toks) or toks[k + 2].text in
                     (")", ";", "&&", "||", "?")):
                add("8.6.a constant on the right of ==", name, t.line,
                    lines[t.line - 1])
        if t.text in ("&&", "||"):
            for side in (-1, 1):
                ops = operand_bounds(toks, k, side)
                if not operand_ok(ops):
                    add("1.4.b operand of && or || not parenthesised", name,
                        t.line, " ".join(x.text for x in ops)[:100])
        if t.text == "(":
            c = is_cast(toks, k)
            if c:
                ctype, _ = c
                if ctype == "void":
                    add("deviation-note 1.6.a (void) discard cast",
                        name, t.line, lines[t.line - 1])
                    continue
                ok = False
                for back in range(0, 30):
                    ln = t.line - back
                    if ln < 1:
                        break
                    if back > 0 and lines[ln - 1].startswith("}"):
                        break
                    text = clines.get(ln)
                    if text is not None:
                        ok = "cast" in text.lower()
                        if ok or back > 0:
                            break
                if not ok:
                    add("1.6.a cast without an explaining comment", name,
                        t.line, lines[t.line - 1])

    # ---- declarations ----
    depth = 0
    for k, t in enumerate(toks):
        if t.text == "{":
            depth += 1
        elif t.text == "}":
            depth -= 1
        if t.kind != "ident" or t.text in KEYWORDS:
            continue
        nxt = toks[k + 1].text if k + 1 < len(toks) else ""
        prev = toks[k - 1] if k > 0 else None
        if nxt not in ("=", ";", ",", "[", ")") or prev is None:
            continue
        # declaration if preceded by a type name (optionally '*'s)
        j = k - 1
        stars = 0
        while j >= 0 and toks[j].text in ("*", "const", "volatile"):
            stars += toks[j].text == "*"
            j -= 1
        if j < 0 or toks[j].kind != "ident" or \
                not TYPE_NAMES.match(toks[j].text) or \
                (j > 0 and toks[j - 1].text in (".", "->")):
            continue
        while j > 0 and toks[j - 1].text in ("const", "volatile", "static",
                                              "unsigned", "signed"):
            j -= 1
        if j > 0 and toks[j - 1].text == "typedef":
            continue
        if j > 0 and toks[j - 1].text == "struct":
            j -= 1
        if j > 0 and toks[j - 1].text in ("(", ",") and nxt == ")" and \
                stars == 0 and toks[j].text == "void":
            continue
        var = t.text
        in_struct = False
        # skip struct / enum members: look for enclosing 'struct {' / enum
        d = 0
        for b in range(k, -1, -1):
            if toks[b].text == "}":
                d += 1
            elif toks[b].text == "{":
                if d == 0:
                    in_struct = b > 0 and toks[b - 1].text in (
                        "struct", "union", "enum") or \
                        (b > 1 and toks[b - 2].text in ("struct", "union"))
                    break
                d -= 1
        if in_struct or var in EXTERNAL_NAMES:
            continue
        typename = toks[j].text
        is_global = depth == 0 and not (j > 0 and toks[j - 1].text in
                                        ("(", ","))
        base = var
        if is_global:
            if not base.startswith("g"):
                add("7.1.j global name does not start with g", name, t.line,
                    lines[t.line - 1])
            base = base[1:]
        if stars >= 2 and not base.startswith("pp"):
            add("7.1.l pointer-to-pointer name does not start with pp", name,
                t.line, lines[t.line - 1])
        elif stars == 1 and nxt != "[" and not base.startswith("p"):
            add("7.1.k pointer name does not start with p", name, t.line,
                lines[t.line - 1])
        if stars and nxt != "[":
            base = base[2:] if base.startswith("pp") else base[1:]
        if typename == "bool" and not base.startswith("b_"):
            add("7.1.m Boolean name does not start with b", name, t.line,
                lines[t.line - 1])
        if typename == "ID" and stars == 0 and not base.startswith("h_") \
                and not var.startswith("gh_"):
            add("7.1.n handle name does not start with h", name, t.line,
                lines[t.line - 1])
        if len(var) < 3:
            add("7.1.e name shorter than 3 characters", name, t.line,
                lines[t.line - 1])
        if re.search(r"[A-Z]", var):
            add("7.1.f upper-case letter in variable name", name, t.line,
                lines[t.line - 1])
        if stars and depth > 0 and nxt == ";" and \
                not (j > 0 and toks[j - 1].text in ("(", ",")):
            add("7.2.d pointer declared without an initial value", name,
                t.line, lines[t.line - 1])
        if nxt == "," and depth > 0 and \
                not (j > 0 and toks[j - 1].text in ("(", ",")):
            add("8.1.a comma in a variable declaration", name, t.line,
                lines[t.line - 1])

    # ---- functions ----
    seen_static_body = False
    first_body_line = None
    for fname, name_i, open_i, close_i in functions(toks):
        start_line = toks[name_i].line
        end_line = toks[close_i].line
        first_body_line = first_body_line or start_line
        body = toks[open_i:close_i + 1]
        head_line = lines[start_line - 1]
        prev_line = lines[start_line - 2] if start_line > 1 else ""
        is_static = prev_line.startswith("static") or \
            head_line.startswith("static")
        if not re.match(r"^[a-z_][a-z0-9_]* \(", head_line):
            add("3.1.j definition not 'name (' on its own line", name,
                start_line, head_line)
        if re.search(r"[A-Z]", fname):
            add("6.1.e upper-case letter in function name", name, start_line,
                head_line)
        module = name.split(".")[0]
        module = module[:-len("_stub")] if module.endswith("_stub") \
            else module                # a stub implements the real API
        if not is_static and fname not in EXTERNAL_NAMES and \
                not fname.startswith(module + "_"):
            add("6.1.i public function without module prefix", name,
                start_line, head_line)
        if fname in EXTERNAL_NAMES and fname != "main":
            add("deviation 6.1.i name fixed by kernel or template", name,
                start_line, head_line)
        if TASK_SIGNATURE.search(src[toks[name_i].pos:body[0].pos]) and \
                not fname.endswith("_task"):
            add("6.4.a task name does not end with _task", name, start_line,
                head_line)
        # Doxygen block right above the definition
        k = start_line - 2
        while k >= 0 and lines[k].strip() == "":
            k -= 1
        while k >= 0 and lines[k].startswith(("static", "void", "bool",
                                              "int", "uint", "float",
                                              "char", "INT", "UINT",
                                              "err_t", "avoidance",
                                              "barcode", "terrain",
                                              "line_follow", "search",
                                              "mqtt")) and \
                not lines[k].endswith(";"):
            k -= 1
        if k < 0 or not lines[k].strip().endswith("*/") or \
                "@brief" not in "\n".join(lines[max(0, k - 40):k + 1]):
            add("2.2.h function without a Doxygen @brief block", name,
                start_line, head_line)
        if end_line - start_line > 100:
            add("6.2.a function longer than 100 lines", name, start_line,
                "%s (%d lines)" % (fname, end_line - start_line))
        returns = [t for t in body if t.text == "return"]
        if len(returns) > 1:
            add("6.2.c more than one return", name, start_line, fname)
        elif len(returns) == 1:
            # the single return must be the last statement
            idx = body.index(returns[0])
            semi = idx
            while body[semi].text != ";":
                semi += 1
            if semi != len(body) - 2:
                add("6.2.c return is not the last statement", name,
                    returns[0].line, fname)
        # nesting of if statements, else-if chains, switch default
        stack = []
        pending = None
        chain_open = []
        k = 0
        while k < len(body):
            t = body[k]
            if t.text == "if":
                is_else_if = k > 0 and body[k - 1].text == "else"
                level = sum(1 for s in stack if s == "if")
                if level + 1 > 2:
                    add("8.2.b if nested deeper than two levels", name,
                        t.line, lines[t.line - 1])
                pending = "if"
                close = match_paren(body, k + 1)
                k = close + 1
                continue
            if t.text == "else":
                pending = "if"
            if t.text in ("switch", "for", "while", "do"):
                pending = "other"
            if t.text == "{":
                stack.append(pending or "other")
                pending = None
            elif t.text == "}":
                if stack:
                    stack.pop()
            k += 1
        for k, t in enumerate(body):
            if t.text == "switch":
                close = match_paren(body, match_paren(body, k + 1) + 1)
                seg = body[k:close + 1]
                if not any(x.text == "default" for x in seg):
                    add("8.3.b switch without default", name, t.line,
                        lines[t.line - 1])
            if t.text == "else" and k + 1 < len(body) and \
                    body[k + 1].text == "if":
                # find the end of this else-if block and check what follows
                close_paren = match_paren(body, k + 2)
                if body[close_paren + 1].text == "{":
                    end = match_paren(body, close_paren + 1)
                    if end + 1 < len(body) and body[end + 1].text != "else":
                        add("8.2.d else-if chain without final else", name,
                            t.line, lines[t.line - 1])
            if t.text in ("for", "while") and k + 1 < len(body) and \
                    body[k + 1].text == "(":
                close = match_paren(body, k + 1)
                ctrl = body[k + 2:close]
                if t.text == "for":
                    semis = [i for i, x in enumerate(ctrl) if x.text == ";"]
                    if len(semis) == 2:
                        init = ctrl[:semis[0]]
                        cond = ctrl[semis[0] + 1:semis[1]]
                        step = ctrl[semis[1] + 1:]
                        if any(x.text in ("=", "+=", "-=", "++", "--")
                               for x in cond):
                            add("8.4.b assignment in a loop condition",
                                name, t.line, lines[t.line - 1])
                        for x in init + cond:
                            if x.kind == "number" and \
                                    not is_zero_or_one(x.text):
                                add("8.4.a magic number in a for loop",
                                    name, t.line, lines[t.line - 1])
                        if not ctrl or (not init and not cond and not step):
                            pass
                else:
                    if any(x.text in ("=", "+=", "-=", "++", "--")
                           for x in ctrl):
                        add("8.4.b assignment in a loop condition", name,
                            t.line, lines[t.line - 1])
                    for x in ctrl:
                        if x.kind == "number" and \
                                not is_zero_or_one(x.text):
                            add("8.4.a magic number in a while loop", name,
                                t.line, lines[t.line - 1])
                    if len(ctrl) == 1 and ctrl[0].text in ("1", "true"):
                        add("8.4.c infinite loop not written for (;;)",
                            name, t.line, lines[t.line - 1])
            if t.text == "if" and k + 1 < len(body):
                close = match_paren(body, k + 1)
                if any(x.text in ("=", "+=", "-=", "*=", "/=", "++", "--")
                       for x in body[k + 2:close]):
                    add("8.2.c assignment inside an if test", name, t.line,
                        lines[t.line - 1])
        if is_static:
            seen_static_body = True
        elif seen_static_body and fname not in EXTERNAL_NAMES:
            add("4.3.b public function body after a private one", name,
                start_line, head_line)

    # ---- file structure (4.3.b, 4.3.c) ----
    if is_c:
        header = name[:-2] + ".h"
        if not any(re.search(r'#include\s+"%s"' % re.escape(header), p.text)
                   for p in pps):
            add("4.3.c source does not include its own header", name, 1,
                header)
        if first_body_line:
            for p in pps:
                if p.text.startswith("#include") and p.line > first_body_line:
                    add("4.3.b include after a function body", name, p.line,
                        p.text)
            depth = 0
            for k, t in enumerate(toks):
                if t.text == "{":
                    depth += 1
                elif t.text == "}":
                    depth -= 1
                elif depth == 0 and t.text == "static" and \
                        t.line > first_body_line:
                    # a prototype or data after the first body?
                    j = k
                    while toks[j].text not in (";", "{"):
                        j += 1
                    if toks[j].text == ";":
                        add("4.3.b declaration after a function body",
                            name, t.line, lines[t.line - 1])


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    for p in args:
        if p.split("/")[-1] in TEMPLATE_OWNED:
            print("skipped (template-owned): %s" % p)
            continue
        check(p)
    total = 0
    for rule in sorted(findings):
        items = findings[rule]
        if not rule.startswith("deviation"):
            total += len(items)
        files = sorted({f for f, _, _ in items})
        print("%-56s %4d in %2d files" % (rule, len(items), len(files)))
        if verbose or not rule.startswith("deviation"):
            for f, n, text in items[:400 if verbose else 12]:
                print("    %s:%d: %s" % (f, n, text))
    print("TOTAL (excluding deviations):", total)
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
