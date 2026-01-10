#!/usr/bin/env python3
"""
localpp.py — "local-only" C/C++ preprocessor

What it does:
- Applies ONLY macro definitions you provide (via -D or a text file) plus any #define/#undef in the file itself.
- Evaluates #if/#ifdef/#ifndef/#elif/#else/#endif and keeps only active branches.
- Expands object-like and function-like macros only when their definitions are known from the above.
- DOES NOT follow #include (leaves #include lines as-is; no header expansion).

This is intentionally not a full C preprocessor; it's a pragmatic tool for untangling conditional compilation
without pulling in headers.

Usage examples:
  python3 localpp.py -DLOAD_GLCD=1 -DLOAD_FONT2=1 -DESP32=1 Sprite.cpp -o Sprite.localpp.cpp
  python3 localpp.py --defs defs.txt TFT_eSPI.cpp -o TFT_eSPI.localpp.cpp

defs.txt can contain lines like:
  -D LOAD_GLCD=1
  -DLOAD_FONT2=1
  -D ESP32=1
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional


# -----------------------------
# Macro model + expansion
# -----------------------------

@dataclass
class Macro:
    name: str
    params: Optional[List[str]]  # None for object-like
    body: str                    # raw replacement list (best-effort)
    # Note: this is not a token-accurate macro engine; it's sufficient for many embedded projects.


def normalize_define_value(v: Optional[str]) -> str:
    # In the preprocessor, `-DFOO` typically implies `FOO=1`
    if v is None or v == "":
        return "1"
    return v.strip()


def parse_D_arg(s: str) -> Tuple[str, str]:
    """
    Accepts:
      "NAME"
      "NAME=VALUE"
    Returns (NAME, VALUE)
    """
    if "=" in s:
        n, v = s.split("=", 1)
        return n.strip(), normalize_define_value(v)
    return s.strip(), "1"


def load_defs_from_text(text: str) -> Dict[str, Macro]:
    """
    Parses lines like:
      -DNAME=VALUE
      -D NAME=VALUE
      -DNAME
      -D NAME
    """
    macros: Dict[str, Macro] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        # Accept the user's format with spaces, e.g. "-D LOAD_GLCD=1"
        m = re.match(r"^-D\s*(.+)$", line)
        if not m:
            continue
        spec = m.group(1).strip()

        name, value = parse_D_arg(spec)
        macros[name] = Macro(name=name, params=None, body=value)
    return macros


def is_ident_char(ch: str) -> bool:
    return ch.isalnum() or ch == "_"


def expand_object_macros_once(s: str, macros: Dict[str, Macro]) -> str:
    """
    Expand object-like macros once (no recursion explosion safeguards beyond a few passes).
    Token-boundary-ish: matches identifiers and replaces if object-like macro exists.
    """
    def repl(m: re.Match) -> str:
        ident = m.group(0)
        mac = macros.get(ident)
        if mac and mac.params is None:
            return mac.body
        return ident

    return re.sub(r"\b[A-Za-z_]\w*\b", repl, s)


def split_args(arg_str: str) -> List[str]:
    """
    Very small argument splitter for function-like macros.
    Does not fully implement preprocessor tokenization; handles nested parentheses reasonably.
    """
    args: List[str] = []
    cur = []
    depth = 0
    i = 0
    while i < len(arg_str):
        ch = arg_str[i]
        if ch == "(":
            depth += 1
            cur.append(ch)
        elif ch == ")":
            if depth > 0:
                depth -= 1
            cur.append(ch)
        elif ch == "," and depth == 0:
            args.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
        i += 1
    tail = "".join(cur).strip()
    if tail != "":
        args.append(tail)
    return args


def expand_function_macros_once(s: str, macros: Dict[str, Macro]) -> str:
    """
    Expands function-like macros in a best-effort manner:
      NAME(arg1, arg2)
    Replaces parameter occurrences in body with provided args.
    """
    # Find candidate calls NAME(...)
    # This regex is intentionally conservative; it won't catch every case but avoids many false positives.
    pattern = re.compile(r"\b([A-Za-z_]\w*)\s*\(")

    out = []
    i = 0
    while i < len(s):
        m = pattern.search(s, i)
        if not m:
            out.append(s[i:])
            break
        name = m.group(1)
        mac = macros.get(name)
        if not mac or mac.params is None:
            out.append(s[i:m.end()])
            i = m.end()
            continue

        # Parse argument list starting at the '('
        start_paren = m.end() - 1
        j = start_paren
        depth = 0
        while j < len(s):
            if s[j] == "(":
                depth += 1
            elif s[j] == ")":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        if j >= len(s) or s[j] != ")":
            # Unbalanced; give up
            out.append(s[i:m.end()])
            i = m.end()
            continue

        arg_text = s[start_paren + 1 : j]
        args = split_args(arg_text)
        params = mac.params

        # If arg count mismatch, do not expand
        if params is None or len(args) != len(params):
            out.append(s[i:j+1])
            i = j + 1
            continue

        # Expand object macros inside args first (typical behavior)
        args_exp = [expand_object_macros_once(a, macros) for a in args]

        body = mac.body
        # Replace parameters (identifier-boundary replace)
        for p, a in zip(params, args_exp):
            body = re.sub(rf"\b{re.escape(p)}\b", a, body)

        out.append(s[i:m.start()] + body)
        i = j + 1

    return "".join(out)


def expand_macros(s: str, macros: Dict[str, Macro], passes: int = 5) -> str:
    """
    Best-effort macro expansion with limited passes to avoid runaway recursion.
    """
    prev = s
    for _ in range(passes):
        cur = expand_function_macros_once(prev, macros)
        cur = expand_object_macros_once(cur, macros)
        if cur == prev:
            return cur
        prev = cur
    return prev


# -----------------------------
# Conditional expression eval
# -----------------------------

def pp_defined(name: str, macros: Dict[str, Macro]) -> int:
    return 1 if name in macros else 0


def eval_if_expr(expr: str, macros: Dict[str, Macro]) -> bool:
    """
    Evaluate a #if / #elif expression using a small, safe subset:
    - supports defined(NAME) and defined NAME
    - replaces identifiers with 0 or their macro body if it looks like an int
    - supports operators: !, &&, ||, ==, !=, <, <=, >, >=, +, -, *, /, %, parentheses
    This is best-effort; complex expressions may not evaluate exactly like a real preprocessor.
    """
    e = expr.strip()

    # Expand object macros in the expression first
    e = expand_object_macros_once(e, macros)

    # Handle defined(X) and defined X
    e = re.sub(r"\bdefined\s*\(\s*([A-Za-z_]\w*)\s*\)", lambda m: str(pp_defined(m.group(1), macros)), e)
    e = re.sub(r"\bdefined\s+([A-Za-z_]\w*)\b", lambda m: str(pp_defined(m.group(1), macros)), e)

    # Replace any remaining identifiers with 0 (undefined) or numeric macro body if possible
    def ident_repl(m: re.Match) -> str:
        ident = m.group(0)
        mac = macros.get(ident)
        if mac and mac.params is None:
            v = mac.body.strip()
            # allow simple integer literals (decimal/hex)
            if re.fullmatch(r"0x[0-9A-Fa-f]+|[0-9]+", v):
                return v
        return "0"

    e = re.sub(r"\b[A-Za-z_]\w*\b", ident_repl, e)

    # Map C operators to Python
    e = e.replace("&&", " and ")
    e = e.replace("||", " or ")
    # Be careful with '!' vs '!='
    e = re.sub(r"(?<![=!<>])!(?!=)", " not ", e)

    # Evaluate safely
    try:
        val = eval(e, {"__builtins__": {}}, {})
        return bool(val)
    except Exception:
        # Unknown expression forms default to False (conservative)
        return False


# -----------------------------
# File processing
# -----------------------------

@dataclass
class IfFrame:
    parent_active: bool
    this_branch_active: bool
    any_branch_taken: bool


def parse_define_line(rest: str) -> Optional[Macro]:
    """
    Parse '#define NAME ...' and '#define NAME(args) ...'
    """
    rest = rest.strip()
    if not rest:
        return None

    # function-like define
    m = re.match(r"^([A-Za-z_]\w*)\s*\(\s*([A-Za-z_]\w*(?:\s*,\s*[A-Za-z_]\w*)*)?\s*\)\s*(.*)$", rest)
    if m:
        name = m.group(1)
        params_s = m.group(2)
        body = m.group(3) or ""
        params = []
        if params_s and params_s.strip():
            params = [p.strip() for p in params_s.split(",")]
        return Macro(name=name, params=params, body=body.rstrip())

    # object-like define
    m = re.match(r"^([A-Za-z_]\w*)\s*(.*)$", rest)
    if not m:
        return None
    name = m.group(1)
    body = m.group(2).rstrip()
    body = normalize_define_value(body) if body == "" else body
    return Macro(name=name, params=None, body=body)


def local_preprocess(lines: List[str], macros: Dict[str, Macro], expand_in_code: bool = True) -> List[str]:
    """
    Applies local-only preprocessing and returns output lines.
    Includes are preserved as-is.
    """
    out: List[str] = []
    stack: List[IfFrame] = []
    active = True

    directive_re = re.compile(r"^\s*#\s*([A-Za-z_]\w*)\b(.*)$")

    for line in lines:
        m = directive_re.match(line)
        if m:
            directive = m.group(1)
            rest = m.group(2) or ""

            if directive == "include":
                # Do not follow includes. Keep line if active.
                if active:
                    out.append(line)
                continue

            if directive == "define":
                if active:
                    mac = parse_define_line(rest)
                    if mac:
                        macros[mac.name] = mac
                # Drop defines from output to reduce noise (common preference).
                # If you want to keep them, append when active.
                continue

            if directive == "undef":
                if active:
                    name = rest.strip().split()[0] if rest.strip() else ""
                    if name:
                        macros.pop(name, None)
                continue

            if directive in ("if", "ifdef", "ifndef"):
                parent_active = active
                if not parent_active:
                    frame = IfFrame(parent_active=False, this_branch_active=False, any_branch_taken=False)
                    stack.append(frame)
                    active = False
                    continue

                if directive == "if":
                    cond = eval_if_expr(rest, macros)
                elif directive == "ifdef":
                    name = rest.strip().split()[0] if rest.strip() else ""
                    cond = (name in macros)
                else:  # ifndef
                    name = rest.strip().split()[0] if rest.strip() else ""
                    cond = (name not in macros)

                frame = IfFrame(parent_active=True, this_branch_active=cond, any_branch_taken=cond)
                stack.append(frame)
                active = cond
                continue

            if directive == "elif":
                if not stack:
                    continue
                frame = stack[-1]
                if not frame.parent_active:
                    active = False
                    continue
                if frame.any_branch_taken:
                    frame.this_branch_active = False
                    active = False
                else:
                    cond = eval_if_expr(rest, macros)
                    frame.this_branch_active = cond
                    frame.any_branch_taken = cond
                    active = cond
                stack[-1] = frame
                continue

            if directive == "else":
                if not stack:
                    continue
                frame = stack[-1]
                if not frame.parent_active:
                    active = False
                    continue
                take = not frame.any_branch_taken
                frame.this_branch_active = take
                frame.any_branch_taken = True
                stack[-1] = frame
                active = take
                continue

            if directive == "endif":
                if not stack:
                    continue
                frame = stack.pop()
                active = frame.parent_active
                continue

            # Any other directive: keep if active (optional)
            if active:
                out.append(line)
            continue

        # Normal code line
        if not active:
            continue

        if expand_in_code:
            out.append(expand_macros(line, macros))
        else:
            out.append(line)

    return out


# -----------------------------
# CLI
# -----------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="Input .c/.cpp file")
    ap.add_argument("-o", "--output", required=True, help="Output file")
    ap.add_argument("-D", dest="defs", action="append", default=[], help="Macro definition (NAME or NAME=VALUE)")
    ap.add_argument("--defs", dest="defs_file", help="Text file containing lines like '-D NAME=VALUE'")
    ap.add_argument("--keep-code-unexpanded", action="store_true", help="Only resolve conditionals; do not expand macros in code lines")
    args = ap.parse_args()

    macros: Dict[str, Macro] = {}

    # Load -D from CLI
    for d in args.defs:
        name, val = parse_D_arg(d)
        macros[name] = Macro(name=name, params=None, body=val)

    # Load from defs file (your format)
    if args.defs_file:
        with open(args.defs_file, "r", encoding="utf-8", errors="replace") as f:
            macros.update(load_defs_from_text(f.read()))

    with open(args.input, "r", encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines(True)

    out_lines = local_preprocess(
        lines,
        macros,
        expand_in_code=not args.keep_code_unexpanded,
    )

    with open(args.output, "w", encoding="utf-8") as f:
        f.writelines(out_lines)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

