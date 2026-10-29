"""Minimal KiCad s-expression reader/writer."""
import re

class Sym(str):
    """Bare symbol token (unquoted)."""
    pass

_tok = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)

def parse(text):
    pos = 0
    stack = [[]]
    n = len(text)
    while pos < n:
        m = _tok.match(text, pos)
        if not m:
            break
        pos = m.end()
        if m.group(1):
            stack.append([])
        elif m.group(2):
            lst = stack.pop()
            stack[-1].append(lst)
        elif m.group(3) is not None:
            stack[-1].append(m.group(3).replace('\\"', '"'))
        elif m.group(4) is not None:
            t = m.group(4)
            stack[-1].append(Sym(t))
        if pos >= n:
            break
        # skip whitespace
        while pos < n and text[pos].isspace():
            pos += 1
    return stack[0]

def dump(node, indent=0):
    if isinstance(node, list):
        if not node:
            return "()"
        inner = []
        simple = all(not isinstance(x, list) for x in node)
        if simple:
            return "(" + " ".join(dump(x) for x in node) + ")"
        s = "(" + dump(node[0])
        rest = node[1:]
        # keep leading atoms on the same line
        i = 0
        while i < len(rest) and not isinstance(rest[i], list):
            s += " " + dump(rest[i]); i += 1
        for x in rest[i:]:
            s += "\n" + "  " * (indent + 1) + dump(x, indent + 1)
        s += "\n" + "  " * indent + ")"
        return s
    if isinstance(node, Sym):
        return str(node)
    if isinstance(node, str):
        return '"' + node.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'
    if isinstance(node, float):
        return ("%.6f" % node).rstrip('0').rstrip('.') if node != int(node) else str(int(node))
    return str(node)

def find(node, key):
    """First child list whose head == key."""
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None

def findall(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]

_libcache = {}
def load_lib(path):
    if path not in _libcache:
        _libcache[path] = parse(open(path, encoding="utf-8").read())[0]
    return _libcache[path]

def get_symbol(libpath, name):
    lib = load_lib(libpath)
    for s in findall(lib, "symbol"):
        if s[1] == name:
            return s
    raise KeyError(name)

def resolve_symbol(libpath, name):
    """Return a symbol with 'extends' resolved (deep copy), renamed to name."""
    import copy
    s = copy.deepcopy(get_symbol(libpath, name))
    ext = find(s, "extends")
    if ext:
        base = resolve_symbol(libpath, ext[1])
        # merge: base body with derived's properties overriding
        merged = copy.deepcopy(base)
        merged[1] = name
        # drop base properties that derived redefines
        dprops = {p[1]: p for p in findall(s, "property")}
        out = [x for x in merged if not (isinstance(x, list) and x and x[0] == "property" and x[1] in dprops)]
        out = out[:2] + [p for p in findall(s, "property")] + [x for x in out[2:] if not (isinstance(x, list) and x and x[0] == "property")] + [x for x in out[2:] if isinstance(x, list) and x and x[0] == "property"]
        # rename sub-units from base name to derived name
        for u in findall(out, "symbol"):
            u[1] = name + u[1][len(base[1]):]
        s = out
    return s

def symbol_pins(sym):
    """Return list of dicts: unit, number, name, x, y, rot, length, type (lib coords, y up)."""
    pins = []
    for u in findall(sym, "symbol"):
        m = re.match(r'.*_(\d+)_(\d+)$', u[1])
        unit = int(m.group(1)) if m else 0
        for p in findall(u, "pin"):
            at = find(p, "at"); ln = find(p, "length")
            pins.append(dict(unit=unit, number=find(p, "number")[1], name=find(p, "name")[1],
                             x=float(at[1]), y=float(at[2]), rot=float(at[3]) if len(at) > 3 else 0.0,
                             length=float(ln[1]) if ln else 0.0, type=str(p[1])))
    return pins
