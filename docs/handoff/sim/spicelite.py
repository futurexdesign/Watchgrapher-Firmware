#!/usr/bin/env python3
"""spicelite: a small SPICE-subset circuit simulator (modified nodal analysis).

Supported netlist elements (SPICE syntax, case-insensitive):
  Rxxx n1 n2 value
  Cxxx n1 n2 value
  Lxxx n1 n2 value
  Vxxx n+ n- [DC v] [AC mag [phase]] [PULSE(...) | SIN(...) | PWL(...)]
  Ixxx n+ n- (same source syntax; current flows n+ -> n- through the source)
  Exxx n+ n- nc+ nc- gain          (VCVS)
  Gxxx n+ n- nc+ nc- gm            (VCCS, current n+ -> n- through the source)
  Dxxx anode cathode model         (.model name D(IS=.. N=..))
  Xxxx nodes... subckt             (.subckt name pins ... / .ends)
  .include file
Other dot-lines (.tran, .ac, .noise, .end, .options) are ignored here; analyses are run from Python.

Analyses: op() DC operating point (Newton), ac(freqs, src) small-signal, tran(tstop, h) backward
Euler with Newton per step, noise(...) via adjoint-free per-source transfer functions.
"""
import math, re, os
import numpy as np

SUFFIX = {"t": 1e12, "g": 1e9, "meg": 1e6, "k": 1e3, "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12, "f": 1e-15}
VT = 0.025852  # thermal voltage at 300 K
KB, TEMP = 1.380649e-23, 300.0


def val(s):
    s = s.strip().lower()
    m = re.match(r"^([-+]?[0-9]*\.?[0-9]+(?:e[-+]?[0-9]+)?)(meg|[tgkmunpf])?[a-z]*$", s)
    if not m:
        raise ValueError(f"bad value {s!r}")
    return float(m.group(1)) * SUFFIX.get(m.group(2) or "", 1.0)


class Source:
    def __init__(self, toks):
        self.dc, self.acmag, self.acph, self.kind, self.args = 0.0, 0.0, 0.0, None, []
        s = " ".join(toks)
        s = re.sub(r"\s*\(\s*", "(", s)
        m = re.search(r"(pulse|sin|pwl)\(([^)]*)\)", s, re.I)
        if m:
            self.kind = m.group(1).lower()
            self.args = [val(a) for a in re.split(r"[\s,]+", m.group(2).strip()) if a]
            s = s[:m.start()] + s[m.end():]
        t = s.split()
        i = 0
        while i < len(t):
            w = t[i].lower()
            if w == "dc":
                self.dc = val(t[i + 1]); i += 2
            elif w == "ac":
                self.acmag = val(t[i + 1]); i += 2
                if i < len(t) and re.match(r"^[-+0-9.]", t[i]):
                    self.acph = val(t[i]); i += 1
            else:
                self.dc = val(t[i]); i += 1
        if self.kind == "pwl":
            a = self.args
            self.pwl_t = np.array(a[0::2]); self.pwl_v = np.array(a[1::2])
        if self.kind and self.dc == 0.0:
            self.dc = self.at(0.0)

    def at(self, t):
        a = self.args
        if self.kind is None:
            return self.dc
        if self.kind == "pulse":
            v1, v2, td, tr, tf, pw, per = (a + [0] * 7)[:7]
            tr = tr or 1e-9; tf = tf or 1e-9
            if t < td:
                return v1
            tt = (t - td) % per if per else t - td
            if tt < tr:
                return v1 + (v2 - v1) * tt / tr
            if tt < tr + pw:
                return v2
            if tt < tr + pw + tf:
                return v2 + (v1 - v2) * (tt - tr - pw) / tf
            return v1
        if self.kind == "sin":
            vo, va, f = a[0], a[1], a[2]
            td = a[3] if len(a) > 3 else 0.0
            ph = a[5] if len(a) > 5 else 0.0
            if t < td:
                return vo + va * math.sin(math.radians(ph))
            return vo + va * math.sin(2 * math.pi * f * (t - td) + math.radians(ph))
        if self.kind == "pwl":
            return float(np.interp(t, self.pwl_t, self.pwl_v))
        return self.dc


class Circuit:
    def __init__(self, path=None, text=None):
        self.models, self.subckts, self.elems = {}, {}, []
        lines = self._read(path, text)
        self._parse(lines)
        self._index()

    # ------------------------------------------------------------ parsing
    def _read(self, path, text):
        base = os.path.dirname(path) if path else "."
        raw = open(path).read() if path else text
        out = []
        for ln in raw.splitlines():
            ln = ln.split(";")[0].rstrip()
            if not ln.strip() or ln.lstrip().startswith("*"):
                continue
            if ln.lstrip().startswith("+") and out:
                out[-1] += " " + ln.lstrip()[1:]
                continue
            if ln.lower().startswith(".include"):
                f = ln.split(None, 1)[1].strip().strip('"')
                out += self._read(os.path.join(base, f), None)
                continue
            out.append(ln.strip())
        return out

    def _parse(self, lines):
        cur = None
        for ln in lines:
            low = ln.lower()
            if low.startswith(".subckt"):
                t = ln.split()
                cur = (t[1].lower(), [p.lower() for p in t[2:]], [])
                continue
            if low.startswith(".ends"):
                self.subckts[cur[0]] = cur
                cur = None
                continue
            if low.startswith(".model"):
                m = re.match(r"\.model\s+(\S+)\s+(\w+)\s*\((.*)\)", ln, re.I)
                params = dict((k.lower(), val(v)) for k, v in re.findall(r"(\w+)\s*=\s*([^\s,)]+)", m.group(3)))
                self.models[m.group(1).lower()] = params
                continue
            if low.startswith("."):
                continue
            if cur is not None:
                cur[2].append(ln)
            else:
                self.elems += self._elem(ln, "", {})

    def _elem(self, ln, prefix, nmap):
        t = ln.split()
        name = prefix + t[0].lower()
        kind = t[0][0].lower()

        def n(x):
            x = x.lower()
            if x in ("0", "gnd"):
                return "0"
            return nmap.get(x, prefix + x)
        if kind in "rcl":
            return [(kind, name, n(t[1]), n(t[2]), val(t[3]))]
        if kind in "vi":
            return [(kind, name, n(t[1]), n(t[2]), Source(t[3:]))]
        if kind in "eg":
            return [(kind, name, n(t[1]), n(t[2]), n(t[3]), n(t[4]), val(t[5]))]
        if kind == "d":
            return [("d", name, n(t[1]), n(t[2]), t[3].lower())]
        if kind == "x":
            sub = self.subckts[t[-1].lower()]
            pins = [n(x) for x in t[1:-1]]
            m2 = dict(zip(sub[1], pins))
            out = []
            for l2 in sub[2]:
                out += self._elem(l2, name + ".", m2)
            return out
        raise ValueError(f"unsupported element: {ln}")

    def _index(self):
        nodes = []
        for e in self.elems:
            for x in self._nodes_of(e):
                if x != "0" and x not in nodes:
                    nodes.append(x)
        self.nodes = nodes
        self.ni = {x: i for i, x in enumerate(nodes)}
        self.br = {}
        k = len(nodes)
        for e in self.elems:
            if e[0] in "vel":
                self.br[e[1]] = k
                k += 1
        self.N = k
        self.byname = {e[1]: e for e in self.elems}

    @staticmethod
    def _nodes_of(e):
        if e[0] in "eg":
            return e[2:6]
        return e[2:4]

    # ------------------------------------------------------------ helpers
    def idx(self, node):
        return -1 if node == "0" else self.ni[node]

    def set_value(self, name, value):
        name = name.lower()
        for i, e in enumerate(self.elems):
            if e[1] == name:
                self.elems[i] = e[:-1] + (value,)
                self.byname[name] = self.elems[i]
                return
        raise KeyError(name)

    def _stamp_lin(self, G, Cm, gmin=1e-15):
        """linear conductance matrix G and storage matrix Cm (KCL rows, branch eqns)"""
        def add(M, r, c, v):
            if r >= 0 and c >= 0:
                M[r, c] += v
        for e in self.elems:
            k = e[0]
            if k == "r":
                a, b, g = self.idx(e[2]), self.idx(e[3]), 1.0 / e[4]
                add(G, a, a, g); add(G, b, b, g); add(G, a, b, -g); add(G, b, a, -g)
            elif k == "c":
                a, b, c = self.idx(e[2]), self.idx(e[3]), e[4]
                add(Cm, a, a, c); add(Cm, b, b, c); add(Cm, a, b, -c); add(Cm, b, a, -c)
            elif k in "vl":
                a, b, j = self.idx(e[2]), self.idx(e[3]), self.br[e[1]]
                add(G, a, j, 1); add(G, b, j, -1); add(G, j, a, 1); add(G, j, b, -1)
                if k == "l":
                    Cm[j, j] -= e[4]
            elif k == "e":
                a, b, c, d, j = self.idx(e[2]), self.idx(e[3]), self.idx(e[4]), self.idx(e[5]), self.br[e[1]]
                add(G, a, j, 1); add(G, b, j, -1); add(G, j, a, 1); add(G, j, b, -1)
                add(G, j, c, -e[6]); add(G, j, d, e[6])
            elif k == "g":
                a, b, c, d, gm = self.idx(e[2]), self.idx(e[3]), self.idx(e[4]), self.idx(e[5]), e[6]
                add(G, a, c, gm); add(G, a, d, -gm); add(G, b, c, -gm); add(G, b, d, gm)
        for i in range(len(self.nodes)):
            G[i, i] += gmin

    def _src_vec(self, t=None, dc_only=False):
        b = np.zeros(self.N)
        for e in self.elems:
            if e[0] == "v":
                b[self.br[e[1]]] += e[4].dc if (t is None or dc_only) else e[4].at(t)
            elif e[0] == "i":
                v = e[4].dc if (t is None or dc_only) else e[4].at(t)
                a, c = self.idx(e[2]), self.idx(e[3])
                if a >= 0: b[a] -= v
                if c >= 0: b[c] += v
        return b

    def _diodes(self, x, G, b):
        for e in self.elems:
            if e[0] != "d":
                continue
            m = self.models[e[4]]
            Is, N = m.get("is", 1e-14), m.get("n", 1.0)
            a, c = self.idx(e[2]), self.idx(e[3])
            vd = (x[a] if a >= 0 else 0) - (x[c] if c >= 0 else 0)
            nvt = N * VT
            vd_c = min(vd, 1.2)
            ex = math.exp(vd_c / nvt)
            idd = Is * (ex - 1) + (vd - vd_c) * Is * ex / nvt
            gd = Is * ex / nvt + 1e-12
            ieq = idd - gd * vd
            for (r, cc, v) in ((a, a, gd), (c, c, gd), (a, c, -gd), (c, a, -gd)):
                if r >= 0 and cc >= 0:
                    G[r, cc] += v
            if a >= 0: b[a] -= ieq
            if c >= 0: b[c] += ieq

    def _limit(self, xnew, xold):
        """damp node-voltage steps that are large (helps the diode clamps converge)"""
        d = xnew - xold
        nn = len(self.nodes)
        big = np.max(np.abs(d[:nn])) if nn else 0
        if big > 0.5:
            return xold + d * (0.5 / big)
        return xnew

    # ------------------------------------------------------------ analyses
    def op(self, x0=None, maxit=200):
        G0 = np.zeros((self.N, self.N)); Cm = np.zeros((self.N, self.N))
        self._stamp_lin(G0, Cm)
        b0 = self._src_vec()
        x = np.zeros(self.N) if x0 is None else x0.copy()
        # source stepping for robustness
        for scale in (0.25, 0.5, 0.75, 1.0):
            for it in range(maxit):
                G = G0.copy(); b = b0 * scale
                self._diodes(x, G, b)
                xn = np.linalg.solve(G, b)
                xn = self._limit(xn, x)
                if np.max(np.abs(xn - x)) < 1e-9:
                    x = xn
                    break
                x = xn
        self.xop = x
        return x

    def ac(self, freqs, src, out_nodes):
        """small-signal response to AC=1 on source `src` (all other AC sources zeroed)"""
        x = self.xop if hasattr(self, "xop") else self.op()
        G = np.zeros((self.N, self.N)); Cm = np.zeros((self.N, self.N))
        self._stamp_lin(G, Cm)
        self._diodes(x, G, np.zeros(self.N))
        b = np.zeros(self.N, dtype=complex)
        e = self.byname[src.lower()]
        if e[0] == "v":
            b[self.br[e[1]]] = 1.0
        elif e[0] == "i":
            a, c = self.idx(e[2]), self.idx(e[3])
            if a >= 0: b[a] -= 1
            if c >= 0: b[c] += 1
        res = {n: np.zeros(len(freqs), dtype=complex) for n in out_nodes}
        for k, f in enumerate(freqs):
            A = G + 1j * 2 * math.pi * f * Cm
            xs = np.linalg.solve(A, b)
            for n in out_nodes:
                res[n][k] = self._v(xs, n)
        return res

    def ac_inject(self, freqs, inj, out_node):
        """response at out_node to a unit current injected from node a to node b (a->b through the source)"""
        x = self.xop if hasattr(self, "xop") else self.op()
        G = np.zeros((self.N, self.N)); Cm = np.zeros((self.N, self.N))
        self._stamp_lin(G, Cm)
        self._diodes(x, G, np.zeros(self.N))
        a, c = self.idx(inj[0]), self.idx(inj[1])
        b = np.zeros(self.N, dtype=complex)
        if a >= 0: b[a] -= 1
        if c >= 0: b[c] += 1
        out = np.zeros(len(freqs), dtype=complex)
        for k, f in enumerate(freqs):
            xs = np.linalg.solve(G + 1j * 2 * math.pi * f * Cm, b)
            out[k] = self._v(xs, out_node)
        return out

    def tran(self, tstop, h, record, x0=None, tstart_rec=0.0, every=1):
        G0 = np.zeros((self.N, self.N)); Cm = np.zeros((self.N, self.N))
        self._stamp_lin(G0, Cm)
        Ch = Cm / h
        x = (self.xop if hasattr(self, "xop") else self.op()) if x0 is None else x0.copy()
        has_d = any(e[0] == "d" for e in self.elems)
        nsteps = int(round(tstop / h))
        ts, rec = [], {n: [] for n in record}
        A_lin = G0 + Ch
        for k in range(1, nsteps + 1):
            t = k * h
            bs = self._src_vec(t) + Ch @ x
            if has_d:
                xi = x.copy()
                for it in range(60):
                    A = A_lin.copy(); b = bs.copy()
                    self._diodes(xi, A, b)
                    xn = np.linalg.solve(A, b)
                    xn = self._limit(xn, xi)
                    if np.max(np.abs(xn - xi)) < 1e-7:
                        xi = xn
                        break
                    xi = xn
                x = xi
            else:
                x = np.linalg.solve(A_lin, bs)
            if t >= tstart_rec and k % every == 0:
                ts.append(t)
                for n in record:
                    rec[n].append(self._v(x, n))
        return np.array(ts), {n: np.array(v) for n, v in rec.items()}

    def _v(self, x, n):
        n = n.lower()
        if n.startswith("i(") :
            return x[self.br[n[2:-1]]]
        return 0.0 if n == "0" else x[self.ni[n]]

    def resistors(self):
        return [e for e in self.elems if e[0] == "r"]


# ---------------------------------------------------------------- self tests
def _selftest():
    import cmath
    ok = True
    # RC low-pass corner
    c = Circuit(text="V1 in 0 AC 1\nR1 in out 1k\nC1 out 0 159.155n\n")
    c.op()
    r = c.ac([1000.0], "v1", ["out"])["out"][0]
    ok &= abs(abs(r) - 1 / math.sqrt(2)) < 1e-4
    # LR high-pass corner (output across L)
    c = Circuit(text="V1 in 0 AC 1\nR1 in out 1k\nL1 out 0 159.155m\n")
    c.op()
    r = c.ac([1000.0], "v1", ["out"])["out"][0]
    ok &= abs(abs(r) - 1 / math.sqrt(2)) < 1e-4
    # series RLC resonance and Q
    c = Circuit(text="V1 in 0 AC 1\nR1 in a 10\nL1 a b 1m\nC1 b 0 1u\n")
    c.op()
    f0 = 1 / (2 * math.pi * math.sqrt(1e-3 * 1e-6))
    r = c.ac([f0], "v1", ["b"])["b"][0]
    ok &= abs(abs(r) - math.sqrt(1e-3 / 1e-6) / 10) < 1e-3 * 3.16
    # RC step response at t = tau
    c = Circuit(text="V1 in 0 PULSE(0 1 0 1n 1n 1 2)\nR1 in out 1k\nC1 out 0 1u\n")
    c.op()
    t, v = c.tran(2e-3, 1e-7, ["out"])
    k = np.argmin(abs(t - 1e-3))
    ok &= abs(v["out"][k] - (1 - math.exp(-1))) < 2e-3
    # diode forward drop
    c = Circuit(text="V1 in 0 5\nR1 in a 1k\nD1 a 0 dd\n.model dd D(IS=1e-14 N=1)\n")
    x = c.op()
    vd = c._v(x, "a")
    ok &= 0.6 < vd < 0.75
    # VCVS non-inverting gain-of-11 with ideal op amp
    c = Circuit(text="V1 in 0 AC 1\nE1 out 0 in fb 1e6\nR1 out fb 10k\nR2 fb 0 1k\n")
    c.op()
    r = c.ac([100.0], "v1", ["out"])["out"][0]
    ok &= abs(abs(r) - 11) < 1e-3
    return ok


if __name__ == "__main__":
    print("self-test", "PASS" if _selftest() else "FAIL")
