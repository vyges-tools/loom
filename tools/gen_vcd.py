#!/usr/bin/env python3
"""Generate random VCDs whose toggle counts are known BY CONSTRUCTION.

No reference implementation is consulted: the generator decides every value change, so the
expected count is arithmetic, not an oracle. Emits `<name>.vcd` plus `<name>.expected`
(net<TAB>toggles), for both a scalar and a vector signal, including x/z excursions and a
$dumpoff window -- the semantics that cost us four defects.
"""
import random, sys, os

def gen(seed, path):
    rnd = random.Random(seed)
    nscal = rnd.randint(1, 4)
    nvec = rnd.randint(0, 2)
    vw = rnd.randint(2, 5)
    codes = [chr(33 + i) for i in range(nscal + nvec)]
    scal = [(f"s{i}", codes[i]) for i in range(nscal)]
    vecs = [(f"v{i}", codes[nscal + i], vw) for i in range(nvec)]

    lines = ["$timescale 1ns $end", "$scope module top $end"]
    for n, c in scal:
        lines.append(f"$var wire 1 {c} {n} $end")
    for n, c, w in vecs:
        lines.append(f"$var wire {w} {c} {n} [{w-1}:0] $end")
    lines += ["$upscope $end", "$enddefinitions $end"]

    # last KNOWN value per signal; None = never known
    last = {n: None for n, _ in scal}
    vlast = {n: [None] * w for n, _, w in vecs}
    exp = {f"top.{n}": 0 for n, _ in scal}
    for n, _, w in vecs:
        for b in range(w):
            exp[f"top.{n}[{b}]"] = 0

    lines.append("#0")
    lines.append("$dumpvars")
    for n, c in scal:
        v = rnd.choice("01")
        lines.append(f"{v}{c}")
        last[n] = v
    for n, c, w in vecs:
        bits = "".join(rnd.choice("01") for _ in range(w))
        lines.append(f"b{bits} {c}")
        vlast[n] = list(bits)
    lines.append("$end")

    t = 0
    dumping = True
    for _ in range(rnd.randint(5, 40)):
        t += rnd.randint(1, 10)
        lines.append(f"#{t}")
        # occasionally toggle the dump state; a dumpoff x-flood is NOT activity
        if rnd.random() < 0.1:
            if dumping:
                lines.append("$dumpoff")
                for n, c in scal:
                    lines.append(f"x{c}")
                for n, c, w in vecs:
                    lines.append(f"bx {c}")
                lines.append("$end")
                dumping = False
            else:
                lines.append("$dumpon")
                for n, c in scal:
                    v = last[n] if last[n] else "0"
                    lines.append(f"{v}{c}")
                for n, c, w in vecs:
                    bits = "".join(b if b else "0" for b in vlast[n])
                    lines.append(f"b{bits} {c}")
                lines.append("$end")
                dumping = True
            continue
        if not dumping:
            continue
        for n, c in scal:
            if rnd.random() < 0.5:
                v = rnd.choice("01xz")
                if v in "01":
                    if last[n] is not None and last[n] != v:
                        exp[f"top.{n}"] += 1
                    last[n] = v
                lines.append(f"{v}{c}")
        for n, c, w in vecs:
            if rnd.random() < 0.5:
                bits = [rnd.choice("01" if rnd.random() < 0.85 else "xz") for _ in range(w)]
                for i, b in enumerate(bits):
                    if b in "01":
                        prev = vlast[n][i]
                        if prev is not None and prev != b:
                            # bit i of a [w-1:0] vector is named [w-1-i]
                            exp[f"top.{n}[{w-1-i}]"] += 1
                        vlast[n][i] = b
                # VCD drops leading ZEROS, but a reader extends with the leading character
                # when that is x or z -- so stripping `0x101` down to `x101` would legally
                # mean `xx101`, a different value. A conforming writer keeps a leading 0 in
                # that case. (Our reader was right about this; this generator was not, and
                # the disagreement showed up only on high-order bits.)
                s = "".join(bits)
                while len(s) > 1 and s[0] == "0" and s[1] in "01":
                    s = s[1:]
                lines.append(f"b{s} {c}")
    t += rnd.randint(1, 10)
    lines.append(f"#{t}")
    open(path + ".vcd", "w").write("\n".join(lines) + "\n")
    with open(path + ".expected", "w") as fh:
        for k in sorted(exp):
            fh.write(f"{k}\t{exp[k]}\n")

if __name__ == "__main__":
    out = sys.argv[1]
    n = int(sys.argv[2])
    os.makedirs(out, exist_ok=True)
    for i in range(n):
        gen(i, os.path.join(out, f"g{i:03d}"))
    print(f"generated {n}")
