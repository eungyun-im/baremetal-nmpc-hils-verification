#!/usr/bin/env python3
"""Static analysis with cppcheck (MISRA C 2012 addon) and clang-tidy.

Findings are compared with verification/static/baseline.json by tool, rule,
file and message, without line numbers, so moving code does not count as new.
Exit 1 when a key appears more often than in the baseline.

  sut/static_analysis.py                    # run and compare
  sut/static_analysis.py --update-baseline  # accept the current findings
"""
import argparse, collections, json, os, re, shutil, subprocess, sys
import xml.etree.ElementTree as ET

SUT = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(SUT)
OUT = os.path.join(SUT, "build_static"); BASE = os.path.join(ROOT, "verification", "static", "baseline.json")
SOURCES = sorted([os.path.join("core/src", f) for f in os.listdir(os.path.join(SUT, "core/src")) if f.endswith(".c")] +
                 ["integ/bmpc_safety.c", "third_party/kuuve_safety/safety_ctrl.c"])
INC = ["-Icore/include", "-Iinteg", "-Ithird_party/kuuve_safety"]
DEFS = ["-DBMPC_USE_FLOAT", "-DBS_PLAUS", "-D_POSIX_C_SOURCE=200809L"]
TIDY = "clang-analyzer-*,bugprone-*,cert-*,performance-*,portability-*,-bugprone-easily-swappable-parameters"


def tool_version(cmd):
    try:
        out = subprocess.run([cmd, "--version"], capture_output=True, text=True).stdout.strip().splitlines()
        return next((l.strip() for l in out if "version" in l.lower() or l.lower().startswith("cppcheck")), out[0] if out else None)
    except OSError:
        return None


def run_cppcheck(exe):
    xml = os.path.join(OUT, "cppcheck.xml")
    cmd = [exe, "--enable=warning,style,performance,portability", "--std=c99", "--addon=misra", "--inline-suppr",
           "--suppress=missingIncludeSystem", "--xml", "--output-file=" + xml] + INC + DEFS + SOURCES
    r = subprocess.run(cmd, cwd=SUT, capture_output=True, text=True)
    if not os.path.exists(xml):
        sys.exit("cppcheck produced no report: " + r.stderr[-500:])
    found = []
    for e in ET.parse(xml).getroot().iter("error"):
        loc = e.find("location")
        if loc is None:
            continue
        rule = e.get("id")
        if rule.startswith("misra"):
            rule = "misra-c2012-" + re.sub(r"^misra-c2012-", "", rule)
        found.append(dict(tool="cppcheck", rule=rule, severity=e.get("severity"), file=os.path.relpath(os.path.join(SUT, loc.get("file")), SUT),
                          line=int(loc.get("line", 0)), message=e.get("msg", "")))
    return found


def run_clang_tidy(exe):
    found = []
    pat = re.compile(r"^(.+?):(\d+):\d+: (warning|error): (.*) \[([^\]]+)\]$")
    for src in SOURCES:
        r = subprocess.run([exe, "-checks=-*," + TIDY, src, "--", "-std=c99"] + INC + DEFS, cwd=SUT, capture_output=True, text=True)
        for line in r.stdout.splitlines():
            m = pat.match(line)
            if m and "clang-diagnostic" not in m.group(5):
                found.append(dict(tool="clang-tidy", rule=m.group(5), severity=m.group(3), file=os.path.relpath(os.path.join(SUT, m.group(1)), SUT),
                                  line=int(m.group(2)), message=m.group(4)))
    uniq = {(f["file"], f["line"], f["rule"], f["message"]): f for f in found}   # headers repeat across sources
    return sorted(uniq.values(), key=lambda f: (f["file"], f["line"], f["rule"]))


def key(f):
    return "%s|%s|%s|%s" % (f["tool"], f["rule"], f["file"], re.sub(r"\d+", "N", f["message"]))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--update-baseline", action="store_true"); a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    cpp, tidy = shutil.which(os.environ.get("CPPCHECK", "cppcheck")), shutil.which(os.environ.get("CLANG_TIDY", "clang-tidy"))
    if not cpp or not tidy:
        sys.exit("cppcheck and clang-tidy are both required")
    found = run_cppcheck(cpp) + run_clang_tidy(tidy)
    versions = dict(cppcheck=tool_version(cpp), clang_tidy=tool_version(tidy))
    counts = collections.Counter(key(f) for f in found)
    json.dump(dict(versions=versions, sources=SOURCES, findings=found), open(os.path.join(OUT, "findings.json"), "w"), indent=1)
    by_rule = collections.Counter((f["tool"], f["rule"]) for f in found)
    print("%d findings, %s" % (len(found), versions))
    for (t, r), n in by_rule.most_common(12):
        print("  %4d  %-10s %s" % (n, t, r))
    if a.update_baseline:
        json.dump(dict(versions=versions, counts=dict(sorted(counts.items()))), open(BASE, "w"), indent=1); open(BASE, "a").write("\n")
        print("baseline updated"); return
    if not os.path.exists(BASE):
        print("no baseline yet, nothing to compare"); return
    base = json.load(open(BASE))
    if base["versions"] != versions:
        print("warning: tool versions differ from the baseline %s" % base["versions"])
    new = {k: n - base["counts"].get(k, 0) for k, n in counts.items() if n > base["counts"].get(k, 0)}
    fixed = sum(max(0, n - counts.get(k, 0)) for k, n in base["counts"].items())
    print("new: %d, fixed since baseline: %d" % (sum(new.values()), fixed))
    for k, n in sorted(new.items()):
        print("  NEW x%d  %s" % (n, k))
    sys.exit(1 if new else 0)


if __name__ == "__main__":
    main()
