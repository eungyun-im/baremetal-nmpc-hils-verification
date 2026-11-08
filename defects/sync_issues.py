#!/usr/bin/env python3
"""Create or update GitHub issues from defects/issues/*.md (front matter: title, labels, state).

The files are the source of truth; the issue tracker mirrors them. An issue is matched by the
ID prefix of its title ("DEF-6:"), so running this again updates instead of duplicating.
usage: sync_issues.py [--dry-run]     (needs an authenticated `gh` CLI)
"""
import argparse, glob, json, os, re, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS = {
    "defect": ("d73a4a", "Product defect in firmware or integration code"),
    "test-infra": ("bfdadc", "Defect in the test setup, not in the product"),
    "sev:high": ("b60205", "Safety, security or control quality affected"),
    "sev:medium": ("fbca04", "Wrong behaviour that normal inputs do not reach"),
    "sev:low": ("c2e0c6", "Cosmetic or documentation"),
    "state:open": ("ededed", "Not fixed"),
    "state:fixed-not-on-board": ("fef2c0", "Fixed and unit-tested, not yet run on the S32K312"),
    "state:verified-on-board": ("0e8a16", "Fixed and verified in HILS laps on the board"),
    "found-by:review": ("5319e7", "Code review"),
    "found-by:fuzzing": ("5319e7", "libFuzzer with sanitizers"),
    "found-by:fault-injection": ("5319e7", "HILS fault injection"),
    "found-by:bag-analysis": ("5319e7", "Analysis of recorded runs"),
    "found-by:log-review": ("5319e7", "Test log review"),
}


def gh(*args):
    return subprocess.run(["gh", *args], check=True, capture_output=True, text=True).stdout


def parse(path):
    text = open(path).read()
    m = re.match(r"---\n(.*?)\n---\n(.*)", text, re.S)
    meta, body = m.group(1), m.group(2).strip()
    title = re.search(r'^title: "(.*)"$', meta, re.M).group(1)
    labels = [l.strip().strip('"') for l in re.search(r"^labels: \[(.*)\]$", meta, re.M).group(1).split(",")]
    state = re.search(r"^state: (\w+)$", meta, re.M).group(1)
    body += "\n\n---\n_Mirrored from `defects/issues/%s` by `defects/sync_issues.py`._" % os.path.basename(path)
    return title, labels, state, body


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    items = [parse(p) for p in sorted(glob.glob(os.path.join(HERE, "issues", "*.md")))]
    if a.dry_run:
        for t, l, s, _ in items:
            print(f"{s:6} {t}  {l}")
        return
    have = {l["name"] for l in json.loads(gh("label", "list", "--limit", "200", "--json", "name"))}
    for name, (color, desc) in LABELS.items():
        if name not in have:
            gh("label", "create", name, "--color", color, "--description", desc)
    existing = {i["title"].split(":")[0]: i for i in json.loads(gh("issue", "list", "--state", "all", "--limit", "200", "--json", "number,title,state"))}
    for title, labels, state, body in items:
        key = title.split(":")[0]
        if key in existing:
            num = str(existing[key]["number"])
            gh("issue", "edit", num, "--title", title, "--body", body, "--add-label", ",".join(labels))
            cur = existing[key]["state"].lower()
        else:
            url = gh("issue", "create", "--title", title, "--body", body, "--label", ",".join(labels)).strip()
            num, cur = url.rsplit("/", 1)[1], "open"
        if state == "closed" and cur == "open":
            gh("issue", "close", num)
        elif state == "open" and cur == "closed":
            gh("issue", "reopen", num)
        print(f"#{num} {state:6} {title}")


if __name__ == "__main__":
    main()
