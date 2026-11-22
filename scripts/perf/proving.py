#!/usr/bin/env python3
"""The proving ground's battery for the game's building model (game state only): every shipped
ram pass (scripts/perf/ram*.json) at every one of its contacts, one local run each, and their table.

  scripts/perf/proving.py OUT              run every contact OUT lacks, then print the table
  scripts/perf/proving.py OUT --only STEM  the same for one pass file (repeatable), by its stem
  scripts/perf/proving.py OUT --table      print the table of what OUT holds, running nothing

Each run is `collapse.sh --target local --rehearsal --ram PASS --contact NAME` into
OUT/STEM/NAME, so a battery stopped part-way resumes where it stopped and a failed contact is run
again. A kept result.json drops its per-tick `ticks` and `rows`, which the run's server.log holds
(about 40 MB a run otherwise). A row is one run: compare two batteries row by row, never a row with its neighbour as if
one run were a distribution (scripts/perf/collapse.md)."""

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import collapse

ROOT = collapse.ROOT
COLUMNS = ("pass", "contact", "contact m/s", "least forward m/s", "worst tick change m/s", "crossing s",
           "in place", "opening m2", "end")


def row(stem, name, folder, keep):
    """One run's row, from its result.json (collapse.py, HULL_DEFINITIONS); a run that left none
    reads as not run. Unless `keep`, the file is rewritten without its per-tick lists."""
    path = folder / "result.json"
    if not path.is_file():
        return (stem, name) + ("",) * (len(COLUMNS) - 3) + ("no result",)
    result = json.loads(path.read_text())
    if not keep and (result.pop("ticks", None), result.pop("rows", None)) != (None, None):
        path.write_text(json.dumps(result, indent=2) + "\n")
    crossing, structure = result["crossing_s"], result["structure"]
    return (stem, name, f"{result['impact']['speed_m_s']:.2f}", f"{result['min_forward_m_s']:.2f}",
            f"{result['largest_forward_change_m_s']:+.2f}", "never" if crossing is None else f"{crossing:.2f}",
            f"{100 * structure['in_place_share']:.0f} %", f"{structure['opening_m2']:.1f}", result["end"])


def wait_for_port():
    """Another lane's local run holds the instrument's port (collapse.py binds it): wait it out
    rather than fail this contact."""
    waited = False
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind(("127.0.0.1", 5888))
                return
            except OSError:
                if not waited:
                    print("waiting for another local run to release 127.0.0.1:5888", flush=True)
                waited = True
        time.sleep(5)


def table(rows):
    lines = ["| " + " | ".join(COLUMNS) + " |", "|" + "---|" * len(COLUMNS)]
    return "\n".join(lines + ["| " + " | ".join(r) + " |" for r in rows]) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", type=Path, help="the battery's folder")
    parser.add_argument("--only", action="append", metavar="STEM", help="repeatable: one pass file by its stem (ram-wall-brick)")
    parser.add_argument("--table", action="store_true", help="print the table of what OUT holds; run nothing")
    args = parser.parse_args()
    out = args.out.resolve()
    passes = {path.stem: path for path in sorted((ROOT / "scripts/perf").glob("ram*.json"))}
    unknown = sorted(set(args.only or ()) - set(passes))
    if unknown:
        parser.error(f"no pass named {unknown}; the shipped passes are {sorted(passes)}")
    runs = [(stem, name, out / stem / name) for stem, path in passes.items() if not args.only or stem in args.only
            for name in collapse.ram_recipes(path)]
    failed = []
    for stem, name, folder in runs:
        if args.table or (folder / "result.json").is_file():
            continue
        folder.mkdir(parents=True, exist_ok=True)
        wait_for_port()
        status = subprocess.run([str(ROOT / "scripts/perf/collapse.sh"), "--target", "local", "--rehearsal", "--ram",
                                 str(passes[stem]), "--contact", name, "--out", str(folder)], cwd=ROOT,
                                stdout=(folder / "collapse.log").open("w"), stderr=subprocess.STDOUT).returncode
        print(f"{stem} {name}: {'complete' if status == 0 else f'failed (status {status}); see {folder}/collapse.log'}", flush=True)
        if status:
            failed.append(f"{stem}/{name}")
    text = table([row(*run, keep=args.table) for run in runs])
    if not args.table:
        (out / "table.md").write_text(text)
    print(text, end="")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
