# Runs the naive test on problems 2000 to 2199.
# Prints one line per problem and saves the same info to results/naive_simx.csv.
# Run: cd vortex/tests/robotics && python3 run_all.py

import csv
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "..", "..", "build")
os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
csv_path = os.path.join(HERE, "results", "naive_simx.csv")

with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["problem", "status", "cycles", "cdq", "misses", "extras"])

    for pid in range(2000, 2200):
        data = os.path.join(HERE, "test_data", f"data_{pid}.bin")
        if not os.path.exists(data):
            continue

        out = subprocess.run(
            ["make", "-C", "tests/robotics/naive", "run-simx", f"OPTS=-f {data}"],
            cwd=BUILD, capture_output=True, text=True).stdout

        status = "PASSED" if "PASSED!" in out else "FAILED"
        cycles = re.search(r"cycles=(\d+)", out)
        cdq = re.search(r"cdq=(\d+)", out)
        extras = re.search(r"(\d+) of \d+ motions", out)

        cycles = cycles.group(1) if cycles else "?"
        cdq = cdq.group(1) if cdq else "?"
        extras = extras.group(1) if extras else 0
        misses = out.count("pybullet collided, box free")

        print(f"{pid}: {status}  cycles={cycles} cdq={cdq} misses={misses} extras={extras}")
        w.writerow([pid, status, cycles, cdq, misses, extras])
        f.flush()

print(f"saved {csv_path}")