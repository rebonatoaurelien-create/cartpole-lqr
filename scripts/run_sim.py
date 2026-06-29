import subprocess
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

script_dir = Path(__file__).resolve().parent

project_root = script_dir.parent
exe_path = project_root / "build" / "simulation"

csv_path = project_root / "simulation.csv"

subprocess.run([str(exe_path)], check=True, cwd=project_root)

print("simulation finished")

data = pd.read_csv(csv_path, delimiter=",")
variables = ["x","dx", "theta", "dtheta"]

plt.figure()
for var in variables:
    plt.plot(data["time"].values, data[var].values, label=var)
plt.legend()
plt.title("Cart pole state")
plt.show()