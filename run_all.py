"""run_all.py -- runs every step in order (about 5 minutes).  Usage:  python run_all.py   (set MPC_NO_SHOW=1 to skip pop-up windows)"""
import subprocess, sys, os, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
for s in ["run_1_components", "run_2_single_gate", "run_3_three_gates", "run_4_baselines", "run_5_animation"]:
    print(f"\n{'=' * 20} {s} {'=' * 20}"); t0 = time.time()
    if subprocess.run([sys.executable, s + ".py"]).returncode: sys.exit(f"{s} failed")
    print(f"({time.time() - t0:.0f}s)")
print("\nAll done -- figures and animation are in the results/ folder.")
