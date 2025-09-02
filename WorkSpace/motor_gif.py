# %%
# Re-generate with lighter settings to avoid timeouts.
import os

# Single-phase (lighter)
from matplotlib.animation import PillowWriter
print("Generating single-phase...")
try:
    make_single_phase(path="/mnt/data/scene1_single_phase.gif", fps=20, duration_s=4)
    print("Done single-phase:", os.path.exists("/mnt/data/scene1_single_phase.gif"))
except Exception as e:
    print("Single-phase generation error:", e)

# Three-phase (lighter)
print("Generating three-phase...")
try:
    make_three_phase(path="/mnt/data/scene2_three_phase.gif", fps=20, duration_s=4)
    print("Done three-phase:", os.path.exists("/mnt/data/scene2_three_phase.gif"))
except Exception as e:
    print("Three-phase generation error:", e)
