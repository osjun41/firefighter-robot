"""Base configuration of the Velocity-ASMR-Biped-S2R tasks (config/asmr_biped_s2r). Not registered as a task: the
ASMR configuration extends the dynamics randomisation defined here.

The old task names Velocity-Berkeley-Humanoid-Lite-Biped-* are no longer registered. Use the Velocity-ASMR-Biped-* names.
"""

from . import agents, env_cfg  # noqa: F401
