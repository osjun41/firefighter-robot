"""Base configuration of the Velocity-ASMR-Biped push tasks (config/asmr_biped_push). Not registered as a task: the
ASMR configuration extends the push events and rewards defined here.

The old task names Velocity-Berkeley-Humanoid-Lite-Biped-* are no longer registered. Use the Velocity-ASMR-Biped-* names.
"""

from . import agents, env_cfg  # noqa: F401
