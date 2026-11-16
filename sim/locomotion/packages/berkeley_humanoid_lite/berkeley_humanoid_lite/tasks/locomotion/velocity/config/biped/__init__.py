"""Base configuration of the Velocity-ASMR-Biped tasks (config/asmr_biped). Not registered as a task: it keeps the
shared observations, rewards and randomisation that the ASMR configuration extends.

The old task names Velocity-Berkeley-Humanoid-Lite-Biped-* are no longer registered. Use the Velocity-ASMR-Biped-* names.
"""

from . import agents, env_cfg  # noqa: F401
