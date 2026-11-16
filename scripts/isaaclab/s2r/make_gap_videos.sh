#!/bin/bash
# Record the dynamics-gap videos (videos/s2r). Statistics in the captions come from
# scripts/isaaclab/s2r/scenario_fall_rate.py on the same scenario (logs/s2r_study/video_scenario_fall_rate_v2*.log)
# and from the final gap evaluation (logs/s2r_study/eval_final).
# usage: bash scripts/isaaclab/s2r/make_gap_videos.sh [seed] [parts, e.g. 123 or 3]
cd "$(dirname "$0")/../../.."
seed=${1:-0}
parts=${2:-123}
R=logs/rsl_rl
ckpt() { echo "$(ls -d $R/s2r_${1,,}/*_seed42_ext | tail -1)/model_11998.pt"; }
out=videos/s2r
record() { uv run --project scripts/isaaclab python -u scripts/isaaclab/s2r/record_gap_video.py --headless --seed $seed --out_dir "$@" 2>&1 | grep -E "RESULT|INFO\] fell|Traceback|Error:"; }

trained_fixed="Trained on the nominal dynamics only (no randomization)"
train050="Trained on: torso ±0.75 kg, legs ±10%, CoM ±1.5 cm, friction 0.6–1.0"
train100="Trained on: torso ±1.5 kg, legs ±20%, CoM ±3 cm, friction 0.4–1.2"
train200="Trained on: torso −2.5…+3 kg, legs ±40%, CoM ±6 cm, friction 0.1–1.6"

# 1. fixed-dynamics policy, torso mass gap
if [[ $parts == *1* ]]; then
title0="DR000: trained on fixed (nominal) dynamics"
declare -A scen=([-2]="0 of 32 runs fell (0%)" [0]="1 of 32 runs fell (3%)" [1]="3 of 32 runs fell (9%)"
                 [2]="12 of 32 runs fell (38%)" [3]="31 of 32 runs fell (97%)")
declare -A study=([-2]="0%" [0]="0%" [1]="0%" [2]="31%" [3]="81%")
for m in -2 0 1 2 3; do
  if [ $m -eq 0 ]; then name=1_DR000_torso_0kg; else name=$(printf "1_DR000_torso_%+dkg" $m); fi
  record $out/1_fixed_dynamics --policy $(ckpt DR000) --title "$title0" --torso_mass_add $m --name $name \
    --stat "$trained_fixed|This scenario with sensor noise: ${scen[$m]}|Study evaluation (random commands): ${study[$m]} fell"
done

fi

# 2. same gap (torso +2.75 kg), without and with dynamics randomization
if [[ $parts == *2* ]]; then
record $out/2_randomization_vs_fixed --policy $(ckpt DR000) --title "$title0" --torso_mass_add 2.75 --name 2_DR000_torso_+2.75kg \
  --stat "$trained_fixed|This scenario with sensor noise: 60 of 64 runs fell (94%)"
record $out/2_randomization_vs_fixed --policy $(ckpt DR100) --title "DR100: trained with dynamics randomization, width α = 1" \
  --torso_mass_add 2.75 --name 2_DR100_torso_+2.75kg --stat "$train100|This scenario with sensor noise: 4 of 64 runs fell (6%)"

fi

# 3. walking quality on the nominal dynamics vs randomization width
if [[ $parts == *3* ]]; then
q() { echo "Study evaluation, nominal dynamics (128 runs):|yaw-rate error $1 rad/s, speed error $2 m/s"; }
record $out/3_quality_vs_width --policy $(ckpt DR000) --title "$title0" --name 3_DR000_nominal --graph \
  --stat "$trained_fixed|$(q 0.088 0.064)"
record $out/3_quality_vs_width --policy $(ckpt DR050) --title "DR050: trained with dynamics randomization, width α = 0.5" \
  --name 3_DR050_nominal --graph --stat "$train050|$(q 0.090 0.066)"
record $out/3_quality_vs_width --policy $(ckpt DR100) --title "DR100: trained with dynamics randomization, width α = 1" \
  --name 3_DR100_nominal --graph --stat "$train100|$(q 0.101 0.067)"
record $out/3_quality_vs_width --policy $(ckpt DR200) --title "DR200: trained with dynamics randomization, width α = 2" \
  --name 3_DR200_nominal --graph --stat "$train200|$(q 0.147 0.077)"
fi
