#!/usr/bin/env bash
# expert-check.sh: Pollen's alpha_stand (the stand-up's BC teacher) on the old and corrected ground-contact models,
# robot-like (kd 0.07, legs 0.7, no spread). -> runs/eval/standup-bodycontact/expert-check.txt. The checkpoint only
# builds the runner; make_expert_eval.py writes runs/standup_eval_expert.py from microduck_rl's dukki_standup_eval.py.
# Runs on the training host from ~/not_work/dukki/runs/, where this folder's make_expert_eval.py is copied.
. ~/not_work/dukki/env.sh
cd ~/not_work/dukki/microduck_rl
python3 ../runs/make_expert_eval.py
CK=logs/rsl_rl/microduck_stand/2026-11-23_15-11-18_dukki-standup-bodycontact-kp1-kdr/model_8500.pt
OUT=~/not_work/dukki/runs/eval/standup-bodycontact/expert-check.txt; : > $OUT.tmp
export DUKKI_KP=1 DUKKI_KD=0.05 DUKKI_KD_RANGE=0.07,0.07 DUKKI_CALIBRATION_SPREAD=0 DUKKI_LEGS_LOWPASS=0.7 MUJOCO_GL=egl
for cfg in "1 plain 1,1" "1 play 1,4" "0 plain 1,1" "0 play 1,4"; do
  set -- $cfg
  task=Mjlab-StandUp-Flat-Dukki-Distill; [ $2 = play ] && task=Mjlab-StandUp-Flat-Backlash-Dukki-Distill
  echo "== alpha_stand, body contact $1, $2 friction $3" >> $OUT.tmp
  DUKKI_BODY_CONTACT=$1 DUKKI_FRICTION=$3 uv run python ../runs/standup_eval_expert.py --expert --task $task --checkpoint $CK --envs 256 2>&1 \
    | grep -E "got up|Error" >> $OUT.tmp
done
mv $OUT.tmp $OUT
