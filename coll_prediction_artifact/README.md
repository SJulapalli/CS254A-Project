# KUKA collision data for Vortex


## 1. Install the planner's dependencies

From the top of the repo:

```
cd coll_prediction_artifact/trace_generation/bit_planning
pip install torch --extra-index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt --no-build-isolation --extra-index-url https://download.pytorch.org/whl/cpu
```

## 2. Run planner

```
mkdir -p logfiles_BIT_link
python3 eval_all.py 2000 BIT*
```