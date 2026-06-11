#!/usr/bin/env bash
# Run cuVSLAM on all EuRoC datasets (with and without IMU) and generate a report.
#
# Usage:
#   ./run_all_euroc.sh [OPTIONS]
#
# Options:
#   --euroc_dir DIR   Root directory containing EuRoC sequences (default: /mnt/wdssd/Euroc)
#   --bin BIN         Path to euroc_test binary (default: <repo>/build/bin/euroc_test)
#   --report FILE     Output report file (default: <euroc_dir>/report.txt)
#   --no_eval         Skip evaluation (just run tracker)
#   --imu_only        Only run with-IMU variant
#   --no_imu_only     Only run no-IMU variant
#   --skip_existing   Skip sequences whose pose file already exists (resume a partial run)
#   --seq SEQ         Run only this sequence name (e.g. MH_01_easy); repeat for multiple

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EUROC_DIR="/mnt/wdssd/Euroc/"
BIN="$SCRIPT_DIR/../../build/bin/euroc_test"
GT_SCRIPT="$SCRIPT_DIR/euroc_gt_to_tum.py"
EVAL_SCRIPT="$SCRIPT_DIR/evaluate_tum.py"
ANALYZE_SCRIPT="$SCRIPT_DIR/analyze_runs.py"
REPORT_FILE=""
RUN_EVAL=true
RUN_IMU=true
RUN_NO_IMU=true
SKIP_EXISTING=false
FILTER_SEQS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --euroc_dir)     EUROC_DIR="$2";   shift 2 ;;
    --bin)           BIN="$2";         shift 2 ;;
    --report)        REPORT_FILE="$2"; shift 2 ;;
    --no_eval)       RUN_EVAL=false;   shift ;;
    --imu_only)      RUN_NO_IMU=false; shift ;;
    --no_imu_only)   RUN_IMU=false;    shift ;;
    --skip_existing) SKIP_EXISTING=true; shift ;;
    --seq)           FILTER_SEQS+=("$2"); shift 2 ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

BIN=$(realpath "$BIN")
[[ -z "$REPORT_FILE" ]] && REPORT_FILE="$EUROC_DIR/report.txt"

if [[ ! -x "$BIN" ]]; then
  echo "ERROR: euroc_test binary not found or not executable: $BIN"
  exit 1
fi

# Collect sequences: if --seq filters given, use those; otherwise scan euroc_dir
SEQUENCES=()
if [[ ${#FILTER_SEQS[@]} -gt 0 ]]; then
  for name in "${FILTER_SEQS[@]}"; do
    d="$EUROC_DIR/$name"
    if [[ -d "$d/mav0" ]]; then
      SEQUENCES+=("$d/")
    else
      echo "ERROR: Sequence '$name' not found or missing mav0/ under $EUROC_DIR"
      exit 1
    fi
  done
else
  for d in "$EUROC_DIR"/*/; do
    [[ -d "$d/mav0" ]] && SEQUENCES+=("$d")
  done
fi

if [[ ${#SEQUENCES[@]} -eq 0 ]]; then
  echo "ERROR: No EuRoC sequences found under $EUROC_DIR"
  exit 1
fi

START_TIME=$(date '+%Y-%m-%d %H:%M:%S')
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  cuVSLAM EuRoC Benchmark  —  $START_TIME"
echo "  Binary  : $BIN"
echo "  Datasets: ${#SEQUENCES[@]}  ($(basename -a "${SEQUENCES[@]}" | tr '\n' ' '))"
echo "  Report  : $REPORT_FILE"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ── helpers ──────────────────────────────────────────────────────────────────

run_tracker() {
  local dataset="$1" poses_file="$2" use_imu="$3" state_file="${4:-}"
  if $SKIP_EXISTING && [[ -f "$poses_file" ]]; then
    echo "  [tracker] use_imu=$use_imu → $(basename "$poses_file") (skipped, already exists)"
    return
  fi
  echo "  [tracker] use_imu=$use_imu → $(basename "$poses_file")"
  local extra_args=()
  if [[ -n "$state_file" ]]; then
    extra_args+=(--state_file "$state_file")
  fi
  "$BIN" --dataset "$dataset" --poses_file "$poses_file" --use_imu="$use_imu" \
    "${extra_args[@]}" 2>&1 | grep -E "Done\.|Tracking lost|frame [0-9]" | tail -2 || true
}

# Run evaluate_tum.py and capture all metrics
evaluate() {
  local gt_tum="$1" est_tum="$2"
  python3 "$EVAL_SCRIPT" "$gt_tum" "$est_tum" 2>&1 || true
}

extract_metric() {
  # extract_metric "ATE" "mean" from evaluate output
  local output="$1" section="$2" key="$3"
  echo "$output" | grep "^$section" | grep -oP "$key=\K[0-9.]+" || echo "N/A"
}

# ── per-sequence arrays ───────────────────────────────────────────────────────
declare -a SEQ_NAMES
declare -a IMU_ATE_MEAN IMU_ATE_MAX IMU_ATE_99 IMU_RPE_T IMU_RPE_R
declare -a NOIMU_ATE_MEAN NOIMU_ATE_MAX NOIMU_ATE_99 NOIMU_RPE_T NOIMU_RPE_R

# ── main loop ─────────────────────────────────────────────────────────────────
for seq_dir in "${SEQUENCES[@]}"; do
  seq_name=$(basename "$seq_dir")
  echo "── $seq_name ──────────────────────────────────────────────"

  # Convert GT once
  gt_tum="$seq_dir/gt_pose_tum.txt"
  if [[ ! -f "$gt_tum" ]]; then
    echo "  Converting GT → $gt_tum"
    python3 "$GT_SCRIPT" --dataset "$seq_dir"
  fi

  # with IMU
  imu_eval=""
  if $RUN_IMU; then
    with_imu="$seq_dir/with_imu.txt"
    state_file="$seq_dir/state.txt"
    run_tracker "$seq_dir" "$with_imu" true "$state_file"
    if $RUN_EVAL; then
      imu_eval=$(evaluate "$gt_tum" "$with_imu")
      echo "$imu_eval" | sed 's/^/    /'
    fi
    # Run state analysis if state file was produced
    if [[ -f "$state_file" ]]; then
      gt_csv="$seq_dir/mav0/state_groundtruth_estimate0/data.csv"
      if [[ -f "$gt_csv" ]]; then
        analysis_png="$seq_dir/analysis.png"
        echo "  [analyze] → $(basename "$analysis_png")"
        python3 "$ANALYZE_SCRIPT" --state "$state_file" --gt "$gt_csv" --output "$analysis_png" 2>&1 | sed 's/^/    /'
      fi
    fi
  fi

  # without IMU
  noimu_eval=""
  if $RUN_NO_IMU; then
    no_imu="$seq_dir/no_imu.txt"
    run_tracker "$seq_dir" "$no_imu" false
    if $RUN_EVAL; then
      noimu_eval=$(evaluate "$gt_tum" "$no_imu")
      echo "$noimu_eval" | sed 's/^/    /'
    fi
  fi

  SEQ_NAMES+=("$seq_name")
  IMU_ATE_MEAN+=("$(extract_metric "$imu_eval"   "ATE" "mean")")
  IMU_ATE_MAX+=("$(extract_metric  "$imu_eval"   "ATE" "max")")
  IMU_ATE_99+=("$(extract_metric   "$imu_eval"   "ATE" "99%")")
  IMU_RPE_T+=("$(extract_metric    "$imu_eval"   "RPE(t.)" "rmse")")
  IMU_RPE_R+=("$(extract_metric    "$imu_eval"   "RPE(r.)" "rmse")")


  NOIMU_ATE_MEAN+=("$(extract_metric "$noimu_eval" "ATE" "mean")")
  NOIMU_ATE_MAX+=("$(extract_metric  "$noimu_eval" "ATE" "max")")
  NOIMU_ATE_99+=("$(extract_metric   "$noimu_eval" "ATE" "99%")")
  NOIMU_RPE_T+=("$(extract_metric    "$noimu_eval" "RPE(t.)" "rmse")")
  NOIMU_RPE_R+=("$(extract_metric    "$noimu_eval" "RPE(r.)" "rmse")")


  echo ""
done

# ── report ────────────────────────────────────────────────────────────────────
if $RUN_EVAL; then
  {
    echo "cuVSLAM EuRoC Benchmark Report"
    echo "Generated : $START_TIME"
    echo "Binary    : $BIN"
    echo ""
    echo "ATE = Absolute Trajectory Error (m, after Umeyama alignment)"
    echo "RPE = Relative Pose Error  (t: m,  r: deg,  delta=10 frames)"
    echo ""
    printf "%-22s │ %34s │ %34s\n" \
      "" "── with IMU ──────────────────" "── no IMU ───────────────────"
    printf "%-22s │ %8s %8s %8s %8s │ %8s %8s %8s %8s\n" \
      "Sequence" "ATE_mean" "ATE_max" "ATE_99%" "RPE_t" \
                 "ATE_mean" "ATE_max" "ATE_99%" "RPE_t"
    printf "%-22s─┼─%34s─┼─%34s\n" \
      "──────────────────────" "──────────────────────────────────" \
      "──────────────────────────────────"

    for i in "${!SEQ_NAMES[@]}"; do
      printf "%-22s │ %8s %8s %8s %8s │ %8s %8s %8s %8s\n" \
        "${SEQ_NAMES[$i]}" \
        "${IMU_ATE_MEAN[$i]}"  "${IMU_ATE_MAX[$i]}"  "${IMU_ATE_99[$i]}"  "${IMU_RPE_T[$i]}" \
        "${NOIMU_ATE_MEAN[$i]}" "${NOIMU_ATE_MAX[$i]}" "${NOIMU_ATE_99[$i]}" "${NOIMU_RPE_T[$i]}"
    done

    printf "%-22s─┴─%34s─┴─%34s\n" \
      "──────────────────────" "──────────────────────────────────" \
      "──────────────────────────────────"
  } | tee "$REPORT_FILE"

  echo ""
  echo "Report saved to: $REPORT_FILE"
fi
