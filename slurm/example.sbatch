#!/bin/bash
#SBATCH --job-name=prmkit
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=01:00:00
#SBATCH --output=prmkit-%j.log
set -euo pipefail
cd "${SLURM_SUBMIT_DIR}"
source .venv/bin/activate
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
prmkit run examples/01_single_wall.yaml --output "runs/slurm_${SLURM_JOB_ID}" --set roadmap.workers="${SLURM_CPUS_PER_TASK}"
