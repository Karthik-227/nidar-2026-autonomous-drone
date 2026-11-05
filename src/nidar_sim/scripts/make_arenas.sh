#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
out=../worlds

python3 generate_arena.py --out $out --name nidar_arena_1 --seed 101 --rooms 8 --loop-prob 0.10 --start-cells 3
python3 generate_arena.py --out $out --name nidar_arena_2 --seed 202 --rooms 7 --loop-prob 0.20 --start-cells 3
python3 generate_arena.py --out $out --name nidar_arena_3 --seed 303 --rooms 9 --loop-prob 0.32 --start-cells 3

echo
python3 verify_arena.py $out/*.sdf
