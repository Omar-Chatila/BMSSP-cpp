#!/usr/bin/env python3
"""
generate_big_dot.py
-------------------
Generate a large random directed weighted graph in DOT format.

Usage:
    python generate_big_dot.py output.dot [num_nodes] [avg_out_degree]

Defaults:
    num_nodes       = 12000
    avg_out_degree  = 3

The graph is connected but has random edge directions and weights.
"""

import random
import sys


def generate_csv(num_nodes=12000, avg_out_degree=3, filename="biggraph.dot", undirected = False):
    with open(filename, "w") as f:

        edges = set()

        # Ensure weak connectivity by linking i -> i+1
        for i in range(num_nodes - 1):
            u, v = i, i + 1
            w = random.randint(1, 20)
            f.write(f"{u},{v},{w}\n")
            edges.add((i, i+1))

        # Add extra random edges without duplicates
        num_edges = num_nodes * avg_out_degree
        added = 0

        while added < num_edges:
            u = random.randint(0, num_nodes - 1)
            v = random.randint(0, num_nodes - 1)

            if u == v:
                continue

            edge = (u, v)
            if (u, v) in edges or (undirected and (v, u) in edges):
                continue

            w = random.randint(1, 50)
            f.write(f"{edge[0]},{edge[1]},{w}\n")
            edges.add(edge)
            added += 1

    print(f"DOT file with {num_nodes} nodes written to {filename}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python generate_big_dot.py output.csv [num_nodes] [avg_out_degree] [directed<0,1>]")
        sys.exit(1)

    from pathlib import Path
    script_dir = Path(__file__).resolve().parent

    filename = sys.argv[1]
    nodes = int(sys.argv[2]) if len(sys.argv) >= 3 else 12000
    deg = int(sys.argv[3]) if len(sys.argv) >= 4 else 3
    undirected = True if int(sys.argv[4]) == 0 else False

    requested_random_edges = nodes * deg
    base_edges = nodes - 1

    if undirected:
        max_edges = nodes * (nodes - 1) // 2
    else:
        max_edges = nodes * (nodes - 1)

    if not base_edges + requested_random_edges > max_edges:
        bench_dir = "benchmarks"
        out_file = script_dir / bench_dir / filename
        generate_csv(nodes, deg, str(out_file), undirected)
        print(f"{nodes=} {deg=}")
