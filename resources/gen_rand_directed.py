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

import sys
import random

def generate_dot(num_nodes=12000, avg_out_degree=3, filename="biggraph.dot"):
    with open(filename, "w") as f:

        nodes = [f"{i}" for i in range(num_nodes)]
        edges = set()   # speichert (u, v)

        # Ensure weak connectivity by linking i -> i+1
        for i in range(num_nodes - 1):
            u, v = nodes[i], nodes[i + 1]
            w = random.randint(1, 20)
            f.write(f"{u},{v},{w}\n")
            edges.add((i, i+1))

        # Add extra random edges (ohne Duplikate)
        num_edges = num_nodes * avg_out_degree
        added = 0

        while added < num_edges:
            u = random.randint(0, num_nodes - 1)
            v = random.randint(0, num_nodes - 1)

            if u == v:
                continue

            edge = (u, v)
            if (u, v) in edges or (v, u) in edges:
                continue

            w = random.randint(1, 50)
            f.write(f"{edge[0]},{edge[1]},{w}\n")
            edges.add(edge)
            added += 1

    print(f"DOT file with {num_nodes} nodes written to {filename}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python generate_big_dot.py output.dot [num_nodes] [avg_out_degree]")
        sys.exit(1)

    filename = sys.argv[1]
    num_nodes = int(sys.argv[2]) if len(sys.argv) >= 3 else 12000
    avg_out_degree = int(sys.argv[3]) if len(sys.argv) >= 4 else 3

    generate_dot(num_nodes, avg_out_degree, filename)
