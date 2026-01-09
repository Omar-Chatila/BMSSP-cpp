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

def generate_dot(num_nodes=12000, avg_out_degree=3, filename="biggraph.dot", undirected = False):
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
    undirected = True

    for n in range(1, 8):
        nodes = 10**n

        for deg in [3] + [2**d for d in range(1, 7)]:
            requested_random_edges = nodes * deg
            base_edges = nodes - 1

            if undirected:
                max_edges = nodes * (nodes - 1) // 2
            else:
                max_edges = nodes * (nodes - 1)

            if base_edges + requested_random_edges > max_edges:
                continue

            file = f"benchmarks/graph_{nodes}_{deg}" + ("_undir" if undirected else "")
            generate_dot(nodes, deg, file, undirected)
            print(f"{nodes=} {deg=}")
