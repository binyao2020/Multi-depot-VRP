# Multi-depot-VRP

This project is the result of an optimization competition held by AIMMS. The project considers vehicle routing problem under multi-depot scenario (more details refer to ./doc/description.pdf). The goal is to (1) decide the optimal number of depots and its location; (2) determine the fleet size of each depot. We developed a routing algorithm based on column generation. To obtain reasonable solution with large instances, we apply random coloring algorithm to heuristically solve the subproblem. More details refer to ./doc/report.pdf.
