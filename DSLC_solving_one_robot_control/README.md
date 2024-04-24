# DLPC for distributedly solving centralized control: problem -- Policy Training

### Introduction

For this project, you will run the codes in the Matlab environment. The codes verify the optimality gap to the centralized version by solving one robot's centralized path-following control problem distributedly.

### Running the code

- Run `DSLC_solving_centralized_main` to solve the one robot's path-following problem by partitioning the robot model (input-state: 2-5) into two subsystems (1-2 and 1-3).
- Run `centralized_version_main` to generate the results of the centralized version for comparison.
- Run `plot_optimality` to display the results.
