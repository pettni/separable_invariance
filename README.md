# InvSepSynth

Matlab code for synthesis of separable invariant sets from the paper

 - Petter Nilsson and Necmiye Ozay, ***Synthesis of separable controlled invariant sets for modular local control design***, in *Proceedings of the American Control Conference*, pp. 5656-5663, 2016

## Classes

 - `SepDyn`: class for representing linear systems coupled through the dynamics, together with input, disturbance, and state constraints.

## Functions
 - `solvelmi`: solve the synthesis problem
 - `check_sol`: verify that a solution is correct
 - `plot_sol`: plot a solution

## Example usage

```
addpath('/path/to/library/')
cd example_rotations
rotations
```

## Dependencies

 - Yalmip and a compatible solver (SeDuMi, Mosek)
 - MPT (http://people.ee.ethz.ch/~mpt/3/) for plotting
