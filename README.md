# 3D non-rigid tricubic pointcloud registration

## Introduction

This is an implementation of the method described in the paper "**Non-rigid point cloud registration using piece-wise tricubic polynomials as transformation model**".

The preprint of the paper can be found [here](https://www.preprints.org/manuscript/202310.1120) - it can be cited as:

```
@article{glira2023,
	doi = {10.20944/preprints202310.1120.v1},
	url = {https://doi.org/10.20944/preprints202310.1120.v1},
	year = 2023,
	month = {October},
	publisher = {Preprints},
	author = {Philipp Glira and Christoph Weidinger and Johannes Otepka-Schremmer and Camillo Ressl and Norbert Pfeifer and Michaela Haberler-Weber},
	title = {Non-Rigid Point Cloud Registration Using Piece-Wise Tricubic Polynomials as Transformation Model},
	journal = {Preprints}
}
```

## 2D Prototype

A prototype implementation of this algorithm for 2D point clouds written in Matlab can be found [here](https://github.com/AIT-Assistive-Autonomous-Systems/2D_nonrigid_tricubic_pointcloud_registration).

## Development

A predefined development environment for [VSCode](https://code.visualstudio.com) is provided [here](.devcontainer/). To use this environment, simply open this repository in VSCode and open the devcontainer with the command ``Dev Containers: Reopen in Container``. This will automatically build the development environment and open it in a new VSCode window. In the container the command ``CMake: Build`` can be used to build all targets.