# Implicit Cloth Simulation with Gradient Descent

A gentle introduction to optimization-based implicit time integration for [CSCI 5611](https://umtc.catalog.prod.coursedog.com/courses/8103481).

_This is a work in progress. I released this around 2021 and it hasn't seen many updates. Please let me know if you have suggestions: mattoverby@gmail.com_

## Introduction

Cloth simulation is standard coursework for undergraduate computer graphics.
Many assignments focus on explicit methods like [forward Euler](https://en.wikipedia.org/wiki/Euler_method).
While efficient and simple, explicit integrators are notoriously unstable for large time steps and high stiffness.
Implicit methods, e.g., [backward Euler](https://en.wikipedia.org/wiki/Backward_Euler_method), offer a solution to that problem.
The standard [Baraff & Witkin approach](https://doi.org/10.1145/280814.280821) (B&W) forms a Taylor approximation to the Eqs. of motion, i.e., Newton's method.
This results in a block-sparse linear system for which the velocity deltas are solved via [conjugate gradient](https://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf).

The B&W method is conceptually straight forward. However, there are numerous gotchas not apparent in the original references.
In particular, working out the Jacobians of some models can be nontrivial. Even when correct, the derivatives can still produce hard-to-debug instabilities.
The [dynamic deformables course notes](https://doi.org/10.1145/3388769.3407490) describes this well, and I highly recommend reviewing the later chapters on cloth simulation for a thorough review of the topic.
Nonetheless, implicit time integration remains a valuable and sometimes necessary tool for modern graphics applications.

Recently, optimization-based methods have grown in popularity. This approach formulates the Eqs. of motion as an [iteratively minimized objective function](https://en.wikipedia.org/wiki/Optimization_problem).
These methods provide unparalleled robustness and extensibility, e.g., [state of the art solvers](https://doi.org/10.1145/3450626.3459767) for cloth simulation.
The implementation is often easier to construct piece-by-piece than B&W-style integrators.

Below we describe a simple implicit solver for mass-spring systems using gradient descent.
This is meant solely as a tutorial to optimization-based time stepping, and not for practical applications.
For that, you'll want to consider higher-order optimization algorithms or [local-global type solvers](https://github.com/alecjacobson/computer-graphics-mass-spring-systems).

## Implementation

The objective we aim to minimize is the sum of internal elastic energy potentials plus a quadratic penalty of linear momentum:

$`\bar{x} = x + hv + h^2 M^{-1} f_{ext}`$,

$`g(x) = \frac{1}{2h^2}\|M^{1/2}(x-\bar{x})\|^2 + \sum E(x)`$,

for vertex locations $x$, time step (sec) $h$, diagonal mass matrix $M$, velocities $v$, and external forces (gravity, wind, etc...)
$`f_{ext}`$. The term $`\bar{x}`$ is the *explicit predictor*
that is computed at the beginning of the time step. Computing a frame of animation amounts to iteratively minimizing the above objective until some convergence criteria is met
(e.g., the norm of the gradient is below some threshold).

### Interface

Below we lay out the steps to produce an optimization-based solver for implicit cloth simulation.
The dependencies will be fetched through a [DownloadProject](https://github.com/Crascit/DownloadProject) CMake script.
[Eigen](https://eigen.tuxfamily.org) is used for linear algebra, which is an industry standard library for vector math.
Rendering and mesh I/O is handled with [libigl](https://libigl.github.io) which also contains many excellent routines for mesh processing.
In libigl, meshes are stored as two dense matrices, an nx3 matrix for vertices and an mx3 matrix for faces.
Animating the mesh is done through a pre-draw callback in which we call our solver to update the vertex positions.
It takes a little finesse to improve the rendering, so we'll save that for another day.

To compile, execute the following commands:
1. mkdir build
2. cd build
3. cmake ..
4. make -j

### Energies

Whenever I write a new simulator, I start with the most basic elastic energy as the deformable primitives: a Hookean spring with stiffness $k$ and rest length $`\ell`$. The potential energy of a spring is $`(k/2)(\|x_a-x_b\|-\ell)^2`$.
These springs are created from the unique edges of the triangle mesh, with bending springs created between adjacent triangles.
The [ClothMesh class](src/ClothMesh.hpp) generates this data given vertex and face buffers.
We can also evaluate temporary springs to deal with collisions. One end point is the vertex, and the other is the projection onto the surface it's penetrating.

### Optimization

[Gradient descent](https://en.wikipedia.org/wiki/Gradient_descent) is a simple first-order method that steps along the negative gradient w.r.t. $x$ at each iteration: $`p = -\nabla g(x_i)`$. We use $i$ to denote the iteration and $p$ as the descent direction. By all accounts, gradient descent is a poor choice due to its low rate of convergence. You can certainly improve performance with other higher-order optimizers. However, I personally advocate starting with gradient descent when writing new optimization code. The optimizer is so simple it's hard to make a mistake - if there are bugs, they are surely in your energy or gradient calculations. This can be a helpful debugging strategy!

Each iteration, we have to make sure we aren't overshooting the objective and increase the energy. This is accomplished by scaling the descent direction with a scalar $s$ using [line search](https://en.wikipedia.org/wiki/Line_search). All together, an iteration of gradient descent involves:

$`p = -\nabla g(x_i)`$,

$`s = linesearch(x_i, p)`$,

$`x_{i + 1} = x_i + s * p`$.

Both energy and gradient use similar calculations so you can save resources (and implementation efforts) by computing them both at same time. I like to use one function for both and skip gradient calculation with a conditional during line search. You can see the evaluation of the objective and gradient in the [Objective class](src/Objective.hpp), with gradient descent in [Solver](src/Solver.hpp).

### Common Problems
With these pieces you have enough to implement an implicit cloth solver. But there are a number of common problems to be aware of if your solver starts behaving erratically:
- Using masses that aren't scaled by the area of the mesh
- The cloth mesh is unrealistically large or small: Each unit of the implementation should correspond to one meter or centimeter. Whatever you choose, be consistent.
- The time step is too large or small (it should be somewhere between 1/1000 and 1/20)
- Forgetting to resize or initialize matrix values
- Dividing by zero and error propogation: A single zero-length spring can produce instabilities, as its result is passed all the way up the food chain to the solver. Using asserts, guards, and std::isfinite any time you do a division will save you a lot of trouble (see the [Objective class](src/Objective.hpp) for an example).

## Next Steps

There are many ways to improve the solver illustrated above:
- Parallelizing the objective and gradient calculation (I recommend [TBB](https://software.intel.com/en-us/oneapi/onetbb))
- Collision against triangle meshes with a [signed distance field](https://github.com/InteractiveComputerGraphics/TriangleMeshDistance)
- Better energy models with triangular elements ([Dynamic Deformables appendix D](https://doi.org/10.1145/3388769.3407490))
- Preconditioning or accelerated versions of gradient descent
- Optimizers with better convergence like [L-BFGS](https://en.wikipedia.org/wiki/Limited-memory_BFGS) or [projected Newton](https://doi.org/10.1145/1073368.1073394)
