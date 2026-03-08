# ConvexMPC (MuJoCo, from scratch)

A from-scratch MuJoCo project that re-implements Convex MPC by studying and reconstructing the design of MIT Cheetah-Software.

Core goals:

1. Reconstruct the system by understanding not only the paper-level ideas, but also the actual software design, timing structure, and constraint construction used in code.
2. Build a modular codebase (`modules/*`) that is easy to debug, replace, and extend.
3. Preserve a structure that can be re-tuned for Cheetah 3 physical parameters.

---

## 1) Recommended reading order in the MIT codebase

To quickly understand the end-to-end data flow, read the MIT code in the following order:

1. Entry point: `user/MIT_Controller/main.cpp`
2. Main loop: `user/MIT_Controller/MIT_Controller.cpp`
3. FSM execution: `user/MIT_Controller/FSM_States/ControlFSM.cpp`
4. Locomotion state: `user/MIT_Controller/FSM_States/FSM_State_Locomotion.cpp`
5. Convex MPC core: `user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp`
6. QP formulation and solve: `user/MIT_Controller/Controllers/convexMPC/SolverMPC.cpp`
7. Final torque/leg command application: `common/src/Controllers/LegController.cpp`

In short:

`FSM -> Locomotion -> ConvexMPCLocomotion -> SolverMPC -> leg command`

---

## 2) Key implementation points to learn from this project

1. Timing structure
   - The control loop period and the MPC update period (`iterationsBetweenMPC`) are separated.
   - The QP is not solved at every control tick.
2. Contact schedule construction
   - The gait table (0/1 contact pattern) is built over the full prediction horizon and injected directly into the constraints.
3. Problem formulation
   - The QP is built from a linearized/discretized model, friction-cone and normal-force constraints, and costs for state tracking and force regularization.
4. Swing/stance split
   - Stance uses MPC force feedforward, while swing uses trajectory tracking (typically PD-based).
5. Practical implementation details
   - The code contains important deviations from the clean paper equations, such as numerical stabilization, clamping, and branch-specific logic.

---

## 3) Important caution (for Cheetah 3)

The public MIT Convex MPC code follows the paper-level structure, but some constants and tuning choices are closer to the Mini Cheetah setup.

- Reusing values such as mass/inertia or force limits (`f_max`) without adjustment can lead to poor behavior on Cheetah 3.
- Before running, verify at least:
  - robot mass and inertia tensor
  - leg force limits
  - target body height and gait frequency
  - friction coefficient and ground/contact model

In other words, the algorithmic structure can be reused as a reference, but the numerical parameters must be identified and re-tuned separately.

---

## 4) Current repository structure

- `apps/`: application entry points and top-level module wiring
- `cmake/`: CMake helper scripts for compiler options and dependency discovery
- `config/`: YAML/JSON parameter files
- `models/`: MuJoCo XML models and assets
- `modules/`: core libraries
- `tests/`: unit and integration tests
- `scripts/`: experiment and automation scripts

### Module summary

| module | role |
|---|---|
| `common` | common types, utilities, parameters, and base interfaces |
| `robot` | robot model layer (FK/Jacobian/physical parameters) |
| `sim` | MuJoCo I/O adapter (sensor/actuator bridge) |
| `estimator` | state and contact estimation |
| `planner` | gait, swing, and reference generation |
| `mpc` | Convex MPC QP construction and solver interface |
| `controller` | stance/swing composition and final command generation |
| `fsm` | mode execution, transition logic, and safety rules |

---

## 5) Dependency rules (fixed)

From a layered perspective, the structure is:

- `L0`: `common`
- `L1`: `robot`
- `L2`: `sim / estimator / planner / mpc`
- `L3`: `controller`
- `L4`: `fsm`
- `L5`: `apps`

The allowed dependencies are:

1. `common -> robot`
2. `common, robot -> sim / estimator / planner / mpc`
3. `common, robot, estimator, planner, mpc -> controller`
4. `common, controller -> fsm`
5. `common, sim, controller, fsm -> apps`

Interpretation:

- `sim` is the only module allowed to depend directly on MuJoCo.
- The control-core modules (`estimator`, `planner`, `mpc`, `controller`, `fsm`) must communicate only through internal project types defined in `common`.
- Dependencies must remain strictly top-down by layer. Reverse references and cyclic dependencies are forbidden.

---

## 6) Recommended implementation order

1. `sim + robot + common`: establish the minimum loop that reads state and writes commands
2. `planner`: generate gait and swing references
3. `mpc`: connect force computation
4. `controller`: combine stance and swing control
5. `fsm`: integrate mode transitions
6. `tests/unit`: add minimal unit tests for each module

---

## 7) Cleanup candidates in the current structure

- If `simulate/` is essentially a copied MuJoCo sample viewer, consider either moving it to `third_party/mujoco_simulate/` or removing it.
- In the long run, unify include namespaces under `include/convexmpc/<module>/...`.

---

## 8) `modules/*` responsibilities and MIT mapping

### `modules/common`

Responsibilities:

- common types (`RobotState`, `Command`, `ContactSchedule`)
- math and coordinate-transform utilities
- parameter loading, logging, and timers

MIT mapping:

- `common/include/cppTypes.h`
- `common/include/Types.h`
- `common/include/Utilities/*`

Key lesson:

- Fixing inter-module interfaces around simple shared types makes it much easier to replace solvers or estimators later.

### `modules/robot`

Responsibilities:

- robot physical parameters (mass, inertia, links)
- FK/Jacobian/foot-frame computation APIs
- model information consumed by the controller and MPC layers

MIT mapping:

- `common/include/Dynamics/Quadruped.h`
- `common/include/Dynamics/Cheetah3.h`
- `common/include/Dynamics/MiniCheetah.h`
- `common/include/Dynamics/FloatingBaseModel.h`

Key lesson:

- The controller code should remain separated from the robot-model implementation if portability matters.

### `modules/sim`

Responsibilities:

- MuJoCo input/output adapter
- `mjData` -> internal state conversion
- internal command -> actuator command mapping
- reset/step/viewer hooks

MIT mapping:

- `sim/src/*`
- `robot/src/SimulationBridge.cpp`
- `robot/src/RobotRunner.cpp`

Key lesson:

- Keeping simulator-specific dependencies in one place makes it much easier to switch later to hardware or another simulator.

### `modules/estimator`

Responsibilities:

- base pose and velocity estimation
- contact-state estimation
- fused state outputs for planner/controller use

MIT mapping:

- `common/include/Controllers/StateEstimatorContainer.h`
- `common/src/Controllers/OrientationEstimator.cpp`
- `common/src/Controllers/PositionVelocityEstimator.cpp`
- `common/src/Controllers/ContactEstimator.cpp`

Key lesson:

- In practice, MPC performance is often limited more by estimator quality than by the optimizer itself.

### `modules/planner`

Responsibilities:

- gait schedule generation
- swing-foot trajectory generation
- body and foot reference generation

MIT mapping:

- `common/src/Controllers/GaitScheduler.cpp`
- `user/MIT_Controller/Controllers/convexMPC/Gait.cpp`
- `common/src/Controllers/FootSwingTrajectory.cpp`
- `user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp` (foot placement logic)

Key lesson:

- Swing foot placement rules (e.g., Raibert-style placement plus yaw compensation) strongly affect overall stability.

### `modules/mpc`

Responsibilities:

- linear model construction and discretization
- cost and constraint matrix construction
- QP solver calls and solution interpretation (force sequence)

MIT mapping:

- `user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp`
- `user/MIT_Controller/Controllers/convexMPC/SolverMPC.cpp`
- `user/MIT_Controller/Controllers/convexMPC/convexMPC_interface.cpp`
- `common/src/SparseCMPC/SparseCMPC.cpp`
- `common/src/SparseCMPC/OsqpTriples.cpp`

Key lesson:

- Separating dense/sparse paths and fixing the solver I/O interface early makes long-term maintenance much easier.

### `modules/controller`

Responsibilities:

- stance/swing control composition
- gain scheduling
- final actuator command generation

MIT mapping:

- `user/MIT_Controller/FSM_States/FSM_State_Locomotion.cpp`
- `common/include/Controllers/LegController.h`
- `common/src/Controllers/LegController.cpp`
- `user/MIT_Controller/Controllers/WBC_Ctrl/*` (optional integration)

Key lesson:

- A dedicated layer is needed to convert MPC outputs (forces) into safe and usable actuator commands (torques/targets).

### `modules/fsm`

Responsibilities:

- execution of mode states (`Idle`, `Stand`, `Locomotion`)
- transition logic and safety checks
- selection of planner/controller behavior per mode

MIT mapping:

- `user/MIT_Controller/FSM_States/ControlFSM.cpp`
- `user/MIT_Controller/FSM_States/FSM_State*.cpp`

Key lesson:

- In full systems, operational logic often matters as much as, or more than, the core control algorithm itself.
