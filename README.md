# ConvexMPC (MuJoCo, from scratch)

A from-scratch MuJoCo project that re-implements Convex MPC by studying and reconstructing the design of MIT Cheetah-Software.

Core goals:

1. Reconstruct the system by understanding not only the paper-level ideas, but also the actual software design, timing structure, and constraint construction used in code.
2. Build a modular codebase (`common/`, `robot/`, `sim/`, `controller/`) that is easy to debug, replace, and extend.
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

- `apps/`: application entry points and top-level wiring
- `cmake/`: CMake helper scripts for compiler options and dependency discovery
- `common/`: shared types, utilities, and base interfaces
- `controller/`: control logic and concrete controller implementations
- `robot/`: robot model selection and robot-specific mappings
- `sim/`: MuJoCo integration layer
- `config/`: YAML/JSON parameter files
- `models/`: MuJoCo XML models and assets
- `tests/`: unit and integration tests
- `scripts/`: experiment and automation scripts

---

## 5) Dependency rules (fixed)

From a layered perspective, the structure is:

- `L0`: `common`
- `L1`: `robot`
- `L2`: `sim`
- `L3`: `controller`
- `L4`: `apps`

The allowed dependencies are:

1. `common -> robot`
2. `common, robot -> sim`
3. `common, robot -> controller`
4. `common, robot, sim, controller -> apps`

Interpretation:

- `sim` is the only directory allowed to depend directly on MuJoCo.
- Shared project types should live in `common`.
- Dependencies should stay top-down and acyclic.

---

## 6) Recommended implementation order

1. `common + robot + sim`: establish the minimum loop that reads state and writes commands
2. `controller`: connect a first feedback controller
3. `tests/unit`: add minimal unit tests for each directory

---

## 7) Cleanup candidates in the current structure

- If `simulate/` is essentially a copied MuJoCo sample viewer, consider either moving it to `third_party/mujoco_simulate/` or removing it.
- In the long run, unify include namespaces under `include/convexmpc/<module>/...`.

---

## 8) Directory responsibilities and MIT mapping

### `common`

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

### `robot`

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

### `sim`

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

### `controller`

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

- A dedicated layer is needed to convert higher-level control outputs into safe and usable actuator commands.
