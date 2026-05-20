# Physics Expt Development Guide & Preferences

Welcome to the **Physics Expt** development reference guide. This document centralizes key technical decisions, user preferences, and implementation architecture learned across sessions to maintain long-term codebase health.

---

## 🌍 User Preferences

1. **High Physical Realism & Zero Magic Items**:
   * Presets must rely on realistic structural mechanics. Never use "floating pegs", visual-only guide joints, or floating physics hacks in presets.
   * If parts are aligned on a specific plane (e.g. `Y = -0.25`), ensure *all* relevant objects share this exact coordinate to enable perfect physical contact and prevent clipping.
2. **Explicit Physics Explanations**:
   * Arcane settings (like `contype`/`conaffinity` for collision, sliding friction coefficients, and mechanical joint equality constraints) should never feel like black boxes.
   * Expose properties transparently with clear labels and interactive `(i)` info icons that link directly to the on-screen reference guide.
3. **Autonomous Execution with Direct Collaboration**:
   * Never execute automated browser testing scripts during pair-programming sessions. Always present layout changes, UI enhancements, and functional updates directly for manual approval.

---

## 🛠️ Architecture & Core Mechanics

### 1. ⚙️ Proximity-Aware Joint Coupling (Pinion & Gears)
* **The Goal**: Direct rigid-body collision between complex teeth gears in discrete simulators causes severe jittering, interlocking, and physics explosions. To counter this, we use mathematical joint constraints (`<equality><joint ... /></equality>`).
* **The Proximity Rule**: Hard-coded constraints locked pinion/rack movement even when separated in 3D space. We resolved this with **Proximity Coupling**:
  * The system computes the absolute world positions of gears and pinions relative to parent bodies during compiler compilation.
  * **Pinion-Rack**: Coupled if and only if center-to-center distance $\le 0.5$ meters.
  * **Gears**: Coupled if and only if center-to-center distance $\le (r_1 + r_2) \times 1.15$ (touching/meshing distance).
* **Configurable Toggle**: Each node exposes an `allowCoupling` property. Users can uncheck "Allow Mechanical Coupling" in the sidebar to bypass the proximity solver and test direct physical contact!

### 2. 🪐 Gravity and DOF (Degrees of Freedom)
* **Static Bodies**: Nodes with `joints: []` have infinite mass and are welded directly to the world frame. Gravity has no effect on them.
* **Hinge Joints**: constrained to 1D axial rotation. Symmetrical components have their center of mass exactly on the axis pivot, which yields zero gravity-torque.
* **Free Joints**: Complete 6-DOF unconstrained movement, yielding free-fall acceleration.

### 3. 💥 Solid vs Ephemeral Collisions
* Map directly to MuJoCo's `contype` and `conaffinity` attributes.
* **Solid (Solid check active)**: `contype="1" conaffinity="1"`. Enters the contact solver.
* **Ephemeral (Solid check inactive)**: `contype="0" conaffinity="0"`. Visual guide only; other bodies pass through freely.

### 4. 🔄 3D Multi-Axis Rotation (Euler representation)
* Redesigned to accept explicit array updates along `[X, Y, Z]` axes.
* Stale quaternion representation (`node.quat`) is deleted dynamically to let the MuJoCo compiler resolve rotation purely from the custom Euler array, preventing canvas lag or value conflicts.

---

## 📂 Key Files & Directories

* `src/App.tsx`: Main user interface, Sidebar, Environment configuration, and Documentation Modal.
* `src/store/useStore.ts`: State management and scene node mutation actions.
* `src/utils/mjcf.ts`: Compiles the Zustand scene node graph into high-fidelity MJCF XML code.
* `src/presets/presetScenes.ts`: Initial scene definitions and configurations.
