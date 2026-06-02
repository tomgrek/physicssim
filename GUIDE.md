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

---

## 🚀 LQR Control Law & Non-Minimum Phase Dynamics (Cartpole)

### 1. 🧬 Hierarchical Dynamic Modeling in MuJoCo
* **Body Tree Structure**: Reconstructing components as parent-child body relationships (e.g., `pole` parent capsule body + `pole_weight` child sphere body) rather than a single body with multiple geoms enables modular editing in the scene tree but alters the multi-body dynamic tree in MuJoCo. 
* **Automatic Mass & Inertia Integration**: MuJoCo automatically computes composite body mass, center of mass, and the multi-body inertia matrix at the hinge level of the parent body.

### 2. 🧭 Coordinate Alignment & Sign Consistency
* **Right-Handed System**: In MuJoCo, $+X$ is right, $+Y$ is forward (into screen), and $+Z$ is up.
* **Hinge Rotation**: For a hinge rotating about the Y-axis `[0, 1, 0]`, a positive rotation $\theta > 0$ tilts the pole to the **RIGHT (+X)**.
* **Corrective Sign**: To catch a pole tilting right ($\theta > 0$), the cart must accelerate to the right ($F > 0$). Thus, angle gains $k_\theta$ and $k_\omega$ must be **positive**.

### 3. 🎯 Non-Minimum Phase Centering Feedback
* **The Centering Paradox**: In non-minimum phase systems, trying to force the cart to the center ($x > 0 \implies F < 0$) fights the catching force. To return left to the center, the cart must first accelerate *further to the right* to tilt the pole to the left, and then ride that tilt back.
* **Optimal Gains**: Through high-frequency grid simulation searching, we discovered that positive position gains ($k_x > 0, k_v > 0$) combined with dominant vertical tracking gains ($k_\theta > 0, k_\omega > 0$) create highly stable, centering asymptotic decay:
  * **$k_x = 22.0$** (optimized centering stiffness)
  * **$k_v = 15.0$** (smooth cart velocity damping)
  * **$k_\theta = 80.0$** (dominant vertical catch)
  * **$k_\omega = 20.0$** (angular swing rate damping)

---

## 💻 Practical WSL Development & Simulation Workflow

### 1. 🖧 Windows/WSL Path Mappings
* **Path Resolution**: The workspace files are accessed in Windows via UNC paths (`\\wsl$\Ubuntu-20.04\home\boab\physics`). However, Windows-based `npm`/`npx` tools will fail with UNC path errors. All dev servers and scripts must be run directly inside the native WSL Linux file system (`/home/boab/physics`).
* **Environment Execution**: To ensure Node and local tools resolve paths cleanly, run terminal operations via an interactive login shell inside WSL:
  ```bash
  wsl bash -i -l -c "npm run dev"
  ```

### 2. 🚀 Executing TypeScript & ESM in WSL Node
* **ESM Compatibility**: Standard Node v20.20.0 in WSL cannot natively strip or load `.ts` module imports inside `.mjs` scripts (returning `ERR_UNKNOWN_FILE_EXTENSION`).
* **Execution Standard**: Use `npx tsx` inside the WSL interactive environment to run diagnostic scratch scripts seamlessly:
  ```bash
  wsl bash -i -l -c "npx tsx scratch/test_your_script.mjs"
  ```

### 3. 🔍 Grid Search & Decay Evaluation Workflow
* **Early Evaluation Failures**: In MuJoCo simulation grid searches, immediately terminate steps when the state escapes reasonable bounds ($|x| > 1.9$m or $|\theta| > 0.6$ rad) to speed up searches from milliseconds to microseconds per candidate.
* **Interval-Based Decay Analysis**: To distinguish between unstable limit-cycle oscillations and perfect asymptotic centering decay, split simulations into 10-second intervals (e.g. Phase 1: 0-10s, Phase 2: 10-20s, Phase 3: 20-30s). A set of gains is strictly stable/centering if:
  $$\text{Max}(|x|)_{\text{Phase 3}} < \text{Max}(|x|)_{\text{Phase 2}} < \text{Max}(|x|)_{\text{Phase 1}}$$
  $$\text{Max}(|\theta|)_{\text{Phase 3}} < \text{Max}(|\theta|)_{\text{Phase 2}} < \text{Max}(|\theta|)_{\text{Phase 1}}$$

---

## 🧊 Mesh Geom Coordinate System

Mesh geoms (`type: 'mesh'`) use inline `vertices` and `faces` arrays and are rendered differently from primitive geoms. Hard-won discoveries:

### Rendering path
- Primitive geoms (`box`, `sphere`, `capsule`, etc.) live inside a `<group rotation={[-π/2, 0, 0]}>` that converts MuJoCo Z-up → Three.js Y-up. Their positions come from `geom_xpos` (updated every frame via `useFrame`).
- Mesh geoms are rendered **outside** that rotated group, with vertices interpreted as raw Three.js world-space coordinates.
- `useFrame` must **not** update mesh geom positions — their vertices already encode world position. The fix: `if (type === 'mesh') return;` at the top of the `useFrame` callback.

### Axis convention for mesh vertices
Vertices are in standard **Three.js Y-up world space**:
- **X** = left/right
- **Y** = up/down (height)
- **Z** = front/back (depth)

This is the same space the `<Grid>` lives in (at `position={[0, -0.01, 0]}`).

### box() helper — confirmed working signature
```ts
// box(cx, cy, cz, hx, hy, hz)
// cx/hx = left-right,  cy/hy = up/height,  cz/hz = front-back
box(0, 0.3, 0,  4.8, 0.06, 0.3)  // flat road: wide in X, thin in Y, shallow in Z
box(-2, 1.5, 0,  0.08, 1.5, 0.08) // vertical post: tall in Y, narrow in X and Z
```

### Common mistakes
- Do **not** apply any coordinate conversion to mesh vertices — they go straight into Three.js `BufferGeometry`.
- Do **not** put mesh geoms inside the `rotation={[-π/2,0,0]}` group — they're already in the right space.
- `useFrame` **must** bail early for mesh type (`if (type === 'mesh') return`) — otherwise it overwrites the group position every frame with `geom_xpos` (MuJoCo Z-up coords), scrambling the mesh.
- Mesh groups must have no `ref`, no `initialPos`, no `quaternion` applied — just `<group><mesh geometry={...} /></group>`.
- `pos`, `quat`, `euler` fields on a mesh `SceneGeom` are ignored for rendering (they only affect MuJoCo physics). Bake all positions into the vertex data.

### Mesh geoms are VISUAL ONLY — not simulated

**Mesh geoms do not participate in physics simulation.** Their vertex data is used only by the Three.js renderer; it is never read back from MuJoCo at runtime. Consequences:

- A mesh body **will not move** even if it has a free joint — the rendered vertices are frozen at their authored positions.
- Mesh geoms **do not collide** with anything during simulation (MuJoCo does register the mesh asset for collision, but the visual representation never updates to reflect body motion).
- Use meshes only for **static scenery** that will never move: backgrounds, decorative structures, architectural detail.
- For anything that needs to move, bounce, sway, or collide visually — use primitive geoms (`box`, `sphere`, `capsule`, `cylinder`, `ellipsoid`).

**Useful applications for mesh:**
- Static environment dressing (buildings, terrain features, decorative bridges)
- High-detail visual shells around a simpler primitive collision proxy
- Reference geometry or scale indicators that don't interact with the scene

### Ellipsoid rendering
Ellipsoids are rendered as a unit sphere scaled by `[rx, ry, rz]` (the three semi-axes from `geom_size`). This means normals are distorted under non-uniform scale — lighting looks slightly off on very squashed shapes, but it's visually acceptable and physically correct (MuJoCo uses the real ellipsoid for collision).

