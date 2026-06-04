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

### 1. 🖧 Windows/WSL Path Mappings & Command Execution
* **Path Resolution**: The workspace files are accessed in Windows via UNC paths (`\\wsl.localhost\Ubuntu-20.04\home\boab\physics`). However, Windows-based `npm`/`npx` tools will fail with UNC path errors or `ERR_INVALID_URL` because standard Windows Node/npm cannot resolve UNC paths natively in CMD.
* **Avoid Host Leakage**: Calling `npm` directly on the Windows host inside a WSL workspace directory can default to running Windows `npm.cmd` via `cmd.exe`, resulting in errors like `'tsc' is not recognized as an internal or external command`.
* **WSL NVM Environment Execution**: Node/npm are managed via NVM inside WSL (e.g. `~/.nvm/`). Because NVM is initialized in `.bashrc` / `.bash_profile`, non-interactive shells cannot resolve node or npm commands. Always run builds, dev servers, and diagnostic scripts using an **interactive** bash shell (`-i`) inside the specific WSL distribution:
  ```bash
  # Execute dev server:
  wsl -d Ubuntu-20.04 -e bash -i -c "cd /home/boab/physics && npm run dev"

  # Run build:
  wsl -d Ubuntu-20.04 -e bash -i -c "cd /home/boab/physics && npm run build"
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

## 🧊 Mesh Geoms — Complete Reference

Mesh geoms (`type: 'mesh'`) come in two modes: **static** (visual only) and **dynamic** (full physics + collision). Both are hard-won — read this carefully before touching mesh code.

---

### Coordinate systems

- **Three.js** is Y-up: X=right, Y=up, Z=toward camera. Ground plane is Y=0.
- **MuJoCo** is Z-up: X=right, Y=forward (into screen), Z=up. Ground plane is Z=0.
- The `mjcf.ts` builder **automatically swaps Y↔Z** when emitting mesh vertices into the `<mesh>` asset XML. So you always author `vertices` in Three.js Y-up space.
- Primitive geoms live inside a `<group rotation={[-π/2, 0, 0]}>` in `App.tsx` that converts MuJoCo Z-up world positions into Three.js Y-up for rendering.

---

### Static mesh geoms (default)

`dynamic` field absent or `false`. Vertices baked in Three.js Y-up world space. The mesh renders at a fixed position — never moves, never collides. Rendered **outside** the rotated group; no `ref`, no `useFrame` tracking.

Good for: scenery, decorative structures, visual shells around primitive collision proxies.

```ts
{ name: 'deck', type: 'mesh', size: [1], rgba: [...], vertices: [...], faces: [...] }
```

The `box()` helper used in `goldenGateMeshPreset` (Three.js Y-up coords):
```ts
// box(cx, cy, cz, hx, hy, hz) — cy/hy = height
box(0, 0.3, 0,  4.8, 0.06, 0.3)  // flat deck: wide in X, thin in Y
box(cx, 1.5, 0, 0.08, 1.5, 0.08) // tall post: large hy
```

---

### Dynamic mesh geoms (`dynamic: true`)

Full physics simulation and collision. MuJoCo takes the **convex hull** of the mesh — concave shapes won't collide correctly as a single mesh.

Requires two extra fields:
```ts
{
  type: 'mesh',
  vertices: [...],        // Three.js Y-up space — mjcf builder swaps Y↔Z for MuJoCo
  faces: [...],
  dynamic: true,
  renderVertices: [...],  // Raw MuJoCo Z-up space (Y↔Z swap only, NO centroid subtraction)
}
```

**Rendering path for dynamic meshes:**
- Rendered **inside** the `rotation={[-π/2, 0, 0]}` group alongside primitives
- Position tracked every frame from `data.xpos[bodyId]` / `data.xmat[bodyId]` — **body** transform
- `renderVertices` are in raw Z-up space; MuJoCo recenters the mesh internally and `xpos` tracks the recentered body frame, so render + physics are automatically aligned

---

### Computing renderVertices — the correct workflow

**Simple**: just swap Y↔Z on each Y-up vertex. No centroid subtraction needed — MuJoCo handles recentering internally.

```js
function toRenderVerts(yupVerts) {
  const out = [];
  for (let i = 0; i < yupVerts.length; i += 3) {
    const x = yupVerts[i], y = yupVerts[i+1], z = yupVerts[i+2];
    out.push(x, -z, y);  // Y-up (x,y,z) → Z-up (x,-z,y)
  }
  return out;
}
```

**Setting body pos**: `body_pos = [0, 0, 0]` places the mesh where its vertices are in Y-up space. To start an object at a specific height, set `body_pos.z` to the desired height of the body's MuJoCo origin (which MuJoCo places at the mesh's volume centroid). For a mesh whose Y-up base is at Y=0 and centroid is at Y=0.125, set `body_pos.z = 0.125` for the base to sit flush with the ground — **or** just set `body_pos.z = 0` and the mesh will render with its MuJoCo origin at ground level (which visually puts the base at Z = -centroid_height, slightly below ground). Empirically verify by checking `xpos[2]` via the `_mesh_xpos` debug object.

**Face winding**: use outward-facing normals (CCW winding viewed from outside). Wrong winding causes inside-out contact normals and sinking behavior.

**Child bodies**: for compound mesh objects (mesh + child bodies), the child `pos` offset is in MuJoCo Z-up relative to the **parent body's MuJoCo origin** (which is the volume centroid, not the mesh base). Measure with `_mesh_xpos` debug log.

**Reference** (from `meshCollisionPreset`):
- Pyramid (base 0.6×0.6×0.6, height 0.5): MuJoCo origin at Z≈0 when resting on floor (`xpos.z ≈ 0`)
- Ramp (fixed): `body_pos = [0,0,0]`, base flush with ground

---

### Adding a new preset — three places to update

1. **`src/presets/presetScenes.ts`**: add the `export const myPreset` and add it to the `PRESETS` map at the bottom.
2. **`src/App.tsx`**: add `<option value="my_preset">My Preset</option>` to the hardcoded `<select>` dropdown (~line 1495). This does NOT auto-populate from `PRESETS`.
3. **`src/store/useStore.ts`**: add `'my_preset'` to the `loadPreset` union type (~line 100).

---

### Keeping MCP Documentation and Schemas in sync

The MCP server dynamically loads tool descriptions and simulator schemas from JSON documentation. When tool capabilities, presets, or schema fields change, you MUST update:
1. **`mcp-docs.json`** at the root of the simulator repository (e.g. `physics/mcp-docs.json`).
2. **`mcp-docs/physics.json`** inside the MCP server repository (`expt_mcp/mcp-docs/physics.json`). This is a fallback copy committed to the MCP repo so users can run it standalone without cloning the simulator.

If you also change the React hook commands:
- Update **`src/hooks/useMCPBridge.ts`** to handle the new command and map it to Zustand store mutations or selectors.

Stale bridge documentation or schemas will cause external agent copilots to generate invalid scene graphs or make broken calls.

---

### Ellipsoid rendering
Ellipsoids are rendered as a unit sphere scaled by `[rx, ry, rz]` (the three semi-axes from `geom_size`). Normals are distorted under non-uniform scale — lighting looks slightly off on very squashed shapes, but physically correct (MuJoCo uses the real ellipsoid for collision).

