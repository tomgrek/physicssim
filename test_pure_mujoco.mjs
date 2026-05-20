import load_mujoco from '@mujoco/mujoco';

(async () => {
  try {
    console.log("Loading MuJoCo...");
    const mujoco = await load_mujoco();
    console.log("MuJoCo loaded successfully!");

    const xml = `
<mujoco model="dynamic_scene">
  <option timestep="0.002" gravity="0 0 -9.81" />
  <worldbody>
    <light directional="true" pos="-0.5 0.5 3" dir="0.5 -0.5 -3" diffuse="0.8 0.8 0.8" />
    <body name="cube1" pos="0 0 0">
      <joint name="cube1_hinge" type="hinge" pos="0 0 0" axis="0 1 0" />
      <geom name="cube1_geom" type="capsule" size="0.05" rgba="0.8 0.2 0.2 1" fromto="0 0 0 1 0 0" mass="1" />
    </body>
    <body name="cube2" pos="5 0 0">
      <joint name="cube2_hinge" type="hinge" pos="0 0 0" axis="0 1 0" />
      <geom name="cube2_geom" type="sphere" size="0.15" rgba="0.2 0.8 0.2 1" mass="14" />
    </body>
  </worldbody>
</mujoco>
    `;

    console.log("Compiling model...");
    const model = mujoco.MjModel.from_xml_string(xml);
    console.log("Model compiled successfully! nq:", model.nq, "nv:", model.nv);

    console.log("Creating MjData...");
    const data = new mujoco.MjData(model);
    console.log("MjData created!");

    console.log("Calling mj_forward...");
    mujoco.mj_forward(model, data);
    console.log("mj_forward succeeded!");

    console.log("Stepping 10 times...");
    for (let i = 0; i < 10; i++) {
      console.log(`Step ${i} start`);
      mujoco.mj_step(model, data);
      console.log(`Step ${i} done`);
    }
    console.log("Simulation test succeeded!");

  } catch (err) {
    console.error("Error:", err);
  }
})();
