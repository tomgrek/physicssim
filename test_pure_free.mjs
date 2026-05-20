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
    <geom name="floor" type="plane" size="0 0 0.1" pos="0 0 0" rgba="0.9 0.9 0.9 1" />
    <body name="cube1" pos="0 0 0.3">
      <freejoint name="cube1_free" />
      <geom name="cube1_geom" type="box" size="0.2 0.2 0.2" rgba="0.8 0.2 0.2 1" mass="1" />
    </body>
    <body name="cube2" pos="0 0 1">
      <freejoint name="cube2_free" />
      <geom name="cube2_geom" type="box" size="0.2 0.2 0.2" rgba="0.2 0.8 0.2 1" mass="1" />
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

    console.log("Stepping 100 times...");
    for (let i = 0; i < 100; i++) {
      mujoco.mj_step(model, data);
    }
    console.log("Simulation test succeeded!");

  } catch (err) {
    console.error("Error:", err);
  }
})();
