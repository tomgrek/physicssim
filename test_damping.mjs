import load_mujoco from '@mujoco/mujoco';
const pendulumXML = `
<mujoco model="pendulum">
  <worldbody>
    <body name="pole" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0" damping="0" />
      <geom name="pole_geom" type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
    </body>
  </worldbody>
</mujoco>
`;
load_mujoco().then((mujoco) => {
  const model = mujoco.MjModel.from_xml_string(pendulumXML);
  const data = new mujoco.MjData(model);
  
  // No damping
  for (let i = 0; i < 100; i++) mujoco.mj_step(model, data);
  const v1 = data.qvel[0];
  
  // Reset
  data.qpos[0] = 0; data.qvel[0] = 0;
  mujoco.mj_step(model, data);
  
  // Set damping high
  model.dof_damping[0] = 1000.0;
  for (let i = 0; i < 100; i++) mujoco.mj_step(model, data);
  const v2 = data.qvel[0];
  
  console.log("No damping velocity:", v1);
  console.log("High damping velocity:", v2);
});
