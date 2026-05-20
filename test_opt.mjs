import load_mujoco from '@mujoco/mujoco';
const pendulumXML = `
<mujoco model="pendulum">
  <worldbody>
    <body name="pole" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0" damping="0.1" />
      <geom name="pole_geom" type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
    </body>
  </worldbody>
</mujoco>
`;
load_mujoco().then((mujoco) => {
  const model = mujoco.MjModel.from_xml_string(pendulumXML);
  model.dof_damping[0] = 0.5;
  console.log("dof_damping after mutate:", model.dof_damping[0]);
  model.delete();
});
