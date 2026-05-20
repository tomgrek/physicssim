import load_mujoco from '@mujoco/mujoco';
const newXml = `
<mujoco model="test">
  <worldbody>
    <body name="pole" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" />
      <geom type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
    </body>
    <body name="box" pos="0 0 2">
      <joint name="free" type="free" />
      <geom type="box" size="0.2 0.2 0.2" />
    </body>
  </worldbody>
</mujoco>
`;

load_mujoco().then((m) => {
  const newModel = m.MjModel.from_xml_string(newXml);
  const newData = new m.MjData(newModel);
  console.log("qpos after init:", newData.qpos);
});
