import load_mujoco from '@mujoco/mujoco';

const newXml = `
<mujoco model="test">
  <worldbody>
    <body name="pole" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" />
      <geom type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
    </body>
    <body name="box" pos="Infinity Infinity 2">
      <joint name="free" type="free" />
      <geom type="box" size="0.2 0.2 0.2" />
    </body>
  </worldbody>
</mujoco>
`;

load_mujoco().then((m) => {
  const newModel = m.MjModel.from_xml_string(newXml);
  const newData = new m.MjData(newModel);
  m.mj_forward(newModel, newData);
  
  console.time("10000 steps");
  for(let i=0; i<10000; i++) {
    m.mj_step(newModel, newData);
  }
  console.timeEnd("10000 steps");
  console.log("Did not freeze");
});
