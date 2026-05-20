import load_mujoco from '@mujoco/mujoco';

const newXml = `
<mujoco model="test">
  <worldbody>
    <body name="box" pos="NaN NaN NaN">
      <joint name="free" type="free" />
      <geom type="box" size="0.2 0.2 0.2" />
    </body>
  </worldbody>
</mujoco>
`;

load_mujoco().then((m) => {
  try {
    console.log("Compiling...");
    const newModel = m.MjModel.from_xml_string(newXml);
    console.log("Compiled!");
  } catch (e) {
    console.log("Error:", e);
  }
});
