import load_mujoco from '@mujoco/mujoco';
const pendulumXML = `
<mujoco model="pendulum">
  <worldbody>
    <body name="bob" pos="1 0 0">
      <geom name="bob_geom" type="sphere" size="0.15" />
    </body>
  </worldbody>
</mujoco>
`;
load_mujoco().then((mujoco) => {
  const model = mujoco.MjModel.from_xml_string(pendulumXML);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  const idBob = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "bob_geom");
  console.log("bob geom xmat:", data.geom_xmat.subarray(idBob*9, idBob*9+9));
});
