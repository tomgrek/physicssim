import load_mujoco from '@mujoco/mujoco';
const pendulumXML = `
<mujoco model="pendulum">
  <worldbody>
    <body name="pole" pos="0 0 0">
      <geom name="pole_geom" type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
    </body>
  </worldbody>
</mujoco>
`;
load_mujoco().then((mujoco) => {
  const model = mujoco.MjModel.from_xml_string(pendulumXML);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  const idPole = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "pole_geom");
  console.log("pole geom xmat:", data.geom_xmat.subarray(idPole*9, idPole*9+9));
});
