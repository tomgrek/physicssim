import load_mujoco from '@mujoco/mujoco';
const pendulumXML = `
<mujoco model="pendulum">
  <worldbody>
    <body name="pole" pos="0 0 0">
      <geom name="pole_geom" type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
      <body name="bob" pos="1 0 0">
        <geom name="bob_geom" type="sphere" size="0.15" />
      </body>
    </body>
  </worldbody>
</mujoco>
`;
load_mujoco().then((mujoco) => {
  const model = mujoco.MjModel.from_xml_string(pendulumXML);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  
  const idPole = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "pole_geom");
  const idBob = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "bob_geom");
  
  console.log("pole geom size:", model.geom_size.subarray(idPole*3, idPole*3+3));
  console.log("pole geom xpos:", data.geom_xpos.subarray(idPole*3, idPole*3+3));
  console.log("bob geom size:", model.geom_size.subarray(idBob*3, idBob*3+3));
  console.log("bob geom xpos:", data.geom_xpos.subarray(idBob*3, idBob*3+3));
});
