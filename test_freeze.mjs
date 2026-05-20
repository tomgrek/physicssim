import load_mujoco from '@mujoco/mujoco';

let mujoco;

const initialXml = `
<mujoco model="test">
  <worldbody>
    <body name="pole" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" />
      <geom type="capsule" fromto="0 0 0 1 0 0" size="0.05" />
    </body>
  </worldbody>
</mujoco>
`;

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
  mujoco = m;
  
  const oldModel = mujoco.MjModel.from_xml_string(initialXml);
  const oldData = new mujoco.MjData(oldModel);
  mujoco.mj_forward(oldModel, oldData);
  
  const newModel = mujoco.MjModel.from_xml_string(newXml);
  const newData = new mujoco.MjData(newModel);
  
  const nq = Math.min(oldModel.nq, newModel.nq);
  const nv = Math.min(oldModel.nv, newModel.nv);
  for (let i = 0; i < nq; i++) newData.qpos[i] = oldData.qpos[i];
  for (let i = 0; i < nv; i++) newData.qvel[i] = oldData.qvel[i];
  
  mujoco.mj_forward(newModel, newData);
  
  console.time("10000 steps");
  for(let i=0; i<10000; i++) {
    mujoco.mj_step(newModel, newData);
  }
  console.timeEnd("10000 steps");
  
});
