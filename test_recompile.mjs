import load_mujoco from '@mujoco/mujoco';
import fs from 'fs';

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
  
  console.log("Old model initialized.");
  
  const newModel = mujoco.MjModel.from_xml_string(newXml);
  const newData = new mujoco.MjData(newModel);
  
  console.log("New model initialized.");
  
  const nq = Math.min(oldModel.nq, newModel.nq);
  const nv = Math.min(oldModel.nv, newModel.nv);
  for (let i = 0; i < nq; i++) newData.qpos[i] = oldData.qpos[i];
  for (let i = 0; i < nv; i++) newData.qvel[i] = oldData.qvel[i];
  
  console.log("Copied state.");
  
  mujoco.mj_forward(newModel, newData);
  console.log("mj_forward successful.");
  
  for(let i=0; i<100; i++) {
    mujoco.mj_step(newModel, newData);
  }
  console.log("mj_step successful.");
  
});
