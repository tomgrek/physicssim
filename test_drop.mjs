import { compileToMJCF } from './src/utils/mjcf.ts';
import load_mujoco from '@mujoco/mujoco';

const initialScene = {
  nodes: [
    {
      id: 'pole',
      name: 'pole',
      type: 'body',
      pos: [0, 0, 0],
      joints: [
        { name: 'hinge', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0 }
      ],
      geoms: [
        { name: 'pole_geom', type: 'capsule', fromto: [0, 0, 0, 1, 0, 0], size: [0.05], mass: 1, rgba: [0.6, 0.6, 0.6, 1] }
      ],
      children: [
        {
          id: 'bob',
          name: 'bob',
          type: 'body',
          pos: [1, 0, 0],
          joints: [],
          geoms: [
            { name: 'bob_geom', type: 'sphere', size: [0.15], mass: 14, rgba: [0.2, 0.6, 1.0, 1] }
          ],
          children: []
        }
      ]
    }
  ]
};

load_mujoco().then(mujoco => {
    let xml = compileToMJCF(initialScene, -9.81, 0, 0, 0);
    const oldModel = mujoco.MjModel.from_xml_string(xml);
    const oldData = new mujoco.MjData(oldModel);
    mujoco.mj_forward(oldModel, oldData);
    
    // Add box
    const newScene = JSON.parse(JSON.stringify(initialScene));
    newScene.nodes.push({
      id: "box_1",
      name: "box_1",
      type: 'body',
      pos: [0, 0, 2],
      joints: [
        { name: `box_1_free`, type: 'free' }
      ],
      geoms: [
        { name: `box_1_geom`, type: 'box', size: [0.2, 0.2, 0.2], mass: 1, rgba: [0.8, 0.2, 0.2, 1] }
      ],
      children: []
    });
    
    xml = compileToMJCF(newScene, -9.81, 0, 0, 0);
    const newModel = mujoco.MjModel.from_xml_string(xml);
    const newData = new mujoco.MjData(newModel);
    
    const nq = Math.min(oldModel.nq, newModel.nq);
    const nv = Math.min(oldModel.nv, newModel.nv);
    for (let i = 0; i < nq; i++) newData.qpos[i] = oldData.qpos[i];
    for (let i = 0; i < nv; i++) newData.qvel[i] = oldData.qvel[i];
    
    mujoco.mj_forward(newModel, newData);
    console.log("Success");
});
