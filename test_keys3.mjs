import load_mujoco from '@mujoco/mujoco';
load_mujoco().then((mujoco) => {
  const modelKeys = Object.keys(mujoco).filter(k => k.toLowerCase().includes('model'));
  console.log("Model related keys:", modelKeys);
  const m = new mujoco.Model('/model.xml');
});
