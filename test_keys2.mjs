import load_mujoco from '@mujoco/mujoco';
load_mujoco().then((mujoco) => {
  console.log(Object.keys(mujoco).filter(k => k.toLowerCase().includes('loadxml')));
  console.log("Model loading methods:", Object.keys(mujoco.Model || {}));
});
