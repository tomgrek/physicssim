import load_mujoco from '@mujoco/mujoco';
load_mujoco().then((mujoco) => {
  console.log(Object.keys(mujoco).filter(k => k.includes('XML') || k.includes('Model') || k.includes('Data') || k.includes('mj_')));
});
