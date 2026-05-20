const load_mujoco = require('@mujoco/mujoco');

load_mujoco().then((mujoco) => {
  const xml = '<mujoco><worldbody><body name="test"><geom type="sphere" size="1"/></body></worldbody></mujoco>';
  
  try {
    const model = new mujoco.Model(xml);
    console.log("new mujoco.Model(xml) works");
  } catch(e) { console.log(e.message); }

  try {
    const model = mujoco.Model.load_from_xml(xml);
    console.log("mujoco.Model.load_from_xml works");
  } catch(e) { console.log("load_from_xml fail", e?.message); }

  try {
    // VFS way
    const vfs = new mujoco.MjVFS();
    console.log("new mujoco.MjVFS() works");
  } catch(e) { console.log("new mujoco.MjVFS fail", e?.message); }
  
  try {
    console.log("mujoco keys:", Object.keys(mujoco).filter(k => k.toLowerCase().includes('model') || k.includes('mj_')));
  } catch(e) {}
});
