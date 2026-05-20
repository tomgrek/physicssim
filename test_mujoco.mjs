import load_mujoco from '@mujoco/mujoco';

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
    const model = mujoco.mj_loadXML(xml);
    console.log("mj_loadXML(xml) works");
  } catch(e) { console.log("mj_loadXML fail", e?.message); }
  
  try {
    console.log("Model key:", !!mujoco.Model);
    console.log("MjModel key:", !!mujoco.MjModel);
  } catch(e) {}
});
