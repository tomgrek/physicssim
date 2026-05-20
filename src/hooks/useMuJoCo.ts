import { useEffect } from 'react';
import load_mujoco from '@mujoco/mujoco';
import { useStore } from '../store/useStore';
import { compileToMJCF } from '../utils/mjcf';

let isInitializing = false;

export const useMuJoCoInit = () => {
  const { mujoco, setEngine, sceneGraph, gravityZ } = useStore();

  useEffect(() => {
    if (mujoco || isInitializing) return;
    
    isInitializing = true;
    
    const init = async () => {
      try {
        const mujocoModule = await load_mujoco();
        const xml = compileToMJCF(sceneGraph, gravityZ);
        
        const m = mujocoModule.MjModel.from_xml_string(xml);
        const d = new mujocoModule.MjData(m);
        
        mujocoModule.mj_forward(m, d);
        
        setEngine(mujocoModule, m, d);
      } catch (e) {
        console.error("MuJoCo Init Error:", e);
      } finally {
        isInitializing = false;
      }
    };
    
    init();
  }, []);
};
