/**
 * MCP bridge for Physics Sim.
 * Uses the Zustand store directly (getState/setState) from outside React.
 */

import { useEffect } from 'react';
import { useStore } from '../store/useStore';
import { compileToMJCF } from '../utils/mjcf';

export function useMCPBridge() {
  useEffect(() => {
    let ws: WebSocket | null = null;
    let dead = false;
    let retryTimer: ReturnType<typeof setTimeout>;

    const connect = () => {
      if (dead) return;
      ws = new WebSocket(`ws://${location.host}/mcp?role=browser`);

      ws.onopen = () =>
        ws!.send(JSON.stringify({ event: 'HELLO', app: 'physics', port: location.port }));

      ws.onmessage = (evt) => {
        let msg: any;
        try { msg = JSON.parse(evt.data); } catch { return; }
        const { cmd, id } = msg;
        if (!cmd) return;

        let result: unknown;
        try { result = handle(cmd, msg); } catch (e) {
          ws?.send(JSON.stringify({ event: 'ERROR', cmd, id, error: String(e) }));
          return;
        }
        Promise.resolve(result)
          .then(data => ws?.send(JSON.stringify({ event: 'RESULT', cmd, id, data })))
          .catch(e  => ws?.send(JSON.stringify({ event: 'ERROR', cmd, id, error: String(e) })));
      };

      ws.onclose = () => { if (!dead) retryTimer = setTimeout(connect, 2000); };
      ws.onerror = () => ws?.close();
    };

    const handle = (cmd: string, msg: any): unknown => {
      // Access Zustand store directly — works outside React render
      const store = useStore.getState();

      switch (cmd) {
        case 'GET_STATE':
          return {
            sceneGraph:   store.sceneGraph,
            isPlaying:    store.isPlaying,
            isLoaded:     store.isLoaded,
            gravityZ:     store.gravityZ,
            windX:        store.windX,
            windY:        store.windY,
            density:      store.density,
            floorFriction: store.floorFriction,
          };

        case 'GET_SCENE':
          return store.sceneGraph;

        case 'GET_HISTORY':
          return (window as any)._physics_history || [];

        case 'RUN_HEADLESS': {
          const ticks = Number(msg.ticks) || 300;
          const { mujoco, sceneGraph, gravityZ, floorFriction, windX, windY, density } = store;
          if (!mujoco) return { error: 'MuJoCo library not loaded' };
          
          try {
            const xml = compileToMJCF(sceneGraph, gravityZ, floorFriction, windX, windY, density);
            const headlessModel = mujoco.MjModel.from_xml_string(xml);
            const headlessData = new mujoco.MjData(headlessModel);
            
            mujoco.mj_forward(headlessModel, headlessData);
            
            const trajectory: any[] = [];
            
            for (let i = 0; i < ticks; i++) {
              headlessData.xfrc_applied.fill(0);
              headlessData.qfrc_applied.fill(0);
              
              mujoco.mj_step(headlessModel, headlessData);
              
              if (isNaN(headlessData.qpos[0])) {
                break;
              }
              
              const bodies: Record<string, any> = {};
              const joints: Record<string, any> = {};
              
              const collectNodeData = (nodesList: any[]) => {
                if (!nodesList) return;
                for (const node of nodesList) {
                  const bodyName = node.id;
                  const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                  if (bId !== -1) {
                    bodies[bodyName] = {
                      pos: [
                        headlessData.xpos[bId * 3],
                        headlessData.xpos[bId * 3 + 1],
                        headlessData.xpos[bId * 3 + 2]
                      ],
                      vel: [
                        headlessData.cvel[bId * 6 + 3],
                        headlessData.cvel[bId * 6 + 4],
                        headlessData.cvel[bId * 6 + 5]
                      ],
                      angvel: [
                        headlessData.cvel[bId * 6 + 0],
                        headlessData.cvel[bId * 6 + 1],
                        headlessData.cvel[bId * 6 + 2]
                      ]
                    };
                  }
                  
                  node.joints?.forEach((j: any) => {
                    const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, j.name);
                    if (jId !== -1) {
                      const qposadr = headlessModel.jnt_qposadr[jId];
                      const dofadr = headlessModel.jnt_dofadr[jId];
                      joints[j.name] = {
                        pos: headlessData.qpos[qposadr],
                        vel: headlessData.qvel[dofadr]
                      };
                    }
                  });
                  
                  if (node.children) collectNodeData(node.children);
                }
              };
              
              collectNodeData(sceneGraph.nodes);
              
              trajectory.push({
                time: headlessData.time,
                bodies,
                joints
              });
            }
            
            headlessModel.delete();
            headlessData.delete();
            
            return {
              ok: true,
              ticksSimulated: trajectory.length,
              trajectory
            };
          } catch (e: any) {
            return { ok: false, error: e.message };
          }
        }

        case 'TOGGLE_PLAY':
          store.togglePlay();
          return { ok: true, isPlaying: !store.isPlaying };

        case 'PLAY':
          if (!store.isPlaying) store.togglePlay();
          return { ok: true };

        case 'STOP':
          if (store.isPlaying) store.togglePlay();
          return { ok: true };

        case 'RESET':
          store.resetSimulation();
          if ((window as any)._physics_history) (window as any)._physics_history = [];
          return { ok: true };

        case 'LOAD_PRESET': {
          const name = msg.preset as Parameters<typeof store.loadPreset>[0];
          if (!name) return { ok: false, error: 'Missing preset name' };
          store.loadPreset(name);
          if ((window as any)._physics_history) (window as any)._physics_history = [];
          return { ok: true, preset: name };
        }

        case 'LIST_PRESETS':
          return ['pendulum', 'cubes', 'gears', 'machine', 'rack_pinion',
                  'inclined_plane', 'pulley_system', 'cartpole', 'newtons_cradle',
                  'suspension_bridge', 'paper_plane'];

        case 'UPDATE_SCENE': {
          if (!msg.sceneGraph) return { ok: false, error: 'Missing sceneGraph' };
          store.updateScene(msg.sceneGraph);
          return { ok: true };
        }

        case 'SET_ENVIRONMENT': {
          const { gravityZ, windX, windY, density, floorFriction } = msg;
          store.setEnvironment({ gravityZ, windX, windY, density, floorFriction });
          return { ok: true };
        }

        default:
          return { error: `Unknown command: ${cmd}` };
      }
    };

    connect();
    return () => {
      dead = true;
      clearTimeout(retryTimer);
      ws?.close();
    };
  }, []);
}
