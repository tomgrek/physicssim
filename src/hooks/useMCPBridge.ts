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
      const params = new URLSearchParams(location.search);
      const wsPort = params.get('mcpPort') || '3142';
      ws = new WebSocket(`ws://localhost:${wsPort}`);

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

        case 'GET_TELEMETRY': {
          const history = (window as any)._physics_history || [];
          return history.length > 0 ? history[history.length - 1] : { error: 'No simulation telemetry available' };
        }

        case 'GET_HISTORY':
          return (window as any)._physics_history || [];

        case 'RUN_HEADLESS': {
          const ticks = Number(msg.ticks) || 300;
          const { mujoco, sceneGraph, gravityZ, floorFriction, windX, windY, density } = store;
          if (!mujoco) return { error: 'MuJoCo library not loaded' };
          
          let warnings: string[] = [];
          try {
            mujoco.on_warning = (msg: string) => {
              warnings.push(msg);
            };
            const xml = compileToMJCF(sceneGraph, gravityZ, floorFriction, windX, windY, density);
            const headlessModel = mujoco.MjModel.from_xml_string(xml);
            const headlessData = new mujoco.MjData(headlessModel);
            
            mujoco.mj_forward(headlessModel, headlessData);

            // Inject initial velocities (without compiler-level magic randomness)
            const initVelJoints: { name: string; vel: number[] }[] = [];
            const traverseVel = (nodes: any[]) => {
              if (!nodes) return;
              for (const node of nodes) {
                node.joints?.forEach((j: any) => { if (j.initialVelocity) initVelJoints.push({ name: j.name, vel: j.initialVelocity }); });
                traverseVel(node.children);
              }
            };
            traverseVel(sceneGraph.nodes);
            
            let needForward = false;
            for (const j of initVelJoints) {
              const jntId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, j.name);
              if (jntId !== -1) {
                const dofAdr = headlessModel.jnt_dofadr[jntId];
                for (let i = 0; i < j.vel.length; i++) {
                  headlessData.qvel[dofAdr + i] = j.vel[i];
                }
                needForward = true;
              }
            }
            if (needForward) {
              mujoco.mj_forward(headlessModel, headlessData);
            }
            
            const trajectory: any[] = [];
            
            const scriptCache: Record<string, Function> = {};
            const findNodeByIdInLoop = (nodes: any[], targetId: string): any => {
              if (!nodes) return null;
              for (const n of nodes) {
                if (n.id === targetId) return n;
                const c = findNodeByIdInLoop(n.children || [], targetId);
                if (c) return c;
              }
              return null;
            };

            const executeScripts = (nodes: any[], aeroDiagnostics?: Record<string, any>) => {
              if (!nodes) return;
              for (const node of nodes) {
                if (node.isAerodynamic) {
                  // Generic aerodynamic logic for any geom type (box, mesh, ellipsoid, etc.)
                  const geom = node.geoms?.[0];
                  if (geom) {
                    const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, node.name || node.id);
                    if (bId !== -1) {
                      // Find parent independent body ID (ancestor with degrees of freedom)
                      let parentId = bId;
                      while (parentId > 0 && headlessModel.body_dofnum[parentId] === 0) {
                        parentId = headlessModel.body_parentid[parentId];
                      }

                      const gId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_GEOM.value, geom.name || '');
                      let geomWorldX = headlessData.xpos[bId * 3 + 0];
                      let geomWorldY = headlessData.xpos[bId * 3 + 1];
                      let geomWorldZ = headlessData.xpos[bId * 3 + 2];
                      if (gId !== -1) {
                        geomWorldX = headlessData.geom_xpos[gId * 3 + 0];
                        geomWorldY = headlessData.geom_xpos[gId * 3 + 1];
                        geomWorldZ = headlessData.geom_xpos[gId * 3 + 2];
                      }

                      const rx = geomWorldX - headlessData.xpos[parentId * 3 + 0];
                      const ry = geomWorldY - headlessData.xpos[parentId * 3 + 1];
                      const rz = geomWorldZ - headlessData.xpos[parentId * 3 + 2];

                      const wx = headlessData.cvel[bId * 6 + 0];
                      const wy = headlessData.cvel[bId * 6 + 1];
                      const wz = headlessData.cvel[bId * 6 + 2];
                      const vO_x = headlessData.cvel[bId * 6 + 3];
                      const vO_y = headlessData.cvel[bId * 6 + 4];
                      const vO_z = headlessData.cvel[bId * 6 + 5];

                      const vx = vO_x + (wy * rz - wz * ry);
                      const vy = vO_y + (wz * rx - wx * rz);
                      const vz = vO_z + (wx * ry - wy * rx);
                      
                      const o = bId * 9;
                      const noseX = headlessData.xmat[o+0], noseY = headlessData.xmat[o+3], noseZ = headlessData.xmat[o+6];
                      const spanX = headlessData.xmat[o+1], spanY = headlessData.xmat[o+4], spanZ = headlessData.xmat[o+7];
                      const upX   = headlessData.xmat[o+2], upY   = headlessData.xmat[o+5], upZ   = headlessData.xmat[o+8];
                      
                      const relVx = vx - (windX || 0);
                      const relVy = vy - (windY || 0);
                      const relVz = vz;
                      
                      // Project velocity perpendicular to local span axis to isolate 2D airfoil flow
                      const spanDotV = relVx*spanX + relVy*spanY + relVz*spanZ;
                      const airfoilVx = relVx - spanDotV*spanX;
                      const airfoilVy = relVy - spanDotV*spanY;
                      const airfoilVz = relVz - spanDotV*spanZ;
                      const relSpeed = Math.sqrt(airfoilVx*airfoilVx + airfoilVy*airfoilVy + airfoilVz*airfoilVz);
                      
                      if (relSpeed >= 0.05) {
                        // Derive wing area and chord from geom size
                        const s = geom.size || [];
                        const halfX = s[0] || 0.3;
                        const halfY = s[1] || 0.2;
                        const wingArea = (halfX * 2) * (halfY * 2);
                        const chord = halfX * 2;
                        
                        const q = 0.5 * 1.225 * relSpeed * relSpeed;
                        
                        // Normalized flow direction in airfoil plane
                        const vhx = airfoilVx / relSpeed;
                        const vhy = airfoilVy / relSpeed;
                        const vhz = airfoilVz / relSpeed;
                        
                        // Local velocity components
                        const u_nose = -(vhx*noseX + vhy*noseY + vhz*noseZ);
                        const u_up   = -(vhx*upX   + vhy*upY   + vhz*upZ);
                        
                        // Angle of attack
                        const alpha = Math.atan2(u_up, u_nose);
                        
                        // Lift and drag coefficients
                        const CL = 1.5 * Math.sin(2 * alpha);
                        const CD = 0.08 + 1.2 * Math.sin(alpha) * Math.sin(alpha);
                        
                        // Lift direction perpendicular to flow in airfoil plane
                        const ldx = -u_up * noseX + u_nose * upX;
                        const ldy = -u_up * noseY + u_nose * upY;
                        const ldz = -u_up * noseZ + u_nose * upZ;
                        
                        // Drag direction opposite to flow
                        const ddx = -vhx;
                        const ddy = -vhy;
                        const ddz = -vhz;
                        
                        // Force vectors
                        const fx = (CL * ldx + CD * ddx) * q * wingArea;
                        const fy = (CL * ldy + CD * ddy) * q * wingArea;
                        const fz = (CL * ldz + CD * ddz) * q * wingArea;
                        
                        // Aerodynamic pitch moment
                        const pitchMoment = -0.05 * alpha * q * wingArea * chord;
                        const tx_aero = pitchMoment * spanX;
                        const ty_aero = pitchMoment * spanY;
                        const tz_aero = pitchMoment * spanZ;
                        
                        // Aerodynamic roll restoring moment
                        const bankAngle = Math.atan2(upX*spanY - upY*spanX, upZ);
                        const rollRestoring = -0.1 * bankAngle * q * wingArea * chord;
                        const tx_roll = rollRestoring * noseX;
                        const ty_roll = rollRestoring * noseY;
                        const tz_roll = rollRestoring * noseZ;
                        

                        
                        // Torque due to force lever arm: r x F
                        const tx_lever = ry * fz - rz * fy;
                        const ty_lever = rz * fx - rx * fz;
                        const tz_lever = rx * fy - ry * fx;
                        
                        // Apply linear forces to parent independent body
                        headlessData.xfrc_applied[parentId * 6 + 0] += fx;
                        headlessData.xfrc_applied[parentId * 6 + 1] += fy;
                        headlessData.xfrc_applied[parentId * 6 + 2] += fz;
                        
                        // Apply torque to parent independent body
                        headlessData.xfrc_applied[parentId * 6 + 3] += tx_aero + tx_roll + tx_lever;
                        headlessData.xfrc_applied[parentId * 6 + 4] += ty_aero + ty_roll + ty_lever;
                        headlessData.xfrc_applied[parentId * 6 + 5] += tz_aero + tz_roll + tz_lever;

                        if (aeroDiagnostics) {
                          aeroDiagnostics[node.name || node.id] = {
                            relSpeed,
                            alpha: alpha * 180 / Math.PI,
                            CL,
                            CD,
                            force: [fx, fy, fz],
                            torque: [tx_aero + tx_roll + tx_lever, ty_aero + ty_roll + ty_lever, tz_aero + tz_roll + tz_lever]
                          };
                        }
                      } else {
                        if (aeroDiagnostics) {
                          aeroDiagnostics[node.name || node.id] = {
                            relSpeed,
                            alpha: 0,
                            CL: 0,
                            CD: 0,
                            force: [0, 0, 0],
                            torque: [0, 0, 0]
                          };
                        }
                      }

                      // Rotational damping (applied to the parent independent body)
                      const DAMPING = 0.0005;
                      headlessData.xfrc_applied[parentId * 6 + 3] -= DAMPING * wx;
                      headlessData.xfrc_applied[parentId * 6 + 4] -= DAMPING * wy;
                      headlessData.xfrc_applied[parentId * 6 + 5] -= DAMPING * wz;
                    }
                  }
                }

                if (node.script && node.script.trim() !== '') {
                  let fn = scriptCache[node.id];
                  if (!fn) {
                    try {
                      fn = new Function('api', node.script);
                      scriptCache[node.id] = fn;
                    } catch (e: any) {
                      console.error(`[Headless Script Compilation Error on node ${node.name}]:`, e);
                      fn = () => {};
                      scriptCache[node.id] = fn;
                    }
                  }

                  const api = {
                    id: node.id,
                    name: node.name,
                    isKeyPressed: (_keyName: string) => false,
                    setPosition: (pos: number[] | number, bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return;
                      const targetNode = findNodeByIdInLoop(sceneGraph.nodes, bodyName);
                      if (!targetNode || !targetNode.joints || targetNode.joints.length === 0) return;
                      const joint = targetNode.joints[0];
                      const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, joint.name);
                      if (jId !== -1) {
                        const qposadr = headlessModel.jnt_qposadr[jId];
                        if (joint.type === 'free') {
                          if (Array.isArray(pos) && pos.length >= 3) {
                            headlessData.qpos[qposadr + 0] = pos[0];
                            headlessData.qpos[qposadr + 1] = pos[1];
                            headlessData.qpos[qposadr + 2] = pos[2];
                          }
                        } else if (joint.type === 'ball') {
                          if (Array.isArray(pos) && pos.length >= 4) {
                            headlessData.qpos[qposadr + 0] = pos[0];
                            headlessData.qpos[qposadr + 1] = pos[1];
                            headlessData.qpos[qposadr + 2] = pos[2];
                            headlessData.qpos[qposadr + 3] = pos[3];
                          }
                        } else {
                          headlessData.qpos[qposadr] = typeof pos === 'number' ? pos : (Array.isArray(pos) ? pos[0] : 0);
                        }
                      }
                    },
                    setVelocity: (vel: number[] | number, bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return;
                      const targetNode = findNodeByIdInLoop(sceneGraph.nodes, bodyName);
                      if (!targetNode || !targetNode.joints || targetNode.joints.length === 0) return;
                      const joint = targetNode.joints[0];
                      const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, joint.name);
                      if (jId !== -1) {
                        const dofadr = headlessModel.jnt_dofadr[jId];
                        if (joint.type === 'free') {
                          if (Array.isArray(vel) && vel.length >= 3) {
                            headlessData.qvel[dofadr + 0] = vel[0];
                            headlessData.qvel[dofadr + 1] = vel[1];
                            headlessData.qvel[dofadr + 2] = vel[2];
                          }
                        } else {
                          headlessData.qvel[dofadr] = typeof vel === 'number' ? vel : (Array.isArray(vel) ? vel[0] : 0);
                        }
                      }
                    },
                    setAngularVelocity: (angvel: number[] | number, bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return;
                      const targetNode = findNodeByIdInLoop(sceneGraph.nodes, bodyName);
                      if (!targetNode || !targetNode.joints || targetNode.joints.length === 0) return;
                      const joint = targetNode.joints[0];
                      const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, joint.name);
                      if (jId !== -1) {
                        const dofadr = headlessModel.jnt_dofadr[jId];
                        if (joint.type === 'free') {
                          if (Array.isArray(angvel) && angvel.length >= 3) {
                            headlessData.qvel[dofadr + 3] = angvel[0];
                            headlessData.qvel[dofadr + 4] = angvel[1];
                            headlessData.qvel[dofadr + 5] = angvel[2];
                          }
                        } else if (joint.type === 'ball') {
                          if (Array.isArray(angvel) && angvel.length >= 3) {
                            headlessData.qvel[dofadr + 0] = angvel[0];
                            headlessData.qvel[dofadr + 1] = angvel[1];
                            headlessData.qvel[dofadr + 2] = angvel[2];
                          }
                        } else if (joint.type === 'hinge') {
                          headlessData.qvel[dofadr] = typeof angvel === 'number' ? angvel : (Array.isArray(angvel) ? angvel[0] : 0);
                        }
                      }
                    },
                    getPosition: (bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return [0, 0, 0];
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      if (bId !== -1) {
                        return [
                          headlessData.xpos[bId * 3],
                          headlessData.xpos[bId * 3 + 1],
                          headlessData.xpos[bId * 3 + 2]
                        ];
                      }
                      return [0, 0, 0];
                    },
                    getVelocity: (bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return [0, 0, 0];
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      if (bId !== -1) {
                        return [
                          headlessData.cvel[bId * 6 + 3],
                          headlessData.cvel[bId * 6 + 4],
                          headlessData.cvel[bId * 6 + 5]
                        ];
                      }
                      return [0, 0, 0];
                    },
                    getAngularVelocity: (bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return [0, 0, 0];
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      if (bId !== -1) {
                        return [
                          headlessData.cvel[bId * 6 + 0],
                          headlessData.cvel[bId * 6 + 1],
                          headlessData.cvel[bId * 6 + 2]
                        ];
                      }
                      return [0, 0, 0];
                    },
                    getMass: (bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return 0;
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      return bId !== -1 ? headlessModel.body_mass[bId] : 0;
                    },
                    getJointPosition: (jointName: string) => {
                      if (!headlessModel || !mujoco || !headlessData) return 0;
                      const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, jointName);
                      if (jId !== -1) {
                        const adr = headlessModel.jnt_qposadr[jId];
                        return headlessData.qpos[adr];
                      }
                      return 0;
                    },
                    getJointVelocity: (jointName: string) => {
                      if (!headlessModel || !mujoco || !headlessData) return 0;
                      const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, jointName);
                      if (jId !== -1) {
                        const adr = headlessModel.jnt_dofadr[jId];
                        return headlessData.qvel[adr];
                      }
                      return 0;
                    },
                    applyForce: (forceVec: number[], bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData || !Array.isArray(forceVec)) return;
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      if (bId !== -1) {
                        headlessData.xfrc_applied[bId * 6 + 0] += forceVec[0] || 0;
                        headlessData.xfrc_applied[bId * 6 + 1] += forceVec[1] || 0;
                        headlessData.xfrc_applied[bId * 6 + 2] += forceVec[2] || 0;
                      }
                    },
                    applyTorque: (torqueVec: number[], bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData || !Array.isArray(torqueVec)) return;
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      if (bId !== -1) {
                        headlessData.xfrc_applied[bId * 6 + 3] += torqueVec[0] || 0;
                        headlessData.xfrc_applied[bId * 6 + 4] += torqueVec[1] || 0;
                        headlessData.xfrc_applied[bId * 6 + 5] += torqueVec[2] || 0;
                      }
                    },
                    getOrientation: (bodyName = node.id) => {
                      if (!headlessModel || !mujoco || !headlessData) return [1,0,0, 0,1,0, 0,0,1];
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
                      if (bId !== -1) {
                        const o = bId * 9;
                        return [
                          headlessData.xmat[o+0], headlessData.xmat[o+1], headlessData.xmat[o+2],
                          headlessData.xmat[o+3], headlessData.xmat[o+4], headlessData.xmat[o+5],
                          headlessData.xmat[o+6], headlessData.xmat[o+7], headlessData.xmat[o+8]
                        ];
                      }
                      return [1,0,0, 0,1,0, 0,0,1];
                    },
                    applyJointForce: (jointName: string, forceVal: number) => {
                      if (!headlessModel || !mujoco || !headlessData || typeof forceVal !== 'number') return;
                      const jId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_JOINT.value, jointName);
                      if (jId !== -1) {
                        const adr = headlessModel.jnt_dofadr[jId];
                        headlessData.qfrc_applied[adr] += forceVal;
                      }
                    },
                    setActuatorControl: (actuatorName: string, ctrlVal: number) => {
                      if (!headlessModel || !mujoco || !headlessData || typeof ctrlVal !== 'number') return;
                      const actId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_ACTUATOR.value, actuatorName);
                      if (actId !== -1) {
                        headlessData.ctrl[actId] = ctrlVal;
                      }
                    },
                    getTime: () => {
                      return headlessData ? headlessData.time : 0;
                    },
                    getWind: () => {
                      return [windX || 0, windY || 0];
                    },
                    log: (msg: any) => {
                      console.log(`[HeadlessScript:${node.name}]`, msg);
                    }
                  };

                  try {
                    fn(api);
                  } catch (e: any) {
                    console.error(`[Headless Script Runtime Error on node ${node.name}]:`, e);
                  }
                }

                if (node.children) {
                  executeScripts(node.children, aeroDiagnostics);
                }
              }
            };

            const applyFreeJointDamping = (nodes: any[]) => {
              if (!nodes) return;
              for (const node of nodes) {
                if (node.joints) {
                  for (const joint of node.joints) {
                    if (joint.type === 'free' && joint.damping !== undefined && joint.damping > 0) {
                      const bId = mujoco.mj_name2id(headlessModel, mujoco.mjtObj.mjOBJ_BODY.value, node.name || node.id);
                      if (bId !== -1) {
                        const wx = headlessData.cvel[bId * 6 + 0];
                        const wy = headlessData.cvel[bId * 6 + 1];
                        const wz = headlessData.cvel[bId * 6 + 2];
                        const vx = headlessData.cvel[bId * 6 + 3];
                        const vy = headlessData.cvel[bId * 6 + 4];
                        const vz = headlessData.cvel[bId * 6 + 5];

                        const c = joint.damping;
                        const mass = headlessModel.body_mass[bId] || 1.0;
                        const ix = headlessModel.body_inertia[bId * 3 + 0] || 1.0;
                        const iy = headlessModel.body_inertia[bId * 3 + 1] || 1.0;
                        const iz = headlessModel.body_inertia[bId * 3 + 2] || 1.0;

                        headlessData.xfrc_applied[bId * 6 + 0] -= c * mass * vx;
                        headlessData.xfrc_applied[bId * 6 + 1] -= c * mass * vy;
                        headlessData.xfrc_applied[bId * 6 + 2] -= c * mass * vz;
                        headlessData.xfrc_applied[bId * 6 + 3] -= c * ix * wx;
                        headlessData.xfrc_applied[bId * 6 + 4] -= c * iy * wy;
                        headlessData.xfrc_applied[bId * 6 + 5] -= c * iz * wz;
                      }
                    }
                  }
                }
                applyFreeJointDamping(node.children || []);
              }
            };

            for (let i = 0; i < ticks; i++) {
              headlessData.xfrc_applied.fill(0);
              headlessData.qfrc_applied.fill(0);
              
              const aeroDiagnostics: Record<string, any> = {};
              executeScripts(sceneGraph.nodes, aeroDiagnostics);
              applyFreeJointDamping(sceneGraph.nodes);
              
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
                    const wx = headlessData.cvel[bId * 6 + 0];
                    const wy = headlessData.cvel[bId * 6 + 1];
                    const wz = headlessData.cvel[bId * 6 + 2];
                    const vO_x = headlessData.cvel[bId * 6 + 3];
                    const vO_y = headlessData.cvel[bId * 6 + 4];
                    const vO_z = headlessData.cvel[bId * 6 + 5];
                    const x_pos = headlessData.xpos[bId * 3 + 0];
                    const y_pos = headlessData.xpos[bId * 3 + 1];
                    const z_pos = headlessData.xpos[bId * 3 + 2];
                    
                    const vx = vO_x + (wy * z_pos - wz * y_pos);
                    const vy = vO_y + (wz * x_pos - wx * z_pos);
                    const vz = vO_z + (wx * y_pos - wy * x_pos);

                    bodies[bodyName] = {
                      pos: [x_pos, y_pos, z_pos],
                      vel: [vx, vy, vz],
                      angvel: [wx, wy, wz],
                      xfrc_applied: [
                        headlessData.xfrc_applied[bId * 6 + 0],
                        headlessData.xfrc_applied[bId * 6 + 1],
                        headlessData.xfrc_applied[bId * 6 + 2],
                        headlessData.xfrc_applied[bId * 6 + 3],
                        headlessData.xfrc_applied[bId * 6 + 4],
                        headlessData.xfrc_applied[bId * 6 + 5]
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
                        vel: headlessData.qvel[dofadr],
                        qfrc_applied: headlessData.qfrc_applied[dofadr]
                      };
                    }
                  });
                  
                  if (node.children) collectNodeData(node.children);
                }
              };
              
              collectNodeData(sceneGraph.nodes);
              
              const contacts: any[] = [];
              for (let c = 0; c < headlessData.contact.size(); c++) {
                const contact = headlessData.contact.get(c);
                const g1 = contact.geom1;
                const g2 = contact.geom2;
                const geom1Name = mujoco.mj_id2name(headlessModel, mujoco.mjtObj.mjOBJ_GEOM.value, g1) || `geom_${g1}`;
                const geom2Name = mujoco.mj_id2name(headlessModel, mujoco.mjtObj.mjOBJ_GEOM.value, g2) || `geom_${g2}`;
                contacts.push({
                  geom1: geom1Name,
                  geom2: geom2Name,
                  dist: contact.dist
                });
              }

              trajectory.push({
                time: headlessData.time,
                bodies,
                joints,
                contacts,
                aeroDiagnostics
              });
            }
            
            headlessModel.delete();
            headlessData.delete();
            
            return {
              ok: true,
              ticksSimulated: trajectory.length,
              trajectory,
              warnings
            };
          } catch (e: any) {
            return { ok: false, error: e.message, warnings };
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
          return ['empty', 'pendulum', 'cubes', 'gears', 'machine', 'rack_pinion',
                  'inclined_plane', 'pulley_system', 'cartpole', 'newtons_cradle',
                  'suspension_bridge', 'paper_plane', 'monkey_head',
                  'golden_gate', 'golden_gate_mesh', 'mesh_collision'];

        case 'UPDATE_SCENE': {
          if (!msg.sceneGraph) return { ok: false, error: 'Missing sceneGraph' };
          store.updateScene(msg.sceneGraph);
          return { ok: true };
        }

        case 'SET_ENVIRONMENT': {
          const { gravityZ, windX, windY, density, floorFriction } = msg;
          const env: Record<string, number> = {};
          if (gravityZ !== undefined) env.gravityZ = gravityZ;
          if (windX !== undefined) env.windX = windX;
          if (windY !== undefined) env.windY = windY;
          if (density !== undefined) env.density = density;
          if (floorFriction !== undefined) env.floorFriction = floorFriction;
          store.setEnvironment(env);
          return { ok: true };
        }

        case 'GET_SCHEMA':
          return {
            geomTypes: ['box', 'sphere', 'capsule', 'cylinder', 'ellipsoid', 'plane', 'mesh'],
            geomSizes: {
              box:       'half-extents [hx, hy, hz]',
              sphere:    'radius [r]',
              capsule:   'radius and half-height [r, hh] — cylinder between the two end-caps',
              cylinder:  'radius and half-height [r, hh]',
              ellipsoid: 'semi-axes [rx, ry, rz]',
              plane:     'ignored by MuJoCo (infinite plane) — set to [0, 0, 1] or any non-zero',
              mesh:      'not used — shape defined by vertices+faces. Two modes: STATIC (default, visual only) and DYNAMIC (dynamic:true, full physics+collision). See tips.',
            },
            jointTypes: ['hinge', 'slide', 'ball', 'free'],
            geomFields: {
              name:        'string — unique identifier',
              type:        'GeomType (see geomTypes)',
              size:        'number[] — interpretation depends on type (see geomSizes)',
              rgba:        'number[4] — [r, g, b, a] each 0-1, default white opaque',
              pos:         'number[3] — local offset from body origin',
              quat:        'number[4] — [w, x, y, z] rotation quaternion',
              euler:       'number[3] — [roll, pitch, yaw] in degrees, alternative to quat',
              fromto:      'number[6] — [x1,y1,z1, x2,y2,z2] for capsule/cylinder endpoints (overrides size/pos/quat)',
              mass:        'number — if set, overrides density-based mass for this geom',
              friction:    'number[3] — [slide, spin, roll] friction coefficients',
              contype:     'number — bitmask for collision group membership',
              conaffinity: 'number — bitmask for which groups this geom collides with',
              condim:      'number — contact dimensionality (1, 3, 4, or 6)',
              solref:      'number[2] — constraint solver reference [timeconst, dampratio]',
              solimp:      'number[5] — constraint solver impedance params',
              vertices:    'number[] — flat array of vertex positions for mesh type: [x0,y0,z0, x1,y1,z1, ...] in Three.js Y-up space',
              faces:       'number[] — flat array of triangle indices for mesh type: [i0,j0,k0, i1,j1,k1, ...]',
              dynamic:     'boolean — if true, mesh participates in simulation and collision; requires renderVertices',
              renderVertices: 'number[] — dynamic mesh only: flat [x0,y0,z0,...] in raw MuJoCo Z-up space. Convert from Y-up vertices: (x,y,z)→(x,-z,y). Do NOT subtract centroid — MuJoCo recenters internally.',
            },
            nodeFields: {
              id:            'string — unique body identifier (used in coupling/weld/connect refs)',
              name:          'string — display name, also used in MuJoCo XML',
              pos:           'number[3] — position relative to parent (world for root nodes)',
              quat:          'number[4] — body orientation quaternion [w,x,y,z]',
              euler:         'number[3] — body orientation in degrees, alternative to quat',
              geoms:         'SceneGeom[] — one or more geoms composing the body shape',
              joints:        'SceneJoint[] — joints attaching this body to its parent',
              children:      'SceneNode[] — child bodies (rigidly offset unless they have joints)',
              coupleTargetId:'string — id of another body; couples their first joints with coupleRatio',
              coupleRatio:   'number — gear ratio for explicit joint coupling (default -1)',
              weldTargetId:  'string — id of body to weld to (closed-loop rigid constraint)',
              connectTargetId:'string — id of body to connect to via a ball-and-socket point constraint',
              connectAnchor: 'number[3] — world-space anchor point for the connect constraint',
            },
            tips: [
              'Compound shapes: add multiple geoms to one body with different pos/quat/euler offsets',
              'Asymmetric shapes: combine box + sphere + cylinder geoms on a single body',
              'Torus-like shapes: ring of capsule geoms arranged with pos+euler offsets',
              'L/T/cross shapes: multiple box geoms with offset positions on one body',
              'fromto on capsule lets you specify start/end points directly in local space',
              'Use rgba to color each geom independently for visual variety',
              'ellipsoid semi-axes let you squash/stretch independently on all 3 axes',
              'Children without joints are rigid offsets — useful for adding detail geometry',
              'Arbitrary mesh: type=mesh with vertices=[x0,y0,z0,...] and faces=[i0,j0,k0,...] (triangles)',
              'CRITICAL — mesh vertex coordinate system: X=right, Y=up (height), Z=toward camera. This is Three.js world space, NOT MuJoCo Z-up. The ground plane is at Y=0.',
              'Mesh vertical post example: vertices centred at (cx, halfHeight, cz) with hy=halfHeight (tall in Y)',
              'Mesh flat plank example: box(cx, 0.3, cz, halfSpan, 0.06, halfWidth) — small hy=thickness, large hx=span',
              'Mesh tetrahedron example: vertices=[0,0,0, 1,0,0, 0.5,1,0, 0.5,0.5,1], faces=[0,1,2, 0,1,3, 1,2,3, 0,2,3]',
              'Static mesh (no dynamic field): visual-only. Vertices in Three.js Y-up world space. Never moves, never collides. Good for scenery and decorative structures.',
              'Dynamic mesh (dynamic:true): full physics+collision. MuJoCo takes convex hull — concave shapes will not collide correctly. Requires renderVertices.',
              'Dynamic mesh renderVertices: just swap Y↔Z on each Y-up vertex: (x,y,z)→(x,-z,y). Do NOT subtract centroid. MuJoCo recenters internally.',
              'Dynamic mesh face winding: use outward-facing CCW winding. Wrong winding causes inside-out contacts and objects sinking through surfaces.',
              'Dynamic mesh body pos: set body_pos=[0,0,0] to place mesh where its Y-up base sits. Adjust body_pos.z to raise/lower.',
              'Working example: mesh_collision preset (pyramid + ramp with full collision).',
            ],
          };

        case 'BUILD_SCENE': {
          // High-level helper: accepts an array of body descriptors and assembles a valid sceneGraph.
          // Each descriptor can have the same fields as SceneNode but `geoms` may be a shorthand array
          // of plain objects — missing fields are filled with safe defaults so agents don't need to
          // supply every field.
          const bodies: any[] = msg.bodies;
          if (!Array.isArray(bodies) || bodies.length === 0) {
            return { ok: false, error: 'bodies must be a non-empty array' };
          }

          const fillGeomDefaults = (g: any, bodyName: string, idx: number) => ({
            name:    g.name    ?? `${bodyName}_geom_${idx}`,
            type:    g.type    ?? 'box',
            size:    g.size    ?? [0.25, 0.25, 0.25],
            rgba:    g.rgba    ?? [0.6, 0.6, 0.9, 1],
            ...(g.pos         !== undefined ? { pos: g.pos }         : {}),
            ...(g.quat        !== undefined ? { quat: g.quat }       : {}),
            ...(g.euler       !== undefined ? { euler: g.euler }     : {}),
            ...(g.fromto      !== undefined ? { fromto: g.fromto }   : {}),
            ...(g.mass        !== undefined ? { mass: g.mass }       : {}),
            ...(g.friction    !== undefined ? { friction: g.friction }: {}),
            ...(g.contype     !== undefined ? { contype: g.contype } : {}),
            ...(g.conaffinity !== undefined ? { conaffinity: g.conaffinity } : {}),
            ...(g.condim      !== undefined ? { condim: g.condim }   : {}),
            ...(g.solref      !== undefined ? { solref: g.solref }   : {}),
            ...(g.solimp      !== undefined ? { solimp: g.solimp }   : {}),
            ...(g.vertices    !== undefined ? { vertices: g.vertices }: {}),
            ...(g.faces       !== undefined ? { faces: g.faces }     : {}),
          });

          const fillJointDefaults = (j: any, bodyName: string, idx: number) => ({
            name:    j.name    ?? `${bodyName}_joint_${idx}`,
            type:    j.type    ?? 'free',
            ...(j.axis     !== undefined ? { axis: j.axis }         : {}),
            ...(j.pos      !== undefined ? { pos: j.pos }           : {}),
            ...(j.damping  !== undefined ? { damping: j.damping }   : {}),
            ...(j.stiffness!== undefined ? { stiffness: j.stiffness}: {}),
            ...(j.limited  !== undefined ? { limited: j.limited }   : {}),
            ...(j.range    !== undefined ? { range: j.range }       : {}),
            ...(j.actuator !== undefined ? { actuator: j.actuator } : {}),
          });

          const fillBodyDefaults = (b: any): any => {
            const name = b.name ?? b.id ?? `body_${Math.random().toString(36).slice(2, 7)}`;
            const id   = b.id   ?? name;
            return {
              id,
              name,
              type:     'body',
              pos:      b.pos     ?? [0, 0, 1],
              ...(b.quat  !== undefined ? { quat: b.quat }   : {}),
              ...(b.euler !== undefined ? { euler: b.euler } : {}),
              geoms:    (b.geoms   ?? [{ type: 'box', size: [0.25, 0.25, 0.25] }])
                          .map((g: any, i: number) => fillGeomDefaults(g, name, i)),
              joints:   (b.joints  ?? [{ type: 'free' }])
                          .map((j: any, i: number) => fillJointDefaults(j, name, i)),
              children: (b.children ?? []).map(fillBodyDefaults),
              ...(b.coupleTargetId  !== undefined ? { coupleTargetId: b.coupleTargetId }   : {}),
              ...(b.coupleRatio     !== undefined ? { coupleRatio: b.coupleRatio }         : {}),
              ...(b.weldTargetId    !== undefined ? { weldTargetId: b.weldTargetId }       : {}),
              ...(b.connectTargetId !== undefined ? { connectTargetId: b.connectTargetId } : {}),
              ...(b.connectAnchor   !== undefined ? { connectAnchor: b.connectAnchor }     : {}),
              ...(b.script          !== undefined ? { script: b.script }                   : {}),
            };
          };

          const nodes = bodies.map(fillBodyDefaults);
          store.updateScene({ nodes });
          return { ok: true, nodeCount: nodes.length };
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
