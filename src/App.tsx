
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import { useMuJoCoInit } from './hooks/useMuJoCo';
import { useMCPBridge } from './hooks/useMCPBridge';
import { useStore, scaleMeshGeoms } from './store/useStore';
import type { SceneNode } from './types/scene';
import { Play, Square, Settings2, SlidersHorizontal, Settings, Box, Circle, X, RotateCcw, Eye, Trash2, Layers, CircleDot, Zap, Info, Triangle, Disc, Code, Menu, Shapes, Minimize2 } from 'lucide-react';
import { useRef, useMemo, useEffect, useCallback, useState } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';

// Physics Step Hook
const PhysicsLoop = ({ model, data, mujoco, isPlaying }: { model: any, data: any, mujoco: any, isPlaying: boolean }) => {

  const accumulator = useRef<number>(0);
  const scriptCache = useRef<Record<string, Function>>({});
  const sceneGraph = useStore(state => state.sceneGraph);

  // Auto-clear cache when scene graph is edited (e.g. scripts saved)
  useEffect(() => {
    scriptCache.current = {};
  }, [sceneGraph]);

  const executeScripts = (nodes: any[]) => {
    if (!nodes) return;
    for (const node of nodes) {
      if (node.isAerodynamic) {
        // Generic aerodynamic logic for any geom type (box, mesh, ellipsoid, etc.)
        const geom = node.geoms?.[0];
        if (geom) {
          // Derive wing area and chord from geom size, or fall back to sensible defaults
          const s = geom.size || [];
          const halfX = s[0] || 0.3;
          const halfY = s[1] || 0.2;
          const wingArea = (halfX * 2) * (halfY * 2);
          const chord = halfX * 2;
          
          const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, node.name || node.id);
          if (bId !== -1) {
            const vx = data.cvel[bId * 6 + 3];
            const vy = data.cvel[bId * 6 + 4];
            const vz = data.cvel[bId * 6 + 5];
            const wx = data.cvel[bId * 6 + 0];
            const wy = data.cvel[bId * 6 + 1];
            const wz = data.cvel[bId * 6 + 2];
            
            // Rotational damping
            const DAMPING = 0.0005;
            data.xfrc_applied[bId * 6 + 3] -= DAMPING * wx;
            data.xfrc_applied[bId * 6 + 4] -= DAMPING * wy;
            data.xfrc_applied[bId * 6 + 5] -= DAMPING * wz;
            
            const o = bId * 9;
            const noseX = data.xmat[o+0], noseY = data.xmat[o+3], noseZ = data.xmat[o+6];
            const spanX = data.xmat[o+1], spanY = data.xmat[o+4], spanZ = data.xmat[o+7];
            const upX   = data.xmat[o+2], upY   = data.xmat[o+5], upZ   = data.xmat[o+8];
            
            const state = useStore.getState();
            const windX = state.windX || 0;
            const windY = state.windY || 0;
            
            const relVx = vx - windX;
            const relVy = vy - windY;
            const relVz = vz;
            const relSpeed = Math.sqrt(relVx*relVx + relVy*relVy + relVz*relVz);
            
            if (relSpeed >= 0.1) {
              const q = 0.5 * 1.225 * relSpeed * relSpeed;
              const vDotNose = relVx*noseX + relVy*noseY + relVz*noseZ;
              const cosAoA = Math.max(-1, Math.min(1, vDotNose / relSpeed));
              const sinAoA = Math.sqrt(Math.max(0, 1 - cosAoA*cosAoA));
              const aoaSign = (relVz - vDotNose*noseZ) < 0 ? 1 : -1;
              const alpha = aoaSign * Math.asin(sinAoA);
              
              const stallFactor = Math.max(0, 1 - Math.pow(Math.abs(alpha) / 0.35, 3));
              const CL = 3.0 * alpha * stallFactor;
              const CD = 0.04 + CL * CL / (Math.PI * 4.5 * 0.8);
              
              const liftMag = CL * q * wingArea;
              const dragMag = CD * q * wingArea;
              
              const vhx = relVx/relSpeed, vhy = relVy/relSpeed, vhz = relVz/relSpeed;
              const upDotV = upX*vhx + upY*vhy + upZ*vhz;
              let ldx = upX - upDotV*vhx;
              let ldy = upY - upDotV*vhy;
              let ldz = upZ - upDotV*vhz;
              const ldMag = Math.sqrt(ldx*ldx + ldy*ldy + ldz*ldz);
              if (ldMag > 1e-6) { ldx/=ldMag; ldy/=ldMag; ldz/=ldMag; }
              
              const ddx = -vhx, ddy = -vhy, ddz = -vhz;
              
              data.xfrc_applied[bId * 6 + 0] += liftMag*ldx + dragMag*ddx;
              data.xfrc_applied[bId * 6 + 1] += liftMag*ldy + dragMag*ddy;
              data.xfrc_applied[bId * 6 + 2] += liftMag*ldz + dragMag*ddz;
              
              const pitchMoment = -0.1 * alpha * stallFactor * q * wingArea * chord;
              data.xfrc_applied[bId * 6 + 3] += pitchMoment * spanX;
              data.xfrc_applied[bId * 6 + 4] += pitchMoment * spanY;
              data.xfrc_applied[bId * 6 + 5] += pitchMoment * spanZ;
              
              const bankAngle = Math.atan2(upX*spanY - upY*spanX, upZ);
              const rollRestoring = -0.1 * bankAngle * q * wingArea * chord;
              data.xfrc_applied[bId * 6 + 3] += rollRestoring * noseX;
              data.xfrc_applied[bId * 6 + 4] += rollRestoring * noseY;
              data.xfrc_applied[bId * 6 + 5] += rollRestoring * noseZ;
            }
          }
        }
      }

      if (node.script && node.script.trim() !== '') {
        let fn = scriptCache.current[node.id];
        if (!fn) {
          try {
            fn = new Function('api', node.script);
            scriptCache.current[node.id] = fn;
          } catch (e: any) {
            console.error(`[Script Compilation Error on node ${node.name}]:`, e);
            fn = () => {}; // Cache dummy fallback to avoid infinite frame error logs
            scriptCache.current[node.id] = fn;
          }
        }

        const api = {
          id: node.id,
          name: node.name,

          // Reads physical position [X, Y, Z] of any body in the scene
          getPosition: (bodyName = node.id) => {
            if (!model || !mujoco || !data) return [0, 0, 0];
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            if (bId !== -1) {
              return [
                data.xpos[bId * 3],
                data.xpos[bId * 3 + 1],
                data.xpos[bId * 3 + 2]
              ];
            }
            return [0, 0, 0];
          },

          // Reads linear velocity [VX, VY, VZ] of any body
          getVelocity: (bodyName = node.id) => {
            if (!model || !mujoco || !data) return [0, 0, 0];
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            if (bId !== -1) {
              return [
                data.cvel[bId * 6 + 3], // linear velocity X
                data.cvel[bId * 6 + 4], // linear velocity Y
                data.cvel[bId * 6 + 5]  // linear velocity Z
              ];
            }
            return [0, 0, 0];
          },

          // Reads angular velocity [WX, WY, WZ] of any body
          getAngularVelocity: (bodyName = node.id) => {
            if (!model || !mujoco || !data) return [0, 0, 0];
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            if (bId !== -1) {
              return [
                data.cvel[bId * 6 + 0], // angular X
                data.cvel[bId * 6 + 1], // angular Y
                data.cvel[bId * 6 + 2]  // angular Z
              ];
            }
            return [0, 0, 0];
          },

          // Reads total body mass in kg
          getMass: (bodyName = node.id) => {
            if (!model || !mujoco || !data) return 0;
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            return bId !== -1 ? model.body_mass[bId] : 0;
          },

          // Reads 1D joint position (translation in m for sliders, angle in rad for hinges)
          getJointPosition: (jointName: string) => {
            if (!model || !mujoco || !data) return 0;
            const jId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, jointName);
            if (jId !== -1) {
              const adr = model.jnt_qposadr[jId];
              return data.qpos[adr];
            }
            return 0;
          },

          // Reads 1D joint velocity (translation speed for sliders, angular velocity for hinges)
          getJointVelocity: (jointName: string) => {
            if (!model || !mujoco || !data) return 0;
            const jId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, jointName);
            if (jId !== -1) {
              const adr = model.jnt_dofadr[jId];
              return data.qvel[adr];
            }
            return 0;
          },

          // Applies a direct external force vector [FX, FY, FZ] to any body
          applyForce: (forceVec: number[], bodyName = node.id) => {
            if (!model || !mujoco || !data || !Array.isArray(forceVec)) return;
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            if (bId !== -1) {
              // xfrc_applied layout: [force_x, force_y, force_z, torque_x, torque_y, torque_z]
              data.xfrc_applied[bId * 6 + 0] += forceVec[0] || 0;
              data.xfrc_applied[bId * 6 + 1] += forceVec[1] || 0;
              data.xfrc_applied[bId * 6 + 2] += forceVec[2] || 0;
            }
          },

          // Applies a world-frame torque vector [TX, TY, TZ] to any body
          applyTorque: (torqueVec: number[], bodyName = node.id) => {
            if (!model || !mujoco || !data || !Array.isArray(torqueVec)) return;
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            if (bId !== -1) {
              // xfrc_applied layout: [force_x, force_y, force_z, torque_x, torque_y, torque_z]
              data.xfrc_applied[bId * 6 + 3] += torqueVec[0] || 0;
              data.xfrc_applied[bId * 6 + 4] += torqueVec[1] || 0;
              data.xfrc_applied[bId * 6 + 5] += torqueVec[2] || 0;
            }
          },

          // Returns the body's 3x3 rotation matrix as a flat 9-element row-major array
          // Use this to transform vectors between world and body frames
          getOrientation: (bodyName = node.id) => {
            if (!model || !mujoco || !data) return [1,0,0, 0,1,0, 0,0,1];
            const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
            if (bId !== -1) {
              const o = bId * 9;
              return [
                data.xmat[o+0], data.xmat[o+1], data.xmat[o+2],
                data.xmat[o+3], data.xmat[o+4], data.xmat[o+5],
                data.xmat[o+6], data.xmat[o+7], data.xmat[o+8]
              ];
            }
            return [1,0,0, 0,1,0, 0,0,1];
          },

          // Applies an internal joint-aligned force (torque for hinges, linear thrust for sliders)
          applyJointForce: (jointName: string, forceVal: number) => {
            if (!model || !mujoco || !data || typeof forceVal !== 'number') return;
            const jId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, jointName);
            if (jId !== -1) {
              const adr = model.jnt_dofadr[jId];
              data.qfrc_applied[adr] += forceVal;
            }
          },

          // Sets active actuator control input (for motor speed/torque)
          setActuatorControl: (actuatorName: string, ctrlVal: number) => {
            if (!model || !mujoco || !data || typeof ctrlVal !== 'number') return;
            const actId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR.value, actuatorName);
            if (actId !== -1) {
              data.ctrl[actId] = ctrlVal;
            }
          },

          // Queries elapsed simulation time
          getTime: () => {
            return data ? data.time : 0;
          },

          // Returns current wind velocity [windX, windY] from environment settings
          getWind: () => {
            const state = useStore.getState();
            return [state.windX || 0, state.windY || 0];
          },

          // Safe debugger logging
          log: (msg: any) => {
            console.log(`[Script:${node.name}]`, msg);
          }
        };

        try {
          fn(api);
        } catch (e: any) {
          console.error(`[Script Runtime Error on node ${node.name}]:`, e);
        }
      }

      if (node.children) {
        executeScripts(node.children);
      }
    }
  };
  
  useFrame((_state, delta) => {
    if ((window as any).DISABLE_USEFRAME) return;
    
    // Safety check: ensure closure model/data match current store active ones
    const activeModel = useStore.getState().model;
    const activeData = useStore.getState().data;
    if (model !== activeModel || data !== activeData) return;
    
    if (!isPlaying || !model || !data || !mujoco) return;
    
    // Accumulate elapsed real time (capped to avoid spiral of death on lag spikes)
    const maxDelta = 0.1;
    accumulator.current += Math.min(delta, maxDelta);
    
    const stepSize = model.opt.timestep;
    const stepsNeeded = Math.floor(accumulator.current / stepSize);
    accumulator.current -= stepsNeeded * stepSize;
    
    for (let i = 0; i < stepsNeeded; i++) {
      try {
        // Reset applied external forces and joint forces at step start
        data.xfrc_applied.fill(0);
        data.qfrc_applied.fill(0);

        const { draggedNodeId, dragTarget } = useStore.getState();
        if (draggedNodeId && dragTarget) {
          // Resolve the body to apply force to: prefer the heaviest child body
          // (e.g. cradle bob) rather than the low-mass pivot rope.
          let targetBodyName = draggedNodeId;
          let bestMass = -1;
          const findHeaviestDescendant = (nodeId: string) => {
            const bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, nodeId);
            if (bid !== -1) {
              const m = model.body_mass[bid] || 0;
              if (m > bestMass) { bestMass = m; targetBodyName = nodeId; }
            }
            // Walk children via scene graph
            const findNode = (nodes: any[], id: string): any => {
              for (const n of nodes) {
                if (n.id === id) return n;
                const c = findNode(n.children || [], id);
                if (c) return c;
              }
              return null;
            };
            const node = findNode(sceneGraph.nodes, nodeId);
            for (const child of node?.children || []) findHeaviestDescendant(child.id);
          };
          findHeaviestDescendant(draggedNodeId);

          const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, targetBodyName);
          if (bId !== -1) {
            const bx = data.xpos[bId * 3];
            const by = data.xpos[bId * 3 + 1];
            const bz = data.xpos[bId * 3 + 2];
            
            // Get linear velocities of the body in MuJoCo coordinates
            const vx = data.cvel[bId * 6 + 3];
            const vy = data.cvel[bId * 6 + 4];
            const vz = data.cvel[bId * 6 + 5];
            
            const mass = model.body_mass[bId] || 1.0;
            const K = 200.0;
            // Critically damped spring coefficient: D = 2 * sqrt(mass * K)
            const D = 2.0 * Math.sqrt(mass * K);
            
            // Calculate spring + damping force components
            let fx = K * (dragTarget.x - bx) - D * vx;
            let fy = K * (dragTarget.y - by) - D * vy;
            let fz = K * (dragTarget.z - bz) - D * vz;
            
            // Cap the net PD force to avoid rocket-like energy injection.
            // Use 3×weight so the user can still pull meaningfully.
            const maxForce = 3.0 * mass * 9.81;
            const netMag = Math.sqrt(fx * fx + fy * fy + fz * fz);
            if (netMag > maxForce) {
              const scale = maxForce / netMag;
              fx *= scale;
              fy *= scale;
              fz *= scale;
            }
            
            data.xfrc_applied[bId * 6 + 0] = fx;
            data.xfrc_applied[bId * 6 + 1] = fy;
            data.xfrc_applied[bId * 6 + 2] = fz;
          }
        }
        
        // Execute active control scripts immediately prior to physics iteration
        executeScripts(sceneGraph.nodes);

        mujoco.mj_step(model, data);
        
        // Record physics history frame
        if (!(window as any)._physics_history) {
          (window as any)._physics_history = [];
        }
        if (data && data.time !== undefined && model && mujoco) {
          const bodies: Record<string, any> = {};
          const joints: Record<string, any> = {};
          const collectNodeData = (nodesList: any[]) => {
            if (!nodesList) return;
            for (const node of nodesList) {
              const bodyName = node.id;
              const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyName);
              if (bId !== -1) {
                bodies[bodyName] = {
                  pos: [
                    data.xpos[bId * 3],
                    data.xpos[bId * 3 + 1],
                    data.xpos[bId * 3 + 2]
                  ],
                  vel: [
                    data.cvel[bId * 6 + 3],
                    data.cvel[bId * 6 + 4],
                    data.cvel[bId * 6 + 5]
                  ],
                  angvel: [
                    data.cvel[bId * 6 + 0],
                    data.cvel[bId * 6 + 1],
                    data.cvel[bId * 6 + 2]
                  ]
                };
              }
              node.joints?.forEach((j: any) => {
                const jId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, j.name);
                if (jId !== -1) {
                  const qposadr = model.jnt_qposadr[jId];
                  const dofadr = model.jnt_dofadr[jId];
                  joints[j.name] = {
                    pos: data.qpos[qposadr],
                    vel: data.qvel[dofadr]
                  };
                }
              });
              if (node.children) collectNodeData(node.children);
            }
          };
          collectNodeData(sceneGraph.nodes);
          (window as any)._physics_history.push({
            time: data.time,
            bodies,
            joints
          });
          if ((window as any)._physics_history.length > 5000) {
            (window as any)._physics_history.shift();
          }
        }
        
        // Safety check for NaN values in positions
        const nq = model.nq;
        for (let j = 0; j < nq; j++) {
          if (isNaN(data.qpos[j])) {
            console.error(`[PhysicsLoop] NaN detected in qpos at index ${j}! Stopping simulation.`);
            (window as any).DISABLE_USEFRAME = true;
            // Force pause in state
            useStore.setState({ isPlaying: false });
            return;
          }
        }
      } catch (e) {
        console.error("Simulation step error:", e);
        useStore.setState({ isPlaying: false });
        (window as any).DISABLE_USEFRAME = true;
        return;
      }
    }
  });
  
  return null;
};

// Camera Controller
const CameraController = () => {
  const { camera } = useThree();
  const cameraView = useStore(state => state.cameraView);
  const controlsRef = useRef<any>(null);
  
  useEffect(() => {
    if (cameraView === 'topDown') {
      camera.position.set(0, 15, 0);
      camera.up.set(0, 0, -1);
      camera.lookAt(0, 0, 0);
      if (controlsRef.current) {
        controlsRef.current.target.set(0, 0, 0);
        controlsRef.current.update();
      }
    } else {
      camera.position.set(5, 2, 5);
      camera.up.set(0, 1, 0);
      camera.lookAt(0, 0, 0);
      if (controlsRef.current) {
        controlsRef.current.target.set(0, 0, 0);
        controlsRef.current.update();
      }
    }
    camera.updateProjectionMatrix();
  }, [cameraView, camera]);

  const draggedNodeId = useStore((state) => state.draggedNodeId);
  return <OrbitControls enabled={draggedNodeId === null} ref={controlsRef} makeDefault enableDamping dampingFactor={0.1} mouseButtons={{ LEFT: 99 as any, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE }} />;
};

// Drop Handler for precise spawning
const DropHandler = ({ addComponent }: { addComponent: (type: 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear' | 'wedge' | 'pulley_wheel' | 'pulley_rope', pos: [number, number, number]) => void }) => {
  const { camera } = useThree();
  
  useEffect(() => {
    const handler = (e: DragEvent) => {
      e.preventDefault();
      const type = e.dataTransfer?.getData('type') as 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear' | 'wedge' | 'pulley_wheel' | 'pulley_rope';
      if (!type) return;
      
      const canvasEl = document.querySelector('canvas');
      let rect = { left: 0, top: 0, width: window.innerWidth, height: window.innerHeight };
      if (canvasEl) {
        rect = canvasEl.getBoundingClientRect();
      }
      const xLocal = e.clientX - rect.left;
      const yLocal = e.clientY - rect.top;

      const vec = new THREE.Vector3(
        (xLocal / rect.width) * 2 - 1,
        -(yLocal / rect.height) * 2 + 1,
        0.5
      );
      vec.unproject(camera);
      const dir = vec.sub(camera.position).normalize();
      
      // Intersect with Canvas Y=0 plane (which maps to MuJoCo Z=0)
      if (Math.abs(dir.y) < 0.001) return;
      const distance = -camera.position.y / dir.y; 
      if (distance < 0) return;
      
      const pos = camera.position.clone().add(dir.multiplyScalar(distance));
      
      let x = pos.x;
      let z = -pos.z;
      
      if (isNaN(x) || isNaN(z)) return;
      
      x = Math.max(-20, Math.min(20, x));
      z = Math.max(-20, Math.min(20, z));
      
      addComponent(type, [x, z, 2]);
    };
    
    const dragOverHandler = (e: DragEvent) => e.preventDefault();
    
    window.addEventListener('drop', handler);
    window.addEventListener('dragover', dragOverHandler);
    return () => {
      window.removeEventListener('drop', handler);
      window.removeEventListener('dragover', dragOverHandler);
    };
  }, [camera, addComponent]);
  
  return null;
};


// Custom Triangular Prism Wedge Geometry
const WedgeGeometry = ({ width = 2.0, depth = 1.0, height = 0.5 }: { width: number; depth: number; height: number }) => {
  const vertices = useMemo(() => {
    const L = Math.sqrt(width * width + height * height);
    const D = depth;
    const cosTheta = width / L;
    const H_local = height / cosTheta;
    const halfL = L / 2;
    const halfD = D / 2;

    // 6 Vertices of the pre-tilted solid wedge prism:
    // T0, T1, T2, T3 (top slanted face at z = 0)
    // B0, B1 (bottom back vertices at z = -H_local so that base becomes perfectly horizontal after -theta rotation)
    return new Float32Array([
      -halfL, -halfD, 0,          // 0: T0
       halfL, -halfD, 0,          // 1: T1
       halfL,  halfD, 0,          // 2: T2
      -halfL,  halfD, 0,          // 3: T3
      -halfL, -halfD, -H_local,   // 4: B0
      -halfL,  halfD, -H_local    // 5: B1
    ]);
  }, [width, depth, height]);

  const indices = useMemo(() => {
    return new Uint16Array([
      // Slanted Top Face (looking up)
      0, 2, 1,
      0, 3, 2,
      // Bottom Face (looking down)
      1, 5, 4,
      1, 2, 5,
      // Back Vertical Wall (looking left)
      0, 4, 5,
      0, 5, 3,
      // Front Triangle side (y = -halfD, looking front)
      0, 1, 4,
      // Back Triangle side (y = halfD, looking back)
      3, 5, 2
    ]);
  }, []);

  const geomRef = useRef<THREE.BufferGeometry>(null);
  useEffect(() => {
    if (geomRef.current) {
      geomRef.current.computeVertexNormals();
    }
  }, [vertices]);

  return (
    <bufferGeometry ref={geomRef}>
      <bufferAttribute
        attach="attributes-position"
        args={[vertices, 3]}
      />
      <bufferAttribute
        attach="index"
        args={[indices, 1]}
      />
    </bufferGeometry>
  );
};


// Dynamic Geom Renderer
const DynamicGeom = ({ nodeId, name, type, color, mujoco, model, data, selectedNodeId, setSelectedNodeId, vertices, faces, dynamic: isDynamic }: any) => {
  const meshRef = useRef<THREE.Group>(null);
  const isPlaying = useStore(state => state.isPlaying);
  
  const node = useStore(state => {
    const find = (nodes: any[]): any => {
      if (!nodes) return null;
      for (const n of nodes) {
        if (n.id === nodeId) return n;
        const c = find(n.children);
        if (c) return c;
      }
      return null;
    };
    return find(state.sceneGraph.nodes);
  });
  
  const geomId = useMemo(() => {
    if (!model || !mujoco) return -1;
    const id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, name);
    console.log(`[DynamicGeom ${name}] computed geomId:`, id);
    return id;
  }, [model, mujoco, name]);

  const geometryArgs = useMemo(() => {
    if (geomId === -1 || !model) return [];
    try {
      const ngeom = model.ngeom;
      if (geomId >= ngeom) return [];

      const r = model.geom_size[geomId * 3];
      const hl = model.geom_size[geomId * 3 + 1];
      const hz = model.geom_size[geomId * 3 + 2];
      
      if (type === 'sphere') return [r, 32, 32];
      if (type === 'box') return [r * 2, hl * 2, hz * 2];
      if (type === 'capsule') return [r, hl * 2, 4, 16];
      if (type === 'cylinder') return [r, hl];
      if (type === 'ellipsoid') return [r, hl, hz];
      return [r];
    } catch (e) {
      console.error(`[DynamicGeom ${name}] geometryArgs Error:`, e);
      return [];
    }
  }, [geomId, type, model]);

  const rotationMatrix = useMemo(() => new THREE.Matrix4(), []);
  const isSelected = selectedNodeId === nodeId;

  // Handlers for physical spring dragging, mapped from Three.js coordinates to MuJoCo coordinate space
  const dragHandlers = useMemo(() => ({
    onClick: (e: any) => {
      e.stopPropagation();
      setSelectedNodeId(nodeId);
    },
    onPointerDown: (e: any) => {
      if (isPlaying) {
        e.stopPropagation();
        useStore.getState().setDraggedNodeId(nodeId);
        useStore.getState().setDragDistance(e.distance);
        
        const pt = e.point;
        // Transform standard Three.js world coordinates (Y-up) to MuJoCo coordinate space (Z-up)
        useStore.getState().setDragTarget({ x: pt.x, y: -pt.z, z: pt.y });
        try {
          (e.target as HTMLElement).setPointerCapture(e.pointerId);
        } catch (err) {}
      }
    },
    onPointerUp: (e: any) => {
      if (useStore.getState().draggedNodeId === nodeId) {
        e.stopPropagation();
        try {
          (e.target as HTMLElement).releasePointerCapture(e.pointerId);
        } catch (err) {}
        useStore.getState().setDraggedNodeId(null);
        useStore.getState().setDragTarget(null);
      }
    },
    onPointerCancel: (e: any) => {
      if (useStore.getState().draggedNodeId === nodeId) {
        e.stopPropagation();
        try {
          (e.target as HTMLElement).releasePointerCapture(e.pointerId);
        } catch (err) {}
        useStore.getState().setDraggedNodeId(null);
        useStore.getState().setDragTarget(null);
      }
    }
  }), [isPlaying, nodeId, setSelectedNodeId]);

  // For dynamic meshes, use body xpos/xmat so renderVertices (centroid-local) align correctly.
  const bodyId = useMemo(() => {
    if (!isDynamic || !model || !mujoco) return -1;
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, nodeId);
  }, [isDynamic, model, mujoco, nodeId]);

  // Compute initial position and rotation from the model/data
  const [initialPos, initialQuat] = useMemo(() => {
    if (!model || !data) return [[0, 0, 0] as [number, number, number], [0, 0, 0, 1] as [number, number, number, number]];
    try {
      // Dynamic meshes: use body xpos/xmat (renderVertices are in body-local space)
      if (isDynamic && bodyId !== -1) {
        const px = data.xpos[bodyId * 3];
        const py = data.xpos[bodyId * 3 + 1];
        const pz = data.xpos[bodyId * 3 + 2];
        const m = data.xmat;
        const offset = bodyId * 9;
        const mat = new THREE.Matrix4().set(
          m[offset], m[offset+1], m[offset+2], 0,
          m[offset+3], m[offset+4], m[offset+5], 0,
          m[offset+6], m[offset+7], m[offset+8], 0,
          0, 0, 0, 1
        );
        const q = new THREE.Quaternion().setFromRotationMatrix(mat);
        return [[px, py, pz] as [number, number, number], [q.x, q.y, q.z, q.w] as [number, number, number, number]];
      }
      if (geomId === -1) return [[0, 0, 0] as [number, number, number], [0, 0, 0, 1] as [number, number, number, number]];
      const ngeom = model.ngeom;
      if (geomId >= ngeom) return [[0, 0, 0] as [number, number, number], [0, 0, 0, 1] as [number, number, number, number]];

      const px = data.geom_xpos[geomId * 3];
      const py = data.geom_xpos[geomId * 3 + 1];
      const pz = data.geom_xpos[geomId * 3 + 2];

      const m = data.geom_xmat;
      const offset = geomId * 9;
      const mat = new THREE.Matrix4().set(
        m[offset],     m[offset + 1], m[offset + 2], 0,
        m[offset + 3], m[offset + 4], m[offset + 5], 0,
        m[offset + 6], m[offset + 7], m[offset + 8], 0,
        0,             0,             0,             1
      );
      const q = new THREE.Quaternion().setFromRotationMatrix(mat);
      return [[px, py, pz] as [number, number, number], [q.x, q.y, q.z, q.w] as [number, number, number, number]];
    } catch (e) {
      return [[0, 0, 0] as [number, number, number], [0, 0, 0, 1] as [number, number, number, number]];
    }
  }, [isDynamic, bodyId, geomId, model, data]);

  useFrame(() => {
    // Safety check: ensure closure model/data match current store active ones
    const activeModel = useStore.getState().model;
    const activeData = useStore.getState().data;
    if (model !== activeModel || data !== activeData) return;

    if ((window as any).DISABLE_USEFRAME) return;
    if (type === 'mesh' && !isDynamic) return;
    if (!meshRef.current || !model || !data) return;

    try {
      // Dynamic meshes: track body xpos/xmat (renderVertices are in body-local space)
      if (isDynamic && bodyId !== -1) {
        const px = data.xpos[bodyId * 3];
        const py = data.xpos[bodyId * 3 + 1];
        const pz = data.xpos[bodyId * 3 + 2];
        const m = data.xmat;
        const offset = bodyId * 9;
        rotationMatrix.set(
          m[offset], m[offset+1], m[offset+2], 0,
          m[offset+3], m[offset+4], m[offset+5], 0,
          m[offset+6], m[offset+7], m[offset+8], 0,
          0, 0, 0, 1
        );
        meshRef.current.position.set(px, py, pz);
        meshRef.current.quaternion.setFromRotationMatrix(rotationMatrix);
        return;
      }

      if (geomId === -1) return;
      const ngeom = model.ngeom;
      if (geomId >= ngeom) return;

      const px = data.geom_xpos[geomId * 3];
      const py = data.geom_xpos[geomId * 3 + 1];
      const pz = data.geom_xpos[geomId * 3 + 2];

      const m = data.geom_xmat;
      const offset = geomId * 9;
      rotationMatrix.set(
        m[offset],     m[offset + 1], m[offset + 2], 0,
        m[offset + 3], m[offset + 4], m[offset + 5], 0,
        m[offset + 6], m[offset + 7], m[offset + 8], 0,
        0,             0,             0,             1
      );

      meshRef.current.position.set(px, py, pz);
      meshRef.current.quaternion.setFromRotationMatrix(rotationMatrix);
    } catch (e) {
      // Safely ignore deleted object or transition errors
    }
  });

  // Build Three.js BufferGeometry from inline vertex/face arrays for mesh type
  const meshBufferGeometry = useMemo(() => {
    if (type !== 'mesh' || !vertices || !faces) return null;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(vertices), 3));
    geo.setIndex(new THREE.BufferAttribute(new Uint32Array(faces), 1));
    geo.computeVertexNormals();
    return geo;
  }, [type, vertices, faces]);

  if (type === 'mesh') {
    if (!meshBufferGeometry) return null;
    if (isDynamic) {
      // Dynamic mesh: transform tracked from MuJoCo via geom_xpos/geom_xmat (Z-up coords, handled by parent group rotation).
      return (
        <group ref={meshRef} position={initialPos} quaternion={new THREE.Quaternion(...initialQuat)}>
          <mesh castShadow receiveShadow geometry={meshBufferGeometry} {...dragHandlers}>
            <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} side={THREE.DoubleSide} />
          </mesh>
        </group>
      );
    }
    // Static mesh: vertices baked in Three.js world space — no position/rotation applied.
    return (
      <group>
        <mesh castShadow receiveShadow geometry={meshBufferGeometry} {...dragHandlers}>
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} side={THREE.DoubleSide} />
        </mesh>
      </group>
    );
  }

  if (geomId === -1 || !geometryArgs || geometryArgs.length === 0 || geometryArgs.some(arg => arg === undefined || isNaN(arg))) {
    return null;
  }

  return (
    <group
      ref={meshRef}
      position={initialPos}
      quaternion={new THREE.Quaternion(...initialQuat)}
    >
      {node?.isWedge ? (
        <mesh castShadow receiveShadow {...dragHandlers}>
          <WedgeGeometry width={node.width || 2.0} depth={node.depth || 1.0} height={node.height || 0.5} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      ) : type === 'sphere' ? (
        <mesh castShadow receiveShadow {...dragHandlers}>
          <sphereGeometry args={geometryArgs as any} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      ) : type === 'box' ? (
        <mesh castShadow receiveShadow {...dragHandlers}>
          <boxGeometry args={geometryArgs as any} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      ) : type === 'ellipsoid' ? (
        <mesh castShadow receiveShadow scale={[geometryArgs[0], geometryArgs[1], geometryArgs[2]]} {...dragHandlers}>
          <sphereGeometry args={[1, 32, 32]} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      ) : null}
      {type === 'capsule' && (
        <mesh castShadow receiveShadow rotation={[Math.PI / 2, 0, 0]} {...dragHandlers}>
          <capsuleGeometry args={geometryArgs as any} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      )}
      {type === 'cylinder' && (
        <mesh castShadow receiveShadow rotation={[Math.PI / 2, 0, 0]} {...dragHandlers}>
          <cylinderGeometry args={[geometryArgs[0], geometryArgs[0], geometryArgs[1] * 2, 32]} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      )}
    </group>
  );
};

// Dynamic glowing pulley cable/rope renderer
const PulleyRopesRenderer = ({ model, data, mujoco, sceneGraph }: any) => {
  const lineRefs = useRef<{ [ropeId: string]: any }>({});
  
  // Find all pulley rope nodes in the scene
  const pulleyRopes = useMemo(() => {
    const ropes: any[] = [];
    const traverse = (nodes: any[]) => {
      if (!nodes) return;
      for (const n of nodes) {
        if (n.isPulleyRope && n.leftTargetId && n.rightTargetId) {
          ropes.push(n);
        }
        traverse(n.children);
      }
    };
    traverse(sceneGraph.nodes);
    return ropes;
  }, [sceneGraph]);

  // Helper to find wheel node radius reactively
  const findWheelNode = useCallback((wheelId: string) => {
    const traverse = (nodes: any[]): any => {
      if (!nodes) return null;
      for (const n of nodes) {
        if (n.id === wheelId) return n;
        const c = traverse(n.children);
        if (c) return c;
      }
      return null;
    };
    return traverse(sceneGraph.nodes);
  }, [sceneGraph]);

  useFrame(() => {
    const activeModel = useStore.getState().model;
    const activeData = useStore.getState().data;
    if (model !== activeModel || data !== activeData) return;
    if ((window as any).DISABLE_USEFRAME) return;
    if (!model || !data || !mujoco) return;

    for (const rope of pulleyRopes) {
      try {
        const leftId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, rope.leftTargetId);
        const rightId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, rope.rightTargetId);

        if (leftId === -1 || rightId === -1) continue;

        const lx = data.xpos[leftId * 3];
        const ly = data.xpos[leftId * 3 + 1];
        const lz = data.xpos[leftId * 3 + 2];

        const rx = data.xpos[rightId * 3];
        const ry = data.xpos[rightId * 3 + 1];
        const rz = data.xpos[rightId * 3 + 2];

        const points: THREE.Vector3[] = [];

        if (rope.pulleyWheelId) {
          // Pulley wheel present: arc-over-wheel geometry
          const wheelId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, rope.pulleyWheelId);
          if (wheelId === -1) {
            // Wheel not yet spawned — fall back to straight line
            points.push(new THREE.Vector3(lx, ly, lz));
            points.push(new THREE.Vector3(rx, ry, rz));
          } else {
            const wx = data.xpos[wheelId * 3];
            const wy = data.xpos[wheelId * 3 + 1];
            const wz = data.xpos[wheelId * 3 + 2];
            const wheelNode = findWheelNode(rope.pulleyWheelId);
            const rad = wheelNode?.pulleyRadius || 0.4;

            points.push(new THREE.Vector3(lx, ly, lz + 0.15));
            points.push(new THREE.Vector3(wx - rad, wy, wz));
            const segments = 12;
            for (let i = 1; i < segments; i++) {
              const phi = Math.PI - (Math.PI * i) / segments;
              points.push(new THREE.Vector3(
                wx + rad * Math.cos(phi),
                wy,
                wz + rad * Math.sin(phi)
              ));
            }
            points.push(new THREE.Vector3(wx + rad, wy, wz));
            points.push(new THREE.Vector3(rx, ry, rz + 0.15));
          }
        } else {
          // No wheel — straight rope between the two bodies
          points.push(new THREE.Vector3(lx, ly, lz));
          points.push(new THREE.Vector3(rx, ry, rz));
        }

        const line = lineRefs.current[rope.id];
        if (line) {
          line.geometry.setFromPoints(points);
        }
      } catch (e) {
        // Safe check
      }
    }
  });

  if (pulleyRopes.length === 0) return null;

  return (
    <>
      {pulleyRopes.map((rope) => (
        <line key={rope.id} ref={(el) => { lineRefs.current[rope.id] = el; }}>
          <bufferGeometry />
          <lineBasicMaterial color="#3b82f6" linewidth={3.5} transparent opacity={0.9} />
        </line>
      ))}
    </>
  );
};


// Drag interaction controller that handles window-level mouse/pointer movements
const DragInteractionController = () => {
  const { camera, raycaster, gl } = useThree();
  
  useEffect(() => {
    const handlePointerMove = (e: PointerEvent) => {
      const { draggedNodeId, dragDistance } = useStore.getState();
      if (!draggedNodeId) return;

      // Project mouse screen coordinates relative to canvas bounding client rect
      const rect = gl.domElement.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      
      const ndcX = (x / rect.width) * 2 - 1;
      const ndcY = -(y / rect.height) * 2 + 1;

      raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
      
      const targetPt = new THREE.Vector3();
      raycaster.ray.at(dragDistance, targetPt);

      // Transform standard Three.js world coordinates (Y-up) to MuJoCo coordinate space (Z-up)
      useStore.getState().setDragTarget({
        x: targetPt.x,
        y: -targetPt.z,
        z: targetPt.y
      });
    };

    const handlePointerUp = () => {
      const { draggedNodeId } = useStore.getState();
      if (draggedNodeId) {
        useStore.getState().setDraggedNodeId(null);
        useStore.getState().setDragTarget(null);
      }
    };

    window.addEventListener('pointermove', handlePointerMove);
    window.addEventListener('pointerup', handlePointerUp);
    window.addEventListener('pointercancel', handlePointerUp);

    return () => {
      window.removeEventListener('pointermove', handlePointerMove);
      window.removeEventListener('pointerup', handlePointerUp);
      window.removeEventListener('pointercancel', handlePointerUp);
    };
  }, [camera, raycaster, gl]);

  return null;
};


// Real-time mouse drag physical spring force line renderer
const MouseDragForceRenderer = ({ model, data, mujoco }: any) => {
  const draggedNodeId = useStore((state) => state.draggedNodeId);
  const dragTarget = useStore((state) => state.dragTarget);
  const lineRef = useRef<any>(null);

  useFrame(() => {
    const activeModel = useStore.getState().model;
    const activeData = useStore.getState().data;
    if (model !== activeModel || data !== activeData) return;
    if ((window as any).DISABLE_USEFRAME) return;
    if (!model || !data || !mujoco || !draggedNodeId || !dragTarget || !lineRef.current) return;

    try {
      const bId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, draggedNodeId);
      if (bId === -1) return;

      const px = data.xpos[bId * 3];
      const py = data.xpos[bId * 3 + 1];
      const pz = data.xpos[bId * 3 + 2];

      const points = [
        new THREE.Vector3(px, py, pz),
        new THREE.Vector3(dragTarget.x, dragTarget.y, dragTarget.z)
      ];
      lineRef.current.geometry.setFromPoints(points);
    } catch (e) {
      // Safe check
    }
  });

  if (!draggedNodeId || !dragTarget) return null;

  return (
    <line ref={lineRef}>
      <bufferGeometry />
      <lineBasicMaterial color="#f43f5e" linewidth={4} transparent opacity={0.9} />
    </line>
  );
};

// Rope node placeholder marker – renders a glowing ring for each pulley_rope scene node
const PulleyRopeMarkers = ({ sceneGraph, selectedNodeId, setSelectedNodeId }: any) => {
  const isPlaying = useStore(state => state.isPlaying);

  const ropeNodes = useMemo(() => {
    const ropes: any[] = [];
    const traverse = (nodes: any[]) => {
      if (!nodes) return;
      for (const n of nodes) {
        if (n.isPulleyRope) ropes.push(n);
        traverse(n.children);
      }
    };
    traverse(sceneGraph.nodes);
    return ropes;
  }, [sceneGraph]);

  if (ropeNodes.length === 0) return null;

  return (
    <>
      {ropeNodes.map((rope) => {
        // pos is [x, y_mujoco, z_mujoco] in scene graph space.
        // The SceneVisuals group is rotated [-PI/2, 0, 0], so we skip that rotation
        // and place markers in raw world space (no group rotation wrapper here).
        // MuJoCo X→Three.js X, MuJoCo Y→Three.js -Z, MuJoCo Z→Three.js Y
        const [mx, my, mz] = rope.pos;
        const threePos: [number, number, number] = [mx, mz, -my];
        const isSelected = selectedNodeId === rope.id;

        return (
          <group key={rope.id} position={threePos}>
            {/* Outer glowing torus ring */}
            <mesh
              rotation={[Math.PI / 2, 0, 0]}
              onClick={(e: any) => { e.stopPropagation(); setSelectedNodeId(rope.id); }}
              onPointerDown={(e: any) => {
                if (isPlaying) {
                  e.stopPropagation();
                  useStore.getState().setDraggedNodeId(rope.id);
                  useStore.getState().setDragDistance(e.distance);
                  const pt = e.point;
                  useStore.getState().setDragTarget({ x: pt.x, y: -pt.z, z: pt.y });
                }
              }}
              onPointerUp={(e: any) => {
                if (useStore.getState().draggedNodeId === rope.id) {
                  e.stopPropagation();
                  useStore.getState().setDraggedNodeId(null);
                  useStore.getState().setDragTarget(null);
                }
              }}
            >
              <torusGeometry args={[0.18, 0.035, 12, 40]} />
              <meshStandardMaterial
                color={isSelected ? '#60a5fa' : '#10b981'}
                emissive={isSelected ? '#3b82f6' : '#047857'}
                emissiveIntensity={isSelected ? 0.8 : 0.4}
                transparent
                opacity={0.92}
              />
            </mesh>
            {/* Small inner dot */}
            <mesh rotation={[Math.PI / 2, 0, 0]}>
              <torusGeometry args={[0.07, 0.025, 8, 24]} />
              <meshStandardMaterial
                color={isSelected ? '#93c5fd' : '#6ee7b7'}
                emissive={isSelected ? '#93c5fd' : '#6ee7b7'}
                emissiveIntensity={0.5}
                transparent
                opacity={0.85}
              />
            </mesh>
          </group>
        );
      })}
    </>
  );
};


const SceneVisuals = ({ model, data, mujoco, sceneGraph, selectedNodeId, setSelectedNodeId }: any) => {
  const geoms = useMemo(() => {
    if (!sceneGraph) return [];
    const list: any[] = [];
    const traverse = (nodes: any[]) => {
      if (!nodes) return;
      for (const node of nodes) {
        if (node.geoms) {
          for (const geom of node.geoms) {
            list.push({ nodeId: node.id, ...geom });
          }
        }
        traverse(node.children);
      }
    };
    traverse(sceneGraph.nodes);
    return list;
  }, [sceneGraph]);

  if (!model || !data || !mujoco) return null;
  
  const primitiveGeoms = geoms.filter(g => g.type !== 'mesh');
  const staticMeshGeoms = geoms.filter(g => g.type === 'mesh' && !g.dynamic);
  const dynamicMeshGeoms = geoms.filter(g => g.type === 'mesh' && g.dynamic);

  return (
    <>
      {/* Primitive geoms and dynamic meshes live in a Z-up→Y-up rotated group */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        {primitiveGeoms.map(g => (
          <DynamicGeom
            key={g.name}
            nodeId={g.nodeId}
            name={g.name}
            type={g.type}
            color={g.rgba || [0.8,0.8,0.8,1]}
            mujoco={mujoco}
            model={model}
            data={data}
            selectedNodeId={selectedNodeId}
            setSelectedNodeId={setSelectedNodeId}
          />
        ))}
        {dynamicMeshGeoms.map(g => (
          <DynamicGeom
            key={g.name}
            nodeId={g.nodeId}
            name={g.name}
            type={g.type}
            color={g.rgba || [0.8,0.8,0.8,1]}
            mujoco={mujoco}
            model={model}
            data={data}
            selectedNodeId={selectedNodeId}
            setSelectedNodeId={setSelectedNodeId}
            vertices={g.renderVertices}
            faces={g.faces}
            dynamic={true}
          />
        ))}
        <PulleyRopesRenderer model={model} data={data} mujoco={mujoco} sceneGraph={sceneGraph} />
        <MouseDragForceRenderer model={model} data={data} mujoco={mujoco} />
      </group>
      {/* Static mesh geoms: vertices already in Three.js Y-up space, no rotation needed */}
      {staticMeshGeoms.map(g => (
        <DynamicGeom
          key={g.name}
          nodeId={g.nodeId}
          name={g.name}
          type={g.type}
          color={g.rgba || [0.8,0.8,0.8,1]}
          mujoco={mujoco}
          model={model}
          data={data}
          selectedNodeId={selectedNodeId}
          setSelectedNodeId={setSelectedNodeId}
          vertices={g.vertices}
          faces={g.faces}
        />
      ))}
    </>
  );
};

const CAMERA_CONFIG = { position: [5, 2, 5] as [number, number, number], fov: 50 };

function App() {
  if (typeof window !== 'undefined') {
    (window as any).useStore = useStore;
  }
  useMuJoCoInit();
  const [isDocsOpen, setIsDocsOpen] = useState(false);
  const [docsTab, setDocsTab] = useState<'gravity' | 'coupling' | 'collision' | 'friction' | 'scripting'>('gravity');
  const [scriptText, setScriptText] = useState('');
  const [scriptError, setScriptError] = useState<string | null>(null);
  const [showApiRef, setShowApiRef] = useState(false);
  const [propertiesWidth, setPropertiesWidth] = useState(380);
  const [isLeftSidebarOpen, setIsLeftSidebarOpen] = useState(false);

  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    const handleMouseMove = (moveEvent: MouseEvent) => {
      const newWidth = window.innerWidth - moveEvent.clientX;
      if (newWidth >= 280 && newWidth <= 800) {
        setPropertiesWidth(newWidth);
      }
    };
    const handleMouseUp = () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
  }, []);

  const { 
    model, data, mujoco, recompileId,
    isPlaying, togglePlay, isLoaded, 
    isSettingsOpen, setSettingsOpen, 
    gravityZ, windX, windY, density, floorFriction, setEnvironment,
    cameraView, setCameraView,
    sceneGraph, selectedNodeId, setSelectedNodeId,
    updateNodeGeom, updateNodeJoint, updateGearTeeth, addComponent, loadPreset, updateScene,
    resetSimulation, updateNodePos,
    updateNodeJointsList, deleteNode, renameNode,
    addPusherPeg, deletePusherPeg, updatePusherPeg, updateNodeRotation,
    updateWedgeParams, updatePulleyParams, updateRopeParams,
    parentUnderSelected, setParentUnderSelected, updateNodeScript, updateNode
  } = useStore();

  // Helper to find a node by ID in hierarchy
  const findNodeById = useCallback((nodes: any[], targetId: string): any | null => {
    for (const node of nodes) {
      if (node.id === targetId) return node;
      if (node.children) {
        const res = findNodeById(node.children, targetId);
        if (res) return res;
      }
    }
    return null;
  }, []);

  // Helper to get recursive world position of a node
  const getNodeWorldPos = useCallback((nodes: any[], targetId: string, currentOffset: [number, number, number] = [0, 0, 0]): [number, number, number] | null => {
    for (const node of nodes) {
      const nodeWorld: [number, number, number] = [
        currentOffset[0] + node.pos[0],
        currentOffset[1] + node.pos[1],
        currentOffset[2] + node.pos[2]
      ];
      if (node.id === targetId) return nodeWorld;
      if (node.children) {
        const childResult = getNodeWorldPos(node.children, targetId, nodeWorld);
        if (childResult) return childResult;
      }
    }
    return null;
  }, []);

  const allPulleyWheels = useMemo(() => {
    const list: any[] = [];
    const traverse = (nodes: any[]) => {
      if (!nodes) return;
      for (const n of nodes) {
        if (n.isPulleyWheel) list.push(n);
        traverse(n.children);
      }
    };
    traverse(sceneGraph.nodes);
    return list;
  }, [sceneGraph]);

  const allJointedNodes = useMemo(() => {
    const list: any[] = [];
    const traverse = (nodes: any[]) => {
      if (!nodes) return;
      for (const n of nodes) {
        if (n.joints && n.joints.length > 0 && !n.isPulleyWheel) {
          list.push(n);
        }
        traverse(n.children);
      }
    };
    traverse(sceneGraph.nodes);
    return list;
  }, [sceneGraph]);

  const handlePointerMissed = useCallback(() => setSelectedNodeId(null), [setSelectedNodeId]);

  console.log("App rendering, selectedNodeId:", selectedNodeId);

  const handleDragStart = (e: React.DragEvent, type: string) => {
    e.dataTransfer.setData('type', type);
  };

  // Find selected node details
  const selectedNode = useMemo(() => {
    if (!selectedNodeId) return null;
    let found = null;
    const traverse = (nodes: any[]) => {
      if (!nodes) return;
      for (const node of nodes) {
        if (node.id === selectedNodeId) found = node;
        traverse(node.children);
      }
    };
    traverse(sceneGraph.nodes);
    console.log("evaluated selectedNode:", found);
    return found as any;
  }, [selectedNodeId, sceneGraph]);

  // Sync selected node's script into local text state
  useEffect(() => {
    if (selectedNode) {
      setScriptText(selectedNode.script || '');
      setScriptError(null);
    } else {
      setScriptText('');
      setScriptError(null);
    }
  }, [selectedNodeId, selectedNode?.id]);

  const handleSaveScript = useCallback(() => {
    if (!selectedNode) return;
    try {
      if (scriptText.trim() !== '') {
        // Syntax compilation check
        new Function('api', scriptText);
      }
      setScriptError(null);
      updateNodeScript(selectedNode.id, scriptText);
    } catch (e: any) {
      setScriptError(e.message || 'Compilation Error');
    }
  }, [selectedNode, scriptText, updateNodeScript]);

  // Utility to handle moving free bodies
  const handleMove = (axis: 0 | 1 | 2, val: number) => {
    if (!selectedNode) return;
    const cleanVal = isNaN(val) ? 0 : val;

    // Always update the scene-graph initial position (persists on reset/restart)
    const currentPos = [...selectedNode.pos] as [number, number, number];
    currentPos[axis] = cleanVal;
    updateNodePos(selectedNode.id, currentPos);

    // Also directly write to qpos so only THIS body moves in the live sim,
    // regardless of whether playing or paused. This avoids the full forceReset
    // recompile (from updateNodePos alone) which was snapping all other bodies
    // back to their initial positions.
    if (model && mujoco && data) {
      const freeJoint = selectedNode.joints?.find((j: any) => j.type === 'free');
      if (freeJoint) {
        const jointId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, freeJoint.name);
        if (jointId !== -1) {
          const adr = model.jnt_qposadr[jointId];
          data.qpos[adr + axis] = cleanVal;
          // Zero out velocities to prevent crazy snaps
          const vadr = model.jnt_dofadr[jointId];
          for (let i = 0; i < 6; i++) data.qvel[vadr + i] = 0;
          // Force propagation to update positions in 3D visually
          mujoco.mj_forward(model, data);
        }
      }
    }
  };

  const handleAddComponentClick = (type: 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear' | 'wedge' | 'pulley_wheel' | 'pulley_rope') => {
    if (selectedNodeId) {
      const parentNode = findNodeById(sceneGraph.nodes, selectedNodeId);
      if (parentNode) {
        const worldPos = getNodeWorldPos(sceneGraph.nodes, selectedNodeId) || [0, 0, 0];
        const offset = (type === 'capsule' || type === 'bob') ? [0, 0, -0.6] : [0.5, 0, 0];
        addComponent(type, [worldPos[0] + offset[0], worldPos[1] + offset[1], worldPos[2] + offset[2]]);
        setIsLeftSidebarOpen(false);
        return;
      }
    }
    addComponent(type, [0, 0, 1.2]); // Spawn slightly above floor
    setIsLeftSidebarOpen(false);
  };

  const renderHierarchyNode = useCallback((node: any, depth: number = 0): React.ReactNode => {
    const isSelected = selectedNodeId === node.id;
    
    // Choose pretty visual emoji
    let emoji = '📦';
    if (node.id.includes('gear')) emoji = '⚙️';
    else if (node.id.includes('pole') || node.id.includes('capsule')) emoji = '🥢';
    else if (node.id.includes('bob')) emoji = '🔵';
    else if (node.id.includes('cylinder')) emoji = '🛢️';
    else if (node.id.includes('sphere')) emoji = '🟢';
    else if (node.id.includes('wedge')) emoji = '📐';
    else if (node.id.includes('pulley_wheel')) emoji = '🛞';
    else if (node.isPulleyRope) emoji = '🧵';

    return (
      <div key={node.id} className="flex flex-col">
        <div 
          onClick={() => {
            setSelectedNodeId(node.id);
            setIsLeftSidebarOpen(false);
          }} 
          style={{ paddingLeft: `${depth * 14 + 8}px` }}
          className={`flex items-center px-2 py-1.5 rounded-md border cursor-pointer transition-colors shadow-sm mb-1 ${isSelected ? 'bg-blue-50 border-blue-200 text-blue-600 font-semibold' : 'bg-white border-transparent hover:bg-slate-100/70 text-slate-600'}`}
        >
          <span className="text-sm flex items-center gap-1.5">
            <span>{emoji}</span> {node.name}
          </span>
        </div>
        {node.children && node.children.map((child: any) => renderHierarchyNode(child, depth + 1))}
      </div>
    );
  }, [selectedNodeId, setSelectedNodeId, findNodeById, setIsLeftSidebarOpen]);

  useMCPBridge();

  return (
    <div className="flex flex-col h-screen w-screen bg-slate-50 text-slate-900 font-sans">
      <header className="glass-panel h-14 flex items-center justify-between px-3 md:px-6 z-10 border-b border-slate-200">
        <div className="flex items-center gap-2 md:gap-3">
          {/* Mobile Sidebar Toggle */}
          <button
            onClick={() => setIsLeftSidebarOpen(!isLeftSidebarOpen)}
            className="p-1.5 rounded-lg text-slate-600 hover:bg-slate-100 hover:text-slate-900 transition-colors md:hidden focus:outline-none cursor-pointer flex-shrink-0"
            title="Toggle Sidebar"
          >
            {isLeftSidebarOpen ? <X className="w-5 h-5" /> : <Menu className="w-5 h-5" />}
          </button>

          <div className="w-8 h-8 rounded-lg bg-blue-500/10 flex items-center justify-center flex-shrink-0">
            <Settings2 className="w-5 h-5 physics-accent" />
          </div>
          <h1 className="font-bold text-lg tracking-wide hidden sm:block">
            Physics <span className="text-blue-500">Expt</span>
          </h1>
        </div>
        
        <div className="flex items-center gap-1.5 md:gap-4">
          {/* Presets Select */}
          <div className="flex items-center gap-1 md:gap-2">
            <span className="text-xs font-semibold text-slate-500 uppercase tracking-wider hidden lg:inline">Preset:</span>
            <select 
              onChange={(e) => loadPreset(e.target.value as any)}
              className="px-2 md:px-3 py-1.5 rounded-full bg-white hover:bg-slate-100 transition-colors border border-slate-200 text-xs md:text-sm font-medium text-slate-700 shadow-sm outline-none cursor-pointer focus:border-blue-500 max-w-[100px] sm:max-w-[130px] md:max-w-none"
              defaultValue="pendulum"
            >
              <option value="pendulum">Double Pendulum</option>
              <option value="cubes">Stacked Cubes</option>
              <option value="gears">Gear System</option>
              <option value="machine">Gear Train Machine</option>
              <option value="rack_pinion">Rack & Pinion</option>
              <option value="inclined_plane">Inclined Plane</option>
              <option value="pulley_system">Pulley Stand</option>
              <option value="cartpole">Cartpole</option>
              <option value="newtons_cradle">Newton's Cradle</option>
              <option value="suspension_bridge">Suspension Bridge</option>
              <option value="paper_plane">✈ Paper Plane</option>
              <option value="monkey_head">🐵 Monkey Head</option>
              <option value="golden_gate">🌉 Golden Gate Bridge</option>
              <option value="golden_gate_mesh">🌉 Golden Gate (Mesh)</option>
              <option value="mesh_collision">🔺 Mesh Collision Demo</option>
            </select>
          </div>

          <button
            onClick={() => setIsDocsOpen(true)}
            className="flex items-center justify-center w-8 h-8 md:w-9 md:h-9 rounded-full border-2 border-blue-200 text-blue-600 hover:bg-blue-50 hover:border-blue-300 transition-colors focus:outline-none flex-shrink-0 cursor-pointer"
            title="Documentation"
          >
            <Info className="w-4 h-4" />
          </button>

          <button 
            onClick={() => setSettingsOpen(!isSettingsOpen)}
            className={`flex items-center justify-center w-8 h-8 md:w-9 md:h-9 rounded-full border-2 transition-colors focus:outline-none flex-shrink-0 cursor-pointer ${
              isSettingsOpen 
                ? 'bg-blue-50 border-blue-300 text-blue-600' 
                : 'border-slate-200 text-slate-600 hover:bg-slate-50 hover:border-slate-300'
            }`}
            title="Global Settings"
          >
            <Settings className="w-4 h-4" />
          </button>

          <button 
            onClick={togglePlay}
            disabled={!isLoaded}
            className="flex items-center justify-center gap-1.5 w-8 h-8 md:w-28 py-1.5 rounded-full bg-white hover:bg-slate-100 transition-colors border border-slate-200 disabled:opacity-50 shadow-sm flex-shrink-0 cursor-pointer"
            title={isPlaying ? "Stop Simulation" : "Start Simulation"}
          >
            {isPlaying ? (
              <><Square className="w-4 h-4 text-red-500" /><span className="hidden md:inline text-sm font-medium">Stop</span></>
            ) : (
              <><Play className="w-4 h-4 text-emerald-500" /><span className="hidden md:inline text-sm font-medium text-slate-700">Simulate</span></>
            )}
          </button>

          <button 
            onClick={resetSimulation}
            disabled={!isLoaded}
            className="flex items-center justify-center gap-1.5 w-8 h-8 md:w-auto md:px-4 py-1.5 rounded-full bg-white hover:bg-slate-100 transition-colors border border-slate-200 disabled:opacity-50 shadow-sm text-slate-600 hover:text-slate-900 flex-shrink-0 cursor-pointer"
            title="Reset Simulation"
          >
            <RotateCcw className="w-4 h-4" /> <span className="hidden md:inline text-sm font-medium">Reset</span>
          </button>

          <button 
            onClick={() => setCameraView(cameraView === 'topDown' ? 'perspective' : 'topDown')}
            className={`flex items-center justify-center gap-1.5 w-8 h-8 md:w-auto md:px-4 py-1.5 rounded-full transition-colors border shadow-sm flex-shrink-0 cursor-pointer ${cameraView === 'topDown' ? 'bg-blue-100 text-blue-700 border-blue-200' : 'bg-white hover:bg-slate-100 border-slate-200 text-slate-600'}`}
            title="Toggle Top Down View"
          >
            <Eye className="w-4 h-4" /> <span className="hidden md:inline text-sm font-medium">{cameraView === 'topDown' ? 'Perspective' : 'Top Down'}</span>
          </button>
        </div>
      </header>

      <div className="flex flex-1 overflow-hidden relative">
        {/* Global Settings */}
        {isSettingsOpen && (
          <div className="absolute top-4 right-6 w-64 glass-panel rounded-lg p-4 z-30 shadow-lg border border-slate-200">
            <h3 className="font-semibold text-sm mb-4 flex items-center justify-between">
              <span className="flex items-center gap-2"><Settings className="w-4 h-4 text-slate-500" /> Environment</span>
              <button onClick={() => setSettingsOpen(false)}><X className="w-4 h-4 text-slate-400 hover:text-slate-600" /></button>
            </h3>
            <div className="flex flex-col gap-4">
              <div className="flex flex-col gap-1">
                <label className="text-xs font-medium text-slate-500 flex justify-between">Gravity Z <span>{gravityZ.toFixed(1)}</span></label>
                <input type="range" min="-20" max="20" step="0.1" value={gravityZ} onChange={(e) => setEnvironment({gravityZ: parseFloat(e.target.value)})} className="w-full accent-blue-500" />
              </div>
              <div className="flex flex-col gap-1">
                <label className="text-xs font-medium text-slate-500 flex justify-between">Wind X <span>{windX.toFixed(1)}</span></label>
                <input type="range" min="-10" max="10" step="0.1" value={windX} onChange={(e) => setEnvironment({windX: parseFloat(e.target.value)})} className="w-full accent-blue-500" />
              </div>
              <div className="flex flex-col gap-1">
                <label className="text-xs font-medium text-slate-500 flex justify-between">Wind Y <span>{windY.toFixed(1)}</span></label>
                <input type="range" min="-10" max="10" step="0.1" value={windY} onChange={(e) => setEnvironment({windY: parseFloat(e.target.value)})} className="w-full accent-blue-500" />
              </div>
              <div className="flex flex-col gap-1">
                <label className="text-xs font-medium text-slate-500 flex justify-between">Air Density (Drag) <span>{density.toFixed(2)}</span></label>
                <input type="range" min="0" max="5" step="0.01" value={density} onChange={(e) => setEnvironment({density: parseFloat(e.target.value)})} className="w-full accent-blue-500" />
              </div>
              <div className="flex flex-col gap-1">
                <label className="text-xs font-medium text-slate-500 flex justify-between">Floor Friction <span>{floorFriction.toFixed(2)}</span></label>
                <input type="range" min="0" max="2" step="0.01" value={floorFriction} onChange={(e) => setEnvironment({floorFriction: parseFloat(e.target.value)})} className="w-full accent-blue-500" />
              </div>
            </div>
          </div>
        )}

        {/* Mobile Sidebar Backdrop Scrim */}
        {isLeftSidebarOpen && (
          <div 
            className="fixed inset-0 bg-slate-950/40 backdrop-blur-xs z-15 md:hidden transition-all duration-300"
            onClick={() => setIsLeftSidebarOpen(false)}
          />
        )}

        {/* Left Sidebar */}
        <aside className={`w-64 md:w-56 shrink-0 glass-panel border-r border-slate-200 flex-col p-4 bg-white/95 md:bg-white/50 overflow-y-auto transition-transform duration-200 ease-in-out fixed md:relative inset-y-14 md:inset-auto left-0 z-20 shadow-2xl md:shadow-none ${
          isLeftSidebarOpen ? 'flex translate-x-0' : 'hidden md:flex -translate-x-full md:translate-x-0'
        }`}>
          <h2 className="text-xs font-semibold text-slate-400 uppercase tracking-wider mb-3">Hierarchy</h2>
          <div className="flex flex-col gap-1.5 mb-6">
            <div 
              className={`px-3 py-1.5 rounded-md border cursor-pointer transition-colors shadow-sm flex items-center gap-1.5 ${!selectedNodeId ? 'bg-blue-50 border-blue-200 text-blue-600 font-bold' : 'bg-white border-transparent hover:bg-slate-50 text-slate-600'}`}
              onClick={() => {
                setSelectedNodeId(null);
                setIsLeftSidebarOpen(false);
              }}
            >
              <span>🌍</span> <span className="text-sm font-medium">Worldbody</span>
            </div>
            <div className="flex flex-col mt-1">
              {sceneGraph.nodes.map(node => renderHierarchyNode(node, 0))}
            </div>
          </div>

          <h2 className="text-xs font-semibold text-slate-400 uppercase tracking-wider mb-1 flex items-center justify-between">
            <span>Components</span>
          </h2>
          <div className="text-[11px] font-medium text-slate-500 mb-2 bg-slate-100/80 px-2 py-1.5 rounded-lg border border-slate-200/50">
            Adding to: <span className="text-blue-600 font-semibold">{selectedNode && parentUnderSelected ? selectedNode.name : '🌍 Worldbody'}</span>
          </div>

          {selectedNode && (
            <label className="text-[11px] font-semibold text-slate-600 flex items-center gap-2 mb-3 bg-slate-50 border border-slate-200/60 p-2 rounded-lg cursor-pointer select-none hover:bg-slate-100/50 transition-colors shadow-sm">
              <input 
                type="checkbox" 
                checked={parentUnderSelected} 
                onChange={(e) => setParentUnderSelected(e.target.checked)} 
                className="w-4 h-4 rounded text-blue-600 border-slate-300 focus:ring-blue-400 accent-blue-500 cursor-pointer"
              />
              <span>Nest under selected</span>
            </label>
          )}

          <div className="flex flex-col gap-2.5">
            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'box')} 
              onClick={() => handleAddComponentClick('box')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Box className="w-4 h-4 text-rose-500 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Cube</span>
                <span className="text-[10px] text-slate-400">Box element</span>
              </div>
            </div>

            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'sphere')} 
              onClick={() => handleAddComponentClick('sphere')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Circle className="w-4 h-4 text-emerald-500 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Sphere</span>
                <span className="text-[10px] text-slate-400">Regular ball</span>
              </div>
            </div>

            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'cylinder')} 
              onClick={() => handleAddComponentClick('cylinder')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Layers className="w-4 h-4 text-amber-500 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Cylinder</span>
                <span className="text-[10px] text-slate-400">3D cylinder block</span>
              </div>
            </div>

            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'capsule')} 
              onClick={() => handleAddComponentClick('capsule')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Zap className="w-4 h-4 text-indigo-500 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Pole</span>
                <span className="text-[10px] text-slate-400">Capsule rod</span>
              </div>
            </div>


            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'gear')} 
              onClick={() => handleAddComponentClick('gear')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Settings className="w-4 h-4 text-slate-500 group-hover:rotate-45 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Gear</span>
                <span className="text-[10px] text-slate-400">Cogs with square teeth</span>
              </div>
            </div>

            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'wedge')} 
              onClick={() => handleAddComponentClick('wedge')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Triangle className="w-4 h-4 text-amber-600 rotate-90 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Wedge</span>
                <span className="text-[10px] text-slate-400">Procedural inclined plane</span>
              </div>
            </div>

            <div 
              draggable 
              onDragStart={(e) => handleDragStart(e, 'pulley_wheel')} 
              onClick={() => handleAddComponentClick('pulley_wheel')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Disc className="w-4 h-4 text-cyan-600 group-hover:spin transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Pulley Wheel</span>
                <span className="text-[10px] text-slate-400">Pulley stand system disk</span>
              </div>
            </div>

            <div
              draggable
              onDragStart={(e) => handleDragStart(e, 'pulley_rope')}
              onClick={() => handleAddComponentClick('pulley_rope')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <CircleDot className="w-4 h-4 text-emerald-600 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Rope</span>
                <span className="text-[10px] text-slate-400">Couples two bodies together</span>
              </div>
            </div>

            <div
              draggable
              onDragStart={(e) => handleDragStart(e, 'mesh')}
              onClick={() => handleAddComponentClick('mesh')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <Shapes className="w-4 h-4 text-violet-500 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Mesh</span>
                <span className="text-[10px] text-slate-400">Custom geometry (visual)</span>
              </div>
            </div>
          </div>
        </aside>

        {/* Viewport */}
        <main className="flex-1 relative min-w-0">
          {!isLoaded && (
            <div className="absolute inset-0 flex items-center justify-center z-20 pointer-events-none bg-slate-50/50 backdrop-blur-sm">
              <div className="text-slate-500 flex flex-col items-center gap-4 font-medium">
                <div className="w-8 h-8 border-4 border-blue-500 border-t-transparent rounded-full animate-spin"></div>
                Initializing MuJoCo...
              </div>
            </div>
          )}
          
          <Canvas camera={CAMERA_CONFIG} shadows onPointerMissed={handlePointerMissed}>
            <DropHandler addComponent={addComponent} />
            <color attach="background" args={['#f8fafc']} />
            <ambientLight intensity={0.6} />
            <directionalLight position={[5, 10, 5]} intensity={1.2} castShadow />
            <Grid infiniteGrid fadeDistance={20} cellColor="#e2e8f0" sectionColor="#cbd5e1" position={[0, -0.01, 0]} />
            
            {model && data && mujoco && (
              <PhysicsLoop 
                key={`loop-${recompileId}`} 
                model={model} 
                data={data} 
                mujoco={mujoco} 
                isPlaying={isPlaying} 
              />
            )}
            {model && data && mujoco && (
              <SceneVisuals 
                key={`visuals-${recompileId}`}
                model={model} 
                data={data} 
                mujoco={mujoco} 
                sceneGraph={sceneGraph} 
                selectedNodeId={selectedNodeId}
                setSelectedNodeId={setSelectedNodeId}
              />
            )}
            
            {/* Rope markers rendered in raw world space (no coordinate system rotation) */}
            <PulleyRopeMarkers
              sceneGraph={sceneGraph}
              selectedNodeId={selectedNodeId}
              setSelectedNodeId={setSelectedNodeId}
            />
            
            <CameraController />
            <DragInteractionController />
          </Canvas>
        </main>

        {/* Contextual Properties Sidebar */}
        {selectedNode && (
          <aside 
            style={{ width: `${propertiesWidth}px` }} 
            className="shrink-0 glass-panel border-l border-slate-200 flex flex-col p-4 z-20 bg-white/50 overflow-y-auto"
          >
            {/* Elegant Resize Handle */}
            <div
              onMouseDown={handleMouseDown}
              className="absolute top-0 left-0 bottom-0 w-1.5 cursor-col-resize hover:bg-blue-500/20 active:bg-blue-500/40 transition-colors z-20 group hidden md:flex items-center justify-center"
              title="Drag to resize panel"
            >
              <div className="w-[2px] h-8 bg-slate-300 group-hover:bg-blue-500 group-active:bg-blue-600 rounded transition-colors" />
            </div>

            <h2 className="text-xs font-semibold text-slate-400 uppercase tracking-wider mb-4 flex items-center justify-between">
              <span className="flex items-center gap-2"><SlidersHorizontal className="w-4 h-4" /> Properties</span>
              <button onClick={() => setSelectedNodeId(null)}><X className="w-4 h-4 text-slate-400 hover:text-slate-600" /></button>
            </h2>
            
            <div className="flex flex-col gap-4">
                <div className="flex items-center justify-between p-3 bg-slate-50 border border-slate-200/60 rounded-lg">
                  <div>
                    <div className="text-xs font-semibold text-slate-700">Aerodynamics</div>
                    <div className="text-[10px] text-slate-500">Apply lift and drag automatically</div>
                  </div>
                  <label className="relative inline-flex items-center cursor-pointer">
                    <input 
                      type="checkbox" 
                      className="sr-only peer"
                      checked={selectedNode.isAerodynamic || false}
                      onChange={(e) => updateNode(selectedNode.id, { isAerodynamic: e.target.checked })}
                    />
                    <div className="w-8 h-4 bg-slate-200 peer-focus:outline-none rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-slate-300 after:border after:rounded-full after:h-3 after:w-3 after:transition-all peer-checked:bg-blue-500"></div>
                  </label>
                </div>

                <div className="flex flex-col gap-1.5 p-3 bg-slate-50 border border-slate-200/60 rounded-lg">
                <div className="flex justify-between items-center">
                  <label className="text-[10px] font-semibold text-slate-400 uppercase tracking-wider">Component Name</label>
                  <span className="font-mono text-[9px] text-blue-600 font-semibold bg-blue-50 px-1 py-0.5 rounded cursor-pointer select-all border border-blue-100" title="Body API Reference Name. Click to select/copy.">
                    api.getPosition('{selectedNode.name || selectedNode.id}')
                  </span>
                </div>
                <input 
                  type="text" 
                  value={selectedNode.name || ''} 
                  onChange={(e) => renameNode(selectedNode.id, e.target.value)}
                  className="w-full px-2.5 py-1.5 border border-slate-200 rounded text-sm bg-white font-medium text-slate-800 outline-none focus:border-blue-500 shadow-sm"
                  placeholder="Rename component..."
                />
                <span className="text-[9px] font-mono text-slate-400 mt-0.5">ID: {selectedNode.id}</span>
              </div>

              {/* Joint Type Configuration */}
              <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center justify-between">
                  <span className="flex items-center gap-1">🔗 Joint Type</span>
                  <button 
                    onClick={() => {
                      setDocsTab('gravity');
                      setIsDocsOpen(true);
                    }} 
                    className="text-slate-400 hover:text-slate-600 transition-colors cursor-pointer" 
                    title="Click for documentation"
                  >
                    <Info className="w-3.5 h-3.5" />
                  </button>
                </h3>
                <select 
                  value={selectedNode.joints?.length > 0 ? selectedNode.joints[0].type : 'fixed'}
                  onChange={(e) => {
                    const jointType = e.target.value;
                    let newJoints: any[] = [];
                    if (jointType !== 'fixed') {
                      const name = `${selectedNode.id}_joint`;
                      if (jointType === 'free') {
                        newJoints = [{ name, type: 'free' }];
                      } else if (jointType === 'hinge') {
                        newJoints = [{ name, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }];
                      } else if (jointType === 'slide') {
                        newJoints = [{ name, type: 'slide', axis: [0, 0, 1], pos: [0, 0, 0], damping: 0.5 }];
                      } else if (jointType === 'ball') {
                        newJoints = [{ name, type: 'ball', pos: [0, 0, 0], damping: 0.5 }];
                      }
                    }
                    updateNodeJointsList(selectedNode.id, newJoints);
                  }}
                  className="w-full px-2 py-1.5 border border-slate-200 rounded text-sm bg-white font-medium text-slate-700 outline-none focus:border-blue-500 cursor-pointer"
                >
                  <option value="fixed">Fixed / Welded to Parent</option>
                  <option value="free">Free (6-DOF Movable)</option>
                  <option value="hinge">Hinge (Rotational Joint)</option>
                  <option value="slide">Slider (Prismatic Joint)</option>
                  <option value="ball">Ball Joint (Spherical)</option>
                </select>

                {selectedNode.joints?.length > 0 && (
                  <div className="mt-2 flex flex-col gap-1.5 p-2 bg-slate-50 rounded-lg border border-slate-150">
                    <div className="flex justify-between items-center">
                      <label className="text-[9px] font-semibold text-slate-400 uppercase tracking-wider">Joint Name (for API)</label>
                      <span className="font-mono text-[9px] text-blue-600 font-semibold bg-blue-50 px-1 py-0.5 rounded cursor-pointer select-all border border-blue-100" title="Joint API Reference. Click to select/copy.">
                        api.getJointPosition('{selectedNode.joints[0].name}')
                      </span>
                    </div>
                    <input 
                      type="text" 
                      value={selectedNode.joints[0].name || ''} 
                      onChange={(e) => {
                        const cleanName = e.target.value.replace(/[^a-zA-Z0-9_]/g, '_');
                        updateNodeJoint(selectedNode.id, { name: cleanName });
                      }}
                      className="w-full px-2 py-1 border border-slate-200 rounded text-xs font-mono bg-white text-slate-800 outline-none focus:border-blue-500 shadow-sm"
                      placeholder="e.g. cart_slide"
                    />
                  </div>
                )}
                
                {/* Free Joint Launch Velocity */}
                {selectedNode.joints?.length > 0 && selectedNode.joints[0].type === 'free' && (
                  <div className="flex flex-col gap-2 mt-2 pt-2 border-t border-slate-100">
                    <h3 className="text-xs font-semibold text-slate-600 mb-1">Launch Velocity (m/s)</h3>
                    {['X (Forward)', 'Y (Side)', 'Z (Up)'].map((label, i) => (
                      <div key={label} className="flex flex-col gap-1">
                        <label className="text-xs font-medium text-slate-500 flex justify-between">
                          {label} <span>{selectedNode.joints[0].initialVelocity?.[i] || 0}</span>
                        </label>
                        <input
                          type="range"
                          min="-20"
                          max="20"
                          step="0.5"
                          value={selectedNode.joints[0].initialVelocity?.[i] || 0}
                          onChange={(e) => {
                            const vel = [...(selectedNode.joints[0].initialVelocity || [0,0,0,0,0,0])];
                            vel[i] = parseFloat(e.target.value);
                            updateNodeJoint(selectedNode.id, { ...selectedNode.joints[0], initialVelocity: vel });
                          }}
                          className="w-full accent-blue-500 cursor-pointer"
                        />
                      </div>
                    ))}
                  </div>
                )}
              </div>

              {/* Motor Actuator Option for Hinge/Slide joints */}
              {selectedNode.joints?.length > 0 && (selectedNode.joints[0].type === 'hinge' || selectedNode.joints[0].type === 'slide') && (
                <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                  <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">⚡ Joint Actuator / Motor</h3>
                  <label className="text-xs font-semibold text-slate-500 flex items-center gap-2 cursor-pointer py-1">
                    <input 
                      type="checkbox" 
                      checked={!!selectedNode.joints[0].actuator}
                      onChange={(e) => {
                        const enabled = e.target.checked;
                        const updatedJoint = {
                          ...selectedNode.joints[0],
                          actuator: enabled ? { type: 'velocity', kv: 10, ctrlValue: 0 } : undefined
                        };
                        updateNodeJoint(selectedNode.id, updatedJoint);
                      }}
                      className="w-4 h-4 rounded text-blue-500 focus:ring-blue-400 accent-blue-500 cursor-pointer"
                    />
                    Enable Motor Drive
                  </label>

                  {selectedNode.joints[0].actuator && (
                    <div className="flex flex-col gap-2.5 mt-1 pt-2 border-t border-slate-100">
                      <div className="flex flex-col gap-1">
                        <label className="text-[10px] font-semibold text-slate-400 uppercase tracking-wider">Actuator Type</label>
                        <select
                          value={selectedNode.joints[0].actuator.type}
                          onChange={(e) => {
                            const type = e.target.value as 'velocity' | 'motor';
                            updateNodeJoint(selectedNode.id, {
                              ...selectedNode.joints[0],
                              actuator: { ...selectedNode.joints[0].actuator, type, kv: type === 'velocity' ? 10 : undefined }
                            });
                          }}
                          className="w-full px-2 py-1.5 border border-slate-200 rounded text-xs bg-white font-medium text-slate-700 outline-none cursor-pointer focus:border-blue-500"
                        >
                          <option value="velocity">Velocity Drive (Target Speed)</option>
                          <option value="motor">Torque Drive (Direct Force)</option>
                        </select>
                      </div>

                      {selectedNode.joints[0].actuator.type === 'velocity' && (
                        <div className="flex flex-col gap-1">
                          <label className="text-xs font-medium text-slate-500 flex justify-between">
                            Velocity Gain (kv) <span>{selectedNode.joints[0].actuator.kv || 10}</span>
                          </label>
                          <input
                            type="range"
                            min="0.5"
                            max="100"
                            step="0.5"
                            value={selectedNode.joints[0].actuator.kv || 10}
                            onChange={(e) => {
                              updateNodeJoint(selectedNode.id, {
                                ...selectedNode.joints[0],
                                actuator: { ...selectedNode.joints[0].actuator, kv: parseFloat(e.target.value) }
                              });
                            }}
                            className="w-full accent-blue-500 cursor-pointer"
                          />
                        </div>
                      )}
                    </div>
                  )}
                </div>
              )}

              {/* Position Coordinates (Applicable to all nodes!) */}
              <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2">Position Offset</h3>
                <div className="flex flex-col gap-3">
                  <div className="flex flex-col gap-1.5">
                    <label className="text-xs text-slate-500 flex items-center justify-between font-medium">X Position
                      <span>{selectedNode.pos[0].toFixed(2)} m</span>
                    </label>
                    <input 
                      type="range" 
                      min="-10" 
                      max="10" 
                      step="0.05" 
                      className="w-full accent-blue-500 cursor-pointer" 
                      value={selectedNode.pos[0]} 
                      onChange={(e) => handleMove(0, parseFloat(e.target.value))} 
                    />
                  </div>
                  <div className="flex flex-col gap-1.5">
                    <label className="text-xs text-slate-500 flex items-center justify-between font-medium">Y Position
                      <span>{selectedNode.pos[1].toFixed(2)} m</span>
                    </label>
                    <input 
                      type="range" 
                      min="-10" 
                      max="10" 
                      step="0.05" 
                      className="w-full accent-blue-500 cursor-pointer" 
                      value={selectedNode.pos[1]} 
                      onChange={(e) => handleMove(1, parseFloat(e.target.value))} 
                    />
                  </div>
                  <div className="flex flex-col gap-1.5">
                    <label className="text-xs text-slate-500 flex items-center justify-between font-medium">Z Position (Height)
                      <span>{selectedNode.pos[2].toFixed(2)} m</span>
                    </label>
                    <input 
                      type="range" 
                      min="0" 
                      max="10" 
                      step="0.05" 
                      className="w-full accent-blue-500 cursor-pointer" 
                      value={selectedNode.pos[2]} 
                      onChange={(e) => handleMove(2, parseFloat(e.target.value))} 
                    />
                  </div>
                  <div className="flex flex-col gap-1.5 mt-1 border-t border-slate-100 pt-2">
                    <label className="text-xs text-slate-500 flex items-center justify-between font-medium">X Rotation
                      <span>{(selectedNode.euler ? selectedNode.euler[0] : 0).toFixed(0)}°</span>
                    </label>
                    <input 
                      type="range" 
                      min="0" 
                      max="360" 
                      step="1" 
                      className="w-full accent-blue-500 cursor-pointer" 
                      value={selectedNode.euler ? selectedNode.euler[0] : 0} 
                      onChange={(e) => updateNodeRotation(selectedNode.id, 0, parseFloat(e.target.value))} 
                    />
                  </div>
                  <div className="flex flex-col gap-1.5 mt-1">
                    <label className="text-xs text-slate-500 flex items-center justify-between font-medium">Y Rotation
                      <span>{(selectedNode.euler ? selectedNode.euler[1] : 0).toFixed(0)}°</span>
                    </label>
                    <input 
                      type="range" 
                      min="0" 
                      max="360" 
                      step="1" 
                      className="w-full accent-blue-500 cursor-pointer" 
                      value={selectedNode.euler ? selectedNode.euler[1] : 0} 
                      onChange={(e) => updateNodeRotation(selectedNode.id, 1, parseFloat(e.target.value))} 
                    />
                  </div>
                  <div className="flex flex-col gap-1.5 mt-1">
                    <label className="text-xs text-slate-500 flex items-center justify-between font-medium">Z Rotation
                      <span>{(selectedNode.euler ? selectedNode.euler[2] : 0).toFixed(0)}°</span>
                    </label>
                    <input 
                      type="range" 
                      min="0" 
                      max="360" 
                      step="1" 
                      className="w-full accent-blue-500 cursor-pointer" 
                      value={selectedNode.euler ? selectedNode.euler[2] : 0} 
                      onChange={(e) => updateNodeRotation(selectedNode.id, 2, parseFloat(e.target.value))} 
                    />
                  </div>
                </div>
              </div>

              {/* Gear Config */}
              {selectedNode.id.includes('gear') && selectedNode.geoms && (() => {
                const pegGeom = selectedNode.geoms.find((g: any) => g.name.includes('peg'));
                const gearRadius = selectedNode.geoms[0].size[0];
                return (
                  <div className="flex flex-col gap-4">
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">⚙️ Gear Properties</h3>
                      <label className="text-xs font-medium text-slate-500 flex justify-between">
                        Teeth Count <span>{selectedNode.geoms.length - (pegGeom ? 2 : 1)}</span>
                      </label>
                      <input 
                        type="range" 
                        min="4" 
                        max="24" 
                        step="1" 
                        value={selectedNode.geoms.length - (pegGeom ? 2 : 1)} 
                        onChange={(e) => {
                          const teethVal = parseInt(e.target.value);
                          updateGearTeeth(selectedNode.id, teethVal);
                        }} 
                        className="w-full accent-blue-500 cursor-pointer" 
                      />
                      <label className="text-xs font-medium text-slate-500 flex justify-between mt-2">
                        Gear Radius <span>{gearRadius.toFixed(2)} m</span>
                      </label>
                      <input 
                        type="range" 
                        min="0.05" 
                        max="5.0" 
                        step="0.01" 
                        value={gearRadius} 
                        onChange={(e) => {
                          const r = parseFloat(e.target.value);
                          updateNodeGeom(selectedNode.id, { size: [r, selectedNode.geoms[0].size[1]] });
                        }} 
                        className="w-full accent-blue-500 cursor-pointer" 
                      />
                    </div>

                    {pegGeom ? (
                      <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                        <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">📍 Pusher Peg Properties</h3>
                        <label className="text-xs font-medium text-slate-500 flex justify-between">
                          Peg Offset (Radius) <span>{pegGeom.pos[0].toFixed(2)} m</span>
                        </label>
                        <input 
                          type="range" 
                          min="0.01" 
                          max="5.0" 
                          step="0.01" 
                          value={pegGeom.pos[0]} 
                          onChange={(e) => {
                            const offsetVal = parseFloat(e.target.value);
                            updatePusherPeg(selectedNode.id, { offset: offsetVal });
                          }} 
                          className="w-full accent-blue-500 cursor-pointer" 
                        />
                        <label className="text-xs font-medium text-slate-500 flex justify-between mt-2">
                          Peg Thickness <span>{pegGeom.size[0].toFixed(3)} m</span>
                        </label>
                        <input 
                          type="range" 
                          min="0.005" 
                          max="0.5" 
                          step="0.005" 
                          value={pegGeom.size[0]} 
                          onChange={(e) => {
                            const rVal = parseFloat(e.target.value);
                            updatePusherPeg(selectedNode.id, { size: [rVal, pegGeom.size[1]] });
                          }} 
                          className="w-full accent-blue-500 cursor-pointer" 
                        />
                        <label className="text-xs font-medium text-slate-500 flex justify-between mt-2">
                          Peg Length <span>{pegGeom.size[1].toFixed(2)} m</span>
                        </label>
                        <input 
                          type="range" 
                          min="0.01" 
                          max="1.0" 
                          step="0.01" 
                          value={pegGeom.size[1]} 
                          onChange={(e) => {
                            const hVal = parseFloat(e.target.value);
                            updatePusherPeg(selectedNode.id, { size: [pegGeom.size[0], hVal] });
                          }} 
                          className="w-full accent-blue-500 cursor-pointer" 
                        />
                        <button
                          onClick={() => deletePusherPeg(selectedNode.id)}
                          className="mt-2 w-full py-1.5 px-3 bg-red-50 text-red-600 hover:bg-red-100 rounded-md text-xs font-medium transition duration-150 shadow-sm border border-red-100"
                        >
                          🗑️ Remove Pusher Peg
                        </button>
                      </div>
                    ) : (
                      <button
                        onClick={() => addPusherPeg(selectedNode.id)}
                        className="py-2 px-3 bg-blue-50 hover:bg-blue-100 text-blue-600 rounded-lg text-xs font-semibold transition duration-150 border border-blue-100 flex items-center justify-center gap-1.5 shadow-sm"
                      >
                        ➕ Add Pusher Peg
                      </button>
                    )}
                  </div>
                );
              })()}

              {/* Damping, Limits, and Actuator Target Speed properties */}
              {selectedNode.joints?.map((joint: any, i: number) => (
                <div key={`joint-${i}`} className="flex flex-col gap-4">
                  {joint.damping !== undefined && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center gap-1.5">
                        <span>🔗 Joint Damping</span>
                      </h3>
                      <label className="text-xs font-medium text-slate-500 flex justify-between">Damping <span>{joint.damping}</span></label>
                      <input type="range" min="0" max="500" step="0.1" value={joint.damping} onChange={(e) => updateNodeJoint(selectedNode.id, {damping: parseFloat(e.target.value)})} className="w-full accent-blue-500 cursor-pointer" />
                    </div>
                  )}

                  {(joint.type === 'hinge' || joint.type === 'slide' || joint.type === 'ball') && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center gap-1.5">
                        <span>🌸 Joint Springs</span>
                      </h3>
                      
                      <div className="flex flex-col gap-1">
                        <label className="text-xs font-medium text-slate-500 flex justify-between">
                          Spring Stiffness (K) <span>{(joint.stiffness || 0).toFixed(0)} N/m</span>
                        </label>
                        <input 
                          type="range" 
                          min="0" 
                          max="5000" 
                          step="10" 
                          value={joint.stiffness || 0} 
                          onChange={(e) => updateNodeJoint(selectedNode.id, { stiffness: parseFloat(e.target.value) })}
                          className="w-full accent-blue-500 cursor-pointer" 
                        />
                      </div>

                      {(joint.stiffness || 0) > 0 && (joint.type === 'hinge' || joint.type === 'slide') && (
                        <div className="flex flex-col gap-1 mt-1 border-t border-slate-50 pt-2">
                          <label className="text-xs font-medium text-slate-500 flex justify-between">
                            Spring Rest Position <span>{(joint.springref || 0).toFixed(joint.type === 'slide' ? 2 : 0)}{joint.type === 'slide' ? ' m' : '°'}</span>
                          </label>
                          <input 
                            type="range" 
                            min={joint.type === 'slide' ? -20.0 : -360} 
                            max={joint.type === 'slide' ? 20.0 : 360} 
                            step={joint.type === 'slide' ? 0.05 : 1} 
                            value={joint.springref || 0} 
                            onChange={(e) => updateNodeJoint(selectedNode.id, { springref: parseFloat(e.target.value) })}
                            className="w-full accent-blue-500 cursor-pointer" 
                          />
                        </div>
                      )}
                    </div>
                  )}

                  {(joint.type === 'hinge' || joint.type === 'slide') && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center gap-1.5">
                        <span>🔒 Joint Limits</span>
                      </h3>
                      
                      <label className="text-xs font-semibold text-slate-500 flex items-center gap-2 cursor-pointer py-1">
                        <input 
                          type="checkbox" 
                          checked={joint.limited === true || joint.limited === 'true'}
                          onChange={(e) => {
                            const enabled = e.target.checked;
                            const defaultRange = joint.type === 'slide' ? [-1.0, 1.0] : [-90, 90];
                            updateNodeJoint(selectedNode.id, { 
                              limited: enabled,
                              range: enabled ? (joint.range || defaultRange) : undefined
                            });
                          }}
                          className="w-4 h-4 rounded text-blue-500 focus:ring-blue-400 accent-blue-500 cursor-pointer"
                        />
                        Enable Range Limits
                      </label>

                      {(joint.limited === true || joint.limited === 'true') && (() => {
                        const range = joint.range || (joint.type === 'slide' ? [-1.0, 1.0] : [-90, 90]);
                        const isSlide = joint.type === 'slide';
                        const minVal = range[0];
                        const maxVal = range[1];
                        
                        return (
                          <div className="flex flex-col gap-3 mt-1 border-t border-slate-50 pt-2">
                            <div className="flex flex-col gap-1">
                              <label className="text-xs font-medium text-slate-500 flex justify-between">
                                Minimum Limit <span>{minVal.toFixed(isSlide ? 2 : 0)}{isSlide ? ' m' : '°'}</span>
                              </label>
                              <input 
                                type="range" 
                                min={isSlide ? -20.0 : -360}
                                max={isSlide ? 20.0 : 360}
                                step={isSlide ? 0.05 : 1}
                                value={minVal}
                                onChange={(e) => {
                                  const val = parseFloat(e.target.value);
                                  const newMin = Math.min(val, maxVal);
                                  updateNodeJoint(selectedNode.id, { range: [newMin, maxVal] });
                                }}
                                className="w-full accent-blue-500 cursor-pointer" 
                              />
                            </div>

                            <div className="flex flex-col gap-1">
                              <label className="text-xs font-medium text-slate-500 flex justify-between">
                                Maximum Limit <span>{maxVal.toFixed(isSlide ? 2 : 0)}{isSlide ? ' m' : '°'}</span>
                              </label>
                              <input 
                                type="range" 
                                min={isSlide ? -20.0 : -360}
                                max={isSlide ? 20.0 : 360}
                                step={isSlide ? 0.05 : 1}
                                value={maxVal}
                                onChange={(e) => {
                                  const val = parseFloat(e.target.value);
                                  const newMax = Math.max(val, minVal);
                                  updateNodeJoint(selectedNode.id, { range: [minVal, newMax] });
                                }}
                                className="w-full accent-blue-500 cursor-pointer" 
                              />
                            </div>
                          </div>
                        );
                      })()}
                    </div>
                  )}

                  {joint.actuator && (() => {
                    const isTorque = joint.actuator.type === 'motor';
                    
                    return (
                      <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                        <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">
                          {isTorque ? '💪 Target Torque/Force' : '⚡ Target Velocity'}
                        </h3>
                        <label className="text-xs font-medium text-slate-500 flex justify-between">
                          {isTorque ? 'Control Force' : 'Control Speed'}
                          <span>{joint.actuator.ctrlValue || 0}</span>
                        </label>
                        <input 
                          type="range" 
                          min={isTorque ? "-1000" : "-100"} 
                          max={isTorque ? "1000" : "100"} 
                          step={isTorque ? "1" : "0.1"} 
                          value={joint.actuator.ctrlValue || 0} 
                          onChange={(e) => {
                            const val = parseFloat(e.target.value);
                            if (isPlaying && model && data && mujoco) {
                              const actId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR.value, `${joint.name}_actuator`);
                              if (actId !== -1) data.ctrl[actId] = val;
                            }
                            updateNodeJoint(selectedNode.id, { actuator: { ...joint.actuator, ctrlValue: val } });
                          }} 
                          className="w-full accent-blue-500 cursor-pointer" 
                        />
                      </div>
                    );
                  })()}
                </div>
              ))}

              {/* Dimensions Resizing and Color Properties */}
              {(() => {
                const geom = selectedNode.geoms?.find((g: any) => g.type === 'sphere' || g.type === 'box' || g.type === 'cylinder') || selectedNode.geoms?.[0];
                if (!geom) return null;
                const hasMeshGeom = selectedNode.geoms?.some((g: any) => g.type === 'mesh');
                return (
                  <div key="geom-properties" className="flex flex-col gap-4">
                  {!selectedNode.id.includes('gear') && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2">📏 Resize Component</h3>

                      {hasMeshGeom && (
                        <div className="flex flex-col gap-2">
                          <label className="text-xs font-medium text-slate-500 flex justify-between">
                            Uniform Scale <span>scales all sub-geoms together</span>
                          </label>
                          <input
                            type="range"
                            min="0.1"
                            max="3.0"
                            step="0.05"
                            defaultValue="1.0"
                            onMouseUp={(e) => {
                              const scale = parseFloat((e.target as HTMLInputElement).value);
                              (e.target as HTMLInputElement).value = '1.0';
                              const newScene = JSON.parse(JSON.stringify(useStore.getState().sceneGraph));
                              const find = (nodes: any[]): any => {
                                for (const n of nodes) {
                                  if (n.id === selectedNode.id) return n;
                                  const c = find(n.children);
                                  if (c) return c;
                                }
                                return null;
                              };
                              const node = find(newScene.nodes);
                              if (!node) return;
                              // Scale this node and all children recursively
                              const scaleNode = (n: any) => {
                                scaleMeshGeoms(n, scale);
                                for (const g of n.geoms) {
                                  if (g.type === 'mesh') continue;
                                  if (g.size) g.size = g.size.map((s: number) => s * scale);
                                  if (g.pos) g.pos = g.pos.map((p: number) => p * scale);
                                  if (g.fromto) g.fromto = g.fromto.map((f: number) => f * scale);
                                }
                                // Scale child body pos offsets too
                                for (const child of (n.children || [])) {
                                  if (child.pos) child.pos = child.pos.map((p: number) => p * scale);
                                  scaleNode(child);
                                }
                              };
                              scaleNode(node);
                              useStore.getState().updateScene(newScene);
                            }}
                            className="w-full accent-violet-500 cursor-pointer"
                          />
                          <p className="text-[10px] text-slate-400">Slider resets to 1× after release — each drag applies multiplicative scale to all sub-geoms.</p>
                        </div>
                      )}
                      
                      {geom.type === 'sphere' && (
                        <div className="flex flex-col gap-2">
                          <label className="text-xs font-medium text-slate-500 flex justify-between">Radius <span>{geom.size[0].toFixed(2)} m</span></label>
                          <input 
                            type="range" 
                            min="0.05" 
                            max="2.0" 
                            step="0.01" 
                            value={geom.size[0]} 
                            onChange={(e) => {
                              const r = parseFloat(e.target.value);
                              updateNodeGeom(selectedNode.id, { size: [r] });
                            }}
                            className="w-full accent-blue-500 cursor-pointer" 
                          />
                        </div>
                      )}

                      {geom.type === 'box' && selectedNode.isWedge && (
                        <div className="flex flex-col gap-3">
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Base Width (X) <span>{(selectedNode.width || 2.0).toFixed(2)} m</span></label>
                            <input 
                              type="range" 
                              min="0.5" 
                              max="5.0" 
                              step="0.05" 
                              value={selectedNode.width || 2.0} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateWedgeParams(selectedNode.id, { width: val });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Depth (Y) <span>{(selectedNode.depth || 1.0).toFixed(2)} m</span></label>
                            <input 
                              type="range" 
                              min="0.2" 
                              max="4.0" 
                              step="0.05" 
                              value={selectedNode.depth || 1.0} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateWedgeParams(selectedNode.id, { depth: val });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Height (Z) <span>{(selectedNode.height || 0.5).toFixed(2)} m</span></label>
                            <input 
                              type="range" 
                              min="0.1" 
                              max="3.0" 
                              step="0.05" 
                              value={selectedNode.height || 0.5} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateWedgeParams(selectedNode.id, { height: val });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                          <div className="flex flex-col gap-1 border-t border-slate-100 pt-2">
                            <label className="text-xs font-medium text-slate-600 flex justify-between">Wedge Angle <span>{(selectedNode.wedgeAngle !== undefined ? selectedNode.wedgeAngle : 14.036).toFixed(1)}°</span></label>
                            <input 
                              type="range" 
                              min="2" 
                              max="85" 
                              step="1" 
                              value={selectedNode.wedgeAngle !== undefined ? selectedNode.wedgeAngle : 14.036} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateWedgeParams(selectedNode.id, { wedgeAngle: val });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                        </div>
                      )}

                      {geom.type === 'box' && !selectedNode.isWedge && (
                        <div className="flex flex-col gap-3">
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Width (X) <span>{geom.size[0].toFixed(2)} m</span></label>
                            <input 
                              type="range" 
                              min="0.05" 
                              max="2.0" 
                              step="0.01" 
                              value={geom.size[0]} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateNodeGeom(selectedNode.id, { size: [val, geom.size[1], geom.size[2]] });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Depth (Y) <span>{geom.size[1].toFixed(2)} m</span></label>
                            <input 
                              type="range" 
                              min="0.05" 
                              max="2.0" 
                              step="0.01" 
                              value={geom.size[1]} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateNodeGeom(selectedNode.id, { size: [geom.size[0], val, geom.size[2]] });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Height (Z) <span>{geom.size[2].toFixed(2)} m</span></label>
                            <input 
                              type="range" 
                              min="0.05" 
                              max="2.0" 
                              step="0.01" 
                              value={geom.size[2]} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateNodeGeom(selectedNode.id, { size: [geom.size[0], geom.size[1], val] });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                        </div>
                      )}

                      {(geom.type === 'capsule' || geom.type === 'cylinder') && !selectedNode.isPulleyWheel && (
                        <div className="flex flex-col gap-3">
                          <div className="flex flex-col gap-1">
                            <label className="text-xs font-medium text-slate-500 flex justify-between">Radius <span>{geom.size[0].toFixed(3)} m</span></label>
                            <input 
                              type="range" 
                              min="0.01" 
                              max="0.8" 
                              step="0.005" 
                              value={geom.size[0]} 
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateNodeGeom(selectedNode.id, { 
                                  size: geom.size[1] !== undefined ? [val, geom.size[1]] : [val] 
                                });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
                          {geom.size[1] !== undefined && (
                            <div className="flex flex-col gap-1">
                              <label className="text-xs font-medium text-slate-500 flex justify-between">Length (Half-Height) <span>{geom.size[1].toFixed(2)} m</span></label>
                              <input 
                                type="range" 
                                min="0.05" 
                                max="3.0" 
                                step="0.01" 
                                value={geom.size[1]} 
                                onChange={(e) => {
                                  const val = parseFloat(e.target.value);
                                  updateNodeGeom(selectedNode.id, { size: [geom.size[0], val] });
                                }}
                                className="w-full accent-blue-500 cursor-pointer" 
                              />
                            </div>
                          )}
                          {geom.fromto !== undefined && (() => {
                            const dirX = geom.fromto[3] - geom.fromto[0];
                            const dirY = geom.fromto[4] - geom.fromto[1];
                            const dirZ = geom.fromto[5] - geom.fromto[2];
                            const currentLength = Math.sqrt(dirX*dirX + dirY*dirY + dirZ*dirZ) || 1.0;
                            
                            return (
                              <div className="flex flex-col gap-1">
                                <label className="text-xs font-medium text-slate-500 flex justify-between">
                                  Length (Segment) <span>{currentLength.toFixed(2)} m</span>
                                </label>
                                <input 
                                  type="range" 
                                  min="0.1" 
                                  max="5.0" 
                                  step="0.05" 
                                  value={currentLength} 
                                  onChange={(e) => {
                                    const newVal = parseFloat(e.target.value);
                                    const scale = newVal / currentLength;
                                    const newFromto = [
                                      geom.fromto[0],
                                      geom.fromto[1],
                                      geom.fromto[2],
                                      geom.fromto[0] + dirX * scale,
                                      geom.fromto[1] + dirY * scale,
                                      geom.fromto[2] + dirZ * scale
                                    ];
                                    updateNodeGeom(selectedNode.id, { fromto: newFromto });
                                  }}
                                  className="w-full accent-blue-500 cursor-pointer" 
                                />
                              </div>
                            );
                          })()}
                        </div>
                      )}
                    </div>
                  )}

                  <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                    <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">Mass</h3>
                    <label className="text-xs font-medium text-slate-500 flex justify-between">Value <span>{geom.mass} kg</span></label>
                    <input type="range" min="0" max="50" step="0.01" value={geom.mass} onChange={(e) => updateNodeGeom(selectedNode.id, {mass: parseFloat(e.target.value)})} className="w-full accent-blue-500 cursor-pointer" />
                  </div>

                   <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                    <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center justify-between">
                      <span className="flex items-center gap-1">💥 Collision Physics</span>
                      <button 
                        onClick={() => {
                          setDocsTab('collision');
                          setIsDocsOpen(true);
                        }} 
                        className="text-slate-400 hover:text-slate-600 transition-colors cursor-pointer" 
                        title="Click for documentation"
                      >
                        <Info className="w-3.5 h-3.5" />
                      </button>
                    </h3>
                    <label className="text-xs font-semibold text-slate-500 flex items-center gap-2 cursor-pointer py-1">
                      <input 
                        type="checkbox" 
                        checked={geom.contype !== 0 && geom.conaffinity !== 0}
                        onChange={(e) => {
                          const enabled = e.target.checked;
                          updateNodeGeom(selectedNode.id, {
                            contype: enabled ? 1 : 0,
                            conaffinity: enabled ? 1 : 0
                          });
                        }}
                        className="w-4 h-4 rounded text-blue-500 focus:ring-blue-400 accent-blue-500 cursor-pointer"
                      />
                      Enable Collisions
                    </label>
                  </div>

                  {/* Material Properties Card */}
                  <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                    <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center justify-between">
                      <span className="flex items-center gap-1">🧪 Physical Material</span>
                      <button 
                        onClick={() => {
                          setDocsTab('friction');
                          setIsDocsOpen(true);
                        }} 
                        className="text-slate-400 hover:text-slate-600 transition-colors cursor-pointer" 
                        title="Click for documentation"
                      >
                        <Info className="w-3.5 h-3.5" />
                      </button>
                    </h3>

                    {/* Bounciness (Restitution) Slider */}
                    {(() => {
                      const currentBounce = (() => {
                        if (!geom.solref) return 0.0;
                        if (geom.solref[0] >= 0) return 0.0;
                        const damping = -geom.solref[1];
                        if (damping >= 50) return 0.0;
                        const b = Math.exp(-damping / 20);
                        return Math.min(0.99, Math.max(0.0, b));
                      })();
                      return (
                        <div className="flex flex-col gap-1.5">
                          <label className="text-xs font-semibold text-slate-500 flex justify-between">
                            Bounciness (Bounce)
                            <span className="text-blue-600 font-bold">{currentBounce.toFixed(2)}</span>
                          </label>
                          <input 
                            type="range" 
                            min="0.0" 
                            max="0.99" 
                            step="0.01" 
                            className="w-full accent-blue-500 cursor-pointer" 
                            value={currentBounce} 
                            onChange={(e) => {
                              const bounce = parseFloat(e.target.value);
                              if (bounce > 0) {
                                const damping = -20 * Math.log(bounce);
                                const stiffness = bounce > 0.95 ? -1000 - (bounce - 0.95) * 475000 : -1000;
                                updateNodeGeom(selectedNode.id, {
                                  solref: [stiffness, -damping],
                                  solimp: [0.99, 0.99, 0.001, 0.5, 2]
                                });
                              } else {
                                // Omit or reset solref/solimp
                                const updatedGeom = { ...geom };
                                delete updatedGeom.solref;
                                delete updatedGeom.solimp;
                                updateNodeGeom(selectedNode.id, updatedGeom);
                              }
                            }}
                          />
                          <span className="text-[10px] text-slate-400 leading-tight">
                            Controls elastic bounce restitution (elastic limit is 0.99).
                          </span>
                        </div>
                      );
                    })()}

                    {/* Friction Slider */}
                    {(() => {
                      const currentFriction = geom.friction ? geom.friction[0] : 0.7;
                      return (
                        <div className="flex flex-col gap-1.5 mt-1 border-t border-slate-100 pt-2">
                          <label className="text-xs font-semibold text-slate-500 flex justify-between">
                            Sliding Friction
                            <span className="text-blue-600 font-bold">{currentFriction.toFixed(2)}</span>
                          </label>
                          <input 
                            type="range" 
                            min="0.0" 
                            max="1.5" 
                            step="0.05" 
                            className="w-full accent-blue-500 cursor-pointer" 
                            value={currentFriction} 
                            onChange={(e) => {
                              const f = parseFloat(e.target.value);
                              updateNodeGeom(selectedNode.id, {
                                friction: [f, 0.005, 0.0001]
                              });
                            }}
                          />
                          <span className="text-[10px] text-slate-400 leading-tight">
                            Low friction makes it slippery like ice. High friction creates rubbery resistance.
                          </span>
                        </div>
                      );
                    })()}
                  </div>

                  {/* Joint Mechanical Coupling Configuration */}
                  {selectedNode.joints && selectedNode.joints.length > 0 && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2.5">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center justify-between">
                        <span className="flex items-center gap-1">⚙️ Mechanical Coupling</span>
                        <button 
                          onClick={() => {
                            setDocsTab('coupling');
                            setIsDocsOpen(true);
                          }} 
                          className="text-slate-400 hover:text-slate-600 transition-colors cursor-pointer" 
                          title="Click for documentation"
                        >
                          <Info className="w-3.5 h-3.5" />
                        </button>
                      </h3>

                      <label className="text-xs font-semibold text-slate-500 flex items-center gap-2 cursor-pointer py-1">
                        <input 
                          type="checkbox" 
                          checked={selectedNode.allowCoupling !== false}
                          onChange={(e) => {
                            const enabled = e.target.checked;
                            const newScene = JSON.parse(JSON.stringify(sceneGraph));
                            const traverse = (nodes: any[]) => {
                              if (!nodes) return false;
                              for (const node of nodes) {
                                if (node.id === selectedNode.id) {
                                  node.allowCoupling = enabled;
                                  return true;
                                }
                                if (traverse(node.children)) return true;
                              }
                              return false;
                            };
                            traverse(newScene.nodes);
                            updateScene(newScene);
                          }}
                          className="w-4 h-4 rounded text-blue-500 focus:ring-blue-400 accent-blue-500 cursor-pointer"
                        />
                        Enable Coupling
                      </label>

                      {selectedNode.allowCoupling !== false && (() => {
                        // Gather list of other jointed nodes in the scene
                        const list: SceneNode[] = [];
                        const traverse = (items: SceneNode[]) => {
                          for (const item of items) {
                            if (item.id !== selectedNode.id && item.joints && item.joints.length > 0) {
                              list.push(item);
                            }
                            if (item.children) {
                              traverse(item.children);
                            }
                          }
                        };
                        traverse(sceneGraph.nodes);

                        return (
                          <div className="flex flex-col gap-2 mt-1 border-t border-slate-100 pt-2">
                            <div className="flex flex-col gap-1">
                              <span className="text-[10px] font-semibold text-slate-400 uppercase tracking-wider">Couple Target Component</span>
                              <select
                                value={selectedNode.coupleTargetId || ''}
                                onChange={(e) => {
                                  const val = e.target.value || undefined;
                                  const newScene = JSON.parse(JSON.stringify(sceneGraph));
                                  const traverse2 = (nodes: any[]) => {
                                    if (!nodes) return false;
                                    for (const node of nodes) {
                                      if (node.id === selectedNode.id) {
                                        node.coupleTargetId = val;
                                        // Default ratio depending on type if target selected and no custom ratio set
                                        if (val && node.coupleRatio === undefined) {
                                          node.coupleRatio = val.includes('rack') || selectedNode.id.includes('rack') ? 0.2 : -1.0;
                                        }
                                        return true;
                                      }
                                      if (traverse2(node.children)) return true;
                                    }
                                    return false;
                                  };
                                  traverse2(newScene.nodes);
                                  updateScene(newScene);
                                }}
                                className="w-full px-2 py-1.5 border border-slate-200 rounded text-xs bg-white text-slate-700 outline-none focus:border-blue-500 cursor-pointer"
                              >
                                <option value="">[Auto Proximity Fallback]</option>
                                {list.map(node => (
                                  <option key={node.id} value={node.id}>
                                    {node.name || node.id} ({node.joints[0].type})
                                  </option>
                                ))}
                              </select>
                            </div>

                            {selectedNode.coupleTargetId && (
                              <div className="flex flex-col gap-1">
                                <span className="text-[10px] font-semibold text-slate-400 uppercase tracking-wider">Coupling Ratio</span>
                                <div className="flex gap-1.5 items-center">
                                  <select
                                    value={
                                      selectedNode.coupleRatio === -1.0 ? 'gear' :
                                      selectedNode.coupleRatio === 0.2 ? 'pinion_rack' :
                                      selectedNode.coupleRatio === 1.0 ? 'direct' :
                                      'custom'
                                    }
                                    onChange={(e) => {
                                      const type = e.target.value;
                                      let ratio = -1.0;
                                      if (type === 'gear') ratio = -1.0;
                                      else if (type === 'pinion_rack') ratio = 0.2;
                                      else if (type === 'direct') ratio = 1.0;
                                      else ratio = selectedNode.coupleRatio !== undefined ? selectedNode.coupleRatio : -1.0;

                                      const newScene = JSON.parse(JSON.stringify(sceneGraph));
                                      const traverse2 = (nodes: any[]) => {
                                        if (!nodes) return false;
                                        for (const node of nodes) {
                                          if (node.id === selectedNode.id) {
                                            node.coupleRatio = ratio;
                                            return true;
                                          }
                                          if (traverse2(node.children)) return true;
                                        }
                                        return false;
                                      };
                                      traverse2(newScene.nodes);
                                      updateScene(newScene);
                                    }}
                                    className="px-2 py-1.5 border border-slate-200 rounded text-xs bg-white text-slate-700 outline-none focus:border-blue-500 cursor-pointer flex-1"
                                  >
                                    <option value="gear">Gears meshing (-1.0)</option>
                                    <option value="pinion_rack">Rack & Pinion (0.2)</option>
                                    <option value="direct">Direct link (1.0)</option>
                                    <option value="custom">Custom Ratio...</option>
                                  </select>

                                  <input
                                    type="number"
                                    step="0.05"
                                    value={selectedNode.coupleRatio !== undefined ? selectedNode.coupleRatio : -1.0}
                                    onChange={(e) => {
                                      const val = parseFloat(e.target.value);
                                      if (isNaN(val)) return;
                                      const newScene = JSON.parse(JSON.stringify(sceneGraph));
                                      const traverse2 = (nodes: any[]) => {
                                        if (!nodes) return false;
                                        for (const node of nodes) {
                                          if (node.id === selectedNode.id) {
                                            node.coupleRatio = val;
                                            return true;
                                          }
                                          if (traverse2(node.children)) return true;
                                        }
                                        return false;
                                      };
                                      traverse2(newScene.nodes);
                                      updateScene(newScene);
                                    }}
                                    className="w-16 px-1.5 py-1.5 border border-slate-200 rounded text-xs text-center font-mono outline-none focus:border-blue-500"
                                    title="Custom gear coupling ratio"
                                  />
                                </div>
                              </div>
                            )}
                          </div>
                        );
                      })()}
                    </div>
                  )}
                  <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                    <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">Appearance</h3>
                    <div className="flex items-center justify-between">
                      <span className="text-xs text-slate-500">Color (RGB)</span>
                      <input type="color" value={`#${Math.floor(geom.rgba[0]*255).toString(16).padStart(2,'0')}${Math.floor(geom.rgba[1]*255).toString(16).padStart(2,'0')}${Math.floor(geom.rgba[2]*255).toString(16).padStart(2,'0')}`} 
                        onChange={(e) => {
                          const hex = e.target.value;
                          const r = parseInt(hex.slice(1,3), 16)/255;
                          const g = parseInt(hex.slice(3,5), 16)/255;
                          const b = parseInt(hex.slice(5,7), 16)/255;
                          updateNodeGeom(selectedNode.id, {rgba: [r,g,b,1]});
                        }} 
                        className="w-8 h-8 rounded cursor-pointer border-0 p-0 shadow-sm" 
                      />
                    </div>
                  </div>
                </div>
              );
            })()}

              {/* Mesh Properties — shown when the body or any child has a mesh geom */}
              {(() => {
                const allGeoms: any[] = [];
                const collectGeoms = (node: any) => { node.geoms?.forEach((g: any) => allGeoms.push({...g, _fromChildId: node.id !== selectedNode.id ? node.id : null})); node.children?.forEach(collectGeoms); };
                collectGeoms(selectedNode);
                if (!allGeoms.some((g: any) => g.type === 'mesh')) return null;
                return (
                <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                  <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center gap-1.5">
                    <Shapes className="w-3.5 h-3.5 text-violet-500" /> Body Geoms ({allGeoms.length})
                  </h3>
                  <p className="text-[10px] text-slate-400 -mt-1 leading-snug">
                    Static mesh geoms are <strong>visual only</strong>. Primitive geoms handle physics. Dynamic meshes simulate and collide.
                  </p>
                  {allGeoms.map((g: any) => (
                    <div key={g.name} className="flex flex-col gap-1.5 p-2 bg-slate-50 rounded border border-slate-100">
                      <div className="flex items-center justify-between">
                        <span className="text-[10px] font-semibold text-slate-500 uppercase tracking-wider">
                          {g.name}{g._fromChildId ? <span className="text-violet-400 font-normal"> (child)</span> : null}
                        </span>
                        <span className="text-[10px] text-slate-400">
                          {g.type === 'mesh'
                            ? (g.vertices ? `mesh · ${g.vertices.length / 3} verts · ${g.faces ? g.faces.length / 3 : 0} tris${g.dynamic ? ' · dynamic' : ' · static'}` : 'mesh · no geometry')
                            : `${g.type} · size [${(g.size || []).map((s: number) => s.toFixed(2)).join(', ')}]`}
                        </span>
                      </div>
                      {g.type === 'mesh' && g.vertices && g.vertices.length > 0 && (
                        <button
                          onClick={() => {
                            const unique = new Map<string, number>();
                            const newVerts: number[] = [];
                            const remap: number[] = [];
                            for (let i = 0; i < g.vertices.length; i += 3) {
                              const key = `${g.vertices[i].toFixed(4)},${g.vertices[i+1].toFixed(4)},${g.vertices[i+2].toFixed(4)}`;
                              if (!unique.has(key)) { unique.set(key, newVerts.length / 3); newVerts.push(g.vertices[i], g.vertices[i+1], g.vertices[i+2]); }
                              remap[i/3] = unique.get(key)!;
                            }
                            const filteredFaces: number[] = [];
                            for (let i = 0; i < g.faces.length; i += 3) {
                              const a = remap[g.faces[i]], b = remap[g.faces[i+1]], c = remap[g.faces[i+2]];
                              if (a !== b && b !== c && a !== c) filteredFaces.push(a, b, c);
                            }
                            const newScene = JSON.parse(JSON.stringify(useStore.getState().sceneGraph));
                            const traverse = (nodes: any[]): boolean => {
                              for (const node of nodes) {
                                const idx = node.geoms?.findIndex((ng: any) => ng.name === g.name);
                                if (idx >= 0) { node.geoms[idx] = { ...node.geoms[idx], vertices: newVerts, faces: filteredFaces }; return true; }
                                if (traverse(node.children)) return true;
                              }
                              return false;
                            };
                            traverse(newScene.nodes);
                            useStore.getState().updateScene(newScene);
                          }}
                          className="flex items-center justify-center gap-1.5 px-2 py-1.5 bg-violet-50 hover:bg-violet-100 border border-violet-200 rounded text-[10px] font-semibold text-violet-700 transition-colors cursor-pointer"
                        >
                          <Minimize2 className="w-3 h-3" /> Simplify (deduplicate vertices)
                        </button>
                      )}
                    </div>
                  ))}
                </div>
                );
              })()}

              {selectedNode.isPulleyWheel && (
                <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                  <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">🛞 Pulley Properties</h3>
                  <label className="text-xs font-medium text-slate-500 flex justify-between">
                    Pulley Radius <span>{(selectedNode.pulleyRadius || 0.4).toFixed(2)} m</span>
                  </label>
                  <input 
                    type="range" 
                    min="0.15" 
                    max="1.5" 
                    step="0.01" 
                    value={selectedNode.pulleyRadius || 0.4} 
                    onChange={(e) => {
                      const radVal = parseFloat(e.target.value);
                      updatePulleyParams(selectedNode.id, { pulleyRadius: radVal });
                    }} 
                    className="w-full accent-blue-500 cursor-pointer" 
                  />
                </div>
              )}

              {selectedNode.isPulleyRope && (
                <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                  <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center gap-1.5">
                    <span>🧵 Rope Properties</span>
                  </h3>

                  <p className="text-[10px] text-slate-400 leading-snug -mt-1">
                    Connect two bodies directly, or optionally route through a Pulley Wheel for an Atwood-style coupling.
                  </p>
                  
                  <div className="flex flex-col gap-1">
                    <label className="text-xs font-semibold text-slate-500 flex justify-between">
                      Body A <span className="font-normal text-rose-400">required</span>
                    </label>
                    <select
                      value={selectedNode.leftTargetId || ''}
                      onChange={(e) => updateRopeParams(selectedNode.id, { leftTargetId: e.target.value })}
                      className="w-full text-xs border border-slate-200 rounded p-1.5 bg-slate-50 font-medium text-slate-700 focus:border-blue-500 outline-none"
                    >
                      <option value="">-- Select Body A --</option>
                      {allJointedNodes.map(n => (
                        <option key={n.id} value={n.id}>{n.id}</option>
                      ))}
                    </select>
                  </div>

                  <div className="flex flex-col gap-1">
                    <label className="text-xs font-semibold text-slate-500 flex justify-between">
                      Body B <span className="font-normal text-rose-400">required</span>
                    </label>
                    <select
                      value={selectedNode.rightTargetId || ''}
                      onChange={(e) => updateRopeParams(selectedNode.id, { rightTargetId: e.target.value })}
                      className="w-full text-xs border border-slate-200 rounded p-1.5 bg-slate-50 font-medium text-slate-700 focus:border-blue-500 outline-none"
                    >
                      <option value="">-- Select Body B --</option>
                      {allJointedNodes.map(n => (
                        <option key={n.id} value={n.id}>{n.id}</option>
                      ))}
                    </select>
                  </div>

                  <div className="flex flex-col gap-1">
                    <label className="text-xs font-semibold text-slate-500 flex justify-between">
                      Pulley Wheel <span className="font-normal text-slate-400">optional</span>
                    </label>
                    <select
                      value={selectedNode.pulleyWheelId || ''}
                      onChange={(e) => updateRopeParams(selectedNode.id, { pulleyWheelId: e.target.value })}
                      className="w-full text-xs border border-slate-200 rounded p-1.5 bg-slate-50 font-medium text-slate-700 focus:border-blue-500 outline-none"
                    >
                      <option value="">-- None (direct coupling) --</option>
                      {allPulleyWheels.map(wheel => (
                        <option key={wheel.id} value={wheel.id}>
                          {wheel.id} (r={( wheel.pulleyRadius || 0.4).toFixed(2)}m)
                        </option>
                      ))}
                    </select>
                  </div>
                </div>
              )}

              {/* Component Control Script Card */}
              <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2.5">
                <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center justify-between">
                  <span className="flex items-center gap-1.5 font-semibold text-slate-800">
                    <Code className="w-4 h-4 text-blue-500" />
                    Component Script
                  </span>
                  <div className="flex items-center gap-1.5">
                    {selectedNode.script ? (
                      <span className="flex items-center gap-1 text-[10px] font-semibold text-emerald-600 bg-emerald-50 px-1.5 py-0.5 rounded-full border border-emerald-100 animate-pulse">
                        <span className="w-1.5 h-1.5 rounded-full bg-emerald-500" />
                        Active
                      </span>
                    ) : (
                      <span className="text-[10px] font-medium text-slate-400 bg-slate-50 px-1.5 py-0.5 rounded-full border border-slate-100">
                        Disabled
                      </span>
                    )}
                  </div>
                </h3>

                <p className="text-[10px] text-slate-400 -mt-1 leading-tight">
                  Write custom real-time JavaScript to control this component at 1000Hz.
                </p>

                {/* Templates Selector */}
                <div className="flex items-center justify-between text-xs text-slate-500 gap-1.5 bg-slate-50 p-1.5 rounded-md border border-slate-100">
                  <span className="text-[10px] font-medium text-slate-400 uppercase tracking-wider">Templates:</span>
                  <select
                    onChange={(e) => {
                      const templateVal = e.target.value;
                      if (templateVal === 'lqr') {
                        setScriptText(`// Cartpole LQR Balancing Controller
const x = api.getJointPosition('cart_slide');
const v = api.getJointVelocity('cart_slide');
const theta = api.getJointPosition('pole_hinge');
const omega = api.getJointVelocity('pole_hinge');

// State-feedback LQR controller gains
const kx = 22.0;      // Cart position gain
const kv = 15.0;      // Cart velocity damping
const kTheta = 80.0;  // Pole angle gain (robust tracking)
const kOmega = 20.0;  // Pole angular velocity damping

// Compute the balancing force
const force = (kx * x) + (kv * v) + (kTheta * theta) + (kOmega * omega);

// Apply force directly to the cart slide joint
api.applyJointForce('cart_slide', force);
`);
                      } else if (templateVal === 'sine') {
                        setScriptText(`// Sinusoidal Driver
const forceX = Math.sin(api.getTime() * 5.0) * 8.0;
api.applyForce([forceX, 0, 0]);
`);
                      } else if (templateVal === 'spring') {
                        setScriptText(`// PD Harmonic Spring / Return-to-Center
const pos = api.getPosition();
const dist = 0.0 - pos[0];
const vel = api.getVelocity()[0];

// PD coefficients
const kp = 25.0; // Spring constant
const kd = 5.0;  // Damping

const force = (kp * dist) - (kd * vel);
api.applyForce([force, 0, 0]);
`);
                      } else if (templateVal === 'clear') {
                        setScriptText('');
                      }
                      e.target.value = ''; // Reset selection
                    }}
                    className="text-xs bg-white border border-slate-200 rounded px-1.5 py-0.5 text-slate-700 outline-none focus:border-blue-500 cursor-pointer"
                  >
                    <option value="">-- Select Template --</option>
                    <option value="lqr">LQR Cartpole Balancer</option>
                    <option value="sine">Sinusoidal Driver</option>
                    <option value="spring">PD Harmonic Spring</option>
                    <option value="clear">Clear Script</option>
                  </select>
                </div>

                {/* Text Area Code Editor */}
                <div className="relative">
                  <textarea
                    value={scriptText}
                    onChange={(e) => setScriptText(e.target.value)}
                    placeholder="// Write control logic here... e.g. api.applyForce([10, 0, 0])"
                    className="w-full h-40 font-mono text-[11px] leading-relaxed p-2.5 bg-slate-950 text-emerald-400 rounded-lg border border-slate-800 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent resize-y shadow-inner"
                    spellCheck={false}
                  />
                  <div className="absolute right-2.5 bottom-2.5 text-[8px] font-mono text-slate-600 bg-slate-900/50 px-1 rounded pointer-events-none select-none border border-slate-800">
                    JS
                  </div>
                </div>

                {/* Compilation Error Display */}
                {scriptError && (
                  <div className="p-2.5 bg-amber-50 border border-amber-200 text-amber-800 rounded-lg text-[10px] flex gap-1.5 items-start leading-tight">
                    <span className="font-bold shrink-0">⚠️ Error:</span>
                    <span className="font-mono text-slate-700 break-all">{scriptError}</span>
                  </div>
                )}

                {/* Control Actions Row */}
                <div className="flex gap-2 items-center justify-between">
                  <button
                    onClick={() => setShowApiRef(!showApiRef)}
                    className="text-[10px] font-semibold text-blue-600 hover:text-blue-700 transition-colors flex items-center gap-1 cursor-pointer"
                  >
                    <Info className="w-3.5 h-3.5" />
                    {showApiRef ? 'Hide API Reference' : 'Show API Reference'}
                  </button>

                  <button
                    onClick={handleSaveScript}
                    className="px-3 py-1.5 bg-blue-600 hover:bg-blue-700 active:bg-blue-800 text-white rounded-lg text-[11px] font-semibold shadow transition-colors flex items-center gap-1 cursor-pointer"
                  >
                    <Zap className="w-3.5 h-3.5" />
                    Save & Execute
                  </button>
                </div>

                {/* API Reference Collapsible */}
                {showApiRef && (
                  <div className="text-[10px] bg-slate-50 border border-slate-150 rounded-lg p-2.5 flex flex-col gap-2 font-sans text-slate-600 max-h-48 overflow-y-auto">
                    <div className="font-semibold text-slate-700 border-b border-slate-200 pb-1 mb-1">Available API Methods:</div>
                    <div className="flex flex-col gap-1.5">
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getPosition(bodyName?)</code>
                        <p className="text-slate-500 mt-0.5">Returns array <code className="font-mono">[x, y, z]</code> of body position.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getVelocity(bodyName?)</code>
                        <p className="text-slate-500 mt-0.5">Returns array <code className="font-mono">[vx, vy, vz]</code> of body velocity.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getAngularVelocity(bodyName?)</code>
                        <p className="text-slate-500 mt-0.5">Returns array <code className="font-mono">[wx, wy, wz]</code> of body angular velocity.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getMass(bodyName?)</code>
                        <p className="text-slate-500 mt-0.5">Returns body mass in kg.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getJointPosition(jointName)</code>
                        <p className="text-slate-500 mt-0.5">Returns joint position (m for slide, rad for hinge).</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getJointVelocity(jointName)</code>
                        <p className="text-slate-500 mt-0.5">Returns joint velocity (m/s for slide, rad/s for hinge).</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.applyForce(forceVec, bodyName?)</code>
                        <p className="text-slate-500 mt-0.5">Applies external force <code className="font-mono">[fx, fy, fz]</code> to a body.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.applyJointForce(jointName, forceVal)</code>
                        <p className="text-slate-500 mt-0.5">Applies joint-aligned torque or force.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.setActuatorControl(actuatorName, ctrlVal)</code>
                        <p className="text-slate-500 mt-0.5">Sets input control value on an actuator.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.getTime()</code>
                        <p className="text-slate-500 mt-0.5">Returns elapsed simulation time in seconds.</p>
                      </div>
                      <div>
                        <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">api.log(msg)</code>
                        <p className="text-slate-500 mt-0.5">Logs message to browser developer console.</p>
                      </div>
                    </div>
                  </div>
                )}
              </div>

              {/* Delete Component Button */}
              <button 
                onClick={() => deleteNode(selectedNode.id)}
                className="mt-2 flex items-center justify-center gap-2 w-full py-2 border border-red-200 rounded-lg text-xs font-semibold text-red-600 bg-red-50 hover:bg-red-100 hover:border-red-300 transition-colors shadow-sm cursor-pointer"
              >
                <Trash2 className="w-4 h-4" /> Delete Component
              </button>
            </div>
          </aside>
        )}
      </div>

      {isDocsOpen && (
        <div className="fixed inset-0 bg-slate-900/60 backdrop-blur-sm z-50 flex items-center justify-center p-4">
          <div className="bg-white rounded-2xl border border-slate-100 shadow-2xl max-w-2xl w-full max-h-[80vh] flex flex-col overflow-hidden animate-in fade-in zoom-in-95 duration-200">
            {/* Modal Header */}
            <div className="px-6 py-4 border-b border-slate-150 flex items-center justify-between bg-slate-50">
              <div className="flex items-center gap-2">
                <Info className="w-5 h-5 text-blue-500" />
                <h2 className="font-bold text-slate-800 text-base">Physics Expt Reference Guide</h2>
              </div>
              <button 
                onClick={() => setIsDocsOpen(false)}
                className="p-1 rounded-lg hover:bg-slate-200 text-slate-400 hover:text-slate-600 transition-colors cursor-pointer"
              >
                <X className="w-5 h-5" />
              </button>
            </div>

            {/* Modal Content Split */}
            <div className="flex flex-1 overflow-hidden min-h-0">
              {/* Tab Navigation */}
              <div className="w-48 bg-slate-50 border-r border-slate-150 p-3 flex flex-col gap-1.5 shrink-0 overflow-y-auto">
                <button 
                  onClick={() => setDocsTab('gravity')}
                  className={`px-3 py-2 text-left rounded-lg text-xs font-semibold transition-all ${docsTab === 'gravity' ? 'bg-blue-500 text-white shadow' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                  🪐 Gravity & Inertia
                </button>
                <button 
                  onClick={() => setDocsTab('coupling')}
                  className={`px-3 py-2 text-left rounded-lg text-xs font-semibold transition-all ${docsTab === 'coupling' ? 'bg-blue-500 text-white shadow' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                  ⚙️ Joint Coupling
                </button>
                <button 
                  onClick={() => setDocsTab('collision')}
                  className={`px-3 py-2 text-left rounded-lg text-xs font-semibold transition-all ${docsTab === 'collision' ? 'bg-blue-500 text-white shadow' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                  💥 Collision Physics
                </button>
                <button 
                  onClick={() => setDocsTab('friction')}
                  className={`px-3 py-2 text-left rounded-lg text-xs font-semibold transition-all ${docsTab === 'friction' ? 'bg-blue-500 text-white shadow' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                  🛷 Friction Controls
                </button>
                <button 
                  onClick={() => setDocsTab('scripting')}
                  className={`px-3 py-2 text-left rounded-lg text-xs font-semibold transition-all ${docsTab === 'scripting' ? 'bg-blue-500 text-white shadow' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                  💻 Control Scripting
                </button>
              </div>

              {/* Tab Panel */}
              <div className="flex-1 p-6 overflow-y-auto">
                {docsTab === 'gravity' && (
                  <div className="flex flex-col gap-4">
                    <h3 className="font-bold text-slate-800 text-lg flex items-center gap-1.5">🪐 Gravity, Active Joints & Inertia</h3>
                    <p className="text-xs text-slate-600 leading-relaxed">
                      In the physics solver (powered by MuJoCo), gravity exerts a continuous force vector downward along the Z-axis. However, how components react depends entirely on their <strong>Degrees of Freedom (joints)</strong> and <strong>Inertia</strong>:
                    </p>
                    <div className="bg-slate-50 border border-slate-200/60 rounded-xl p-4 flex flex-col gap-3">
                      <div className="text-xs">
                        <strong className="text-slate-700">🌍 Static Elements (No Joints)</strong>
                        <p className="text-slate-500 mt-1">Shelves, pegs, and support structures have no joints. The solver treats them as having infinite mass welded directly to the world body, so gravity never moves them.</p>
                      </div>
                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-700">⚙️ Hinge Gears (Rotational Hinge Joints)</strong>
                        <p className="text-slate-500 mt-1">Gears are locked to a single pivot point. Because gravity acts straight down through the pivot, it produces zero torque around the rotation axis. Symmetrical shapes also have their center of mass balanced perfectly at the pivot, preventing gravity from inducing rotation.</p>
                      </div>
                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-700">📦 Unconstrained Bodies (Free Joints)</strong>
                        <p className="text-slate-500 mt-1">A floating box (like the gold cube) has a free joint, allowing full 3D physics simulation to pull it down naturally.</p>
                      </div>
                    </div>
                  </div>
                )}

                {docsTab === 'coupling' && (
                  <div className="flex flex-col gap-4">
                    <h3 className="font-bold text-slate-800 text-lg flex items-center gap-1.5">⚙️ Mechanical Joint Coupling vs Collision</h3>
                    <p className="text-xs text-slate-600 leading-relaxed">
                      Why does this application use mathematical joint coupling rather than direct tooth-on-tooth rigid collisions?
                    </p>
                    <div className="bg-slate-50 border border-slate-200/60 rounded-xl p-4 flex flex-col gap-3">
                      <div className="text-xs">
                        <strong className="text-slate-700">⚡ The Jitter & Penetration Problem</strong>
                        <p className="text-slate-500 mt-1">In discrete time-step simulators, rigid teeth can slightly overlap between steps. Resolving these penetrations produces massive outward impulses, causing gears to lock up, vibrate, or explode.</p>
                      </div>
                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-700">🔗 Mathematical Coupling</strong>
                        <p className="text-slate-500 mt-1">By applying a mathematical joint relationship (bilateral constraint), the system simulates perfectly smooth, 100% stable, and silent transmission of energy at all speeds.</p>
                      </div>
                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-700">🎯 Dynamic Proximity Engine</strong>
                        <p className="text-slate-500 mt-1">To ensure realistic spatial mechanics, the coupling is proximity-aware! Gears and pinion-racks only couple when they are touching. You can toggle this constraint using the "Allow Mechanical Coupling" checkbox in the sidebar.</p>
                      </div>
                    </div>
                  </div>
                )}

                {docsTab === 'collision' && (
                  <div className="flex flex-col gap-4">
                    <h3 className="font-bold text-slate-800 text-lg flex items-center gap-1.5">💥 Collision Physics & Solid vs Ephemeral</h3>
                    <p className="text-xs text-slate-600 leading-relaxed">
                      You can toggle whether components behave as solid, physical obstacles or ephemeral visual guides:
                    </p>
                    <div className="bg-slate-50 border border-slate-200/60 rounded-xl p-4 flex flex-col gap-3">
                      <div className="text-xs">
                        <strong className="text-slate-700">🛑 Solid Mode (Collision Enabled)</strong>
                        <p className="text-slate-500 mt-1">The body participates in the contact solver. It blocks other objects, pushes them, and participates fully in normal physics collisions.</p>
                      </div>
                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-700">👻 Ephemeral Mode (Collision Disabled)</strong>
                        <p className="text-slate-500 mt-1">Sets <code>contype="0"</code> and <code>conaffinity="0"</code>. The body becomes completely non-solid. Other items can pass straight through it. Excellent for creating decorative supports or visual-only guides!</p>
                      </div>
                    </div>
                  </div>
                )}

                 {docsTab === 'friction' && (
                  <div className="flex flex-col gap-4">
                    <h3 className="font-bold text-slate-800 text-lg flex items-center gap-1.5">🛷 Dynamic Friction Tuning</h3>
                    <p className="text-xs text-slate-600 leading-relaxed">
                      Friction coefficients dictate how easily objects slide against each other:
                    </p>
                    <div className="bg-slate-50 border border-slate-200/60 rounded-xl p-4 flex flex-col gap-3">
                      <div className="text-xs">
                        <strong className="text-slate-700">🌍 Floor Friction</strong>
                        <p className="text-slate-500 mt-1">Adjusts the grip of the ground plane. Setting it to 0.0 makes the ground an frictionless ice-sheet. Increased values yield high-traction surfaces.</p>
                      </div>
                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-700">📦 Component Friction</strong>
                        <p className="text-slate-500 mt-1">Sets the sliding friction coefficient of the selected object. Lower values allow materials to slip easily past support shelves and guide Rails, while high values prevent slipping.</p>
                      </div>
                    </div>
                  </div>
                )}

                {docsTab === 'scripting' && (
                  <div className="flex flex-col gap-4">
                    <h3 className="font-bold text-slate-800 text-lg flex items-center gap-1.5">💻 Control Scripting & Joint Names</h3>
                    <p className="text-xs text-slate-600 leading-relaxed">
                      Custom JavaScript control scripts run inside the physics solver loop on every physics time-step. To query state or apply forces, you pass string-based <strong>body names</strong> or <strong>joint names</strong> to the API.
                    </p>
                    
                    <div className="bg-slate-50 border border-slate-200/60 rounded-xl p-4 flex flex-col gap-4">
                      <div className="text-xs">
                        <strong className="text-slate-800 font-semibold flex items-center gap-1">🏷️ Where do joint & body names come from?</strong>
                        <p className="text-slate-500 mt-1 leading-relaxed">
                          All names map directly to the values you configure in the <strong>Properties Panel</strong> when a component is selected:
                        </p>
                        <ul className="list-disc pl-4 mt-1.5 text-slate-500 flex flex-col gap-1">
                          <li><strong>Body Names:</strong> Equal to the <strong>Component Name</strong> at the top of the properties panel (e.g. <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">"cart"</code> or <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">"pole"</code>).</li>
                          <li><strong>Joint Names:</strong> Configured in the <strong>Joint Name (for API)</strong> text input under the <strong>🔗 Joint Type</strong> card (e.g. <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">"cart_slide"</code> or <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">"pole_hinge"</code>).</li>
                          <li><strong>Actuator/Motor Names:</strong> If you select "Enable Motor Drive", the actuator is automatically named by appending <code className="font-mono">_actuator</code> to the joint name (e.g. <code className="font-mono text-blue-600 bg-blue-50 px-1 rounded">"cart_slide_actuator"</code>).</li>
                        </ul>
                      </div>

                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-800 font-semibold">🔄 Retrieving Sensor Data & Joint Positions</strong>
                        <p className="text-slate-500 mt-1 leading-relaxed">
                          Use the following API methods in your script:
                        </p>
                        <pre className="mt-2 bg-slate-950 text-emerald-400 p-2.5 rounded-lg font-mono text-[10px] leading-relaxed shadow-inner overflow-x-auto">
{`// 1. Get positions & velocities of components in world space
const [x, y, z] = api.getPosition('cart');
const [vx, vy, vz] = api.getVelocity('cart');

// 2. Get joint-aligned values (highly recommended for controls)
const position = api.getJointPosition('cart_slide'); // Slider: meters, Hinge: radians
const velocity = api.getJointVelocity('cart_slide'); // Slider: m/s, Hinge: rad/s`}
                        </pre>
                      </div>

                      <div className="text-xs border-t border-slate-150 pt-3">
                        <strong className="text-slate-800 font-semibold">⚡ Applying Forces & Controlling Motors</strong>
                        <p className="text-slate-500 mt-1 leading-relaxed">
                          Apply forces directly, or write a PD / LQR control loop to command motors:
                        </p>
                        <pre className="mt-2 bg-slate-950 text-emerald-400 p-2.5 rounded-lg font-mono text-[10px] leading-relaxed shadow-inner overflow-x-auto">
{`// Apply torque or force aligned to the joint
api.applyJointForce('cart_slide', 15.5); // Applies linear force

// Command actuator motor velocity target
api.setActuatorControl('cart_slide_actuator', 1.0); // Drive cart at 1.0 m/s`}
                        </pre>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default App;
