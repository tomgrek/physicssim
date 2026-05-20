import { create } from 'zustand';
import load_mujoco from '@mujoco/mujoco';
import type { SceneGraph, SceneNode } from '../types/scene';
import { compileToMJCF } from '../utils/mjcf';
import { PRESETS, pendulumPreset, generateGearGeoms } from '../presets/presetScenes';

const initialScene: SceneGraph = pendulumPreset;

const getNodeWorldPos = (nodes: any[], targetId: string, currentOffset: [number, number, number] = [0, 0, 0]): [number, number, number] | null => {
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
};

const addChildNode = (nodes: any[], parentId: string, newNode: any): boolean => {
  for (const node of nodes) {
    if (node.id === parentId) {
      if (!node.children) node.children = [];
      node.children.push(newNode);
      return true;
    }
    if (node.children && addChildNode(node.children, parentId, newNode)) {
      return true;
    }
  }
  return false;
};

export interface PhysicsState {
  mujoco: any;
  model: any;
  data: any;
  
  isPlaying: boolean;
  isLoaded: boolean;
  isSettingsOpen: boolean;
  cameraView: 'perspective' | 'topDown';
  
  // Environment
  gravityZ: number;
  windX: number;
  windY: number;
  density: number;
  floorFriction: number;
  
  // Scene
  sceneGraph: SceneGraph;
  selectedNodeId: string | null;
  recompileId: number;
  
  // Actions
  setEngine: (mujoco: any, model: any, data: any) => void;
  togglePlay: () => void;
  setLoaded: (loaded: boolean) => void;
  setSettingsOpen: (open: boolean) => void;
  setCameraView: (view: 'perspective' | 'topDown') => void;
  setEnvironment: (env: Partial<{gravityZ: number, windX: number, windY: number, density: number, floorFriction: number}>) => void;
  
  setSelectedNodeId: (id: string | null) => void;
  updateScene: (sceneGraph: SceneGraph) => void;
  updateNodePos: (id: string, newPos: [number, number, number]) => void;
  updateNodeGeom: (id: string, updates: any) => void;
  updateNodeJoint: (id: string, updates: any) => void;
  updateGearTeeth: (id: string, teeth: number) => void;
  updateNodeRotation: (id: string, axis: 0 | 1 | 2, deg: number) => void;
  
  renameNode: (id: string, newName: string) => void;
  updateNodeJointsList: (id: string, joints: any[]) => void;
  deleteNode: (id: string) => void;
  addPusherPeg: (gearId: string) => void;
  deletePusherPeg: (gearId: string) => void;
  updatePusherPeg: (gearId: string, updates: { offset?: number, size?: [number, number] }) => void;
  
  addComponent: (type: 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear', position: number[]) => void;
  recompile: (overrideScene?: SceneGraph, overrideSelectedId?: string | null, forceReset?: boolean) => void;
  loadPreset: (name: 'pendulum' | 'cubes' | 'gears' | 'machine' | 'rack_pinion') => void;
  resetSimulation: () => void;
}

export const useStore = create<PhysicsState>()((set, get) => ({
  mujoco: null,
  model: null,
  data: null,
  recompileId: 0,

  isPlaying: false,
  isLoaded: false,
  isSettingsOpen: false,
  cameraView: 'perspective',
  
  gravityZ: -9.81,
  windX: 0,
  windY: 0,
  density: 0,
  floorFriction: 1.0,
  
  sceneGraph: initialScene,
  selectedNodeId: null,

  setEngine: (mujoco, model, data) => set({ mujoco, model, data, isLoaded: true }),
  
  togglePlay: () => set((state) => ({ isPlaying: !state.isPlaying })),
  setLoaded: (loaded) => set({ isLoaded: loaded }),
  setSettingsOpen: (open) => set({ isSettingsOpen: open }),
  setCameraView: (view) => set({ cameraView: view }),
  
  resetSimulation: () => {
    set({ isPlaying: false });
    get().recompile(undefined, undefined, true);
  },
  
  loadPreset: (name) => {
    const preset = PRESETS[name];
    if (!preset) return;
    set({ isPlaying: false, selectedNodeId: null });
    get().recompile(preset.scene, null, true);
  },
  
  setEnvironment: (env) => {
    set(env);
    get().recompile(get().sceneGraph);
  },
  
  setSelectedNodeId: (id) => set({ selectedNodeId: id }),
  
  updateScene: (newScene) => {
    set({ sceneGraph: newScene });
    get().recompile(newScene);
  },
  
  renameNode: (id, newName) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph));
    const traverse = (nodes: any[]) => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          node.name = newName;
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    set({ sceneGraph: newScene });
    get().recompile(newScene);
  },
  
  updateNodePos: (id, newPos) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph));
    const traverse = (nodes: any[]) => {
      if (!nodes) return false; for (const node of nodes) {
        if (node.id === id) {
          node.pos = newPos;
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    set({ sceneGraph: newScene });
    get().recompile(newScene, undefined, true);
  },

  updateNodeRotation: (id, axis, deg) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]) => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          const currentEuler = node.euler ? [...node.euler] : [0, 0, 0];
          currentEuler[axis] = deg;
          node.euler = currentEuler as [number, number, number];
          delete node.quat;
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    if (traverse(newScene.nodes)) {
      set({ sceneGraph: newScene });
      get().recompile(newScene, undefined, true);
    }
  },

  updateNodeGeom: (id, updates) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph));
    const traverse = (nodes: any[]) => {
      if (!nodes) return false; for (const node of nodes) {
        if (node.id === id && node.geoms?.length > 0) {
          if (id.includes('gear')) {
            const centerGeom = node.geoms[0];
            const currentRadius = centerGeom.size[0];
            const currentTeeth = node.geoms.length - 1;
            const currentColor = centerGeom.rgba || [0.5, 0.5, 0.5, 1];
            const currentContype = centerGeom.contype !== undefined ? centerGeom.contype : 0;
            const currentConaffinity = centerGeom.conaffinity !== undefined ? centerGeom.conaffinity : 0;
            
            let newRadius = currentRadius;
            if (updates.size && Array.isArray(updates.size)) {
              newRadius = updates.size[0];
            }
            
            let newColor = currentColor;
            if (updates.rgba) {
              newColor = updates.rgba;
            }

            let newContype = currentContype;
            if (updates.contype !== undefined) {
              newContype = updates.contype;
            }

            let newConaffinity = currentConaffinity;
            if (updates.conaffinity !== undefined) {
              newConaffinity = updates.conaffinity;
            }
            
            const gearNum = parseInt(id.replace(/\D/g, '')) || 1;
            const isSecondGear = gearNum % 2 === 0;
            
            node.geoms = generateGearGeoms(id, newRadius, currentTeeth, newColor, isSecondGear, newContype, newConaffinity);
          } else {
            Object.assign(node.geoms[0], updates);
          }
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    set({ sceneGraph: newScene });
    get().recompile(newScene, undefined, true);
  },

  updateGearTeeth: (id, teeth) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]): boolean => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id && node.geoms?.length > 0) {
          const centerGeom = node.geoms[0];
          const radius = centerGeom.size[0];
          const color = centerGeom.rgba || [0.5, 0.5, 0.5, 1];
          const gearNum = parseInt(id.replace(/\D/g, '')) || 1;
          const isSecondGear = gearNum % 2 === 0;
          const contype = centerGeom.contype !== undefined ? centerGeom.contype : 0;
          const conaffinity = centerGeom.conaffinity !== undefined ? centerGeom.conaffinity : 0;
          
          node.geoms = generateGearGeoms(id, radius, teeth, color, isSecondGear, contype, conaffinity);
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    if (traverse(newScene.nodes)) {
      get().recompile(newScene, id);
    }
  },

  addPusherPeg: (gearId) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]): boolean => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === gearId && node.geoms?.length > 0) {
          const hasPeg = node.geoms.some((g: any) => g.name.includes('peg'));
          if (!hasPeg) {
            const radius = node.geoms[0].size[0];
            node.geoms.push({
              name: `${gearId}_pusher_peg`,
              type: 'cylinder',
              size: [0.03, 0.08], // radius, half-height
              pos: [radius * 0.8, 0, 0.09], // relative offset
              rgba: [0.9, 0.2, 0.2, 1], // red color
              mass: 0.05,
              condim: 3,
              solref: [0.015, 1.0],
              solimp: [0.95, 0.99, 0.001, 0.5, 2]
            });
          }
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    if (traverse(newScene.nodes)) {
      get().recompile(newScene, gearId);
    }
  },

  deletePusherPeg: (gearId) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]): boolean => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === gearId && node.geoms?.length > 1) {
          node.geoms = node.geoms.filter((g: any) => !g.name.includes('peg'));
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    if (traverse(newScene.nodes)) {
      get().recompile(newScene, gearId);
    }
  },

  updatePusherPeg: (gearId, updates) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]): boolean => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === gearId && node.geoms?.length > 1) {
          const peg = node.geoms.find((g: any) => g.name.includes('peg'));
          if (peg) {
            if (updates.offset !== undefined) {
              peg.pos = [updates.offset, 0, peg.pos[2]];
            }
            if (updates.size !== undefined) {
              peg.size = updates.size; // [radius, half_height]
              peg.pos = [peg.pos[0], 0, updates.size[1] + 0.01]; // adjust Z pos dynamically so bottom touches the disc
            }
          }
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    if (traverse(newScene.nodes)) {
      get().recompile(newScene, gearId);
    }
  },

  updateNodeJoint: (id, updates) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph));
    const traverse = (nodes: any[]) => {
      if (!nodes) return false; for (const node of nodes) {
        if (node.id === id && node.joints?.length > 0) {
          Object.assign(node.joints[0], updates);
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    set({ sceneGraph: newScene });
    get().recompile();
  },

  updateNodeJointsList: (id, joints) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]): boolean => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          node.joints = joints;
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    if (traverse(newScene.nodes)) {
      get().recompile(newScene, id);
    }
  },

  deleteNode: (id) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverseAndRemove = (nodes: any[]): boolean => {
      if (!nodes) return false;
      for (let i = 0; i < nodes.length; i++) {
        if (nodes[i].id === id) {
          nodes.splice(i, 1);
          return true;
        }
        if (traverseAndRemove(nodes[i].children)) return true;
      }
      return false;
    };
    traverseAndRemove(newScene.nodes);
    get().recompile(newScene, null);
  },

  addComponent: (type, position) => {
    console.log("addComponent START", type, position);
    const { sceneGraph, selectedNodeId } = get();
    const newScene = JSON.parse(JSON.stringify(sceneGraph)) as SceneGraph;
    
    const id = `${type}_${Date.now()}`;
    
    // Determine target local position
    let localPos: [number, number, number] = [position[0], position[1], position[2]];
    
    if (selectedNodeId) {
      // Find parent world position to make drop coordinates relative to parent!
      const parentWorldPos = getNodeWorldPos(newScene.nodes, selectedNodeId);
      if (parentWorldPos) {
        localPos = [
          position[0] - parentWorldPos[0],
          position[1] - parentWorldPos[1],
          position[2] - parentWorldPos[2]
        ];
      }
    }

    let geomType: any = type;
    let size: number[] = [0.2];
    let rgba = [0.5, 0.5, 0.5, 1];
    let mass = 1;
    let joints: any[] = [];
    
    const isChild = !!selectedNodeId;
    
    if (type === 'gear') {
      const radius = 0.5;
      const teeth = 12;
      const color = [0.5, 0.5, 0.5, 1];
      const geoms = generateGearGeoms(id, radius, teeth, color, false);
      joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 0, 1], damping: 0.5 }];
      
      const newNode: SceneNode = {
        id, name: id, type: 'body', pos: localPos,
        joints, geoms, children: []
      };
      
      if (selectedNodeId) {
        addChildNode(newScene.nodes, selectedNodeId, newNode);
      } else {
        newScene.nodes.push(newNode);
      }
    } else {
      if (type === 'box') {
        geomType = 'box';
        size = [0.2, 0.2, 0.2];
        rgba = [0.8, 0.2, 0.2, 1];
        joints = isChild ? [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }] : [{ name: `${id}_free`, type: 'free' }];
      } else if (type === 'sphere') {
        geomType = 'sphere';
        size = [0.2];
        rgba = [0.2, 0.8, 0.2, 1];
        joints = isChild ? [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }] : [{ name: `${id}_free`, type: 'free' }];
      } else if (type === 'capsule') {
        geomType = 'capsule';
        size = [0.04, 0.4];
        rgba = [0.6, 0.6, 0.6, 1];
        joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.1 }];
      } else if (type === 'cylinder') {
        geomType = 'cylinder';
        size = [0.2, 0.1];
        rgba = [0.9, 0.6, 0.1, 1];
        joints = isChild ? [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }] : [{ name: `${id}_free`, type: 'free' }];
      } else if (type === 'bob') {
        geomType = 'sphere';
        size = [0.15];
        rgba = [0.2, 0.6, 1.0, 1];
        mass = 10.0;
        joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.1 }];
      }
      
      const newNode: SceneNode = {
        id, name: id, type: 'body', pos: localPos,
        joints,
        geoms: [{ name: `${id}_geom`, type: geomType, size, mass, rgba }],
        children: []
      };
      
      if (selectedNodeId) {
        addChildNode(newScene.nodes, selectedNodeId, newNode);
      } else {
        newScene.nodes.push(newNode);
      }
    }
    
    console.log("addComponent DONE, calling recompile");
    const selectId = typeof window !== 'undefined' && (window as any).NO_SELECT ? null : id;
    get().recompile(newScene, selectId);
  },
  
  recompile: async (overrideScene?: SceneGraph, overrideSelectedId?: string | null, forceReset?: boolean) => {
    console.log("recompile START");
    const { gravityZ, floorFriction, model: oldModel, data: oldData } = get();
    const sceneGraph = overrideScene ?? get().sceneGraph;
    
    const xml = compileToMJCF(sceneGraph, gravityZ, floorFriction);
    console.log("XML generated:\n", xml);
    if (typeof window !== 'undefined') {
      (window as any).compiledXML = xml;
    }
    
    try {
      console.log("Loading a fresh MuJoCo WASM module...");
      const freshMujoco = await load_mujoco();
      
      const newModel = freshMujoco.MjModel.from_xml_string(xml);
      console.log("newModel created");
      
      console.log("body_mass:", Array.from(newModel.body_mass).join(', '));
      console.log("body_inertia:", Array.from(newModel.body_inertia).join(', '));
      
      const newData = new freshMujoco.MjData(newModel);
      console.log("newData created");
      
      if (!forceReset && oldModel && oldData && oldModel.nq === newModel.nq && oldModel.nv === newModel.nv) {
        console.log("Copying old state");
        const nq = Math.min(oldModel.nq, newModel.nq);
        const nv = Math.min(oldModel.nv, newModel.nv);
        const nu = Math.min(oldModel.nu, newModel.nu);
        for (let i = 0; i < nq; i++) newData.qpos[i] = oldData.qpos[i];
        for (let i = 0; i < nv; i++) newData.qvel[i] = oldData.qvel[i];
        for (let i = 0; i < nu; i++) newData.ctrl[i] = oldData.ctrl[i];
        
        console.log("newData qpos:", Array.from(newData.qpos).join(', '));
        console.log("newData qvel:", Array.from(newData.qvel).join(', '));
        
        freshMujoco.mj_forward(newModel, newData);
        console.log("newData geom_xpos:", Array.from(newData.geom_xpos).join(', '));
        console.log("newData geom_xmat:", Array.from(newData.geom_xmat).join(', '));
      } else {
        console.log("Initial qpos:", Array.from(newData.qpos).join(', '));
        console.log("Initial qvel:", Array.from(newData.qvel).join(', '));
        
        // Initialize control values from actuators
        const actuators: any[] = [];
        const traverse = (nodes: any[]) => {
          for (const node of nodes) {
            node.joints?.forEach((j: any) => { if (j.actuator) actuators.push(j); });
            traverse(node.children);
          }
        };
        traverse(sceneGraph.nodes);
        console.log("TRAVERSE SCENE NODES COUNT:", sceneGraph.nodes.length);
        console.log("FOUND ACTUATORS:", actuators.length);
        actuators.forEach((j, idx) => {
          console.log(`ACTUATOR ${idx} name:`, j.name, "ctrlValue:", j.actuator?.ctrlValue);
          if (j.actuator && j.actuator.ctrlValue !== undefined) {
            newData.ctrl[idx] = j.actuator.ctrlValue;
            console.log(`SET newData.ctrl[${idx}] =`, newData.ctrl[idx]);
          }
        });

        freshMujoco.mj_forward(newModel, newData);
        console.log("Initial geom_xpos:", Array.from(newData.geom_xpos).join(', '));
      }
      
      // ONE atomic set — sceneGraph, model, data, mujoco, recompileId and selectedNodeId all update together
      // Deferred to next animation frame to avoid blocking the event loop
      const updates: Partial<PhysicsState> = { mujoco: freshMujoco, model: newModel, data: newData, sceneGraph, recompileId: Date.now() };
      if (overrideSelectedId !== undefined) updates.selectedNodeId = overrideSelectedId;
      requestAnimationFrame(() => {
        console.log("Atomic set: fresh mujoco + model + scene together");
        set(updates);
      });
    } catch (e) {
      console.error("Failed to compile MJCF:", e);
    }
  }
}));
