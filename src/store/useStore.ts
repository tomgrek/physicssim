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
  parentUnderSelected: boolean;
  
  draggedNodeId: string | null;
  dragTarget: { x: number; y: number; z: number } | null;
  dragDistance: number;
  
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
  updateNodeScript: (id: string, script: string) => void;
  
  renameNode: (id: string, newName: string) => void;
  updateNodeJointsList: (id: string, joints: any[]) => void;
  deleteNode: (id: string) => void;
  addPusherPeg: (gearId: string) => void;
  deletePusherPeg: (gearId: string) => void;
  updatePusherPeg: (gearId: string, updates: { offset?: number, size?: [number, number] }) => void;
  
  setDraggedNodeId: (id: string | null) => void;
  setDragTarget: (target: { x: number; y: number; z: number } | null) => void;
  setDragDistance: (distance: number) => void;
  updateWedgeParams: (id: string, params: { width?: number; depth?: number; height?: number; wedgeAngle?: number }) => void;
  updatePulleyParams: (id: string, params: { leftTargetId?: string; rightTargetId?: string; pulleyRadius?: number }) => void;
  updateRopeParams: (id: string, params: { pulleyWheelId?: string; leftTargetId?: string; rightTargetId?: string }) => void;
  
  setParentUnderSelected: (val: boolean) => void;
  addComponent: (type: 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear' | 'wedge' | 'pulley_wheel' | 'pulley_rope', position: number[]) => void;
  recompile: (overrideScene?: SceneGraph, overrideSelectedId?: string | null, forceReset?: boolean) => void;
  loadPreset: (name: 'pendulum' | 'cubes' | 'gears' | 'machine' | 'rack_pinion' | 'inclined_plane' | 'pulley_system' | 'cartpole') => void;
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
  parentUnderSelected: false,
  draggedNodeId: null,
  dragTarget: null,
  dragDistance: 0,

  setParentUnderSelected: (val) => set({ parentUnderSelected: val }),
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
  
  setDraggedNodeId: (id) => set({ draggedNodeId: id }),
  setDragTarget: (target) => set({ dragTarget: target }),
  setDragDistance: (distance) => set({ dragDistance: distance }),

  updateWedgeParams: (id, params) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]) => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          if (params.width !== undefined) {
            node.width = params.width;
            // Recalculate wedgeAngle
            const h = node.height || 0.5;
            node.wedgeAngle = Math.atan(h / node.width) * 180 / Math.PI;
          }
          if (params.height !== undefined) {
            node.height = params.height;
            // Recalculate wedgeAngle
            const w = node.width || 2.0;
            node.wedgeAngle = Math.atan(node.height / w) * 180 / Math.PI;
          }
          if (params.depth !== undefined) {
            node.depth = params.depth;
          }
          if (params.wedgeAngle !== undefined) {
            node.wedgeAngle = params.wedgeAngle;
            // Recalculate height
            const w = node.width || 2.0;
            node.height = w * Math.tan(node.wedgeAngle * Math.PI / 180);
          }
          
          // Update first geom size
          if (node.geoms && node.geoms.length > 0) {
            const w = node.width || 2.0;
            const h = node.height || 0.5;
            const L = Math.sqrt(w * w + h * h);
            const d = node.depth || 1.0;
            node.geoms[0].size = [L / 2, d / 2, 0.025];
          }
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    get().recompile(newScene);
  },
  
  updatePulleyParams: (id, params) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]) => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          if (params.leftTargetId !== undefined) node.leftTargetId = params.leftTargetId;
          if (params.rightTargetId !== undefined) node.rightTargetId = params.rightTargetId;
          if (params.pulleyRadius !== undefined) {
            node.pulleyRadius = params.pulleyRadius;
            if (node.geoms && node.geoms.length === 3) {
              node.geoms[0].size[0] = params.pulleyRadius * 0.8;
              node.geoms[1].size[0] = params.pulleyRadius;
              node.geoms[2].size[0] = params.pulleyRadius;
            } else if (node.geoms && node.geoms.length > 0) {
              node.geoms[0].size[0] = params.pulleyRadius;
            }
          }
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    get().recompile(newScene);
  },

  updateRopeParams: (id, params) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph)) as SceneGraph;
    const traverse = (nodes: any[]) => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          if (params.pulleyWheelId !== undefined) node.pulleyWheelId = params.pulleyWheelId;
          if (params.leftTargetId !== undefined) node.leftTargetId = params.leftTargetId;
          if (params.rightTargetId !== undefined) node.rightTargetId = params.rightTargetId;
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    get().recompile(newScene, get().selectedNodeId);
  },
  
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
    // Do NOT forceReset here: that would snap all other bodies back to their
    // initial positions. Without forceReset, the recompile copies old qpos/qvel
    // so every other body stays exactly where it is.
    get().recompile(newScene, undefined, false);
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
            let targetGeom = node.geoms[0];
            const mainGeom = node.geoms.find((g: any) => g.type === 'sphere' || g.type === 'box' || g.type === 'cylinder');
            if (mainGeom) {
              targetGeom = mainGeom;
            }
            if (updates.fromto && targetGeom.fromto) {
              const oldFromto = targetGeom.fromto;
              const newFromto = updates.fromto;
              const oldLen = Math.sqrt((oldFromto[3]-oldFromto[0])**2 + (oldFromto[4]-oldFromto[1])**2 + (oldFromto[5]-oldFromto[2])**2) || 1.0;
              const newLen = Math.sqrt((newFromto[3]-newFromto[0])**2 + (newFromto[4]-newFromto[1])**2 + (newFromto[5]-newFromto[2])**2) || 1.0;
              const ratio = newLen / oldLen;
              if (node.children) {
                node.children.forEach((child: any) => {
                  child.pos = [child.pos[0] * ratio, child.pos[1] * ratio, child.pos[2] * ratio];
                });
              }
            }
            Object.assign(targetGeom, updates);
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
              condim: 3
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

  updateNodeScript: (id, script) => {
    const newScene = JSON.parse(JSON.stringify(get().sceneGraph));
    const traverse = (nodes: any[]) => {
      if (!nodes) return false;
      for (const node of nodes) {
        if (node.id === id) {
          node.script = script;
          return true;
        }
        if (traverse(node.children)) return true;
      }
      return false;
    };
    traverse(newScene.nodes);
    set({ sceneGraph: newScene });
    // Note: Live updates do not force model recompiles to support hot-editing of control gains!
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
    const { sceneGraph, selectedNodeId, parentUnderSelected } = get();
    const newScene = JSON.parse(JSON.stringify(sceneGraph)) as SceneGraph;
    
    // 8-character random unique suffix (no millisecond timestamp)
    const id = `${type}_${Math.random().toString(36).substring(2, 10)}`;
    
    // Determine target local position
    let localPos: [number, number, number] = [position[0], position[1], position[2]];
    
    const isChild = !!(selectedNodeId && parentUnderSelected);
    
    if (isChild && selectedNodeId) {
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
    let geoms: any[] = [];
    
    const isChildJoint = isChild;
    
    if (type === 'gear') {
      const radius = 0.5;
      const teeth = 12;
      const color = [0.5, 0.5, 0.5, 1];
      geoms = generateGearGeoms(id, radius, teeth, color, false);
      joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 0, 1], damping: 0.5 }];
    } else if (type === 'pulley_wheel') {
      const r = 0.4;
      const thickness = 0.08;
      const spindle_r = r * 0.8;
      const spindle_h = thickness / 2 - 0.01;
      const flange_h = 0.01;
      
      geoms = [
        { name: `${id}_spindle`, type: 'cylinder', size: [spindle_r, spindle_h], pos: [0, 0, 0], euler: [90, 0, 0], rgba: [0.3, 0.4, 0.6, 1], mass: 0.5 },
        { name: `${id}_flange_l`, type: 'cylinder', size: [r, flange_h], pos: [0, -spindle_h - flange_h / 2, 0], euler: [90, 0, 0], rgba: [0.2, 0.3, 0.5, 1], mass: 0.25 },
        { name: `${id}_flange_r`, type: 'cylinder', size: [r, flange_h], pos: [0, spindle_h + flange_h / 2, 0], euler: [90, 0, 0], rgba: [0.2, 0.3, 0.5, 1], mass: 0.25 }
      ];
      joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.2 }];
    } else if (type === 'pulley_rope') {
      geoms = [];
      joints = [];
    } else {
      if (type === 'box') {
        geomType = 'box';
        size = [0.2, 0.2, 0.2];
        rgba = [0.8, 0.2, 0.2, 1];
        joints = isChildJoint ? [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }] : [{ name: `${id}_free`, type: 'free' }];
      } else if (type === 'sphere') {
        geomType = 'sphere';
        size = [0.2];
        rgba = [0.2, 0.8, 0.2, 1];
        joints = isChildJoint ? [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }] : [{ name: `${id}_free`, type: 'free' }];
      } else if (type === 'capsule') {
        geomType = 'capsule';
        size = [0.04, 0.4];
        rgba = [0.6, 0.6, 0.6, 1];
        joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.1 }];
      } else if (type === 'cylinder') {
        geomType = 'cylinder';
        size = [0.2, 0.1];
        rgba = [0.9, 0.6, 0.1, 1];
        joints = isChildJoint ? [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.5 }] : [{ name: `${id}_free`, type: 'free' }];
      } else if (type === 'bob') {
        geomType = 'sphere';
        size = [0.15];
        rgba = [0.2, 0.6, 1.0, 1];
        mass = 10.0;
        joints = [{ name: `${id}_hinge`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.1 }];
      } else if (type === 'wedge') {
        geomType = 'box';
        size = [1.0, 0.5, 0.25];
        rgba = [0.8, 0.5, 0.2, 1];
        joints = []; // static by default
      }
      geoms = [{ name: `${id}_geom`, type: geomType, size, mass, rgba }];
    }

    const newNode: SceneNode = {
      id, name: id, type: 'body', pos: localPos,
      joints,
      geoms,
      children: [],
      ...(type === 'wedge' ? {
        isWedge: true,
        width: 2.0,
        depth: 1.0,
        height: 0.5,
        wedgeAngle: 14.036
      } : {}),
      ...(type === 'pulley_wheel' ? {
        isPulleyWheel: true,
        pulleyRadius: 0.4
      } : {}),
      ...(type === 'pulley_rope' ? {
        isPulleyRope: true,
        pulleyWheelId: '',
        leftTargetId: '',
        rightTargetId: ''
      } : {})
    };

    if (isChild && selectedNodeId) {
      addChildNode(newScene.nodes, selectedNodeId, newNode);
    } else {
      newScene.nodes.push(newNode);
    }
    
    console.log("addComponent DONE, calling recompile");
    const selectId = typeof window !== 'undefined' && (window as any).NO_SELECT ? null : id;
    get().recompile(newScene, selectId);
  },
  
  recompile: async (overrideScene?: SceneGraph, overrideSelectedId?: string | null, forceReset?: boolean) => {
    console.log("recompile START");
    if (typeof window !== 'undefined') {
      (window as any).DISABLE_USEFRAME = false;
    }
    const { gravityZ, windX, windY, density, floorFriction, model: oldModel, data: oldData } = get();
    const sceneGraph = overrideScene ?? get().sceneGraph;
    
    const xml = compileToMJCF(sceneGraph, gravityZ, floorFriction, windX, windY, density);
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
          if (!nodes) return;
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
