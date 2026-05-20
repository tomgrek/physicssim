
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import { useMuJoCoInit } from './hooks/useMuJoCo';
import { useStore } from './store/useStore';
import { Play, Square, Settings2, SlidersHorizontal, Settings, Box, Circle, X, RotateCcw, Eye, Trash2, Layers, CircleDot, Zap, Info } from 'lucide-react';
import { useRef, useMemo, useEffect, useCallback, useState } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';

// Physics Step Hook
const PhysicsLoop = ({ model, data, mujoco, isPlaying }: { model: any, data: any, mujoco: any, isPlaying: boolean }) => {

  const accumulator = useRef<number>(0);
  
  useFrame((_state, delta) => {
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
        mujoco.mj_step(model, data);
        
        // Safety check for NaN values in positions
        const nq = model.nq;
        for (let j = 0; j < nq; j++) {
          if (isNaN(data.qpos[j])) {
            console.error(`[PhysicsLoop] NaN detected in qpos at index ${j}! Stopping simulation.`);
            (window as any).DISABLE_USEFRAME = true;
            // Force pause in state
            useStore.getState().togglePlay();
            return;
          }
        }
      } catch (e) {
        console.error("Simulation step error:", e);
        useStore.getState().togglePlay();
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

  return <OrbitControls ref={controlsRef} makeDefault enableDamping dampingFactor={0.1} mouseButtons={{ LEFT: 99 as any, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE }} />;
};

// Drop Handler for precise spawning
const DropHandler = ({ addComponent }: { addComponent: (type: 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear', pos: [number, number, number]) => void }) => {
  const { camera } = useThree();
  
  useEffect(() => {
    const handler = (e: DragEvent) => {
      e.preventDefault();
      const type = e.dataTransfer?.getData('type') as 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear';
      if (!type) return;
      
      const vec = new THREE.Vector3(
        (e.clientX / window.innerWidth) * 2 - 1,
        -(e.clientY / window.innerHeight) * 2 + 1,
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


// Dynamic Geom Renderer
const DynamicGeom = ({ nodeId, name, type, color, mujoco, model, data, selectedNodeId, setSelectedNodeId }: any) => {
  const meshRef = useRef<THREE.Group>(null);
  
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
      return [r];
    } catch (e) {
      console.error(`[DynamicGeom ${name}] geometryArgs Error:`, e);
      return [];
    }
  }, [geomId, type, model]);

  const rotationMatrix = useMemo(() => new THREE.Matrix4(), []);
  const isSelected = selectedNodeId === nodeId;

  // Compute initial position and rotation from the model/data
  const [initialPos, initialQuat] = useMemo(() => {
    if (geomId === -1 || !model || !data) return [[0, 0, 0] as [number, number, number], [0, 0, 0, 1] as [number, number, number, number]];
    try {
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
  }, [geomId, model, data]);

  useFrame(() => {
    // Safety check: ensure closure model/data match current store active ones
    const activeModel = useStore.getState().model;
    const activeData = useStore.getState().data;
    if (model !== activeModel || data !== activeData) return;

    if ((window as any).DISABLE_USEFRAME) return;
    if (!meshRef.current || geomId === -1 || !model || !data) return;
    
    try {
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

  if (geomId === -1 || !geometryArgs || geometryArgs.length === 0 || geometryArgs.some(arg => arg === undefined || isNaN(arg))) {
    return null;
  }

  return (
    <group 
      ref={meshRef}
      position={initialPos}
      quaternion={new THREE.Quaternion(...initialQuat)}
      onClick={(e) => {
        e.stopPropagation();
        setSelectedNodeId(nodeId);
      }}
    >
      {type === 'sphere' && (
        <mesh castShadow receiveShadow>
          <sphereGeometry args={geometryArgs as any} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      )}
      {type === 'box' && (
        <mesh castShadow receiveShadow>
          <boxGeometry args={geometryArgs as any} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      )}
      {type === 'capsule' && (
        <mesh castShadow receiveShadow rotation={[Math.PI / 2, 0, 0]}>
          <capsuleGeometry args={geometryArgs as any} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      )}
      {type === 'cylinder' && (
        <mesh castShadow receiveShadow rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[geometryArgs[0], geometryArgs[0], geometryArgs[1] * 2, 32]} />
          <meshStandardMaterial color={new THREE.Color(color[0], color[1], color[2])} emissive={isSelected ? '#3b82f6' : '#000'} emissiveIntensity={isSelected ? 0.2 : 0} />
        </mesh>
      )}
    </group>
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
  
  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      {geoms.map(g => (
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
    </group>
  );
};

const CAMERA_CONFIG = { position: [5, 2, 5] as [number, number, number], fov: 50 };

function App() {
  if (typeof window !== 'undefined') {
    (window as any).useStore = useStore;
  }
  useMuJoCoInit();
  const [isDocsOpen, setIsDocsOpen] = useState(false);
  const [docsTab, setDocsTab] = useState<'gravity' | 'coupling' | 'collision' | 'friction'>('gravity');
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
    addPusherPeg, deletePusherPeg, updatePusherPeg, updateNodeRotation
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

  // Utility to handle moving free bodies
  const handleMove = (axis: 0 | 1 | 2, val: number) => {
    if (!selectedNode) return;
    const cleanVal = isNaN(val) ? 0 : val;
    
    if (isPlaying) {
      if (!model || !mujoco || !data) return;
      // Mutate active physics WASM memory directly
      const jointId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, `${selectedNode.id}_free`);
      if (jointId !== -1) {
        const adr = model.jnt_qposadr[jointId];
        data.qpos[adr + axis] = cleanVal;
        // Zero out velocities to prevent crazy snaps
        const vadr = model.jnt_dofadr[jointId];
        for (let i = 0; i < 6; i++) data.qvel[vadr + i] = 0;
        // Force propagation to update positions in 3D visually
        mujoco.mj_forward(model, data);
      }
    } else {
      // Edit initial position in the scene graph (Zustand store) when paused
      const currentPos = [...selectedNode.pos] as [number, number, number];
      currentPos[axis] = cleanVal;
      updateNodePos(selectedNode.id, currentPos);
    }
  };

  const handleAddComponentClick = (type: 'box' | 'sphere' | 'capsule' | 'cylinder' | 'bob' | 'gear') => {
    if (selectedNodeId) {
      const parentNode = findNodeById(sceneGraph.nodes, selectedNodeId);
      if (parentNode) {
        const worldPos = getNodeWorldPos(sceneGraph.nodes, selectedNodeId) || [0, 0, 0];
        const offset = (type === 'capsule' || type === 'bob') ? [0, 0, -0.6] : [0.5, 0, 0];
        addComponent(type, [worldPos[0] + offset[0], worldPos[1] + offset[1], worldPos[2] + offset[2]]);
        return;
      }
    }
    addComponent(type, [0, 0, 1.2]); // Spawn slightly above floor
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

    return (
      <div key={node.id} className="flex flex-col">
        <div 
          onClick={() => setSelectedNodeId(node.id)} 
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
  }, [selectedNodeId, setSelectedNodeId, findNodeById]);

  return (
    <div className="flex flex-col h-screen w-screen bg-slate-50 text-slate-900 font-sans">
      <header className="glass-panel h-14 flex items-center justify-between px-6 z-10 border-b border-slate-200">
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 rounded-lg bg-blue-500/10 flex items-center justify-center">
            <Settings2 className="w-5 h-5 physics-accent" />
          </div>
          <h1 className="font-bold text-lg tracking-wide">
            Physics <span className="text-blue-500">Expt</span>
          </h1>
        </div>
        
        <div className="flex items-center gap-4">
          {/* Presets Select */}
          <div className="flex items-center gap-2">
            <span className="text-xs font-semibold text-slate-500 uppercase tracking-wider">Preset:</span>
            <select 
              onChange={(e) => loadPreset(e.target.value as any)}
              className="px-3 py-1.5 rounded-full bg-white hover:bg-slate-100 transition-colors border border-slate-200 text-sm font-medium text-slate-700 shadow-sm outline-none cursor-pointer focus:border-blue-500"
              defaultValue="pendulum"
            >
              <option value="pendulum">Double Pendulum</option>
              <option value="cubes">Stacked Cubes</option>
              <option value="gears">Gear System</option>
              <option value="machine">Gear Train Machine</option>
              <option value="rack_pinion">Rack and Pinion Converter</option>
            </select>
          </div>

          <button
            onClick={() => setIsDocsOpen(true)}
            className="flex items-center justify-center w-8 h-8 rounded-full border-2 border-blue-200 text-blue-600 hover:bg-blue-50 hover:border-blue-300 transition-colors focus:outline-none flex-shrink-0 cursor-pointer"
            title="Documentation"
          >
            <Info className="w-4 h-4" />
          </button>

          <button 
            onClick={() => setSettingsOpen(!isSettingsOpen)}
            className={`flex items-center justify-center w-8 h-8 rounded-full border-2 transition-colors focus:outline-none flex-shrink-0 cursor-pointer ${
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
            className="flex items-center justify-center gap-2 w-28 py-1.5 rounded-full bg-white hover:bg-slate-100 transition-colors border border-slate-200 disabled:opacity-50 shadow-sm"
          >
            {isPlaying ? (
              <><Square className="w-4 h-4 text-red-500" /> Stop</>
            ) : (
              <><Play className="w-4 h-4 text-emerald-500" /> Simulate</>
            )}
          </button>

          <button 
            onClick={resetSimulation}
            disabled={!isLoaded}
            className="flex items-center gap-2 px-4 py-1.5 rounded-full bg-white hover:bg-slate-100 transition-colors border border-slate-200 disabled:opacity-50 shadow-sm text-slate-600 hover:text-slate-900"
            title="Reset Simulation"
          >
            <RotateCcw className="w-4 h-4" /> Reset
          </button>

          <button 
            onClick={() => setCameraView(cameraView === 'topDown' ? 'perspective' : 'topDown')}
            className={`flex items-center gap-2 px-4 py-1.5 rounded-full transition-colors border shadow-sm ${cameraView === 'topDown' ? 'bg-blue-100 text-blue-700 border-blue-200' : 'bg-white hover:bg-slate-100 border-slate-200 text-slate-600'}`}
            title="Toggle Top Down View"
          >
            <Eye className="w-4 h-4" /> {cameraView === 'topDown' ? 'Perspective' : 'Top Down'}
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

        {/* Left Sidebar */}
        <aside className="w-56 shrink-0 glass-panel border-r border-slate-200 flex flex-col p-4 z-10 bg-white/50 overflow-y-auto">
          <h2 className="text-xs font-semibold text-slate-400 uppercase tracking-wider mb-3">Hierarchy</h2>
          <div className="flex flex-col gap-1.5 mb-6">
            <div 
              className={`px-3 py-1.5 rounded-md border cursor-pointer transition-colors shadow-sm flex items-center gap-1.5 ${!selectedNodeId ? 'bg-blue-50 border-blue-200 text-blue-600 font-bold' : 'bg-white border-transparent hover:bg-slate-50 text-slate-600'}`}
              onClick={() => setSelectedNodeId(null)}
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
          <div className="text-[11px] font-medium text-slate-500 mb-3 bg-slate-100/80 px-2 py-1.5 rounded-lg border border-slate-200/50">
            Adding to: <span className="text-blue-600 font-semibold">{selectedNode ? selectedNode.name : '🌍 Worldbody'}</span>
          </div>

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
              onDragStart={(e) => handleDragStart(e, 'bob')} 
              onClick={() => handleAddComponentClick('bob')}
              className="p-2.5 border border-slate-200 rounded-lg bg-white shadow-sm flex items-center gap-3 cursor-pointer hover:border-blue-300 hover:shadow transition-all group"
            >
              <CircleDot className="w-4 h-4 text-blue-500 group-hover:scale-110 transition-transform" />
              <div className="flex flex-col">
                <span className="text-xs font-semibold text-slate-700">Bob</span>
                <span className="text-[10px] text-slate-400">Heavy pendulum weight</span>
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
            
            <CameraController />
          </Canvas>
        </main>

        {/* Contextual Properties Sidebar */}
        {selectedNode && (
          <aside className="w-64 shrink-0 glass-panel border-l border-slate-200 flex flex-col p-4 z-10 bg-white/50 overflow-y-auto">
            <h2 className="text-xs font-semibold text-slate-400 uppercase tracking-wider mb-4 flex items-center justify-between">
              <span className="flex items-center gap-2"><SlidersHorizontal className="w-4 h-4" /> Properties</span>
              <button onClick={() => setSelectedNodeId(null)}><X className="w-4 h-4 text-slate-400 hover:text-slate-600" /></button>
            </h2>
            
            <div className="flex flex-col gap-4">
              <div className="flex flex-col gap-1 p-3 bg-slate-50 border border-slate-200/60 rounded-lg">
                <label className="text-[10px] font-semibold text-slate-400 uppercase tracking-wider">Component Name</label>
                <input 
                  type="text" 
                  value={selectedNode.name || ''} 
                  onChange={(e) => renameNode(selectedNode.id, e.target.value)}
                  className="w-full px-2.5 py-1.5 border border-slate-200 rounded text-sm bg-white font-medium text-slate-800 outline-none focus:border-blue-500 shadow-sm"
                  placeholder="Rename component..."
                />
                <span className="text-[9px] font-mono text-slate-400 mt-1">ID: {selectedNode.id}</span>
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
                </div>
              )}

              {/* Position Coordinates (Applicable to all nodes!) */}
              <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2">Position Offset</h3>
                <div className="flex flex-col gap-2">
                  <label className="text-xs text-slate-500 flex items-center justify-between">X Position
                    <input 
                      type="number" 
                      step="0.1" 
                      className="w-20 px-1.5 py-1 border border-slate-200 rounded text-slate-700 font-medium text-sm outline-none focus:border-blue-500" 
                      value={selectedNode.pos[0]} 
                      onChange={(e) => handleMove(0, parseFloat(e.target.value))} 
                    />
                  </label>
                  <label className="text-xs text-slate-500 flex items-center justify-between">Y Position
                    <input 
                      type="number" 
                      step="0.1" 
                      className="w-20 px-1.5 py-1 border border-slate-200 rounded text-slate-700 font-medium text-sm outline-none focus:border-blue-500" 
                      value={selectedNode.pos[1]} 
                      onChange={(e) => handleMove(1, parseFloat(e.target.value))} 
                    />
                  </label>
                  <label className="text-xs text-slate-500 flex items-center justify-between">Z Position
                    <input 
                      type="number" 
                      step="0.1" 
                      className="w-20 px-1.5 py-1 border border-slate-200 rounded text-slate-700 font-medium text-sm outline-none focus:border-blue-500" 
                      value={selectedNode.pos[2]} 
                      onChange={(e) => handleMove(2, parseFloat(e.target.value))} 
                    />
                  </label>
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
                        min="0.15" 
                        max="2.0" 
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
                          min="0.05" 
                          max={gearRadius * 1.5} 
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
                          min="0.01" 
                          max="0.10" 
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
                          min="0.02" 
                          max="0.30" 
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

              {/* Damping and Actuator Target Speed properties */}
              {selectedNode.joints?.map((joint: any, i: number) => (
                <div key={`joint-${i}`} className="flex flex-col gap-4">
                  {joint.damping !== undefined && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">🔗 Joint Damping</h3>
                      <label className="text-xs font-medium text-slate-500 flex justify-between">Damping <span>{joint.damping}</span></label>
                      <input type="range" min="0" max="100" step="0.1" value={joint.damping} onChange={(e) => updateNodeJoint(selectedNode.id, {damping: parseFloat(e.target.value)})} className="w-full accent-blue-500 cursor-pointer" />
                    </div>
                  )}
                  {joint.actuator && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">⚡ Target Velocity</h3>
                      <label className="text-xs font-medium text-slate-500 flex justify-between">
                        Control Speed
                        <span>{joint.actuator.ctrlValue || 0}</span>
                      </label>
                      <input 
                        type="range" 
                        min="-20" 
                        max="20" 
                        step="0.5" 
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
                  )}
                </div>
              ))}

              {/* Dimensions Resizing and Color Properties */}
              {selectedNode.geoms?.slice(0, 1).map((geom: any, i: number) => (
                <div key={`geom-${i}`} className="flex flex-col gap-4">
                  {!selectedNode.id.includes('gear') && (
                    <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-3">
                      <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2">📏 Resize Component</h3>
                      
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

                      {geom.type === 'box' && (
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

                      {(geom.type === 'capsule' || geom.type === 'cylinder') && (
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
                                updateNodeGeom(selectedNode.id, { size: [val, geom.size[1]] });
                              }}
                              className="w-full accent-blue-500 cursor-pointer" 
                            />
                          </div>
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
                        </div>
                      )}
                    </div>
                  )}

                  <div className="p-3 bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col gap-2">
                    <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1">Mass</h3>
                    <label className="text-xs font-medium text-slate-500 flex justify-between">Value <span>{geom.mass} kg</span></label>
                    <input type="range" min="0.1" max="50" step="0.1" value={geom.mass} onChange={(e) => updateNodeGeom(selectedNode.id, {mass: parseFloat(e.target.value)})} className="w-full accent-blue-500 cursor-pointer" />
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
                      Enable Collision (Solid)
                    </label>
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
                    <h3 className="text-sm font-medium text-slate-700 border-b border-slate-100 pb-2 mb-1 flex items-center justify-between">
                      <span className="flex items-center gap-1">Friction</span>
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
                    <label className="text-xs font-medium text-slate-500 flex justify-between">Slide Friction <span>{(geom.friction !== undefined ? geom.friction[0] : 1.0).toFixed(2)}</span></label>
                    <input 
                      type="range" 
                      min="0.0" 
                      max="2.0" 
                      step="0.01" 
                      value={geom.friction !== undefined ? geom.friction[0] : 1.0} 
                      onChange={(e) => {
                        const val = parseFloat(e.target.value);
                        updateNodeGeom(selectedNode.id, {
                          friction: [val, 0.005, 0.0001]
                        });
                      }} 
                      className="w-full accent-blue-500 cursor-pointer" 
                    />
                  </div>

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
              ))}

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
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default App;
