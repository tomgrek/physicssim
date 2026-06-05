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

            // Inject initial velocities (including launch-time random spin on free joints)
            const initVelJoints: { name: string; vel: number[]; type?: string }[] = [];
            const traverseVel = (nodes: any[]) => {
              if (!nodes) return;
              for (const node of nodes) {
                node.joints?.forEach((j: any) => { if (j.initialVelocity) initVelJoints.push({ name: j.name, vel: j.initialVelocity, type: j.type }); });
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
                  let val = j.vel[i];
                  if (j.type === 'free' && i >= 3) {
                    val += (Math.random() - 0.5) * (Math.abs(val) * 0.1 + 0.5);
                  }
                  headlessData.qvel[dofAdr + i] = val;
                }
                needForward = true;
              }
            }
            if (needForward) {
              mujoco.mj_forward(headlessModel, headlessData);
            }
            
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
          store.setEnvironment({ gravityZ, windX, windY, density, floorFriction });
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
