import type { SceneGraph, SceneNode, SceneGeom, SceneJoint } from '../types/scene';

const buildGeom = (geom: SceneGeom) => {
  let attrs = `name="${geom.name}" type="${geom.type}" size="${geom.size.join(' ')}"`;
  if (geom.rgba) attrs += ` rgba="${geom.rgba.join(' ')}"`;
  if (geom.fromto) attrs += ` fromto="${geom.fromto.join(' ')}"`;
  if (geom.pos) attrs += ` pos="${geom.pos.join(' ')}"`;
  if (geom.quat) {
    attrs += ` quat="${geom.quat.join(' ')}"`;
  } else if (geom.euler) {
    attrs += ` euler="${geom.euler.join(' ')}"`;
  }
  if (geom.mass !== undefined) attrs += ` mass="${geom.mass}"`;
  if (geom.contype !== undefined) attrs += ` contype="${geom.contype}"`;
  if (geom.conaffinity !== undefined) attrs += ` conaffinity="${geom.conaffinity}"`;
  if (geom.condim !== undefined) attrs += ` condim="${geom.condim}"`;
  if (geom.friction !== undefined) attrs += ` friction="${geom.friction.join(' ')}"`;
  if (geom.solref) attrs += ` solref="${geom.solref.join(' ')}"`;
  if (geom.solimp) attrs += ` solimp="${geom.solimp.join(' ')}"`;
  return `<geom ${attrs} />`;
};

const buildJoint = (joint: SceneJoint) => {
  if (joint.type === 'free') {
    return `<freejoint name="${joint.name}" />`;
  }
  let attrs = `name="${joint.name}" type="${joint.type}"`;
  if (joint.pos) attrs += ` pos="${joint.pos.join(' ')}"`;
  if (joint.axis) attrs += ` axis="${joint.axis.join(' ')}"`;
  if (joint.damping !== undefined) attrs += ` damping="${joint.damping}"`;
  if (joint.limited !== undefined) attrs += ` limited="${joint.limited}"`;
  if (joint.range !== undefined) attrs += ` range="${joint.range.join(' ')}"`;
  return `<joint ${attrs} />`;
};

const buildNode = (node: SceneNode): string => {
  let innerXml = '';
  
  node.joints.forEach(j => innerXml += buildJoint(j));
  node.geoms.forEach(g => innerXml += buildGeom(g));
  node.children.forEach(c => innerXml += buildNode(c));
  
  let attrs = `name="${node.name}" pos="${node.pos.join(' ')}"`;
  if (node.quat) {
    attrs += ` quat="${node.quat.join(' ')}"`;
  } else if (node.euler) {
    attrs += ` euler="${node.euler.join(' ')}"`;
  }
  
  return `<body ${attrs}>${innerXml}</body>`;
};

export const compileToMJCF = (scene: SceneGraph, gravityZ: number = -9.81, floorFriction: number = 1.0) => {
  const actuators: SceneJoint[] = [];
  let pinionJoint: string | null = null;
  let rackJoint: string | null = null;

  // First, traverse and compute absolute world positions and sizes for all bodies
  const nodeWorldData: { [id: string]: { pos: [number, number, number]; size: number[] } } = {};

  const getAbsolutePositions = (nodes: SceneNode[], parentPos: [number, number, number] = [0, 0, 0]) => {
    for (const node of nodes) {
      const worldPos: [number, number, number] = [
        parentPos[0] + node.pos[0],
        parentPos[1] + node.pos[1],
        parentPos[2] + node.pos[2]
      ];
      const mainGeom = node.geoms[0];
      const size = mainGeom ? mainGeom.size : [0.1, 0.1, 0.1];
      nodeWorldData[node.id] = { pos: worldPos, size };
      getAbsolutePositions(node.children, worldPos);
    }
  };
  getAbsolutePositions(scene.nodes);

  // Keep track of gear joints along with their absolute position and size
  const gearJointNodes: { jointName: string; nodeId: string; pos: [number, number, number]; radius: number }[] = [];
  const jointedNodes: Record<string, string> = {};
  const explicitCouplingNodes: SceneNode[] = [];

  const traverse = (nodes: SceneNode[]) => {
    for (const node of nodes) {
      if (node.joints && node.joints.length > 0) {
        jointedNodes[node.id] = node.joints[0].name;
      }
      if (node.coupleTargetId) {
        explicitCouplingNodes.push(node);
      }

      const couplingAllowed = node.allowCoupling !== false;
      node.joints.forEach(j => { 
        if (j.actuator) actuators.push(j); 
        if (couplingAllowed && node.id.includes('gear') && j.type === 'hinge') {
          const wData = nodeWorldData[node.id];
          gearJointNodes.push({
            jointName: j.name,
            nodeId: node.id,
            pos: wData ? wData.pos : [0, 0, 0],
            radius: wData ? wData.size[0] : 0.25
          });
        }
        if (couplingAllowed && node.id.includes('pinion') && j.type === 'hinge') {
          pinionJoint = j.name;
        }
        if (couplingAllowed && node.id.includes('rack') && j.type === 'slide') {
          rackJoint = j.name;
        }
      });
      traverse(node.children);
    }
  };
  traverse(scene.nodes);

  const actuatorsXml = actuators.map((j) => {
    const act = j.actuator!;
    let attrs = `name="${j.name}_actuator" joint="${j.name}"`;
    if (act.kv !== undefined) attrs += ` kv="${act.kv}"`;
    if (act.gear !== undefined) attrs += ` gear="${act.gear}"`;
    return `    <${act.type} ${attrs} />`;
  }).join('\n');

  let equalityXml = '';
  let equalityConstraints = '';

  // 0. Explicit joint couplings
  let couplingIndex = 1;
  const explicitlyCoupledJointPairs = new Set<string>();

  for (const node of explicitCouplingNodes) {
    const joint1 = jointedNodes[node.id];
    const joint2 = jointedNodes[node.coupleTargetId!];
    if (joint1 && joint2) {
      const ratio = node.coupleRatio !== undefined ? node.coupleRatio : -1.0;
      const pairKey = [joint1, joint2].sort().join('::');
      explicitlyCoupledJointPairs.add(pairKey);
      equalityConstraints += `\n    <joint name="explicit_coupling_${couplingIndex++}" joint1="${joint1}" joint2="${joint2}" polycoef="0 ${ratio} 0 0 0" />`;
    }
  }

  // 1. Gears fallback dynamic proximity coupling
  const coupledPairs = new Set<string>();
  for (let i = 0; i < gearJointNodes.length; i++) {
    for (let j = i + 1; j < gearJointNodes.length; j++) {
      const g1 = gearJointNodes[i];
      const g2 = gearJointNodes[j];
      const dx = g1.pos[0] - g2.pos[0];
      const dy = g1.pos[1] - g2.pos[1];
      const dz = g1.pos[2] - g2.pos[2];
      const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
      // Touch/meshing criteria: centers within sum of radii + 15% margin
      const maxMeshingDist = (g1.radius + g2.radius) * 1.15;
      if (dist <= maxMeshingDist) {
        const pairKey = [g1.jointName, g2.jointName].sort().join('::');
        if (!coupledPairs.has(pairKey) && !explicitlyCoupledJointPairs.has(pairKey)) {
          coupledPairs.add(pairKey);
          equalityConstraints += `\n    <joint name="gear_coupling_${coupledPairs.size}" joint1="${g1.jointName}" joint2="${g2.jointName}" polycoef="0 -1 0 0 0" />`;
        }
      }
    }
  }

  // 2. Pinion and Rack fallback proximity coupling
  if (pinionJoint && rackJoint) {
    const pinionData = Object.entries(nodeWorldData).find(([id]) => id.includes('pinion'));
    const rackData = Object.entries(nodeWorldData).find(([id]) => id.includes('rack'));
    if (pinionData && rackData) {
      const [, pInfo] = pinionData;
      const [, rInfo] = rackData;
      const dx = pInfo.pos[0] - rInfo.pos[0];
      const dy = pInfo.pos[1] - rInfo.pos[1];
      const dz = pInfo.pos[2] - rInfo.pos[2];
      const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
      // Only couple if their centers are within 0.5 meters of each other
      if (dist <= 0.5) {
        const pairKey = [pinionJoint, rackJoint].sort().join('::');
        if (!explicitlyCoupledJointPairs.has(pairKey)) {
          equalityConstraints += `\n    <joint name="pinion_rack_coupling" joint1="${pinionJoint}" joint2="${rackJoint}" polycoef="0 0.2 0 0 0" />`;
        }
      }
    }
  }

  if (equalityConstraints) {
    equalityXml = `\n  <equality>${equalityConstraints}\n  </equality>`;
  }

  return `
<mujoco model="dynamic_scene">
  <option timestep="0.001" gravity="0 0 ${gravityZ}" />
  <worldbody>
    <light directional="true" pos="-0.5 0.5 3" dir="0.5 -0.5 -3" diffuse="0.8 0.8 0.8" />
    <geom name="floor" type="plane" size="0 0 0.1" pos="0 0 0" rgba="0.9 0.9 0.9 1" friction="${floorFriction} 0.005 0.0001" />
    
    ${scene.nodes.map(buildNode).join('\n')}
  </worldbody>
${actuators.length > 0 ? `  <actuator>\n${actuatorsXml}\n  </actuator>` : ''}${equalityXml}
</mujoco>
  `.trim();
};
