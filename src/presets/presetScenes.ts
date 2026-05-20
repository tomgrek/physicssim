import type { SceneGraph, SceneNode, SceneGeom } from '../types/scene';

export const pendulumPreset: SceneGraph = {
  nodes: [
    {
      id: 'pole',
      name: 'pole',
      type: 'body',
      pos: [0, 0, 1.5],
      joints: [
        { name: 'hinge', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0 }
      ],
      geoms: [
        { name: 'pole_geom', type: 'capsule', fromto: [0, 0, 0, 1, 0, 0], size: [0.05], mass: 1, rgba: [0.6, 0.6, 0.6, 1] }
      ],
      children: [
        {
          id: 'bob',
          name: 'bob',
          type: 'body',
          pos: [1, 0, 0],
          joints: [
            { name: 'bob_hinge', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0 }
          ],
          geoms: [
            { name: 'bob_geom', type: 'sphere', size: [0.15], mass: 14, rgba: [0.2, 0.6, 1.0, 1] }
          ],
          children: []
        }
      ]
    }
  ]
};

export const stackedCubesPreset: SceneGraph = {
  nodes: [
    {
      id: 'cube1',
      name: 'cube1',
      type: 'body',
      pos: [0, 0, 0.3],
      joints: [
        { name: 'cube1_free', type: 'free' }
      ],
      geoms: [
        { name: 'cube1_geom', type: 'box', size: [0.2, 0.2, 0.2], mass: 1, rgba: [0.8, 0.2, 0.2, 1] }
      ],
      children: []
    },
    {
      id: 'cube2',
      name: 'cube2',
      type: 'body',
      pos: [0, 0, 1.0],
      joints: [
        { name: 'cube2_free', type: 'free' }
      ],
      geoms: [
        { name: 'cube2_geom', type: 'box', size: [0.2, 0.2, 0.2], mass: 1, rgba: [0.2, 0.8, 0.2, 1] }
      ],
      children: []
    }
  ]
};

export const generateGearGeoms = (
  id: string,
  radius: number,
  teeth: number,
  color: number[],
  isSecondGear: boolean = false,
  contype: number = 0,
  conaffinity: number = 0
): SceneGeom[] => {
  const geoms: SceneGeom[] = [];
  
  // Center cylinder
  geoms.push({
    name: `${id}_center`,
    type: 'cylinder',
    size: [radius, 0.05], // radius, half-height
    rgba: color,
    mass: 0.05,
    contype,
    conaffinity
  });

  // Teeth as single boxes radiating outward (square cogs!)
  const toothWidth = (Math.PI * radius * 2) / (teeth * 2.8);
  const toothThickness = 0.08;
  const toothLength = radius * 0.25;
  const startAngle = isSecondGear ? (Math.PI / teeth) : 0;

  for (let i = 0; i < teeth; i++) {
    const angle = startAngle + (i / teeth) * Math.PI * 2;
    const cosAngle = Math.cos(angle);
    const sinAngle = Math.sin(angle);
    
    // Position of tooth box center radiating outward
    const toothCenterX = cosAngle * (radius + toothLength / 2);
    const toothCenterY = sinAngle * (radius + toothLength / 2);
    
    // Box dimensions: size = [half_length_outward, half_width_tangent, half_thickness_vertical]
    const size = [toothLength / 2, toothWidth / 2, toothThickness / 2];
    
    // Pure Z-rotation quaternion so teeth point outward and remain perfectly flat
    const halfAngle = angle / 2;
    const quat = [Math.cos(halfAngle), 0, 0, Math.sin(halfAngle)];
    
    geoms.push({
      name: `${id}_tooth_${i}`,
      type: 'box',
      size,
      pos: [toothCenterX, toothCenterY, 0],
      quat,
      rgba: color,
      mass: 0.01,
      contype,
      conaffinity
    });
  }

  return geoms;
};

const createGear = (id: string, name: string, pos: number[], radius: number, teeth: number, color: number[], isMotor: boolean, isSecondGear: boolean = false): SceneNode => {
  return {
    id,
    name,
    type: 'body',
    pos,
    joints: [
      { 
        name: `${id}_hinge`, 
        type: 'hinge', 
        axis: [0, 0, 1], // Z-axis hinge
        damping: 0.5,
        ...(isMotor && { actuator: { type: 'velocity', kv: 20, ctrlValue: 1.5 } })
      }
    ],
    geoms: generateGearGeoms(id, radius, teeth, color, isSecondGear),
    children: []
  };
};

export const gearsPreset: SceneGraph = {
  nodes: [
    // Two meshing gears.
    // If Gear 1 is at (0,0,0) and radius 0.5, outer radius is 0.625.
    // We use D = 1.13 to provide a tiny clearance gap and guarantee start-up stability.
    // Placed at Z = 0.05 so they lie exactly flat on the plane
    createGear('gear1', 'gear1', [0, 0, 0.05], 0.5, 12, [0.8, 0.4, 0.2, 1], true, false),
    createGear('gear2', 'gear2', [1.13, 0, 0.05], 0.5, 12, [0.2, 0.6, 0.8, 1], false, true)
  ]
};

const machineGear1 = createGear('gear1', 'gear1', [0, 0, 0.05], 0.3, 12, [0.8, 0.4, 0.2, 1], true, false);
machineGear1.geoms.push({
  name: 'gear1_shaft',
  type: 'cylinder',
  size: [0.05, 0.12], // radius 0.05, half-height 0.12
  pos: [0, 0, 0.12],
  rgba: [0.45, 0.45, 0.45, 1],
  mass: 0.1,
  contype: 0,
  conaffinity: 0
});

const machineGear2 = createGear('gear2', 'gear2', [0.72, 0, 0.05], 0.4, 16, [0.2, 0.6, 0.8, 1], false, true);

const machineGear3 = createGear('gear3', 'gear3', [0.72, 0.82, 0.05], 0.4, 16, [0.2, 0.8, 0.4, 1], false, false);
machineGear3.geoms.push({
  name: 'gear3_pusher_peg',
  type: 'cylinder',
  size: [0.03, 0.08], // radius 0.03, half-height 0.08
  pos: [0.52, 0.0, 0.14], // relative offset placed outside the gear disc (0.52 > 0.40) and high enough to clear the shelf top
  rgba: [0.9, 0.2, 0.2, 1],
  mass: 0.05,
  condim: 3,
  solref: [0.015, 1.0],
  solimp: [0.95, 0.99, 0.001, 0.5, 2]
});

const machineShelf: SceneNode = {
  id: 'shelf',
  name: 'shelf',
  type: 'body',
  pos: [1.35, 0.82, 0.05],
  joints: [],
  geoms: [
    {
      name: 'shelf_geom',
      type: 'box',
      size: [0.12, 0.12, 0.05], // sits at Z=0.05, top is at 0.10, covers X from 1.23 to 1.47
      rgba: [0.7, 0.7, 0.7, 1],
      mass: 10,
      condim: 3,
      friction: [0.1, 0.005, 0.0005]
    }
  ],
  children: []
};

const machineBlock: SceneNode = {
  id: 'push_block',
  name: 'push_block',
  type: 'body',
  pos: [1.26, 0.82, 0.16], // sits perfectly on shelf, X is 1.26, in the clean sweep path of the pusher peg
  joints: [
    { name: 'push_block_free', type: 'free' }
  ],
  geoms: [
    {
      name: 'push_block_geom',
      type: 'box',
      size: [0.05, 0.05, 0.05], // 10cm cube
      rgba: [0.95, 0.8, 0.25, 1],
      mass: 0.1,
      condim: 3,
      friction: [0.1, 0.005, 0.0005],
      solref: [0.015, 1.0],
      solimp: [0.95, 0.99, 0.001, 0.5, 2]
    }
  ],
  children: []
};

export const machinePreset: SceneGraph = {
  nodes: [
    machineGear1,
    machineGear2,
    machineGear3,
    machineShelf,
    machineBlock
  ]
};

// Generate visual cogs matching pinion pitch spaced along X as child SceneNodes
const pinionPitch = (Math.PI * 2 * 0.20) / 8; // 0.157079
const rackTeethChildren: SceneNode[] = [];
let toothIndex = 0;
// Space teeth along a 0.70m long rack base (from -0.30 to 0.30)
for (let x = -0.30; x <= 0.30; x += pinionPitch) {
  rackTeethChildren.push({
    id: `rack_tooth_${toothIndex}`,
    name: `tooth_${toothIndex}`,
    type: 'body',
    pos: [x, 0.045, 0],
    joints: [], // Fixed / welded to parent!
    geoms: [
      {
        name: `rack_tooth_${toothIndex}_geom`,
        type: 'box',
        size: [0.01, 0.015, 0.02],
        rgba: [0.7, 0.7, 0.7, 1],
        mass: 0.01
        // Full physical collision enabled (no contype/conaffinity overrides)
      }
    ],
    children: []
  });
  toothIndex++;
}

// Rack body node with X slide joint limited to [-0.6, 0.6] range
const rackNode: SceneNode = {
  id: 'rack',
  name: 'rack',
  type: 'body',
  pos: [0, -0.25, 0.05],
  joints: [
    { name: 'rack_slide', type: 'slide', axis: [1, 0, 0], damping: 0.5, limited: true, range: [-0.6, 0.6] }
  ],
  geoms: [
    {
      name: 'rack_base',
      type: 'box',
      size: [0.35, 0.04, 0.04], // 0.70m long rack base
      rgba: [0.8, 0.8, 0.8, 1],
      mass: 0.5
      // Full physical collision enabled (no contype/conaffinity overrides)
    }
  ],
  children: rackTeethChildren
};

// Pinion gear with velocity motor
const pinionNode = createGear('pinion', 'pinion', [0, 0, 0.05], 0.2, 8, [0.2, 0.6, 0.8, 1], true);

// Shelf and block at the right end of the rack path, perfectly aligned in Y (-0.25)
const rackShelf: SceneNode = {
  id: 'rack_shelf',
  name: 'rack_shelf',
  type: 'body',
  pos: [0.90, -0.25, 0.05], // Aligned in Y with the rack path
  joints: [],
  geoms: [
    {
      name: 'rack_shelf_geom',
      type: 'box',
      size: [0.10, 0.10, 0.05],
      rgba: [0.65, 0.65, 0.65, 1],
      mass: 10,
      condim: 3,
      friction: [0.1, 0.005, 0.0005]
    }
  ],
  children: []
};

const rackBlock: SceneNode = {
  id: 'rack_block',
  name: 'rack_block',
  type: 'body',
  pos: [0.76, -0.25, 0.15], // Resting perfectly on the left edge of the shelf
  joints: [
    { name: 'rack_block_free', type: 'free' }
  ],
  geoms: [
    {
      name: 'rack_block_geom',
      type: 'box',
      size: [0.05, 0.05, 0.05],
      rgba: [0.95, 0.8, 0.25, 1],
      mass: 0.1,
      condim: 3,
      friction: [0.1, 0.005, 0.0005],
      solref: [0.015, 1.0],
      solimp: [0.95, 0.99, 0.001, 0.5, 2]
    }
  ],
  children: []
};

export const rackPinionPreset: SceneGraph = {
  nodes: [
    pinionNode,
    rackNode,
    rackShelf,
    rackBlock
  ]
};

export const PRESETS = {
  pendulum: {
    name: 'Double Pendulum',
    scene: pendulumPreset
  },
  cubes: {
    name: 'Stacked Cubes',
    scene: stackedCubesPreset
  },
  gears: {
    name: 'Gear System',
    scene: gearsPreset
  },
  machine: {
    name: 'Gear Train Machine',
    scene: machinePreset
  },
  rack_pinion: {
    name: 'Rack and Pinion Converter',
    scene: rackPinionPreset
  }
};
