import type { SceneGraph, SceneNode, SceneGeom } from '../types/scene';

export const pendulumPreset: SceneGraph = {
  nodes: [
    {
      id: 'pole',
      name: 'pole',
      type: 'body',
      pos: [0, 0, 3.0],
      joints: [
        { name: 'hinge', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0 }
      ],
      geoms: [
        { name: 'pole_geom', type: 'capsule', fromto: [0, 0, 0, 1, 0, 0], size: [0.05], mass: 1, rgba: [0.6, 0.6, 0.6, 1] },
        { name: 'pole_bob_geom', type: 'sphere', size: [0.12], pos: [1, 0, 0], mass: 7, rgba: [0.3, 0.5, 0.8, 1] }
      ],
      children: [
        {
          id: 'pole2',
          name: 'pole2',
          type: 'body',
          pos: [1, 0, 0],
          joints: [
            { name: 'hinge2', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0 }
          ],
          geoms: [
            { name: 'pole2_geom', type: 'capsule', fromto: [0, 0, 0, 1, 0, 0], size: [0.05], mass: 1, rgba: [0.5, 0.5, 0.5, 1] },
            { name: 'bob_geom', type: 'sphere', size: [0.15], pos: [1, 0, 0], mass: 14, rgba: [0.2, 0.6, 1.0, 1] }
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
  condim: 3
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
      friction: [0.1, 0.005, 0.0005]
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
      friction: [0.1, 0.005, 0.0005]
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

export const inclinedPlanePreset: SceneGraph = {
  nodes: [
    {
      id: 'inclined_wedge',
      name: 'inclined_wedge',
      type: 'body',
      pos: [0, 0, 0],
      isWedge: true,
      width: 3.0,
      depth: 1.2,
      height: 1.0,
      wedgeAngle: 18.435,
      geoms: [
        {
          name: 'wedge_geom',
          type: 'box',
          size: [1.581, 0.6, 0.025],
          rgba: [0.8, 0.5, 0.2, 1],
          mass: 50,
          condim: 3,
          friction: [0.2, 0.005, 0.0005]
        }
      ],
      joints: [],
      children: []
    },
    {
      id: 'sliding_cube',
      name: 'sliding_cube',
      type: 'body',
      pos: [-1.0, 0, 1.3],
      geoms: [
        {
          name: 'sliding_cube_geom',
          type: 'box',
          size: [0.2, 0.2, 0.2],
          rgba: [0.95, 0.8, 0.2, 1],
          mass: 2.0,
          condim: 3,
          friction: [0.1, 0.005, 0.0005]
        }
      ],
      joints: [
        { name: 'sliding_cube_free', type: 'free' }
      ],
      children: []
    }
  ]
};

export const pulleySystemPreset: SceneGraph = {
  nodes: [
    {
      id: 'pulley_support',
      name: 'pulley_support',
      type: 'body',
      pos: [0, 0, 0],
      geoms: [
        {
          name: 'support_column',
          type: 'capsule',
          fromto: [0, 0, 0, 0, 0, 1.8],
          size: [0.05],
          rgba: [0.4, 0.4, 0.4, 1]
        },
        {
          name: 'support_peg',
          type: 'capsule',
          fromto: [0, 0, 1.6, 0, -0.35, 1.6],
          size: [0.03],
          rgba: [0.3, 0.3, 0.3, 1]
        }
      ],
      joints: [],
      children: []
    },
    {
      id: 'pulley_wheel',
      name: 'pulley_wheel',
      type: 'body',
      pos: [0, -0.3, 1.6],
      isPulleyWheel: true,
      pulleyRadius: 0.4,
      geoms: [
        { name: 'pulley_wheel_spindle', type: 'cylinder', size: [0.32, 0.03], pos: [0, 0, 0], euler: [90, 0, 0], rgba: [0.3, 0.4, 0.6, 1], mass: 0.5 },
        { name: 'pulley_wheel_flange_l', type: 'cylinder', size: [0.4, 0.01], pos: [0, -0.04, 0], euler: [90, 0, 0], rgba: [0.2, 0.3, 0.5, 1], mass: 0.25 },
        { name: 'pulley_wheel_flange_r', type: 'cylinder', size: [0.4, 0.01], pos: [0, 0.04, 0], euler: [90, 0, 0], rgba: [0.2, 0.3, 0.5, 1], mass: 0.25 }
      ],
      joints: [
        { name: 'pulley_wheel_hinge', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.2 }
      ],
      children: []
    },
    {
      id: 'left_weight',
      name: 'left_weight',
      type: 'body',
      pos: [-0.4, -0.3, 0.8],
      geoms: [
        {
          name: 'left_weight_geom',
          type: 'box',
          size: [0.15, 0.15, 0.15],
          rgba: [0.2, 0.6, 1.0, 1],
          mass: 5.0,
          condim: 3
        }
      ],
      joints: [
        { name: 'left_weight_joint', type: 'slide', axis: [0, 0, 1], damping: 1.0 }
      ],
      children: []
    },
    {
      id: 'right_weight',
      name: 'right_weight',
      type: 'body',
      pos: [0.4, -0.3, 0.8],
      geoms: [
        {
          name: 'right_weight_geom',
          type: 'box',
          size: [0.15, 0.15, 0.15],
          rgba: [0.95, 0.8, 0.2, 1],
          mass: 3.0,
          condim: 3
        }
      ],
      joints: [
        { name: 'right_weight_joint', type: 'slide', axis: [0, 0, 1], damping: 1.0 }
      ],
      children: []
    },
    {
      id: 'pulley_rope_preset',
      name: 'pulley_rope_preset',
      type: 'body',
      pos: [0, 0, 0],
      isPulleyRope: true,
      pulleyWheelId: 'pulley_wheel',
      leftTargetId: 'left_weight',
      rightTargetId: 'right_weight',
      geoms: [],
      joints: [],
      children: []
    }
  ]
};

export const cartpolePreset: SceneGraph = {
  nodes: [
    {
      id: 'rail',
      name: 'rail',
      type: 'body',
      pos: [0, 0, 1.0],
      joints: [],
      geoms: [
        {
          name: 'rail_geom',
          type: 'cylinder',
          size: [0.015], // 3cm diameter
          rgba: [0.3, 0.35, 0.4, 0.5], // semi-transparent slate
          fromto: [-2.0, 0, 0, 2.0, 0, 0], // horizontal slider rail along X
          contype: 0,
          conaffinity: 0
        }
      ],
      children: []
    },
    {
      id: 'cart',
      name: 'cart',
      type: 'body',
      pos: [0, 0, 1.0],
      joints: [
        { name: 'cart_slide', type: 'slide', axis: [1, 0, 0], damping: 1.0, limited: true, range: [-2.0, 2.0] }
      ],
      script: `// Cartpole LQR Balancing Controller
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
`,
      geoms: [
        {
          name: 'cart_geom',
          type: 'box',
          size: [0.25, 0.18, 0.12], // cart dimensions
          rgba: [0.15, 0.5, 0.85, 1], // vibrant premium blue
          mass: 2.0,
          condim: 3
        }
      ],
      children: [
        {
          id: 'pole',
          name: 'pole',
          type: 'body',
          pos: [0, 0, 0.12], // hinge sits on top of cart
          euler: [0, 5, 0],  // Initial tilt (5 degrees around Y)
          joints: [
            { name: 'pole_hinge', type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.05 }
          ],
          geoms: [
            {
              name: 'pole_rod_geom',
              type: 'capsule',
              fromto: [0, 0, 0, 0, 0, 0.7],
              size: [0.018],
              rgba: [0.65, 0.65, 0.65, 1],
              mass: 0.2
            }
          ],
          children: [
            {
              id: 'pole_weight',
              name: 'pole_weight',
              type: 'body',
              pos: [0, 0, 0.7],
              joints: [],
              geoms: [
                {
                  name: 'pole_weight_geom',
                  type: 'sphere',
                  pos: [0, 0, 0],
                  size: [0.06], // spherical tip weight
                  rgba: [0.85, 0.25, 0.25, 1], // crimson tip
                  mass: 0.6,
                  condim: 3
                }
              ],
              children: []
            }
          ]
        }
      ]
    }
  ]
};

export const newtonsCradlePreset: SceneGraph = {
  nodes: [
    {
      id: 'support_bar',
      name: 'support_bar',
      type: 'body',
      pos: [0, 0, 2.5],
      joints: [],
      geoms: [
        { name: 'bar_geom', type: 'cylinder', size: [0.03, 0.6], pos: [0, 0, 0], euler: [0, 90, 0], rgba: [0.3, 0.3, 0.3, 1], contype: 0, conaffinity: 0 }
      ],
      children: []
    },
    ...Array.from({ length: 5 }).map((_, idx): SceneNode => {
      const x = -0.4 + idx * 0.2;
      const isFirst = idx === 0;
      
      return {
        id: `cradle_${idx}`,
        name: `cradle_${idx}`,
        type: 'body',
        pos: [x, 0, 2.5],
        euler: isFirst ? [0, 35, 0] : [0, 0, 0],
        joints: [
          { name: `cradle_joint_${idx}`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.05, limited: true, range: [-90, 90] }
        ],
        geoms: [
          { name: `cradle_rod_${idx}`, type: 'capsule', fromto: [0, 0, 0, 0, 0, -1.0], size: [0.01], mass: 0.1, rgba: [0.7, 0.7, 0.7, 1], contype: 0, conaffinity: 0 }
        ],
        children: [
          {
            id: `cradle_bob_body_${idx}`,
            name: `cradle_bob_body_${idx}`,
            type: 'body',
            pos: [0, 0, -1.0],
            joints: [],
            geoms: [
              { name: `cradle_ball_${idx}`, type: 'sphere', size: [0.0995], pos: [0, 0, 0], mass: 6, rgba: [0.8, 0.8, 0.8, 1], solref: [-5000, -50.0], solimp: [0.98, 0.99, 0.001, 0.5, 2] }
            ],
            children: []
          }
        ]
      };
    })
  ]
};

const generateBridgePlanks = (): SceneNode[] => {
  const planksCount = 10;
  const plankLength = 0.28;
  
  const createPlankNode = (index: number): SceneNode => {
    const isFirst = index === 1;
    const isLast = index === planksCount;
    
    return {
      id: `plank_${index}`,
      name: `plank_${index}`,
      type: 'body',
      pos: isFirst ? [-1.4, 0, 1.3] : [plankLength, 0, 0],
      joints: [
        { name: `plank_joint_${index}`, type: 'hinge', axis: [0, 1, 0], pos: [0, 0, 0], damping: 0.1 }
      ],
      geoms: [
        { 
          name: `plank_geom_${index}`, 
          type: 'box', 
          size: [plankLength / 2, 0.25, 0.02], 
          pos: [plankLength / 2, 0, 0], 
          rgba: [0.65, 0.45, 0.25, 1], 
          mass: 0.5,
          condim: 3,
          friction: [0.8, 0.01, 0.001]
        }
      ],
      children: isLast ? [] : [createPlankNode(index + 1)],
      ...(isLast ? { connectTargetId: 'support_r', connectAnchor: [1.4, 0, 1.3] } : {})
    };
  };
  
  return [createPlankNode(1)];
};

export const suspensionBridgePreset: SceneGraph = {
  nodes: [
    {
      id: 'support_l',
      name: 'support_l',
      type: 'body',
      pos: [-1.5, 0, 0.65],
      joints: [],
      geoms: [
        { name: 'support_l_geom', type: 'box', size: [0.1, 0.35, 0.65], rgba: [0.4, 0.45, 0.5, 1] }
      ],
      children: []
    },
    {
      id: 'support_r',
      name: 'support_r',
      type: 'body',
      pos: [1.5, 0, 0.65],
      joints: [],
      geoms: [
        { name: 'support_r_geom', type: 'box', size: [0.1, 0.35, 0.65], rgba: [0.4, 0.45, 0.5, 1] }
      ],
      children: []
    },
    ...generateBridgePlanks(),
    {
      id: 'rolling_ball',
      name: 'rolling_ball',
      type: 'body',
      pos: [0, 0, 2.2],
      joints: [
        { name: 'rolling_ball_free', type: 'free' }
      ],
      geoms: [
        { 
          name: 'ball_geom', 
          type: 'sphere', 
          size: [0.38], 
          rgba: [0.85, 0.25, 0.25, 1], 
          mass: 30.0, 
          condim: 3, 
          friction: [0.4, 0.01, 0.001] 
        }
      ],
      children: []
    }
  ]
};

export const paperPlanePreset: SceneGraph = {
  nodes: [
    {
      id: 'paper_plane_wing',
      name: 'paper_plane_wing',
      type: 'body',
      pos: [0, 0, 3.5],
      euler: [0, 8, 0],
      isAerodynamic: true,
      joints: [
        { name: 'plane_free', type: 'free', initialVelocity: [4.0, 0.0, 0.5, 0.0, 0.0, 0.0] }
      ],
      geoms: [
        {
          name: 'wing_geom',
          type: 'box',
          size: [0.06, 0.28, 0.003],
          rgba: [0.96, 0.96, 0.94, 1],
          mass: 0.004,
          condim: 3,
          friction: [0.3, 0.005, 0.0005]
        }
      ],
      children: [
        {
          id: 'paper_plane_spine',
          name: 'paper_plane_spine',
          type: 'body',
          pos: [0, 0, 0],
          euler: [0, 0, 0],
          joints: [],
          geoms: [
            {
              name: 'spine_geom',
              type: 'box',
              size: [0.13, 0.018, 0.012],
              rgba: [0.88, 0.88, 0.86, 1],
              mass: 0.003,
              condim: 3
            }
          ],
          children: []
        },
        {
          id: 'paper_plane_nose',
          name: 'paper_plane_nose',
          type: 'body',
          pos: [0.13, 0, 0],
          euler: [0, 0, 0],
          joints: [],
          geoms: [
            {
              name: 'nose_geom',
              type: 'sphere',
              size: [0.011],
              rgba: [0.75, 0.75, 0.72, 1],
              mass: 0.003,
              condim: 3
            }
          ],
          children: []
        }
      ]
    }
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
  },
  inclined_plane: {
    name: 'Inclined Plane',
    scene: inclinedPlanePreset
  },
  pulley_system: {
    name: 'Pulley System Stand',
    scene: pulleySystemPreset
  },
  cartpole: {
    name: 'Cartpole System',
    scene: cartpolePreset
  },
  newtons_cradle: {
    name: "Newton's Cradle",
    scene: newtonsCradlePreset
  },
  suspension_bridge: {
    name: 'Suspension Bridge',
    scene: suspensionBridgePreset
  },
  paper_plane: {
    name: 'Paper Plane',
    scene: paperPlanePreset
  }
};
