export type GeomType = 'capsule' | 'sphere' | 'box' | 'plane' | 'cylinder' | 'ellipsoid' | 'mesh';
export type JointType = 'hinge' | 'slide' | 'ball' | 'free';

export interface SceneGeom {
  name: string;
  type: GeomType;
  size: number[];
  rgba?: number[];
  fromto?: number[];
  pos?: number[];
  quat?: number[];
  euler?: number[];
  mass?: number;
  contype?: number;
  conaffinity?: number;
  condim?: number;
  friction?: number[];
  solref?: number[];
  solimp?: number[];
  // For type='mesh': flat array of vertex positions (x0,y0,z0, x1,y1,z1, ...) and
  // flat array of triangle face indices (i0,j0,k0, i1,j1,k1, ...).
  vertices?: number[];
  faces?: number[];
}

export interface SceneJoint {
  name: string;
  type: JointType;
  axis?: number[];
  pos?: number[];
  damping?: number;
  stiffness?: number;
  springref?: number;
  limited?: boolean;
  range?: number[];
  actuator?: {
    type: 'velocity' | 'motor';
    kv?: number; // For velocity actuators
    gear?: number; // Optional gear ratio
    ctrlValue?: number; // Target speed or force from UI
  };
  initialVelocity?: number[]; // [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
}

export interface SceneNode {
  id: string;
  name: string;
  type: 'body';
  pos: number[];
  quat?: number[];
  euler?: number[];
  geoms: SceneGeom[];
  joints: SceneJoint[];
  children: SceneNode[];
  allowCoupling?: boolean;
  coupleTargetId?: string;
  coupleRatio?: number;
  weldTargetId?: string;
  connectTargetId?: string;
  connectAnchor?: number[];
  isWedge?: boolean;
  width?: number;
  depth?: number;
  height?: number;
  wedgeAngle?: number;
  isPulleyWheel?: boolean;
  leftTargetId?: string;
  rightTargetId?: string;
  pulleyRadius?: number;
  isPulleyRope?: boolean;
  pulleyWheelId?: string;
  isAerodynamic?: boolean;
  script?: string;
}

export interface SceneGraph {
  nodes: SceneNode[];
}
