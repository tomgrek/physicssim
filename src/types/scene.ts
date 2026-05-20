export type GeomType = 'capsule' | 'sphere' | 'box' | 'plane' | 'cylinder';
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
}

export interface SceneJoint {
  name: string;
  type: JointType;
  axis?: number[];
  pos?: number[];
  damping?: number;
  limited?: boolean;
  range?: number[];
  actuator?: {
    type: 'velocity' | 'motor';
    kv?: number; // For velocity actuators
    gear?: number; // Optional gear ratio
    ctrlValue?: number; // Target speed or force from UI
  };
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
}

export interface SceneGraph {
  nodes: SceneNode[];
}
