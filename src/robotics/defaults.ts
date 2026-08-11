/// <reference types="vite/client" />

import type { JointState, RobotParameters } from './types'

function deepFreeze<T>(value: T): T {
  if (value !== null && typeof value === 'object' && !Object.isFrozen(value)) {
    for (const nestedValue of Object.values(value)) {
      deepFreeze(nestedValue)
    }
    Object.freeze(value)
  }
  return value
}

const robotParameters: RobotParameters = {
  geometry: { d1: 0.8, l2: 2, l3: 1.5 },
  links: [
    {
      mass: 4,
      centerOfMass: [0, -0.4, 0],
      inertia: [[0.2133, 0, 0], [0, 0.02, 0], [0, 0, 0.2133]],
    },
    {
      mass: 3,
      centerOfMass: [-1, 0, 0],
      inertia: [[0.03, 0, 0], [0, 1, 0], [0, 0, 1]],
    },
    {
      mass: 2,
      centerOfMass: [-0.75, 0, 0],
      inertia: [[0.02, 0, 0], [0, 0.375, 0], [0, 0, 0.375]],
    },
  ],
  gravity: [0, 0, -9.81],
  viscousFriction: [0.08, 0.08, 0.05],
  frictionEnabled: true,
  jointLimits: [
    [-Math.PI, Math.PI],
    [-Math.PI / 2, Math.PI / 2],
    [-5 * Math.PI / 6, 5 * Math.PI / 6],
  ],
}

const jointState: JointState = {
  q: [Math.PI / 6, 5 * Math.PI / 36, -5 * Math.PI / 18],
  qd: [0, 0, 0],
  qdd: [0, 0, 0],
}

export const DEFAULT_ROBOT_PARAMETERS: RobotParameters = import.meta.env.DEV
  ? deepFreeze(robotParameters)
  : robotParameters

export const DEFAULT_JOINT_STATE: JointState = import.meta.env.DEV
  ? deepFreeze(jointState)
  : jointState
