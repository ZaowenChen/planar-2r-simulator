import { describe, expect, it } from 'vitest'
import type { SimulationSample } from '../robotics/integration'
import { simulationSamplesToCsv, trajectorySamplesToCsv } from './csv'

describe('simulationSamplesToCsv', () => {
  it('exports every simulation quantity with explicit SI units and a UTF-8 BOM', () => {
    const samples: SimulationSample[] = [{
      time: 0.25,
      q: [0.1, -0.2, 0.3],
      qd: [1, -2, 3],
      qdd: [4, -5, 6],
      tau: [7, -8, 9],
      kinetic: 10,
      potential: -11,
      totalEnergy: -1,
      jointPower: [7, 16, 27],
    }]

    const csv = simulationSamplesToCsv(samples)
    const [header, row] = csv.slice(1).trim().split('\n')

    expect(csv.charCodeAt(0)).toBe(0xfeff)
    expect(header).toBe([
      'time_s',
      'theta_1_rad', 'theta_2_rad', 'theta_3_rad',
      'omega_1_rad_s', 'omega_2_rad_s', 'omega_3_rad_s',
      'alpha_1_rad_s2', 'alpha_2_rad_s2', 'alpha_3_rad_s2',
      'tau_1_N_m', 'tau_2_N_m', 'tau_3_N_m',
      'kinetic_J', 'potential_J', 'total_energy_J',
      'power_1_W', 'power_2_W', 'power_3_W',
    ].join(','))
    expect(row).toBe('0.25,0.1,-0.2,0.3,1,-2,3,4,-5,6,7,-8,9,10,-11,-1,7,16,27')
  })

  it('exports an empty simulation as a BOM-prefixed header without invented samples', () => {
    const csv = simulationSamplesToCsv([])

    expect(csv.startsWith('\ufefftime_s,theta_1_rad')).toBe(true)
    expect(csv.trim().split('\n')).toHaveLength(1)
  })
})

describe('trajectorySamplesToCsv', () => {
  it('exports only time and joint trajectory state with explicit SI units', () => {
    const csv = trajectorySamplesToCsv([{
      time: 0.5,
      q: [0.1, 0.2, 0.3],
      qd: [1, 2, 3],
      qdd: [4, 5, 6],
    }])
    const [header, row] = csv.slice(1).trim().split('\n')

    expect(header).toBe([
      'time_s',
      'theta_1_rad', 'theta_2_rad', 'theta_3_rad',
      'omega_1_rad_s', 'omega_2_rad_s', 'omega_3_rad_s',
      'alpha_1_rad_s2', 'alpha_2_rad_s2', 'alpha_3_rad_s2',
    ].join(','))
    expect(row).toBe('0.5,0.1,0.2,0.3,1,2,3,4,5,6')
    expect(header).not.toContain('tau')
    expect(header).not.toContain('energy')
  })
})
