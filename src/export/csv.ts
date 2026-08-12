import type { SimulationSample } from '../robotics/integration'

const CSV_COLUMNS = [
  'time_s',
  'theta_1_rad', 'theta_2_rad', 'theta_3_rad',
  'omega_1_rad_s', 'omega_2_rad_s', 'omega_3_rad_s',
  'alpha_1_rad_s2', 'alpha_2_rad_s2', 'alpha_3_rad_s2',
  'tau_1_N_m', 'tau_2_N_m', 'tau_3_N_m',
  'kinetic_J', 'potential_J', 'total_energy_J',
  'power_1_W', 'power_2_W', 'power_3_W',
] as const

function valuesForSample(sample: SimulationSample): readonly number[] {
  return [
    sample.time,
    ...sample.q,
    ...sample.qd,
    ...sample.qdd,
    ...sample.tau,
    sample.kinetic,
    sample.potential,
    sample.totalEnergy,
    ...sample.jointPower,
  ]
}

export function simulationSamplesToCsv(samples: readonly SimulationSample[]): string {
  const rows = samples.map((sample) => valuesForSample(sample).join(','))
  return `\ufeff${[CSV_COLUMNS.join(','), ...rows].join('\n')}\n`
}
