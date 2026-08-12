import { defineConfig } from 'vitest/config'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  test: {
    environment: 'jsdom',
    fileParallelism: !process.env.CI,
    include: ['src/**/*.test.{ts,tsx}'],
    exclude: ['e2e/**', 'node_modules/**'],
    setupFiles: ['./src/test/setup.ts'],
    testTimeout: process.env.CI ? 30_000 : 5_000,
  },
})
