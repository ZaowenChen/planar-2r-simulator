import { expect, test } from '@playwright/test'

test('completes a kinematics and inverse-dynamics learning flow', async ({ page }) => {
  await page.goto('/')
  await expect(page.getByRole('heading', { name: '空间 3R 机器人学交互实验室' })).toBeVisible()
  await page.getByRole('button', { name: '运动学' }).click()
  await page.getByLabel('关节角 θ₂').fill('35')
  for (let step = 0; step < 5; step += 1) {
    await page.getByRole('button', { name: '下一步' }).click()
  }
  await expect(page.getByTestId('endpoint-result')).toContainText('m')
  await page.getByRole('button', { name: '动态实验' }).click()
  await page.getByRole('tab', { name: '逆动力学' }).click()
  await page.getByRole('button', { name: '生成实验' }).click()
  await page.getByRole('button', { name: '播放' }).click()
  await expect(page.getByTestId('simulation-time')).not.toHaveText('0.000 s')
})

test('captures the 1440 by 1000 desktop preview artifact', async ({ page }, testInfo) => {
  await page.setViewportSize({ width: 1440, height: 1000 })
  await page.goto('/')
  await expect(page.getByRole('heading', { name: '空间 3R 机器人学交互实验室' })).toBeVisible()
  await expect(page.locator('.robot-scene__frame-label', { hasText: '{e}' })).toBeVisible()
  await page.screenshot({
    path: testInfo.outputPath('robotics-lab-preview.png'),
    fullPage: true,
  })
})

test('keeps every desktop learning module within its three-column workbench', async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 1000 })
  await page.goto('/')

  for (const moduleName of ['机器人模型', '运动学', '动力学', '动态实验']) {
    await page.getByRole('button', { name: moduleName, exact: true }).click()
    const sections = page.locator('.workbench > section')
    await expect(sections).toHaveCount(moduleName === '动态实验' ? 4 : 3)

    const boxes = await sections.evaluateAll((elements) => elements.map((element) => {
      const rect = element.getBoundingClientRect()
      return { left: rect.left, right: rect.right, scrollWidth: element.scrollWidth }
    }))
    boxes.slice(0, 3).forEach((box, sectionIndex) => {
      expect(box.right, `${moduleName} section ${sectionIndex + 1} right edge`).toBeLessThanOrEqual(1440)
      expect(box.scrollWidth, `${moduleName} section ${sectionIndex + 1} content width`)
        .toBeLessThanOrEqual(Math.ceil(box.right - box.left))
    })
  }
})

test('keeps the Three.js canvas inside the visual workbench column', async ({ page }) => {
  await page.goto('/')

  const visual = page.getByRole('region', { name: '机器人三维视图' })
  const canvas = visual.locator('canvas')
  const [visualBox, canvasBox] = await Promise.all([
    visual.boundingBox(),
    canvas.boundingBox(),
  ])

  expect(visualBox).not.toBeNull()
  expect(canvasBox).not.toBeNull()
  expect(canvasBox!.x + canvasBox!.width).toBeLessThanOrEqual(
    visualBox!.x + visualBox!.width,
  )
})

test('stacks the workbench cleanly at 1024 pixels', async ({ page }) => {
  await page.setViewportSize({ width: 1024, height: 1000 })
  await page.goto('/')

  const visual = page.getByRole('region', { name: '机器人三维视图' })
  const controls = page.getByRole('region', { name: '实验控制' })
  const analysis = page.getByRole('region', { name: '公式与结果' })
  const [visualBox, controlsBox, analysisBox] = await Promise.all([
    visual.boundingBox(),
    controls.boundingBox(),
    analysis.boundingBox(),
  ])

  expect(visualBox).not.toBeNull()
  expect(controlsBox).not.toBeNull()
  expect(analysisBox).not.toBeNull()
  expect(controlsBox!.y).toBeGreaterThan(visualBox!.y + visualBox!.height - 1)
  expect(analysisBox!.y).toBeGreaterThan(controlsBox!.y + controlsBox!.height - 1)
  expect(Math.abs(visualBox!.x - controlsBox!.x)).toBeLessThan(1)
  expect(Math.abs(visualBox!.width - controlsBox!.width)).toBeLessThan(1)

  const horizontalOverflow = await page.evaluate(() => (
    document.documentElement.scrollWidth - document.documentElement.clientWidth
  ))
  expect(horizontalOverflow).toBeLessThanOrEqual(1)
})
