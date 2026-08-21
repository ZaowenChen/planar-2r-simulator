import { expect, test } from '@playwright/test'

test('completes a kinematics and inverse-dynamics learning flow', async ({ page }) => {
  await page.goto('/')
  await expect(page.getByRole('heading', { name: '空间 3R 机器人学交互实验室' })).toBeVisible()
  await page.getByRole('button', { name: '运动学' }).click()
  await page.getByRole('button', { name: '修改关节角' }).click()
  await page.getByLabel('关节角 θ₂').fill('35')
  for (let step = 0; step < 4; step += 1) {
    await page.getByRole('button', { name: '下一步' }).click()
  }
  await expect(page.getByTestId('endpoint-result')).toContainText('mm')
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

test('keeps desktop learning modules inside their intended workbench layout', async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 1000 })
  await page.goto('/')

  for (const moduleName of ['机器人模型', '运动学', '动力学', '动态实验']) {
    await page.getByRole('button', { name: moduleName, exact: true }).click()
    const sections = moduleName === '运动学'
      ? page.locator('.kinematics-stage > section')
      : page.locator('.workbench > section')
    await expect(sections).toHaveCount(moduleName === '运动学' ? 2 : moduleName === '动态实验' ? 4 : 3)

    const boxes = await sections.evaluateAll((elements) => elements.map((element) => {
      const rect = element.getBoundingClientRect()
      return { left: rect.left, right: rect.right, scrollWidth: element.scrollWidth }
    }))
    boxes.slice(0, moduleName === '运动学' ? 2 : 3).forEach((box, sectionIndex) => {
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

test('links the D–H table, geometric derivation, and 3D teaching annotations', async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 1000 })
  await page.goto('/')
  await page.getByRole('button', { name: '运动学', exact: true }).click()

  await expect(page.getByText('当前世界坐标系 {W} 与基座坐标系 {0} 重合。')).toBeVisible()
  await expect(page.getByRole('button', { name: '回到本步视角' })).toHaveCount(1)
  const teachingCanvas = page.getByRole('region', { name: '机器人三维教学看板' }).locator('canvas')
  const teachingCanvasBox = await teachingCanvas.boundingBox()
  expect(teachingCanvasBox).not.toBeNull()
  await page.mouse.move(
    teachingCanvasBox!.x + teachingCanvasBox!.width / 2,
    teachingCanvasBox!.y + teachingCanvasBox!.height / 2,
  )
  await page.mouse.wheel(0, 180)
  await expect(page.getByRole('checkbox', { name: '跟随推导视角' })).not.toBeChecked()
  await page.getByRole('button', { name: '回到本步视角' }).click()
  await expect(page.getByRole('checkbox', { name: '跟随推导视角' })).toBeChecked()
  await expect(page.locator('.robot-scene__dimension-label')).toHaveCount(2)
  await expect(page.locator('.robot-scene__dimension-label')).toContainText(['z₀', 'z₁'])
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__axis-label')).toHaveCount(3)
  await page.getByRole('button', { name: '沿 x₂ 平移 a₂' }).click()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.getByText('当前子步骤：沿 x₂ 平移 a₂')).toBeVisible()
  await expect(page.locator('.robot-scene__axis-label')).toHaveCount(1)
  await expect(page.locator('.robot-scene__axis-label').filter({ hasText: /^x2$/ })).toHaveCount(1)

  await page.getByRole('tab', { name: /^位置逆运动学/ }).click()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 'r =' })).toBeVisible()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 'h =' })).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 's =' })).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 'l₂ =' })).toBeVisible()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 'l₃ =' })).toBeVisible()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 's =' })).toBeVisible()

  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__angle-label', { hasText: 'θ₃ =' })).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__angle-label', { hasText: 'γ =' })).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__angle-label', { hasText: 'δ =' })).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.locator('.robot-scene__angle-label')).toHaveCount(3)
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.getByRole('group', { name: '全部解析几何候选' })).toBeVisible()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: 'rₛ =' })).toBeVisible()

  await page.getByRole('button', { name: /位置回代/ }).click()
  await expect(page.getByRole('table', { name: '逆解回代比较' })).toBeVisible()
  await expect(page.locator('.robot-scene__dimension-label', { hasText: '已放大，仅用于观察' })).toBeVisible()
})

test('separates FK, position IK, and differential-motion inputs', async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 1000 })
  await page.goto('/')
  await page.getByRole('button', { name: '运动学', exact: true }).click()

  await expect(page.getByRole('tab', { name: /^正运动学/ })).toHaveAttribute('aria-selected', 'true')
  await page.getByRole('tab', { name: /^位置逆运动学/ }).click()
  await page.getByRole('button', { name: '修改目标 / 选择逆解' }).click()
  await expect(page.getByLabel('期望位置 x')).toBeVisible()
  await expect(page.getByLabel('关节角 θ₁')).toHaveCount(0)
  await page.getByRole('button', { name: '收起参数' }).click()

  await page.getByRole('tab', { name: /^微分运动学/ }).click()
  await page.getByRole('button', { name: '修改运动状态' }).click()
  await page.getByLabel('关节速度 θ̇₂').fill('12')
  await page.getByLabel('关节速度 θ̇₂').press('Enter')
  await expect(page.getByText(/θ̇₂ 12\.0°\/s/)).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.getByTestId('jacobian-result')).toBeVisible()
  await page.getByRole('button', { name: '下一步' }).click()
  await expect(page.getByRole('heading', { name: '计算末端线速度与角速度' })).toBeVisible()
  const vectorLabels = page.locator('.robot-scene__vector-label')
  await expect(vectorLabels).toHaveCount(2)
  const [canvasBox, labelBoxes] = await Promise.all([
    page.getByRole('region', { name: '机器人三维教学看板' }).locator('canvas').boundingBox(),
    vectorLabels.evaluateAll((labels) => labels.map((label) => {
      const rect = label.getBoundingClientRect()
      return { left: rect.left, right: rect.right, top: rect.top, bottom: rect.bottom }
    })),
  ])
  expect(canvasBox).not.toBeNull()
  labelBoxes.forEach((box) => {
    expect(box.left).toBeGreaterThanOrEqual(canvasBox!.x)
    expect(box.right).toBeLessThanOrEqual(canvasBox!.x + canvasBox!.width)
    expect(box.top).toBeGreaterThanOrEqual(canvasBox!.y)
    expect(box.bottom).toBeLessThanOrEqual(canvasBox!.y + canvasBox!.height)
  })
})

test('uses compact kinematics panes without page-level overflow on mobile', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 })
  await page.goto('/')
  await page.getByRole('button', { name: '运动学', exact: true }).click()

  const visual = page.getByRole('region', { name: '机器人三维教学看板' })
  const controls = page.getByRole('region', { name: '运动学参数编辑' })
  const analysis = page.getByRole('region', { name: '运动学公式推导' })
  await expect(page.getByRole('tab', { name: '推导' })).toBeVisible()
  await expect(page.getByRole('tab', { name: '3D' })).toBeVisible()
  await expect(page.getByRole('tab', { name: '参数' })).toBeVisible()
  await expect(analysis).toBeVisible()
  await expect(visual).toBeHidden()
  await expect(controls).toHaveCount(0)

  await page.getByRole('tab', { name: '3D' }).click()
  await expect(visual).toBeVisible()
  await expect(analysis).toBeHidden()
  await page.getByRole('tab', { name: '参数' }).click()
  await expect(controls).toBeVisible()
  await expect(visual).toBeHidden()

  const horizontalOverflow = await page.evaluate(() => (
    document.documentElement.scrollWidth - document.documentElement.clientWidth
  ))
  expect(horizontalOverflow).toBeLessThanOrEqual(1)
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
