# Figma–Codex Design Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure Codex Desktop and Codex CLI so Figma is a reliable Codex-controlled design canvas, with a supported local runtime, durable repository guidance, and a verified write workflow.

**Architecture:** Keep the authenticated app-backed Figma plugin as the primary design surface and install the same public plugin for CLI sessions. Store behavioral rules in the repository root `AGENTS.md`, keep user-facing usage examples in `docs/figma-codex-workflow.md`, and avoid a duplicate manually configured Figma MCP server. Validate the setup by creating and inspecting a dedicated Figma design file.

**Tech Stack:** Codex Desktop, Codex CLI, Figma plugin 2.x, Figma Plugin API, Node.js 24 LTS, Homebrew, zsh, Markdown.

## Global Constraints

- Default direction is Codex → Figma; do not modify project code unless the user explicitly requests Figma → Code.
- Keep the existing authenticated Figma plugin and do not store a static Figma token.
- Do not add a duplicate `[mcp_servers.figma]` entry.
- Install Node.js 24 LTS and resolve `node` and `npm` from `/opt/homebrew/opt/node@24/bin`.
- Back up `~/.codex/config.toml` and `~/.zshrc` before modifying global state.
- Preserve the user's current uncommitted product changes; stage and commit only files created by this plan.
- Do not enable the Figma plugin's draft post-write hook.
- Figma writes must inspect first, use Auto Layout, return all affected node IDs, and validate structure plus screenshot output.

---

## File Structure

- Create `AGENTS.md`: repository-wide instructions for Codex-controlled Figma design work and project verification.
- Create `docs/figma-codex-workflow.md`: user-facing quickstart, prompt recipes, link requirements, and troubleshooting.
- Modify `~/.codex/config.toml`: only through `codex update` or `codex plugin add`; retain a backup.
- Modify `~/.zshrc`: append one Node 24 PATH block; retain a backup.
- Create one external Figma file named `Codex Design Sandbox`: end-to-end write verification artifact.

### Task 1: Upgrade Codex CLI and install the Figma CLI plugin

**Files:**
- Backup: `/Users/chenzaowen/.codex/config.toml.backup-20260821-figma-workflow`
- Modify: `/Users/chenzaowen/.codex/config.toml` through supported Codex commands

**Interfaces:**
- Consumes: working Codex ChatGPT authentication and configured `openai-curated` marketplace.
- Produces: an updated `codex` executable and an installed, enabled `figma@openai-curated` CLI plugin.

- [ ] **Step 1: Capture the pre-change state**

Run:

```bash
codex --version
codex doctor --summary --no-color
codex plugin list | grep 'figma@openai-curated'
```

Expected: CLI reports `0.139.0`; doctor reports the config parses; the Figma row reports `not installed`.

- [ ] **Step 2: Back up the Codex configuration**

Run:

```bash
cp -p /Users/chenzaowen/.codex/config.toml /Users/chenzaowen/.codex/config.toml.backup-20260821-figma-workflow
cmp /Users/chenzaowen/.codex/config.toml /Users/chenzaowen/.codex/config.toml.backup-20260821-figma-workflow
```

Expected: `cmp` exits with status 0 and prints nothing.

- [ ] **Step 3: Upgrade the CLI using its supported updater**

Run:

```bash
codex update
codex --version
```

Expected: update succeeds and the version is at least `0.149.0`.

- [ ] **Step 4: Re-run the configuration health check**

Run:

```bash
codex doctor --summary --no-color
```

Expected: config, auth, MCP, sandbox, network, and installation checks remain healthy. Ignore only `TERM=dumb` when the command runs without a TTY and the known historical rollout scan warning.

- [ ] **Step 5: Install the Figma plugin for CLI sessions**

Run:

```bash
codex plugin add figma@openai-curated --json
codex plugin list | grep 'figma@openai-curated'
```

Expected: install JSON identifies `figma@openai-curated`; plugin list reports `installed, enabled` and a concrete version/path.

- [ ] **Step 6: Confirm the install did not add a duplicate manual MCP server**

Run:

```bash
codex mcp list | grep -i figma
```

Expected: no manually configured Figma MCP row. Figma remains plugin-provided.

### Task 2: Normalize Node.js and npm on Node 24 LTS

**Files:**
- Backup: `/Users/chenzaowen/.zshrc.backup-20260821-figma-workflow`
- Modify: `/Users/chenzaowen/.zshrc`

**Interfaces:**
- Consumes: Homebrew at `/opt/homebrew/bin/brew` and zsh startup files.
- Produces: `node` and `npm` commands resolved from `/opt/homebrew/opt/node@24/bin` in new login shells.

- [ ] **Step 1: Record the mismatch before changing it**

Run:

```bash
node --version
npm --version
command -v node
command -v npm
```

Expected: Node reports `v20.15.0`, while npm resolves from `/opt/homebrew/bin`; npm warns that this Node version is unsupported.

- [ ] **Step 2: Back up the zsh configuration**

Run:

```bash
cp -p /Users/chenzaowen/.zshrc /Users/chenzaowen/.zshrc.backup-20260821-figma-workflow
cmp /Users/chenzaowen/.zshrc /Users/chenzaowen/.zshrc.backup-20260821-figma-workflow
```

Expected: `cmp` exits with status 0 and prints nothing.

- [ ] **Step 3: Install the supported LTS runtime**

Run:

```bash
/opt/homebrew/bin/brew install node@24
/opt/homebrew/bin/brew info node@24 | sed -n '1,12p'
```

Expected: Homebrew reports Node `24.19.0` or a newer 24.x LTS bottle installed as keg-only.

- [ ] **Step 4: Append one idempotent Node PATH block**

If `/Users/chenzaowen/.zshrc` does not already contain `/opt/homebrew/opt/node@24/bin`, append exactly:

```zsh

# Codex/Figma workflow: Node.js 24 LTS
export PATH="/opt/homebrew/opt/node@24/bin:$PATH"
```

Use `apply_patch`; do not rewrite the existing ROS 2 PATH declarations.

- [ ] **Step 5: Verify a new login shell uses one coherent runtime**

Run:

```bash
zsh -lic 'node --version; npm --version; command -v node; command -v npm'
```

Expected: Node is 24.x; both command paths begin with `/opt/homebrew/opt/node@24/bin`; no unsupported-Node warning appears.

- [ ] **Step 6: Verify the repository TypeScript toolchain starts successfully**

Run:

```bash
zsh -lic 'cd "/Users/chenzaowen/Documents/Simulink (Robotics System Toolbox)" && npm run typecheck'
```

Expected: typecheck passes. If it fails only because of the user's existing product changes, preserve the failure output and do not edit those product files as part of this task.

### Task 3: Add durable repository Figma rules

**Files:**
- Create: `AGENTS.md`

**Interfaces:**
- Consumes: repository layout, `package.json` scripts, and the approved Figma workflow design.
- Produces: repository-wide instructions automatically loaded by future Codex tasks.

- [ ] **Step 1: Confirm no root instruction file exists**

Run:

```bash
test ! -e AGENTS.md
```

Expected: exit status 0.

- [ ] **Step 2: Create the root instruction file**

Create `AGENTS.md` with exactly these sections and rules:

```markdown
# Repository Guidance

## Project

- The product is a React 19 + TypeScript + Vite browser-based 3R robotics teaching lab.
- Reuse shared UI from `src/components/` and CSS variables from `src/app/app.css`.
- Product verification commands are `npm test`, `npm run typecheck`, `npm run build`, and `npm run e2e`.

## Figma as a Design Tool

- When the user asks Codex to design in Figma, default to Codex → Figma. Do not edit product code unless the user explicitly asks for Figma → Code.
- Before writing, inspect the target Figma file's pages, components, variables, styles, fonts, and naming conventions.
- Reuse existing Figma components, styles, and variables. Use Auto Layout for structurally related content instead of absolute child positioning.
- Split large writes into small validated batches. Return every created or modified node ID from each Figma write.
- Load the current font before every text mutation and await every asynchronous Figma API call.
- After each meaningful write, check node structure and take a screenshot. Fix text clipping, overlap, alignment, spacing, and unexpected resizing before continuing.
- On a Figma tool error, stop, read the error, inspect the current file state, and then make one targeted retry. Do not blindly regenerate the whole page.
- Do not store Figma access tokens, duplicate the plugin with a manual MCP server, or enable the plugin's draft post-write hook.

## Figma to Code

- Only enter Figma → Code when explicitly requested.
- Read design context before implementation, treat generated code as reference, and adapt it to this repository's React components and CSS variables.
- Download exact exported assets for committed code; do not replace them with invented SVGs or placeholders.
- Verify implementation with typecheck, relevant tests, browser preview, and screenshot-based parity review.

## Git Safety

- Preserve unrelated user changes in a dirty worktree.
- Stage and commit only files that belong to the active task.
```

- [ ] **Step 3: Validate the instruction file**

Run:

```bash
grep -n '## Figma as a Design Tool' AGENTS.md
grep -n 'default to Codex → Figma' AGENTS.md
if grep -n '[[:blank:]]$' AGENTS.md; then exit 1; fi
```

Expected: both content grep commands find one line and the trailing-whitespace check prints nothing.

- [ ] **Step 4: Commit only the repository guidance**

Run:

```bash
git add -- AGENTS.md
git diff --cached --name-only
git diff --cached --check
git commit -m "docs: add Figma design workflow guidance"
```

Expected: the cached file list contains only `AGENTS.md` before commit.

### Task 4: Add the Figma–Codex quickstart guide

**Files:**
- Create: `docs/figma-codex-workflow.md`

**Interfaces:**
- Consumes: the repository rules from Task 3 and authenticated Figma plugin behavior.
- Produces: copy-ready prompts and a troubleshooting checklist for daily use.

- [ ] **Step 1: Create the guide with the required sections**

Create `docs/figma-codex-workflow.md` with:

```markdown
# 使用 Codex 在 Figma 中设计

## 推荐入口

优先在 Codex 桌面端使用已连接的 Figma 插件。CLI 安装插件后适合作为备用入口；开始新会话后再使用新安装的插件技能。

## 新建设计

> 请使用 Figma 创建一个设计文件，目标是：[目标]。先检查页面、组件、变量、样式和字体，再使用 Auto Layout 分批创建。每个阶段返回节点 ID，并用结构检查和截图检查验证。

## 修改现有设计

提供设计文件链接；如果要改具体区域，提供带 `node-id` 的链接。

> 请修改这个 Figma 节点：[带 node-id 的链接]。先检查当前结构和截图，只修改目标节点及必要依赖。完成后检查文本裁切、重叠、间距、对齐和尺寸。

## 从代码更新 Figma

> 请把当前代码中的 [页面或组件] 同步成 Figma 设计。先检查代码组件、CSS 变量和现有 Figma 组件，再在 Figma 中增量更新；不要修改产品代码。

## 视觉复查

> 请复查这个 Figma 节点：[带 node-id 的链接]。按严重程度报告并修复结构、排版、色彩、组件复用、响应式意图和可访问性问题；修复后重新截图验证。

## Figma → Code

只有明确要求实现代码时才使用：

> 请把这个 Figma 节点实现到代码中：[带 node-id 的链接]，目标文件是 [路径]。先读取设计上下文，复用现有 React 组件和 `src/app/app.css` 变量，下载真实资产，并运行类型检查、测试和截图一致性复查。

## 链接要求

- 设计文件：`https://www.figma.com/design/<fileKey>/<name>`
- 具体节点：`https://www.figma.com/design/<fileKey>/<name>?node-id=1-2`
- 修改具体节点时不要省略 `node-id`。

## 故障排查

1. 用 Figma 身份检查确认账号和计划。
2. 用 `codex plugin list` 确认 CLI 插件已安装并启用。
3. 插件刚安装时启动新的 Codex CLI 会话。
4. 写入失败时先检查文件类型、节点 ID、页面和字体，不连续盲目重试。
5. 桌面端正常而 CLI 不正常时，检查的是 CLI 插件状态，不要重复添加手工 Figma MCP。
```

- [ ] **Step 2: Validate the guide**

Run:

```bash
grep -n '^## ' docs/figma-codex-workflow.md
grep -n '不要修改产品代码' docs/figma-codex-workflow.md
if grep -n '[[:blank:]]$' docs/figma-codex-workflow.md; then exit 1; fi
```

Expected: all eight section headings are present, the code → Figma constraint is found, and the trailing-whitespace check prints nothing.

- [ ] **Step 3: Commit only the quickstart guide**

Run:

```bash
git add -- docs/figma-codex-workflow.md
git diff --cached --name-only
git diff --cached --check
git commit -m "docs: add Figma Codex quickstart"
```

Expected: the cached file list contains only `docs/figma-codex-workflow.md` before commit.

### Task 5: Run the Figma end-to-end write test

**Files:**
- Create externally: Figma design file `Codex Design Sandbox`

**Interfaces:**
- Consumes: authenticated Figma plan key `team::1658127778171681639`, the `figma-create-new-file` skill, and the `figma-use` skill.
- Produces: a Figma file URL, a top-level status card node ID, metadata evidence, and screenshot evidence.

- [ ] **Step 1: Confirm Figma authentication immediately before writing**

Call the Figma identity tool.

Expected: handle `Steven Chen`; plan key `team::1658127778171681639`; Pro Full seat.

- [ ] **Step 2: Load the mandatory new-file instructions and create the test file**

Read `figma-create-new-file/SKILL.md` completely, then call the Figma new-file tool with:

```json
{
  "editorType": "design",
  "fileName": "Codex Design Sandbox",
  "planKey": "team::1658127778171681639"
}
```

Expected: a new Figma design file key and URL.

- [ ] **Step 3: Inspect the blank file before writing**

Call the metadata tool with only the returned `fileKey`.

Expected: the file exposes its top-level page list and contains no user design content.

- [ ] **Step 4: Load mandatory Figma write references**

Read the `figma-use` skill, `plugin-api-standalone.index.md`, and `gotchas.md` before calling the write tool. Use `skillNames: "figma-use"`.

- [ ] **Step 5: Create the status card in one bounded write**

Call the Figma write tool with the returned `fileKey`, description `Create the Codex–Figma readiness card`, and this JavaScript:

```js
const available = await figma.listAvailableFontsAsync();
const regular = available.find(f => f.fontName.family === 'Inter' && f.fontName.style === 'Regular')?.fontName;
const semibold = available.find(f => f.fontName.family === 'Inter' && f.fontName.style === 'Semi Bold')?.fontName;
if (!regular || !semibold) throw new Error('Required Inter fonts are unavailable');
await figma.loadFontAsync(regular);
await figma.loadFontAsync(semibold);

const card = figma.createAutoLayout('VERTICAL', { name: 'Figma ↔ Codex Ready', itemSpacing: 16 });
card.resize(560, 320);
card.primaryAxisSizingMode = 'AUTO';
card.counterAxisSizingMode = 'FIXED';
card.x = 160;
card.y = 160;
card.paddingTop = 32;
card.paddingRight = 32;
card.paddingBottom = 32;
card.paddingLeft = 32;
card.cornerRadius = 24;
card.fills = [{ type: 'SOLID', color: { r: 0.965, g: 0.976, b: 0.992 } }];
card.strokes = [{ type: 'SOLID', color: { r: 0.773, g: 0.82, b: 0.91 } }];
card.strokeWeight = 1;

const eyebrow = figma.createText();
eyebrow.fontName = semibold;
eyebrow.fontSize = 13;
eyebrow.letterSpacing = { unit: 'PERCENT', value: 8 };
eyebrow.characters = 'CODEX DESIGN TOOL';
eyebrow.fills = [{ type: 'SOLID', color: { r: 0.19, g: 0.36, b: 0.72 } }];
eyebrow.textAutoResize = 'HEIGHT';
eyebrow.resize(496, eyebrow.height);
card.appendChild(eyebrow);
eyebrow.layoutSizingHorizontal = 'FILL';

const title = figma.createText();
title.fontName = semibold;
title.fontSize = 32;
title.lineHeight = { unit: 'PIXELS', value: 40 };
title.characters = 'Figma ↔ Codex Ready';
title.fills = [{ type: 'SOLID', color: { r: 0.07, g: 0.12, b: 0.22 } }];
title.textAutoResize = 'HEIGHT';
title.resize(496, title.height);
card.appendChild(title);
title.layoutSizingHorizontal = 'FILL';

const body = figma.createText();
body.fontName = regular;
body.fontSize = 16;
body.lineHeight = { unit: 'PIXELS', value: 25 };
body.characters = 'Codex can inspect, create, refine, and verify this Figma canvas through the authenticated design workflow.';
body.fills = [{ type: 'SOLID', color: { r: 0.27, g: 0.33, b: 0.43 } }];
body.textAutoResize = 'HEIGHT';
body.resize(496, body.height);
card.appendChild(body);
body.layoutSizingHorizontal = 'FILL';

const status = figma.createAutoLayout('HORIZONTAL', { name: 'Connection Status', itemSpacing: 10 });
status.paddingTop = 10;
status.paddingRight = 14;
status.paddingBottom = 10;
status.paddingLeft = 14;
status.cornerRadius = 999;
status.fills = [{ type: 'SOLID', color: { r: 0.86, g: 0.96, b: 0.9 } }];
card.appendChild(status);

const dot = figma.createEllipse();
dot.resize(10, 10);
dot.fills = [{ type: 'SOLID', color: { r: 0.09, g: 0.62, b: 0.33 } }];
status.appendChild(dot);

const statusText = figma.createText();
statusText.fontName = semibold;
statusText.fontSize = 14;
statusText.characters = 'Authenticated · Write enabled';
statusText.fills = [{ type: 'SOLID', color: { r: 0.07, g: 0.38, b: 0.2 } }];
status.appendChild(statusText);

await card.screenshot();
return {
  createdNodeIds: [card.id, eyebrow.id, title.id, body.id, status.id, dot.id, statusText.id],
  rootNodeId: card.id,
  size: { width: card.width, height: card.height }
};
```

Expected: the tool returns seven created node IDs, one root node ID, a 560-pixel card width, and an inline screenshot without an error.

- [ ] **Step 6: Validate structure and visual output independently**

Call metadata and screenshot tools with the returned `fileKey` and `rootNodeId`.

Expected metadata: one top-level auto-layout frame named `Figma ↔ Codex Ready`, containing three text nodes and one status auto-layout row with an ellipse and text child.

Expected screenshot: no clipped text, overlap, unexpected wrapping, or misalignment; status badge fits its content; all content stays inside the card.

- [ ] **Step 7: Run final local checks and report handoff**

Run:

```bash
codex --version
codex doctor --summary --no-color
codex plugin list | grep 'figma@openai-curated'
zsh -lic 'node --version; npm --version; command -v node; command -v npm'
git status --short
```

Expected: updated Codex; Figma CLI plugin installed and enabled; Node 24/npm paths coherent; only the user's original uncommitted product changes remain; all plan-owned repository files are committed.

Report the Figma file URL, root node ID, created local files, backup paths, committed revisions, and any non-blocking doctor warnings.
