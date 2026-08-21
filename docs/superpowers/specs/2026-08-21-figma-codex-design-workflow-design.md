# Figma–Codex 设计工具工作流

## 目标

让用户以 Codex 桌面端为主要入口，把 Figma 当作可由 Codex 直接创建、修改、检查和迭代的设计画布。CLI 作为备用入口。默认方向是 Codex → Figma；除非用户明确要求，不把设计同步回项目代码。

完成后应满足以下条件：

- Codex 桌面端能够使用已经认证的 Figma 插件执行读写操作。
- Codex CLI 安装并启用同一官方 Figma 插件，重新启动会话后可发现其技能。
- 项目拥有持久的 Figma 操作规则和可复制的日常提示词。
- Node/npm 工具链满足当前项目和本地验证命令的版本要求。
- 一个独立的 Figma 测试文件完成创建、写入、截图和结构检查的端到端验证。

## 已确认的现状

- Figma 桌面应用已安装，版本为 `126.7.10`。
- Figma 插件在当前 Codex 桌面任务中已安装、启用并完成认证。
- 当前 Figma 账号拥有 Pro 计划的 Full seat。
- Figma 的应用权限继承全局“Allow low-risk actions”设置。
- Codex 全局配置可以正常解析，当前仓库已被标记为可信项目。
- Codex CLI 为 `0.139.0`，`codex doctor` 报告 `0.149.0` 可用。
- CLI 本地插件目录尚未安装 `figma@openai-curated`。
- 仓库没有项目级 `.codex/config.toml` 或根级 `AGENTS.md`。
- 当前 `node` 为 `20.15.0`，低于项目声明的 `>=20.19.0`；当前 npm 11 也不支持该 Node 版本。
- 仓库存在用户的未提交改动，实施不得把这些改动混入配置文档提交。

## 方案选择

采用“桌面插件优先，CLI 插件补齐，暂不增加第二套 Figma MCP”的方案。

不直接添加 `[mcp_servers.figma]`，因为当前桌面插件已经提供经过认证的 Figma 读写工具。重复配置远程或本地 MCP 会增加重复工具、额外认证和故障定位成本。只有在未来明确需要 IDE 或独立 MCP 客户端访问 Figma 时，才单独增加官方远程 MCP。

## 配置层次

### 全局 Codex 层

1. 在任何升级前备份 `~/.codex/config.toml`。
2. 使用 Codex 自带更新命令升级 CLI，并复查 `codex doctor`。
3. 使用 `codex plugin add figma@openai-curated` 安装 CLI 插件。
4. 确认全局配置中 Figma 插件处于启用状态；不保存静态 Figma Token。
5. 保留现有 Figma 桌面插件认证和低风险自动许可设置。

### 本地运行时层

通过 `brew install node@24` 安装 Node.js 24 LTS，并在备份 `~/.zshrc` 后将 `export PATH="/opt/homebrew/opt/node@24/bin:$PATH"` 添加到文件末尾，确保它不被现有 ROS 2 PATH 重置覆盖。`node` 与 `npm` 必须来自同一 Homebrew 前缀。保留回滚前的路径信息，不覆盖或删除旧二进制。验证以下命令输出一致：

```bash
node --version
npm --version
which node
which npm
```

Node 24 LTS 同时满足 npm 11 和本仓库的最低 Node 版本要求。

### 项目层

在仓库根目录增加简洁的 `AGENTS.md`，包含：

- 默认把 Figma 当作设计画布，除非明确要求，不修改项目代码。
- 写入前先检查目标文件中的页面、组件、变量和命名规则。
- 结构化容器使用 Auto Layout；复用已有组件、样式和变量。
- 将大操作拆成小批次；每次返回所有创建或修改的节点 ID。
- 文本修改必须先加载字体，异步调用必须等待完成。
- 写入后先检查节点结构，再用截图验证视觉结果。
- Figma 工具失败时先阅读错误并检查文件状态，不盲目连续重试。
- 只有用户明确提出 Figma → Code 时，才读取设计上下文并修改代码。

另外增加 `docs/figma-codex-workflow.md`，记录常用入口、设计提示词、修改提示词、复查提示词、链接格式和故障排查步骤。

项目暂不增加 `.codex/config.toml`。当前需求不需要仓库覆盖用户的模型、权限或 MCP 设置；项目行为由 `AGENTS.md` 表达即可。

## 工作流

### 新建设计

1. 用户说明设计目标、画布类型和输出范围。
2. Codex 创建或选择专用 Figma 文件。
3. Codex 读取页面、组件、变量和可用字体。
4. Codex 先创建顶层结构和占位区域。
5. Codex 分区填充内容，使用 Auto Layout、组件和变量。
6. 每个阶段返回节点 ID，并执行结构和截图检查。
7. Codex 修复发现的问题，给出最终 Figma 文件和节点结果。

### 修改现有设计

1. 用户提供 Figma 文件链接；修改具体节点时优先提供带 `node-id` 的链接。
2. Codex 先检查当前结构和视觉状态。
3. Codex 只修改目标节点及必要依赖，不重建整个页面。
4. Codex 复查层级、尺寸、文本裁切、重叠、间距和对齐。

### Figma → Code

该方向不是默认行为。只有用户明确提出实现代码时，Codex 才读取 `get_design_context`，复用仓库组件与 CSS 变量，并执行测试和视觉一致性复查。

## 错误处理与回滚

- Codex 更新失败：保留现有 CLI，记录错误并继续使用桌面端。
- CLI 插件安装失败：不修改 Figma OAuth 状态；检查市场源后重试一次。
- Node 安装失败：保留原 `/usr/local/bin/node`，不删除旧运行时。
- Figma 写入失败：依赖写入工具的原子性；检查错误、文件类型、页面和字体后再重试。
- 视觉验证失败：只修复不一致节点，不重新生成整个文件。
- 配置文件修改异常：用带时间戳的备份恢复 `~/.codex/config.toml`。

## 验证方案

### Codex 与插件

```bash
codex --version
codex doctor --summary
codex plugin list
```

检查 Figma 插件已安装并启用，Codex 配置和认证正常。非交互式诊断产生的 `TERM=dumb` 提示不视为 Figma 故障。

### 项目运行时

```bash
node --version
npm --version
npm run typecheck
```

如果实施没有修改产品代码，不运行会覆盖或格式化用户工作区的命令。

### Figma 端到端冒烟测试

1. 再次执行 Figma 身份检查。
2. 创建独立文件 `Codex Design Sandbox`。
3. 创建一个使用 Auto Layout 的 `Figma ↔ Codex Ready` 状态卡片。
4. 返回创建的文件和节点 ID。
5. 检查节点层级、名称和尺寸。
6. 截图确认文字没有裁切，元素没有重叠，间距和对齐正确。

## 非目标

- 不把用户现有 Figma 文件批量迁移或重构。
- 不自动修改当前 React 应用的视觉设计。
- 不启用静态 Figma Token 或提交任何认证信息。
- 不启用插件中的草稿写入后 hook。
- 不提交或清理用户当前未提交的产品代码。
