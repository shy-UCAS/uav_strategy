---
name: git-commit-message
description: 结构化 git commit message 模板，规范提交信息格式（Markdown 分节风格）
when_to_use: 当用户要求创建 git commit、提交代码、或讨论 commit message 写法时自动加载
---

# Git Commit Message 规范

所有 commit message 必须遵循 Conventional Commits 格式，使用 **Markdown 分节风格** 书写，描述使用中文。

## 格式模板

```markdown
# <type>[(scope)]: <subject>

## 变更说明

<为什么做这个改动，而非重复做了什么>

## 关联

- <Closes #xxx / Refs #xxx / BREAKING CHANGE: 说明>
```

### 各节说明

- `# 标题行`：必填。包含 type、可选 scope、subject
- `## 变更说明`：较大改动时添加，说明 **为什么** 做这个改动
- `## 关联`：按需添加，用列表罗列 issue 关联或 Breaking Change

简单提交可以只保留 `# 标题行`，省略其余小节。

## Type（必填）

| type     | 用途                     |
| -------- | ------------------------ |
| feat     | 新功能                   |
| fix      | 修复 bug                 |
| refactor | 重构（不改变外部行为）   |
| docs     | 文档变更                 |
| test     | 测试相关                 |
| chore    | 构建/工具/依赖等杂项     |
| style    | 格式调整（不影响逻辑）   |
| perf     | 性能优化                 |
| ci       | CI/CD 配置变更           |
| build    | 构建系统或外部依赖变更   |

## Scope（可选）

括号内填写受影响的模块名，可省略。示例：`gcjp`, `verifier`, `demos`, `agents`, `prompts`

## Subject 规范

- 用中文描述本次变更的核心内容
- 不超过 50 个字符
- 不加句号结尾
- 用祈使语气（"新增…"、"修复…"、"重构…"）

## 变更说明规范

- 与标题行之间用 `## 变更说明` 分隔
- 说明 **为什么** 做这个改动，而非重复 **做了什么**
- 每行不超过 72 个字符

## 关联规范

- 用 `## 关联` 小节，以列表形式罗列
- **Breaking Change**: `- BREAKING CHANGE: 说明`
- **Issue 关联**: `- Closes #123` 或 `- Refs #456`

## 行为规范

- **禁止自动提交**：只生成 commit message 并展示给用户审阅，绝对不要执行 `git commit`、`git add` 或任何 git 写操作
- 将生成的 commit message 以 Markdown 代码块形式输出，方便用户复制
- 用户确认或修改后，由用户自行执行提交
- **列出涉及文件**：在 commit message 代码块之后，以"涉及文件"小节列出本次提交涉及的所有文件路径（相对于仓库根目录），方便用户确认 `git add` 的范围
- **暂存提示**：在每个批次末尾给出可直接复制的 `git add` 命令，仅列出该批次涉及的文件
- **自动分批**：当工作区有多个变更文件时，先按"分批策略"聚类，再逐批输出；如果只有一个逻辑变更则省略批次编号

## 分批策略

当工作区存在多个变更文件时，必须分析文件间的逻辑关系，将属于**同一逻辑变更**的文件归入同一批次，分别生成独立的 commit message。

### 分组原则

1. **同一功能/特性**：实现代码 + 对应测试 + 对应文档 + 对应数据/配置变更 → 同一批次
2. **因果链变更**：A 的改动导致 B 必须同步更新（如 prompt 改动 → 重跑基线 → 更新报告） → 同一批次
3. **独立重构**：与功能改动无关的纯重构、格式化、杂项 → 单独批次
4. **不相关模块**：不同模块的独立改动 → 各自单独批次

### 分析流程

1. 读取所有变更文件的 diff，理解每个文件的变更意图
2. 按"变更意图"对文件聚类——意图相同或存在因果关系的归为一组
3. 对每组生成独立的 commit message
4. 按逻辑依赖顺序排列批次（被依赖的在前）

### 输出格式

每个批次用 `---` 分隔，包含：
- 批次编号与总批次数（如 `**批次 1/3**`）
- commit message 代码块
- 涉及文件列表
- 该批次可单独执行的 `git add` + `git commit` 命令提示

## 禁止事项

- 禁止添加 `Co-Authored-By` trailer
- 禁止使用 `#` 作为 subject 前缀（`#` 只用于 Markdown 标题语法）
- 禁止执行 `git commit`、`git add`、`git push` 等任何 git 写操作

## 示例

### 单行简洁提交

```markdown
# feat(gcjp): 新增 group_sync 多任务组同步约束
```

### 带变更说明的提交

```markdown
# refactor(verifier): 将错误码从字符串常量迁移到枚举类型

## 变更说明

统一错误码管理方式，便于 IDE 自动补全和类型检查，
同时消除跨模块引用时的拼写错误风险
```

### 完整格式提交

```markdown
# fix(agents): 修复流式响应在超时后未正确关闭连接的问题

## 变更说明

原实现在 timeout 异常路径中跳过了 response.close()，
导致连接池泄漏

## 关联

- BREAKING CHANGE: LLMClient.stream() 返回类型从 Generator 改为 AsyncGenerator
- Closes #42
```

### 省略 Scope 的提交

```markdown
# chore: 更新 .gitignore 忽略临时日志文件
```

### 完整输出示例（多批次 + 涉及文件）

假设工作区有 6 个文件变更，分析后分为 2 个批次：

````
**批次 1/2**

```markdown
# feat(prompts): 增强指令规范化 prompt，新增任务消耗字段

## 变更说明

为规范化引擎补充 duration_lb/energy_cost/ammo_cost 三要素，
区分任务消耗与资源约束语义，并增加动作归一化映射与同步集结示例
```

**涉及文件：**
- `prompts/instruction_normalization_prompt.md`
- `experiments/baselines/exp_01f_instruction_normalization.json`
- `docs/phase1_baseline_report.md`

> 暂存提示：`git add prompts/instruction_normalization_prompt.md experiments/baselines/exp_01f_instruction_normalization.json docs/phase1_baseline_report.md`

---

**批次 2/2**

```markdown
# refactor(experiments): 将基线报告时间戳精度提升至秒级
```

**涉及文件：**
- `experiments/phase1_common.py`

> 暂存提示：`git add experiments/phase1_common.py`
````

### 单批次输出示例

当所有文件属于同一逻辑变更时，省略批次编号，直接输出：

````
```markdown
# feat(gcjp): 新增 group_sync 多任务组同步约束
```

**涉及文件：**
- `gcjp/constraints.py`
- `tests/test_group_sync.py`

> 暂存提示：`git add gcjp/constraints.py tests/test_group_sync.py`
````
