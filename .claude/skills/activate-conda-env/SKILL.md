---
name: activate-conda-env
description: 提醒使用 llm conda 虚拟环境作为 Python 唯一运行环境
when_to_use: 当用户要求运行 Python 脚本、安装 pip 包、执行 pytest、或任何涉及 python/pip/conda 的操作时自动加载此 skill
shell: powershell
---

# Conda 环境规范

本项目所有 Python 相关操作必须在 `llm` conda 虚拟环境中执行。

## 推荐方式（无需 conda init）

`conda run` 不依赖 `conda init`，可直接在任何 shell 中使用：

1. 运行 Python 脚本：

   ```powershell
   conda run -n llm --no-capture-output python <script>
   ```

2. 运行模块：

   ```powershell
   conda run -n llm --no-capture-output python -m <module>
   ```

3. 安装包：

   ```powershell
   conda run -n llm --no-capture-output pip install <package>
   ```

4. 或者直接使用完整路径（同样无需 init）：

   ```powershell
   & "C:\Users\shy\anaconda3\envs\llm\python.exe" <script>
   ```

## 关于 conda activate

`conda activate llm` 需要先在当前 shell 中执行 `conda init powershell`（且重启 shell 生效）。
如果 shell 未经 init，`conda activate` 会报错。因此推荐使用上述 `conda run` 方式。

如果确实需要 activate，先执行：

```powershell
conda init powershell
# 重启 shell 后
conda activate llm
```

## 禁止

- 禁止使用裸 `python` 或 `python3` 命令
- 禁止使用裸 `pip` 或 `pip3` 命令
- 禁止激活或使用其他 conda 环境运行本项目代码

## 环境信息

- 环境名称: `llm`
- 路径: `C:\Users\shy\anaconda3\envs\llm`
