# 文档约定

1. **所有图一律用 Mermaid**（`flowchart` / `sequenceDiagram` / `stateDiagram-v2` / `gantt` 等），
   不写 ASCII art、不贴外部图片。
   - 架构/数据流 → `flowchart`
   - 时序/状态迁移 → `sequenceDiagram`
   - 状态机 → `stateDiagram-v2`
2. 图里节点标题用中文 + 模块名（例如 `配准<br/>matcher`），便于对照代码。
3. 参数表用 Markdown 表格，标出**单位**与**默认值**。
4. 结论必须能被日志/指标复现：写数字，不写“差不多”“明显”。
5. 设计文档放 `docs/superpowers/specs/YYYY-MM-DD-<topic>-design.md`；
   现场复盘/审查结论放 `docs/dev/`。
