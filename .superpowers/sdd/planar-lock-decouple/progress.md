# SDD ledger — plan: docs/superpowers/plans/2026-11-12-planar-lock-decouple.md

Setup: 内联执行（用户选 native）。工作区=当前 git 工作树（HEAD + 已有未提交改动），
未新建 worktree：本地未提交改动即"最新程序"，另开 worktree 会丢掉它们。
提交策略：每个任务只 `git add` 它改到的文件，不整树提交。
Pre-flight: Task1→2 无共享接口；Task2→3 `updatePlanar(double,const M3D&,bool)`；
Task3→4 `setPlanarConstraint(bool,bool,double)` 与 `SlopeDetector`；一致，无冲突。

