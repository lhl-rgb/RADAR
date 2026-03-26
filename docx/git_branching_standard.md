# Radar Git 分支规范（执行版）

## 1. 分支角色
- `main`：稳定分支，只放可发布版本。
- `dev`：日常集成分支，所有功能先合入这里。
- `feature/*`：功能开发分支，从 `dev` 拉出，完成后合回 `dev`。
- `fix/*`：普通缺陷修复分支，从 `dev` 拉出，完成后合回 `dev`。
- `hotfix/*`：线上紧急修复分支，从 `main` 拉出，修复后同时回合 `main` 和 `dev`。
- `release/*`：发版整理分支，从 `dev` 拉出，验证通过后合入 `main`，再回合 `dev`。

## 2. 命名规则
- `feature/noise-engine`
- `feature/antenna-scan-refactor`
- `fix/csv-parse-error`
- `hotfix/main-crash`
- `release/v0.1.0`

## 3. 标准开发流程（以后固定执行）
1. 更新基础分支：
```bash
git checkout dev
git pull
```
2. 创建任务分支：
```bash
git checkout -b feature/<task-name>
```
3. 开发并小步提交：
```bash
git add <files>
git commit -m "feat(noise): xxx"
```
4. 本地验证（至少跑核心测试）：
```bash
./out/build/test/radar_tests
```
5. 推送任务分支：
```bash
git push -u origin feature/<task-name>
```
6. 合并策略：
- 常规任务：`feature/*` -> `dev`
- 发版：`dev` -> `release/*` -> `main`，然后 `main` 变更回合 `dev`

## 4. 禁止事项
- 不在 `main` 直接做功能开发。
- 不在一个提交里混入多个不相关需求。
- 已推送分支不做 `reset --hard + force push`（除非你明确确认且仅你自己使用）。

## 5. 回退规范
- 未 push 的提交回退：`git reset --soft/mixed HEAD~1`
- 已 push 的提交回退：优先 `git revert <commit>`
- 仅当你明确允许时，才使用破坏性回退（`reset --hard`）

## 6. 我们后续协作约定
- 我后续默认从 `dev` 拉 `feature/*` 分支开发。
- 每次改动先本地测试，再提交，再推送。
- 需要回退时优先用 `revert`，确保历史可追溯。
