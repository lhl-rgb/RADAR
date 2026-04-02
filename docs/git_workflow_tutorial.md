# Radar 项目 Git 版本管理教程（新手实操版）

## 1. 你先记住这 4 个区
- 工作区（Working Tree）：你正在改的文件。
- 暂存区（Stage/Index）：准备提交的文件清单。
- 本地仓库（Local Repo）：你在本机的提交历史。
- 远程仓库（Remote Repo）：例如 GitHub/GitLab 上的仓库。

一句话理解：
`工作区 -> 暂存区 -> 本地提交 -> 推送远程`

## 2. 第一次初始化（本项目）
```bash
cd /home/lhlj/Radar
git init -b main
# 如未设置身份，先设置（建议全局）
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"

git add .
git commit -m "chore: initialize repository"
```

如果你已经有远程仓库：
```bash
git remote add origin <你的仓库地址>
git push -u origin main
```

## 3. 日常开发标准流程（最重要）

### 3.1 开始一个需求/修复
```bash
git checkout main
git pull
git checkout -b feature/noise-module
```

### 3.2 开发中如何提交
每完成一个“可说明的最小功能块”就提交一次，例如：
- 新增一个完整接口并可编译
- 修复一个 bug 并补了测试
- 完成一组参数重构并验证通过

命令：
```bash
git status
git add <文件1> <文件2>
git commit -m "feat(noise): add power/variance tests"
```

### 3.3 什么时候 push？
建议按下面节奏：
- 本地有 1~3 个有效提交后
- 一组测试通过后（至少 `radar_tests` 通过）
- 下班前 / 切换电脑前
- 需要别人 review 前

命令：
```bash
git push
```
首次推送分支：
```bash
git push -u origin feature/noise-module
```

## 4. 回退怎么做（按场景选）

### 场景 A：文件改乱了，还没 `git add`
丢弃工作区修改（危险，无法恢复）：
```bash
git restore <文件>
# 全部文件
git restore .
```

### 场景 B：已经 `git add`，但还没 commit
先撤销暂存，再决定要不要丢工作区：
```bash
git restore --staged <文件>
# 如还要丢弃工作区
git restore <文件>
```

### 场景 C：已经 commit，但还没 push
保留修改，只撤销提交：
```bash
git reset --soft HEAD~1
```
撤销提交并取消暂存（修改还在工作区）：
```bash
git reset --mixed HEAD~1
```
彻底回退（修改丢失，慎用）：
```bash
git reset --hard HEAD~1
```

### 场景 D：已经 push 到远程
不要 `reset --hard` 强推覆盖历史，优先用“反向提交”：
```bash
git revert <commit_id>
git push
```

## 5. 常用排查命令
```bash
git status                 # 当前状态
git log --oneline --graph  # 简洁提交图
git diff                   # 工作区差异
git diff --staged          # 暂存区差异
git branch                 # 分支列表
git remote -v              # 远程地址
```

## 6. 推荐提交信息模板
- `feat(noise): add IQ variance validation`
- `fix(scan): guard empty beam table`
- `test(noise): verify ThermalKTB power stats`
- `docs(git): add rollback guide`

## 7. 你这个项目的实用工作习惯
- 每次改完先跑：
```bash
./out/build/test/radar_tests
```
- 测试通过再 commit。
- commit 保持小而清晰，不要攒很大一坨。
- 不确定要不要推时，先 push 到你的 feature 分支，不直接动 `main`。

## 8. 出事故后的“救命命令”
误操作后先看历史指针：
```bash
git reflog
```
找到目标位置后可恢复：
```bash
git reset --hard <reflog里的哈希>
```

> 注意：`reset --hard` 会丢本地未保存修改，只在你确认后使用。
