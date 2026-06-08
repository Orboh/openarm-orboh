# AGENTS.md — Orboh × トヨタ車体研究所 オクラ収穫PoC 運用指示書

> **AIエージェントへ:** あなたが AI コーディング/ドキュメントエージェント（Claude Code / Cursor / GitHub Copilot / Codex / Windsurf / Aider / Gemini CLI / Zed / Devin など）なら、本書のルールを読んで従うこと。本書は下記3リポジトリ共通の運用指示書。各セクションがどのリポジトリに適用されるか明記してある。`§5 適用方法` に、本書を各ツールの設定に取り込む手順がある。

## 0. リポジトリ構成（3リポジトリ）

| リポジトリ | 用途 | Git運用 |
|---|---|---|
| `Orboh/toyota-body-orboh` | 共有 Obsidian vault（設計書・計画・議事録・index・PMタスク・共有ナレッジ） | obsidian-git が main に自動コミット&プッシュ（PR不要） |
| `Orboh/DimOS_base_G1-TOYOTA-BODY-` | G1 / dimOS 開発（ヒューマノイド全体） | **PR必須**（main直push禁止） |
| `Orboh/openarm-orboh` | OpenArm 開発 | **PR必須**（main直push禁止） |

## 1. PR徹底（dev repo のみ）

- dev repo（`DimOS_base_G1-TOYOTA-BODY-` / `openarm-orboh`）では **main に直接 push しない**。必ずブランチを切って commit → Pull Request を作成 → レビュー/マージする。
- main は branch protection で保護されている（直push不可）。`git checkout -b <branch>` → push → `gh pr create` の流れ。
- 例外: 共有vault `toyota-body-orboh` は obsidian-git が main に自動同期するため PR 不要。vault には branch protection をかけない。

## 2. 成果物・devlog の置き場（重複させない）

役割を分けることで二重管理を避ける:

- **コードに紐づく開発進捗** → dev repo の **PR**（コードの変更記録）。
- **全員が共有すべき散文ナレッジ**（ハード不良調査・実験知見・判断理由など、コード差分にならない知識） → vault `Free-space/dev-log/`（トピック別サブフォルダ可）。
- **個人の作業ログ・スクラッチ** → vault `Free-space/<自分の名前>/`。

PR が「何をどうコード変更したか」、vault devlog が「その日何が起き何を学んだか」を担う補完関係。

## 3. 共有vault への出力先ルール（知識ノートを生成する時）

vault にノートを出力する際は、種別に応じて下記の場所を**自分から提案**してから書くこと。

| 文書種別 | 置き場所 |
|---|---|
| 設計書・計画書・context 等の中核文書 | vault root |
| 議事録 | `meetings/YYYY-MM-DD-<トピック>.md` |
| 個人の作業ログ・スクラッチ | `Free-space/<名前>/` |
| 全員共有すべき散文ナレッジ | `Free-space/dev-log/`（トピック別サブフォルダ可） |
| コードに紐づく開発進捗 | dev repo の PR（vault に二重化しない） |
| PMタスク | Obsidian の Project Manager UI（`100-PM-plugin-config/` は手編集しない） |

- dev repo で作業中に vault へ書きたい場合、vault がそのマシンに clone されていないことがある。**vault のローカルパスが不明なら user に確認してから書く**こと。
- 作業の「正解doc」は vault にある。着手前に該当するものを読む（あれば）: `03-dev-plan-okra-v3`（計画）/ `04-context`（体制・制約）/ `課題リスト` / `02-DimOS_base_G1Design`（G1設計）。

## 4. フロントマター規約（vault の `.md`）

vault で人間が読む `.md`（`100-PM-plugin-config/` のPMタスクを除く）を新規作成・編集する際、YAMLフロントマターに**最低限この2つを必ず残す**:

```yaml
---
created_by: <作成者>   # 必須・不変（最初の作成者）。例: 上田 / Kento / Sota / Claude (Kento)
updated: YYYY-MM-DD    # 必須・編集のたびに当日へ更新
---
```

- `title` / `tags` / `status` / `created` 等は任意（あってよい）。
- 既存ファイルを編集したら `updated` を当日に更新する。`created_by` は変更しない。

### 変更点など複数行を書く時の YAML 破壊防止（重要）

過去に変更履歴をフロントマターに箇条書きして YAML が壊れ、Obsidian が赤いエラー表示になった事故がある。フロントマター内では:

- ✅ 値は1行スカラー、または複数行なら `|`（リテラルブロックスカラー、インデント統一）
- ❌ スカラー値の直後に `  - 箇条書き` をぶら下げない（スカラー + ブロックリストの混在は不正）
- ❌ 同じキーを2回書かない（キー重複は不正）
- 詳細な変更履歴は本文 `## 変更履歴` セクションに書く

## 5. 適用方法（ツール別）

本書はリポジトリ root に置けば多くのエージェントが自動で読む。ツール別の取り込み:

| ツール | 取り込み方 |
|---|---|
| Cursor / Copilot / Codex / Windsurf / Aider / Gemini CLI / Zed / Devin 等 | `AGENTS.md` を root から native に認識（追加設定不要） |
| Claude Code | 各自で `CLAUDE.md` を作り `@AGENTS.md` で本書を読み込む（CLAUDE.md は共有せず gitignore・各自設定） |
| Cursor（スコープ別ルールを足す場合・任意） | `.cursor/rules/*.mdc` に本書を転記 |
| その他 / 上記以外 | 各ツールの rules ファイルに本書の内容を転記 |

- **canonical は vault（`toyota-body-orboh`）の本 `AGENTS.md`**。dev repo 側の `AGENTS.md` はそのコピー。ルールを変える時は vault 版を編集し、dev repo 側へ再配布する。

## 6. 各自の設定ファイルに貼る最小版

カスタムエージェントの導入は不要。各メンバーは自分のAIエージェントの設定ファイル（Claude Code: `CLAUDE.md` / Cursor: `.cursor/rules/*.mdc` / その他: 各ツールの rules ファイル）に下記を貼れば足りる:

```text
# Orboh×TBR オクラPoC ドキュメントルール
- 知識ノート・議事録は共有vault `toyota-body-orboh` に作る（ローカルパス不明ならuserに確認）:
  - 中核文書(設計/計画/context) -> vault root
  - 議事録 -> meetings/YYYY-MM-DD-<トピック>.md
  - 個人の作業ログ -> Free-space/<自分の名前>/
  - 全員共有の散文ナレッジ -> Free-space/dev-log/
  - コードに紐づく進捗 -> dev repo の PR（vaultに二重化しない）
- vaultの.mdフロントマター必須: created_by（不変）, updated（編集日 YYYY-MM-DD）
- dev repo（DimOS_base_G1 / openarm-orboh）は main直push禁止・ブランチ+PR必須
```
