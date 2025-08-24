了解です！では **WSL/Docker 禁止・winget 可・pyenv-win 固定版**として、
新入社員向けに“貼るだけで動く”セットアップ手順（PowerShell スクリプト付き）にまとめ直しました。

# 会社PC 初期セットアップ（Python + Obsidian / Windowsネイティブ）

## 手順のゴール

* 全員が同じ Python バージョン（例：3.11.9）
* 既存の `requirements.txt`（社内テンプレ）でパッケージ統一
* winget のみ使用、WSL/Docker 不要

---

## 1) 管理者 PowerShell で一括セットアップ

管理者として PowerShell を開き、以下を丸ごと貼り付けて実行してください：

```powershell
# ==== 0) 事前確認 ====
winget --version

# ==== 1) 基本ツール ====
winget install --id Git.Git -e --source winget
winget install --id Microsoft.WindowsTerminal -e --source winget
winget install --id Microsoft.VisualStudioCode -e --source winget
winget install --id Obsidian.Obsidian -e --source winget

# ==== 2) pyenv-win ====
winget install --id pyenv.pyenv-win -e --source winget

# システム環境変数に pyenv パスが追加されるまで一度シェルを再起動するのが確実
# 以降は "新しい" PowerShell（管理者でなくてOK）を開いて実行してください
```

> ここで一度 PowerShell を閉じて、**新しい PowerShell（通常権限でOK）** を開いてください。

---

## 2) Python を統一インストール（例：3.11.9）

```powershell
# ==== 3) Python 統一バージョン ====
# バージョンはチームで決めた値に合わせて変更可
$PYVER = "3.11.9"
pyenv install $PYVER
pyenv global  $PYVER
python --version

# ==== 4) pip を最新化 ====
python -m pip install --upgrade pip
```

---

## 3) 依存パッケージの一括導入（社内テンプレ使用）

> 既存の `requirements.txt` をネットワーク共有・リポジトリなどから取得してある前提です。
> 例として `C:\dev\project\requirements.txt` に置いた場合：

```powershell
cd C:\dev\project
pip install -r requirements.txt
```

※ プロキシ環境がある場合は、必要に応じて `pip.ini` を設定してください（例）：

```powershell
# C:\Users\<you>\AppData\Roaming\pip\pip.ini を作成
# （ない場合はフォルダも作る）
notepad "$env:APPDATA\pip\pip.ini"
```

pip.ini の例（必要な場合のみ）：

```
[global]
proxy = http://user:pass@proxy.example.com:8080
timeout = 60
```

---

## 4) VS Code 推奨拡張

VS Code を起動 → 拡張からインストール：

* Python（Microsoft）
* Pylance
* Jupyter（必要な場合）
* （任意）GitLens, Remote Repositories

コマンドで入れる場合：

```powershell
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-toolsai.jupyter
```

---

## 5) Obsidian

* Obsidian を起動し、**チームで決めた共有 Vault**（OneDrive/SharePoint/社内ファイルサーバ等）を開く
* プラグインやテーマはチーム標準に合わせて設定

---

# 運用メモ（チーム向け）

* **Python バージョン固定**：全員が `pyenv global 3.11.9` のように同一に。将来上げるときは「全員同日切り替え」をルールに。
* **パッケージ統一**：依存は必ず `requirements.txt` へ。追加時はPR・承認フローに。
* **PATH/優先順位**：`pyenv-win` の shims 経由になるため、古い Python が残っていて PATH が競合する場合は削除/無効化を推奨。
* **権限/プロキシ**：社内プロキシ下では pip の `proxy` 設定や証明書設定が必要な場合あり（上記参照）。
* **トラブル時の確認**：

  * `where python` で解決されているパスが `pyenv` の `shims` かを確認
  * `pyenv versions` でインストール済み一覧、`pyenv global` で有効版を確認
  * `pip debug -v` でインデックス解決や証明書周りを確認

---

# 付録：アンインストール/やり直し（必要時）

```powershell
# pyenv 経由の Python を消す（例：3.11.9）
pyenv uninstall 3.11.9

# pyenv-win 自体をアンインストール
winget uninstall --id pyenv.pyenv-win

# VS Code / Obsidian / Git のアンインストール（必要時のみ）
winget uninstall --id Microsoft.VisualStudioCode
winget uninstall --id Obsidian.Obsidian
winget uninstall --id Git.Git
```

---

必要なら、上記を **社内Wiki/Obsidianのテンプレページ** 用にそのまま貼れる形で整形してお渡しします。
Python の固定バージョン（例の 3.11.9）を別の値にしたい場合だけ、数値を教えてください。
