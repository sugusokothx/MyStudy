承知いたしました。
会社のセキュリティポリシー（WSL/Docker禁止）と、チームでの環境標準化という要件を踏まえて、Windowsネイティブ環境で完結する、新人研修にも使えるPCセットアップ手順を再構成します。

-----

### \#\# 会社PC向け Python開発環境 導入手順書

この手順書は、会社のWindows PCにPython開発環境とObsidianをセットアップするためのものです。
`winget`というツールを使い、コマンドをコピー＆ペーストするだけで、誰でも同じ環境を構築できるようにしています。

#### \#\#\# 導入の考え方

  * **WSL/Dockerは使いません**: 会社のポリシーに従い、すべてのツールをWindows上で直接動かします。
  * **`pyenv-win`でPythonを管理**: チーム全員が「Python 3.11.9」のように、全く同じバージョンを使うことを保証し、「自分のPCでだけ動かない」といったトラブルを防ぎます。
  * **`winget`で効率化**: コマンド一つで関連ツールをまとめてインストールし、手作業によるミスを減らします。

-----

### \#\# ステップ1：基本ツールのインストール (PowerShell)

まず、開発に必要となる基本的なツールをまとめてインストールします。
**管理者としてPowerShellを開き**、以下のコマンドをコピーして貼り付け、実行してください。

> ℹ️ **管理者としてPowerShellを開く方法**
> `スタートボタン`を右クリックし、`[ターミナル (管理者)]` または `[Windows PowerShell (管理者)]` を選択します。

```powershell
# === 必須ツールのインストール ===
# Windows Terminal: 高機能なコマンド実行ツール
winget install --id Microsoft.WindowsTerminal -e

# Git: バージョン管理システム
winget install --id Git.Git -e

# Visual Studio Code (VS Code): 定番のコードエディタ
winget install --id Microsoft.VisualStudioCode -e

# Obsidian: メモ・ドキュメント管理ツール
winget install --id Obsidian.Obsidian -e

# === Pythonバージョン管理ツール ===
# pyenv-win: Windowsでpyenvを使うためのツール
winget install --id pyenv.pyenv-win -e

Write-Host "基本的なツールのインストールが完了しました。PowerShellを一度閉じて、新しいウィンドウで開いてください。" -ForegroundColor Green
```

コマンド実行後、一度PowerShellを閉じてから、**管理者ではない通常のPowerShell**を新しく開いて、次のステップに進んでください。

-----

### \#\# ステップ2：Pythonのインストールと設定

次に、`pyenv-win`を使って、チームで指定されたバージョンのPythonをインストールします。
ここでは例として `3.11.9` を使いますが、プロジェクトの指定に合わせてバージョン番号を変更してください。

新しいPowerShellで、以下のコマンドを一行ずつ実行します。

```powershell
# 1. pyenv-win のバージョンを更新（初回は不要ですが念のため）
pyenv update

# 2. 指定バージョンのPythonをインストール
# ※時間がかかります。気長に待ってください。
pyenv install 3.11.9

# 3. PC全体で使うPythonのバージョンとして設定
pyenv global 3.11.9

# 4. 設定が反映されたか確認
# "3.11.9" と表示されれば成功です
python --version
pip --version
```

> ⚠️ **トラブルシューティング**
> `python` コマンドが見つからないとエラーが出た場合は、環境変数PATHが正しく設定されていない可能性があります。PCを**再起動**すると解決することがほとんどです。

-----

### \#\# ステップ3：VS Codeの初期設定

VS Codeが、先ほどインストールしたPythonを正しく認識できるように設定します。

1.  **VS Codeを起動**します。
2.  左側のメニューから**拡張機能**（四角いブロックのアイコン）をクリックします。
3.  検索バーに `Python` と入力し、Microsoft製の拡張機能 **(Python Extension for Visual Studio Code)** をインストールします。
4.  インストール後、VS Codeを再起動します。

これで、Pythonのコードを書く際に、自動補完やエラーチェックが機能するようになります。

-----

### \#\# ステップ4：プロジェクトの準備（実際の業務で行う作業）

チームのプロジェクトに参加する際、以下の手順で自分のPC内に作業環境を準備します。
これはプロジェクトごとに一度だけ行います。

1.  **プロジェクトフォルダへ移動**
    `cd` コマンドを使って、作業したいフォルダに移動します。

    ```powershell
    # 例: Cドライブのprojectsフォルダの中のmy-projectに移動する場合
    cd C:\projects\my-project
    ```

2.  **プロジェクト専用の仮想環境を作成**
    プロジェクトで使うライブラリをPC本体から隔離し、クリーンに保つための「部屋」を作ります。

    ```powershell
    # 「.venv」という名前の仮想環境（部屋）を作成
    python -m venv .venv
    ```

3.  **仮想環境を有効化（部屋に入る）**
    これ以降のコマンドが、このプロジェクト専用の環境で実行されるようになります。

    ```powershell
    # 仮想環境を有効にする
    .\.venv\Scripts\Activate.ps1
    ```

    実行後、コマンドプロンプトの行頭に `(.venv)` と表示されれば成功です。

4.  **必要なライブラリをインストール**
    チームで共通のライブラリリスト (`requirements.txt`) を使って、一括でインストールします。

    ```powershell
    pip install -r requirements.txt
    ```

これで、チームメンバーと全く同じ開発環境で作業を開始できます。
VS Codeでこのフォルダを開けば (`code .` と入力)、自動的に `.venv` のPython環境が認識されます。