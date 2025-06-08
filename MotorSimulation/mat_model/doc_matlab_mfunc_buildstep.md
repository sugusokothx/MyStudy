## 事前準備

1. **ライセンス確認**  
   - MATLAB Coder および Embedded Coder のライセンスが有効であることを確認。

2. **モデル設定の確認**  
   - モデルが動作すること（シミュレーションエラーなし）を確認。
   - MATLAB Function ブロック内は `#codegen` 指定済みであること。

---

## 手順

1. **モデルの設定を開く**  
   Simulink モデルウィンドウ上部のメニューから  
   `Simulation` → `Model Configuration Parameters...` を選択。

2. **システムターゲットファイルを変更**  
   - 左ペインで **Code Generation** → **System target file**  
   - ドロップダウンから `ert.tlc` を選択。  
     - （`ert.tlc` は Embedded Coder 用のリアルタイムターゲットです）

3. **コード生成オプションを設定**  
   同じく **Code Generation** ペインで：  
   - **Language** を `C` に。  
   - **Generate code only** のチェックを外す（実行可能ファイルまでビルドする場合）。  
   - **Toolchain** はデフォルトの `Automatically locate an installed toolchain` で OK。  
   - **Build configuration** を `Faster Runs` や `Faster Builds` に適宜設定。

4. **MATLAB Function ブロックの設定確認**  
   各 MATLAB Function ブロックをダブルクリック →  
   右上の **Function Block Properties** で  
   - **Support code generation** が有効になっていること。  
   - 必要に応じて **Interface** タブでデータ型・サイズを固定化しておく。

5. **モデルを保存してビルド**  
   - モデルウィンドウ上で ⌘+B（または Ctrl+B）を押す。  
   - 下部の Diagnostic Viewer に、コード生成～コンパイルログが流れ、  
     最終的に `<model_name>_ert_rtw` フォルダに C ソースや Makefile が生成され、  
     実行バイナリがビルドされます。

6. **生成コードの確認**  
   - `model_name_ert_rtw` フォルダ内の  
     `model_name.c` / `model_name.h` をエディタで開く。  
   - MATLAB Function ブロック由来の関数が自動生成されていることを確認。

7. **リアルタイム／ハードウェアでの実行（任意）**  
   - 必要であれば **Toolchain** をターゲットボード向けに変更して再ビルド。  
   - 生成されたバイナリをターゲットにダウンロードし、リアルタイム実行。

---

## ポイント解説

- **ert.tlc** を選ぶことで、デフォルトの Simulink Coder よりも  
  軽量で実機／リアルタイム向けのコードが生成されます。
- **Build configuration** を `Faster Runs` にすると、最適化されたコードが生成され、  
  シミュレーション実行時の速度が向上します。
- ビルド後、Model Explorer から **Code Generation Report** を開くと、  
  ブロックごとのコードサイズや実行時間予測が可視化できます。

以上で、MATLAB Function ブロックを含むモデルを丸ごと C/C++ コード化し、  
ネイティブにビルド＆実行できるようになります。
