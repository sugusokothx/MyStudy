# Simulink C Function ― Configuration Set 共有マニュアル  

> **対象バージョン**：R2020b 以降  
> **目的**：C Function ブロックの *Custom Code* 設定を含む **Configuration Set** を  
> 他モデルへコピー／参照させる方法をまとめる。

---

## 0. 構成セットの場所

- **Model Explorer** で  
  `モデル名 ▸ Configurations ▸ Configuration (Active)`  
  に格納されている。  
- **Simulation Target → Custom Code** の  
  - 追加ソース／ヘッダー  
  - インクルードパス  
  - プリプロセッサ定義  
  なども **すべて構成セット側に保存** される。

---

## 1. 方式比較

| 方式 | 共有イメージ | メリット | デメリット |
|------|-------------|----------|-----------|
| **コピー (Import / Export)** | 構成セットを複製してモデルに貼り付け | - モデルごとに独立運用<br>- 個別に編集しやすい | - 元ファイルを変更しても自動反映されない |
| **参照 (Configuration Reference)** | `*.sldcfg` をリンク | - 一か所変更で全モデルへ即反映<br>- 設定の統一が容易 | - 全モデルが同じバージョンに縛られる<br>- 個別差分を付けにくい |

---

## 2. 方法 A：コピーして取り込む（1 回だけ複製）

> **用途**：モデルごとに微調整が必要な場合。

### GUI 手順

1. **コピー元**モデルを開き、Model Explorer で  
   **Configuration (Active)** を右クリック → **Export**  
   - 保存形式：`*.sldcfg`（推奨）または `*.mat`
2. **コピー先**モデルを開き、Model Explorer で  
   **Configurations** を右クリック → **Import**  
3. 取り込んだ構成セットを右クリック → **Set Active**

### コマンドライン例

```matlab
% ── エクスポート ──────────────────────────
mdlSrc = 'tes_conf';
cfg    = getActiveConfigSet(mdlSrc);
save('CFunction_cfg.mat', 'cfg');

% ── インポート ──────────────────────────
mdlDst = 'myTargetModel';
load('CFunction_cfg.mat', 'cfg');
attachConfigSet(mdlDst, cfg, true);   % true: 名前衝突時に上書き
setActiveConfigSet(mdlDst, cfg.Name);

```


## 3. 方法 B：Configuration Reference で参照共有

> **用途**：複数モデルで **同一ビルド設定を維持** したいときに最適。  
> **特徴**：構成セットファイル（`.sldcfg`）をリンクするので、元ファイルを更新すると参照先すべてに即時反映されます。

### 3-1. GUI 手順

1. **コピー元**モデルで、Model Explorer  
   `Configuration (Active)` を右クリック → **Export…**  
   `CFunction_cfg.sldcfg` など任意名で保存。  
2. **参照先**モデルを開き、Model Explorer → **Configurations** を右クリック  
   → **Add ▸ Configuration Reference** を選択。  
3. **Source Name** に手順 1 で保存した `CFunction_cfg.sldcfg` を指定。  
4. 追加された *ConfigRef* を右クリック → **Set Active**。  
   以降はこの参照を通じて設定を共有します。

### 3-2. コマンドライン例

````matlab
mdl = 'myTargetModel';              % 参照先モデル

cfgRef = Simulink.ConfigSetReference;
cfgRef.SourceName = 'CFunction_cfg.sldcfg';  % 相対 / 絶対どちらでも可

attachConfigSet(mdl, cfgRef, true); % true: 名前衝突時に上書き
setActiveConfigSet(mdl, cfgRef.Name);

````

## 4. 依存ファイル・パスの注意点

| 項目 | 注意点 |
|------|-------|
| **ソース／ヘッダパス** | `Simulation Target → Custom Code` で指定したパスは **参照先 PC から見える相対パス** にするのが安全。 |
| **Toolchain / ハードウェア設定** | モデルごとに異なるとビルドエラーの原因に。共通化できない場合は「コピー方式」に切り替える。 |
| **旧バージョン (R2019a 以前)** | Configuration Reference 機能が限定的／未実装。MAT ファイルコピー運用が無難。 |

## 5. 方式比較まとめ

| 方式 | メリット | デメリット |
|------|----------|-----------|
| **コピー (Import / Export)** | - モデルごとに独立運用<br>- 個別編集が容易 | - 変更を配布するたび再コピーが必要 |
| **参照 (Configuration Reference)** | - 一か所変更で全モデルに反映<br>- 設定の統一を維持しやすい | - 全モデルが同じ設定に縛られる<br>- 個別差分を付けづらい |

