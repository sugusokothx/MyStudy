## 手順① — モデルワークスペースに変数を定義する

1. **Model Explorer** で対象モデルを開き、  
   **Model Workspace** タブを選択します。

2. ワークスペース上部の *“Script”* ペインに、次の MATLAB スクリプトを貼り付けて **Evaluate** します。  
   （実機データや既存ファイルを使う場合は、`load` 行だけを残して調整してください）

   ```matlab
   % ── ルックアップ軸 ────────────────────────────────
   psi_d_axis = linspace(-0.05, 0.05, 201);   % 1×Nx [Wb]
   psi_q_axis = linspace(-0.05, 0.05, 201);   % 1×Ny [Wb]

   % ── ルックアップ・テーブル (Ny×Nx) ────────────────
   % 例: 外部 *.mat* からロード
   load('motorMagnetisation.mat', 'IdTable', 'IqTable');
   % 上記ファイルに Ny×Nx の double 配列が含まれている前提

   % ── 定数パラメータ ───────────────────────────────
   Rs = 0.03;          % Stator resistance [Ω]
   Pn = 4;             % 極対数
   Ts = 1/20e3;        % 離散サンプル時間 [s]

## 手順② — MATLAB Function ブロックで各変数を「パラメータ」にバインドする

1. **Simulink** で対象の **MATLAB Function** ブロックを  
   ダブルクリックし、エディタ右上の **「Edit Data」**（または *Symbols* パネル）を開きます。

2. **新規シンボル**を追加し、下表のように **Scope** を **Parameter** に設定します。  
   *Size* と *Type* は “`Inherited (-1)`” にしておくと軸長が変わっても自動対応します。

   | Name           | Scope      | Type / Size*               | 説明                         |
   |----------------|-----------|----------------------------|------------------------------|
   | `psi_d_axis`   | Parameter | Inherited (`1 × Nx`)       | d 軸フラックス軸             |
   | `psi_q_axis`   | Parameter | Inherited (`1 × Ny`)       | q 軸フラックス軸             |
   | `IdTable`      | Parameter | Inherited (`Ny × Nx`)      | φd–φq → Id ルックアップ      |
   | `IqTable`      | Parameter | Inherited (`Ny × Nx`)      | φd–φq → Iq ルックアップ      |
   | `Rs`           | Parameter | `double` / `1 × 1`         | ステータ抵抗 [Ω]             |
   | `Pn`           | Parameter | `double` / `1 × 1`         | 極対数                       |
   | `Ts`           | Parameter | `double` / `1 × 1`         | サンプル時間 [s]             |

   \* **Size** は参考値です。`Inherited (-1)` のままでも可。

3. **Argument** 列を空欄にしたまま **Apply / OK** を押すと、 
   ```matlab
    i_d = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IdTable);
   ```
   コード中で変数名を直接参照できるようになります。

4. コード冒頭で使用していた

   ```matlab
   coder.extrinsic('evalin');
   Rs = evalin('base', 'Rs');  % … 等
   % パラメータは変数名を直接使用
   dpsi_d = v_d - Rs*i_d - omega*psi_q;
   ```
    のように書くだけで OK です。

## その他:Config 設定
   ![Config 設定][def]

   [def]: ./simulink_sample/Config_setting.png "Config 設定"
