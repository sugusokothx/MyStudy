### NaN を外挿で埋める 3 つの代表パターン

> **一番手軽なのは「scatteredInterpolant の ExtrapolationMethod を変える」だけ** です。
> さらに滑らかさを保ちたい場合は ②・③ を検討してください。

| #                       | 方法                         | 特徴                   | ひと言で |
| ----------------------- | -------------------------- | -------------------- | ---- |
| **① 最近傍外挿 (`nearest`)** | **追記 2 文字で済む**／外挿値 = 最近測定点 | 「まずは動かす」             |      |
| **② 線形外挿 (`linear`)**   | 端部で *線形に伸びる*／連続的だが斜面が急な時あり | MTPA 計算などで微分を使うなら推奨  |      |
| **③ inpaint + ガウス平滑**   | 不規則に欠けた穴も補完／滑らかさ自由度◎       | FileExchange 関数などを利用 |      |

---

## ① 追記 2 文字で最速解決

```matlab
% --- 補間器作成 (ExtrapolationMethod = 'nearest') -------------
Fd = scatteredInterpolant(id, iq, phid, 'natural', 'nearest');
Fq = scatteredInterpolant(id, iq, phiq, 'natural', 'nearest');
```

* `'nearest'` の部分だけ変更（以前は `'none'`）
* `phi_d_map = Fd(ID, IQ);` 以降はそのまま

**メリット**

* 処理 0.0 秒増で NaN 消滅
* 外挿値は「最後に測定した値のコピー」なので暴れにくい

**デメリット**

* 測定レンジを一歩出た瞬間に *勾配が 0* になる（微分が不連続）

---

## ② 線形外挿で連続性を保つ

```matlab
Fd = scatteredInterpolant(id, iq, phid, 'natural', 'linear');
Fq = scatteredInterpolant(id, iq, phiq, 'natural', 'linear');
```

* `'linear'` は三角形メッシュをそのまま延長
* 端部でも φ 面の傾きが *途切れない* ので、後段で Ld/Lq の微分を取っても “段差” が出にくい

---

## ③ inpaint\_nans（File Exchange）＋ガウス平滑

```matlab
phi_d_map(isnan(phi_d_map)) = 0;   % 初期化 (外挿領域を 0 で仮埋め)
phi_q_map(isnan(phi_q_map)) = 0;

phi_d_map = inpaint_nans(phi_d_map, 4);  % Laplacian ベース
phi_q_map = inpaint_nans(phi_q_map, 4);

% 念のため周辺を軽く平滑化
sigma = 1;
phi_d_map = imgaussfilt(phi_d_map, sigma, 'Padding','replicate');
phi_q_map = imgaussfilt(phi_q_map, sigma, 'Padding','replicate');
```

* **inpaint\_nans.m** は 1 ファイルで使える有名ツール
* “外挿” ではなく “内挿延長” なのでスムース
* 追加の ToolBox 不要（Signal Processing TB があれば `imgaussfilt` のみ）

---

### 実装後のチェックポイント

1. **元データの Convex Hull** を `trisurf` 等で描き、
   埋めた領域が *どこまで* 制御器に使われるか把握する
2. MTPA / トルク試験用シミュレーションを 1 周だけ流して、
   **Ld・Lq の外挿値が滑らかか** をプロット確認
3. もし「端部は絶対に使わない」なら、**外挿せず NaN のまま** にして
   制御側で **電流飽和** を掛ける方が安全な場合も

---

## まとめ

* **最速で NaN を解消** → `'nearest'` に変更
* **微分連続性も欲しい** → `'linear'`
* **見た目滑らかに仕上げたい** → `inpaint_nans` + 平滑化

まずは ① を試して挙動を確認し、必要に応じて ②・③ にアップグレードすると手間が少なく済みます。


