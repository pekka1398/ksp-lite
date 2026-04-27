# Kerbin Rotation 教訓

## 問題

要在 Kerbin 上實現自轉效果，同時保持 Rapier 物理穩定（火箭在發射台上不噴飛）。

## 嘗試過的方案（全部失敗）

### 1. Prelaunch visual-only（原始 staged changes）
火箭 spawn 時無 RigidBody/Collider，按 Space 時動態插入。
**結果：** 插入瞬間 Rapier 偵測到穿透 → 爆炸式修正力 → 火箭噴飛。

### 2. KinematicPositionBased → Dynamic
火箭 spawn 為 Kinematic，每幀覆蓋 Transform，按 Space 時改成 Dynamic。
**結果：** Rapier 從位置差異推算出內部運動學速度（~50 m/s），切換 Dynamic 時這個速度被保留 → 火箭噴飛。

### 3. Always-Dynamic + SurfaceLocked 每幀覆蓋 Transform
火箭一出生就是 Dynamic + Velocity::zero，SurfaceLocked 期間每幀強制覆蓋 Transform 和 Velocity。
**結果：** Rapier 偵測到 Dynamic body 的 Transform 每幀跳變 → 從位置差異推算出速度 → 移除 SurfaceLocked 後火箭噴飛。

### 4. Dynamic + Kerbin child（Transform propagation）
火箭 spawn 為 Kerbin 的 child，靠 Bevy 的 Transform propagation 自動跟轉。
**結果：** 火箭的 Transform 變成 Kerbin 本地座標，所有讀 rocket Transform 的系統（camera、telemetry、readback）全部壞掉。Rapier 對 parent hierarchy 裡的 Dynamic body 行為也不確定。

## 根本原因

**Rapier 和旋轉的天體 Transform 衝突。** 不管怎麼包裝，只要 Kerbin 的 Transform 在轉、火箭是 Dynamic，就會有問題：

- Dynamic body 在旋轉的 static collider 旁邊 → 慣性力/摩擦力方向不對
- 每幀覆蓋 Dynamic body 的 Transform → Rapier 偵測到跳變 → 推算出巨大速度
- 切換 RigidBody 類型 → 內部狀態殘留

**Principia 的做法：** 物理永遠在慣性系裡跑（自己的 N-body 積分器），參照系只是顯示層的變換。它不用 Rapier。

**Seb Lague Solar System 的做法：** 星球根本不自轉。N-body + Rigidbody.MovePosition。

## 最終方案：ECEF 物理 + 虛擬力

**Rapier 永遠在 ECEF（天體固定系）裡跑。** Kerbin Transform 不轉，發射台不動，火箭 Dynamic + Velocity::zero 穩穩站著。

太陽繞 Kerbin 轉（舊版做法）→ 畫夜交替。

手動加兩個非慣性力讓 ECEF 物理等價於 ECI：

1. **離心力** `F = m·ω²·(x, 0, z)` — 推離自轉軸，抵消部分重力
2. **科里奧利力** `F = 2m·(-ω·vz, 0, ω·vx)` — 運動物體偏轉

這兩個力加在 `compute_gravity_and_drag` 閉包裡，SOI body 的 `rotation_speed > 0` 時啟用。

### SimState 仍存 ECI 速度

`sim_state_readback_system` 讀 Rapier 的 Velocity（ECEF），加回 `ω×r` 得到 ECI 速度寫入 SimState。軌道計算、遙測、map view 都用 SimState（ECI），不受影響。

### Drag 計算

大氣跟著星球轉，ECEF 裡大氣靜止。Drag 用 Rapier 的 `vel.linvel`（ECEF 速度）= 相對大氣的速度。✓

### 視覺自轉

火箭在軌道上時，因為正確的相對運動（科里奧利力 + 離心力讓 ECEF 裡的軌跡正確），飛船往下看會看到地表特徵正確地移動 — **不需要轉貼圖或 Transform。**

---

## Map View 軌道旋轉 Bug 調查

### 問題描述

在 MapView 裡，Kerbin 星球有在轉（MapMesh 被 RotationAngle 驅動旋轉），但軌道預測線（gizmo 畫的橢圓）也跟著在轉——相對背景星空以大約 1.43°/s 的速率進動。

### 初步診斷：加入診斷 log

在 `orbit_prediction_system` 的 `OrbitalElements` 計算後加了 `pe_dir`（periapsis direction）的 log，每 2 秒印一次。

**結果（修復前）：**

| 時間 | pe_dir Z | 對應角度 |
|---|---|---|
| 0s | 0.1057 | 6.1° |
| 2s | 0.1545 | 8.9° |
| 4s | 0.2035 | 11.6° |
| 6s | 0.2512 | 14.5° |

進動率 ≈ 1.40°/s。而 KERBIN_ROTATION_SPEED = 0.025 rad/s = 1.43°/s。

**進動率幾乎完美等於 Kerbin 的自轉角速度 ω。** 這不是巧合。

### 排除法分析

我們問了三個可能的根源：

1. **Rapier 積分本身的誤差？** — Symplectic Euler 對分離哈密頓系統（只有位置相關的力）是保結構的，誤差是 O(dt) 但不會產生系統性進動。即使 Coriolis 力破壞 symplectic 結構，誤差也應該是隨機漂移，不可能精確等於 1.43°/s。

2. **潮汐力 / 引力梯度？** — 太陽 mu=0（零引力），Mun 在 12000m 外，潮汐加速度 ≈ 1e-4 m/s²，6 秒累積速度誤差 ~6e-4 m/s，對 108 m/s 軌道速度完全不會造成 8° 進動。

3. **參照系混用？** — **這才是真正原因。**

### 根本原因：參照系混用 (r_ECEF, v_ECI)

追蹤 `orbit_prediction_system` 的資料流：

```
rel_pos = rocket_sim.position - planet_sim.position    ← 位置
rel_vel = rocket_sim.velocity - planet_sim.velocity    ← 速度
OrbitalElements::from_state(rel_pos, rel_vel, mu)      ← 用混合數據算軌道元素
```

- **`rocket_sim.position`** 來自 `sim_state_readback_system`：`offset.ssb_from_local(tf.translation)`。`tf.translation` 是 Rapier 用 ECEF 速度更新的 → **位置是 ECEF**
- **`rocket_sim.velocity`** 來自 `sim_state_readback_system`：`vel.linvel.as_dvec3()` + `ω×r` 修正 → **速度是 ECI**
- **`planet_sim.position`** 來自 `celestial_orbit_system`：解析計算 → **ECI**（Kerbin 恆為零）
- **`planet_sim.velocity`** 來自 `celestial_orbit_system`：解析計算 → **ECI**

所以 `rel_pos = rocket_ECEF - planet_ECI`，`rel_vel = rocket_ECI - planet_ECI`。

**軌道元素是用 (r_ECEF, v_ECI) 算的——兩個不同參照系的資料混在一起。**

### 為什麼混用會造成精確 ω 進動

正確的軌道元素需要 (r_ECI, v_ECI)。ECEF 和 ECI 之間的關係是繞 Y 軸旋轉 θ=ωt：

```
r_ECI = R(ωt) · r_ECEF
```

混合計算得到的偏心率向量：

```
e_mixed = f(R(-ωt) · r_ECI, v_ECI) = R(-ωt) · f(r_ECI, v_ECI) = R(-ωt) · e_ECI
```

所以 `e_mixed` 以 -ω 速率相對 ECI 旋轉，在 ECEF 座標下看起來就是以 +ω 進動——完美解釋觀察到的 1.43°/s。

### 第一次修復嘗試：在算軌道元素前把 r_ECEF 轉回 ECI

加了三個 helper function：

1. `rotate_ecef_to_eci(v: DVec3, theta: f64)` — 繞 Y 軸旋轉 θ 把位置從 ECEF 轉 ECI
2. `rotate_eci_to_ecef_vec(v: Vec3, theta: f32)` — 反向旋轉，用於 gizmo 畫圖
3. `rotate_elements_to_ecef(elems, theta)` — 把軌道元素的方向向量（eccentricity_vec, angular_momentum）從 ECI 轉 ECEF

**修改 `orbit_prediction_system`：**

1. 取得 SOI body 的 `RotationAngle` 作為 θ
2. `rel_pos_eci = rotate_ecef_to_eci(rel_pos_ecef, θ)` — 統一到 ECI
3. 用 (rel_pos_eci, rel_vel_eci) 算軌道元素 — 現在兩個都在 ECI
4. 畫 gizmo 前用 `rotate_elements_to_ecef` 轉回 ECEF — 因為 gizmo 座標系是 ECEF（= local frame = Rapier 座標系）
5. Maneuver node 的 trail 和 post-maneuver orbit 也做同樣的 ECI↔ECEF 轉換

**編譯通過，但測試結果：pe_dir 仍然在漂移，而且比修復前更快！**

### 第二次修復的測試結果

| 時間 | pe_dir Z | 角度 | e | energy |
|---|---|---|---|---|
| 0s | 0.6126 | 37.8° | 0.296 | -3220 |
| 2s | 0.7712 | 50.5° | 0.300 | -3219 |
| 4s | 0.8920 | 63.4° | 0.305 | -3217 |
| 6s | 0.9693 | 75.8° | 0.313 | -3216 |

漂移率 ≈ 6.3°/s，比修復前的 1.43°/s 還快。而且偏心率 e 從 0.296 漲到 0.313，能量從 -3220 漲到 -3216。

### 分析：為什麼修好參照系反而更差

修復前，參照系混用造成的「假進動」恰好和 Rapier 積分誤差的「真進動」部分抵消（一個是精確 ω 的系統進動，一個是隨機漂移），所以看起來只有 1.43°/s。

修復後，參照系一致了，假進動消失了，但 **Rapier ECEF 物理的數值誤差裸露出來**。而且因為這些誤差是實際的能量注入（e 在漲、energy 在漲），軌道元素本質上就不穩定——不是參照系問題，是 **物理積分不準**。

### 根本根源：Rapier ECEF 物理不穩定

**為什麼 Coriolis 力讓 Rapier 積分器失穩：**

1. Rapier 用 Symplectic Euler（半隱式 Euler）。這個方法對「分離哈密頓系統」（H = T(p) + V(q)，力只依賴位置）是 symplectic 的——長期保能量，誤差有界。

2. Coriolis 力 `F = -2m(ω×v)` 依賴速度。這讓系統不再是分離哈密頓系統，symplectic 性質被破壞。積分器不再保能量，誤差會系統性地累積。

3. **量級問題：** 在 ksp-lite 的�</think><tool_call>Write<arg_key>content</arg_key><arg_value># Kerbin Rotation 教訓

## 問題

要在 Kerbin 上實現自轉效果，同時保持 Rapier 物理穩定（火箭在發射台上不噴飛）。

## 嘗試過的方案（全部失敗）

### 1. Prelaunch visual-only（原始 staged changes）
火箭 spawn 時無 RigidBody/Collider，按 Space 時動態插入。
**結果：** 插入瞬間 Rapier 偵測到穿透 → 爆炸式修正力 → 火箭噴飛。

### 2. KinematicPositionBased → Dynamic
火箭 spawn 為 Kinematic，每幀覆蓋 Transform，按 Space 時改成 Dynamic。
**結果：** Rapier 從位置差異推算出內部運動學速度（~50 m/s），切換 Dynamic 時這個速度被保留 → 火箭噴飛。

### 3. Always-Dynamic + SurfaceLocked 每幀覆蓋 Transform
火箭一出生就是 Dynamic + Velocity::zero，SurfaceLocked 期間每幀強制覆蓋 Transform 和 Velocity。
**結果：** Rapier 偵測到 Dynamic body 的 Transform 每幀跳變 → 從位置差異推算出速度 → 移除 SurfaceLocked 後火箭噴飛。

### 4. Dynamic + Kerbin child（Transform propagation）
火箭 spawn 為 Kerbin 的 child，靠 Bevy 的 Transform propagation 自動跟轉。
**結果：** 火箭的 Transform 變成 Kerbin 本地座標，所有讀 rocket Transform 的系統（camera、telemetry、readback）全部壞掉。Rapier 對 parent hierarchy 裡的 Dynamic body 行為也不確定。

## 根本原因

**Rapier 和旋轉的天體 Transform 衝突。** 不管怎麼包裝，只要 Kerbin 的 Transform 在轉、火箭是 Dynamic，就會有問題：

- Dynamic body 在旋轉的 static collider 旁邊 → 慣性力/摩擦力方向不對
- 每幀覆蓋 Dynamic body 的 Transform → Rapier 偵測到跳變 → 推算出巨大速度
- 切換 RigidBody 類型 → 內部狀態殘留

**Principia 的做法：** 物理永遠在慣性系裡跑（自己的 N-body 積分器），參照系只是顯示層的變換。它不用 Rapier。

**Seb Lague Solar System 的做法：** 星球根本不自轉。N-body + Rigidbody.MovePosition。

## 最終方案：ECEF 物理 + 虛擬力

**Rapier 永遠在 ECEF（天體固定系）裡跑。** Kerbin Transform 不轉，發射台不動，火箭 Dynamic + Velocity::zero 穩穩站著。

太陽繞 Kerbin 轉（舊版做法）→ 畫夜交替。

手動加兩個非慣性力讓 ECEF 物理等價於 ECI：

1. **離心力** `F = m·ω²·(x, 0, z)` — 推離自轉軸，抵消部分重力
2. **科里奧利力** `F = 2m·(-ω·vz, 0, ω·vx)` — 運動物體偏轉

這兩個力加在 `compute_gravity_and_drag` 閉包裡，SOI body 的 `rotation_speed > 0` 時啟用。

### SimState 仍存 ECI 速度

`sim_state_readback_system` 讀 Rapier 的 Velocity（ECEF），加回 `ω×r` 得到 ECI 速度寫入 SimState。軌道計算、遙測、map view 都用 SimState（ECI），不受影響。

### Drag 計算

大氣跟著星球轉，ECEF 裡大氣靜止。Drag 用 Rapier 的 `vel.linvel`（ECEF 速度）= 相對大氣的速度。✓

### 視覺自轉

火箭在軌道上時，因為正確的相對運動（科里奧利力 + 離心力讓 ECEF 裡的軌跡正確），飛船往下看會看到地表特徵正確地移動 — **不需要轉貼圖或 Transform。**

---

## Map View 軌道旋轉 Bug 調查

### 問題描述

在 MapView 裡，Kerbin 星球有在轉（MapMesh 被 RotationAngle 驅動旋轉），但軌道預測線（gizmo 畫的橢圓）也跟著在轉——相對背景星空以大約 1.43°/s 的速率進動。

### 初步診斷：加入診斷 log

在 `orbit_prediction_system` 的 `OrbitalElements` 計算後加了 `pe_dir`（periapsis direction）的 log，每 2 秒印一次。

**結果（修復前，混合 ECEF/ECI 參照系）：**

| 時間 | pe_dir Z | 對應角度 |
|---|---|---|
| 0s | 0.1057 | 6.1° |
| 2s | 0.1545 | 8.9° |
| 4s | 0.2035 | 11.6° |
| 6s | 0.2512 | 14.5° |

進動率 ≈ 1.40°/s。而 KERBIN_ROTATION_SPEED = 0.025 rad/s = 1.43°/s。

**進動率幾乎完美等於 Kerbin 的自轉角速度 ω。** 這不是巧合。

### 排除法分析

我們問了三個可能的根源：

1. **Rapier 積分本身的誤差？** — Symplectic Euler 對分離哈密頓系統（只有位置相關的力，如重力）是保結構的，誤差是 O(dt) 但不會產生精確的系統性進動。即使 Coriolis 力破壞 symplectic 結構，誤差也應該是隨機漂移，不可能精確等於 1.43°/s。

2. **潮汐力 / 引力梯度？** — 太陽 mu=0（零引力），Mun 在 12000m 外 mu=40000，潮汐加速度 ≈ 1e-4 m/s²，6 秒累積速度誤差 ~6e-4 m/s，對 108 m/s 軌道速度完全不會造成 8° 進動。

3. **參照系混用？** — **這才是真正原因。**

### 根本原因：參照系混用 (r_ECEF, v_ECI)

追蹤 `orbit_prediction_system` 的資料流：

```
rel_pos = rocket_sim.position - planet_sim.position    ← 位置差
rel_vel = rocket_sim.velocity - planet_sim.velocity    ← 速度差
OrbitalElements::from_state(rel_pos, rel_vel, mu)      ← 用這兩個算軌道元素
```

逐一檢查每個資料來源的參照系：

- **`rocket_sim.position`** 來自 `sim_state_readback_system`：`offset.ssb_from_local(tf.translation)`。`tf.translation` 是 Rapier 用 ECEF 速度更新位置出來的 → **位置是 ECEF**
- **`rocket_sim.velocity`** 來自 `sim_state_readback_system`：`vel.linvel.as_dvec3()` + `ω×r` 修正 → **速度是 ECI**
- **`planet_sim.position`** 來自 `celestial_orbit_system`：解析計算 → **ECI**（Kerbin 恆為零向量）
- **`planet_sim.velocity`** 來自 `celestial_orbit_system`：解析計算 → **ECI**（Kerbin 恆為零向量）

所以 `rel_pos = rocket_ECEF - planet_ECI`，`rel_vel = rocket_ECI - planet_ECI`。

**軌道元素是用 (r_ECEF, v_ECI) 算的——兩個不同參照系的資料混在一起。**

### 為什麼混用會造成精確等於 ω 的進動

正確的軌道元素需要 (r_ECI, v_ECI)。ECEF 和 ECI 之間的關係是繞 Y 軸旋轉 θ=ωt：

```
r_ECI = R(ωt) · r_ECEF
```

混合計算得到的偏心率向量：

```
e_mixed = f(R(-ωt) · r_ECI, v_ECI) = R(-ωt) · f(r_ECI, v_ECI) = R(-ωt) · e_ECI
```

所以 `e_mixed` 以 -ω 速率相對 ECI 旋轉，在 ECEF 座標下看起來就是以 +ω 進動——完美解釋觀察到的 1.43°/s 進動率。

### 第一次修復嘗試：在算軌道元素前把 r_ECEF 轉回 ECI

在 `orbit.rs` 加了三個 helper function：

1. **`rotate_ecef_to_eci(v: DVec3, theta: f64)`** — 繞 Y 軸旋轉 θ 把位置從 ECEF 轉 ECI。公式：`(x·cosθ + z·sinθ, y, -x·sinθ + z·cosθ)`
2. **`rotate_eci_to_ecef_vec(v: Vec3, theta: f32)`** — 反向旋轉，用於 gizmo 畫圖時把 ECI 座標轉回 ECEF
3. **`rotate_elements_to_ecef(elems, theta)`** — 把軌道元素的方向向量（eccentricity_vec, angular_momentum）從 ECI 轉 ECEF，這樣 `draw_orbit_gizmo` 畫出的橢圓就在正確的 ECEF 座標裡

**修改 `orbit_prediction_system` 的邏輯：**

1. 從 `planet_q` 取得 SOI body 的 `RotationAngle` 作為 θ（需要擴充 Query 加入 `Option<&RotationAngle>`）
2. `rel_pos_eci = rotate_ecef_to_eci(rel_pos_ecef, θ)` — 把火箭相對位置從 ECEF 轉到 ECI
3. 用 `(rel_pos_eci, rel_vel_eci)` 算軌道元素 — 現在位置和速度都在 ECI，參照系一致
4. 畫 gizmo 前用 `rotate_elements_to_elef` 把軌道元素方向從 ECI 轉回 ECEF — 因為 gizmo 座標系是 ECEF（= local frame = Rapier 座標系 = Transform 座標系）
5. Maneuver node 的 trail 和 post-maneuver orbit 也做同樣的 ECI→ECEF 轉換：`rotate_eci_to_ecef_vec(future_pos, θ)`

**編譯通過，但測試結果：pe_dir 仍然在漂移，而且比修復前更快！**

### 第二次修復的測試結果（參照系已一致）

| 時間 | pe_dir Z | 角度 | e | energy |
|---|---|---|---|---|
| 0s | 0.6126 | 37.8° | 0.296 | -3220 |
| 2s | 0.7712 | 50.5° | 0.300 | -3219 |
| 4s | 0.8920 | 63.4° | 0.305 | -3217 |
| 6s | 0.9693 | 75.8° | 0.313 | -3216 |

漂移率 ≈ 6.3°/s，比修復前的 1.43°/s 還快了 4 倍多。而且偏心率 e 從 0.296 漲到 0.313（理論應該是常數），能量從 -3220 漲到 -3216（理論應該是常數）。

### 為什麼修好參照系反而更差

修復前，參照系混用造成的「假進動」（精確 ω 的系統旋轉）和 Rapier 積分誤差的「真進動」（能量注入造成的偏心率漂移）部分抵消——因為假進動方向和真進動方向相反，所以視覺上看起來只有 1.43°/s 的淨漂移。

修復後，參照系一致了，假進動消失了，但 **Rapier ECEF 物理的數值誤差裸露出來**。現在看到的 6.3°/s 全部來自真實的物理積分誤差——偏心率在漲、能量在漲，軌道元素本質上就不穩定。

### 最終結論：問題的雙重根源

1. **參照系混用（已修復）：** `sim.position` 是 ECEF（來自 Rapier），`sim.velocity` 是 ECI（readback 加了 ω×r），兩者相減算軌道元素造成精確 ω 的假進動。已透過 `rotate_ecef_to_eci` 修正。

2. **Rapier ECEF 物理不穩定（尚未修復）：** 即使參照系正確，Rapier 的 Symplectic Euler 積分器無法處理速度相關的力（Coriolis），造成系統性能量注入，軌道元素漂移。

### 為什麼 Rapier ECEF 物理不穩定——詳細力學分析

**Symplectic Euler 的性質：**

Symplectic Euler（半隱式 Euler）的更新規則是：
```
v_{n+1} = v_n + a(x_n) · dt      ← 先用舊位置算加速度，更新速度
x_{n+1} = x_n + v_{n+1} · dt     ← 再用新速度更新位置
```

對分離哈密頓系統 H = T(p) + V(q)（力只依賴位置，如重力），這個方法是 symplectic 的：
- 長期保能量（能量誤差有界，不會系統性增長）
- 相空間體積守恆
- 誤差是 O(dt) 但是「振盪的」，不會累積

**Coriolis 力破壞 symplectic 性質：**

Coriolis 力 `F = -2m(ω×v)` 依賴速度。這讓系統不再是分離哈密頓系統。正確的更新應該是：
```
v_{n+1} = v_n + [a(x_n) - 2(ω×v_?)] · dt
```
但 `v_?` 該用 v_n 還是 v_{n+1}？用 v_n（顯式）→ 一階誤差；用 v_{n+1}（隱式）→ 需要解線性系統。Rapier 用顯式，所以 Coriolis 力的離散化有一階誤差，而且這個誤差是 **系統性的**（不是隨機的），會持續往同一個方向注入能量。

**ksp-lite 的嚴重比例失衡：**

在 ksp-lite 的縮小太陽系裡：

| 參數 | ksp-lite | 真實 KSP | 說明 |
|---|---|---|---|
| Kerbin 半徑 | 2000 m | 600,000 m | 1/300 |
| Kerbin 自轉週期 | ~251 s | 21,600 s (6hr) | 1/86 |
| ω | 0.025 rad/s | 0.000291 rad/s | 86x |
| LKO 軌道速度 | ~108 m/s | ~3,400 m/s | 1/31 |
| LKO 軌道週期 | ~242 s | ~1,800 s (30min) | 1/7.4 |
| Coriolis 加速度 | ~5.4 m/s² | ~2.0 m/s² | 2.7x |
| Kerbin 表面重力 | ~4 m/s² | 9.81 m/s² | 0.4x |
| **Coriolis/重力比** | **~135%** | **~20%** | **6.8x** |

關鍵數字：**Coriolis 加速度（~5.4 m/s²）比 Kerbin 表面重力（~4 m/s²）還大**。在真實 KSP 裡這個比例只有 20%。Coriolis 力不是「小修正」，它是主導力——積分器在這個條件下根本無法穩定。

另外，自轉週期（251s）和軌道週期（242s）幾乎相同，意味著表面速度佔軌道速度的比例接近 46%。在真實 KSP 裡，自轉週期是軌道週期的 12 倍，表面速度只佔軌道速度的 7.6%。這也加劇了問題。

**對比：如果只有離心力沒有 Coriolis：**

離心力只依賴位置，不依賴速度。Symplectic Euler 對離心力處理得很好（和重力一樣是位置相關力）。所以如果只有離心力，積分器應該是穩定的。問題完全出在 Coriolis——它依賴速度，破壞了積分器的 symplectic 結構。

### 正確的長期解法：On-Rails Kepler

KSP 原版就是這麼做的：
- **大氣層外 + 無推力 + 無碰撞** → 用解析軌道（Kepler 方程）計算位置/速度，每幀直接寫入 Transform + SimState
- **大氣內 / 有推力 / 有碰撞** → 交給 Rapier 物理引擎

這樣在軌道上滑行時：
1. 位置和速度是解析精確的，零數值誤差
2. 不需要 Coriolis/離心力（因為用 ECI 解析解）
3. 軌道預測和實際軌跡完全一致
4. 進大氣或著陸時切回 Rapier ECEF 物理

**On-rails 的切換邏輯：**
- 條件：altitude > atmosphere_height AND throttle == 0 AND no collision → on-rails
- 進 on-rails：從 SimState 的 (r_ECI, v_ECI) 算出 OrbitalElements，設 RigidBody::KinematicPositionBased
- 離 on-rails（進大氣/按油門/碰撞）：從 OrbitalElements 推算當前 (r_ECI, v_ECI)，轉 ECEF 寫入 Transform + Velocity，設回 RigidBody::Dynamic

### 目前程式碼狀態

目前保留了參照系修復（ECEF→ECI 旋轉）在程式碼裡。雖然它沒有解決軌道漂移問題（那是 Rapier 積分的問題），但它讓軌道計算至少在參照系上是正確的。等 on-rails 實作後，這些旋轉函數仍然有用——on-rails 的軌道元素在 ECI，但 Transform 在 ECEF，需要互相轉換。

### 附錄：偽力公式推導

在 ECEF 座標系中，牛頓第二定律寫成：

```
m·a_ECEF = F_gravity + F_other - m·[ω×(ω×r)] - 2m·(ω×v_ECEF) - m·(dω/dt × r)
```

其中：
- `-m·[ω×(ω×r)]` = 離心力。對於 ω = (0, ω, 0)（繞 Y 軸旋轉）和 r = (x, y, z)：
  - ω×r = (ωz, 0, -ωx)
  - ω×(ω×r) = (0, ω, 0) × (ωz, 0, -ωx) = (-ω²x, 0, -ω²z)
  - 離心力 = -m·(-ω²x, 0, -ω²z) = m·ω²·(x, 0, z) ✓

- `-2m·(ω×v)` = 科里奧利力。對於 v = (vx, vy, vz)：
  - ω×v = (ωvz, 0, -ωvx)
  - 科里奧利力 = -2m·(ωvz, 0, -ωvx) = 2m·(-ωvz, 0, ωvx) ✓

- `-m·(dω/dt × r)` = Euler 功。因為 ω 是常數（勻速自轉），這項為零。✓

### 附錄：ECEF↔ECI 速度轉換

```
v_ECI = v_ECEF + ω × r
```

對於 ω = (0, ω, 0) 和 r = (x, y, z)：
- ω × r = (ωz, 0, -ωx)

所以：
```
v_ECI = v_ECEF + (ωz, 0, -ωx)
```

反過來：
```
v_ECEF = v_ECI - (ωz, 0, -ωx) = v_ECI - ω × r
```

在 `sim_state_readback_system` 中使用正轉（ECEF→ECI），在 `debug_orbit_apply_system` 中使用反轉（ECI→ECEF）。

### 附錄：ECEF↔ECI 位置轉換

位置轉換是繞 Y 軸旋轉 θ=ωt：

```
r_ECI = R(θ) · r_ECEF
```

其中 R(θ) 是繞 Y 軸的旋轉矩陣：

```
[ cosθ  0  sinθ ]   [x]   [x·cosθ + z·sinθ]
[  0    1   0   ] · [y] = [       y        ]
[-sinθ  0  cosθ ]   [z]   [-x·sinθ + z·cosθ]
```

反轉：
```
r_ECEF = R(-θ) · r_ECI
```

在 `rotate_ecef_to_eci` 和 `rotate_eci_to_ecef_vec` 中實作。

---

## On-Rails Kepler 實作紀錄

### 已實作的系統

1. **`KeplerOrbit`**（`orbit_math.rs`）— 解析開普勒軌道，存 (a, e, mu, p_dir, q_dir, M_epoch, epoch)，可計算任意時間的 (r_ECI, v_ECI)。用 Newton 法解 Kepler 方程 `M = E - e·sin(E)`。

2. **`FlightMode` component**（`main.rs`）— 每個飛行實體有 `Ground` 或 `Orbit(KeplerOrbit)` 標記。

3. **`kepler_propagate_system`**（`flight.rs`, Update）— 核心系統：
   - **Ground→Orbit 轉換：** altitude > 0 且 `pe_alt >= 0`（穩定軌道）。記錄每個 part 的 offset，把火箭和 parts 切成 `KinematicPositionBased`
   - **Orbit→Ground 轉換：** `pe_alt < 0` 或 altitude < 2。轉回 Dynamic，設 Rapier 速度（ECI→ECEF 轉換）
   - **推力 Δv：** 把推力方向從 ECEF 轉 ECI，加到速度上，重算軌道
   - **大氣 drag Δv：** 從 `orbit.state_at(t)` 取 ECI 速度算阻力，減速度，重算軌道
   - **SOI 切換偵測：** mu 變了就重算軌道

4. **`kepler_writeback_system`**（`flight.rs`, PostUpdate before SyncBackend）— 把傳播結果寫入 Transform + SimState + Velocity。火箭和 parts 在同一個系統裡原子更新。

5. **Skybox rotation**（`main.rs`）— Flight view 裡旋轉 skybox（`Skybox.rotation = Quat::from_rotation_y(θ)`），Map view 裡固定（IDENTITY）。

### 踩過的坑

#### 1. Ground↔Orbit 切換震盪

最初心想：altitude > 0 就切 Orbit。結果垂直發射的火箭在低空就切了，但它的軌道是 sub-orbital（pe_alt < 0），下一幀立刻切回 Ground，再下一幀又切 Orbit……無限震盪。

**修復：** 加 `pe_alt >= 0` 條件。只有軌道真正穩定（不會撞地面）才切到 Orbit。

#### 2. 火箭部件分離 + 碰撞拉扯

兩個 stage 的 nozzle collider 互相穿透 → Rapier 碰撞力把它們推開 → 看起來兩截火箭被拉扯。

**修復：** 加 `CollisionGroups` — parts 設 GROUP_1 membership / GROUP_2 filter，只跟環境碰撞，不互相碰撞。

#### 3. 部件定位：delta vs offset

最初用 delta（每幀位置差）移動 parts。但浮點誤差會累積，幾十秒後 parts 漂移。

改用 **offset-based**：Ground→Orbit 轉換時記錄每個 part 相對火箭的 offset，之後每幀 `part_tf.translation = rocket_tf.translation + offset`。零累積誤差。

#### 4. Update vs PostUpdate 跨幀不同步（最關鍵的 bug）

`kepler_parts_system` 在 Update 跑（移動 parts），`kepler_writeback_system` 在 PostUpdate 跑（移動火箭）。這造成：
- 第一幀：parts 已經在新位置，火箭還在舊位置 → parts 從火箭「彈出」
- 下一幀：火箭追上 → 看起來 parts 閃爍、抖動
- Debug wireframe（Rapier 內部狀態）正常，但 mesh（Transform）抖動

**修復：** 合併兩個系統為一個 `kepler_writeback_system`，火箭和 parts 在同一個系統、同一幀原子更新。

#### 5. kepler_writeback_system 排程位置

最初放在 PostUpdate / after Writeback。問題：
- Rapier 的 SyncBackend 在 Writeback 階段讀 Transform（for KinematicPositionBased bodies），然後插值
- 我們在 Writeback 之後覆蓋 Transform → Rapier 的插值和實際位置不同步
- 結果：collider debug 線條（Rapier 插值）平滑，mesh（Transform）跳動

**修復：** 把 `kepler_writeback_system` 移到 **before SyncBackend**（after floating_origin_system）。這樣 Rapier 讀到我們設的正確 Transform，插值和 mesh 一致。

同時，`new_local` 不能用 `kepler_propagate_system` 在 Update 算好的值（可能用了舊的 offset），要從 `abs_pos_ecef` 和 **當前** `offset.0` 即時重算。

#### 6. OrbitAngle vs RotationAngle 混淆

`kepler_propagate_system` 最初用 `OrbitAngle` 計算 `soi_theta_now = orbit_speed * t`。但 `OrbitAngle` 只有 Mun/Minmus 有，Kerbin 用 `RotationAngle`。而且即使用 OrbitAngle，`orbit_speed * t` 和實際累積的 `OrbitAngle.0` 也有浮點差異。

**修復：** 統一用 `RotationAngle`（Kerbin 有，Mun/Minmus 沒有但 rotation_speed=0 所以 theta=0）。所有 ECI↔ECEF 轉換都用同一個來源的 theta。

#### 7. Skybox 在 ECEF flight view 裡不動

Skybox 固定在 `Quat::IDENTITY`。在 ECEF 座標系裡 Kerbin 不動，但星空相對慣性系是固定的，所以在 ECEF 裡看應該在轉。火箭在地面時感受不到差異，但在軌道上會覺得天球不動很奇怪。

**修復：** 加 `skybox_rotation_system`，Flight view 裡根據 Kerbin 的 `RotationAngle` 旋轉 skybox，Map view 裡固定。

### 系統排程總覽

```
Update:
  celestial_orbit_system      → 更新 Mun/Minmus 位置（ECEF SSB）+ Transform
  celestial_rotation_system   → 更新 RotationAngle
  skybox_rotation_system      → 旋轉 skybox（flight view only）
  rocket_input_system         → 油門、推力、SAS、燃料、Ground-mode 外力
  kepler_propagate_system     → On-rails 傳播、模式轉換、寫 KeplerWriteback
  telemetry_system            → 遙測顯示

PostUpdate:
  floating_origin_system      → 浮點位移修正（before SyncBackend）
  kepler_writeback_system     → 寫火箭+parts Transform/SimState/Velocity
                                （after floating_origin, before SyncBackend）
  sim_state_readback_system   → Ground-mode: Rapier → SimState（after Writeback）
  sim_invariant_check_system  → 驗證 SimState ↔ Transform 一致性
  camera_controller           → 相機跟隨
  sun_light_system            → 太陽光位置

MapView only (Update):
  orbit_prediction_system     → 軌道預測 gizmo（ECI frame）
  maneuver_node_system        → 操作節點
  map_focus_click_system      → 地圖焦點
```

### 參照系使用原則

| 系統 | 位置 | 速度 | 說明 |
|---|---|---|---|
| Rapier physics | ECEF | ECEF | 地面穩定，大氣 drag 正確 |
| SimState | ECEF | ECI | 混合設計：位置跟 Rapier 走（ECEF），速度跟軌道力學走（ECI） |
| KeplerOrbit | ECI | ECI | 解析軌道在慣性系裡最簡單 |
| 軌道元素計算 | ECI | ECI | 必須統一，否則產生精確 ω 的假進動 |
| Map view gizmo | ECI | — | MapMesh 旋轉代表 ECI，gizmo 在 ECI 畫 |
| Flight view | ECEF | — | 飛行視角，天體不動，skybox 旋轉 |
| Skybox | ECEF frame 旋轉 | — | 模擬天球在 ECEF 裡的旋轉 |

### 核心教訓

1. **參照系混用會產生精確的系統性誤差，不是隨機漂移。** (r_ECEF, v_ECI) 混合算軌道元素 → 進動率精確等於 ω。這類 bug 特別危險因為看起來像「物理效果」但完全是程式錯誤。

2. **Rapier 的 KinematicPositionBased body 必須在 SyncBackend 之前設好 Transform。** 之後設的話，Rapier 的插值和實際位置不同步，collider 線條和 mesh 分裂。

3. **不同排程的系統更新同一組實體時，會產生一幀不同步。** Update 裡移 parts + PostUpdate 裡移 rocket = parts 先跑一步。解法：合併為同一個系統。

4. **浮點 offset 的累積。** delta-based 移動會累積浮點誤差；offset-based（記錄初始相對位置，每幀重建）不會。On-rails 部件必須用 offset-based。

5. **浮動原點改變 offset 後，不能用舊 offset 算的 local 座標。** `kepler_propagate_system` 在 Update 算 `new_local`，但如果 PostUpdate 的 `floating_origin_system` 位移了 offset，那個 `new_local` 就過時了。必須從絕對座標和**當前** offset 重算。

6. **ECI vs ECEF 的設計取捨：ECEF 物理 + ECI 軌道。** 物理在 ECEF 裡跑（火箭站著不動），軌道在 ECI 裡算（解析解最簡單）。SimState 存 ECEF 位置 + ECI 速度，需要時互相轉換。這個混合設計是穩定的，但每次跨系統讀資料時必須確認參照系。

7. **CollisionGroups 是多部件火箭的必備工具。** 不設的話 nozzle collider 穿透相鄰 stage → 碰撞力拉扯 → 視覺上火箭裂開。

---

## 未來

- **地形 collider：** 目前 Kerbin 是球體，旋不旋轉都一樣。未來加地形後，著陸位置不會跟著自轉偏移（collider 不動）。On-rails 切換到 Rapier 時需要正確處理 ECI→ECEF 座標轉換。
- **UV offset shader：** 如果想要從遠處看到 Kerbin 貼圖在轉，可以用 shader 層面偏移 UV，不影響物理。目前 FlightMesh（ECEF 貼圖）+ MapMesh（ECI 旋轉貼圖）已實作此效果。
- **時間加速（Time warp）+ on-rails：** 目前 time warp 改 `Time<Virtual>` 的速度，kepler 傳播自動跟著快。需要確認高倍速（10x, 50x, 100x）下 Kepler 方程求解仍然收斂。
- **多天體引力（N-body）：** 目前 on-rails 只考慮 SOI body 的引力。跨越 SOI 邊界時有切換邏輯，但沒有雙球體引力。Principia 的 N-body 在 ksp-lite 的縮放比例下可能不需要。
