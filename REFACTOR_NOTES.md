# Sim Layer Refactor

## 動機

原本座標/速度/軌道計算散布在各系統中，每個消費者自己從 `Transform` + `Velocity` + `CelestialBody` 拼凑位置和速度。問題：

1. **浮動原點 bug** — `soi_body_velocity()` 從偏移後的 `Transform.translation` 推導天體速度，浮動原點平移後結果錯誤
2. **重複計算** — 「找 SOI → 算相對位置 → 算軌道速度」這段邏輯複製貼了 4 次
3. **沒有真理層** — 所有座標都是 f32 浮動原點偏移後的值，沒有人能可靠地問「這個東西現在在哪、多快」

## 設計決策

### 單位制
SI（m, m/s, kg, N, s），無例外。不需要型別編碼單位。

### 兩個參照系，只有兩個

| 幀 | 原點 | 精度 | 用途 |
|---|---|---|---|
| **SSB** (Solar System Barycenter) | 太陽系質心 | f64 | 真理層，所有計算 |
| **Local** | SSB 平移到接近飛船 | f32 | 渲染、Rapier 物理 |

兩幀之間**只有平移，沒有旋轉**。速度在兩幀中完全相同。
不需要 ECEF 等旋轉幀 — 地表相關計算（經緯度、地表速度）是查詢函數，不是參照系。

### 數據流

```
天體:  OrbitAngle → SimState(f64 SSB) → Transform(f32 local)
火箭:  Rapier physics → Transform/Velocity(f32 local) → SimState(f64 SSB)
查詢:  所有消費者讀 SimState，需要渲染時 to_local()
```

## 架構

### `src/sim.rs` — 核心型別

- `SsbPosition(DVec3)` — SSB 位置
- `SsbVelocity(DVec3)` — SSB 速度
- `SimState` — Bevy Component，{ position, velocity }
- `LocalOffset(DVec3)` — Resource，SSB→local 的累積平移量
- `SsbPosition::to_local()` / `LocalOffset::ssb_from_local()` — 雙向轉換

### 同步系統

- `celestial_orbit_system` — SSB 為真理，從軌道角算 f64 位置/速度寫入 SimState，推導 Transform
- `sim_state_readback_system` — Rapier writeback 後，把 local f32 轉回 SSB f64 寫入 SimState
- `floating_origin_system` — 改用 `LocalOffset(DVec3)`，平移所有 Transform 時累積 offset

### 軌道接口 (orbit.rs)

- `find_soi_body(SsbPosition, ...)` — 用 f64 SSB 位置判定 SOI
- `surface_rotation_velocity(body, DVec3) → DVec3` — 天體自轉速度
- ~~`soi_body_velocity()`~~ — **已刪除**。原本從偏移後的 Transform 推導天體速度，浮動原點後會算錯。現在直接讀 `SimState.velocity.0`

### 不變量檢查 (sim_invariant_check_system)

每 2 秒自動驗證：

1. SimState → Transform 一致性（誤差 > 0.1m 警告）
2. 天體軌道速度 = angular_speed × orbit_radius（相對誤差 > 1% 警告）
3. Kerbin 固定在 SSB 原點（位置/速度 ≠ 0 警告）
4. 能量守恆：滑行+無大氣時 E = v²/2 - μ/r 應守恆（相對誤差 > 5% 警告）

### Debug 軌道啟動

`cargo run -- --orbit` 直接進入 Kerbin 橢圓軌道飛行：
- `--orbit` — 中等橢圓 200m × 2000m
- `--orbit=2` — 高偏心率 100m × 4000m
- `--orbit=3` — 近圓 500m × 800m

## 已完成的遷移

| 系統 | 改了什麼 |
|---|---|
| `find_soi_body` | 用 SimState f64 位置比較 |
| `soi_body_velocity` | 刪除，直接讀 SimState.velocity |
| `surface_rotation_velocity` | 改用 DVec3 |
| `rocket_flight_system` | SAS/重力從 SimState 讀軌道狀態 |
| `telemetry_system` | 全部從 SimState 讀 |
| `navball_system` | 從 SimState 算軌道速度 |
| `orbit_prediction_system` | f64 計算，gizmo 時轉 local f32 |
| `celestial_orbit_system` | SSB 為真理，Transform 從 SSB 推導 |
| `time_warp_system` | 從 SimState 算高度 |
| `debug_orbit_apply_system` | 直接設 SSB 位置/速度 |
| `camera_controller` | 用 SimState 做 SOI 判定 |

## 測試結果

三個橢圓軌道 preset 各跑 30 秒，0 invariant 違規，能量守恆通過。

## 後續可能

- 軌道計算統一接口（Ap/Pe/period/inclination 目前 telemetry 和 orbit gizmo 各算一次）
- Kerbin 物理旋轉（改為 KinematicPositionBased）
- 更多 invariant：圓軌道閉合性、角動量守恆
