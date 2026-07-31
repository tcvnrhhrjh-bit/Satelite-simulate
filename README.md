# RIoT-2 衛星軌道、遙測、ADCS 與數位分身分析

本專案使用 MATLAB 整合 RIoT-2 衛星的 TLE 軌道資料與 housekeeping telemetry，建立一套小型衛星任務分析與 digital twin workflow。分析內容包含軌道傳播、地面站可視性、熱遙測、通訊品質、ADCS quaternion 姿態重建、姿態指向誤差、熱模型驗證，以及簡化電力預算與電池 SOC 預測。

## 專案亮點

- 使用 TLE 建立 RIoT-2 軌道模型
- 計算中壢地面站 access window、仰角與距離
- 由 ADCS quaternion 重建衛星姿態
- 估算對地與對太陽 pointing error
- 依姿態誤差推測 ADCS 行為
- 分析熱遙測與日照/地影週期的關係
- 建立一階 thermal digital twin 並計算 RMSE
- 建立簡化 EPS power budget 與 battery SOC prediction
- 自動輸出圖表、CSV 摘要與 Markdown 報告

## 資料摘要

| 指標 | 結果 |
|---|---:|
| 遙測資料筆數 | 130 |
| CSV 檔案數 | 5 |
| 資料時間範圍 | 01-Jul-2026 15:28:03 到 04-Jul-2026 14:28:05 UTC |
| 軌道模擬取樣點 | 4262 |
| 中壢地面站可見取樣點 | 133 |
| 中壢地面站可見比例 | 3.12% |
| 有效 quaternion 覆蓋率 | 100.0% |
| 對地指向誤差中位數 | 43.65 deg |
| 對太陽指向誤差中位數 | 87.12 deg |
| 熱模型 RMSE | 10.22 deg C |

## 專案檔案

| 檔案/資料夾 | 說明 |
|---|---|
| `SATELITE.m` | MATLAB 主程式 |
| `figure_outputs/` | 自動輸出的分析圖 |
| `RIoT2_portfolio_summary_metrics.csv` | 作品集摘要指標 |
| `RIoT2_attitude_pointing_and_mode.csv` | 每筆資料的姿態誤差與推測模式 |
| `RIoT2_inferred_adcs_mode_distribution.csv` | ADCS 推測模式統計 |
| `RIoT2_digital_twin_portfolio_explanation_中文.md` | 中文完整說明報告 |

## 主要輸出圖

### 1. 熱遙測與軌道地影週期

![Thermal telemetry versus orbit eclipse](figure_outputs/01_thermal_cycle_eclipse.png)

比較太陽能板、MCU、board 溫度與 TLE 推算出的日照/地影狀態。結果顯示太陽能板溫度在 eclipse 附近明顯下降，代表熱遙測與軌道照明條件具有合理關聯。

### 2. 通訊品質與中壢地面站幾何關係

![Communication quality and Zhongli geometry](figure_outputs/02_communication_time_series.png)

比較 RSSI、地面站仰角與衛星距離。大部分時間衛星仰角為負，代表不在中壢地面站可見範圍內。

### 3. RSSI 與地面站仰角

![RSSI versus ground-station elevation](figure_outputs/03_rssi_versus_elevation.png)

檢查 RSSI 是否隨仰角增加而改善。此資料集中 RSSI 與 elevation/range 幾乎沒有線性關係，可能受到天線姿態、通訊狀態與封包時間影響。

### 4. ADCS 姿態指向重建

![ADCS pointing reconstruction](figure_outputs/04_attitude_pointing_assessment.png)

使用 quaternion telemetry 計算對地與對太陽 pointing error，並推測可能的 ADCS 行為。這是本專案最具研究延伸性的部分。

### 5. 遙測與軌道特徵相關係數矩陣

![Telemetry and orbit-feature correlation](figure_outputs/05_correlation_matrix.png)

以 Pearson correlation 檢查熱、通訊、電力、軌道與姿態變數之間的關係。

### 6. 熱模型驗證

![Thermal model validation](figure_outputs/06_thermal_model_validation.png)

比較實測溫度與一階熱模型預測結果，RMSE 為 `10.22 deg C`。模型可作為 preliminary thermal digital twin baseline。

### 7. 熱模型殘差

![Thermal model residual](figure_outputs/07_thermal_model_residual.png)

顯示 `實測溫度 - 預測溫度`，用於判斷模型在哪些時間點高估或低估溫度。

### 8. 電力預算與電池 SOC 預測

![Power budget and battery state prediction](figure_outputs/08_power_budget_battery.png)

估算太陽能發電、OBC 負載、淨功率、電池 SOC 與實測電池電壓。結果顯示分析期間沒有明顯電量耗盡風險。

## 執行方式

1. 開啟 MATLAB。
2. 將工作目錄切換到本專案資料夾。
3. 確認 TLE 與 telemetry archive 路徑可讀取。
4. 執行：

```matlab
SATELITE
```

程式會自動輸出圖表、CSV 與 Markdown 報告。

## 限制與假設

- ADCS 模式推測依賴 body-axis 假設，尚未使用實際衛星 CAD 校正。
- 熱模型是一階簡化模型，並非完整熱控模型。
- SOC 模型使用假設電池容量與初始 SOC。
- OBC 負載功率由 mode 估算，並非直接量測。
- RSSI 可能受到天線姿態、發射狀態與封包時間影響。
- 地影模型使用簡化 cylindrical eclipse approximation。

## 未來延伸方向

- 使用衛星 CAD 或 ADCS 文件校正 body axis
- 加入 gyro telemetry，分析 detumbling 與 slew rate
- 將 commanded ADCS mode 與推測模式比較
- 建立完整 link budget 與 Doppler shift 分析
- 使用實際電流 telemetry 改善 power budget
- 使用資料擬合 thermal model parameters
- 將 MATLAB workflow 包裝成 App Designer GUI

## 作品集定位

本專案可定位為：

> 基於 TLE 與衛星遙測資料之 RIoT-2 CubeSat 軌道、姿態、熱控、通訊與電力數位分身分析系統

此作品展示衛星任務分析、ADCS 姿態重建、資料處理、工程建模與自動化報告輸出的能力，適合作為航空太空、通訊、控制或系統工程相關碩士申請作品集。

