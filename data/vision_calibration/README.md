# Vision Calibration Data

K230 视觉标定 CSV 和对应图片样本可在本地集中放在这里，但不随 Git 上传。

建议本地目录结构：

- `CSV and PHOTOS/far`：太远、未达到抓取范围的负样本。
- `CSV and PHOTOS/TRUE`：达到抓取范围的正样本。
- `CSV and PHOTOS/test`：临时实测样本。

这些数据用于收敛 `tools/vision/ball_center_calibration_k230_mqtt.py` 里的抓取窗口参数。

注意：`CSV and PHOTOS/` 体积较大，已在 `.gitignore` 中忽略。
