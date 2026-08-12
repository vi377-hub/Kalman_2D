# Kalman 2D Angle - Bias

Code dùng ICM-20602 để ước lượng góc roll và pitch. Mỗi trục có vector trạng thái:

x = [Angle, Gyro Bias]

Gyro dự đoán góc, accelerometer sửa sai lệch. Kalman gain gồm hai giá trị để cập nhật đồng thời góc và gyro bias.

## Nâng cấp so với Kalman 1D

Kalman 1D chỉ ước lượng góc và dùng gyro bias cố định sau khi hiệu chuẩn. Bản 2D dùng ma trận covariance 2x2 để mô tả liên hệ giữa góc và bias, nhờ đó bias tiếp tục được ước lượng khi chương trình đang chạy và giảm trôi do nhiệt độ hoặc cảm biến thay đổi.

Khi tổng gia tốc lệch xa 1 g, code tăng sai số đo để giảm ảnh hưởng của gia tốc chuyển động lên góc accelerometer.
