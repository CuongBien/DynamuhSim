# Camera RGB-D

Camera mô phỏng đã được nâng cấp sang profile Intel RealSense D435i.

Xem hướng dẫn build, topic depth, UI khoảng cách và điều khiển robot tại
[`D435I.md`](D435I.md).

Topic chính:

- RGB: `/camera/color/image_raw`
- Depth: `/camera/depth/image_rect_raw`
- Point cloud: `/camera/depth/color/points`
- Khoảng cách từ camera: `/camera/obstacle/range`
- Khoảng cách từ robot: `/camera/obstacle/distance`
