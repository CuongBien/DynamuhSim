# Camera height A/B — 12cm và40cm

**Đã đo3cặp cảnh tĩnh trên Gazebo GPU: nâng camera tăng độ phủ trong các cảnh này, nhưng khối trụ người vẫn che phần lớn depth khi robot đến sát nhóm. Chưa chọn40cm làm cấu hình robot mặc định.** [Báo cáo](../../../archive/social_navigation/reports/camera_height_ab/CAMERA_HEIGHT_REPORT.md), [audit](../../../archive/social_navigation/reports/camera_height_ab/run04/audit.json), [số đo](../../../archive/social_navigation/reports/camera_height_ab/run04/summary.json).

Đây là thử nghiệm riêng về vị trí camera, không tăng chiều cao thân robot, không chạy Nav2 hoặc native human step. Robot/human XYyaw lấy từ3snapshot của dataset_pilot/pilot01. Hai depth sensors chụp đồng thời cùng timestamp ở cùng x/y/yaw; offset base_link.z=.08/.36m, base_linkworld.z=.04m, tương ứng camera12/40cm cách sàn. FOV87°,424×240,float32,10Hz,clip.28–5m giữ nguyên. Không thêm giá đỡ/mast vật lý; đây là phép thử vị trí cảm biến ảo.

## Cách chạy

Tại root repository trên WSL Ubuntu24.04/Jazzy, dùng dependency đã có:

```bash
cmake -S experiments/social_navigation/perception/camera_height_ab \
  -B experiments/social_navigation/perception/camera_height_ab/generated/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build experiments/social_navigation/perception/camera_height_ab/generated/build -j 1
bash experiments/social_navigation/perception/camera_height_ab/run.sh test
bash experiments/social_navigation/perception/camera_height_ab/run.sh live \
  --output experiments/social_navigation/perception/camera_height_ab/generated/runs/run05
bash experiments/social_navigation/perception/camera_height_ab/run.sh analyze \
  --capture experiments/social_navigation/perception/camera_height_ab/generated/runs/run05 \
  --output experiments/social_navigation/perception/camera_height_ab/reports/run05
```

Output phải là thư mục mới, không chạy song song. Cần raw episode pilot01 local ở đường dẫn trong protocol.json; pose3snapshot có bản nhỏ trong inputs/snapshots.json để kiểm tra. Actor-freeze plugin là code riêng dùng cùng API Actor.SetTrajectoryPose/SetAnimationTime của adapter hiện có, không sửa adapter/vendor. C++ build giới hạn1job, BLAS1thread, rendering kiểm NVIDIA, ROS_DOMAIN_ID91 lock và partition riêng. Không mở GUI trong phép đo.

Raw60depthframes và world/log/runtime ở generated/runs/run04 và generated/run04; bị Gitignore. Báo cáo lưu representative_depths.npz nén lossless6ảnh thật (mỗi camera/scene một ảnh), BEV, source/config hashes, camera URDF copies và kết quả renderer/cleanup. Hình so sánh dựng từ ảnh depth thật, không imagegen; BEV là phép tính, không ảnh chụp từ trên xuống. Không phải dataset train.

## Giới hạn

- Camera40cm chỉ là một điểm thử; vị trí cuối phải theo robot thật và cần test Nav2 riêng sau khi quyết định. Không kết luận chiều cao tối ưu từ3snapshot.
- Khối proxy hình trụ1.7m/radius.36m phục vụ các phép thử geometry/LiDAR cũ, làm depth có hình người dạng cột. Analytic cylinder agreement đo sự khớp mặt trụ, không phải ground truth segmentation hoàn chỉnh; chưa làm ablation tắt proxy để quy mọi sai khác cho nó.
- Sensor pose / camera_joint của URDF runtime / static TF camera được đối chiếu. World->base_link của fixture khai báo rõz=.04; không dùng URDF base_footprint offset để dựng worldpose. Mâu thuẫn baseheight SDF/URDF cũ chưa được sửa trong robot nguồn; không công bố hiệu chuẩn3D toàn robot đã hoàn tất.
- BEV giữ nguyên thuật toán pilot: một frame, không tích lũy thời gian; unknown không phải occupied/free. Known-area nhiều hơn không tự chứng minh occupancy chính xác hơn.
- Nhóm đi/né/nhích tránh và robot chen giữa nhóm không được thay đổi hoặc nghiệm thu trong phép đo camera này.
- run01 là lỗi topic camera_info, run02 còn skeleton tự chạy ở1scene, run03 script-only freeze làm renderer không trả depth và cần dừng cưỡng bức server thuộc phiên. Các dấu vết được giữ, không gộp vào cohort đạt. run04 dùng manual animation time=0,60/60ảnh ổn định bitwise,3server exit0/cleanup rỗng. Chi tiết reports/setup_history và reports/run02/NOT_ACCEPTED.json.

Bước kế đề xuất: kiểm riêng hình học mà Depth/GPU LiDAR nhìn thấy, làm ablation có/không proxy trong môi trường thử để tách ảnh hưởng; giữ baseline cũ và kiểm lại collision/LiDAR trước mọi thay đổi tích hợp. Sau đó mới chọn mount camera và quay lại QA dữ liệu/persona. Không chờ World Model tự khắc phục sai lệch cảm biến hoặc robot chen nhóm.
