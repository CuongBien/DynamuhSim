# Cấu trúc NarrowNav

## Có trong checkout

| Vị trí | Nội dung |
| --- | --- |
| `configs/` | Cấu hình Hydra cho mô phỏng, world, bề rộng hành lang, scenario, profile người, robot và smoke test. Nhóm profile người là `configs/humans/`. |
| `sim/mujoco/` | World, MJCF actor, template và asset. |
| `src/narrow_nav/core/` | Kiểu dữ liệu và cổng v1, gồm `HumanControllerBackend`. |
| `src/narrow_nav/simulation/` | Runtime episode, bộ dựng world, adapter MuJoCo và các điểm vào để kiểm tra cấu hình, xem world và chạy smoke. |
| `src/narrow_nav/visualization/`, `humans/` | Mã skin và animation dùng chung cùng demo Quaternius. |
| `experiments/social_navigation/vendor/` | Gitlink Python-RVO2 và SocNavGym để tham khảo khi tích hợp backend. |
| `tests/` | Kiểm thử đơn vị, tích hợp và mô phỏng. |

## Đang lên kế hoạch

Backend điều hướng người đặt ở `src/narrow_nav/humans/` theo hợp đồng của `core`. Mốc đầu là thực thi `scripted` qua runtime MuJoCo. Module dataset, dự báo, điều hướng và tích hợp ROS 2 thuộc các mốc sau. Khi ghi kết quả thực nghiệm, đặt cấu hình đã resolve, manifest và output dưới `runs/<run_id>/`. [Lộ trình](docs/ROADMAP.md) · [Ranh giới module](docs/architecture/01_MODULE_BOUNDARIES.md)
