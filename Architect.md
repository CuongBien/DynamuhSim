
NarrowNav-Research/
│
├── README.md
├── AGENTS.md                  # Quy tắc làm việc cho Codex/AI
├── CONTRIBUTING.md
├── pyproject.toml             # Python package
├── uv.lock                    # Khóa dependency Python
├── .gitignore
├── dvc.yaml                   # Pipeline dữ liệu
│
├── docs/                      # [01] TÀI LIỆU NGHIÊN CỨU
│   ├── architecture/
│   ├── research/
│   ├── protocols/
│   ├── schemas/
│   ├── decisions/
│   └── migration/
│
├── configs/                   # [02] CẤU HÌNH TẬP TRUNG
│   ├── simulation/
│   ├── worlds/
│   ├── scenarios/
│   ├── human_controllers/
│   ├── robots/
│   ├── sensors/
│   ├── dataset/
│   ├── world_model/
│   ├── navigation/
│   ├── evaluation/
│   └── experiments/
│
├── sim/                       # [03] TÀI NGUYÊN MUJOCO
│   └── mujoco/
│       ├── models/
│       │   ├── robots/
│       │   ├── humans/
│       │   └── objects/
│       ├── sensors/
│       ├── worlds/
│       ├── assets/
│       └── templates/
│
├── src/                       # [04] MÃ NGUỒN NGHIÊN CỨU
│   └── narrow_nav/
│       ├── core/
│       ├── simulation/
│       ├── humans/
│       ├── perception/
│       ├── dataset/
│       ├── prediction/
│       ├── risk/
│       ├── navigation/
│       ├── evaluation/
│       └── visualization/
│
├── ros2_ws/                   # [05] ROS 2 / NAV2 / ROBOT
│   └── src/
│       ├── narrow_nav_interfaces/
│       ├── narrow_nav_bridge/
│       ├── narrow_nav_bringup/
│       ├── narrow_nav_description/
│       └── narrow_nav_hardware/
│
├── scripts/                   # [06] ENTRYPOINTS / CLI
│   ├── simulation/
│   ├── dataset/
│   ├── training/
│   ├── evaluation/
│   └── deployment/
│
├── data/                      # [07] DỮ LIỆU NGHIÊN CỨU
│   ├── external/
│   ├── raw/
│   ├── validated/
│   ├── processed/
│   ├── splits/
│   └── manifests/
│
├── experiments/               # [08] ĐỊNH NGHĨA THỰC NGHIỆM
│   ├── human_behavior/
│   ├── dataset_validation/
│   ├── trajectory_prediction/
│   ├── navigation/
│   ├── ablations/
│   ├── generalization/
│   └── sim_to_real/
│
├── runs/                      # [09] OUTPUT MỖI LẦN CHẠY
│   ├── simulation/
│   ├── data_generation/
│   ├── training/
│   └── evaluation/
│
├── artifacts/                 # [10] CHECKPOINT / MODEL
│   ├── checkpoints/
│   ├── models/
│   └── releases/
│
├── tests/                     # [11] KIỂM THỬ
│   ├── unit/
│   ├── integration/
│   ├── simulation/
│   ├── data_quality/
│   └── regression/
│
├── reports/                   # [12] KẾT QUẢ ĐÃ XÁC THỰC
│   ├── dataset/
│   ├── benchmark/
│   ├── prediction/
│   ├── navigation/
│   └── figures/
│
├── paper/                     # [13] CÔNG BỐ KHOA HỌC
│   ├── manuscript/
│   ├── tables/
│   ├── figures/
│   └── supplementary/
│
└── deployment/                # [14] ĐÓNG GÓI & TRIỂN KHAI
    ├── docker/
    ├── environments/
    └── hardware_configs/