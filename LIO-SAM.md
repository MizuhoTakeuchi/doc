# LIO-SAM 技術ドキュメント

LIO-SAM (Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping) は、LiDARとIMUを緊密に統合した自己位置推定・地図作成システムです。

## 目次

1. [システム構造](#1-システム構造)
2. [アルゴリズム](#2-アルゴリズム)
3. [クラス図](#3-クラス図)
4. [GTSAMによる因子グラフ](#4-gtsamによる因子グラフ)

---

## 1. システム構造

### 1.1 ディレクトリ構成

```
LIO-SAM/
├── src/                              # ソースコード
│   ├── imuPreintegration.cpp          # IMU前積分と因子グラフ構築
│   ├── imageProjection.cpp            # ポイントクラウド投影と動き補正
│   ├── featureExtraction.cpp          # 特徴抽出（エッジ・サーフェス）
│   └── mapOptmization.cpp             # スキャン・マップ最適化とループクロージング
├── include/
│   └── utility.h                      # パラメータサーバーとユーティリティ関数
├── config/
│   └── params.yaml                    # 設定ファイル
├── launch/
│   ├── run.launch                     # メインランチファイル
│   └── include/
│       ├── module_loam.launch         # 4つのROSノード起動
│       ├── module_navsat.launch       # GPS統合
│       └── module_rviz.launch         # 可視化
├── msg/
│   └── cloud_info.msg                 # カスタムメッセージ定義
└── srv/
    └── save_map.srv                   # マップ保存サービス
```

### 1.2 ROSノード構成

LIO-SAMは4つの主要ノードで構成されています：

| ノード名 | ソースファイル | 役割 |
|---------|--------------|------|
| `lio_sam_imuPreintegration` | imuPreintegration.cpp | IMU前処理とiSAM2最適化 |
| `lio_sam_imageProjection` | imageProjection.cpp | 範囲画像投影と動き補正 |
| `lio_sam_featureExtraction` | featureExtraction.cpp | LOAM特徴抽出 |
| `lio_sam_mapOptmization` | mapOptmization.cpp | スキャン・マップ最適化 |

### 1.3 データフロー

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           センサー入力                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  IMU (200Hz)              LiDAR (10Hz)              GPS (optional)      │
│      │                        │                          │              │
│      ▼                        ▼                          │              │
│ ┌────────────────┐    ┌────────────────┐                │              │
│ │IMU Preintegr.  │    │Image Projection│                │              │
│ │                │    │                │                │              │
│ │・IMU積分       │    │・範囲画像投影  │                │              │
│ │・バイアス推定  │    │・動き補正      │                │              │
│ └───────┬────────┘    └───────┬────────┘                │              │
│         │                     │                          │              │
│         │                     ▼                          │              │
│         │            ┌────────────────┐                  │              │
│         │            │Feature Extract.│                  │              │
│         │            │                │                  │              │
│         │            │・曲率計算      │                  │              │
│         │            │・エッジ抽出    │                  │              │
│         │            │・サーフェス抽出│                  │              │
│         │            └───────┬────────┘                  │              │
│         │                    │                           │              │
│         ▼                    ▼                           ▼              │
│     ┌─────────────────────────────────────────────────────────┐        │
│     │                  Map Optimization                        │        │
│     │                                                          │        │
│     │  ・スキャン・マップ マッチング（ICP）                     │        │
│     │  ・GTSAM因子グラフ最適化                                 │        │
│     │  ・ループクロージング検出                                 │        │
│     │  ・GPS統合                                               │        │
│     └─────────────────────────────────────────────────────────┘        │
│                              │                                          │
│                              ▼                                          │
│                    最適化された姿勢・地図                                │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.4 トピック構成

| トピック | 型 | 説明 |
|---------|---|------|
| `lio_sam/deskew/cloud_info` | cloud_info | 動き補正済みポイント＋範囲画像情報 |
| `lio_sam/feature/cloud_info` | cloud_info | コーナー・サーフェス特徴 |
| `odometry/imu_incremental` | nav_msgs/Odometry | IMU前積分オドメトリー（高周波） |
| `lio_sam/mapping/odometry` | nav_msgs/Odometry | 最適化後の最終オドメトリー |
| `lio_sam/mapping/cloud_registered` | PointCloud2 | 登録済みポイントクラウド |

---

## 2. アルゴリズム

### 2.1 全体フロー

LIO-SAMのアルゴリズムは4段階のパイプラインで構成されています：

```
[Stage 1] IMU前処理 (200Hz)
    │
    ├── IMU測定値の座標変換
    ├── PreintegratedImuMeasurementsによる積分
    └── iSAM2による状態推定（姿勢、速度、バイアス）
    │
[Stage 2] 画像投影 (10Hz)
    │
    ├── ポイントクラウドの範囲画像への投影
    ├── IMU/オドメトリーによる動き補正（deskew）
    └── ポイント属性の計算
    │
[Stage 3] 特徴抽出
    │
    ├── 曲率（smoothness）計算
    ├── 隠蔽点・平行ビーム検出
    └── エッジ・サーフェス特徴の分類
    │
[Stage 4] マップ最適化
    │
    ├── スキャン・マップ マッチング（L-M法）
    ├── キーフレーム追加
    ├── ループクロージング検出・処理
    └── GTSAM因子グラフ最適化
```

### 2.2 IMU前積分

IMU前積分は連続するフレーム間のIMU測定値を統合し、相対的な姿勢変化を計算します。

**数学的モデル：**

```
状態変数: x = [R, p, v, b_a, b_g]
  - R: 回転行列
  - p: 位置
  - v: 速度
  - b_a: 加速度バイアス
  - b_g: ジャイロバイアス

IMU測定モデル:
  ã = a + b_a + R^T * g + n_a
  ω̃ = ω + b_g + n_g

前積分（時刻 i から j まで）:
  Δv_ij = Σ R_i^T * (ã_k - b_a) * Δt
  Δp_ij = Σ Δv_ij * Δt + 0.5 * R_i^T * (ã_k - b_a) * Δt²
  ΔR_ij = Π exp((ω̃_k - b_g) * Δt)
```

### 2.3 範囲画像投影

ポイントクラウドを2D範囲画像に投影し、効率的な近傍探索を可能にします。

```
入力: 3Dポイント P = (x, y, z)

範囲計算:
  range = √(x² + y² + z²)

角度計算:
  verticalAngle = atan2(z, √(x² + y²))
  horizontalAngle = atan2(y, x)

画像座標:
  rowInd = (verticalAngle + fov_down) / angRes_vertical
  colInd = (horizontalAngle + π) / angRes_horizontal

出力: 範囲画像 I[rowInd][colInd] = range
```

### 2.4 特徴抽出（LOAM手法）

曲率に基づいてエッジ点とサーフェス点を分類します。

**曲率計算：**

```cpp
// 各点 i に対して、前後5点ずつの範囲差を計算
curvature[i] = 0;
for (j = -5; j <= 5; j++) {
    if (j != 0) {
        curvature[i] += range[i+j] - range[i];
    }
}
curvature[i] = curvature[i] * curvature[i];
```

**特徴分類：**

| 特徴タイプ | 条件 | 用途 |
|-----------|------|------|
| エッジ点 | 曲率 > edgeThreshold | 直線・角の検出 |
| サーフェス点 | 曲率 < surfThreshold | 平面の検出 |

**セクター分割：**

スキャンラインを6セクターに分割し、各セクターから均等に特徴を抽出：
- エッジ点: 最大20点/セクター
- サーフェス点: 残りの有効点

### 2.5 スキャン・マップ マッチング

現在のスキャンをローカルマップにマッチングし、姿勢を推定します。

**エッジ点の最適化（Point-to-Line）：**

```
1. 現在のエッジ点 p に対して、KDTreeで最近傍点 q1, q2 を探索
2. 直線 L = q1 + t(q2 - q1) を定義
3. 点から直線への距離を最小化:
   d = ||(p - q1) × (q2 - q1)|| / ||q2 - q1||
```

**サーフェス点の最適化（Point-to-Plane）：**

```
1. 現在のサーフェス点 p に対して、KDTreeで最近傍点群を探索
2. 最小二乗法で平面 ax + by + cz + d = 0 をフィッティング
3. 点から平面への距離を最小化:
   d = |ax_p + by_p + cz_p + d| / √(a² + b² + c²)
```

**Levenberg-Marquardt最適化：**

```
状態: x = [roll, pitch, yaw, x, y, z]^T

残差: r(x) = [d_edge_1, d_edge_2, ..., d_surf_1, d_surf_2, ...]^T

更新: Δx = -(J^T J + λI)^(-1) J^T r

反復: x_{k+1} = x_k + Δx（収束まで最大30回）
```

### 2.6 ループクロージング

過去に訪れた場所を検出し、ドリフトを補正します。

```
[ループクロージング処理フロー]

1. 候補検出
   ├── 現在位置から半径 R 内の過去キーフレームを検索
   └── 時間差が T 秒以上のものをフィルタリング

2. ICP マッチング
   ├── 候補キーフレーム周辺のマップを構築
   ├── 現在スキャンとICPでアライメント
   └── フィッティングスコアが閾値以下なら成功

3. 因子グラフ更新
   ├── ループ制約を BetweenFactor として追加
   └── iSAM2で再最適化

4. 軌跡修正
   └── 全キーフレームの姿勢を更新
```

---

## 3. クラス図

### 3.1 クラス継承関係

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          ParamServer                                     │
│                          (utility.h)                                     │
│                                                                          │
│  [設定パラメータの一元管理]                                              │
│  - トピック名                                                            │
│  - フレーム定義                                                          │
│  - センサー設定                                                          │
│  - IMUノイズパラメータ                                                   │
│  - LOAM閾値                                                              │
│  - ループクロージング設定                                                │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │
                              │ 継承
          ┌───────────────────┼───────────────────┬───────────────────┐
          │                   │                   │                   │
          ▼                   ▼                   ▼                   ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ ImageProjection │ │FeatureExtraction│ │IMUPreintegration│ │ mapOptimization │
│                 │ │                 │ │                 │ │                 │
│ [範囲画像投影]   │ │ [特徴抽出]      │ │ [IMU前積分]     │ │ [マップ最適化]   │
└─────────────────┘ └─────────────────┘ └─────────────────┘ └─────────────────┘
```

### 3.2 詳細クラス図

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           ParamServer                                    │
├─────────────────────────────────────────────────────────────────────────┤
│ # nh: ros::NodeHandle                                                   │
│ # pointCloudTopic: string                                               │
│ # imuTopic: string                                                      │
│ # odomTopic: string                                                     │
│ # gpsTopic: string                                                      │
│ # lidarFrame: string                                                    │
│ # baselinkFrame: string                                                 │
│ # odometryFrame: string                                                 │
│ # mapFrame: string                                                      │
│ # N_SCAN: int                                                           │
│ # Horizon_SCAN: int                                                     │
│ # imuAccNoise: float                                                    │
│ # imuGyrNoise: float                                                    │
│ # edgeThreshold: float                                                  │
│ # surfThreshold: float                                                  │
├─────────────────────────────────────────────────────────────────────────┤
│ + ParamServer()                                                         │
│ + imuConverter(imu_in: sensor_msgs::Imu): sensor_msgs::Imu              │
└─────────────────────────────────────────────────────────────────────────┘
                                    △
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
┌───────┴───────────────┐   ┌───────┴───────────────┐   ┌───────┴───────────────┐
│    ImageProjection    │   │   FeatureExtraction   │   │   IMUPreintegration   │
├───────────────────────┤   ├───────────────────────┤   ├───────────────────────┤
│ - subImu              │   │ - subLaserCloudInfo   │   │ - imuIntegratorOpt_   │
│ - subOdom             │   │ - pubCornerPoints     │   │ - imuIntegratorImu_   │
│ - subLaserCloud       │   │ - pubSurfacePoints    │   │ - optimizer           │
│ - pubExtractedCloud   │   │ - cloudSmoothness[]   │   │ - graphFactors        │
│ - pubLaserCloudInfo   │   │ - cloudCurvature[]    │   │ - graphValues         │
│ - imuQueue            │   │ - cloudLabel[]        │   │ - prevPose_           │
│ - odomQueue           │   │ - cornerCloud         │   │ - prevVel_            │
│ - cloudQueue          │   │ - surfaceCloud        │   │ - prevBias_           │
│ - rangeMat            │   ├───────────────────────┤   ├───────────────────────┤
│ - fullCloud           │   │ + laserCloudInfoHdlr()│   │ + odometryHandler()   │
│ - extractedCloud      │   │ + calculateSmooth()   │   │ + imuHandler()        │
├───────────────────────┤   │ + markOccludedPts()   │   │ + resetOptimization() │
│ + imuHandler()        │   │ + extractFeatures()   │   │ + failureDetection()  │
│ + odometryHandler()   │   └───────────────────────┘   │ + IMUPreintegration() │
│ + cloudHandler()      │                               └───────────────────────┘
│ + cachePointCloud()   │
│ + deskewInfo()        │                   ┌───────────────────────────────────┐
│ + projectPointCloud() │                   │       TransformFusion             │
│ + cloudExtraction()   │                   ├───────────────────────────────────┤
│ + publishClouds()     │                   │ - lidarOdomAffine                 │
│ + resetParameters()   │                   │ - imuOdomAffineFront/Back         │
└───────────────────────┘                   ├───────────────────────────────────┤
                                            │ + lidarOdometryHandler()          │
                                            │ + imuOdometryHandler()            │
                                            └───────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                            mapOptimization                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│ - isam: gtsam::ISAM2*                                                       │
│ - gtSAMgraph: gtsam::NonlinearFactorGraph                                   │
│ - initialEstimate: gtsam::Values                                            │
│ - isamCurrentEstimate: gtsam::Values                                        │
│ - cloudKeyPoses3D: PointCloud<PointType>::Ptr                               │
│ - cloudKeyPoses6D: PointCloud<PointTypePose>::Ptr                           │
│ - cornerCloudKeyFrames: vector<PointCloud<PointType>::Ptr>                  │
│ - surfCloudKeyFrames: vector<PointCloud<PointType>::Ptr>                    │
│ - laserCloudCornerLast: PointCloud<PointType>::Ptr                          │
│ - laserCloudSurfLast: PointCloud<PointType>::Ptr                            │
│ - laserCloudCornerFromMap: PointCloud<PointType>::Ptr                       │
│ - laserCloudSurfFromMap: PointCloud<PointType>::Ptr                         │
│ - kdtreeCornerFromMap: pcl::KdTreeFLANN<PointType>::Ptr                     │
│ - kdtreeSurfFromMap: pcl::KdTreeFLANN<PointType>::Ptr                       │
│ - transformTobeMapped[6]: float                                             │
│ - loopIndexQueue: deque<pair<int,int>>                                      │
│ - loopPoseQueue: deque<gtsam::Pose3>                                        │
│ - loopNoiseQueue: deque<gtsam::noiseModel::Diagonal::shared_ptr>            │
├─────────────────────────────────────────────────────────────────────────────┤
│ + allocateMemory()                                                          │
│ + laserCloudInfoHandler()                                                   │
│ + gpsHandler()                                                              │
│ + pointAssociateToMap()                                                     │
│ + updatePointAssociateToMap()                                               │
│ + extractSurroundingKeyFrames()                                             │
│ + downsampleCurrentScan()                                                   │
│ + scan2MapOptimization()                                                    │
│ + cornerOptimization()                                                      │
│ + surfOptimization()                                                        │
│ + combineOptimizationCoeffs()                                               │
│ + LMOptimization()                                                          │
│ + saveKeyFramesAndFactor()                                                  │
│ + addOdomFactor()                                                           │
│ + addGPSFactor()                                                            │
│ + addLoopFactor()                                                           │
│ + correctPoses()                                                            │
│ + loopClosureThread()                                                       │
│ + performLoopClosure()                                                      │
│ + detectLoopClosureDistance()                                               │
│ + visualizeGlobalMapThread()                                                │
│ + saveMapService()                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 クラス間の関係

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              依存関係図                                       │
└──────────────────────────────────────────────────────────────────────────────┘

                         sensor_msgs/PointCloud2
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────┐
│                    ImageProjection                           │
│                                                              │
│  入力: PointCloud2, IMU, Odom                               │
│  出力: cloud_info (deskewed + range image)                   │
└────────────────────────────┬────────────────────────────────┘
                             │ cloud_info
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   FeatureExtraction                          │
│                                                              │
│  入力: cloud_info                                           │
│  出力: cloud_info (with corner + surface features)           │
└────────────────────────────┬────────────────────────────────┘
                             │ cloud_info (features)
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    mapOptimization                           │
│                                                              │
│  入力: cloud_info, GPS (optional)                           │
│  出力: Odometry, Map, Trajectory                            │
│                                                              │
│  内部処理:                                                   │
│  ├── Scan-to-Map Matching                                   │
│  ├── GTSAM Factor Graph Optimization                        │
│  └── Loop Closure Detection & Correction                    │
└────────────────────────────┬────────────────────────────────┘
                             │ lio_sam/mapping/odometry
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   IMUPreintegration                          │
│                                                              │
│  入力: IMU, mapping/odometry (correction)                   │
│  出力: odometry/imu_incremental (high-frequency)            │
│                                                              │
│  GTSAM: PreintegratedImuMeasurements + iSAM2                │
└────────────────────────────┬────────────────────────────────┘
                             │ imu_incremental
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    TransformFusion                           │
│                                                              │
│  入力: IMU odometry, LiDAR odometry                         │
│  出力: Fused odometry                                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. GTSAMによる因子グラフ

### 4.1 GTSAMの概要

GTSAM (Georgia Tech Smoothing and Mapping) は、ロボットのSLAM問題を解くためのC++ライブラリです。LIO-SAMでは以下の目的で使用されています：

- **IMU前積分**: 連続するIMU測定値の効率的な統合
- **増分最適化**: iSAM2による高速な姿勢推定
- **マルチセンサー融合**: GPS、ループクロージング制約の統合

### 4.2 因子グラフの基本構造

```
因子グラフ G = (V, F) は以下で構成される:
  - V: 変数ノード（推定対象）
  - F: 因子ノード（制約条件）

LIO-SAMの変数ノード:
  X(i): キーフレーム i の姿勢 (gtsam::Pose3)
  V(i): キーフレーム i の速度 (gtsam::Vector3)
  B(i): キーフレーム i のIMUバイアス (gtsam::imuBias::ConstantBias)

LIO-SAMの因子ノード:
  - PriorFactor: 事前分布制約
  - BetweenFactor: オドメトリー/ループ制約
  - ImuFactor: IMU測定制約
  - GPSFactor: GPS観測制約
```

### 4.3 因子グラフの可視化

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     LIO-SAM 因子グラフ構造                               │
└─────────────────────────────────────────────────────────────────────────┘

   Prior          Prior         Prior
     │              │             │
     ▼              ▼             ▼
   ┌───┐         ┌───┐         ┌───┐
   │X(0)│◄───────│V(0)│◄───────│B(0)│
   └─┬─┘         └─┬─┘         └─┬─┘
     │             │             │
     │  IMUFactor  │             │
     │◄────────────┼─────────────┤
     │             │             │
     │  Odom       │             │
     ▼  Factor     ▼             ▼
   ┌───┐         ┌───┐         ┌───┐
   │X(1)│◄───────│V(1)│◄───────│B(1)│
   └─┬─┘         └─┬─┘         └─┬─┘
     │             │             │
     │  IMUFactor  │             │
     │◄────────────┼─────────────┤
     │             │             │
     │  Odom       │             │
     ▼  Factor     ▼             ▼
   ┌───┐         ┌───┐         ┌───┐
   │X(2)│◄───────│V(2)│◄───────│B(2)│
   └─┬─┘         └─┬─┘         └─┬─┘
     │             │             │
     :             :             :
     │             │             │
     ▼             ▼             ▼
   ┌───┐         ┌───┐         ┌───┐
   │X(n)│◄───────│V(n)│◄───────│B(n)│
   └───┘         └───┘         └───┘
     ▲
     │
     │  Loop Closure Factor
     └────────────────────────────┐
                                  │
   ┌───┐                          │
   │X(m)│◄────────────────────────┘
   └───┘
     ▲
     │
   GPS Factor
```

### 4.4 IMU前積分における因子グラフ（imuPreintegration.cpp）

```cpp
// GTSAM 初期化
gtsam::ISAM2Params optParameters;
optParameters.relinearizeThreshold = 0.1;
optParameters.relinearizeSkip = 1;
optimizer = new gtsam::ISAM2(optParameters);

// IMU前積分パラメータ
boost::shared_ptr<gtsam::PreintegrationParams> p =
    gtsam::PreintegrationParams::MakeSharedU(imuGravity);
p->accelerometerCovariance = gtsam::I_3x3 * pow(imuAccNoise, 2);
p->gyroscopeCovariance = gtsam::I_3x3 * pow(imuGyrNoise, 2);
p->integrationCovariance = gtsam::I_3x3 * pow(1e-4, 2);

imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
```

**初期制約の追加：**

```cpp
// 初期姿勢の事前因子
gtsam::PriorFactor<gtsam::Pose3> priorPose(
    X(0),
    prevPose_,
    priorPoseNoise
);
graphFactors.add(priorPose);

// 初期速度の事前因子
gtsam::PriorFactor<gtsam::Vector3> priorVel(
    V(0),
    prevVel_,
    priorVelNoise
);
graphFactors.add(priorVel);

// 初期バイアスの事前因子
gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
    B(0),
    prevBias_,
    priorBiasNoise
);
graphFactors.add(priorBias);
```

**IMU因子の追加：**

```cpp
// IMU測定値の積分
while (currentImuTime < currentCorrectionTime) {
    double dt = currentImuTime - lastImuTime;
    imuIntegratorOpt_->integrateMeasurement(
        gtsam::Vector3(acc_x, acc_y, acc_z),
        gtsam::Vector3(gyr_x, gyr_y, gyr_z),
        dt
    );
}

// IMU因子の作成と追加
gtsam::ImuFactor imuFactor(
    X(key - 1), V(key - 1),
    X(key), V(key),
    B(key - 1),
    *imuIntegratorOpt_
);
graphFactors.add(imuFactor);

// バイアス制約
graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    B(key - 1), B(key),
    gtsam::imuBias::ConstantBias(),
    biasNoiseModel
));
```

### 4.5 マップ最適化における因子グラフ（mapOptmization.cpp）

**オドメトリー因子の追加：**

```cpp
void addOdomFactor() {
    if (cloudKeyPoses3D->points.empty()) {
        // 最初のキーフレーム: 事前因子
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()
            );
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0,
            trans2gtsamPose(transformTobeMapped),
            priorNoise
        ));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    } else {
        // 連続キーフレーム間: BetweenFactor
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished()
            );

        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(
            cloudKeyPoses6D->points.back()
        );
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);

        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloudKeyPoses3D->size() - 1,
            cloudKeyPoses3D->size(),
            poseFrom.between(poseTo),
            odometryNoise
        ));

        initialEstimate.insert(
            cloudKeyPoses3D->size(),
            poseTo
        );
    }
}
```

**GPS因子の追加：**

```cpp
void addGPSFactor() {
    if (gpsQueue.empty()) return;

    // 初期キーフレームがない場合、または移動距離が小さい場合はスキップ
    if (cloudKeyPoses3D->points.empty()) return;

    while (!gpsQueue.empty()) {
        // タイムスタンプが近いGPS測定を取得
        if (gpsQueue.front().header.stamp.toSec() <
            timeLaserInfoCur - 0.2) {
            gpsQueue.pop_front();
        } else if (gpsQueue.front().header.stamp.toSec() >
                   timeLaserInfoCur + 0.2) {
            break;
        } else {
            nav_msgs::Odometry thisGPS = gpsQueue.front();
            gpsQueue.pop_front();

            // GPS共分散チェック
            float noise_x = thisGPS.pose.covariance[0];
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];

            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                continue;

            // GPS因子の追加
            gtsam::Vector3 gpsPosition(
                thisGPS.pose.pose.position.x,
                thisGPS.pose.pose.position.y,
                thisGPS.pose.pose.position.z
            );

            gtsam::noiseModel::Diagonal::shared_ptr gpsNoise =
                gtsam::noiseModel::Diagonal::Variances(
                    gtsam::Vector3(noise_x, noise_y, noise_z)
                );

            gtsam::GPSFactor gpsFactor(
                cloudKeyPoses3D->size(),
                gpsPosition,
                gpsNoise
            );
            gtSAMgraph.add(gpsFactor);
        }
    }
}
```

**ループクロージング因子の追加：**

```cpp
void addLoopFactor() {
    if (loopIndexQueue.empty()) return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];

        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            indexFrom,
            indexTo,
            poseBetween,
            noiseBetween
        ));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}
```

### 4.6 iSAM2による最適化

```cpp
// 因子グラフの更新と最適化
isam->update(gtSAMgraph, initialEstimate);
isam->update();  // 追加の反復で収束を促進

// ループクロージング検出時は複数回更新
if (aLoopIsClosed) {
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();
}

// 因子グラフをクリア
gtSAMgraph.resize(0);
initialEstimate.clear();

// 最適化結果の取得
isamCurrentEstimate = isam->calculateEstimate();

// 最新の姿勢を取得
gtsam::Pose3 latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(
    isamCurrentEstimate.size() - 1
);

// 共分散の取得（オプション）
gtsam::Marginals marginals(gtSAMgraph, isamCurrentEstimate);
gtsam::Matrix poseCovariance = marginals.marginalCovariance(
    isamCurrentEstimate.size() - 1
);
```

### 4.7 因子グラフの数学的背景

**最適化問題の定式化：**

```
最小化問題:
  x* = argmin Σ ||f_i(x_i)||²_{Σ_i}
       x

ここで:
  - x_i: 変数ノード（姿勢、速度、バイアス）
  - f_i: 因子関数（測定誤差）
  - Σ_i: ノイズ共分散行列
  - ||.||²_Σ: マハラノビス距離
```

**iSAM2の増分更新：**

```
1. 新しい測定値が到着
2. 影響を受けるClique（変数のグループ）を特定
3. 影響を受けるCliqueのみを再線形化
4. Bayes木構造を更新
5. 後ろ向き/前向きの代入で解を更新

計算量: O(変更された変数数) << O(全変数数)
```

### 4.8 ノイズモデル

LIO-SAMで使用されるGTSAMノイズモデル：

| 因子タイプ | ノイズモデル | 典型的な値 |
|-----------|-------------|-----------|
| Prior Pose | Diagonal | [1e-2, 1e-2, π², 1e8, 1e8, 1e8] |
| Prior Velocity | Isotropic | σ = 1e4 |
| Prior Bias | Isotropic | σ = 1e-3 |
| Odometry | Diagonal | [1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4] |
| IMU | 自動計算 | 前積分共分散に基づく |
| GPS | Diagonal | 測定共分散に基づく |
| Loop Closure | Diagonal | ICPフィッティングスコアに基づく |

---

## 付録

### A. 主要パラメータ一覧

| パラメータ                             | デフォルト値  | 説明                  |
| --------------------------------- | ------- | ------------------- |
| `edgeThreshold`                   | 1.0     | エッジ点判定の曲率閾値         |
| `surfThreshold`                   | 0.1     | サーフェス点判定の曲率閾値       |
| `imuAccNoise`                     | 3.99e-3 | 加速度ノイズ [m/s²/√Hz]   |
| `imuGyrNoise`                     | 1.56e-3 | ジャイロノイズ [rad/s/√Hz] |
| `imuAccBiasN`                     | 6.99e-5 | 加速度バイアスノイズ          |
| `imuGyrBiasN`                     | 2.44e-5 | ジャイロバイアスノイズ         |
| `surroundingKeyframeSearchRadius` | 50.0    | マップ検索半径 [m]         |
| `loopClosureFrequency`            | 1.0     | ループ検出周波数 [Hz]       |
| `historyKeyframeSearchRadius`     | 15.0    | ループ検索半径 [m]         |
| `historyKeyframeFitnessScore`     | 0.3     | ICP収束閾値             |

### B. 座標系定義

```
      z (up)
       │
       │
       │
       └──────── x (forward)
      /
     /
    y (left)

フレーム:
  - lidarFrame: LiDARセンサー座標系
  - baselinkFrame: ロボット基準座標系
  - odometryFrame: オドメトリー座標系（初期位置）
  - mapFrame: 地図座標系（グローバル）
```

### C. 参考文献

1. T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus, "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2020.

2. GTSAM: Georgia Tech Smoothing and Mapping library, https://gtsam.org/

3. C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry," IEEE Transactions on Robotics, 2017.
