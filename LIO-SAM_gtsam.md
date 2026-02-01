# LIO-SAMにおけるGTSAM因子グラフ最適化 - 初学者向け解説

このドキュメントでは、LIO-SAMで使用されているGTSAM（Georgia Tech Smoothing and Mapping）ライブラリによる因子グラフ最適化について、初学者にもわかりやすく解説します。

---

## 目次

1. [因子グラフとは何か？](#1-因子グラフとは何か)
2. [なぜ因子グラフを使うのか？](#2-なぜ因子グラフを使うのか)
3. [GTSAMの基本概念](#3-gtsamの基本概念)
4. [LIO-SAMでの因子グラフの使われ方](#4-lio-samでの因子グラフの使われ方)
5. [コード解説：IMU前積分](#5-コード解説imu前積分)
6. [コード解説：マップ最適化](#6-コード解説マップ最適化)
7. [iSAM2による効率的な最適化](#7-isam2による効率的な最適化)
8. [まとめ](#8-まとめ)

---

## 1. 因子グラフとは何か？

### 1.1 日常生活での例え

因子グラフを理解するために、まず日常的な例で考えてみましょう。

**例：友人の家を探す**

あなたは初めて友人の家を訪ねようとしています。以下の情報があります：

1. 「駅から北に500m歩いた」（移動の情報）
2. 「コンビニの隣にある」（ランドマークとの関係）
3. 「GPSでは緯度35.6、経度139.7」（絶対位置の情報）

これらの情報には**すべて誤差**があります。GPSは数メートルずれるかもしれませんし、歩数で測った距離も正確ではありません。因子グラフは、これらの**不確かな情報を組み合わせて、最も確からしい位置を推定する**数学的な枠組みです。

### 1.2 因子グラフの構成要素

```
┌─────────────────────────────────────────────────────────────────────┐
│                      因子グラフの構成要素                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│    ┌─────┐        ┌─────┐        ┌─────┐        ┌─────┐            │
│    │ X₀  │────────│ X₁  │────────│ X₂  │────────│ X₃  │  ← 変数    │
│    └──┬──┘        └──┬──┘        └──┬──┘        └──┬──┘    ノード   │
│       │              │              │              │               │
│    ┌──┴──┐        ┌──┴──┐        ┌──┴──┐        ┌──┴──┐            │
│    │Prior│        │Odom │        │Odom │        │GPS  │  ← 因子    │
│    └─────┘        └─────┘        └─────┘        └─────┘    ノード   │
│                                                                     │
│  変数ノード（○）: 推定したい値（例：ロボットの位置・姿勢）          │
│  因子ノード（□）: 変数間の制約条件（例：センサー測定値）            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**変数ノード（Variable Node）**
- 推定したい未知の値
- LIO-SAMでは：ロボットの姿勢(X)、速度(V)、IMUバイアス(B)

**因子ノード（Factor Node）**
- 変数に対する制約条件
- センサーの測定値や、変数間の関係を表す
- LIO-SAMでは：オドメトリ因子、GPS因子、ループクロージング因子など

---

## 2. なぜ因子グラフを使うのか？

### 2.1 単純なアプローチの問題点

ロボットの位置推定を考えてみましょう。単純なアプローチとして：

```
方法1: オドメトリだけを使う
  → 誤差が蓄積してどんどんずれていく（ドリフト問題）

方法2: GPSだけを使う
  → 測定ノイズが大きく、軌跡がガタガタになる

方法3: 最新の測定値だけを使う
  → 過去の情報を捨ててしまい、もったいない
```

### 2.2 因子グラフのメリット

```
┌─────────────────────────────────────────────────────────────────────┐
│                    因子グラフのメリット                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. 複数のセンサー情報を統一的に扱える                               │
│     ┌────────┐  ┌────────┐  ┌────────┐                              │
│     │  IMU   │  │ LiDAR  │  │  GPS   │                              │
│     └───┬────┘  └───┬────┘  └───┬────┘                              │
│         └──────────┬──────────┘                                     │
│                    ▼                                                │
│              ┌──────────┐                                           │
│              │因子グラフ│ → 最適な推定値                            │
│              └──────────┘                                           │
│                                                                     │
│  2. 不確かさを適切に考慮できる                                       │
│     - 信頼性の高い測定は重視                                         │
│     - ノイズの大きい測定は軽視                                       │
│                                                                     │
│  3. 後から情報を追加できる（ループクロージング）                     │
│     - 「ここは前に来た場所だ！」という情報で過去を修正               │
│                                                                     │
│  4. 計算効率が良い（iSAM2）                                          │
│     - 全部を再計算せず、変更部分だけを更新                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. GTSAMの基本概念

### 3.1 GTSAMとは

GTSAM（Georgia Tech Smoothing and Mapping）は、ジョージア工科大学で開発されたC++ライブラリで、因子グラフを使った最適化問題を効率的に解くことができます。

### 3.2 LIO-SAMで使用するGTSAMのクラス

```cpp
// LIO-SAMのソースコードから抜粋（imuPreintegration.cpp）

#include <gtsam/geometry/Pose3.h>              // 3D姿勢（位置＋回転）
#include <gtsam/slam/PriorFactor.h>            // 事前確率因子
#include <gtsam/slam/BetweenFactor.h>          // 相対変換因子
#include <gtsam/navigation/GPSFactor.h>        // GPS因子
#include <gtsam/navigation/ImuFactor.h>        // IMU因子
#include <gtsam/nonlinear/NonlinearFactorGraph.h>  // 因子グラフ
#include <gtsam/nonlinear/Values.h>            // 変数の値
#include <gtsam/nonlinear/ISAM2.h>             // 増分最適化器

// 変数のシンボル定義
using gtsam::symbol_shorthand::X;  // 姿勢 (Pose3)
using gtsam::symbol_shorthand::V;  // 速度 (Vector3)
using gtsam::symbol_shorthand::B;  // IMUバイアス
```

### 3.3 主要な因子タイプ

#### PriorFactor（事前確率因子）

「この変数はだいたいこの値です」という制約を表します。

```
用途: 初期位置の設定、絶対的な位置情報の追加

例: 「ロボットは原点(0,0,0)からスタートする」
    「この瞬間、GPSによると位置は(100, 200, 0)」
```

#### BetweenFactor（相対変換因子）

「2つの変数の間にはこの関係がある」という制約を表します。

```
用途: オドメトリ制約、ループクロージング制約

例: 「X₀からX₁へは、前に1m、右に0.5m移動した」
    「X₅とX₀は同じ場所である（ループ検出）」
```

#### ImuFactor（IMU因子）

IMUの測定値を使って、姿勢・速度・バイアスを制約します。

```
用途: 高周波のIMU測定値を統合

例: 「0.1秒間のIMU測定から、姿勢がこれだけ変化した」
```

#### GPSFactor（GPS因子）

GPSの測定値による位置制約を表します。

```
用途: グローバルな位置補正

例: 「現在位置は(緯度, 経度, 高度)付近である」
```

---

## 4. LIO-SAMでの因子グラフの使われ方

### 4.1 全体像

LIO-SAMでは、**2つの場所**で因子グラフが使われています：

```
┌─────────────────────────────────────────────────────────────────────┐
│                  LIO-SAMにおける因子グラフの使用場所                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │         1. IMU前積分 (imuPreintegration.cpp)                │   │
│  │                                                              │   │
│  │   目的: IMU測定値から高周波の姿勢推定                        │   │
│  │   更新頻度: ~200Hz（IMUの周波数）                            │   │
│  │   使用因子: PriorFactor, ImuFactor, BetweenFactor(バイアス)  │   │
│  │                                                              │   │
│  │   変数: X(姿勢), V(速度), B(IMUバイアス)                     │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼ 補正情報                             │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │        2. マップ最適化 (mapOptmization.cpp)                  │   │
│  │                                                              │   │
│  │   目的: キーフレーム間の姿勢を最適化                         │   │
│  │   更新頻度: ~10Hz（LiDARの周波数）                           │   │
│  │   使用因子: PriorFactor, BetweenFactor, GPSFactor            │   │
│  │            + ループクロージング制約                          │   │
│  │                                                              │   │
│  │   変数: X(キーフレームの姿勢)                                │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 時間軸で見た因子グラフの成長

```
時刻 t=0（初期化）:

    ┌─────┐
    │ X₀  │ ← 初期姿勢
    └──┬──┘
       │
    [Prior] ← 「X₀は原点である」という制約


時刻 t=1（1つ目のキーフレーム追加）:

    ┌─────┐         ┌─────┐
    │ X₀  │─────────│ X₁  │
    └──┬──┘         └──┬──┘
       │               │
    [Prior]      [BetweenFactor]
                 「X₀からX₁への相対変換」


時刻 t=5（ループクロージング検出！）:

    ┌─────┐         ┌─────┐         ┌─────┐         ┌─────┐         ┌─────┐
    │ X₀  │─────────│ X₁  │─────────│ X₂  │─────────│ X₃  │─────────│ X₄  │
    └──┬──┘         └─────┘         └─────┘         └─────┘         └──┬──┘
       │                                                               │
    [Prior]                                                      [GPSFactor]
       │                                                               │
       └───────────────────────────────────────────────────────────────┘
                              [LoopFactor]
                    「X₄はX₀と同じ場所だ！」

    → ループ制約により、X₁～X₃の姿勢も修正される
```

---

## 5. コード解説：IMU前積分

`imuPreintegration.cpp` の核心部分を初学者向けに解説します。

### 5.1 初期化処理

```cpp
// ファイル: src/imuPreintegration.cpp

IMUPreintegration()
{
    // IMU前積分のパラメータ設定
    boost::shared_ptr<gtsam::PreintegrationParams> p =
        gtsam::PreintegrationParams::MakeSharedU(imuGravity);

    // センサーのノイズ特性を設定
    // ┌─────────────────────────────────────────────────────────┐
    // │ accelerometerCovariance: 加速度計のノイズ               │
    // │   → 小さいほど加速度計を信頼                            │
    // │                                                         │
    // │ gyroscopeCovariance: ジャイロスコープのノイズ           │
    // │   → 小さいほどジャイロを信頼                            │
    // │                                                         │
    // │ integrationCovariance: 積分時の誤差                     │
    // │   → 時間とともに蓄積する不確かさ                        │
    // └─────────────────────────────────────────────────────────┘
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);
    p->gyroscopeCovariance     = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);
    p->integrationCovariance   = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);

    // 初期バイアスはゼロと仮定
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

    // 各変数の事前確率ノイズを設定
    // ┌─────────────────────────────────────────────────────────┐
    // │ priorPoseNoise: 初期姿勢の不確かさ                      │
    // │   → 6次元: roll, pitch, yaw, x, y, z                    │
    // │   → 値が小さい = 初期姿勢を強く信頼                     │
    // │                                                         │
    // │ priorVelNoise: 初期速度の不確かさ                       │
    // │   → 1e4 = 非常に不確か（最初は速度がわからない）        │
    // │                                                         │
    // │ priorBiasNoise: 初期バイアスの不確かさ                  │
    // │   → 1e-3 = バイアスはほぼゼロのはず                     │
    // └─────────────────────────────────────────────────────────┘
    priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    priorVelNoise  = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
    priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    // 2つの前積分器を作成
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
}
```

### 5.2 システム初期化時の因子グラフ構築

```cpp
// odometryHandler関数内 - 初期化部分
// ファイル: src/imuPreintegration.cpp (行274-316)

if (systemInitialized == false)
{
    // 最適化器をリセット
    resetOptimization();

    // ステップ1: 初期姿勢の因子を追加
    // ┌───────────────────────────────────────────────────────────┐
    // │ PriorFactor<Pose3>(X(0), prevPose_, priorPoseNoise)       │
    // │                                                           │
    // │ 意味: 「変数X(0)の値はprevPose_であり、                   │
    // │        その確信度はpriorPoseNoiseで表される」              │
    // │                                                           │
    // │ 図:    [Prior]                                            │
    // │           │                                               │
    // │           ▼                                               │
    // │        ┌─────┐                                            │
    // │        │ X₀  │ = prevPose_                                │
    // │        └─────┘                                            │
    // └───────────────────────────────────────────────────────────┘
    prevPose_ = lidarPose.compose(lidar2Imu);  // LiDAR座標からIMU座標へ変換
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
    graphFactors.add(priorPose);

    // ステップ2: 初期速度の因子を追加
    prevVel_ = gtsam::Vector3(0, 0, 0);  // 静止状態からスタート
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
    graphFactors.add(priorVel);

    // ステップ3: 初期バイアスの因子を追加
    prevBias_ = gtsam::imuBias::ConstantBias();  // ゼロバイアス
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
    graphFactors.add(priorBias);

    // ステップ4: 変数の初期値を設定
    graphValues.insert(X(0), prevPose_);
    graphValues.insert(V(0), prevVel_);
    graphValues.insert(B(0), prevBias_);

    // ステップ5: 最適化を実行
    optimizer.update(graphFactors, graphValues);

    // グラフをクリア（次回の増分更新のため）
    graphFactors.resize(0);
    graphValues.clear();

    systemInitialized = true;
}
```

この初期化後の因子グラフ：

```
初期化完了時の因子グラフ:

    [PriorPose]  [PriorVel]  [PriorBias]
         │            │           │
         ▼            ▼           ▼
      ┌─────┐     ┌─────┐     ┌─────┐
      │ X₀  │     │ V₀  │     │ B₀  │
      └─────┘     └─────┘     └─────┘

    X₀: 初期姿勢（位置と向き）
    V₀: 初期速度（静止 = [0,0,0]）
    B₀: 初期バイアス（ゼロ）
```

### 5.3 IMU測定値の積分と最適化

```cpp
// odometryHandler関数内 - 通常の最適化部分
// ファイル: src/imuPreintegration.cpp (行350-396)

// ステップ1: IMU測定値を積分
// ┌───────────────────────────────────────────────────────────────┐
// │ IMU測定値を時間積分して、相対的な姿勢変化を計算               │
// │                                                               │
// │ 例: t=0.00s から t=0.10s までの間に                          │
// │     加速度 [0.1, 0, 9.8] m/s² が10回測定された場合、          │
// │     integrateMeasurement() で積分すると：                     │
// │     - 速度変化: 約 0.1 m/s (前方向)                          │
// │     - 位置変化: 約 0.005 m (前方向)                          │
// │     - 姿勢変化: ジャイロ値に基づく                           │
// └───────────────────────────────────────────────────────────────┘
while (!imuQueOpt.empty())
{
    sensor_msgs::Imu *thisImu = &imuQueOpt.front();
    double imuTime = ROS_TIME(thisImu);

    if (imuTime < currentCorrectionTime - delta_t)
    {
        double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);

        // IMU測定値を積分器に追加
        imuIntegratorOpt_->integrateMeasurement(
            gtsam::Vector3(thisImu->linear_acceleration.x,
                          thisImu->linear_acceleration.y,
                          thisImu->linear_acceleration.z),
            gtsam::Vector3(thisImu->angular_velocity.x,
                          thisImu->angular_velocity.y,
                          thisImu->angular_velocity.z),
            dt);

        lastImuT_opt = imuTime;
        imuQueOpt.pop_front();
    }
    else
        break;
}

// ステップ2: IMU因子をグラフに追加
// ┌───────────────────────────────────────────────────────────────┐
// │ ImuFactor(X(key-1), V(key-1), X(key), V(key), B(key-1), ...)  │
// │                                                               │
// │ 意味: 「X(key-1)からX(key)への変化は、                       │
// │        積分したIMU測定値と整合する必要がある」                │
// │                                                               │
// │ 図:                                                          │
// │    ┌───────┐              ┌───────┐                          │
// │    │X(k-1) │──[IMU因子]──→│ X(k)  │                          │
// │    └───────┘              └───────┘                          │
// │    ┌───────┐              ┌───────┐                          │
// │    │V(k-1) │──[IMU因子]──→│ V(k)  │                          │
// │    └───────┘              └───────┘                          │
// │    ┌───────┐                                                  │
// │    │B(k-1) │──(バイアスは前ステップを使用)                   │
// │    └───────┘                                                  │
// └───────────────────────────────────────────────────────────────┘
const gtsam::PreintegratedImuMeasurements& preint_imu =
    dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
graphFactors.add(imu_factor);

// ステップ3: バイアス間の制約を追加
// ┌───────────────────────────────────────────────────────────────┐
// │ BetweenFactor<imuBias>(B(key-1), B(key), ゼロ, noise)         │
// │                                                               │
// │ 意味: 「バイアスはゆっくりとしか変化しない」                  │
// │        B(key) ≈ B(key-1) であるべき                          │
// │                                                               │
// │ なぜ重要？                                                    │
// │   IMUのバイアスは時間とともにドリフトするが、                 │
// │   急激には変化しない。この制約がないと、                      │
// │   最適化がバイアスを大きく変えてしまう可能性がある。          │
// └───────────────────────────────────────────────────────────────┘
graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    B(key - 1), B(key),
    gtsam::imuBias::ConstantBias(),  // ゼロ変化を期待
    gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

// ステップ4: LiDARオドメトリによる補正因子を追加
// ┌───────────────────────────────────────────────────────────────┐
// │ PriorFactor<Pose3>(X(key), curPose, correctionNoise)          │
// │                                                               │
// │ 意味: 「LiDARスキャンマッチングによると、                     │
// │        現在の姿勢はcurPoseである」                            │
// │                                                               │
// │ ポイント: これがIMU予測を「補正」する                         │
// │   - IMUだけだとドリフトする                                   │
// │   - LiDARの絶対的な姿勢情報で修正                             │
// └───────────────────────────────────────────────────────────────┘
gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise);
graphFactors.add(pose_factor);

// ステップ5: 変数の予測値を設定
// ┌───────────────────────────────────────────────────────────────┐
// │ predict(): 前のステップの状態とIMU積分から、                  │
// │            現在の状態を予測する                               │
// │                                                               │
// │ 予測値は最適化の初期値として使用される。                      │
// │ 良い初期値があると、最適化が速く収束する。                    │
// └───────────────────────────────────────────────────────────────┘
gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
graphValues.insert(X(key), propState_.pose());
graphValues.insert(V(key), propState_.v());
graphValues.insert(B(key), prevBias_);

// ステップ6: 最適化を実行
optimizer.update(graphFactors, graphValues);
optimizer.update();  // 2回実行で収束を改善

// ステップ7: 結果を取得
gtsam::Values result = optimizer.calculateEstimate();
prevPose_  = result.at<gtsam::Pose3>(X(key));
prevVel_   = result.at<gtsam::Vector3>(V(key));
prevState_ = gtsam::NavState(prevPose_, prevVel_);
prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
```

---

## 6. コード解説：マップ最適化

`mapOptmization.cpp` の因子グラフ関連部分を解説します。

### 6.1 iSAM2の初期化

```cpp
// ファイル: src/mapOptmization.cpp (行157-162)

mapOptimization()
{
    // iSAM2パラメータの設定
    // ┌───────────────────────────────────────────────────────────────┐
    // │ ISAM2Params: 増分最適化のパラメータ                           │
    // │                                                               │
    // │ relinearizeThreshold: 再線形化の閾値                          │
    // │   → 変数の変化がこの値を超えたら再線形化                      │
    // │   → 0.1 = 10%以上変化したら再計算                             │
    // │                                                               │
    // │ relinearizeSkip: 再線形化をスキップする頻度                   │
    // │   → 1 = 毎回チェック（計算コストと精度のトレードオフ）        │
    // └───────────────────────────────────────────────────────────────┘
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    // ... (他の初期化処理)
}
```

### 6.2 オドメトリ因子の追加

```cpp
// ファイル: src/mapOptmization.cpp (行1381-1395)

void addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // ケース1: 最初のキーフレーム
        // ┌───────────────────────────────────────────────────────────┐
        // │ 最初のキーフレームには PriorFactor を追加                  │
        // │                                                           │
        // │ ノイズモデル:                                              │
        // │   [1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8]                   │
        // │    ↑回転は緩め  ↑位置は非常に緩め                         │
        // │                                                           │
        // │ なぜ位置の不確かさが大きい？                               │
        // │   → 初期位置は任意でよい（相対座標系なので）              │
        // │   → 回転は重力方向から推定可能                            │
        // └───────────────────────────────────────────────────────────┘
        noiseModel::Diagonal::shared_ptr priorNoise =
            noiseModel::Diagonal::Variances(
                (Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished());

        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else
    {
        // ケース2: 2番目以降のキーフレーム
        // ┌───────────────────────────────────────────────────────────┐
        // │ 連続するキーフレーム間に BetweenFactor を追加              │
        // │                                                           │
        // │ BetweenFactor(from, to, relative_pose, noise)             │
        // │   from: 前のキーフレームのインデックス                     │
        // │   to: 現在のキーフレームのインデックス                     │
        // │   relative_pose: 2つの間の相対変換                        │
        // │   noise: この測定の不確かさ                                │
        // │                                                           │
        // │ poseFrom.between(poseTo) について:                        │
        // │   世界座標系での2つの姿勢から、相対変換を計算              │
        // │   結果: poseFrom から poseTo への変換行列                  │
        // └───────────────────────────────────────────────────────────┘
        noiseModel::Diagonal::shared_ptr odometryNoise =
            noiseModel::Diagonal::Variances(
                (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);

        gtSAMgraph.add(BetweenFactor<Pose3>(
            cloudKeyPoses3D->size()-1,   // from
            cloudKeyPoses3D->size(),     // to
            poseFrom.between(poseTo),    // 相対変換
            odometryNoise));

        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}
```

グラフの成長イメージ：

```
キーフレーム追加ごとのグラフの変化:

Frame 0:
    [Prior]
       │
       ▼
    ┌─────┐
    │ X₀  │
    └─────┘

Frame 1:
    [Prior]
       │
       ▼
    ┌─────┐  [Between]  ┌─────┐
    │ X₀  │────────────→│ X₁  │
    └─────┘             └─────┘

Frame 2:
    [Prior]
       │
       ▼
    ┌─────┐  [Between]  ┌─────┐  [Between]  ┌─────┐
    │ X₀  │────────────→│ X₁  │────────────→│ X₂  │
    └─────┘             └─────┘             └─────┘
```

### 6.3 GPS因子の追加

```cpp
// ファイル: src/mapOptmization.cpp (行1397-1475)

void addGPSFactor()
{
    if (gpsQueue.empty())
        return;

    // システムが安定するまで待つ
    if (cloudKeyPoses3D->points.empty())
        return;

    // 移動距離が5m未満なら追加しない（初期化の問題を避ける）
    if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
        return;

    // 姿勢の不確かさが小さければGPS不要
    // ┌───────────────────────────────────────────────────────────────┐
    // │ poseCovariance(3,3) と (4,4) は x, y 位置の分散               │
    // │ これが小さい = 現在の推定が十分確か = GPS補正不要             │
    // └───────────────────────────────────────────────────────────────┘
    if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        return;

    while (!gpsQueue.empty())
    {
        // タイムスタンプが近いGPS測定を探す
        if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
        {
            gpsQueue.pop_front();  // 古すぎる
        }
        else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
        {
            break;  // 新しすぎる
        }
        else
        {
            nav_msgs::Odometry thisGPS = gpsQueue.front();
            gpsQueue.pop_front();

            // GPSノイズのチェック
            float noise_x = thisGPS.pose.covariance[0];
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];

            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                continue;  // ノイズが大きすぎる

            // GPS因子を追加
            // ┌───────────────────────────────────────────────────────────┐
            // │ GPSFactor(index, position, noise)                         │
            // │                                                           │
            // │ 意味: 「キーフレーム index の位置は position である」     │
            // │       ただし noise だけの不確かさがある                   │
            // │                                                           │
            // │ 注意: GPSFactor は位置のみ（回転は制約しない）            │
            // └───────────────────────────────────────────────────────────┘
            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            noiseModel::Diagonal::shared_ptr gps_noise =
                noiseModel::Diagonal::Variances(Vector3);

            gtsam::GPSFactor gps_factor(
                cloudKeyPoses3D->size(),
                gtsam::Point3(gps_x, gps_y, gps_z),
                gps_noise);
            gtSAMgraph.add(gps_factor);

            aLoopIsClosed = true;  // GPS追加時も軌跡を再計算
            break;
        }
    }
}
```

### 6.4 ループクロージング因子の追加

```cpp
// ファイル: src/mapOptmization.cpp (行529-608)

void performLoopClosure()
{
    // ... (ループ候補の検出処理) ...

    // ICPでマッチング
    // ┌───────────────────────────────────────────────────────────────┐
    // │ ICP (Iterative Closest Point) アルゴリズム                    │
    // │                                                               │
    // │ 1. 現在のスキャンと、過去のスキャンを比較                     │
    // │ 2. 点群間の対応付けを反復的に改善                             │
    // │ 3. 2つのスキャン間の相対変換を推定                            │
    // │ 4. フィッティングスコアで品質を評価                           │
    // └───────────────────────────────────────────────────────────────┘
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // 収束チェック
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    // ループ制約の計算
    // ┌───────────────────────────────────────────────────────────────┐
    // │ ループ制約の作成手順:                                         │
    // │                                                               │
    // │ 1. ICP結果から補正行列を取得                                  │
    // │    correctionLidarFrame = icp.getFinalTransformation()        │
    // │                                                               │
    // │ 2. 現在の（誤った）姿勢を取得                                 │
    // │    tWrong = 現在キーフレームの姿勢                            │
    // │                                                               │
    // │ 3. 補正後の姿勢を計算                                         │
    // │    tCorrect = correctionLidarFrame * tWrong                   │
    // │                                                               │
    // │ 4. 相対変換を計算                                             │
    // │    poseBetween = poseFrom.between(poseTo)                     │
    // └───────────────────────────────────────────────────────────────┘
    Eigen::Affine3f correctionLidarFrame = icp.getFinalTransformation();
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;

    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

    // ノイズモデル: ICPスコアに基づく
    float noiseScore = icp.getFitnessScore();
    gtsam::Vector Vector6(6);
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise =
        noiseModel::Diagonal::Variances(Vector6);

    // ループ制約をキューに追加
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
}

// ファイル: src/mapOptmization.cpp (行1477-1495)
void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    // キュー内のすべてのループ制約を追加
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];

        // ┌───────────────────────────────────────────────────────────┐
        // │ BetweenFactor(indexFrom, indexTo, poseBetween, noise)     │
        // │                                                           │
        // │ 意味: 「キーフレーム indexFrom と indexTo は、            │
        // │        poseBetween の関係にある」                         │
        // │                                                           │
        // │ これにより、グラフに「ショートカット」が追加される        │
        // │ → 長い経路での累積誤差を修正できる                       │
        // └───────────────────────────────────────────────────────────┘
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}
```

ループクロージングの効果：

```
ループクロージング前:

    X₀ ─── X₁ ─── X₂ ─── X₃ ─── X₄ ─── X₅
                                        │
    誤差が蓄積して、X₅の位置がずれている
    実際はX₅はX₀の近くにいるはず

ループクロージング後:

    X₀ ─── X₁ ─── X₂ ─── X₃ ─── X₄ ─── X₅
    │                                   │
    └─────── [LoopFactor] ──────────────┘

    ループ制約が追加されると、全体の姿勢が調整される
    X₁～X₄も少しずつ修正され、一貫性のある軌跡になる
```

### 6.5 最適化と結果の取得

```cpp
// ファイル: src/mapOptmization.cpp (行1497-1581)

void saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
        return;

    // 因子を追加
    addOdomFactor();
    addGPSFactor();
    addLoopFactor();

    // iSAM2で最適化
    // ┌───────────────────────────────────────────────────────────────┐
    // │ isam->update(graph, values)                                   │
    // │                                                               │
    // │ 1回目のupdate: 新しい因子と値を追加                           │
    // │ 2回目のupdate: 収束を改善（オプション）                       │
    // │                                                               │
    // │ ループクロージング時は5回追加で更新:                          │
    // │   大きな変更があるため、より多くの反復が必要                  │
    // └───────────────────────────────────────────────────────────────┘
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    // グラフをクリア（増分更新のため）
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // 最適化結果を取得
    // ┌───────────────────────────────────────────────────────────────┐
    // │ calculateEstimate(): 全変数の現在の推定値を取得               │
    // │                                                               │
    // │ isamCurrentEstimate.at<Pose3>(i):                             │
    // │   キーフレーム i の最適化された姿勢を取得                     │
    // └───────────────────────────────────────────────────────────────┘
    isamCurrentEstimate = isam->calculateEstimate();
    Pose3 latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

    // キーフレーム情報を保存
    PointType thisPose3D;
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size();
    cloudKeyPoses3D->push_back(thisPose3D);

    // 共分散を取得（姿勢の不確かさ）
    // ┌───────────────────────────────────────────────────────────────┐
    // │ marginalCovariance: 指定した変数の周辺共分散行列              │
    // │                                                               │
    // │ 対角成分: 各次元の分散（不確かさの2乗）                       │
    // │   (0,0), (1,1), (2,2): roll, pitch, yaw の分散               │
    // │   (3,3), (4,4), (5,5): x, y, z の分散                        │
    // │                                                               │
    // │ 用途: GPS因子を追加するかどうかの判断に使用                   │
    // └───────────────────────────────────────────────────────────────┘
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // ... (残りの保存処理)
}
```

---

## 7. iSAM2による効率的な最適化

### 7.1 なぜiSAM2を使うのか？

```
┌─────────────────────────────────────────────────────────────────────┐
│                   バッチ最適化 vs iSAM2                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  【バッチ最適化】                                                    │
│                                                                     │
│    新しいデータが来るたびに、全データを再計算                        │
│                                                                     │
│    時刻1: X₀を最適化                    計算量: O(1)                │
│    時刻2: X₀, X₁を最適化                計算量: O(4)                │
│    時刻3: X₀, X₁, X₂を最適化            計算量: O(9)                │
│    時刻4: X₀, X₁, X₂, X₃を最適化        計算量: O(16)               │
│    ...                                                              │
│    時刻N: X₀～X_{N-1}を最適化           計算量: O(N²)               │
│                                                                     │
│    → 時間とともに計算量が爆発的に増加！                             │
│                                                                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  【iSAM2 (Incremental Smoothing and Mapping 2)】                    │
│                                                                     │
│    変更された部分だけを再計算                                        │
│                                                                     │
│    時刻1: X₀を最適化                    計算量: O(1)                │
│    時刻2: X₁を追加、影響部分のみ更新    計算量: O(1)                │
│    時刻3: X₂を追加、影響部分のみ更新    計算量: O(1)                │
│    ...                                                              │
│    時刻N: X_{N-1}を追加                 計算量: O(1) (通常時)       │
│                                                                     │
│    ループクロージング時: 影響範囲を再計算  計算量: O(影響変数数)    │
│                                                                     │
│    → リアルタイム処理が可能！                                       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2 iSAM2の仕組み（簡略版）

```
iSAM2の内部構造: Bayes木

    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │                        [Root]                               │
    │                           │                                 │
    │              ┌───────────┼───────────┐                      │
    │              │           │           │                      │
    │           [Clique]   [Clique]   [Clique]                    │
    │           X₀,X₁      X₂,X₃      X₄,X₅                       │
    │              │           │           │                      │
    │           [Leaf]     [Leaf]     [Leaf]                      │
    │                                                             │
    │  Clique: 変数のグループ                                     │
    │  - 同じCliqueの変数は密に結合                               │
    │  - 異なるCliqueは疎に結合                                   │
    │                                                             │
    │  新しいデータ追加時:                                         │
    │  1. 影響を受けるCliqueを特定                                │
    │  2. そのCliqueとその祖先のみを再計算                        │
    │  3. 他のCliqueは変更なし                                    │
    │                                                             │
    └─────────────────────────────────────────────────────────────┘
```

### 7.3 LIO-SAMでのiSAM2パラメータ

```cpp
// iSAM2パラメータの意味

ISAM2Params parameters;

// relinearizeThreshold: 再線形化の閾値
// ┌───────────────────────────────────────────────────────────────┐
// │ 非線形最適化では、問題を線形化して解く。                       │
// │ 変数の値が変わると、線形化が不正確になる。                     │
// │                                                               │
// │ この閾値を超える変化があった変数は再線形化される。             │
// │   0.1 = 10%以上の変化で再線形化                               │
// │   小さいほど精度↑、計算量↑                                   │
// │   大きいほど精度↓、計算量↓                                   │
// └───────────────────────────────────────────────────────────────┘
parameters.relinearizeThreshold = 0.1;

// relinearizeSkip: 再線形化チェックの頻度
// ┌───────────────────────────────────────────────────────────────┐
// │ 何回のupdate()ごとに再線形化をチェックするか                  │
// │   1 = 毎回チェック（最も精度が高い）                          │
// │   n = n回に1回チェック（計算量削減）                          │
// └───────────────────────────────────────────────────────────────┘
parameters.relinearizeSkip = 1;
```

---

## 8. まとめ

### 8.1 LIO-SAMにおけるGTSAM因子グラフのポイント

```
┌─────────────────────────────────────────────────────────────────────┐
│                    まとめ: 因子グラフの役割                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. センサー融合のフレームワーク                                     │
│     - IMU、LiDAR、GPSの情報を統一的に扱う                           │
│     - 各センサーの不確かさを適切に考慮                               │
│                                                                     │
│  2. 使用される因子タイプ                                             │
│     ┌─────────────────┬──────────────────────────────────────┐      │
│     │ 因子タイプ      │ 用途                                 │      │
│     ├─────────────────┼──────────────────────────────────────┤      │
│     │ PriorFactor     │ 初期姿勢、LiDAR補正、GPS観測         │      │
│     │ BetweenFactor   │ オドメトリ、ループクロージング       │      │
│     │ ImuFactor       │ IMU前積分                           │      │
│     │ GPSFactor       │ GPS位置観測                          │      │
│     └─────────────────┴──────────────────────────────────────┘      │
│                                                                     │
│  3. 最適化手法                                                       │
│     - iSAM2による増分最適化                                          │
│     - リアルタイム処理を実現                                         │
│     - ループクロージング時に過去の姿勢も修正                         │
│                                                                     │
│  4. 2つの因子グラフ                                                  │
│     - IMU前積分用: 高周波の姿勢推定                                  │
│     - マップ最適化用: キーフレーム間の最適化                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.2 因子グラフ最適化の流れ

```
┌─────────────────────────────────────────────────────────────────────┐
│                  因子グラフ最適化の全体フロー                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ステップ1: グラフの構築                                             │
│      ┌─────────────────────────────────────┐                        │
│      │ // 因子グラフを作成                 │                        │
│      │ NonlinearFactorGraph graph;         │                        │
│      │ Values initial;                     │                        │
│      │                                     │                        │
│      │ // 因子を追加                       │                        │
│      │ graph.add(PriorFactor(...));        │                        │
│      │ graph.add(BetweenFactor(...));      │                        │
│      │                                     │                        │
│      │ // 初期値を設定                     │                        │
│      │ initial.insert(X(0), pose0);        │                        │
│      │ initial.insert(X(1), pose1);        │                        │
│      └─────────────────────────────────────┘                        │
│                          │                                          │
│                          ▼                                          │
│  ステップ2: 最適化の実行                                             │
│      ┌─────────────────────────────────────┐                        │
│      │ // iSAM2で最適化                    │                        │
│      │ isam->update(graph, initial);       │                        │
│      │                                     │                        │
│      │ // 内部で非線形最適化が実行される   │                        │
│      │ // - 線形化                         │                        │
│      │ // - 線形システムを解く             │                        │
│      │ // - 更新を適用                     │                        │
│      └─────────────────────────────────────┘                        │
│                          │                                          │
│                          ▼                                          │
│  ステップ3: 結果の取得                                               │
│      ┌─────────────────────────────────────┐                        │
│      │ // 最適化された値を取得             │                        │
│      │ Values result = isam->calculate();  │                        │
│      │ Pose3 pose = result.at<Pose3>(X(0));│                        │
│      │                                     │                        │
│      │ // 共分散（不確かさ）を取得         │                        │
│      │ Matrix cov = isam->marginalCov(X(0));│                       │
│      └─────────────────────────────────────┘                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.3 さらに学ぶために

1. **GTSAM公式ドキュメント**: https://gtsam.org/
2. **GTSAM Tutorial**: GTSAMリポジトリ内のexamplesフォルダ
3. **LIO-SAM論文**: "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping"
4. **因子グラフの教科書**: "Factor Graphs for Robot Perception" by Dellaert & Kaess

---

## 付録: よくある質問

### Q1: なぜIMU前積分とマップ最適化で別々の因子グラフを使うのですか？

**A**: 更新頻度と目的が異なるためです。
- IMU前積分: 200Hzで更新、短期間の姿勢追跡が目的
- マップ最適化: 10Hzで更新、長期間の一貫性維持が目的

### Q2: ノイズモデルの値はどう決めればいいですか？

**A**: センサーの仕様書や実験から決定します。
- メーカー提供のノイズ仕様
- Allan分散解析（IMUの場合）
- 実際の走行データでの調整

### Q3: ループクロージング時に5回updateを呼ぶのはなぜですか？

**A**: ループクロージングは大きな変更を引き起こすため、追加の反復で収束を確実にします。通常の更新では1-2回で十分ですが、大きな修正が必要な場合は追加の反復が有効です。

### Q4: 計算量が心配です。リアルタイムで動きますか？

**A**: iSAM2を使用しているため、通常の操作はリアルタイムで動作します。ただし、大規模なループクロージングや、非常に長い走行では計算時間が増加する可能性があります。
