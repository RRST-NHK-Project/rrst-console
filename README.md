# rrst-console

RRST 向け Web コンソール群をまとめたディレクトリです。

## ディレクトリ構成

- `nr26_r1_console/`: NR26 R1 用コンソール（React + roslib）
- `nr26_r2_console/`: NR26 R2 用コンソール（React + roslib）
- `rrst_generic_console/`: r / theta / z / hand 機向け汎用コンソール（React + roslib）
- `setup/`: 初期セットアップ用スクリプト

## 前提条件

- Ubuntu + ROS 2 Jazzy 環境
- Node.js / npm
- `ros2` コマンドが実行可能であること
- `rosbridge_server` がインストール済みであること

`rosbridge_server` が未導入の場合:

```bash
sudo apt update
sudo apt install -y ros-jazzy-rosbridge-server
```

## 初期セットアップ（任意）

`setup/setup.sh` で apt パッケージ更新と GUI 関連パッケージの導入ができます。

```bash
cd ~/ros2_ws/src/rrst-console/setup
chmod +x setup.sh
./setup.sh
```

## R1 コンソール起動

```bash
cd ~/ros2_ws/src/rrst-console/nr26_r1_console
./start.sh
```

起動後:

- ROS bridge: `ws://localhost:9090`
- Console backend: `http://localhost:3031`
- Frontend: `http://localhost:3000`

## R2 コンソール起動

```bash
cd ~/ros2_ws/src/rrst-console/nr26_r2_console
./start.sh
```

起動後:

- ROS bridge: `ws://localhost:9090`
- Console backend: `http://localhost:3031`
- Frontend: `http://localhost:3000`

## Generic Arm Console 起動

```bash
cd ~/ros2_ws/src/rrst-console/rrst_generic_console
./start.sh
```

起動後:

- ROS bridge: `ws://localhost:9090`
- Console backend: `http://localhost:3031`
- Frontend: `http://localhost:3000`

## ポート変更

環境変数で変更できます。

```bash
BRIDGE_PORT=9091 CONSOLE_BACKEND_PORT=3032 ./start.sh
```

## Docker 実行（各コンソール配下）

各コンソールには Docker 実行用スクリプトがあります。

```bash
cd ~/ros2_ws/src/rrst-console/nr26_r1_console
./start_docker.sh
```

```bash
cd ~/ros2_ws/src/rrst-console/nr26_r2_console
./start_docker.sh
```

```bash
cd ~/ros2_ws/src/rrst-console/rrst_generic_console
./start_docker.sh
```

## よくある確認ポイント

- `ros2` が見つからない: ROS 2 の環境を source してから実行する
- `node` が見つからない: Node.js をインストールする
- 起動失敗時のログ:
  - R1: `/tmp/r1_console_rosbridge.log`, `/tmp/r1_console_backend.log`
  - R2: `/tmp/r2_console_rosbridge.log`, `/tmp/r2_console_backend.log`
  - Generic: `/tmp/rrst_generic_console_rosbridge.log`, `/tmp/rrst_generic_console_backend.log`
