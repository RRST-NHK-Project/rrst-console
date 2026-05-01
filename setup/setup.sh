
# 依存関係の一括インストール
# ＊実行方法＊
# cd ~/ros2_ws/src/setup
# sudo chmod +x setup.sh
# ./setup.sh

# いつもの
sudo apt-get update -y
sudo apt-get upgrade -y

# 以下GUI用
sudo apt install npm -y
sudo apt install ros-jazzy-rosbridge-server -y

# コンソールの依存関係をインストール
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}/../nr26_r2_console"
npm install

