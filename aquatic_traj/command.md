# 常用命令速查（aquatic_traj）

> 说明：按任务分类整理，命令行可直接复制使用。

## Git 操作

```bash
# 初始化仓库：把当前文件夹变成一个 git 仓库
git init

# 把当前目录下的所有修改和新文件加入“待提交名单”（. 代表当前目录）
git add .

# 把“待提交名单”里的文件正式保存下来，生成一个历史版本
# 注意：-m 后建议填写有意义的提交说明
git commit -m ""

# 强制把当前的主分支改名为 main
git branch -M main

# 关联远程仓库
# 示例：git remote add origin <your-repo-url>
git remote add origin https://github.com/YuyueZhou711/test.git

# 把本地 main 分支上传到 origin 的 main 分支
# 首次推送建议使用 -u 以设置上游分支
git push -u origin main
```

## 显卡启动状态检查

```bash
glxinfo | grep "OpenGL renderer"
```

## CSV 转 TUM

```bash
# 将 csv（纳秒时间戳）转换为 tum 格式
awk -F',' 'NR>1 {printf "%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", $1/1e9, $2, $3, $4, $5, $6, $7, $8}' okvis2-vio-final_trajectory.csv > 01_okvis_slam.txt
```

## 轨迹绘图与评估（evo）

```bash
# 真值 + okvis2 轨迹对比
# 输入：gt.tum + 04_okvis.txt
# 输出：误差统计 + 轨迹图
evo_ape tum ~/datasets/svin/aquatic/04_Loop1_with_board/groundtruth/gt.tum 04_okvis.txt -va --plot --plot_mode xyz

# 只绘制真值轨迹
evo_traj tum groundtruth/gt.tum --plot --plot_mode xyz
```

## 录制 / 转换轨迹包

```bash
# 录制轨迹包
ros2 bag record -o okvis_odometry --topics /silver2/odometry

# 轨迹包转 TUM
evo_traj bag2 okvis_odometry/ /silver2/odometry --save_as_tum
```

## 键盘控制机器人

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## OKVIS 项目构建

```bash
colcon build --cmake-args -DBUILD_ROS2=ON -DUSE_NN=OFF -DBUILD_REALSENSE=OFF --symlink-install
```

## Topic 图像查看

```bash
ros2 run image_view image_view --ros-args -r image:=/silver2/stereo_right/image_raw/image_color
```
