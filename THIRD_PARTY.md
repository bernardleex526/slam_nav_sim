# 第三方代码与数据

`third_party/breezyslam/` 含 [BreezySLAM](https://github.com/simondlevy/BreezySLAM) 的 C 核心、Python 扩展、Python 接口与 `examples/exp2.dat`，来源提交 `1170285cd42a88300eddd275fc3df36e642e3a2e`。保留上游版权声明，随附 GPLv3／LGPLv3 文本。源代码未作功能修改，编译产物不随交付包分发，用户可以更换或重新编译该库。公开日志按上游仓库随附分发，未发现独立的数据许可声明；对外再分发数据前应确认其使用条件。

本项目使用 Python 包 numpy、scipy、Pillow、PyYAML、rosbags、evo、pytest；安装时由各包携带原许可证。本地解析后的版本记录在 `docs/python-local-lock.txt`。基础依赖约束在 `pyproject.toml`，不同 Python／平台会解析不同传递依赖。

ROS 部署使用 ROS 2、rosbag2、ros_gz、Gazebo、Nav2 与 SLAM Toolbox 的已安装包，不随源码仓库复制这些项目。生成 Nav2 参数时读取目标环境安装包的默认参数，因此每次运行保存有效参数和软件版本。

没有把 BenchBot、bagx、nav_explore_eval、MapEval、NavigationAnalyzer 或 ROSNavBench 的实现复制进本项目，也没有声称已验证其兼容性。
