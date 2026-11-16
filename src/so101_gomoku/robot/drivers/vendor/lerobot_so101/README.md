# Vendor 说明

本目录两个 Python 文件是上游 SO101 驱动原样快照，保留版权头；相对导入依赖完整 LeRobot 包，不能独立运行或作为本项目运行驱动导入。

实际使用 ../lerobot_driver.py 调用环境中已安装的 LeRobot。上游完整配套 motors、cameras、基类等仍在根目录 references/lerobot-llm-chess/src/lerobot/，不重复散装复制依赖树。
