无人船路径规划项目


项目概述

本项目聚焦于无人船路径规划技术，基于栅格建模法构建全局环境地图，旨在为无人船在复杂水域环境中提供高效、安全的路径规划解决方案。


技术栈

开发环境：Unity

算法核心：栅格建模法、路径规划算法(A*)


项目结构

yunqi--Path-Planning-for-Unmanned-Vessels/

├── .vs/My project/v16/          # VS项目配置文件

├── Assets/                      # Unity资源目录（包含场景、脚本、模型等）

├── Packages/                    # Unity包管理目录

├── ProjectSettings/             # Unity项目设置

├── .gitignore                   # Git忽略规则配置

├── Assembly-CSharp.csproj       # C#项目文件
├── My project.sln               # VS解决方案文件
└── README.md                    # 项目自述文件（本文）


快速开始

确保安装 Unity Hub 及对应版本的 Unity 编辑器（可通过 Unity Hub 下载适配项目的版本）。

克隆本仓库：git clone https://github.com/Qi-Yu-git/yunqi--Path-Planning-for-Unmanned-Vessels.git

打开 Unity Hub，选择 “打开项目”，导入本仓库目录即可开始开发。


功能说明

栅格环境建模：将水域环境抽象为栅格地图，支持障碍物、可行域的可视化与数字化表达。

路径规划演示：可在 Unity 场景中模拟无人船基于栅格地图的路径规划过程，直观展示算法效果。


贡献指南

欢迎通过 Pull Request 提交优化建议、算法改进或功能扩展。若发现问题，可在 Issues 板块提交反馈。


[环境搭建文档.docx](https://github.com/user-attachments/files/23282669/default.docx)

[在Anaconda中配置的库.txt](https://github.com/user-attachments/files/23282670/Anaconda.txt)
