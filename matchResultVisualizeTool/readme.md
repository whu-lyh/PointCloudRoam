## matchResultVisualizeTool
这个仓库是专门为了展示两两匹配结果的，但是不仅限于两两匹配结果，凡是有匹配结果的，按照要求写好路径都是可以展示的，就是可能会有点卡顿。

## 依赖项
osg: 主要的渲染引擎

boost：文件读写和路径管理

glog：日志文件记录

yaml-cpp：配置文件读写

liblas: 点云读写

pcl: 基础点云格式

## 使用方法
```C++
./matchResultVisualizeTool.exe path/to/configuration.yaml
```
配置详见yaml文件中的注释。