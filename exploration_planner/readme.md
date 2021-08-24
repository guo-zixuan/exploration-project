# exploration_package
---
## frontier update
---
### unknown points update

输入：
1 robot position
2 octomap
3 gird map

接入 grid_map_utils

取出unknown points            finished

1、检测是不是前沿点；
2、检测有没有在检测范围内

potential frontier 是 相邻 free 栅格的 frontier
取出 potential frontier 

对 potential frontier 栅格点 进行 直线检测 碰撞层检测 视线检测
取出 local frontier 



检测 unknown点
检测 frontier


