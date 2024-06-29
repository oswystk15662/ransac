# colcon buildを通すための依存関係設定

## CMakeLists
2行目　project()はパッケージ名をいれる

find_packageにはrclcppを絶対入れ、他必要なものを追加していく。
今回picoscan、tim571を前提としてるので、入れといたけど、本当はいらない

add_executable(ransac_osw_node src/ransac_osw.cpp)
で、ros2 run パッケージ名　ノード名　のノード名を定義できる

ament_target_dependencies(ransac_osw_node rclcpp sensor_msgs sick_scan_xd)
ノードのdependenciesを追加できる。

installはまだよくわかってないが、install/setup.bashするときの、install dirに対する操作の定義。
launchするときは、おまじないを欠かさずに

## package.xml
<name>はパッケージ名
<depend>にCMakeListsにいれた依存関係を入れていく。
<depend>は<build_depend>と<exec_depend>を合わせたもの。そのため、pythonの場合はすべて<exec_depend>。
ros2 launch ...で起動するなら、<exec_depend>ros2launch</exec_depend>が必要

上記を一致させておかないとだるそう