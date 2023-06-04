# emcl2: mcl with expansion resetting (version 2)

emcl2をROS2に移植しました。
nav2_amclを置換するため、インターフェース周りで仕様を変更しています。

## 変更箇所

- mapトピックによる地図受信を追加しました。

- LifecycleNode化しました。

- Componentに対応しました。

### Subscribed Topic

- map (nav_msgs/msg/OccupancyGrid)

    use_map_topicパラメータがtrueに設定された場合、このトピックを受信して尤度場地図を生成します。

### Parameter

- use_map_topic (bool, default: true)

    trueの場合、mapトピックが有効になり、static_mapサービス呼び出しは行われません。

## 参照

[ROS版README](docs/ROS_README.md)<br>
[launchサンプル](docs/launch.md)<br>
[emcl](https://github.com/yo-fuji/emcl)