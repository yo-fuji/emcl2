# emcl2: mcl with expansion resetting (version 2)

emcl2をROS2に移植しました。
nav2_amclを置換するため、インターフェース周りで仕様を変更しています。

## 変更箇所

- mapトピックによる地図受信を追加しました。

- LifecycleNode化しました。

- Componentに対応しました。

## 参照

[ROS版README](docs/ROS_README.md)
[emcl](https://github.com/yo-fuji/emcl)