# ssv102_ros

Hemisphere 社の ssV-102 という GPS ユニット用の ROS パッケージ．

# parameters

## device

GPS をつないでいるポート名．
デフォルト : ```/dev/ttyUSB0```

## baud

GPS をつないでいるポートの通信速度．
デフォルト : ```115200```

## frame_id

各種トピックの header にはいる frame_id ．
デフォルト : ```ssV102```

# publish

## fix

sensor_msgs::NavSatFix をGPGGAセンテンスを受信したタイミングで発行．

## time

sensor_msgs::TimeRef をGPGGAセンテンスを受信したタイミングで発行．

TODO : 今は受信したUTC時刻文字列をそのまま出力しているため，ros::Time とは一切整合しない．
GPZDA メッセージも読むようにして，Unixタイムを計算するべきか．

## pose

geometry_msgs::PoseStamped を PSAT,HPR センテンスを受信したタイミングで発行．
yaw 角は磁北を0度とし，上から見て半時計回りが正となる．

# subscribe

なし