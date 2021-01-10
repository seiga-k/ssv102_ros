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

sensor_msgs::TimeRef をGPZDAセンテンスを受信したタイミングで発行．

## pose

geometry_msgs::PoseStamped を PSAT,HPR センテンスを受信したタイミングで発行．
yaw 角は東を0度とし，上から見て半時計回りが正となる．

# subscribe

なし