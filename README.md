# Carrot STM32F4 project
全自動走行台車のmotor制御用マイコンSTM32F446用のプログラムです。
[Carrotの動画はこちら](https://www.youtube.com/watch?v=5gtot_12dCg&ab_channel=DDProjectII)


# Features

シリアル通信で受け取ったデータをもとにモーターを動かすプログラムです。

メカナムホイールを使用し全方向どの方向にも移動することができ、さらに超信地旋回をすることができます。

また、ロータリーエンコーダー、9軸センサーを使用することにより正確な動作が可能です。

さらに、ブザーや、テープLEDによって周囲の人への警告もできるほか、非常停止ボタンの検知もできます。

未実装の機能として、LCD制御用マイコンとRS485やCAN通信を使って通信することができます。


# Requirement

* STM CubeIDE
* FreeRTOS

回路データは[こちら](https://github.com/rakuseirobot/Carrot-boards)

# Usage

シリアル通信で以下のプロトコルを送信してください。

[0xFF][type][data_high][data_low]

type 1. X速度 (data)

type 2. Y速度 (data)

type 3. Z角速度 (data)

type 4. ブレーキ (1.true 0.false)

dataは0~20000です 内部で-10000されます。

# Note

LiDARを制御しているのRaspberryPiとシリアル通信をすることでロボットを動かしています。

メインCPUのプログラムは[こちら](https://github.com/rakuseirobot/Carrot-ROS-src)

# Author

* Shun Kayaki
* Kyushu institute of Technology
* shun@guetan.dev

# License

Copyright (c) 2019 Shun Kayaki
Released under the MIT license
