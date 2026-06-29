# team_usb_integrated

统一后的整合工程，目标是把三份控制逻辑挂到同一套 USB 控制包上。

## 当前结构
- `src/usb_control.c`: 公共 USB 收包层
- `src/module_r1_catch.c`: 原 `R1_catch` 夹取逻辑，已从 SBUS 改为 USB 按钮触发
- `src/module_lift.c`: 原 `hhy` 升降 DM 电机逻辑
- `src/module_sbus2.c`: 原 `sbus_2` 的 M3508/M2006/继电器逻辑
- `src/main.c`: 主循环，统一分发控制包

## 统一协议
- `SYNC ID = 0x0303`
- Payload = 7 个 `float`
- `[0]` 按钮 `0..15`
- `[1]` 按钮 `16..31`
- `[2]` 按钮 `32..47`
- `[3]` `lx`
- `[4]` `ly`
- `[5]` `rx`
- `[6]` `ry`

## 按钮映射
### lift 模块
- `A(3)`: 降一挡
- `Y(6)`: 升一挡

### sbus_2 模块
- `32`: M3508 left extend
- `33`: M3508 center extend
- `34`: M3508 right extend
- `35`: M2006 left down
- `36`: M2006 right down
- `37`: relay left
- `38`: relay center
- `39`: relay right

### R1 catch 模块
- `40`: prepare
- `41`: raise
- `42`: attack
- `46`: release
- `47`: seize

## 构建
```bash
/home/cqj/zephyr-env/bin/west build -b robomaster_board_c \
  /home/cqj/my-workspace/mambo/rc_jiangyin/usb_init/team_usb_integrated \
  -d /home/cqj/my-workspace/mambo/rc_jiangyin/usb_init/team_usb_integrated/build -p always
```

## 重要说明
这个版本已经可以编译通过，但 `robomaster_board_c.overlay` 目前是从多份样例直接合并出来的。
如果三套机构最终真的要同时上同一块板，必须继续核对：
- CAN 电机 ID 是否冲突
- 同一总线上的收发配置是否冲突
- GPIO/继电器引脚是否与真实接线一致

换句话说：
现在这个工程已经完成了“代码入口统一”和“USB 协议统一”，但硬件资源表还需要你们团队按实物再做一轮校表。
