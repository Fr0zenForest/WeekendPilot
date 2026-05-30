# FlightGear 3D 动画（演示用）

1. 启动 FlightGear，等待 socket 输入（native-FDM，端口 5550）：
   fgfs --fdm=null --native-fdm=socket,in,30,,5550,udp --aircraft=c172p --disable-ai-traffic

2. 在 run_sitl.py 里启用 FlightGear 输出：JSBSim 加载
   C:/Repository/jsbsim/data_output/flightgear.xml 作为 output（set_output_directive），
   IP=127.0.0.1 port=5550 rate=30，每步调用 fdm.enable_output()。
   JSBSim Python API 已验证存在 set_output_directive / enable_output。

3. 调参时不启动 FlightGear（无头跑只读 CSV），演示时再开。
