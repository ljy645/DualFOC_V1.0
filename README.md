# F405 FOC 双轴控制工程

- [F405_FOC_Ctrl](F405_FOC_Ctrl/)：STM32F405 固件与 Keil 工程。
- [FOC_Host](FOC_Host/)：Python / PySide6 上位机，包含 USB 串口控制、参数配置、校准和遥测显示。

固件使用 Keil 打开 `F405_FOC_Ctrl/MDK-ARM/F405_FOC_Ctrl.uvprojx` 编译。
上位机需要 Python 3.10 或更新版本，Windows 下双击 `FOC_Host/start.bat` 启动；使用 `start.bat --demo` 运行离线演示。依赖与详细用法见各目录说明。

上位机运行时自动生成的历史采集数据、个人串口设置和 Python 缓存不纳入版本管理。

第三方代码保留各自许可证。自有代码的开源许可证待作者选定。