# CAN Python SDK

提供与当前解释器匹配的 `xarm_can` 扩展，支持 Linux、CPython 3.10、x86_64 / aarch64。
从 SDK 根目录、在目标 Python 环境内执行：

```bash
python3 -m pip install ./publish/xarm_can/python
python3 -c 'import xarm_can; print(xarm_can.__file__); print(xarm_can.XArm)'
```

安装过程生成匹配 CPU/Python 的二进制 wheel，不依赖开发机的 editable 安装路径，
不访问 CAN、不使能电机。Python 3.11/3.12 等版本不能使用 CPython 3.10 扩展。
ARM64 产物来自现有交付包，需在对应设备独立验收；不能将 x86_64 导入通过视为 ARM64 已验证。

这与 `libxarm_can_sdk.so` 是两种调用方式：本包供 `import xarm_can`，C ABI 动态库供 C/C++ 或 ctypes。
制品来源与哈希见 ARTIFACTS.json；重建工具属于内部开发流程，不随客户包发布。

客户 Git 仓库可能不含预编译大文件；安装时请使用完整 SDK Release 附件，不能只下载 GitHub 自动生成的 Source code.zip。
