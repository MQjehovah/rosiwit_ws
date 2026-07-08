#!/bin/bash
# ============================================================
# install_deps.sh — 安装 rosiwit_ws 所有编译依赖
# 包括: apt 包、PCL 符号链接修复、OpenNI 桩库
#
# 用法: sudo ./install_deps.sh
# ============================================================
set -e

echo "=============================================="
echo "  rosiwit_ws — 依赖安装脚本"
echo "=============================================="

# ==================== APT 包 ====================
echo ""
echo "=== [1/4] 安装 ROS2 依赖包 ==="
apt-get update -qq
apt-get install -y -qq \
    ros-humble-sophus \
    ros-humble-nav2-core \
    ros-humble-nav2-util \
    ros-humble-nav2-costmap-2d \
    libpcap0.8 \
    2>&1 | tail -5

# 检查是否缺少 PCL octree 运行时库
echo ""
echo "=== [2/4] 检查 PCL 库完整性 ==="
if ! ldconfig -p | grep -q libpcl_octree; then
    echo "  libpcl_octree not found, installing..."
    apt-get install -y -qq libpcl-octree1.12 2>&1 | tail -3
fi

# ==================== PCL 符号链接修复 ====================
echo ""
echo "=== [3/4] 修复 PCL 1.12 -> 1.12.1 符号链接 ==="
FIXED=0
for lib in apps keypoints ml octree outofcore people recognition stereo tracking; do
  target="/usr/lib/x86_64-linux-gnu/libpcl_${lib}.so.1.12"
  real="/usr/lib/x86_64-linux-gnu/libpcl_${lib}.so.1.12.1"
  if [ ! -f "$target" ] && [ -f "$real" ]; then
    ln -sf "libpcl_${lib}.so.1.12.1" "$target"
    echo "  Fixed: $target -> libpcl_${lib}.so.1.12.1"
    ((FIXED++))
  fi
done
[ $FIXED -eq 0 ] && echo "  All symlinks OK."

# ==================== OpenNI 桩库 ====================
echo ""
echo "=== [4/4] 生成 OpenNI 桩库 (libOpenNI.so.0 / libOpenNI2.so.0) ==="
# libpcl_io.so 引用了 OpenNI/PrimeSense 符号 (oni*, xn*)。
# 这些是 Kinect/PrimeSense 深度相机驱动，不在本平台使用。
# 创建空桩函数满足链接即可。

cat > /tmp/fake_openni.c << 'STUBEOF'
int oniDeviceClose() { return 0; }
int oniDeviceCreateStream() { return 0; }
int oniDeviceDisableDepthColorSync() { return 0; }
int oniDeviceEnableDepthColorSync() { return 0; }
int oniDeviceGetDepthColorSyncEnabled() { return 0; }
int oniDeviceGetInfo() { return 0; }
int oniDeviceGetProperty() { return 0; }
int oniDeviceGetSensorInfo() { return 0; }
int oniDeviceIsCommandSupported() { return 0; }
int oniDeviceIsImageRegistrationModeSupported() { return 0; }
int oniDeviceIsPropertySupported() { return 0; }
int oniDeviceOpen() { return 0; }
int oniDeviceSetProperty() { return 0; }
int oniFrameAddRef() { return 0; }
int oniFrameRelease() { return 0; }
int oniGetDeviceList() { return 0; }
int oniGetExtendedError() { return 0; }
int oniInitialize() { return 0; }
int oniRegisterDeviceCallbacks() { return 0; }
int oniReleaseDeviceList() { return 0; }
int oniStreamDestroy() { return 0; }
int oniStreamGetProperty() { return 0; }
int oniStreamGetSensorInfo() { return 0; }
int oniStreamIsPropertySupported() { return 0; }
int oniStreamReadFrame() { return 0; }
int oniStreamRegisterNewFrameCallback() { return 0; }
int oniStreamSetProperty() { return 0; }
int oniStreamStart() { return 0; }
int oniStreamStop() { return 0; }
int oniUnregisterDeviceCallbacks() { return 0; }
int xnCanFrameSyncWith() { return 0; }
int xnContextAddRef() { return 0; }
int xnContextOpenFileRecordingEx() { return 0; }
int xnContextRegisterForShutdown() { return 0; }
int xnContextRelease() { return 0; }
int xnContextUnregisterFromShutdown() { return 0; }
int xnCopyDepthMetaData() { return 0; }
int xnCopyIRMetaData() { return 0; }
int xnCopyImageMetaData() { return 0; }
int xnCreateProductionTree() { return 0; }
int xnEnumerateProductionTrees() { return 0; }
int xnFindExistingRefNodeByType() { return 0; }
int xnForceShutdown() { return 0; }
int xnFrameSyncWith() { return 0; }
int xnGetCropping() { return 0; }
int xnGetDepthMetaData() { return 0; }
int xnGetIRMetaData() { return 0; }
int xnGetImageMetaData() { return 0; }
int xnGetIntProperty() { return 0; }
int xnGetMapOutputMode() { return 0; }
int xnGetNodeInfo() { return 0; }
int xnGetNodeName() { return 0; }
int xnGetRealProperty() { return 0; }
int xnGetRefContextFromNodeHandle() { return 0; }
int xnGetStatusString() { return 0; }
int xnInit() { return 0; }
int xnIsCapabilitySupported() { return 0; }
int xnIsFrameSyncedWith() { return 0; }
int xnIsGenerating() { return 0; }
int xnIsPlayerAtEOF() { return 0; }
int xnIsViewPointAs() { return 0; }
int xnIsViewPointSupported() { return 0; }
int xnNodeInfoGetCreationInfo() { return 0; }
int xnNodeInfoGetDescription() { return 0; }
int xnNodeInfoGetInstanceName() { return 0; }
int xnNodeInfoGetNeededNodes() { return 0; }
int xnNodeInfoGetRefHandle() { return 0; }
int xnNodeInfoListAllocate() { return 0; }
int xnNodeInfoListFree() { return 0; }
int xnNodeInfoListGetCurrent() { return 0; }
int xnNodeInfoListGetFirst() { return 0; }
int xnNodeInfoListGetNext() { return 0; }
int xnNodeInfoListIteratorIsValid() { return 0; }
int xnNodeInfoSetInstanceName() { return 0; }
int xnOSFreeAligned() { return 0; }
int xnOSMallocAligned() { return 0; }
int xnOSMemCopy() { return 0; }
int xnOSMemSet() { return 0; }
int xnPlayerReadNext() { return 0; }
int xnProductionNodeAddRef() { return 0; }
int xnProductionNodeRelease() { return 0; }
int xnRegisterToNewDataAvailable() { return 0; }
int xnResetViewPoint() { return 0; }
int xnSeekPlayerToFrame() { return 0; }
int xnSetCropping() { return 0; }
int xnSetIntProperty() { return 0; }
int xnSetMapOutputMode() { return 0; }
int xnSetPixelFormat() { return 0; }
int xnSetPlayerRepeat() { return 0; }
int xnSetViewPoint() { return 0; }
int xnStartGenerating() { return 0; }
int xnStopFrameSyncWith() { return 0; }
int xnStopGenerating() { return 0; }
int xnStopGeneratingAll() { return 0; }
int xnUnregisterFromNewDataAvailable() { return 0; }
int xnWaitAndUpdateData() { return 0; }
STUBEOF

gcc -shared -fPIC -o /usr/lib/x86_64-linux-gnu/libOpenNI2.so.0 /tmp/fake_openni.c
gcc -shared -fPIC -o /usr/lib/x86_64-linux-gnu/libOpenNI.so.0 /tmp/fake_openni.c
rm /tmp/fake_openni.c
echo "  libOpenNI.so.0  — created"
echo "  libOpenNI2.so.0 — created"

ldconfig
echo ""
echo "=============================================="
echo "  全部依赖安装完成。现在可以编译:"
echo ""
echo "  source /opt/ros/humble/setup.bash"
echo "  cd /root/rosiwit_ws"
echo "  colcon build --symlink-install \\"
echo "    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
echo "=============================================="
