import os, time
os.environ["DEPTHAI_RECONNECT_TIMEOUT"] = "0"
import depthai as dai

with dai.Device() as device:
    print("name", device.getDeviceName())
    print("speed", device.getUsbSpeed())
    print("cams", [
        (str(c.socket), c.sensorName, c.width, c.height)
        for c in device.getConnectedCameraFeatures()
    ])
    with dai.Pipeline(device) as pipeline:
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        q = left.requestOutput((640, 480)).createOutputQueue(maxSize=2, blocking=False)
        pipeline.start()
        n, t0 = 0, time.monotonic()
        while time.monotonic() - t0 < 5:
            if q.has():
                q.get(); n += 1
            time.sleep(0.02)
        print("frames", n)