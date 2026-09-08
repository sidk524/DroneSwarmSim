import depthai as dai

vis = dai.RemoteConnection()      # web visualizer on port 8082

with dai.Pipeline() as pipeline:
    # RGB camera
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cam_out = cam.requestOutput((640, 400))          # ← this is cam_out

    # Stereo depth from the two mono cameras
    left  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth).build(
        left.requestOutput((640, 400)),
       right.requestOutput((640, 400)),  )

    vis.addTopic("rgb", cam_out)
    vis.addTopic("depth", stereo.depth)

    pipeline.start()
    vis.registerPipeline(pipeline)
    while pipeline.isRunning():
        vis.waitKey(1)
