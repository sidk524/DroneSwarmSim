import depthai as dai

RGB_SIZE   = (1280, 720)   # change RGB resolution here
MONO_SIZE  = (640, 480)    # change depth resolution here (stereo input res)
FPS        = 30

visualizer = dai.RemoteConnection(httpPort=8080)

with dai.Pipeline() as pipeline:
    # --- RGB ---
    rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb_out = rgb_cam.requestOutput(size=RGB_SIZE, type=dai.ImgFrame.Type.NV12, fps=FPS)

    # --- Depth ---
    left  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_out  = left.requestOutput(size=MONO_SIZE, fps=FPS)
    right_out = right.requestOutput(size=MONO_SIZE, fps=FPS)

    stereo = pipeline.create(dai.node.StereoDepth).build(
        left_out, right_out, dai.node.StereoDepth.PresetMode.ROBOTICS
    )
    stereo.initialConfig.postProcessing.decimationFilter.decimationFactor = 1
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    # --- Visualizer topics ---
    visualizer.addTopic("rgb",   rgb_out,          group="RGB")
    visualizer.addTopic("depth", stereo.depth,     group="Depth")

    pipeline.start()
    visualizer.registerPipeline(pipeline)

    while pipeline.isRunning():
        if visualizer.waitKey(1) == ord('q'):
            pipeline.stop()