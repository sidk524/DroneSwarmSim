import depthai as dai

RGB_SIZE  = (1280, 800)
MONO_SIZE = (640, 400)   # native OV7251 res on the OAK-D Lite
FPS       = 30

visualizer = dai.RemoteConnection(httpPort=8080)

with dai.Pipeline() as pipeline:

    # --- RGB ---
    rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb_out = rgb_cam.requestOutput(size=RGB_SIZE, type=dai.ImgFrame.Type.NV12, fps=FPS)

    # --- Stereo ---
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

    # --- Auto calibration (host node, so we can see results) ---
    auto_calib = pipeline.create(dai.node.AutoCalibration).build(left, right)
    auto_calib.initialConfig.mode = dai.AutoCalibrationConfig.CONTINUOUS
    auto_calib.initialConfig.flashCalibration = True     # False = apply in RAM only, don't write EEPROM
    auto_calib.initialConfig.maxIterations = 3
    auto_calib.initialConfig.sleepingTime = 10
    auto_calib.initialConfig.validationSetSize = 5
    auto_calib.initialConfig.dataConfidenceThreshold = 0.7
    calib_q = auto_calib.output.createOutputQueue()

    # --- Visualizer topics ---
    visualizer.addTopic("rgb",        rgb_out,               group="RGB")
    visualizer.addTopic("depth",      stereo.depth,          group="Depth")
    visualizer.addTopic("disparity",  stereo.disparity,      group="Depth")
    visualizer.addTopic("rect_left",  stereo.rectifiedLeft,  group="Debug")
    visualizer.addTopic("rect_right", stereo.rectifiedRight, group="Debug")

    pipeline.start()
    visualizer.registerPipeline(pipeline)
    print("Auto calibration running (CONTINUOUS). Sweep the camera slowly over textured objects.")

    while pipeline.isRunning():
        if visualizer.waitKey(1) == ord('q'):
            pipeline.stop()

        r = calib_q.tryGet()
        if r is not None:
            if r.passed:
                print(f"[calib] PASSED  dataConfidence={getattr(r, 'dataConfidence', None)}  "
                      f"calibrationConfidence={getattr(r, 'calibrationConfidence', None)}")
            else:
                print("[calib] did not pass (not enough coverage/texture, will retry)")