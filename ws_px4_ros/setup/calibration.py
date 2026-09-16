import depthai as dai

with dai.Pipeline() as pipeline:
    left  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    dyn = pipeline.create(dai.node.DynamicCalibration)
    left.requestFullResolutionOutput().link(dyn.left)
    right.requestFullResolutionOutput().link(dyn.right)

    ctrl     = dyn.inputControl.createInputQueue()
    result_q = dyn.calibrationOutput.createOutputQueue()
    cover_q  = dyn.coverageOutput.createOutputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    pipeline.start()
    ctrl.send(dai.DynamicCalibrationControl.setPerformanceMode(
        dai.DynamicCalibrationControl.PerformanceMode.DEFAULT))
    ctrl.send(dai.DynamicCalibrationControl.resetData())
    ctrl.send(dai.DynamicCalibrationControl.startCalibration())
    print("Collecting. Sweep the camera slowly over the boxes at 1-2 m. Ctrl+C to quit.")

    pending = None
    try:
        while pipeline.isRunning():
            cov = cover_q.tryGet()
            if cov:
                print(f"coverage={cov.meanCoverage:.2f}")

            res = result_q.tryGet()
            if res:
                print(f"result: {res.info}")
                d = getattr(res, "calibrationData", None)
                if d is not None:
                    print("  fields:", [a for a in dir(d) if not a.startswith("_")])
                    diff = getattr(d, "calibrationDifference", None)
                    if diff is not None:
                        print("  diff fields:", [a for a in dir(diff) if not a.startswith("_")])
                        for name in ("pairwiseRotationDifference", "rotationChange",
                                     "depthErrorDifference", "sampsonErrorCurrent", "sampsonErrorNew"):
                            if hasattr(diff, name):
                                print(f"  {name} = {getattr(diff, name)}")
                    pending = getattr(d, "newCalibration", None)
                    if pending is not None:
                        ans = input("apply this calibration? [y/N] ").strip().lower()
                        if ans == "y":
                            device.flashCalibration(pending)
                            print("flashed to EEPROM")
                            ctrl.send(dai.DynamicCalibrationControl.applyCalibration(pending))
                            print("applied")
                            break
    except KeyboardInterrupt:
        pass
    pipeline.stop()