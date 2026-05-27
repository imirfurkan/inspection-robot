
import depthai as dai
import time

pipeline = dai.Pipeline()
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor([dai.IMUSensor.ROTATION_VECTOR], 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
q = imu.out.createOutputQueue()

pipeline.start()
time.sleep(1)

for i in range(20):
    pkt = q.tryGet()
    if pkt:
        for p in pkt.packets:
            print(dir(p))
            rv = p.rotationVector
            print(f'quat: i={rv.i:.4f} j={rv.j:.4f} k={rv.k:.4f} real={rv.real:.4f}')
            break
        break
    time.sleep(0.1)

pipeline.stop()
