import depthai as dai
import time
import math

pipeline = dai.Pipeline()
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.ROTATION_VECTOR], 50)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
q = imu.out.createOutputQueue()

pipeline.start()
time.sleep(1)

print("Hold the OAK-D in different positions and note the values.")
print("Press Ctrl+C to stop.\n")
print(f"{'Accel X':>9} {'Accel Y':>9} {'Accel Z':>9}  |  {'Quat i':>9} {'Quat j':>9} {'Quat k':>9} {'Quat r':>9}  |  {'Pitch':>7} {'Roll':>7} {'Yaw':>7}")
print("-" * 110)

try:
    while True:
        pkt = q.tryGet()
        if pkt:
            p = pkt.packets[-1]
            a = p.acceleroMeter
            rv = p.rotationVector

            # Current euler conversion (ZYX)
            qi, qj, qk, qr = rv.i, rv.j, rv.k, rv.real
            sinr = 2 * (qr * qi + qj * qk)
            cosr = 1 - 2 * (qi * qi + qj * qj)
            roll = math.atan2(sinr, cosr) * 57.2958

            sinp = 2 * (qr * qj - qk * qi)
            pitch = (math.copysign(90, sinp) if abs(sinp) >= 1 else math.asin(sinp) * 57.2958)

            siny = 2 * (qr * qk + qi * qj)
            cosy = 1 - 2 * (qj * qj + qk * qk)
            yaw = math.atan2(siny, cosy) * 57.2958

            print(f"{a.x:9.3f} {a.y:9.3f} {a.z:9.3f}  |  {qi:9.4f} {qj:9.4f} {qk:9.4f} {qr:9.4f}  |  {pitch:7.1f} {roll:7.1f} {yaw:7.1f}", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n\nDone.")
    pipeline.stop()