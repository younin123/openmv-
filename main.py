# 快速线性回归（巡线）例程
#
# 这个例子展示了如何在OpenMV Cam上使用get_regression（）方法来获得
# ROI的线性回归。 使用这种方法，你可以轻松地建立一个机器人，它可以
# 跟踪所有指向相同的总方向但实际上没有连接的线。 在线路上使用
# find_blobs（），以便更好地过滤选项和控制。
#
# 这被称为快速线性回归，因为我们使用最小二乘法来拟合线。然而，这种方法
# 对于任何具有很多（或者甚至是任何）异常点的图像都是不好的，
# 这会破坏线条拟合.

#设置阈值，（0，100）检测黑色线
THRESHOLD = (0, 80) # Grayscale threshold for dark things...

#设置是否使用img.binary()函数进行图像分割
BINARY_VISIBLE = True # 首先执行二进制操作，以便您可以看到正在运行的线性回归...虽然可能会降低FPS。

import sensor, image, time, car
from pid import PID
from pyb import LED
rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.1, i=0)

#LED(1).on()
#LED(2).on()
#LED(3).on()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()
    blobs = img.find_blobs([THRESHOLD], roi = (0,0,80,60),invert=True)
    if (blobs):
        for blob in blobs:
            print(blob.w())
            img.draw_rectangle(blob.rect())
            #a +=
    line = img.get_regression([(THRESHOLD) if BINARY_VISIBLE else THRESHOLD],invert=True)
    if (line): img.draw_line(line.line(), x_stride = 20, color = 127)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        rho_output = rho_pid.get_pid(rho_err,1)
        theta_output = theta_pid.get_pid(theta_err,1)
        output = rho_output+theta_output
        #car.run(50+output, 50-output)
        print(50+output, 50-output)
