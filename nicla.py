from pyb import UART, LED
import pyb
import sensor
import time
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)
sensor.skip_frames(time=2000)
r_LED = LED(1)
g_LED = LED(2)
b_LED = LED(3)
g_LED.on()
r_LED.off()
b_LED.off()
clock = time.clock()
uart = UART("LP1", 115200, timeout_char=2000)
def checksum(arr, initial= 0):
    sum = initial
    for a in arr:
        sum += a
    checksum = 0xFFFF - sum
    chA = checksum >> 8
    chB = checksum & 0xFF
    return chA, chB
def IBus_message(message_arr_to_send):
    msg = bytearray(32)
    msg[0] = 0x20
    msg[1] = 0x40
    for i in range(len(message_arr_to_send)):
        msg_byte_tuple = bytearray(message_arr_to_send[i].to_bytes(2, 'little'))
        msg[int(2*i + 2)] = msg_byte_tuple[0]
        msg[int(2*i + 3)] = msg_byte_tuple[1]
    chA, chB = checksum(msg[:-2], 0)
    msg[-1] = chA
    msg[-2] = chB
    uart.write(msg)
def refreshIbusConnection():
    if uart.any():
        uart_input = uart.read()
while True:
    thresholdsBlue = (10, 78, -25, 25, -64, -12)
    thresholdsOrange = (30, 100, 28, 49, 9, 47)
    thresholdsGreen = (0, 100, -43, -22, 5, 37)
    thresholdsRed = (25, 73, 75, 34, 4, 45)
    ledRed = pyb.LED(1)  # Initiates the red led
    ledGreen = pyb.LED(2)  # Initiates the green led


    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([thresholdsBlue, thresholdsOrange, thresholdsGreen, thresholdsRed], area_threshold=2500, merge=True)
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        center_x = blob.cx()
        center_y = blob.cy()
        with open('blob_centers.txt', 'a') as file:
            file.write("Center coordinates: ({}, {})\n".format(center_x, center_y))
            if len(blobs) > 0:
                ledGreen.on()
                ledRed.off()
            else:
                ledGreen.off()
                ledRed.on()
    #pyb.delay(50)
    print(clock.fps())
    color_is_detected = False
    flag = 0
    if (color_is_detected):
        g_LED.off()
        r_LED.off()
        b_LED.on()
        flag = 1
    else:
        g_LED.on()
        r_LED.off()
        b_LED.off()
    pixels_x = 0
    pixels_y = 0
    pixels_w = 0
    pixels_h = 0
    messageToSend = [flag, pixels_x, pixels_y, pixels_w, pixels_h]
    IBus_message(messageToSend)
    refreshIbusConnection()
