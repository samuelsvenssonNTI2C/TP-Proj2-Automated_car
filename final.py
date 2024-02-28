import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2 as picam, Preview
from time import sleep

GPIO.setmode(GPIO.BOARD)
motorAPin = 33
motorBPin = 35
GPIO.setup(motorAPin, GPIO.OUT)
GPIO.setup(motorBPin, GPIO.OUT)
buttonPin = 10
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

stepSize = 5
testMode = False
imageMode = False
resolution = (800, 600)

def startCamera():
    camera = picam()
    camera.configure(camera.create_still_configuration(main={"format": 'XRGB8888', "size": resolution}))
    camera.start_preview(Preview.NULL) # Makes the camera work without a physical screen
    camera.start()
    print("Camera started")
    return camera
    
def takeImage(camera):
    image = camera.capture_array()
    if testMode:
        print("Image taken")
    return image

def findEdges(image):
    edgeImage = cv.Canny(image, 60, 90, L2gradient=True)
    if testMode:
        print("Edges found")
    if imageMode:
        cv.imshow("edges", edgeImage)
    return edgeImage

def arrayFromEdges(edgeImage):
    edgeArray = []
    for x in range(0, len(edgeImage[0]), stepSize):
        pixel = (x, 0)
        for y in range(len(edgeImage)-1, 0, -1):
            if edgeImage.item(y, x) == 255:
                pixel = (x, y)
                break
        edgeArray.append(pixel)
    if testMode:
        print("edge-array created")
    return edgeArray

def drawLinesFromPoints(image, points):
    for i in range(len(points)-1):
        cv.line(image, points[i], points[i+1], (0, 255, 0), 1)
        
    for i in range(len(points)):
        cv.line(image, (i*stepSize, len(image)), points[i], (0, 255, 0), 1)
    if testMode:
        print("Lines Drawn")
    if imageMode:
        cv.imshow("Test", image)
    else:
        saveImage(image, 'image.jpg')

def getChunks(array, n):
    chunks = []
    chunkSize = int((len(array))/n)
    for i in range(0, len(array)-1, chunkSize):
        chunks.append(array[i:i+chunkSize])
    if testMode:
        print("chunks made")
    return chunks

def avragePoints(points):
    xVals = []
    yVals = []
    for x, y in points:
        xVals.append(x)
        yVals.append(y)
    avrX = int(np.average(xVals))
    avrY = int(np.average(yVals))
    if testMode:
        print("avrage points calculated")
    return avrX, avrY

def saveImage(image, fileName):
    cv.imwrite(fileName, image)

class Car:
    motorA = GPIO.PWM(motorAPin, 2000)
    motorB = GPIO.PWM(motorBPin, 2000)
    
    def start():
        Car.motorA.start(0)
        Car.motorB.start(0)
        print("Motors started")
    
    def forward():
        Car.motorA.ChangeDutyCycle(100.0)
        Car.motorB.ChangeDutyCycle(70.0)
        print("Forward")
    
    def left():
        Car.motorA.ChangeDutyCycle(0)
        Car.motorB.ChangeDutyCycle(100)
        print("Left")
    
    def right():
        Car.motorA.ChangeDutyCycle(100)
        Car.motorB.ChangeDutyCycle(0)
        print("Right")
    
    def reverse():
        Car.motorA.ChangeDutyCycle(0)
        Car.motorB.ChangeDutyCycle(0)
        print("N/A")
    
    def stop():
        Car.motorA.stop()
        Car.motorB.stop()
        print("Stop")

def main():
    cam = startCamera()
    Car.start()
    
    while True:
        frame = takeImage(cam)
        edges = findEdges(frame)
        points = arrayFromEdges(edges)
        drawLinesFromPoints(frame, points)
        chunks = getChunks(points, 3)
        
        hights = []
        for i in range(len(chunks)):
            hights.append(resolution[1] - avragePoints(chunks[i])[1])
        print(hights)
        if hights[1] < 0.40*resolution[1]:
            if hights[0] > hights[2]:
                Car.left()
            else: 
                Car.right()
        else:
            Car.forward()
            
        if GPIO.input(buttonPin) == GPIO.HIGH:
            break
        
        sleep(0.1)
    
    Car.stop()
    cam.stop()
    GPIO.cleanup()
    print("Script finished")

main()