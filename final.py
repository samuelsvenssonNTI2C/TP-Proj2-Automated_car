# Author: Samuel Svensson
# Date: 2024-02-28
# Description: A program that controls a cars forward and steering to keep it from driving into objects.
# It uses a camera at the front to see whats in front of the car and then processes the image recieved to know where a possible object could be.

# Importing libraries
import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2 as picam, Preview
from time import sleep

# Setup for pins
GPIO.setmode(GPIO.BOARD)
motorAPin = 33
motorBPin = 35
GPIO.setup(motorAPin, GPIO.OUT)
GPIO.setup(motorBPin, GPIO.OUT)
buttonPin = 10
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Settings for the program
testMode = False # Program prints where it is in the code
imageMode = False # Program displays images from different stages of image-processing (display required)
resolution = (800, 600) # Resolution of images taken by the camera

# configure and start the camera
# Returns: camera (Object)
def startCamera():
    camera = picam()
    camera.configure(camera.create_still_configuration(main={"format": 'XRGB8888', "size": resolution}))
    if imageMode:
        camera.start_preview()
    else:
        camera.start_preview(Preview.NULL) # Makes the camera work without a physical screen
    camera.start()
    print("Camera started")
    return camera

# Takes an image using the camera
# Parameters:
# camera (object)
# Returns: image (Array)
def takeImage(camera):
    image = camera.capture_array()
    if testMode:
        print("Image taken")
    return image

# Process an image to get the edges of objects whithin the frame
# Parameters:
# image (Array)
# Returns: edgeImage (Array)
def findEdges(image):
    edgeImage = cv.Canny(image, 60, 90, L2gradient=True)
    if testMode:
        print("Edges found")
    if imageMode:
        cv.imshow("edges", edgeImage)
    return edgeImage

# Finds the lowest edges in an image and saves the cordinates in an Array
# Parameters:
# edgeImage (Array)
# Returns: edgeArray (Array)
def arrayFromEdges(edgeImage):
    edgeArray = []
    for x in range(0, len(edgeImage[0]), 5):
        pixel = (x, 0)
        for y in range(len(edgeImage)-1, 0, -1):
            if edgeImage.item(y, x) == 255:
                pixel = (x, y)
                break
        edgeArray.append(pixel)
    if testMode:
        print("edge-array created")
    return edgeArray

# A function for saving an image
# Parameters:
# image (Array)
# fileName (String)
# Returns: None (NoneType)
def saveImage(image, fileName):
    cv.imwrite(fileName, image)

# This function draws a line between cordinates onto an image. If "imageMode" is True, then the image will be displayd, else the image will be saved.
# Parameters:
# image (Array)
# points (Array)
# Returns: None (NoneType)
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

# Divides an Array into n equally wide sections and returns them as seperate Arrays inside an Array
# Parameters:
# array (Array)
# n (Int)
# Returns: chunks (Array)
def getChunks(array, n):
    chunks = []
    chunkSize = int((len(array))/n)
    for i in range(0, len(array)-1, chunkSize):
        chunks.append(array[i:i+chunkSize])
    if testMode:
        print("chunks made")
    return chunks

# Calculates the average values for x and y in an Array.
# Parameters:
# points (Array)
# Returns: avrX (Int), avrY (Int)
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


# A Class for control of the motors
class Car:
    motorA = GPIO.PWM(motorAPin, 2000)
    motorB = GPIO.PWM(motorBPin, 2000)

    # Starts the motors by setting their PWM output to zero
    # Returns: None (NoneType)
    def start():
        Car.motorA.start(0)
        Car.motorB.start(0)
        print("Motors started")

    # Drives the motors forward
    # Returns: None (NoneType)
    def forward():
        Car.motorA.ChangeDutyCycle(100.0)
        Car.motorB.ChangeDutyCycle(70.0)
        print("Forward")

    # Makes right motor spin faster to steer the car left
    # Returns: None (NoneType)
    def left():
        Car.motorA.ChangeDutyCycle(0)
        Car.motorB.ChangeDutyCycle(100)
        print("Left")

    # Makes left motor spin faster to steer the car right
    # Returns: None (NoneType)
    def right():
        Car.motorA.ChangeDutyCycle(100)
        Car.motorB.ChangeDutyCycle(0)
        print("Right")

    # Stops the motors and turns of PWM
    # Returns: None (NoneType)
    def stop():
        Car.motorA.stop()
        Car.motorB.stop()
        print("Stop")

# The main function of the program
# Returns: None (NoneType)
def main():
    # Start sequence
    cam = startCamera()
    Car.start()
    
    while True:
        # Image processing
        frame = takeImage(cam)
        edges = findEdges(frame)
        points = arrayFromEdges(edges)
        drawLinesFromPoints(frame, points)
        chunks = getChunks(points, 3)

        # Calculate if to move forward or steer right or left
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

        # Physical stop button
        if GPIO.input(buttonPin) == GPIO.HIGH:
            break
        
        sleep(0.1) # Delay to slow down CPU usage
        
    # Stop sequence
    Car.stop()
    cam.stop()
    GPIO.cleanup()
    print("Script finished")

main()
