import threading
import os
import sys

if len(sys.argv) > 1 and sys.argv[1] == "-old-compute":

    avgTimeArray = []

    timeArray = []
    for i in range(64):
        timeArray.append(0)
        avgTimeArray.append(0)

    def threadProc(i):
        os.system("~/pro/PhysX/physx/bin/linux.clang/release/Snippet_Simbicon_64 ~/pro/PhysX/physx/snippets/snippet_simbicon/config.txt > {0}.txt".format(i))
        with open('{0}.txt'.format(i), 'r') as file:
            data = file.read()
            timeArray[i] = int(data)

    threads = []

    for testT in range(1, 64):

        threads = []
        for i in range(testT):
            t = threading.Thread(target=threadProc, args=(i,))
            threads.append(t)
            t.start()

        for i in range(testT):
            threads[i].join()
        
        for i in range(testT):
            assert timeArray[i] != 0
            avgTimeArray[testT] += timeArray[i]
        
        avgTimeArray[testT] /= testT

        print("threads = {0}, avg time = {1}".format(testT, avgTimeArray[testT]))
    
    quit()

import time

if len(sys.argv) > 1 and sys.argv[1] == "-compute":

    avgTimeArray = []

    for i in range(64):
        avgTimeArray.append(0)

    def threadProc2(i):
        os.system("~/pro/PhysX/physx/bin/linux.clang/release/Snippet_Simbicon_64 ~/pro/PhysX/physx/snippets/snippet_simbicon/config.txt > /dev/null")

    threads = []

    for testT in range(1, 64):

        threads = []
        for i in range(testT):
            t = threading.Thread(target=threadProc2, args=(i,))
            threads.append(t)

        startTime = time.time()

        for i in range(testT):
            threads[i].start()

        for i in range(testT):
            threads[i].join()
        
        endTime = time.time()
        
        avgTimeArray[testT] = (endTime - startTime) / testT

        print("threads = {0}, avg time = {1}".format(testT, avgTimeArray[testT]))
    
    with open('threadingResult.txt', 'w') as file:
        for x in avgTimeArray:
            file.write(str(x) + "\n")

import math

dataArr = []
dataArrLog = []

with open('threadingResult.txt', 'r') as file:
    lines = file.readlines()
    for line in lines:
        if line.strip() == '':
            continue
        x = float(line.strip())
        if x == 0:
            continue
        dataArr.append(x)
        dataArrLog.append(math.log2(x))

import matplotlib.pyplot as plt
plt.plot(list(range(len(dataArr))), dataArr)
plt.ylabel('average time per scene for 10000 frames')
plt.xlabel('nThreads')
plt.show() 

import matplotlib.pyplot as plt
plt.plot(list(range(20, len(dataArr))), dataArr[20:])
plt.ylabel('average time per scene for 10000 frames')
plt.xlabel('nThreads')
plt.show() 