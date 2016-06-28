# Demo of using multiprocessing for generating data in one process and plotting
# in another.
# Written by Robert Cimrman
# Requires >= Python 2.6 for the multiprocessing module or having the
# standalone processing module installed

from __future__ import print_function
import numpy
import time, threading
import NBPlot

def Controller():
    print (time.time(), "Controller")
    
def main():
    
    data={}
    pl = NBPlot.NBPlot()

    i = 0
    t=0
    for ii in range(500):
        if i == 0:
            data["x"]= numpy.random.random((3,1))
            data["a"]= numpy.random.random((3,1))
            data["theta"] = numpy.zeros((3,1))
            data["t"] = t
            pl.plot(data)
            i = 10
        time.sleep(0.01)
        print (time.time(), "Controller")
        
        i -= 1
        t+=1
        print ("ii: ", ii)
        
#    raw_input('press Enter...')
    pl.plot(None, finished=True)

if __name__ == '__main__':
    main()
