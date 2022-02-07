
'''
@Copyright 2022 Vivek Sood
@file hw1
@author Vivek Sood
@date 02/07/2022
 
@brief Homework 1(ENPM 809T): Moving Average
 
@section LICENSE
 
MIT License
Copyright (c) 2022 Vivek Sood
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''  

import matplotlib.pyplot as plt


def movingAverage(dataIn, window):
    output = list()
    for i in range(len(dataIn)-window + 1):
        output.append(sum(dataIn[i:i+window])/window)
    return output
def main():
    yPlot = list()
    window = 128
    with open('imudata.txt') as imuData:
        for line in imuData:
            data = float(line.split(" ")[4])
            yPlot.append(data)
    
    yPlotAvg = movingAverage(yPlot, window)
    plt.plot(yPlot)
    plt.show()
if __name__ == '__main__':
    main()

