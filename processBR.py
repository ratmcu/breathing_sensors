from __future__ import division
from __future__ import absolute_import
import multiprocessing
import time
import globalVars
import collections
import numpy as np
# from scipy import signal, linalg
from matplotlib import pyplot as plt
import math
# from tftb.processing import ShortTimeFourierTransform
import pyeemd
# from pyeemd.utils import plot_imfs
# from scipy.fftpack import fft
# from scipy.spatial.distance import minkowski
import scipy as sp
import pandas as pd
from scipy import signal

from sklearn import svm
from sklearn.externals import joblib
from sklearn.preprocessing import normalize

# import tensorflow as tf
import random

import csv

import pywt

from collections import Counter

#My classes
from colorPrint import bcolors
from sigplotter import sigplotter

#3rd party functions/classes
# from freq_estimator import *

class processBR(multiprocessing.Process):
    def __init__(self, inputQ, outputQ,stopEvent,radarSettings):
        multiprocessing.Process.__init__(self)
        self.NORMALIZE_RADAR_DATA = False #If using a X2 radar or, a less than 2.0 firmware version for X4 radar, set this to true
        self.SHOW_BREATHING_PLOTS = True #Disable the plotter thread. Disabling this will also disable saving plots if that option is enabled from within the plotter thread
        if radarSettings['RadarType'] == 'X2':
            self.NORMALIZE_RADAR_DATA = True
        self.NUM_HISTORY = 20 # Number of samples to keep as history
        self.NOMINALFS = 17 # Nominal value for Fs
        self.NUM_LONG_HISTORY = 100 # Number of samples(seconds, if NominalFS=realFs) to keep as history
        # self.RADAR_RESOLUTION = 3.90625 / 1000 # X2
        # self.RADAR_RESOLUTION = 51.8617 / 1000 #X4
        self.RADAR_RESOLUTION = radarSettings['RADAR_RESOLUTION']
        self.EMD_EVERY_X_ITER = 1   #Calculate EMD once every 'X'(This number) of iterations
        self.PLOT_EMD_EVERY_X_ITER = 1 #10 is the default value
        self.MOVE_DETECTION_THRESHOLD = 0.5 # a change of range bin with this amount of meters will result in classifying the person to be moving
        self.FAST_MOVE_DETECTION_THRESHOLD = 0.8 #An instantaneous change in range bin with this amount of meters will result in classifying the person to be moving fast
        self.NUM_RANGE_BIN_CHANGE_HISTORY = 3 # compare X previous range bins with the
        # NUM_HISTORY number of bins to see if the person is moving
        self.TRAINSVM = False
        self.TRAINDURATION = 240 # Train using this many seconds of data
        self.TOTAL_ENERGY_THRESHOLD = 0.00
        # self.AUTOCORR_MOVEMENT_TIME_THRESHOLD = 0.5 #TEMP######
        self.AUTOCORR_MOVEMENT_TIME_THRESHOLD = 1.0

        self.pltCounter=0
        self.referenceSignal = []
        self.referenceRate = 0
        self.referenceSignalTime = []
        self.inputQ = inputQ
        self.outputQ = outputQ
        self.stopEvent = stopEvent
        self.radarSettings = radarSettings
        self.currentInputDict = {}
        self.currentData = [] #radarData with timestamps
        self.currentRadarData = np.array([]) #normalized radarData without timestamps
        self.currentRangeBin = 0
        self.currentMedianRangeBin = 0
        self.previousMedianRangeBin = -10
        self.currentRange = 0
        self.filteredPersonBinSignal = []
        self.SVDFiltered = []
        self.currentSlowTimes = []
        self.imfs = []
        self.imfFFTs = [] #FFT of each IMF
        self.minkowskiDistances = []
        self.minkowskiDistancesCombinedIMFs = []
        self.EMDMinkBRate = 0
        # self.minkowskyDist = [] #minkowskyDistance between IMF and original signal
        self.imfMaxFreq = [] #Maximum frequency component of each IMF
        self.totalPowerRatio = [] #Ratio to total power of each IMF
        self.pwrRatio = [] #Ratio between power of two different bands as specified in calculateEMD
        self.emdIMFNotFoundCount = 0
        self.EMDBreathSignal = []
        self.emdTimes = []
        self.emdCalculated = False
        self.wtcoeffs = [] #Wavelet transform coefficients
        self.wtFFTMaxFreq = [0]

        self.newestBreathRate = 0
        self.medianRangeChanged = True
        self.fastMovementDetected = False
        self.fastMovementCount = 0
        self.numMovements = 0
        self.varianceOfAllBins = []

        self.imfScoreValues = []
        self.imfScores = []
        self.scoreMaxIdx = 0
        self.imfScoreRowSums = []

        self.trainingSet = []
        self.labelSet = []
        # self.classifier = joblib.load('IGMoving15BPM.pkl')

        self.ERSBFeatures = []
        self.ERSBLabels = []
        self.ERSBCombined = []
        self.ERSB_FREQ_MAX = 2 #Maximum frequency of FFT in Hz
        self.ERSB_TEST_SET_FILE_NAME = 'trainingData/MVTestV2_1.csv' #Specify test set file name, this variable is overwritten if testing data is saved within this application
        self.ERSB_TRAINING_SET_FILE_NAME = 'trainingData/MVTrainV2_2.csv'#Specify training set file name, this variable is overwritten if training data is saved within this application
        self.fftOfRawERSB = np.zeros(0)
        self.xOfFFTRAWERSB = np.zeros(0)
        self.rawData = np.zeros(0)
        self.normalizedFFT = np.zeros(0)
        self.indexOfMaxFFT = 0
        self.freqOfMaxFFT = 0
        self.localMaximaFFTIdx = np.zeros(0)
        self.localMaximaFFT = np.zeros(0)

        self.emdBreathRates = collections.deque(maxlen=math.ceil(self.NUM_HISTORY/self.EMD_EVERY_X_ITER))
        self.rangeDeck = collections.deque(maxlen=self.NUM_HISTORY) # store NUM_HISTORY(=40) range values
        self.rangeBinDeck = collections.deque(maxlen=self.NUM_HISTORY)  # store NUM_HISTORY(=40) range values

        self.medianrangeDeck = collections.deque(maxlen=self.NUM_HISTORY)  # store NUM_HISTORY(=40) range values
        self.medianrangeBinDeck = collections.deque(maxlen=self.NUM_HISTORY)  # store NUM_HISTORY(=40) range values

        self.totalPowerSVDDeck = collections.deque(maxlen=self.NUM_HISTORY)  # store NUM_HISTORY(=40) power values
        self.totalPowerSVD = 0

        self.breathSignalDeck = collections.deque(maxlen=self.NUM_HISTORY*self.NOMINALFS)
        self.breathSignal5sRangeDeck = collections.deque(maxlen=self.NUM_HISTORY * self.NOMINALFS)
        self.radarDataDeck = collections.deque(maxlen=self.NUM_HISTORY * self.NOMINALFS) #radarData Deck without timestamps
        self.slowTimeDeck = collections.deque(maxlen=self.NUM_HISTORY*self.NOMINALFS) #timestamps of the radarData Deck

        self.longRadarDataDeck = collections.deque(maxlen=self.NUM_HISTORY * self.NUM_LONG_HISTORY)
        self.fftDeck = collections.deque(maxlen=self.NUM_HISTORY)
        self.fftTimesDeck = collections.deque(maxlen=self.NUM_HISTORY)
        self.trackedRangebins = collections.deque(maxlen=5) #Track 5 range bins
        self.fftFeaturesDeck = collections.deque(maxlen=self.NUM_HISTORY)
        self.fftFeatures = np.zeros(0)

        self.maxVariance = 0
        self.maxVarianceDeck = collections.deque(maxlen=100)

        self.binAutoCorrelation = np.zeros(0)
        self.autocorrelationMainLobeWidth = np.zeros(0)
        self.autoCorrelationMinimaIndices = np.zeros(0)
        self.autoCorrelationMinimaValues = np.zeros(0)
        self.autoCorrelationMainLobeWidthTime = np.zeros(0)
        self.autocorrelationMainLobeTimesDeck = collections.deque(maxlen=self.NUM_LONG_HISTORY)
        self.autocorrMovementDetected = False
        self.autoCorrBRate = 0

        self.capturedBreathingSignal = []

        self.currentBreathRates = {}
        self.jobs = []
        self.fs = 0
        self.eventDict = {}
        self.eventDict['event_type'] = 'None'

        self.plotterInputQ = multiprocessing.Queue()
        self.plotterOutputQ = multiprocessing.Queue()
        self.plotterStopEvent = multiprocessing.Event()
        if self.SHOW_BREATHING_PLOTS:
            self.plotter = sigplotter(self.plotterInputQ, self.plotterOutputQ, self.plotterStopEvent)
            self.plotter.start()

    def run(self):
        try:
            iteration = 0
            mainStartTime = time.time()
            while not self.stopEvent.is_set():
                while not self.inputQ.empty():
                    iteration+=1
                    if self.inputQ.qsize() > 2:
                        print (bcolors.WARNING+'Input Q Size: '+str(self.inputQ.qsize())+bcolors.ENDC)
                    self.currentInputDict = self.inputQ.get()
                    if 'action' in self.currentInputDict:
                        if self.currentInputDict['action'] == 'Add to ERSB training set': #Add previous data to, empty room stop breathing training set
                            self.addToERSBTrainSet(self.currentInputDict['ERSB Class'])
                        elif self.currentInputDict['action'] == 'Train ERSB':
                            self.trainERSB()
                        elif self.currentInputDict['action'] == 'Train and test ERSB':
                            self.trainAndTestERSB()
                        elif self.currentInputDict['action'] == 'Test ERSB':
                            self.testERSB()
                        elif self.currentInputDict['action'] == 'Save ERSB train':
                            self.saveERSBTrainingSet()
                        elif self.currentInputDict['action'] == 'Save ERSB test':
                            self.saveERSBTestingSet()
                    self.currentData = self.currentInputDict['radarData']
                    self.referenceSignal = self.currentInputDict['RefSignal']
                    self.referenceRate = self.currentInputDict['RefRate']
                    self.referenceSignalTime = self.currentInputDict['RefSignalTime']
                    if len(self.currentData) < 2: # stop if not enough data (Simulation)
                        print ('\x1b[1;37;41m' + 'Not enough data. Process going to idle mode' + '\x1b[0m')
                        # self.safeStop()
                        # self.stopEvent.set()
                        break
                    if isinstance(self.currentData[-1][0], complex):
                        self.fs = 1000.0*len(self.currentData)/(self.currentData[-1][0].real - self.currentData[0][0].real)
                        self.endTime = self.currentData[-1][0].real
                    else:
                        self.fs = 1000.0 * len(self.currentData) / (self.currentData[-1][0] - self.currentData[0][0])
                        self.endTime = self.currentData[-1][0]
                    print ('Fs : %f' % self.fs)

                    self.normalizeRadarData(modifySelf=True)
                    self.radarDataDeck.extend(self.currentRadarData)
                    # self.longRadarDataDeck.extend(self.currentRadarData)
                    # print(self.currentData[0][1])
                    self.findRange(self.currentRadarData,self.fs,modifySelf=True)
                    self.trackRange()
                    self.filterBin(np.array(self.radarDataDeck)[:,self.currentRangeBin],self.fs,modifySelf=True)
                    self.storeHistory()

                    self.calculateERSB()
                    if iteration%self.EMD_EVERY_X_ITER == 0 and len(self.radarDataDeck) > self.NUM_HISTORY*self.fs/2:
                        # self.calculateEMD()
                        self.calculateEMDWTScore()
                        if self.TRAINSVM:
                            self.buildFeaturesAndTrainingSet(15) # Train with this dataset
                    else:
                        print('EMD not calculated###########')
                        print(iteration%self.EMD_EVERY_X_ITER)
                        print(len(self.radarDataDeck))
                        print(self.NUM_HISTORY*self.fs/2)
                        print('THIS IS WHY###############')


                    # self.currentBreathRates = self.findBreathRateAll(self.currentData,[1,2,3,4,5],self.fs)
                    self.currentBreathRates = self.emdBreathRates

                    # self.outputQ.put([self.currentRangeBin,self.filteredPersonBinSignal, self.currentBreathRates])
                    self.outputQ.put([self.currentMedianRangeBin,\
                                      # np.array(self.radarDataDeck)[:, self.currentMedianRangeBin],\
                                      self.EMDBreathSignal,\
                                      self.currentBreathRates,\
                                      self.eventDict,\
                                      self.freqOfMaxFFT*60,\
                                      self.wtFFTMaxFreq[0]*60,\
                                      self.EMDMinkBRate,
                                      self.currentRangeBin])

                    # self.eventDict['event_type'] = 'None'

                    # self.drawPlots()
                    if self.SHOW_BREATHING_PLOTS:
                        self.sendDataToPlot(iteration)
                    if self.TRAINSVM and (time.time() - mainStartTime) > self.TRAINDURATION:
                        print ('Training and saving data, this may take a while...')
                        self.trainSVM('IGMoving15BPM.pkl')
                        # self.stopEvent.set()
                time.sleep(0.1)

            self.safeStop()
        except KeyboardInterrupt:
            self.safeStop()

    def trackRange(self):
        if self.trackedRangebins.count(self.currentRangeBin)>0:
            self.trackedRangebins.remove(self.currentRangeBin)
            self.trackedRangebins.append(self.currentRangeBin)
        else:
            self.trackedRangebins.append(self.currentRangeBin)
        # print(self.trackedRangebins)

    def calculateERSB(self):
        MAX_FREQ_TO_USE = 3  # in Hz
        MIN_FREQ_TO_USE = 0.15 # in Hz
        NUM_FFT_POINTS_TO_USE = 2**8  # in selected 0-3Hz region
        # NUM_FEATURES_AROUND_HARMONICS = 7
        NUM_TOTAL_FFT_POINTS = int(NUM_FFT_POINTS_TO_USE / MAX_FREQ_TO_USE * self.fs)
        START_FFT_AT = int(NUM_TOTAL_FFT_POINTS / self.fs * MIN_FREQ_TO_USE)+1
        # if START_FFT_AT<NUM_FEATURES_AROUND_HARMONICS:
        #     print(bcolors.FAIL+'Number of features ('+str(NUM_FEATURES_AROUND_HARMONICS)+ ')greater than available FFT points (FFT starts at: '+str(START_FFT_AT)+')'+bcolors.ENDC)
        #     NUM_FEATURES_AROUND_HARMONICS=START_FFT_AT
        # print('FFT start: ', START_FFT_AT)
        self.fftFeatures = []
        # for iadd in range(-1,2):
        self.rawData = sp.signal.detrend(np.array(self.radarDataDeck)[:, self.currentMedianRangeBin], type='constant')
        if self.radarSettings['RadarType'] == 'X4':
            self.rawData = np.angle(self.rawData)
        self.rawData = self.rawData - np.mean(self.rawData)
        windowMult = np.hamming(len(self.rawData))
        self.rawData = self.rawData * windowMult

        self.fftOfRawERSB = abs(sp.fftpack.fft(self.rawData, NUM_TOTAL_FFT_POINTS))
        self.xOfFFTRAWERSB = sp.fftpack.fftfreq(NUM_TOTAL_FFT_POINTS, d=1 / self.fs)
        # print('Signal size: ' + str(self.rawData.size) + ' ,FFT size: ' + str(self.fftOfRawERSB.size) +
        #       ' ,FFTX size: ' + str(self.xOfFFTRAWERSB.size))

        self.maxOfFFT = np.amax(self.fftOfRawERSB[START_FFT_AT:NUM_FFT_POINTS_TO_USE])
        self.normalizedFFT = np.divide(self.fftOfRawERSB[START_FFT_AT:NUM_FFT_POINTS_TO_USE], self.maxOfFFT)
        self.normalizedXOfFFT = self.xOfFFTRAWERSB[START_FFT_AT:NUM_FFT_POINTS_TO_USE]
        # print('X axis...')
        # print(self.normalizedXOfFFT)
        upperCutoff = next(x[0] for x in enumerate(self.normalizedXOfFFT) if x[1] > 0.6) #60*0.6 = 36 breaths per minute
        lowerCutoff = next(x[0] for x in enumerate(self.normalizedXOfFFT) if x[1] > 0.1) #60*0.1 = 6 breaths per minute
        self.normalizedFFT[0:lowerCutoff] = 0
        self.normalizedFFT[upperCutoff:] = 0

        self.indexOfMaxFFT = np.argmax(self.normalizedFFT)
        self.freqOfMaxFFT = self.normalizedXOfFFT[self.indexOfMaxFFT]
        self.localMaximaFFTIdx = sp.signal.argrelextrema(self.normalizedFFT,np.greater)
        self.localMaximaFFT = self.normalizedXOfFFT[self.localMaximaFFTIdx]

        self.fftFeatures.extend(self.normalizedFFT)
        self.fftDeck.append(self.normalizedFFT)
        self.fftTimesDeck.append(self.endTime)
        self.fftFeaturesDeck.append(self.fftFeatures)

    def addToERSBTrainSet(self,trnClass):
        # self.ERSBFeatures.append(self.normalizedFFT)
        self.ERSBFeatures.append(self.fftFeatures)
        self.ERSBLabels.append(trnClass)
        # self.ERSBCombined.append(list(self.normalizedFFT) + [trnClass])
        self.ERSBCombined.append(list(self.fftFeatures) + [trnClass])

    def trainAndTestERSB(self):
        self.trainERSB()
        self.testERSB()

    def trainERSB(self):
        print('Training and saving ERSB')
        startTime = time.time()
        self.training_set = tf.contrib.learn.datasets.base.load_csv_with_header(
            filename=self.ERSB_TRAINING_SET_FILE_NAME,
            target_dtype=np.int,
            features_dtype=np.float32)
        # Specify that all features have real-value data
        feature_columns = [tf.contrib.layers.real_valued_column("", dimension=len(self.ERSBFeatures[0]))]
        # Build 3 layer DNN with 10, 20, 10 units respectively.
        modelDir = 'tfModels/ERTrain_2_'+self.ERSB_TRAINING_SET_FILE_NAME.replace(".csv", "")
        self.ERSBclassifier = tf.contrib.learn.DNNClassifier(feature_columns=feature_columns,
                                                    hidden_units=[10],
                                                    n_classes=2,
                                                    model_dir=modelDir)
        elapsedTime = time.time() - startTime
        print ('Elapsed %f ms' % (elapsedTime * 1000))

        # Fit model.
        self.ERSBclassifier.fit(input_fn=self.get_train_inputs, steps=2000)

    def testERSB(self):
        print('Testing ERSB')
        self.test_set = tf.contrib.learn.datasets.base.load_csv_with_header(
            filename=self.ERSB_TEST_SET_FILE_NAME,
            target_dtype=np.int,
            features_dtype=np.float32)
        # Evaluate accuracy.
        accuracy_score = self.ERSBclassifier.evaluate(input_fn=self.get_test_inputs,
                                             steps=1)["accuracy"]
        print("\nTest Accuracy: {0:f}\n".format(accuracy_score))

    def saveERSBTrainingSet(self):
        """Saves ERSB Training set"""
        print('Saving ERSB training set')
        pathTotrainingData = 'trainingData/ERTrain_2_'
        self.ERSB_TRAINING_SET_FILE_NAME = self.saveERSBFeatureSet(pathTotrainingData)
        print ('Training set saved...')

    def saveERSBTestingSet(self):
        """Saves ERSB Training set"""
        print('Saving ERSB test set')
        pathToTestData = 'trainingData/ERTest_2_'
        self.ERSB_TEST_SET_FILE_NAME = self.saveERSBFeatureSet(pathToTestData)
        print ('Test set saved...')

    def saveERSBFeatureSet(self,pathToData):
        startTime = time.time()
        timeStr = time.strftime("%Y%m%d-%H%M%S")
        csvHeader = [str(len(self.ERSBCombined)) , \
                    str(len(self.ERSBCombined[0]) - 1) , \
                     'breathing','moving']
        fileName = pathToData + timeStr + '.csv'
        with open(fileName, 'w') as csvFile:
            csvWriter = csv.writer(csvFile)
            csvWriter.writerow(csvHeader)
            for trnDataRow in self.ERSBCombined:
                csvWriter.writerow(trnDataRow)
        pathToData = pathToData+'RND_'
        fileName = pathToData + timeStr + '.csv'
        random.shuffle(self.ERSBCombined)
        with open(fileName, 'w') as csvFile:
            csvWriter = csv.writer(csvFile)
            csvWriter.writerow(csvHeader)
            for trnDataRow in self.ERSBCombined:
                csvWriter.writerow(trnDataRow)
        elapsedTime = time.time() - startTime
        print ('Elapsed %f ms' % (elapsedTime * 1000))
        return fileName

    def get_train_inputs(self):
        x = tf.constant(self.training_set.data)
        y = tf.constant(self.training_set.target)
        return x, y
        # Define the test inputs

    def get_test_inputs(self):
        x = tf.constant(self.test_set.data)
        y = tf.constant(self.test_set.target)
        return x, y

    def storeHistory(self):
        self.rangeDeck.append(self.currentRange)
        self.rangeBinDeck.append(self.currentRangeBin)

        self.breathSignalDeck.clear() # Total signal in deck is filtered and stored.
        self.breathSignalDeck.extend(self.filteredPersonBinSignal)

        self.slowTimeDeck.extend(self.currentSlowTimes) #add slow times into slow time deck
        self.currentMedianRangeBin = int(math.ceil(np.median(self.rangeBinDeck))) # find the current median range bin
        # if len(self.rangeBinDeck) > 5:
        #     self.current5sMedianRangeBin = math.ceil(np.median(np.array(self.rangeBinDeck)[-1:-6:-1])) # get last 5 range bins and calculate median. Note: This array is in reverse order because of -1 indexing
        # else:
        #     print('5 values of range bins not found')
        #     self.current5sMedianRangeBin = self.currentMedianRangeBin

        if len(self.rangeBinDeck) > self.NUM_RANGE_BIN_CHANGE_HISTORY:
            self.current5sMedianRangeBin = int(math.ceil(np.median(np.array(self.rangeBinDeck)[-1:-(self.NUM_RANGE_BIN_CHANGE_HISTORY+1):-1]))) # get last 3 range bins and calculate median. Note: This array is in reverse order because of -1 indexing
        else:
            print (self.NUM_RANGE_BIN_CHANGE_HISTORY,' values of range bins not found')
            self.current5sMedianRangeBin = int(math.ceil(self.currentMedianRangeBin))

        self.eventDict['event_type'] = 'Stationary' #Initialize this
        self.medianrangeBinDeck.append(self.currentMedianRangeBin) # append the current median range bin
        self.medianrangeDeck.append(self.currentMedianRangeBin*self.RADAR_RESOLUTION) # append the current median range
        if abs(self.currentMedianRangeBin - self.current5sMedianRangeBin)*self.RADAR_RESOLUTION > self.MOVE_DETECTION_THRESHOLD: # range has changed more than x meters in last y seconds
            print (bcolors.UNDERLINE + 'Median range changed' + bcolors.ENDC)
            self.medianRangeChanged = True
            self.numMovements += 1
            if self.numMovements > 7:
                self.numMovements = 7
            self.eventDict['event_type'] = 'Moving'
            # print (self.current5sMedianRangeBin)
            self.breathSignal5sRangeDeck.extend(sp.signal.detrend(np.array(self.currentRadarData)[:, self.current5sMedianRangeBin],type='constant'))
            # self.breathSignal5sRangeDeck.extend(sp.signal.detrend(np.array(self.currentRadarData)[:, self.current5sMedianRangeBin]))
        else:
            # print (self.currentMedianRangeBin)
            self.breathSignal5sRangeDeck.extend(sp.signal.detrend(np.array(self.currentRadarData)[:, self.currentMedianRangeBin],type='constant'))
            # self.breathSignal5sRangeDeck.extend(sp.signal.detrend(np.array(self.currentRadarData)[:, self.currentMedianRangeBin]))
            self.numMovements -= 1
            if self.numMovements < 1:
                self.medianRangeChanged = False
                self.fastMovementDetected = False
                self.numMovements = 0
                self.eventDict['event_type'] = 'Stationary'
            else:
                self.eventDict['event_type'] = 'Moving'

        if self.autocorrMovementDetected:
            self.eventDict['event_type'] = 'Moving'

        if self.previousMedianRangeBin == -10: # Initialize previous median range bin
            self.previousMedianRangeBin = self.currentRangeBin

        if abs(self.currentRangeBin - self.previousMedianRangeBin)*self.RADAR_RESOLUTION > self.FAST_MOVE_DETECTION_THRESHOLD:
            self.fastMovementCount += 1
            if self.fastMovementCount>3:
                self.fastMovementDetected = True
                self.eventDict['event_type'] = 'Fast movement'
                print (bcolors.ERROR + 'Fast Movement'+bcolors.ENDC)
        else:
            self.fastMovementCount -= 1
            if self.fastMovementCount<1:
                self.fastMovementCount=0
        self.previousMedianRangeBin = self.currentRangeBin

        if self.totalPowerSVD < self.TOTAL_ENERGY_THRESHOLD:
            self.eventDict['event_type'] = 'Vacant'

    def buildFeaturesAndTrainingSet(self,referenceRate):
        fftChanges = []
        for frequencyElem in self.imfMaxFreq:
            fftChanges.append(abs(referenceRate-frequencyElem*60))
        fftChangesNormalized = np.divide(fftChanges,referenceRate) # normalize
        totalPowerNormalized = np.divide(self.totalPowerRatio,np.amax(self.totalPowerRatio))
        pwrRatioNormalized = np.divide(self.pwrRatio,np.amax(self.pwrRatio))

        elementidx = 0
        for element in fftChangesNormalized:
            thisVector = [element, totalPowerNormalized[elementidx], pwrRatioNormalized[elementidx], self.medianRangeChanged]
            self.trainingSet.append(thisVector)
            thislabel = 0
            if (fftChanges[elementidx]<2):
                thislabel = 1 #this is a breathing IMF
            self.labelSet.append(thislabel) #add labels
            elementidx += 1
            print (bcolors.OKBLUE+'Train: '+thisVector+ ', Label: '+ str(thislabel) + bcolors.ENDC)

    def trainSVM(self,saveAs):
        print (self.trainingSet)
        print ('Labels:')
        print (self.labelSet)
        classifier = svm.SVC()
        classifier.fit(self.trainingSet,self.labelSet)
        joblib.dump(classifier, saveAs)

    def calculateEMDWTScore(self):
        emdStartTime = time.time()
        if self.radarSettings['RadarType'] == 'X4':
            unfilteredMedianBinSignal = -1*np.unwrap(
                np.angle(np.array(self.radarDataDeck)[:, self.currentMedianRangeBin]))
        else:
            unfilteredMedianBinSignal = np.abs(np.array(self.radarDataDeck)[:, self.currentMedianRangeBin])
        unfilteredMedianBinSignal = unfilteredMedianBinSignal - np.mean(unfilteredMedianBinSignal)
        self.capturedBreathingSignal = unfilteredMedianBinSignal
        # print('BreathSingal: '+str(self.capturedBreathingSignal))

        #USE WINDOWING
        # unfilteredMedianBinSignal = unfilteredMedianBinSignal - np.mean(unfilteredMedianBinSignal)
        # windowMult = np.hamming(len(unfilteredMedianBinSignal))
        # unfilteredMedianBinSignal = unfilteredMedianBinSignal * windowMult
        #END USE WINDOWING

        self.imfs = pyeemd.ceemdan(unfilteredMedianBinSignal)
        # self.imfs = pyeemd.emd(unfilteredMedianBinSignal)
        emdElapsed1 = time.time() - emdStartTime
        print("EMD elapsed1: "+str(emdElapsed1))

        nForFFT = 2 ** 16

        WAVELET_TYPE = 'sym6'
        # wavelet = pywt.Wavelet(WAVELET_TYPE)
        # maxdecomp = pywt.dwt_max_level(len(unfilteredMedianBinSignal),wavelet.dec_len)
        # self.wtcoeffs = pywt.wavedec(unfilteredMedianBinSignal,WAVELET_TYPE,level=maxdecomp)
        wtcoeffsRAW = pywt.dwt(unfilteredMedianBinSignal,WAVELET_TYPE)
        self.wtcoeffs = []
        for wtcs in wtcoeffsRAW:
            self.wtcoeffs.append(sp.signal.resample(wtcs,len(self.slowTimeDeck)))
        self.wtFFTs = []
        self.wtFFTMaxFreq = []
        for wts in self.wtcoeffs:
            yf = sp.fftpack.fft(wts,nForFFT)
            xf = sp.fftpack.fftfreq(nForFFT,1/self.fs)
            xf = xf[0:len(xf)//2]
            ffty = 2.0/len(wts)*np.abs(yf[0:len(yf)//2])
            self.wtFFTs.append([xf,ffty])
            wtsMaxIndex = np.argmax(ffty)  # Find the index of the maximum freq component
            self.wtFFTMaxFreq.append(xf[wtsMaxIndex])  # Find the frequency corresponding to the selected index

        #Total power
        yfTotal = sp.fftpack.fft(unfilteredMedianBinSignal,nForFFT)
        # xfTotal = np.linspace(0.0, 1.0 / 2.0 * self.fs, len(unfilteredMedianBinSignal) // 2)
        xfTotal = sp.fftpack.fftfreq(nForFFT,1/self.fs)
        fftyTotal = 2.0 / len(unfilteredMedianBinSignal) * np.abs(yfTotal[0:len(yfTotal) // 2])
        psdTotal = np.abs(fftyTotal) ** 2  # Calculate Power
        totalPower = np.sum(psdTotal)

        self.imfs = self.imfs[0:-1] #remove residual from IMFs

        psdLowNumerator0 = 0.17 # starting frequency for PSD ratio Numerator
        psdLowNumerator1 = 1 # End frequency for PSD ratio Numerator
        psdLowDenominator0 = 1 # Starting frequency for PSD ratio denominator
        psdLowDenominator1 = 5 # End frequency for PSD ratio denominator

        IMF_SIMILARITY_FREQ = 0.07 #If adjacent IMFs are less than this freq apart, combine them

        #Scoring parameters
        PSD_TOTAL_WEIGHT = 1.1
        PSD_RATIO_WEIGHT = 0.0
        FFTMAX_DIFF_WEIGHT = -2
        WT_CORR_WEIGHT = 0.5
        DIFF_WITH_MEDIAN_WEIGHT = -1
        MINK_DIST_WEIGHT = -1

        weightsArray = np.array([PSD_TOTAL_WEIGHT,
                                 PSD_RATIO_WEIGHT,
                                 FFTMAX_DIFF_WEIGHT,
                                 WT_CORR_WEIGHT,
                                 DIFF_WITH_MEDIAN_WEIGHT,
                                 MINK_DIST_WEIGHT])

        self.emdTimes = list(self.slowTimeDeck)
        self.imfFFTs = []
        self.imfMaxFreq = []
        self.totalPowerRatio = []
        self.pwrRatio = []
        self.imfHSAs = []
        self.imfHSAMeanFreq = []
        self.imfCorrelation = []
        self.imfScores = []
        self.imfScoreValues = []
        #Combine multiples

        COMBINE_SIMILAR_IMFS = True

        #Combine similar IMFs
        #Calculate FFT and Mink Dist
        self.minkowskiDistances = []
        for imf in self.imfs:
            yf = sp.fftpack.fft(imf, n=nForFFT)
            # xf = np.linspace(0.0,1.0/2.0*self.fs,nForFFT//2)
            xf = sp.fftpack.fftfreq(n=nForFFT, d=1 / self.fs)
            xf = xf[0:len(xf) // 2]
            ffty = 2.0 / len(imf) * np.abs(yf[0:len(yf) // 2])
            imfMaxIndex = np.argmax(ffty)  # Find the index of the maximum freq component
            thisIMFMaxFreq = xf[imfMaxIndex]
            self.imfMaxFreq.append(thisIMFMaxFreq)
            thisMinkDist = sp.spatial.distance.minkowski(imf, unfilteredMedianBinSignal, 2)
            self.minkowskiDistances.append(thisMinkDist)
        minkMinIdx = np.argmin(self.minkowskiDistances)
        self.EMDMinkBRate = self.imfMaxFreq[minkMinIdx]*60


        if COMBINE_SIMILAR_IMFS:
            tempIMFs = []
            imfIdx = 0
            while imfIdx < len(self.imfs)-1:
                if abs(self.imfMaxFreq[imfIdx] - self.imfMaxFreq[imfIdx+1]) <= IMF_SIMILARITY_FREQ:
                    combinedIMF = self.imfs[imfIdx]+self.imfs[imfIdx+1]
                    if imfIdx < len(self.imfs)-2 and \
                                    abs(self.imfMaxFreq[imfIdx+1] - self.imfMaxFreq[imfIdx+2]) <= IMF_SIMILARITY_FREQ:
                        combinedIMF += self.imfs[imfIdx+2]
                        imfIdx+=1
                    tempIMFs.append(combinedIMF)
                    imfIdx+=1
                else:
                    tempIMFs.append(self.imfs[imfIdx])
                imfIdx+=1
            self.imfs = tempIMFs

        self.imfFFTs = []
        self.imfMaxFreq = []
        self.minkowskiDistancesCombinedIMFs = []
        for imf in self.imfs:
            # yf = fft(imf)
            thisIMFScore = 0

            yf = sp.fftpack.fft(imf,n=nForFFT)

            hsIMF = sp.signal.hilbert(imf)
            omegaIMF = sp.unwrap(sp.angle(hsIMF))
            fInstIMF = np.abs(np.diff(omegaIMF))
            meanFreq = np.mean(fInstIMF)
            # meanFreq = np.median(fInstIMF)

            self.imfHSAs.append([self.emdTimes[0:-1],fInstIMF])
            self.imfHSAMeanFreq.append(meanFreq)

            # xf = np.linspace(0.0,1.0/2.0*self.fs,nForFFT//2)
            xf = sp.fftpack.fftfreq(n=nForFFT,d=1/self.fs)
            xf = xf[0:len(xf)//2]
            ffty = 2.0/len(imf)*np.abs(yf[0:len(yf)//2])
            psdIMF = np.abs(ffty)**2 # Calculate Power

            lowerCutoff = next(x[0] for x in enumerate(xf) if x[1] > psdLowNumerator0)
            upperCutoff = next(x[0] for x in enumerate(xf) if x[1] > psdLowNumerator1)
            psdSumNumerator = np.sum(psdIMF[lowerCutoff:upperCutoff])

            lowerCutoff = next(x[0] for x in enumerate(xf) if x[1] > psdLowDenominator0)
            upperCutoff = next(x[0] for x in enumerate(xf) if x[1] > psdLowDenominator1)
            psdSumDenominator = np.sum(psdIMF[lowerCutoff:upperCutoff])

            self.imfFFTs.append([xf,ffty])
            imfMaxIndex = np.argmax(ffty) #Find the index of the maximum freq component
            thisIMFMaxFreq = xf[imfMaxIndex]
            self.imfMaxFreq.append(thisIMFMaxFreq) #Find the frequency corresponding to the selected index

            WTIMFcorrelation = sp.stats.stats.pearsonr(imf, self.wtcoeffs[0])
            self.imfCorrelation.append(WTIMFcorrelation)

            psdTotalRatio = np.sum(psdIMF)/totalPower
            psdRatio = psdSumNumerator/psdSumDenominator
            self.totalPowerRatio.append(psdTotalRatio)
            self.pwrRatio.append(psdRatio)

            if len(self.currentBreathRates)>1:
                diffwithCurrMedian = abs(np.median(self.currentBreathRates)/60 - thisIMFMaxFreq)
            else:
                diffwithCurrMedian = 0

            thisMinkDist = sp.spatial.distance.minkowski(imf, unfilteredMedianBinSignal, 2)
            self.minkowskiDistancesCombinedIMFs.append(thisMinkDist)

            fftmaxDiff = abs(self.freqOfMaxFFT - thisIMFMaxFreq)
            thisIMFScoreValues = [psdTotalRatio,psdRatio,fftmaxDiff,WTIMFcorrelation[0],diffwithCurrMedian,thisMinkDist]
            self.imfScoreValues.append(thisIMFScoreValues)

            # print ('MaxFreq: ' + "{:.2f}".format(xf[imfMaxIndex]*60) +\
            #       ', TotalPowRatio: '+"{:.3f}".format(np.sum(psdIMF)/totalPower) + ', Power ratio: ' + "{:.3f}".format(psdSumNumerator/psdSumDenominator))
        self.imfScoreValues = normalize(self.imfScoreValues,axis=0,norm='max')
        self.imfScores = weightsArray*self.imfScoreValues
        self.imfScoreRowSums = np.sum(self.imfScores,axis=1)
        self.scoreMaxIdx = np.argmax(self.imfScoreRowSums)

        self.newestBreathRate = self.imfMaxFreq[self.scoreMaxIdx]*60
        self.emdCalculated = True

        if self.emdCalculated:
            print (bcolors.OKGREEN + 'Breath rate: ' + str(self.newestBreathRate) + bcolors.ENDC)
            if self.eventDict['event_type'] != 'Stationary' and len(self.emdBreathRates) > 0: #if person is moving, don't add to breathing history
                removedVal = self.emdBreathRates.popleft()
            else:
                self.emdBreathRates.append(self.newestBreathRate)
            self.emdIMFNotFoundCount = 0
        else:
            print (bcolors.ERROR+'Breathing IMF not found'+bcolors.ENDC)
            self.emdIMFNotFoundCount += 1
            if len(self.emdBreathRates) > 0:
                removedVal = self.emdBreathRates.popleft()
        # self.EMDBreathSignal = self.imfs[totpwMaxIndex]
        self.EMDBreathSignal = self.imfs[self.scoreMaxIdx]
        emdElapsed = time.time() - emdStartTime
        if emdElapsed > 0.3:
            print (bcolors.WARNING+'EMD elapsed: '+str(emdElapsed)+bcolors.ENDC)
        # print (bcolors.WARNING + 'EMD elapsed: ' + str(emdElapsed) + bcolors.ENDC)
        # print(self.minkowskyDist)

        self.emdCalculated = True

    def calculateEMD(self):
        emdStartTime = time.time()
        # self.imfs = pyeemd.ceemdan(self.breathSignalDeck)
        # self.imfs = pyeemd.ceemdan(self.breathSignal5sRangeDeck)
        if self.radarSettings['RadarType'] == 'X4':
            unfilteredMedianBinSignal = np.angle(np.array(self.radarDataDeck)[:, self.currentMedianRangeBin])
        else:
            unfilteredMedianBinSignal = np.array(self.radarDataDeck)[:, self.currentMedianRangeBin]
        # if self.nextCorrValues[self.currentMedianRangeBin]>0.8:
        #     print("ADDING NEXT BIN, CORR: "+str(self.nextCorrValues[self.currentMedianRangeBin]))
        #     unfilteredMedianBinSignal += np.angle(np.array(self.radarDataDeck)[:, self.currentMedianRangeBin+1])
        # if self.prevCorrValues[self.currentMedianRangeBin]>0.8:
        #     print("ADDING Previous BIN, CORR: "+str(self.prevCorrValues[self.currentMedianRangeBin]))
        #     unfilteredMedianBinSignal += np.angle(np.array(self.radarDataDeck)[:, self.currentMedianRangeBin - 1])
        # unfilteredMedianBinSignal = np.array(self.radarDataDeck)[:, self.currentMedianRangeBin]
        # unfilteredMedianBinSignal = self.breathSignalDeck # TEMP ################
        self.capturedBreathingSignal = unfilteredMedianBinSignal

        #USE WINDOWING
        # unfilteredMedianBinSignal = unfilteredMedianBinSignal - np.mean(unfilteredMedianBinSignal)
        # windowMult = np.hamming(len(unfilteredMedianBinSignal))
        # unfilteredMedianBinSignal = unfilteredMedianBinSignal * windowMult
        #END USE WINDOWING

        self.imfs = pyeemd.ceemdan(unfilteredMedianBinSignal)

        #Total power
        yfTotal = sp.fftpack.fft(unfilteredMedianBinSignal)
        xfTotal = np.linspace(0.0, 1.0 / 2.0 * self.fs, len(unfilteredMedianBinSignal) // 2)
        fftyTotal = 2.0 / len(unfilteredMedianBinSignal) * np.abs(yfTotal[0:len(unfilteredMedianBinSignal) // 2])
        psdTotal = np.abs(fftyTotal) ** 2  # Calculate Power
        totalPower = np.sum(psdTotal)

        self.imfs = self.imfs[0:-1] #remove residual from IMFs

        psdLowNumerator0 = 0.17 # starting frequency for PSD ratio Numerator
        psdLowNumerator1 = 1 # End frequency for PSD ratio Numerator
        psdLowDenominator0 = 1 # Starting frequency for PSD ratio denominator
        psdLowDenominator1 = 5 # End frequency for PSD ratio denominator

        self.emdTimes = list(self.slowTimeDeck)
        self.imfFFTs = []
        self.imfMaxFreq = []
        self.totalPowerRatio = []
        self.pwrRatio = []
        self.imfHSAs = []
        self.imfHSAMeanFreq = []
        for imf in self.imfs:
            # yf = fft(imf)
            yf = sp.fftpack.fft(imf)

            hsIMF = sp.signal.hilbert(imf)
            omegaIMF = sp.unwrap(sp.angle(hsIMF))
            fInstIMF = np.abs(np.diff(omegaIMF))
            # meanFreq = np.mean(fInstIMF)
            meanFreq = np.median(fInstIMF)

            # roundedFreq = [round(elem,2) for elem in fInstIMF]
            # datafreq = Counter(roundedFreq)
            # meanFreq = datafreq.most_common(1)

            self.imfHSAs.append([self.emdTimes[0:-1],fInstIMF])
            self.imfHSAMeanFreq.append(meanFreq)

            xf = np.linspace(0.0,1.0/2.0*self.fs,len(imf)//2)
            ffty = 2.0/len(imf)*np.abs(yf[0:len(imf)//2])
            psdIMF = np.abs(ffty)**2 # Calculate Power
            psdSumNumerator = np.sum(psdIMF[int(psdLowNumerator0*self.fs//2):\
                                        int(psdLowNumerator1*self.fs//2)])
            psdSumDenominator = np.sum(psdIMF[int(psdLowDenominator0*self.fs//2):\
                                        int(psdLowDenominator1*self.fs//2)])
            # print("PSD Num: "+ str(psdSumNumerator) +
            #       ", Den: " + str(psdSumDenominator) +
            #       ", Ratio: "+ str(psdSumNumerator/psdSumDenominator))
            self.imfFFTs.append([xf,ffty])
            imfMaxIndex = np.argmax(ffty) #Find the index of the maximum freq component
            self.imfMaxFreq.append(xf[imfMaxIndex]) #Find the frequency corresponding to the selected index

            self.totalPowerRatio.append(np.sum(psdIMF)/totalPower)
            self.pwrRatio.append(psdSumNumerator/psdSumDenominator)

            # print ('MaxFreq: ' + "{:.2f}".format(xf[imfMaxIndex]*60) +\
            #       ', TotalPowRatio: '+"{:.3f}".format(np.sum(psdIMF)/totalPower) + ', Power ratio: ' + "{:.3f}".format(psdSumNumerator/psdSumDenominator))
        totpwMaxIndex = np.argmax(self.totalPowerRatio)
        self.newestBreathRate = self.imfMaxFreq[totpwMaxIndex]*60
        sortedTotPw = sorted(self.totalPowerRatio, reverse=True)
        sortedIndex = 1
        self.emdCalculated = False
        if len(self.emdBreathRates)>1:
            while self.emdCalculated == False:
                if self.pwrRatio[totpwMaxIndex] < 0.5: # or \
                        # (self.emdIMFNotFoundCount < 4 and\
                                     # abs(self.newestBreathRate - np.median(self.emdBreathRates)) > 5): #Check if breath rate changed by more than 5
                    totpwMaxIndex = self.totalPowerRatio.index(sortedTotPw[sortedIndex])
                    self.newestBreathRate = self.imfMaxFreq[totpwMaxIndex] * 60
                    sortedIndex += 1
                    if sortedIndex == len(self.totalPowerRatio):
                        break
                else:
                    self.emdCalculated = True
        else:
            while self.emdCalculated == False:
                if self.pwrRatio[totpwMaxIndex] < 1.3:
                    totpwMaxIndex = self.totalPowerRatio.index(sortedTotPw[sortedIndex])
                    self.newestBreathRate = self.imfMaxFreq[totpwMaxIndex] * 60
                    sortedIndex += 1
                    if sortedIndex == len(self.totalPowerRatio):
                        break
                else:
                    self.emdCalculated = True
        if self.emdCalculated:
            print (bcolors.OKGREEN + 'Breath rate: ' + str(self.newestBreathRate) + bcolors.ENDC)
            if self.medianRangeChanged and len(self.emdBreathRates) > 0: #if person is moving, don't add to breathing history
                removedVal = self.emdBreathRates.popleft()
            else:
                self.emdBreathRates.append(self.newestBreathRate)
            self.emdIMFNotFoundCount = 0
        else:
            print (bcolors.ERROR+'Breathing IMF not found'+bcolors.ENDC)
            self.emdIMFNotFoundCount += 1
            if len(self.emdBreathRates) > 0:
                removedVal = self.emdBreathRates.popleft()
        self.EMDBreathSignal = self.imfs[totpwMaxIndex]
        emdElapsed = time.time() - emdStartTime
        if emdElapsed > 0.3:
            print (bcolors.WARNING+'EMD elapsed: '+str(emdElapsed)+bcolors.ENDC)
        # print (bcolors.WARNING + 'EMD elapsed: ' + str(emdElapsed) + bcolors.ENDC)
        # print(self.minkowskyDist)

        self.emdCalculated = True

    def sendDataToPlot(self,iteration):
        inputDict = {}
        plotstartTime = time.time()
        fignum = 1

        inputDict['figNum'] = fignum
        fignum+=1
        inputDict['figTitle'] = 'Breath signal'
        inputDict['horzplots'] = 1
        inputDict['vertplots'] = 5

        inputDict['plotType'] = ['singlexy', \
                                 'singlexy', \
                                 'singlexy', \
                                 'singlexy', \
                                 'multiplexyy']

        unfilteredMedianBinSignal = np.array(self.radarDataDeck)[:, self.currentMedianRangeBin]
        # unfilteredMedianBinPhase = np.gradient(np.unwrap(np.angle(unfilteredMedianBinSignal)))
        unfilteredMedianBinPhase = np.unwrap(np.angle(unfilteredMedianBinSignal))

        timeAxis = np.linspace(self.slowTimeDeck[0], self.slowTimeDeck[-1], len(self.rangeDeck))
        # inputDict['figxy'] = [[list(self.slowTimeDeck),list(self.breathSignal5sRangeDeck)],\
        inputDict['figxy'] = [[list(self.referenceSignalTime),list(self.referenceSignal)], \
                              [list(self.slowTimeDeck), list(unfilteredMedianBinSignal)],\
                              # [list(self.slowTimeDeck), list(self.breathSignalDeck)],\
                              # [list(self.emdTimes),list(self.EMDBreathSignal)],\
                              # [list(self.slowTimeDeck),list(unfilteredMedianBinPhase)], \
                              [list(self.slowTimeDeck),list(self.capturedBreathingSignal)], \
                              [list(self.emdTimes), list(self.EMDBreathSignal)],\
                              [list(timeAxis), list(self.rangeDeck), list(self.medianrangeDeck)]]

        inputDict['ycolors'] = [[], \
                                [], \
                                [], \
                                [], \
                                ['','r', 'g']]
        inputDict['ylabels'] = [[],\
                                [],\
                                [], \
                                [], \
                                ['','Range','Median range']]

        inputDict['figxylabels'] = [['time(ms), Reference signal', 'Amp(V)'], \
                                    ['time(ms), radar signal amplitude', 'Amp(V)'], \
                                    #['time (ms) Filtered median bin', 'Amplitude (V)'], \
                                    ['time (ms),radar signal phase','Ph'],\
                                    ['time (ms), Reconstructed breathing signal','Amp'],\
                                    ['time (ms), range and Median range', 'Rng(m)']]
        if self.fastMovementDetected:
            brTextColor = 'red'
        elif self.emdIMFNotFoundCount > 0:
            brTextColor = 'orange'
        elif self.medianRangeChanged:
            brTextColor = 'magenta'
        elif self.autocorrMovementDetected:
            brTextColor = 'blue'
        else:
            brTextColor = 'green'
        if self.autocorrMovementDetected:
            autocorrText = 'Subject is Moving'
        else:
            autocorrText = 'Subject is Stationary'
        breathRateText = 'Median Breath Rate : ' + u"{:.2f}".format(np.median(self.emdBreathRates)) + "Br/m"
        brMinkCompareText = 'Br.Rate: '+u"{:.2f}".format(self.newestBreathRate)+'\n'+\
                            'Mink. Rate: '+ u"{:.2f}".format(self.EMDMinkBRate)
        inputDict['subplotText'] = ['Ref.Rate:'+u"{:.2f}".format(self.referenceRate), \
                                    'Curent Breath Rate: '+u"{:.2f}".format(self.newestBreathRate)+'Br/m', \
                                     breathRateText, \
                                     brMinkCompareText,\
                                    # 'Phase:(1st elem)'+u"{:.2f}".format(unfilteredMedianBinPhase[0]),\
                                    autocorrText]
        inputDict['subplotTextColor'] = ['green', \
                                         'turquoise', \
                                         brTextColor, \
                                         'green',\
                                         'orange']

        self.plotterInputQ.put(inputDict)

        #Range plots
        inputDict = {}

        inputDict['figNum'] = fignum
        fignum+=1
        inputDict['figTitle'] = 'Range'
        inputDict['horzplots'] = 1
        inputDict['vertplots'] = 3

        inputDict['plotType'] = ['singlexy',
                                 'singlexy',
                                 'singley']

        inputDict['figxy'] = [[np.arange(0,len(self.varianceOfAllBins)*self.RADAR_RESOLUTION,self.RADAR_RESOLUTION),self.varianceOfAllBins],
                              # [list(self.xOfFFTRAWERSB), list(self.fftOfRawERSB)],
                              # [list(self.autocorrelationMainLobeTimesDeck)],
                              [list(self.normalizedXOfFFT),list(self.normalizedFFT)],
                              [list(self.binAutoCorrelation)]]
        # print(self.varianceOfAllBins)
        inputDict['ycolors'] = [[]]
        inputDict['ylabels'] = [[]]

        inputDict['figxylabels'] = [['Range(m)', 'Var'],
                                    ['Frequency (Hz)','Amp'],
                                    ['Autocorrelation','']]
        inputDict['subplotText'] = ['','MaxFFT:'+u"{:.2f}".format(self.freqOfMaxFFT*60)+' Br/m','']
        inputDict['subplotTextColor'] = ['green','green','green']
        self.plotterInputQ.put(inputDict)

        # Single plot
        # inputDict = {}
        #
        # inputDict['figNum'] = fignum
        # fignum+=1
        # inputDict['figTitle'] = 'Autocorrelation'
        # inputDict['horzplots'] = 1
        # inputDict['vertplots'] = 1
        #
        # inputDict['plotType'] = ['singlexy']
        # aclen = len(self.binAutoCorrelation)
        # inputDict['figxy'] = [[np.linspace(int(-aclen/2),int(aclen/2),aclen),list(self.binAutoCorrelation)]]
        # # print(self.varianceOfAllBins)
        # inputDict['ycolors'] = [[]]
        # inputDict['ylabels'] = [[]]
        #
        # inputDict['figxylabels'] = [['Lag ', 'Normalized value']]
        # inputDict['subplotText'] = ['Main lobe width: '+u"{:.2f}".format(self.autoCorrelationMainLobeWidthTime)+' s'+'ZCWidth: '+u"{:.2f}".format(self.autoCorrBRate)]
        # inputDict['subplotTextColor'] = ['green']
        # self.plotterInputQ.put(inputDict)

        if iteration%self.PLOT_EMD_EVERY_X_ITER == 0:
            inputDict = {}
            inputDict['figNum'] = fignum
            fignum+=1
            inputDict['figTitle'] = 'EMD'
            # inputDict['horzplots'] = 2
            inputDict['horzplots'] = 3
            inputDict['vertplots'] = len(self.imfs)
            inputDict['figxy'] = []
            inputDict['plotType'] = []
            # inputDict['figxylabels'] = []
            inputDict['subplotText'] = []
            inputDict['subplotTextColor'] = []
            for imfindex in range(len(self.imfs)):
                inputDict['figxy'].append([self.emdTimes,self.imfs[imfindex]])
                inputDict['plotType'].append(u'singlexy')

                inputDict['figxy'].append(self.imfFFTs[imfindex])
                inputDict['plotType'].append(u'singlexy')

                # YLabel1 = self.imfMaxFreq[imfindex] * 60
                subplotText1 = ''
                for scores in self.imfScoreValues[imfindex]:
                    subplotText1 += "{:.2f}".format(scores)
                    subplotText1 += ', '
                subplotText1+='\n'
                for scores in self.imfScores[imfindex]:
                    subplotText1 += "{:.2f}".format(scores)
                    subplotText1 += ', '
                subplotText1+= 'Total: '+"{:.2f}".format(self.imfScoreRowSums[imfindex])

                YLabel1 = 'Rate: '+"{:.2f}".format(self.imfMaxFreq[imfindex] * 60)+' Br/m'+'\n'+\
                            'Mink Dist: '+"{:.2f}".format(self.minkowskiDistancesCombinedIMFs[imfindex])
                               # "{:.2f}".format(self.imfCorrelation[imfindex][0])+','\
                               #  "{:.2f}".format(self.imfCorrelation[imfindex][1])
                # YLabel2 = "{:.2f}".format(self.imfHSAMeanFreq[imfindex] * 60)
                YLabel2 = ''

                inputDict['figxy'].append(self.imfHSAs[imfindex])
                inputDict['plotType'].append(u'singlexy')

                # inputDict['subplotText'].extend(['',YLabel])
                # inputDict['subplotTextColor'].extend([brTextColor,brTextColor])
                inputDict['subplotText'].extend([subplotText1,YLabel1,YLabel2])
                inputDict['subplotTextColor'].extend([brTextColor,brTextColor,brTextColor])
            self.plotterInputQ.put(inputDict)

        #Wavelet plot
        inputDict = {}
        inputDict['figNum'] = fignum
        fignum+=1
        inputDict['figTitle'] = 'Wavelets'
        inputDict['horzplots'] = 2
        inputDict['vertplots'] = len(self.wtcoeffs)
        inputDict['figxy'] = []
        inputDict['plotType'] = []
        inputDict['figxylabels'] = []
        inputDict['subplotText'] = []
        inputDict['subplotTextColor'] = []
        for wtindex in range(len(self.wtcoeffs)):
            inputDict['figxy'].append([self.emdTimes,self.wtcoeffs[wtindex]])
            inputDict['plotType'].append(u'singlexy')

            inputDict['figxy'].append(self.wtFFTs[wtindex])
            inputDict['plotType'].append(u'singlexy')

            YLabel1 = 'Rate: '+"{:.2f}".format(self.wtFFTMaxFreq[wtindex]*60)+' Br/m'
            inputDict['subplotText'].extend(['', YLabel1])
            inputDict['subplotTextColor'].extend([brTextColor, brTextColor])

            inputDict['figxylabels'].extend([['Time(ms)', 'Amp'],
                                            ['Frequency(Hz)', 'Amp']])

        self.plotterInputQ.put(inputDict)

        plotElapsed = time.time() - plotstartTime
        if plotElapsed > 0.1:
            print (bcolors.WARNING + 'Plotter send Elapsed: ' + str(plotElapsed) + bcolors.ENDC)

    def normalizeRadarData(self,modifySelf = False,radarData = None):
        """
        Normalizes the radar data.
        :param radarData: 2D array of radarData. If Null, the self.currentData will be used as input
        :param modifySelf: If True, self.currentData will be modified. If False, normalized Data will be returned as a 2D array
        :return: if modifySelf is true, return None. Else, return normalized radar data and slow times
        """

        MAXVOLTS = 1.04
        NORMALDACCOUNT_MAXRANGE = 8191

        if radarData is None:
            radarData = self.currentData

        radarData = np.array(radarData)
        _currentRadarData = np.copy(radarData[:,1:]) #extract radar data without timestamp
        _currentSlowTimes = np.copy(radarData[:,0]) #extract timestamps

        if self.NORMALIZE_RADAR_DATA:
            _currentRadarData = np.multiply(_currentRadarData,
                                     ((self.radarSettings['DACStep'] / (self.radarSettings['PulsesPerStep']
                                                                        * self.radarSettings[
                                                                            'Iterations'])) * MAXVOLTS / NORMALDACCOUNT_MAXRANGE))

        for index in range(0, len(_currentRadarData[:, 0]), 1):
            _currentRadarData[index, :] = _currentRadarData[index, :] - np.mean(_currentRadarData[index, :])

        if modifySelf == True:
            self.currentRadarData = _currentRadarData
            self.currentSlowTimes = _currentSlowTimes
            return None
        else:
            return [_currentRadarData,_currentSlowTimes]

    def findRange(self,radarData,fs, modifySelf = False):
        """
        Find range bin from radarData
        :param radarData: Radar data without timestamps
        :param fs: Sampling frequency (Slow time)
        :return: currentRange and personBin if modifySelf = False. Otherwise return None
        """
        USE_TRANSPOSED_SVD = False
        USE_VARIANCE_FOR_RANGE = True
        USE_SVD_CLUTTER_REMOVAL = False

        REMOVE_CLOSE_BINS = True
        REMOVAL_THRESHOLD = math.ceil(1.0/self.RADAR_RESOLUTION) #Remove 1.0m from radar

        REMOVE_FAR_OUT_BINS = True
        FAR_REMOVAL_THRESHOLD = math.ceil(9.0/self.RADAR_RESOLUTION) #Remove data after 9.0m
        # print(type(radarData))
        # print(radarData)
        # print (type(radarData))
        # print('0 to Removal threshold: '+str(REMOVAL_THRESHOLD))
        # print(radarData[0][0:int(REMOVAL_THRESHOLD)])
        if REMOVE_CLOSE_BINS:
            for dataRow in radarData:
                dataRow[0:int(REMOVAL_THRESHOLD)] = [0]*int(REMOVAL_THRESHOLD)
        if REMOVE_FAR_OUT_BINS:
            for dataRow in radarData:
                dataRow[int(FAR_REMOVAL_THRESHOLD):] = [0]*int(len(dataRow) - FAR_REMOVAL_THRESHOLD)

        rawscansV = np.transpose(radarData)
        m, n = rawscansV.shape
        if USE_SVD_CLUTTER_REMOVAL:
            if USE_TRANSPOSED_SVD:
                U, s, V = sp.linalg.svd(np.transpose(rawscansV))
                S = sp.linalg.diagsvd(s, U.shape[0], V.shape[0])
                S[0, 0] = 0
                _SVDFiltered = np.transpose(U.dot(S).dot(np.transpose(V)))
            else:
                U, s, V = sp.linalg.svd(rawscansV)
                S = sp.linalg.diagsvd(s, U.shape[0], V.shape[0])
                S[0, 0] = 0
                _SVDFiltered = U.dot(S).dot(np.transpose(V))
        else:
            _SVDFiltered = rawscansV

        # Range finding
        if USE_VARIANCE_FOR_RANGE:
            # The bin with maximum variance is used for identifying the range
            self.varianceOfAllBins = np.var(np.transpose(_SVDFiltered), 0, ddof=1)
            personBin = np.argmax(self.varianceOfAllBins)
            # personBin = personBin - 1 #personbin is found from a matrix with timestamps as first column#This is fixed now
            _currentRange = personBin*self.RADAR_RESOLUTION
        else:
            # person range changes every slow time sample due to movement of chest
            # while breathing. The index of the bin with the maximum amplitude is
            # identified and the mean of the bins is taken as the bin where the person
            # is located.
            high_filtered = np.transpose(_SVDFiltered)
            maxIndices = np.zeros(len(high_filtered[:, 0]))
            for i in range(0, len(high_filtered[:, 0])):
                maxIndices[i] = np.argmax(high_filtered[i, :])
            # mean of person location. this bin will be used to take the breathing
            # signal
            personBin = np.ceil(np.mean(maxIndices))
            # personBin = personBin - 1  # personbin is found from a matrix with timestamps as first column#This is fixed now
            _currentRange = personBin*self.RADAR_RESOLUTION

        # self.binAutoCorrelation = np.correlate(np.absolute(np.array(self.radarDataDeck)[:,personBin]),np.absolute(np.array(self.radarDataDeck)[:,personBin]),mode='full')
        if self.radarSettings['RadarType'] == 'X4':
            autocorrSignal = np.angle(np.array(self.radarDataDeck)[:, personBin])
        else:
            autocorrSignal = np.abs(np.array(self.radarDataDeck)[:, personBin])
        autocorrSignal = autocorrSignal - np.mean(autocorrSignal)
        autocorrSignalHalf = autocorrSignal[int(len(autocorrSignal)/2):]
        # print(autocorrSignalHalf)
        # print(type(autocorrSignalHalf))
        zero_crossings = np.where(np.diff(np.signbit(autocorrSignalHalf)))[0]
        # print(zero_crossings)
        # self.autoCorrBRate = self.fs/(zero_crossings[0]*4)*60
        # print('BRATE:###############################: ',self.autoCorrBRate)

        self.binAutoCorrelation = np.correlate(autocorrSignal,autocorrSignal, mode='full')
        self.binAutoCorrelation /= self.binAutoCorrelation[self.binAutoCorrelation.argmax()]
        # self.binAutoCorrelation = self.binAutoCorrelation[int(self.binAutoCorrelation.size / 2):]

        # self.autoCorrelationMinimaIndices = sp.signal.argrelextrema(self.binAutoCorrelation[int(self.binAutoCorrelation.size / 2):], np.less)
        self.autoCorrelationMinimaIndices = sp.signal.argrelextrema(self.binAutoCorrelation, np.less)
        self.autoCorrelationMinimaIndices = self.autoCorrelationMinimaIndices[0] #Select the first element of the tuple (Which is an array of local minima indices

        # print (self.autoCorrelationMinimaIndices)
        self.autoCorrelationMinimaValues = np.array(self.binAutoCorrelation[self.autoCorrelationMinimaIndices])
        # print(self.autoCorrelationMinimaValues)
        if len(self.autoCorrelationMinimaIndices)>1:
            self.autocorrelationMainLobeWidth = self.autoCorrelationMinimaIndices[int(len(self.autoCorrelationMinimaIndices) / 2)] - \
                                                self.autoCorrelationMinimaIndices[int(len(self.autoCorrelationMinimaIndices) / 2) - 1]
            print ('Main lobe width: ' + str(self.autocorrelationMainLobeWidth))
            self.autoCorrelationMainLobeWidthTime = self.autocorrelationMainLobeWidth / (self.fs * 2)
            print(bcolors.OKBLUE + 'Main lobe time: ' + str(self.autoCorrelationMainLobeWidthTime) + bcolors.ENDC)
            self.autocorrelationMainLobeTimesDeck.append(self.autoCorrelationMainLobeWidthTime)
            if self.autoCorrelationMainLobeWidthTime>self.AUTOCORR_MOVEMENT_TIME_THRESHOLD:
                self.autocorrMovementDetected = False
            else:
                self.autocorrMovementDetected = True
        else:
            print ('AutoCorr indices empty')
            self.autocorrMovementDetected = False
        # if(len(self.autoCorrelationMinimaValues)>0):
        #     plt.cla()
        #     plt.plot(self.binAutoCorrelation,'-g')
        #     plt.plot(self.autoCorrelationMinimaIndices,self.autoCorrelationMinimaValues,'rD')
        #     plt.pause(0.1)
        # else:
        #     plt.cla()
        #     plt.plot(self.binAutoCorrelation, '-g')
        #     plt.pause(0.1)

        if self.radarSettings['RadarType'] == 'X4':
            radarAsDF = pd.DataFrame(np.angle(np.array(self.radarDataDeck)))
        else:
            radarAsDF = pd.DataFrame(np.array(self.radarDataDeck))
        correlationDF = radarAsDF.corr()
        self.nextCorrValues = []
        self.prevCorrValues = []
        self.sumCorrValues = []
        for corrdfindex,corrdfrow in correlationDF.iterrows():
            # print(corrdfrow[corrdfindex])
            corrNext = 0
            corrPrev = 0
            length = len(correlationDF[0])
            if corrdfindex > 0:
                corrPrev = corrdfrow[corrdfindex-1]
            if corrdfindex < len(correlationDF[0])-1:
                corrNext = corrdfrow[corrdfindex+1]
            self.nextCorrValues.append(corrNext)
            self.prevCorrValues.append(corrPrev)
            self.sumCorrValues.append(corrNext+corrPrev)

        self.totalPowerSVD = np.sum(np.power(np.absolute(_SVDFiltered),2))
        self.totalPowerSVDDeck.append(self.totalPowerSVD)
        self.maxVariance = np.var(np.array(self.radarDataDeck)[:,personBin])
        self.maxVarianceDeck.append(self.maxVariance)

        # if personBin > len(radarData)
        # print('Length of radar data: ')
        # print (len(radarData))
        # print (len(radarData[0]))
        if personBin > len(radarData[0])-2:
            personBin = len(radarData[0])-5
        if modifySelf:
            self.currentRangeBin = personBin
            self.currentRange = _currentRange
            self.SVDFiltered = _SVDFiltered
            return None
        else:
            return [personBin,_currentRange, _SVDFiltered]

    def filterBin(self,dataBin,fs,modifySelf=False):
        USE_VARIANCE_FOR_RANGE = True
        USE_MTI_FILTER = True
        USE_BRBANDPASS_FILTER = True
        USE_HAMPEL_FILTER = False
        USE_MEDIAN_FILTER = False

        # Human breathing: ~0.1 - 1Hz
        APPROX_LOW_FREQ = 0.001  # low frequency. 60 * 0.1 = 6 Breaths per minute
        APPROX_HIGH_FREQ = 0.8  # high frequency. 60 * 1 = 60 Breaths per minute
        # APPROX_HIGH_FREQ = 2.0  # high frequency. 60 * 1 = 60 Breaths per minute

        # MTI filter applied for all scans in every range bin(HPF)
        if USE_MTI_FILTER:
            a = [1, -1]
            b = 1
            MTIFiltered = sp.signal.lfilter(a, b, dataBin)
        else:
            MTIFiltered = dataBin

        # BPF applied for every scan in all range bins (human breathing rate=0.1~1 Hz)
        if USE_BRBANDPASS_FILTER:
            nn = 3  # filter order
            # fs=50
            s1 = (2 * APPROX_LOW_FREQ) / fs  # normalized pass frequency
            s2 = (2 * APPROX_HIGH_FREQ) / fs  # normalized pass frequency
            b1, a1 = sp.signal.butter(nn, [s1, s2], 'bandpass')  # escond bandpass butter worth filter from 0.1-0.8 hz
            BRBPFFiltered = sp.signal.lfilter(b1, a1, MTIFiltered)
        else:
            BRBPFFiltered = MTIFiltered

        if USE_MEDIAN_FILTER:
            mediantemp = BRBPFFiltered
            signalInPersonBin = sp.signal.medfilt(mediantemp,np.ceil(fs/5))
        else:
            signalInPersonBin = BRBPFFiltered

        if modifySelf:
            self.filteredPersonBinSignal = signalInPersonBin
            return None
        else:
            return signalInPersonBin

    def findRangeAndFilter(self,radarData,fs):
        # finds the range of a person. Original Matlab code converted to Python by Britto
        MAXVOLTS = 1.04
        NORMALDACCOUNT_MAXRANGE = 8191

        # Algorithm configuration
        USE_SVD_CLUTTER_REMOVAL = False
        USE_TRANSPOSED_SVD = False
        USE_GHZ_BANDPASS = False
        USE_VARIANCE_FOR_RANGE = True
        USE_MTI_FILTER = True
        USE_BRBANDPASS_FILTER = True
        USE_HAMPEL_FILTER = False
        USE_MEDIAN_FILTER = False

        # Human breathing: ~0.1 - 1Hz
        APPROX_LOW_FREQ = 0.001  # low frequency. 60 * 0.1 = 6 Breaths per minute
        APPROX_HIGH_FREQ = 0.8  # high frequency. 60 * 1 = 60 Breaths per minute

        radarData = np.array(radarData)
        dataMatrix = radarData[:,1:-1]
        self.currentSlowTimes = radarData[:,0]

        dataMatrix = np.multiply(dataMatrix,
                                 ((self.radarSettings['DACStep'] / (self.radarSettings['PulsesPerStep']
                                                                    * self.radarSettings['Iterations'])) * MAXVOLTS / NORMALDACCOUNT_MAXRANGE))
        for index in range(0, len(dataMatrix[:, 0]), 1):
            dataMatrix[index, :] = dataMatrix[index, :] - np.mean(dataMatrix[index, :])

        distances = np.arange(0, len(dataMatrix[0, :])) * self.RADAR_RESOLUTION
        distances = np.matrix(distances)

        k = (fs / math.pow(2, 16)) * np.arange(0, math.pow(2, 16))  # for fft #BR_ removed -1 for len adjust
        rawscansV = np.transpose(dataMatrix)
        m, n = rawscansV.shape

        t = np.arange(0, len(dataMatrix[:, 0])) * 1 / fs

        if USE_TRANSPOSED_SVD:
            U, s, V = sp.linalg.svd(np.transpose(rawscansV))
            S = sp.linalg.diagsvd(s, U.shape[0], V.shape[0])
            S[0, 0] = 0
            SVDFiltered = np.transpose(U.dot(S).dot(np.transpose(V)))
        else:
            U, s, V = sp.linalg.svd(rawscansV)
            S = sp.linalg.diagsvd(s, U.shape[0], V.shape[0])
            S[0, 0] = 0
            SVDFiltered = U.dot(S).dot(np.transpose(V))

        if USE_GHZ_BANDPASS:
            # Ancho radar
            nn1 = 8  # filter order
            fs1 = 39E9  # sampling frequency
            # PGSelect = 6
            s11 = 2 * (6.2E9) / fs1  # normalized pass frequency
            s21 = 2 * (8.6E9) / fs1  # normalized stop frequency
            # PGSelect = 5
            # s11 = 2 * (5.9E9) / fs1 # normalized pass frequency
            # s21 = 2 * (7.8E9) / fs1 # normalized stop frequency
            [b11, a11] = sp.signal.butter(nn1, [s11, s21],
                                       'bandpass')  # first bandpass butter worth filter from 3.1 - 5.1 GHz
            y = np.zeros((m, n))  # IG_ y = np.zeros((m, n - 1000))
            if USE_SVD_CLUTTER_REMOVAL:
                for i in range(0, n):  # IG_ for i=1:n - 1000
                    y[:, i] = sp.signal.lfilter(b11, a11,
                                             SVDFiltered[:, i])  # IG_ y(:, i) = filter(b11, a11, rawscansV(:, i + 999))
            else:
                for i in range(0, n):  # IG_for i=1:n - 1000
                    y[:, i] = sp.signal.lfilter(b11, a11,
                                             rawscansV[:, i])  # IG_ y(:, i) = filter(b11, a11, rawscansV(:, i + 999))
        else:
            if USE_SVD_CLUTTER_REMOVAL:
                y = SVDFiltered
            else:
                y = rawscansV

        # MTI filter applied for all scans in every range bin(HPF)
        if USE_MTI_FILTER:
            MTIFiltered = np.zeros((m, n))  # IG_ filter1=zeros(m, n-1000)
            a = [1, -1]
            b = 1
            for i in range(0, m):  # BR_ made 0 for index adjust
                MTIFiltered[i, :] = sp.signal.lfilter(a, b, y[i, :])
        else:
            MTIFiltered = y

        # BPF applied for every scan in all range bins (human breathing rate=0.1~1 Hz)
        if USE_BRBANDPASS_FILTER:
            nn = 3  # filter order
            # fs=50
            s1 = (2 * APPROX_LOW_FREQ) / fs  # normalized pass frequency
            s2 = (2 * APPROX_HIGH_FREQ) / fs  # normalized pass frequency
            b1, a1 = sp.signal.butter(nn, [s1, s2], 'bandpass')  # escond bandpass butter worth filter from 0.1-0.8 hz
            BRBPFFiltered = np.zeros((m, n))  # y5=zeros(m, n-1000)
            for i in range(0, m):
                BRBPFFiltered[i, :] = sp.signal.lfilter(b1, a1, MTIFiltered[i, :])
        else:
            BRBPFFiltered = MTIFiltered

        # Range finding
        if USE_VARIANCE_FOR_RANGE:
            # The bin with maximum variance is used for identifying the range
            m2 = np.var(np.transpose(SVDFiltered), 0, ddof=1)
            personBin = np.argmax(m2)
            self.currentRange = distances[0, personBin]  # target's range bin
            # print'Target range bin = ',personBin
            # print 'Estimated distance = ',targetRange
            signalInPersonBin = np.transpose(BRBPFFiltered[personBin, :])
        else:
            # person range changes every slow time sample due to movement of chest
            # while breathing. The index of the bin with the maximum amplitude is
            # identified and the mean of the bins is taken as the bin where the person
            # is located.
            high_filtered = np.transpose(SVDFiltered)
            maxIndices = np.zeros(len(high_filtered[:, 0]))
            for i in range(0, len(high_filtered[:, 0])):
                maxIndices[i] = np.argmax(high_filtered[i, :])
            # mean of person location. this bin will be used to take the breathing
            # signal
            personBin = np.ceil(np.mean(maxIndices))
            self.currentRange = distances[0, personBin]
            # print 'Target range bin = ',personBin
            # print 'Estimated distance  = ',targetRange
            # Take mean bin
            signalInPersonBin = np.transpose(BRBPFFiltered[personBin, :])

        # if USE_HAMPEL_FILTER:
        #     hampeltemp = signalInPersonBin
        #     signalInPersonBin = signal.hampel(hampeltemp,np.ceil(fs))
        #
        #
        # if USE_MEDIAN_FILTER:
        #     mediantemp = signalInPersonBin
        #     signalInPersonBin = signal.medfilt(mediantemp,np.ceil(fs/5))

        self.filteredPersonBinSignal = signalInPersonBin - np.mean(signalInPersonBin)

        return personBin  # return range bin

    def findRangeAndBreathRate(self,radarData,fs,outQ):
        rangeBin = self.findRange(radarData,fs)
        breathRateDict = self.findBreathRateAll(radarData,radarData[rangeBin],fs)
        outQ.put([rangeBin,breathRateDict])
        return

    def findBreathRateAll(self,radarData,filteredSig,fs):
        breathRateDict = {}
        breathRateDict['FFTMax'] = 14.32
        return breathRateDict

    def safeStop(self):
        print ('Stopping %s' % self.name)
        self.plotterStopEvent.set()
        print ('Stopping Plotter...')
        # self.plotter.join()
        # self.outputQ.close()
        # self.inputQ.close()
        # print ('Plotter and %s stopped' %self.name)
