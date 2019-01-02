u'''
Created on Mar 24, 2017

@author: isuru, modified by rajitha to accomodate sensor multiplexing 12/24/2018
'''

from __future__ import with_statement
from __future__ import division
from __future__ import absolute_import
from __future__ import print_function

try:
    import rospy
    from std_msgs.msg import String
    from std_msgs.msg import Float32MultiArray
    from std_msgs.msg import Float32, Int32
    from sensor_msgs import msg
    ROS_AVAILABLE = True
except:
    print('ROS not found')
    ROS_AVAILABLE = False
# ROS_AVAILABLE = True # let's keep it so, until we fix the main thread issue for ros node init

import time
import collections
import subprocess
import os
import csv
import math
import numpy as np
import scipy as sp
import scipy.stats
from ast import literal_eval
import logging
import datetime

import multiprocessing
from processBR import processBR
import HttpThread
import threading
import queue
from globalVars import *
from radarHandler import CollectionThread,CollectionThreadX4

import pandas as pd

from colorPrint import bcolors
# from io import open
try:
    import matlab.engine
    MATLAB_AVAILABLE = True
except ImportError:
    print('Matlab Engine not found')
    MATLAB_AVAILABLE = False

# MATLAB_AVAILABLE = False ## Use this to disable Matlab engine
# ROS_AVAILABLE = False ## Use this to disable ROS

class MainClass(object):
    def __init__(self, radarNumber = 1, getExternal = False,simfilepath=None,resultsFileName=None,bbeltFile=None,useX4 = None,simulate = None, port=None):
        self.USE_X4 = True
        self.SIMULATE = False
        self.NON_REALTIME_MODE = False
        self.ERSB_NN_TRAIN_AUTOMATICALLY = False
        self.OPEN_LABELS_FILE = False
        self.OPEN_BBELT_FILE = False
        self.COMPARE_TO_REFERENCE = True
        self.radarNumber = radarNumber

        # Getting file paths: ls -d -1 $PWD/{*,.*}
        if getExternal:
            self.SIMFILEPATHS = simfilepath
            self.RESULTS_FILE_NAME = resultsFileName
            self.BBELT_FILE = bbeltFile
            self.USE_X4 = useX4
            self.SIMULATE = simulate
        else:
            self.SIMFILEPATHS = [os.path.expanduser(
                '~/') + 'radarData/breathingx4/BreathingRadarData/2017-08-01-14-03-21_Xander_Breathing_1_UWBX4.csv']
            self.RESULTS_FILE_NAME = 'TEST0123'
            self.BBELT_FILE = os.path.expanduser(
                '~/') + 'radarData/breathingx4/BreathingRadarData/2017-08-01-14-03-21_BBelt.csv'

        if 'DONT_OPEN_BBELT' in self.BBELT_FILE:
            self.OPEN_BBELT_FILE = False

        if self.OPEN_LABELS_FILE:
            self.labelsForNN = pd.read_csv(self.LABEL_FILE_PATH)

        if self.OPEN_BBELT_FILE:
            self.bbeltTimesRecorded = []
            self.bbeltData = []
            with open(self.BBELT_FILE) as csvFile:
                csvReader = csv.reader(csvFile)
                for row in csvReader:
                    self.bbeltTimesRecorded.append(float(row[0]))
                    thisRowElements = int(row[1])
                    for index in range(0,thisRowElements):
                        self.bbeltData.append(float(row[index+2]))
            self.bbeltFs = len(self.bbeltData)*1000/(self.bbeltTimesRecorded[-1]-self.bbeltTimesRecorded[0])
            # print (bcolors.ERROR+str(self.bbeltFs)+bcolors.ENDC)
            self.bbeltTimesGenerated = np.linspace(self.bbeltTimesRecorded[0],self.bbeltTimesRecorded[-1],
                                                   num=len(self.bbeltData))

        self.SAVE_DATA = True
        #if self.SIMULATE:
        #    self.SAVE_DATA = False #no need to save data again
        self.FILE_LENGTH = 60  # length of file to save in seconds
        if self.USE_X4:
            os.chdir(os.getcwd())
            print('radar_{0}_Data/rawDataX4/'.format(radarNumber) + time.strftime(u"%Y%m%d"))
            self.DIRECTORY_PATH = '/mnt/c/workspace/data/radar/rawData_radar_{0}/'.format(radarNumber)+ time.strftime(u"%Y%m%d")
        else:
            self.DIRECTORY_PATH = os.path.expanduser('~/')+'radarData/rawDataV2/' + time.strftime(u"%Y%m%d")

        # if not os.path.exists('/home/newuser'): #Check which machine is being used
        #     if os.path.exists('/home/isuru'):
        #         self.DIRECTORY_PATH = self.DIRECTORY_PATH.replace('newuser','isuru')
        #         print (bcolors.OKBLUE+'Directory path changed to: '+self.DIRECTORY_PATH+bcolors.ENDC)
        #         self.SIMFILEPATHS = [files.replace('newuser','isuru') for files in self.SIMFILEPATHS]
        #     else:
        #         print (bcolors.FAIL+'Unknown username, aborting'+bcolors.ENDC)
        #         quit()
        # else: #This is the machine in the lab
        #     if os.path.exists('/home/newuser'):
        #         self.DIRECTORY_PATH = self.DIRECTORY_PATH.replace('isuru','newuser')
        #         print (bcolors.OKBLUE+'Directory path changed to: '+self.DIRECTORY_PATH+bcolors.ENDC)
        #         self.SIMFILEPATHS = [files.replace('isuru','newuser') for files in self.SIMFILEPATHS]
        #     else:
        #         print (bcolors.FAIL+'Unknown username, aborting'+bcolors.ENDC)
        #         quit()

        self.ALGORITHM_TO_USE = 'EMD'
        #Possible values: 'EMD','EMD_MEDIAN','NMD','NMD_MEDIAN','AUTO'
        self.NUM_BREATHRATEMEDIANVALUES = 5

        self.USE_CLASSIFIER = False # Make this true to run Zach's posture classifier
        if MATLAB_AVAILABLE == False:
            self.USE_CLASSIFIER = False #Disable matlab based classifier

        if not os.path.exists(self.DIRECTORY_PATH) and not self.SIMULATE:
            os.makedirs(self.DIRECTORY_PATH)
        self.DIRECTORY_PATH = self.DIRECTORY_PATH + '/'
        self.FILE_NAME_APPEND = 'LAB-5130-ABDULLAH-SIT-SIDE-BOOK'
        self.radarDataDeck = collections.deque(maxlen=self.FILE_LENGTH * MAX_FS)

        self.breathRateDeck = collections.deque(maxlen=self.NUM_BREATHRATEMEDIANVALUES) #keep 10 previous values of breathing rates
        self.breathRatesAllAlgos = {}
        self.breathRateHistoryAllAlgos = {}
        self.resultsCSVTable = pd.DataFrame()
        self.resultsCSVTable['Time(ms)'] = []
        self.resultsCSVTable['Reference'] = [] #np.empty((len(self.resultsCSVTable), 0)).tolist()
        self.resultsOnlyCSVTable = pd.DataFrame() #Data frame with only results

        # self.rangeCSVTable = pd.DataFrame(data={'Time(ms)':[],'Range(m)':[],'Activity':[],'InstantaneousRange(m)':[]})
        self.rangeCSVTable = pd.DataFrame()
        self.rangeCSVTable['Activity'] = []
        self.rangeCSVTable['Time(ms)'] = []
        self.rangeCSVTable['Range(m)'] = []
        self.rangeCSVTable['InstantaneousRange(m)'] =[]

        self.resultsDataSaved = False
        self.NUM_BREATHRATEHISTORY = 60 #Store 60 historical breath rates
        self.breathRateAlgoScores = {}
        self.currentBestAlgorithm = 'None'

        self.personState = 'Unknown'

        self.personIntState = 0

        self.EMDBreathRate = 0

        # roscore = subprocess.Popen('roscore')
        # print(roscore)

        # CLOUD START
        # Init HTTP Thread
        self.QDataDict = queue.Queue()
        self.stopHttpThread = threading.Event()
        self.httpThread = HttpThread.HttpThread(self.QDataDict, self.stopHttpThread)
        self.httpThread.start()
        # CLOUD END


        self.referenceRate = 0
        self.previousReferenceRate = 0
        # if self.OPEN_BBELT_FILE == False:
            # self.referenceRate = 15

        if self.USE_X4:
            self.createRadarSettingsDict('x4')
        else:
            self.createRadarSettingsDict('x2')
        if self.USE_CLASSIFIER and MATLAB_AVAILABLE:
            self.engClassification = matlab.engine.start_matlab()
        if MATLAB_AVAILABLE:
            self.engProcRadar = matlab.engine.start_matlab()
        self.stopEvent = threading.Event()
        self.radarResumeEvent = threading.Event()
        self.resumeEvent = threading.Event()
        self.pauseEvent = threading.Event()
        self.radarPauseEvent = threading.Event()
        #self.resumeEvent.set()

        #for the main thread of the class
        self.mainPauseEvent = threading.Event()
        self.mainResumeEvent = threading.Event()

        if MATLAB_AVAILABLE:
            self.engProcRadar.workspace['radarResolution'] = self.RADAR_RESOLUTION
            self.engProcRadar.workspace['calledFrom'] = 'Python35'

        # self.startWallTime = datetime.datetime.now()
        self.startWallTime = time.time()
        # self.startWallTime = self.startWallTime.strftime("%Y-%m-%d %H:%M:%S.%f")

        # radar sensor thread semaphors
        self.radarDataLock = threading.Lock()
        self.radarDataQ = queue.Queue()
        #

        if self.SIMULATE:
            if self.USE_X4:
                self.radarThread = CollectionThreadX4(1,'radarThreadX4',self.stopEvent,self.radarSettings,baseband=True,simulate=True,
                                                      filePaths=self.SIMFILEPATHS,nonRealTimeMode=self.NON_REALTIME_MODE,resumeEvent=self.resumeEvent)
            else:
                self.radarThread = CollectionThread(1, 'radarThread', self.stopEvent, self.radarSettings, radarIP='192.168.7.2', simulate=True,
                                            filePaths=self.SIMFILEPATHS,nonRealTimeMode=self.NON_REALTIME_MODE,resumeEvent=self.resumeEvent)
        else:
            if self.USE_X4:
                self.radarThread = CollectionThreadX4(radarNumber,'radarThreadX4_{0}'.format(radarNumber),self.stopEvent,self.radarSettings,baseband=True,
                                                      nonRealTimeMode=self.NON_REALTIME_MODE,resumeEvent=self.radarResumeEvent,
                                                      dataQueue=self.radarDataQ, dataLock=self.radarDataLock, pauseEvent=self.radarPauseEvent, radarPort=port)
            else:
                self.radarThread = CollectionThread(1, 'radarThread', self.stopEvent, self.radarSettings, radarIP='192.168.7.2',
                                                    nonRealTimeMode=self.NON_REALTIME_MODE,resumeEvent=self.resumeEvent)

        self.radarDataWindow = []
        self.matlabVars = {}
        self.dspVars = {}
        self.dspVars['pythonRangeBin'] = 0
        self.dspVars['fs'] = 0

        self.breathSignal = []

        self.procInputDict = {}
        self.procInputDict['radarData'] = []
        self.procInputQ = multiprocessing.Queue()
        self.procOutputQ = multiprocessing.Queue()
        self.procStopEvent = multiprocessing.Event()
        self.processBRProc = processBR(self.procInputQ,self.procOutputQ,
                                       self.procStopEvent,self.radarSettings)
        self.referenceSignal = []
        self.referenceSignalTime = []

        if ROS_AVAILABLE:
            #ROS publishers
            self.pubBreathSig = rospy.Publisher('RD_breathSignal_{0}'.format(self.radarNumber), Float32MultiArray, queue_size=10)
            self.pubRD_Range = rospy.Publisher('RD_RANGE', Float32, queue_size=10)
            self.pubBreathRateWelch = rospy.Publisher('BR_Welch', Float32, queue_size=10)
            self.pubBreathRateFFTMax = rospy.Publisher('BR_FFTMax', Float32, queue_size=10)
            self.pubBreathRateHS = rospy.Publisher('BR_HS', Float32, queue_size=10)
            self.pubBreathRateNOTCH = rospy.Publisher('BR_NOTCH', Float32, queue_size=10)
            self.pubBreathRateMvngWin = rospy.Publisher('BR_MvngWin', Float32, queue_size=10)
            self.pubBreathRateSUM2_3FREQ = rospy.Publisher('BR_SUM2_3FREQ', Float32, queue_size=10)
            self.pubBreathRateSHANON = rospy.Publisher('BR_SHANON', Float32, queue_size=10)
            self.pubBreathRateLogEnt = rospy.Publisher('BR_LogEnt', Float32, queue_size=10)
            self.pubBreathRateFFTMLE = rospy.Publisher('BR_FFTMLE', Float32, queue_size=10)
            self.pubBreathRateFFTELLIPSE = rospy.Publisher('BR_FFTELLIPSE', Float32, queue_size=10)
            self.pubBreathRateFFTMA = rospy.Publisher('BR_FFTMA', Float32, queue_size=10)
            self.pubBreathRateNMD = rospy.Publisher('BR_NMD', Float32, queue_size=10)
            self.pubBreathRateCombined = rospy.Publisher('RD_BRate', Float32, queue_size=10)
            self.pubBreathRateEMD = rospy.Publisher('BR_EMD', Float32, queue_size=10)
            self.breathingRateGlobal = 0
            self.intPosture = 0

            self.rangeConePublisher = rospy.Publisher('RadarRange_{0}'.format(self.radarNumber), msg.Range, queue_size=10)

            self.activityPublisher = rospy.Publisher('RD_Activity_{0}'.format(self.radarNumber),String,queue_size=10)
            self.posturePublisher = rospy.Publisher('RD_Posture_{0}'.format(self.radarNumber), String, queue_size=10)
        else:
            # ROS publishers
            self.pubBreathSig = None
            self.pubRD_Range = None
            self.pubBreathRateWelch = None
            self.pubBreathRateFFTMax = None
            self.pubBreathRateHS = None
            self.pubBreathRateNOTCH = None
            self.pubBreathRateMvngWin = None
            self.pubBreathRateSUM2_3FREQ = None
            self.pubBreathRateSHANON = None
            self.pubBreathRateLogEnt = None
            self.pubBreathRateFFTMLE = None
            self.pubBreathRateFFTELLIPSE = None
            self.pubBreathRateFFTMA = None
            self.pubBreathRateNMD = None
            self.pubBreathRateCombined = None
            self.pubBreathRateEMD = None

            self.rangeConePublisher = None

            self.activityPublisher = None
            self.posturePublisher = None

        self.ERSB_NN_Trained = False
        self.dataClass = -1

        self.endTime = 0
        self.previousEndTime = 0

    def createRadarSettingsDict(self,moduleName):
        self.radarSettings = {}
        if moduleName == 'x2':
            self.radarSettings['PGSelect'] = 6
            self.radarSettings['FrameStitch'] = 3
            self.radarSettings['SampleDelayToReference'] = 2.9e-9
            self.radarSettings['Iterations'] = 50
            self.radarSettings['DACStep'] = 4
            self.radarSettings['DACMin'] = 0
            self.radarSettings['DACMax'] = 8191
            self.radarSettings['PulsesPerStep'] = 16
            self.radarSettings['RADAR_RESOLUTION'] = 3.90625 / 1000 # X2
            self.radarSettings['RadarType'] = 'X2'
        elif moduleName == 'x4':
            self.radarSettings['Iterations'] = 16
            self.radarSettings['DACMin'] = 949
            self.radarSettings['DACMax'] = 1100
            # self.radarSettings['DACMin'] = 900
            # self.radarSettings['DACMax'] = 1150
            self.radarSettings['PulsesPerStep'] = 26
            self.radarSettings['FrameStart'] = 0
            self.radarSettings['FrameStop'] = 9.75
            self.radarSettings['DACStep'] = 1 #This value is NOT USED. Just put here for the normalization
            self.radarSettings['RADAR_RESOLUTION'] = 51.8617 / 1000  # X4
            self.radarSettings['RadarType'] = 'X4'
        self.RADAR_RESOLUTION = self.radarSettings['RADAR_RESOLUTION']

    def publishRange(self,rangeM):
        rangeData = msg.Range()
        rangeData.header.frame_id = "radarFrame"
        rangeData.header.stamp = rospy.Time.now()
        rangeData.radiation_type = 0
        rangeData.field_of_view = self.breathingRateGlobal
        rangeData.min_range = float(self.intPosture)

        if self.personState == "Unknown":
            self.personIntState = 0
        elif self.personState !=  'Stationary':
            self.personIntState = 1
        else:
            self.personIntState = 2

        rangeData.max_range = self.personIntState
        rangeData.range = rangeM
        self.rangeConePublisher.publish(rangeData)

    def publishActivity(self,actMB, actClass):
        # if (actMB['event_type'] == 'Stationary' and actClass == 1):
        #     self.activityPublisher.publish(actMB['event_type'])
        # elif (actMB['event_type'] != 'Stationary'):
        #     self.activityPublisher.publish(actMB['event_type'])
        # else:
        #     self.activityPublisher.publish("Non Stationary")
        self.activityPublisher.publish(actMB['event_type'])

    def publishPosture(self, postureID):
        strPosture = ""
        if (postureID == 1):
            strPosture = "Non Standing"
            self.intPosture = postureID
        elif (postureID == 2):
            strPosture = "Standing"
            self.intPosture = postureID
        elif (postureID == 3):
            strPosture = "Non Standing"
            self.intPosture = postureID
        else:
            strPosture = "Undefined"
            self.intPosture = 0
        self.posturePublisher.publish(strPosture)

    def fetchAndPublishMatlabWorkspaceVars(self):
        self.matlabVars.clear()
        try:
            # self.matlabVars['filteredPersonBinSignal'] = list(np.float32(self.engProcRadar.workspace['filteredPersonBinSignal']))
            # self.matlabVars['filteredPersonBinSignal'] = list(np.float32(self.engProcRadar.workspace['rawPersonBinSignal'])) #Use raw signal
            self.matlabVars['filteredPersonBinSignal'] = list(np.float32(self.engProcRadar.workspace['NMD_Signal']))  # Use NMD signal
            breathingSignal = Float32MultiArray()
            # breathingSignal.data = self.matlabVars['filteredPersonBinSignal']
            breathingSignal.data = self.breathSignal
            # self.pubBreathSig.publish(breathingSignal)
        except:
            # print (bcolors.FAIL+'filteredPersonBinSignal not found'+bcolors.ENDC)
            pass
        # self.publishMatlabVar('targetRange',self.pubRD_Range)
        # self.publishMatlabVar('BreathRatePxx', self.pubBreathRateWelch)
        # self.publishMatlabVar('breathing_rate_without_processing', self.pubBreathRateFFTMax)
        # self.publishMatlabVar('breathingRateAfterHarmonicSuppression', self.pubBreathRateHS)
        # self.publishMatlabVar('Breathing_rate_notch', self.pubBreathRateNOTCH)
        # self.publishMatlabVar('Breathing_rate_energy_sum_window', self.pubBreathRateMvngWin)
        # self.publishMatlabVar('Breathing_rate_energy_sum_squared', self.pubBreathRateSUM2_3FREQ)
        # self.publishMatlabVar('Breathing_rate_energy_Shannon_squared', self.pubBreathRateSHANON)
        # self.publishMatlabVar('Breathing_rate_Log_entropy', self.pubBreathRateLogEnt)
        # self.publishMatlabVar('breathing_rate_fft_MLE', self.pubBreathRateFFTMLE)
        # self.publishMatlabVar('breathing_rate_fft_ellipse_fitting', self.pubBreathRateFFTELLIPSE)
        # self.publishMatlabVar('breathing_rate_fft_MA', self.pubBreathRateFFTMA) #This is not available
        # in UOTTAWA Matlab
        self.publishMatlabVar('NMD_Breathing_rate',self.pubBreathRateNMD)
        #Publish combined breathing rate
        if self.ALGORITHM_TO_USE == 'NMD':
            self.publishMatlabVar('NMD_Breathing_rate', self.pubBreathRateCombined)
        elif self.ALGORITHM_TO_USE == 'NMD_MEDIAN':
            try:
                self.breathRateDeck.append(self.engProcRadar.workspace['NMD_Breathing_rate'])
                medianRate = np.median(self.breathRateDeck)
                self.pubBreathRateCombined.publish(float(medianRate))
            except:
                # print(bcolors.FAIL+'NMD breathing rate not found'+bcolors.ENDC)
                pass

    def publishMatlabVar(self,matlabKey,publisher):
        try:
            breathRateValue = self.engProcRadar.workspace[matlabKey]
            self.breathRatesAllAlgos[matlabKey] = breathRateValue
            if matlabKey in self.breathRateHistoryAllAlgos.keys():
                self.breathRateHistoryAllAlgos[matlabKey].append(breathRateValue)
            else:
                self.breathRateHistoryAllAlgos[matlabKey] = collections.deque(maxlen=self.NUM_BREATHRATEHISTORY)
                self.breathRateHistoryAllAlgos[matlabKey].append(breathRateValue)
            self.matlabVars[matlabKey] = Float32(self.engProcRadar.workspace[matlabKey])
            if ROS_AVAILABLE:
                publisher.publish(self.matlabVars[matlabKey])
        except:
            # print ('%s not found'%matlabKey)
            pass

    def startMatlabProcessing(self):
        # if isinstance(self.radarDataWindow[0][0],complex): #Check if complex data
        #     self.engProcRadar.workspace['dataMatrix'] = abs(self.radarDataWindow)
        # else:
        # print('Data is: %s'%type(self.radarDataWindow[2][2]))
        # print('Data array is: %s'%type(self.radarDataWindow))
        if isinstance(self.radarDataWindow[2][2],np.complex128):
            self.radarDataWindow = list(map(np.complex64,self.radarDataWindow))
            radarNewData = list()
            for narrays in self.radarDataWindow:
                row = narrays.tolist()
                radarNewData.append(row)
            # print('After cast: %s' % type(radarNewData[2][2]))
            # print('Data array is: %s' % type(radarNewData))
            self.engProcRadar.workspace['dataMatrix'] = radarNewData
            dataList = radarNewData
        else:
            self.engProcRadar.workspace['dataMatrix'] = self.radarDataWindow
            dataList = self.radarDataWindow
        # print(dataList)
        self.engProcRadar.workspace['fs'] = self.dspVars['fs']
        self.engProcRadar.workspace['pythonRangeBin'] = int(self.dspVars['pythonRangeBin'] + 1)
        # self.futureOut = self.engProcRadar.processbr_engine(nargout=0, async=True)
        if self.USE_CLASSIFIER:
            self.classOut = self.engClassification.ClassifyPosture(dataList,nargout=2, async=True)
        self.radarDataWindow = []

    def startMatlabProcessingSingleBin(self):
        self.engProcRadar.workspace['dataMatrix'] = self.radarDataWindow
        self.engProcRadar.workspace['fs'] = self.dspVars['fs']
        # self.futureOut = self.engProcRadar.processbr_sin(nargout=0, async=True)
        self.radarDataWindow = []

    def isClassifierDone(self):
        if self.USE_CLASSIFIER:
            return self.classOut.done()
        else:
            return True

    def isMatlabBreathingDone(self):
        if MATLAB_AVAILABLE:
            return self.futureOut.done()
        else:
            return True

    def getReferenceRate(self,startTime,endTime):
        if self.OPEN_BBELT_FILE:
            try:
                startIndex = next(x[0] for x in enumerate(self.bbeltTimesGenerated) if x[1] >= startTime)
            except:
                # print(bcolors.WARNING+'Start time not found'+bcolors.ENDC)
                startIndex = 0
            try:
                endIndex = next(x[0] for x in enumerate(self.bbeltTimesGenerated) if x[1] >= endTime)
            except:
                # print(bcolors.WARNING+'End time not found'+bcolors.ENDC)
                endIndex = len(self.bbeltData) - 1
            nForFFT = 2**16
            bbeltWindowedData = self.bbeltData[startIndex:endIndex]
            bbeltWindowedData = bbeltWindowedData - np.mean(bbeltWindowedData)
            self.referenceSignal = bbeltWindowedData
            self.referenceSignalTime = self.bbeltTimesGenerated[startIndex:endIndex]
            windowMult = np.hamming(len(bbeltWindowedData))
            bbeltWindowedData = bbeltWindowedData * windowMult
            # print(startTime,endTime,self.bbeltTimesGenerated[startIndex],self.bbeltTimesGenerated[endIndex])
            fftOfBBelt = abs(sp.fftpack.fft(bbeltWindowedData,n=nForFFT))
            fftOfBBelt[0]=0
            # xOfBBeltFFT = sp.fftpack.fftfreq(len(bbeltWindowedData),d=1/self.bbeltFs)
            xOfBBeltFFT = sp.fftpack.fftfreq(n=nForFFT, d=1 / self.bbeltFs)
            maxIdxOfBBeltFFT = np.argmax(fftOfBBelt)
            bbeltReferenceFrequency = xOfBBeltFFT[maxIdxOfBBeltFFT]*60
            # print (bcolors.OKGREEN+'Reference rate: '+str(bbeltReferenceFrequency)+bcolors.ENDC)
            return bbeltReferenceFrequency
        else:
            return 0

    def main(self):
#        global radarDataQ

        if MATLAB_AVAILABLE:
            self.futureOut = self.engProcRadar.sqrt(4.0, async=True)  # create dummy matlab process
        if self.USE_CLASSIFIER and MATLAB_AVAILABLE:
            self.classOut = self.engClassification.sqrt(4.0, async=True)
        # self.radarResumeEvent.set()
        # self.radarPauseEvent.clear()
        self.radarThread.start() # start data collection thread
        self.processBRProc.start() # start breath rate processing process
        # if self.SIMULATE == False:
        #This sleep is needed for the non real time mode as well
        # if self.USE_CLASSIFIER:
        time.sleep(15) #Sleep for radar to initialize
        firstDataProcessed = False
        try:
            # if ROS_AVAILABLE:
            #     rospy.init_node('realtimeBreathing_{0}'.format(self.radarNumber))
            #rospy.sleep(WINDOW_LENGTH) # sleep till first set of useful data is collected
            elapsedTime = 0
            while True:
                # if self.mainPauseEvent.is_set():
                #     print('radar thread {0} is paused'.format(self.radarNumber))
                #     self.mainResumeEvent.wait()
                #     print('radar thread {0} is resumed'.format(self.radarNumber))
                #     self.mainResumeEvent.clear()
                #     while not self.radarDataQ.empty():
                #         self.radarDataQ.get()
                #     self.radarResumeEvent.set()
                sleepTime = PROCESSING_INTERVAL - elapsedTime
                # print('Sleeping : %f seconds' % sleepTime)
                if sleepTime>0:
                    time.sleep(sleepTime)
                startTime = time.time()
                #''' acquire required data from the sensor thread'''
                if not self.radarDataQ.empty():
                    # print('Getting radar data...')
                    # radarDataTemp = []
                    self.radarDataLock.acquire()
                    while not self.radarDataQ.empty():
                        # radarDataTemp.append(radarDataQ.get())
                        self.procInputDict['radarData'].append(self.radarDataQ.get())
                    self.radarDataLock.release()
                    self.radarDataDeck.extend(self.procInputDict['radarData']) #Needed for Matlab and saving the data
                #'''end of the data acquisition'''
                    self.previousEndTime = self.endTime
                    if isinstance(self.radarDataDeck[-1][0], complex):
                        self.endTime = self.radarDataDeck[-1][0].real  # last timestamp of radarData in milliseconds
                    else:
                        self.endTime = self.radarDataDeck[-1][0]
                    startAt = self.endTime - WINDOW_LENGTH * 1000  # starting timestamp of radardata to extract window in milliseconds
                    if self.OPEN_LABELS_FILE:
                        dframeRange = self.labelsForNN[(self.labelsForNN['Time']>=startAt) &
                                                            (self.labelsForNN['Time']<=self.endTime)]
                        self.dataClasses = np.array(dframeRange['Label'])
                        if self.dataClasses[0] == self.dataClasses[-1]:  # Check if class changed
                            self.dataClass = self.dataClasses[-1]
                        elif self.dataClasses[-1]==-20: #Train
                            self.dataClass = -20
                        else:
                            self.dataClass = -1
                    print('End time: ' + str(self.endTime))
                    print(bcolors.OKGREEN+"WallEndTime: " + str(self.startWallTime + (self.endTime/1000))+bcolors.ENDC)
                    startIndex = 0
                    for rdDataRow in self.radarDataDeck:
                        startIndex += 1
                        if isinstance(rdDataRow[0], complex):
                            if rdDataRow[0].real > startAt:
                                startIndex -= 1
                                break
                        else:
                            if rdDataRow[0] > startAt:
                                startIndex -= 1
                                break
                    if isinstance(self.radarDataDeck[-1][0], complex):
                        self.windowStartTime = self.radarDataDeck[startIndex][0].real  # last timestamp of radarData in milliseconds
                    else:
                        self.windowStartTime = self.radarDataDeck[startIndex][0]

                    self.radarDataWindow = list(self.radarDataDeck)[startIndex:-1][:]
                    if isinstance(self.radarDataDeck[-1][0], complex):
                        self.dspVars['fs'] = 1000.0 * len(self.radarDataWindow) / (
                        self.radarDataWindow[-1][0].real - self.radarDataWindow[0][0].real)
                    else:
                        self.dspVars['fs'] = 1000.0 * len(self.radarDataWindow) / (
                        self.radarDataWindow[-1][0] - self.radarDataWindow[0][0])
                    # MATLAB
                    if MATLAB_AVAILABLE:
                        if firstDataProcessed:
                            self.fetchAndPublishMatlabWorkspaceVars()
                            if self.USE_CLASSIFIER:
                                try:
                                    self.publishPosture(self.classOut.result()[0])
                                except:
                                    print(bcolors.FAIL+'Posture not found'+bcolors.ENDC)
                        self.startMatlabProcessing()
                        firstDataProcessed = True


                    dataSaved = False
                    if self.SAVE_DATA:
                        if isinstance(self.radarDataDeck[-1][0],complex):
                            if self.radarDataDeck[-1][0].real - self.radarDataDeck[0][0].real > self.FILE_LENGTH * 1000:
                                self.saveData()
                                # self.procInputDict['radarData'] = []
                                dataSaved = True
                        else:
                            if self.radarDataDeck[-1][0] - self.radarDataDeck[0][0] > self.FILE_LENGTH * 1000:
                                self.saveData()
                                # self.procInputDict['radarData'] = []
                                dataSaved = True
                    if dataSaved :
                        # self.radarPauseEvent.set()
                        # self.mainPauseEvent.set()
                        pass

                    self.previousReferenceRate = self.referenceRate
                    self.referenceRate = self.getReferenceRate(self.windowStartTime, self.endTime)
                    self.procInputDict['RefSignal'] = self.referenceSignal
                    self.procInputDict['RefSignalTime'] = self.referenceSignalTime
                    self.procInputDict['RefRate'] = self.referenceRate
                    self.procInputQ.put(self.procInputDict)  # send data into processing
                    self.procInputDict = {}
                    self.procInputDict['radarData'] = []
                    processedOutputReceived = False
                    while not self.procOutputQ.empty(): # get processed data
                        outputItem = self.procOutputQ.get()
                        processedOutputReceived = True
                    if processedOutputReceived:
                        self.currentRangeM = outputItem[0]*self.RADAR_RESOLUTION
                        self.currentInstRangeM = outputItem[7]*self.RADAR_RESOLUTION
                        self.dspVars['pythonRangeBin'] = outputItem[0]
                        if ROS_AVAILABLE:
                            self.publishRange(self.currentRangeM)
                        if self.USE_CLASSIFIER:
                            try:
                                self.publishActivity(outputItem[3], self.classOut.result()[1])
                            except:
                                print(bcolors.FAIL+'Activity not found'+bcolors.ENDC)
                        if len(outputItem[2])>0:
                            self.EMDBreathRate = outputItem[2][-1] #get latest rate
                            # self.EMDBreathRate = np.median(outputItem[2]) #get median rate
                            self.breathSignal = outputItem[1] #Breathing signal raw
                            # print('EMD Median Rate: %f'%self.EMDBreathRate)
                            # print('EMD current rate: %f'%outputItem[2][-1])
                            if ROS_AVAILABLE:
                                self.breathingRateGlobal = self.EMDBreathRate
                                self.pubBreathRateEMD.publish(self.EMDBreathRate)
                                if self.ALGORITHM_TO_USE=='EMD_MEDIAN':
                                    self.pubBreathRateCombined.publish(self.EMDBreathRate)
                                elif self.ALGORITHM_TO_USE == 'EMD':
                                    self.pubBreathRateCombined.publish(outputItem[2][-1])
                            self.breathRatesAllAlgos['EMD'] = outputItem[2][-1]
                            # self.breathRatesAllAlgos['EMD_Median'] = self.EMDBreathRate
                            if 'EMD' in self.breathRateHistoryAllAlgos.keys():
                                self.breathRateHistoryAllAlgos['EMD'].append(outputItem[2][-1])
                                # self.breathRateHistoryAllAlgos['EMD_Median'].append(self.EMDBreathRate)
                            else:
                                self.breathRateHistoryAllAlgos['EMD'] = collections.deque(
                                    maxlen=self.NUM_BREATHRATEHISTORY)
                                self.breathRateHistoryAllAlgos['EMD'].append(outputItem[2][-1])
                                # self.breathRateHistoryAllAlgos['EMD_Median'] = collections.deque(
                                #     maxlen=self.NUM_BREATHRATEHISTORY)
                                # self.breathRateHistoryAllAlgos['EMD_Median'].append(self.EMDBreathRate)
                        self.personState = outputItem[3]['event_type']
                        self.rangeCSVTable = self.rangeCSVTable.append({'Range(m)': self.currentRangeM,
                                                                        'InstantaneousRange(m)': self.currentInstRangeM,
                                                                        'Time(ms)': self.endTime,
                                                                        'Activity': self.personState},
                                                                        ignore_index=True)
                        self.breathRatesAllAlgos['FFTMaxFreq'] = outputItem[4]
                        if 'FFTMaxFreq' in self.breathRateHistoryAllAlgos.keys():
                            self.breathRateHistoryAllAlgos['FFTMaxFreq'].append(outputItem[4])
                        else:
                            self.breathRateHistoryAllAlgos['FFTMaxFreq'] = collections.deque(maxlen=self.NUM_BREATHRATEHISTORY)
                            self.breathRateHistoryAllAlgos['FFTMaxFreq'].append(outputItem[4])

                        # self.breathRatesAllAlgos['WTBRate'] = outputItem[5]
                        # if 'WTBRate' in self.breathRateHistoryAllAlgos.keys():
                        #     self.breathRateHistoryAllAlgos['WTBRate'].append(outputItem[5])
                        # else:
                        #     self.breathRateHistoryAllAlgos['WTBRate'] = collections.deque(
                        #         maxlen=self.NUM_BREATHRATEHISTORY)
                        #     self.breathRateHistoryAllAlgos['WTBRate'].append(outputItem[5])

                        self.breathRatesAllAlgos['MinkBRate'] = outputItem[6]
                        if 'MinkBRate' in self.breathRateHistoryAllAlgos.keys():
                            self.breathRateHistoryAllAlgos['MinkBRate'].append(outputItem[6])
                        else:
                            self.breathRateHistoryAllAlgos['MinkBRate'] = collections.deque(
                                maxlen=self.NUM_BREATHRATEHISTORY)
                            self.breathRateHistoryAllAlgos['MinkBRate'].append(outputItem[6])

                        if outputItem[3]['event_type']=='fast_movement':
                            print ('Fast movement detectedby radar {0}'.format(self.radarNumber))

                        # ROS SEND
                        if ROS_AVAILABLE:
                            self.pubRD_Range.publish(self.currentRangeM)
                            breathingSignal = Float32MultiArray()
                            # breathingSignal.data = self.matlabVars['filteredPersonBinSignal']
                            breathingSignal.data = list(self.breathSignal)
                            self.pubBreathSig.publish(breathingSignal)
                            self.activityPublisher.publish(self.personState)
                        # END ROS SEND

                        # CLOUD SEND
                        dictData = {}
                        dictData["deviceId"] = 1
                        dictData["time"] = time.time()
                        dictData["BreathingSignal"] = list(np.abs(self.breathSignal))
                        dictData["BreathingRate"] = self.EMDBreathRate
                        dictData["Range"] = self.currentRangeM
                        dictData["Activity"] = self.personState
                        self.QDataDict.put(dictData)  #Uncomment to send data to cloud
                        # CLOUD SEND END
                    if self.personState !=  'Stationary':
                        print(self.personState)
                        print(bcolors.ERROR+'Person is moving, Breath signal not detectable by radar {0}'.format(self.radarNumber)+bcolors.ENDC)
                    else:
                        self.findBreathRateCombined()
                        if self.previousReferenceRate is not 0:
                            self.addToResultsTable(self.previousReferenceRate)
                        if self.OPEN_BBELT_FILE == False:
                            self.addToResultsTable(0)
                    if self.NON_REALTIME_MODE:
                        elapsedTime = time.time() - startTime
                        self.getUserInput()
                elif self.isMatlabBreathingDone() and self.isClassifierDone() and radarDataQ.empty():
                    print('Processing finished, waiting for new data... ')
                    if not self.resultsDataSaved:
                        self.saveResults()
                        self.resultsDataSaved = True
                        self.safeExit()
                        return
                    if self.NON_REALTIME_MODE:
                        elapsedTime = time.time() - startTime
                        self.getUserInput()
                if not self.NON_REALTIME_MODE:
                    elapsedTime = time.time() - startTime
                ## end of main while True loop
            self.safeExit()
        except KeyboardInterrupt:
            self.safeExit()

    def getUserInput(self):
        self.resumeEvent.clear()
        try:
            if self.ERSB_NN_TRAIN_AUTOMATICALLY:
                userInput = ''
                if self.dataClass == -20 and self.ERSB_NN_Trained == False: #-20 is used to train dataset
                    userInput = 'str' #save training/test set
                    self.ERSB_NN_Trained = True
                    print('Command to save training/test set issued')
                elif self.dataClass >= 0:
                    userInput = str(self.dataClass)
            else:
                userInput = raw_input('\nEnter action\n'
                                        'int: class of previous data [0:empty room,1:breathing,2:movements(NA)]\n'
                                        'tr: train ERSB\n'
                                        'trt: train and test ERSB\n'
                                        'te: test ERSB\n'
                                        'str: Save training set\n'
                                        'ste: save test set\n\n'
                                      )
                # print ('Entered: ' + str(userInput))
            inputType = self.get_type(userInput)
            if inputType== int:
                if int(userInput)>=0 and int(userInput)<=1:
                    self.procInputDict['action'] = 'Add to ERSB training set'
                    # print ('Class entered')
                    self.procInputDict['ERSB Class'] = int(userInput)
                else:
                    # print ('Invalid class, skipping this instance')
                    pass
                # print(bcolors.OKBLUE+'Entered: ' + str(userInput)+bcolors.ENDC)
            elif inputType == str:
                if userInput == 'tr':
                    # print('Training')
                    self.procInputDict['action'] = 'Train ERSB'
                if userInput == 'trt':
                    # print('Training and testing ERSB')
                    self.procInputDict['action'] = 'Train and test ERSB'
                if userInput == 'te':
                    # print('Testing ERSB')
                    self.procInputDict['action'] = 'Test ERSB'
                elif userInput == 'str':
                    # print('Saving training set')
                    self.procInputDict['action'] = 'Save ERSB train'
                elif userInput == 'ste':
                    # print('Saving test set')
                    self.procInputDict['action'] = 'Save ERSB test'
                else:
                    print('Unknown String, skipping this instance: '+userInput)
            else:
                print('Unknown input type, skipping this instance')
        except SyntaxError:
            print('Nothing was entered, skipping this instance')

        self.resumeEvent.set()

    def get_type(self,inputData):
        try:
            return type(literal_eval(inputData))
        except (ValueError,SyntaxError):
            return str

    def findBreathRateCombined(self):
        #parameters for scoring
        histBinStart = 5
        histBinEnd = 30
        histBinSize = 2

        HIST_HIGHEST_SCORE = 2
        HIST_SECOND_HI_SCORE = 1

        CHANGE_SCORE_MULTIPLIER = -10.5 #latest change is weighted by this
        CHANGE_SCORE_SECOND_MULTIPLIER = -0.5 #second from latest change is weighted by this
        CHANGE_SCORE_MULT_ADDITION = 0.1 #each change after second from latest is weighted by second_multiplier + this
        CHANGE_SCORE_NUM_CHANGES = 5
        #EG: -1*(most recent change) - 0.5*(second recent change) - (0.5+0.1)*(third recent change)
        #Till CHANGE_SCORE_NUM_CHANGES

        CONF_INTERVAL_MULTIPLIER = 0
        #END parameters for scoring
        # print (self.breathRatesAllAlgos)
        for key in self.breathRatesAllAlgos:
            if key not in self.breathRateAlgoScores:
                self.breathRateAlgoScores[key] = 0
        # print(self.breathRateAlgoScores)
        algo_list = self.breathRatesAllAlgos.values()
        print(list(algo_list))
        histogram = np.histogram(list(algo_list), bins=np.arange(histBinStart,histBinEnd,histBinSize))
        # print(type(histogram[0]))
        maxRangeStart = list(histogram[0]).index(max(histogram[0]))*histBinSize + histBinStart
        # print('Max range start: '+str(maxRangeStart))
        # print('HISTOGRAM###########################')
        # print (histogram)
        # print (self.breathRateHistoryAllAlgos)
        for key,value in self.breathRatesAllAlgos.items():
            if value <= maxRangeStart+histBinSize and value >= maxRangeStart:
                self.breathRateAlgoScores[key]+=HIST_HIGHEST_SCORE
            elif value <=maxRangeStart+2*histBinSize and value >= maxRangeStart-histBinSize:
                self.breathRateAlgoScores[key]+=HIST_SECOND_HI_SCORE
        # print('After histogram selection:')
        # print(self.breathRateAlgoScores)
        for algoName,values in self.breathRateHistoryAllAlgos.items():
            if len(values)>CHANGE_SCORE_NUM_CHANGES: #Wait till enough values are collected
                mean,confLow,confHigh = self.mean_confidence_interval(values)
                # print('Algo: '+str(algoName)+' Latest: '+bcolors.OKBLUE+"{:.2f}".format(values[-1])+bcolors.ENDC+
                #       ' Mean:'+"{:.2f}".format(mean)+
                #       bcolors.OKBLUE+' Conf: ['+"{:.2f}".format(confLow)+
                #       ', '+"{:.2f}".format(confHigh)+']'+bcolors.ENDC+
                #       ' #Val: '+str(len(values)))
                confScoreAddition = CONF_INTERVAL_MULTIPLIER*abs(confHigh-confLow)
                changeScoreAddition = CHANGE_SCORE_MULTIPLIER*abs(values[-1]-values[-2])
                for historyIndex in range(CHANGE_SCORE_NUM_CHANGES-1):
                    changeScoreAddition+=(CHANGE_SCORE_SECOND_MULTIPLIER+historyIndex*CHANGE_SCORE_MULT_ADDITION)*\
                        abs(values[-2-historyIndex]-values[-3-historyIndex])
                # print('Conf score: '+str(confScoreAddition)+' Change score: '+str(changeScoreAddition))
                self.breathRateAlgoScores[algoName]+=confScoreAddition
                self.breathRateAlgoScores[algoName]+=changeScoreAddition
        # latestRateFound = False
        for key, value in sorted(self.breathRateAlgoScores.items(), key=lambda kv: (kv[1], kv[0])): #lambda (k, v): (v, k)):
            # print ("%s: LatestVal: %s, Score: %s" % (key,self.breathRatesAllAlgos[key], value))
            self.currentBestAlgorithm = key
            self.breathRateFromScoring = self.breathRatesAllAlgos[key]
            self.breathRateNewestAlgorithmScore = value
            # print ("%s: %s" % (key, value))
            # print(self.breathRatesAllAlgos[key])
            # pass
                # print('Mean:'+str(mean)+' Conf: ['+str(confLow)+', '+str(confHigh)+']')

    def addToResultsTable(self,referenceRate):
        resultsDict = {}
        resultsOnlyDict = {} #With only results and not absolute error or any other
        resultsDict['Reference'] = referenceRate
        resultsDict['CurrentBestAlgo'] = self.currentBestAlgorithm
        resultsDict['AlgoScoringBreathRate'] = self.breathRateFromScoring
        resultsDict['Max score'] = self.breathRateNewestAlgorithmScore
        resultsDict['Scoring_Abs_Error'] = abs(referenceRate-self.breathRateFromScoring)
        resultsDict['Time(ms)'] = self.previousEndTime
        if 'Scoring' not in self.resultsOnlyCSVTable.columns:
            self.resultsOnlyCSVTable['Scoring'] = []
        resultsOnlyDict['Scoring'] = self.breathRateFromScoring
        for key in self.breathRatesAllAlgos:
            if key not in self.resultsOnlyCSVTable.columns:
                self.resultsOnlyCSVTable[key] = []
            if key not in self.resultsCSVTable.columns:
                self.resultsCSVTable[key] = []
                self.resultsCSVTable[str(key)+'_Abs_Err'] = []
                self.resultsCSVTable[str(key)+'_score'] = []
            resultsOnlyDict[key] = self.breathRatesAllAlgos[key]
            resultsDict[key] = self.breathRatesAllAlgos[key]
            resultsDict[str(key)+'_Abs_Err'] = abs(referenceRate-self.breathRatesAllAlgos[key])
            resultsDict[str(key)+'_score'] = self.breathRateAlgoScores[key]
        self.resultsCSVTable = self.resultsCSVTable.append(resultsDict,ignore_index=True)
        self.resultsOnlyCSVTable = self.resultsOnlyCSVTable.append(resultsOnlyDict, ignore_index=True)

    def saveResults(self):
        # print(self.resultsCSVTable)
        absErrorsDF = self.resultsCSVTable[self.resultsCSVTable.columns[self.resultsCSVTable.columns.to_series().str.contains('_Abs_Err')]]
        # resultsOnlyDF = self.resultsCSVTable[self.resultsCSVTable.columns[~self.resultsCSVTable.columns.to_series().str.contains('_Abs_Err')]]
        emptyRowDict = {'___':'___','---':'---'}
        emptyRowDF = pd.DataFrame(emptyRowDict.items())
        with open('results/'+time.strftime(u"%Y%m%d-%H%M%S")+'_'+self.RESULTS_FILE_NAME+'.csv','w') as handle:
            self.resultsCSVTable.to_csv(handle,index=False)
            emptyRowDF.to_csv(handle, index=False, header=False)

            titleDictDF = pd.DataFrame({'Table type': 'Breath rate results'}.items())
            titleDictDF.to_csv(handle, index=False, header=False)
            self.resultsOnlyCSVTable.to_csv(handle,index=False)
            emptyRowDF.to_csv(handle, index=False, header=False)

            titleDictDF = pd.DataFrame({'Table type': 'Absolute Error'}.items())
            titleDictDF.to_csv(handle, index=False, header=False)
            absErrorsDF.to_csv(handle,index=False)
            emptyRowDF.to_csv(handle, index=False, header=False)

            titleDictDF = pd.DataFrame({'Table type': 'Mean of Absolute Error'}.items())
            titleDictDF.to_csv(handle, index=False, header=False)
            meansOfAbsErrors=absErrorsDF.mean()
            meansOfAbsErrors.to_csv(handle)
            emptyRowDF.to_csv(handle,index=False,header=False)

            titleDictDF = pd.DataFrame({'Table type':'Variance of Absolute Error'}.items())
            titleDictDF.to_csv(handle,index=False,header=False)
            varOfAbsErrors = absErrorsDF.var()
            varOfAbsErrors.to_csv(handle)
            emptyRowDF.to_csv(handle, index=False, header=False)

            titleDictDF = pd.DataFrame({'Table type': 'Standard deviation of Absolute Error'}.items())
            titleDictDF.to_csv(handle, index=False, header=False)
            stdOfAbsErrors = absErrorsDF.std()
            stdOfAbsErrors.to_csv(handle)
            emptyRowDF.to_csv(handle, index=False, header=False)

            titleDictDF = pd.DataFrame({'Table type': 'Correlation table'}.items())
            titleDictDF.to_csv(handle, index=False, header=False)
            corrTable = self.resultsOnlyCSVTable.corrwith(self.resultsCSVTable['Reference'])
            corrTable.to_csv(handle)
            emptyRowDF.to_csv(handle, index=False, header=False)

            self.rangeCSVTable.to_csv(handle,index=False)

        # self.resultsCSVTable.to_csv('results/'+self.RESULTS_FILE_NAME+time.strftime(u"%Y%m%d-%H%M%S")+'.csv')

    def mean_confidence_interval(self, data, confidence=0.95):
        a = 1.0 * np.array(data)
        n = len(a)
        m, se = np.mean(a), scipy.stats.sem(a)
        h = se * sp.stats.t.ppf((1 + confidence) / 2., n - 1)
        return m, m - h, m + h

    def safeExit(self):
        print ('Stopping radarThread')
        self.stopEvent.set()
        self.radarThread.join()
        if MATLAB_AVAILABLE:
            print ('Stopping Matlab')
            self.futureOut.cancel()
            self.engProcRadar.quit()
        print ('Stopping breath rate processing')
        self.procStopEvent.set()
        self.procOutputQ.close()
        self.procInputQ.close()
        self.processBRProc.join()
        print ('Safe exit complete')

    # def saveData(self):
    #     startTime = time.time()
    #     print (bcolors.ERROR+'FIX FILE SAVING'+bcolors.ENDC)
    #     # with open(self.DIRECTORY_PATH + \
    #     #                   self.FILE_NAME_APPEND + '_'+ \
    #     #                   time.strftime(u"%Y%m%d-%H%M%S") + '_.csv', 'w') as csvFile:
    #     #     csvWriter = csv.writer(csvFile)
    #     #     for rdDataRow in self.radarDataDeck:
    #     #         csvWriter.writerow(rdDataRow)
    #     self.radarDataDeck.clear()
    #     # print 'Data saved...................'
    #     elapsedTime = time.time() - startTime
    #     print ('Elapsed %f ms' % (elapsedTime * 1000))
    def saveData(self):
        startTime = time.time()
        with open(self.DIRECTORY_PATH + \
                          self.FILE_NAME_APPEND + '_' + \
                          time.strftime(u"%Y%m%d-%H%M%S") + '_.csv', 'w') as csvFile:
            csvWriter = csv.writer(csvFile)
            for rdDataRow in self.radarDataDeck:
                csvWriter.writerow(rdDataRow)
        self.radarDataDeck.clear()
        print ('Data saved...................{0}'.format(self.radarNumber))
        elapsedTime = time.time() - startTime
        print ('Elapsed %f ms' % (elapsedTime * 1000))

class RadarThread(threading.Thread):
    def __init__(self, threadID, name, object = None):
        threading.Thread.__init__(self)
        self.name = name
        self.threadID = threadID
        self.object = object
        self.resumeEvent = self.object.mainResumeEvent
        self.pauseEvent = self.object.mainPauseEvent

    def run(self):
        self.object.main()




if __name__ == '__main__':
    os.system('roscore &')
    time.sleep(5)
    os.system('python dataCollection1.py &')
    os.system('python dataCollection2.py &')
    # Create required folders
    if ROS_AVAILABLE:
        rospy.init_node('realtimeBreathing')

    if not os.path.exists('results'):  # Save as JPEG in folder
        os.makedirs('results')
    if not os.path.exists('logs'):
        os.makedirs('logs')
    rd1 = MainClass(radarNumber = 1, port='/dev/ttyS3')
    rd2 = MainClass(radarNumber = 2, port='/dev/ttyS5')
    rd_thread_1 = RadarThread(11,'sensorRadarThread1',object=rd1)
    rd_thread_2 = RadarThread(22,'sensorRadarThread2',object=rd2)
    # rd_thread_1.pauseEvent.set()
    # rd_thread_2.pauseEvent.set()
    rd_thread_1.start()
    rd_thread_2.start()
    # time.sleep(16)
    # rd_thread_1.pauseEvent.clear()
    # rd_thread_2.pauseEvent.clear()
    # rd_thread_1.resumeEvent.set()
    # rd_thread_2.resumeEvent.set()
    # while True:
    #     rd_thread_1.pauseEvent.wait()
    #     rd_thread_1.pauseEvent.clear()
    #     rd_thread_1.resumeEvent.set()
    #     rd_thread_2.pauseEvent.wait()
    #     rd_thread_2.pauseEvent.clear()
    #     rd_thread_2.resumeEvent.set()


    rd_thread_1.join()
    rd_thread_2.join()
