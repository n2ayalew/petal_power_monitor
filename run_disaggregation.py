#!/usr/bin/env python

# importing the requests library 
import requests 
import json
import time
import stat
import sysv_ipc
import struct 
from scipy import stats

try:
    import cPickle as pk
except:
    import pickle as pk
import pandas as pd
import numpy as np
from FHMM import FHMM
from preprocessing import Create_combined_states, Appliance, train_test_split,create_matrix
from DataStore import DataStore
import math

# Vars for MsgQueue
QueueKey=8500
PermissionsError = sysv_ipc.PermissionsError
ExistentialError = sysv_ipc.ExistentialError
BusyError = sysv_ipc.BusyError

# defining the api-endpoint  
URL = "https://flask-petal.herokuapp.com/appliancesData"

'''
preprocess data

assumes data is in csv format with first column being timestamp
and second column being the active power at that moment
'''
def load_aggregate_data(samples):
    #filename = filepath + '/' + house + '/' + channel + '.dat'
    #agg_df = pd.read_table(filename, sep=' ',header= None,names = ['unix_date',channel])
    #df = pd.read_csv(filename, sep=' ', names=['timestamp', channel], , header=None)
    df = pd.DataFrame(samples, dtype=np.float64, columns=['timestamp', 'power'])
    df.dropna(inplace=True)
    #print(df[(np.abs(stats.zscore(df)) >= 3).all(axis=1)])
    #print(df.head())
    df = df[(np.abs(stats.zscore(df)) < 3).all(axis=1)] # remove outliers
    df.dropna(inplace=True)
    df['timestamp'] = df['timestamp'] - df['timestamp'][0] 
    df = df.set_index(['timestamp'])
    #df.index = pd.to_datetime(df.index, unit='s')
    df.index = pd.to_timedelta(df.index, unit='s')
    df = df.resample("s").mean().interpolate(method='linear') // 10**9
    #df.index = df.index.astype(np.uint64) // 10**9
    #df['timestamp'] = agg_df['timestamp'].map(convert_to_datetime)
    #df = df.set_index('date').drop('unix_date', axis = 1)
    return df



    
#dstore = DataStore('/home/nathaniel/nilmtk/data/petal', 'house_5')
#dstore.create_store() # Get all channels

# This isn't needed cause we end up just using the dataframe anyways
# aggregate_signal = Appliance(1, dstore.get_channel(1))

#fhmm = FHMM()
#fhmm.import_model('petal_models/house_5_fhmm.model.txt')

with open('petal_models/house_5.pkl', 'rb') as in_file:
    fhmm = pk.load(in_file)

try:
    mq = sysv_ipc.MessageQueue(QueueKey, sysv_ipc.IPC_CREAT, mode=0o666)
except ExistentialError:
    print("ERROR: message queue creation failed")

if mq.current_messages > 0:
    mq.remove()
    mq = sysv_ipc.MessageQueue(QueueKey, sysv_ipc.IPC_CREAT, mode=0o666)

disaggregation_running = True
disaggregate_delay_sec = 120
#timestamps = np.zeros(5000, dtype=np.float64)
#power = np.zeros(5000, dtype=np.float64)
samples = {'timestamp': [], 'power': []}
i = 0
disaggregate_timer_start = time.time()
while (disaggregation_running):

    mq_msg, priority = mq.receive(type=1)
    ts_sec_f, power_f = struct.unpack("dd", mq_msg)
    samples['timestamp'].append(ts_sec_f)
    samples['power'].append(power_f)
    #timestamps[i] = ts_sec_f
    #power[i] = power_f
    #i += 1
    print(samples)
    if (time.time() - disaggregate_timer_start):
        predictions = pd.DataFrame()
        aggregate_signal = load_aggregate_data(samples)
        predictions = fhmm.disaggregate(aggregate_signal, predictions, 1)
        predictions.columns = ['hair iron', 'hair dryer', 'toaster']

        # data to be sent to api 
        data = { 'toaster': predictions['toaster'].tolist(), 
                'hair dryer': predictions['hair dryer'].tolist(), 
                'hair iron': predictions['hair iron'].tolist() 
                } 

        # sending post request and saving response as response object 
        r = requests.post(url = URL, json = data) 
                  
        # extracting response text  
        resp = r.text 
        #print("Response:%s" % resp)
        samples['timestamp'].clear()
        samples['power'].clear()
        
        disaggregate_timer_start = time.time()

mq.remove()
