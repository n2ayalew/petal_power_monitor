# importing the requests library 
import requests 
import json

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

'''
preprocess data

assumes data is in csv format with first column being timestamp
and second column being the active power at that moment
'''
def load_aggregate_data_from_file(filepath, house, channel):
    filename = filepath + '/' + house + '/' + channel + '.dat'
    #agg_df = pd.read_table(filename, sep=' ',header= None,names = ['unix_date',channel])
    df = pd.read_csv(filename, sep=' ', names=['timestamp', channel], dtype={channel:np.float64}, header=None)
    df.dropna(inplace=True)
    df = df[(np.abs(stats.zscore(df)) < 3).all(axis=1)] # remove outliers
    df.dropna(inplace=True)
    df['timestamp'] = df['timestamp'] - df['timestamp'][0] 
    df = df.set_index(['timestamp'])
    #df.index = pd.to_datetime(df.index, unit='s')
    df.index = pd.to_timedelta(df.index, unit='s')
    #df['timestamp'] = agg_df['timestamp'].map(convert_to_datetime)
    #df = df.set_index('date').drop('unix_date', axis = 1)
    return df


# defining the api-endpoint  
URL = "https://flask-petal.herokuapp.com/appliancesData"
    
dstore = DataStore('/home/nathaniel/nilmtk/data/petal', 'house_5')
dstore.create_store() # Get all channels

app_test_list = []
app_test_list.append(Appliance(1, dstore.get_channel(1))) 

with open('petal_models/house_5.pkl', 'rb') as in_file:
    fhmm = pk.load(in_file)
#fhmm = FHMM()
#fhmm.import_model('petal_models/house_5_fhmm.model.txt')
predictions = pd.DataFrame()

predictions = fhmm.disaggregate(app_test_list[0].power_data, predictions, 1)
aggregate_signal = app_test_list[0].power_data
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
print("Response:%s" % resp)
