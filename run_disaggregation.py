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


# defining the api-endpoint  
URL = "https://flask-petal.herokuapp.com/appliancesData"
    
dstore = DataStore('/home/nathaniel/nilmtk/data/petal', 'house_5')
dstore.create_store() # Get all channels

app_test_list = []
app_test_list.append(Appliance(1, dstore.get_channel(1))) 

fhmm = FHMM()
fhmm.import_model('petal_models/house_5_fhmm_model.txt')
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