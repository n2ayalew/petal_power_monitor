# importing the requests library 
import requests 
import json

# defining the api-endpoint  
URL = "https://flask-petal.herokuapp.com/"
    
# your source code here 
source_code = ''' 
print("Hello, world!") 
a = 1 
b = 2 
print(a + b) 
'''

# data to be sent to api 
data = {'timestamp':, 
        'toaster':, 
        'hair dryer':, 
        'hair iron':, 
        } 

# sending post request and saving response as response object 
r = requests.post(url = URL, json = data) 
          
# extracting response text  
resp = r.text 
print("Response:%s" % resp)
