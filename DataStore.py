import pandas as pd
import numpy as np
from scipy import stats
from datetime import datetime
try:
    import cPickle as pk
except:
    import pickle as pk

def convert_to_datetime(x):
    return datetime.fromtimestamp(x)

'''
preprocess data

assumes data is in csv format with first column being timestamp
and second column being the active power at that moment
'''
def load_aggregate_data(filepath, house, channel):
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

# resample data at desired frequency interval and pivot data
def resample_and_pivot(df,resample_freq):
    avg_df = df.resample(resample_freq).mean()
    avg_df['time'] = avg_df.index.time
    avg_df['day'] = avg_df.index.date
    agg_pivot = avg_df.reset_index().pivot('time','day','meter_reading')
    return avg_df, agg_pivot

class DataStore(object):
    '''
    class to store measured power by channel and predictions when fitting the model
    '''
    def __init__(self, path, house):
        self.house = house
        self.channels = {}
        self.labels = None
        self.path = path

        self.predictions = pd.DataFrame()

    def create_store(self, select_channels = None):
        """
        create datastore for selected channels
        :param select_channels: list of channel ID's to select, if none select all channels in the house
        :return: self
        """
        self.create_labels()
        self.create_channels(select_channels)


    def create_labels(self):
        """
        read the labels of all channels from the labels.dat file
        """
        filename = self.path + '/' + self.house + '/labels.dat'
        self.labels = pd.read_csv(filename, sep=' ', names = ['channel_id','channel_desc'], header=None)
        #self.labels = pd.read_table(filename, sep=' ',header=None, names = ['channel_id','channel_desc'])

    def create_channels(self, select_channels = None):
        '''
        creates dictionary of pandas dataframes
        :param select_channels: list of channel id's to select, if None - select all channels in the house
        :return: populate self.channels dictionary
        '''
        if not select_channels:
            select_list = self.labels.channel_id.values
        else:
            select_list = select_channels
        for id in select_list:
            name = 'channel_' + str(id)
            print("Creating data frame for %s " % (name))
            self.channels[id] = load_aggregate_data(self.path, self.house, name)
            print("Done")

    def create_combined_df(self, start, end, freq=None, select_channels = None):
        '''
        resample dataframe into longer time periods and combine multiple channels into 1 dataframe
        :param freq: resampling frequency
        :param start: start date of observations
        :param end: end date of observations
        :param select_channels: list of channel ID's
        :return: aggregated dataframe
        '''
        if not select_channels:
            select_list = self.channels.keys()
        else:
            select_list = select_channels
        agg_df = self.select_window(select_list.pop(), start, end, resample_freq = freq)
        while select_list:
            ch_df = self.select_window(select_list.pop(), start, end, resample_freq = freq)
            agg_df = agg_df.join(ch_df, how = 'union')
        return agg_df

    def select_window(self, channel_id, start, end, resample_freq = None):
        '''
        Select subset of observations by specified dates
        :param channel_id: integer channel ID
        :param start: start date of observations
        :param end: end date of observations
        :param resample_freq: resample frequency
        :return: new dataframe
        '''
        if resample_freq:
            return self.channels[channel_id][start:end].resample(resample_freq).mean()
        return self.channels[channel_id][start:end]

    def select_top_k(self, k, period_start, period_end):
        totals = []
        channels = self.channels.keys()
        for channel in channels:
            total_power = self.channels[channel][period_start:period_end].sum().values[0]
            totals.append(total_power)

        idx = np.argsort(np.array(totals).flatten()).tolist()[::-1][:k]
        return [channels[i] for i in idx]


    def get_channel(self, channel_id):
        return self.channels[channel_id]

    def get_nchannels(self):
        return len(self.channels.keys())


    def pickle_store(self, dest):
        with open(dest,'w') as f:
            pk.dump(self,f)


