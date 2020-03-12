##---------------------------------------------------------------------------------------------------------------------------------------*
##Alpha trimmer filter, uses the stream to take values of "windowsize" and "alpha" to filter unusually large or small data from the stream.
##---------------------------------------------------------------------------------------------------------------------------------------*

def alpha_mean(window, alpha): #Parsing mean data from the stream
    cut = alpha//2
    data1 = sorted(window) #sorting collected list
    data = sorted(window)[cut:-cut] #Trimming the window
    print(data1)
    store =[]
    for item in data:
        store.append(int(item))
    return sum(store)/len(data)

def alphatrimmer(window_size, alpha, sensor_stream):
    window = []
    data_list = [] #empty list to store data
    for item in sensor_stream:
        window.append(item)
        if len(window) >= window_size: #condition for filling the data to window size
            break
        x = ''.join(window)
        x = float(x)
        data_list.append(x)
##        if len(data_list)< window_size:
##            break
##        else:
##            yield alpha_mean(data_list, alpha)

    yield alpha_mean(data_list, alpha)

    for item in sensor_stream: #clearing windows and outputing filtered data
        window.pop(0)
        window.append(item)
        yield alpha_mean(window,alpha)
