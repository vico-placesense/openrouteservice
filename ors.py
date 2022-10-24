import requests
import pandas as pd
import numpy as np
import time
import json

coordinates = pd.read_csv('coordinates/den_haag.csv')

locations = []
for i,j in zip(coordinates['avg_longitude'], coordinates['avg_latitude']):
    locations.append([i,j])
#locations = locations[:10]
body = {"locations":locations,"metrics":["distance", "duration"],"sources":[1]}

headers = {
    'Accept': 'application/json, application/geo+json, application/gpx+xml, img/png; charset=utf-8',
    'Authorization': '5b3ce3597851110001cf624867d60a4709b54dbeb853215f4841061a',
    'Content-Type': 'application/json; charset=utf-8'
}

start = time.time()
call = requests.post('http://localhost:8080/ors/v2/matrix/driving-car', json=body, headers=headers)
result = json.loads(call.text)
end = time.time()

if call.status_code == 200:
    print("\nRequest successfull with response " + str(call.status_code))
    print("\nNumber of geolocations: " + str(len(locations)))
    print("Execution time: " + str(round(end-start, 3)) + " seconds")
    distances = np.array(result['distances'])
    print("Resulting matrix of shape: " + str(distances.shape))
else:
    print("\nRequest failed with response " + str(call.status_code))
    print("\nError: " + str(result['error']))