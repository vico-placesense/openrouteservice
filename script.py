import requests
import pandas as pd

coordinates = pd.read_csv('coordinates_berlin.csv')
# coordinates = pd.read_csv('coordinates.csv')

locations = []
for i,j in zip(coordinates['avg_longitude'], coordinates['avg_latitude']):
    locations.append([i,j])
locations = locations[:10]

# body = {"locations":[[8.67918398548546,49.403812699619749],[8.725236455812885,49.392627223089832]]}
body = {"locations":locations}

headers = {
    'Accept': 'application/json, application/geo+json, application/gpx+xml, img/png; charset=utf-8',
    'Authorization': '5b3ce3597851110001cf624867d60a4709b54dbeb853215f4841061a',
    'Content-Type': 'application/json; charset=utf-8'
}
call = requests.post('http://localhost:8080/ors/v2/matrix/driving-car', json=body, headers=headers)

print(call.status_code, call.reason)
print(call.text)
