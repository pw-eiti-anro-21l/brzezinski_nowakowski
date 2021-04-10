import json
import yaml
from math import pi, sin, cos, sqrt

with open('TabelaDH.json') as json_file:
    data = json.load(json_file)
    a_prev = 0
    parameters = {}
    for index, row in enumerate(data):
        a = row['a']
        alfa = row['alfa'] * pi / 180
        d = row['d']
        theta = row['theta'] * pi / 180
        origin =  str(a) + ' ' + str(d * sin(alfa)) + ' ' + str(d * cos(alfa))
        rpy = str(alfa) + " " + str(sin(alfa) * theta)  + " " + str(cos(alfa) * theta) 
        parameters['joint' + str(index+1)] = [origin, rpy]
        length = str(sqrt(a**2 + d**2))
        origin =  str(a/2) + ' ' + str(d * sin(alfa) / 2) + ' ' + str(d * cos(alfa) / 2)
        rpy = "0 0 0"
        parameters['link' + str(index+1)] = [origin, rpy, length]

with open(r'TabDH.yaml', 'w') as file:
    documents = yaml.dump(parameters, file)