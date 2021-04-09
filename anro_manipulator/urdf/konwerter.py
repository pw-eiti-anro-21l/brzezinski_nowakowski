import json
import yaml
from math import pi

with open('TabelaDH.json') as json_file:
    data = json.load(json_file)
    a_prev = 0
    for p in data['DH']:
        if p['n'] > 1:
            for m in data['DH']:
                if m['n'] == p['n']+1:
                    p['a'] = m['a']
        if p['n'] == 5:
            p['a'] = 0
    parameters = {}
    for p in data['DH']:
        a = p['a']
        alfa = p['alfa']
        d = p['d']
        theta = p['theta']
        n = p['n']
        if n == 1:
            origin = "0 0 " + str(d/2)
            rpy = "0 0 0"
            length = d  
            parameters['base'] = [origin, rpy, length]
            origin = "0 0 0" 
            rpy = "0 0 " + str(pi*theta/180)
            parameters['joint' + str(n)] = [origin, rpy]
            d_prev = d
        elif n < 4:
            origin =  str(a/2) + " 0 0"
            rpy = "0 0 0"
            length = a  
            parameters['link' + str(n+1)] = [origin, rpy, length]
            origin = str(a_prev) + " 0 " + str(d_prev)
            if alfa != 0:
                rpy = str(pi*alfa/180) + " " + str(pi*theta/180) + " 0"
            else:
                rpy = str(pi*alfa/180) + " 0 " + str(pi*theta/180)
            parameters['joint' + str(n)] = [origin, rpy]
            a_prev = a
            d_prev = 0
        elif n == 4:
            a = a - 0.05
            origin =  str((a)/2) + " 0 0"
            rpy = "0 0 0"
            length = a
            parameters['link' + str(n+1)] = [origin, rpy, length]
            origin = str(a_prev) + " 0 0" 
            rpy = "0 0 " + str(pi*theta/180)
            parameters['joint' + str(n)] = [origin, rpy]
            a_prev = a
            a = 0.05
            n = 5
            origin = "0 0 " + str((a)/2)
            rpy = "0 0 0"
            length = a
            parameters['link' + str(n+1)] = [origin, rpy, length]
            origin = str(a_prev) + " 0 0" 
            rpy = "0 " + str(pi*1/2) + " 0"
            parameters['joint' + str(n)] = [origin, rpy]

with open(r'TabDH.yaml', 'w') as file:
    documents = yaml.dump(parameters, file)