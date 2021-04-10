import argparse
import json
import yaml
from math import pi, sin, cos, sqrt

def dh_to_rpy(dh):
    urdf = {}
    for index, row in enumerate(dh):
        a = row['a']
        alfa = row['alfa'] * pi / 180
        d = row['d']
        theta = row['theta'] * pi / 180
        origin =  str(a) + ' ' + str(d * sin(alfa)) + ' ' + str(d * cos(alfa))
        rpy = str(alfa) + " " + str(sin(alfa) * theta)  + " " + str(cos(alfa) * theta) 
        urdf['joint' + str(index+1)] = [origin, rpy]
        length = str(sqrt(a**2 + d**2))
        origin =  str(a/2) + ' ' + str(d * sin(alfa) / 2) + ' ' + str(d * cos(alfa) / 2)
        rpy = "0 0 0"
        urdf['link' + str(index+1)] = [origin, rpy, length]
    return urdf

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-i", "--Input", help = "Path to json file with dh table", required=True)
    arg_parser.add_argument("-o", "--Output", help = "Path where to save yaml file with urdf coordinates", required=True)
    args = arg_parser.parse_args()
    with open(args.Input) as json_file:
        urdf = dh_to_rpy( json.load(json_file) )

    with open(args.Output, 'w') as file:
        documents = yaml.dump(urdf, file)
