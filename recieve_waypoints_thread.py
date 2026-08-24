from telemetry import RF
import params
import ast

rf = RF(params.rf_)

while True:
    rf.recieve_data()

    s = rf.rf_data

    params.square_profile = ast.literal_eval(s)

    print(params.square_profile)

    print(1)

    print(2)

    print(3)