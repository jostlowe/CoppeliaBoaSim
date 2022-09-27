import pandas as pd
import numpy as np
import json


N_TIMES = 10
N_JOINTS = 5

measurement = {
    "sim_time": 0.0,
    "link_sensors": [
        {"sim_time": 0, "cf_x": 0, "cf_y": 0, "acc_x": 0,
            "acc_y": 0, "orientation": 0, "joint_angle": 0},
        {"sim_time": 0, "cf_x": 0, "cf_y": 0, "acc_x": 0,
            "acc_y": 0, "orientation": 0, "joint_angle": 0},
    ],
    "env_sensors": {
        "fx": 0,
        "fy": 0
    }
}

a = [measurement for _ in range(100)]


with open("tissetass.json", "w") as f:
    f.write(json.dumps(a))


b = pd.read_json("tissetass.json")
print(b)


index = pd.MultiIndex.from_product(
    [range(N_TIMES), range(N_JOINTS)],
    names=["", "link_n"]
)


data = np.random.randn(N_TIMES*N_JOINTS, len(sensor_names))
dataframe = pd.DataFrame(data, index=index, columns=sensor_names)
print(dataframe)

dataframe.to_csv("./tissemann.csv")
