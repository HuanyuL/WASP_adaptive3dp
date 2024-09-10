import requests
import json


response_fdr = requests.post(
    "http://192.168.0.129/api/printer/printhead",
    headers={
        "Host": "user",
        "Content-Type": "application/json",
        "Accept": "application/json",
        "X-Api-Key": "B57A663A581543CFAA53BD4CD8532F8B",
    },
    data=json.dumps({"command": "feedrate", "factor": 102}),
)

print(response_fdr)
