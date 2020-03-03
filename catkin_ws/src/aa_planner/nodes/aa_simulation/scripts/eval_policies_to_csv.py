#!/usr/bin/env python
import sys
import csv
import os
import json

rows=[]
#Fields we want in initial columns, to simplify analysis
field_list=[
    "Experiment", "Success Percentage",
    "Max Speeding Coefficient", "Mean Speeding Violation Frac", "Mean Abs Speed Error",
    "Max Incorrect Steering", "Mean Steering Violation",
    "Mean Steer Cmd Jerk", "Mean Speed Cmd Jerk",
    "vv: algo",
    "Mean Speed Error",
    "vv: dt",
    "vv: lambda1",
    "vv: success_reward",
    "vv: pretrained",
    "vv: reset_variance",
    "vv: speeding_penalty_coeff",
    "vv: max_speeding_coeff",
]

fields=set(field_list)
def add(d,key,val):
    if key not in fields:
        fields.add(key)
        field_list.append(key)
    d[key] = val

def add_experiment(exp, eval_policy, variant):
    d = {}
    rows.append(d)
    add(d,'Experiment',exp)
    with open(eval_policy, "r") as f:
        for line in f:
            line = line.strip()
            if line:
                if line [-1] == ':' and line[0:24] == 'Averaged statistics over':
                    add(d, 'Num of rollouts', int(line.split()[3]))
                else:
                    (title, value) = line.split(':')
                    value = value.strip()
                    vals = value.split(' +/- ')
                    if len(vals) == 1:
                        if value[-1] == '%':
                            value = value[:-1]
                        add(d, title, float(value.split()[0]))
                    else:
                        add(d, title, float(vals[0]))
                        add(d, title+", StdDev", float(vals[1]))
    with open(variant, "r") as f:
        vv = json.load(f)
        for k in vv:
            add(d, 'vv: ' + k, vv[k])

for fname in sys.argv[1:]:
    fname = os.path.abspath(os.path.realpath(fname))
    for f in os.listdir(fname):
        if f[0:11] == 'eval_policy':
            suffix = f[11:-4]
            exp = fname
            if len(suffix)>0:
               exp += ":" + suffix
            add_experiment(exp, fname + "/" + f, fname + "/variant" + suffix + ".json")

prefix = os.path.commonprefix([d['Experiment'] for d in rows])
if prefix:
    prefix = prefix[0:prefix.rindex('/')+1]
for d in rows:
    d['Experiment'] = d['Experiment'][len(prefix):]
csv_writer = csv.DictWriter(sys.stdout, fieldnames=field_list, dialect='excel')
csv_writer.writeheader()
csv_writer.writerows(rows)
