#!/usr/bin/env python3

import theano
import numpy as np
import argparse
from pickle import loads
from json import dumps

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--infile', required=True)
    parser.add_argument('-o', '--outfile', required=True)

    args = parser.parse_args()

    with open(args.infile, 'rb') as f:
        s = f.read()

    f = loads(s)

    W0 = f.finder['hidden_0.W'].data.tolist()
    b0 = f.finder['hidden_0.b'].data.tolist()
    W1 = f.finder['hidden_1.W'].data.tolist()
    b1 = f.finder['hidden_1.b'].data.tolist()
    W_out = f.finder['output.W'].data.tolist()
    b_out = f.finder['output.b'].data.tolist()
    
    d = {'kernel0': W0,
         'bias0': b0,
         'kernel1': W1,
         'bias1': b1,
         'kernel2': W_out,
         'bias2': b_out}
         
    with open(args.outfile, 'w') as f:
        f.write(dumps(d))
