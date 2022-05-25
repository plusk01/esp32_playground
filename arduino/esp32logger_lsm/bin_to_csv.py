import struct

import numpy as np

if __name__ == '__main__':
    
    binfile = '/tmp/data.bin'

    data = []

    with open(binfile, 'rb') as file:
        filecontent = file.read()

        for t_sec, seq, ax, ay, az, gx, gy, gz, mx, my, mz  in struct.iter_unpack("<fI3f3f3f", filecontent):

            data.append([seq, t_sec, ax, ay, az, gx, gy, gz, mx, my, mz])

    data = np.array(data)

    np.savetxt('/tmp/bindata.csv', data, fmt='%f', delimiter=',')