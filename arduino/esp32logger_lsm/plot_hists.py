"""
Plot IMU data and histograms

Parker Lusk
5 June 2022
"""
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats

if __name__ == '__main__':
    csvfile = '/tmp/bindata.csv'

    data = np.loadtxt(csvfile, delimiter=',')

    seq = data[:,0]
    t = data[:,1]
    acc = data[:,2:5]
    gyr = data[:,5:8]

    dt = np.mean(np.diff(t))
    Fs = 1. / dt

    fig, ax = plt.subplots()
    ax.plot(seq, acc)
    ax.grid(alpha=0.3)
    ax.set_ylabel('Acceleration [m/s/s]')
    ax.set_xlabel('Sample Number')
    ax.set_title(f"Fs = {Fs:.0f} Hz (dt = {dt*1e3:.3f} ms)")

    fig, ax = plt.subplots()
    ax.plot(seq, gyr)
    ax.grid(alpha=0.6)
    ax.set_ylabel('Angular Velocity [rad/s]')
    ax.set_xlabel('Sample Number')
    ax.set_title(f"Fs = {Fs:.0f} Hz (dt = {dt*1e3:.3f} ms)")


    # center data
    cacc = acc - np.mean(acc, axis=0)
    cgyr = gyr - np.mean(gyr, axis=0)

    fig, ax = plt.subplots(sharex=True, ncols=3, figsize=(16,6))
    for i in range(cacc.shape[1]):
        n, bins, patches = ax[i].hist(cacc[:,i], bins=50, density=True, alpha=0.8)

        mu, std = scipy.stats.norm.fit(cacc[:,i])

        xmin, xmax = ax[i].get_xlim()
        x = np.linspace(xmin, xmax, 100)
        f = scipy.stats.norm.pdf(x, mu, std)

        ax[i].plot(x, f, 'k', linewidth=2)

        ax[i].set_title(f"accel {chr(88+i)}: σ={std:.3f}")

        ax[i].grid(alpha=0.3)
        ax[i].set_xlabel("noise [m/s/s]")

    fig, ax = plt.subplots(sharex=True, ncols=3, figsize=(16,6))
    for i in range(cgyr.shape[1]):
        n, bins, patches = ax[i].hist(cgyr[:,i], bins=50, density=True, alpha=0.8)

        mu, std = scipy.stats.norm.fit(cgyr[:,i])

        xmin, xmax = ax[i].get_xlim()
        x = np.linspace(xmin, xmax, 100)
        f = scipy.stats.norm.pdf(x, mu, std)

        ax[i].plot(x, f, 'k', linewidth=2)

        ax[i].set_title(f"gyr {chr(88+i)}: σ={std:.3f}")

        ax[i].grid(alpha=0.3)
        ax[i].set_xlabel("noise [rad/s]")

    plt.show()