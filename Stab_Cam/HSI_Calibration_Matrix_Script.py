import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import eig, inv


# ~~ Pull CSV data ~~ #
def pull_csv(filename):
    data = np.loadtxt(filename, delimiter=',', skiprows=0)
    return data

# ~~ Plot 3d ~~ #
def plot_3d(data, title="Magnetometer Data", calibrated=False):
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], s=2)
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    if calibrated:
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
    plt.show()

# ~~ Hard Iron Offset ~~ #
def compute_offset(data):
    return np.mean(data, axis=0)

 # ~~ Soft Iron Correction ~~ #
def compute_correction(data, offset):
    centered = data - offset
    cov = np.cov(centered, rowvar=False)
    eig_vals, eig_vecs = eig(cov)
    D = np.diag(1 / np.sqrt(eig_vals))
    correction_matrix = eig_vecs @ D @ eig_vecs.T
    return correction_matrix

# ~~ Calibrate Apply ~~ #
def apply_calibration(data, offset, correction):
    return np.dot(data - offset, correction.T)



# ~~ Calibration Pipline ~~ #
def calibrate(filename):
    data = pull_csv(filename)
    plot_3d(data, "Raw Magnetometer Data")

    offset = compute_offset(data)
    print("Hard iron offset:\n", offset)

    correction = compute_correction(data, offset)
    print("Soft iron correction matrix:\n", correction)

    calibrated = apply_calibration(data, offset, correction)
    plot_3d(calibrated, "Calibrated Magnetometer Data", calibrated=True)

    return offset, correction



# ~~ Run calibration ~~ #
if __name__ == "__main__":
    FILENAME = "Stab_Cam\mag_fig8.csv" # -- change to file name -- #
    calibrate(FILENAME)
