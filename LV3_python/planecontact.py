import numpy as np

def planecontact(W, n, d):
	e = (n * W).sum(1) - d
	return W[np.abs(e) < 0.075,:2]
	