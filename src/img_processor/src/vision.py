













import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

from scipy import ndimage
from scipy.misc import imresize
from skimage import filters
from sklearn.cluster import KMeans

from skimage.measure import block_reduce
import time
import pdb


def write_image(raw):
	return cv2.imwrite('preprocessed.jpg', raw)

def image_process(raw):
	n_clusters = 2
	image = read_image(raw)


	# Downsample img first using the mean to speed up K-means
    img_d = block_reduce(img, block_size=(2, 2, 1), func=np.mean)

    # TODO: Generate a clustered image using K-means

    # first convert our 3-dimensional img_d array to a 2-dimensional array
    # whose shape will be (length * width, number of channels) hint: use img_d.shape
    img_r = np.reshape(img_d, (np.shape(img_d)[0]*np.shape(img_d)[1], 3))
    
    # fit the k-means algorithm on this reshaped array img_r using the
    # the scikit-learn k-means class and fit function
    # see https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html
    # the only parameters you have to worry about are n_clusters and random_state
    kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(img_r)

    # get the labeled cluster image using kmeans.labels_
    clusters = kmeans.labels_

    # reshape this clustered image to the original downsampled image (img_d) shape
    cluster_img = np.reshape(clusters, np.shape(img_d)[0:2])

    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = imresize(cluster_img, (img.shape[0], img.shape[1]), interp='nearest')

    return img_u.astype(np.uint8)

