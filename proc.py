import cv2
import numpy as np


def sum_row(row):

    net_dist = 0
    thresh = 250

    center_idx = row.shape[0] / 2

    idxs = np.argwhere(row > thresh)[:, 0]

    for idx in [min(idxs), max(idxs)]:
        net_dist += idx - center_idx

    return net_dist/2


def proc_img(img):

    # canny edge detection
    img = cv2.Canny(img, 200, 400)

    # delete top half of image then flip
    img = img[img.shape[0]//2:, :]
    img = cv2.flip(img, 0)

    # this essentially calculates distance from center for each row
    dists = [sum_row(img[n, :]) for n in range(img.shape[0])]

    mean_dist = sum(dists) / len(dists)
    return mean_dist / (img.shape[1]//2)


if __name__ == "__main__":

    img = cv2.imread("center.png")
    ret = proc_img(img)
    print("ret = {}".format(ret))
