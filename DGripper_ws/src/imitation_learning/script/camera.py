import cv2
import numpy as np


def get_height_map(t1, ref1):
    t1 = cv2.cvtColor(t1, cv2.COLOR_BGR2GRAY)
    t1 = np.array(t1)
    t1 = t1.astype(np.int16)
    # white is 255, black is 0
    dif1 = ref1 - t1-5
    # gaussian blur
    dif1 = cv2.GaussianBlur(dif1, (5, 5), 0)
    dif1 = cv2.GaussianBlur(dif1, (5, 5), 0)
    # where dif1 < 0, dif1 = 0
    dif1[dif1 < 0] = 0

    mask = np.zeros(dif1.shape, dtype=np.uint8)
    mask[dif1 > 0] = 255
    cv2.imshow("mask", mask)
    mask = cv2.erode(mask, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (7, 7)), iterations=3)
    mask = cv2.dilate(mask, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (7, 7)), iterations=3)

    # 50 的意思是低于 50 的像素点会被认为不是边界点，高于 150 的像素点会被认为是边界点
    # canny = cv2.Canny(mask, 50, 150)
    # cv2.imshow("canny", canny)
    coutours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    max_contour = None
    for contour in coutours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour
    # 找到最大轮廓的最长方向
    if max_contour is not None:
        rect = cv2.minAreaRect(max_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

    cv2.imshow("mask_after", mask)
    dif1[mask == 0] = 0
    # where dif1 > 255, dif1 = 255
    dif1 *= 10
    dif1[dif1 > 255] = 255

    # gaussian blur

    blur = cv2.GaussianBlur(dif1, (5, 5), 0)
    blur = cv2.GaussianBlur(blur, (5, 5), 0)
    blur = cv2.GaussianBlur(blur, (5, 5), 0)

    dif1[dif1 > 230] = blur[dif1 > 230]
    # dif1 to uint8
    dif1 = dif1.astype(np.uint8)
    # dif1 += 60

    # 转伪彩色
    dif1 = cv2.applyColorMap(dif1, cv2.COLORMAP_MAGMA)
    # if max_contour is not None:
    #     cv2.drawContours(dif1, [box], 0, (0, 0, 255), 2)
    #     # 在box的长轴中心线上画一条红线
    #     cv2.line(dif1, (box[0][0], box[0][1]),
    #              (box[2][0], box[2][1]), (0, 0, 255), 2)
    return dif1
    # dif1 = dif


# camera = cv2.VideoCapture(0)
tact1 = cv2.VideoCapture(0)
# tact2 = cv2.VideoCapture(2)

tact1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
tact1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# tact2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# tact2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # ret, frame = camera.read()
    ret1, ref1 = tact1.read()
    # ret2, ref2 = tact2.read()
    # cv2.imshow("center", frame)
    cv2.imshow("ref1", ref1)
    # cv2.imshow("ref2", ref2)
    # cv2.imshow("t2", t2)

    if cv2.waitKey(1) & 0xFF == ord('o'):
        break

ref1 = cv2.cvtColor(ref1, cv2.COLOR_BGR2GRAY)
ref1 = np.array(ref1)
ref1 = ref1.astype(np.int16)

# ref2 = cv2.cvtColor(ref2, cv2.COLOR_BGR2GRAY)
# ref2 = np.array(ref2)
# ref2 = ref2.astype(np.int16)

while True:
    ret1, t1 = tact1.read()
    # ret2, t2 = tact2.read()
    dif1 = get_height_map(t1, ref1)
    # dif2 = get_height_map(t2, ref2)
    # 拼接
    # dif = np.hstack((dif1, dif2))
    cv2.imshow("dif", dif1)
    # cv2.imshow("dif2", dif2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
