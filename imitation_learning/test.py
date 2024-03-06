import cv2
import numpy as np
import random

# camera = cv2.VideoCapture(0)

# while True:
#     ret, frame = camera.read()
#     frame.resize
#     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     frame = frame / 1.1
#     frame += 100
#     frame = frame.astype(np.uint8)
#     frame = cv2.applyColorMap(frame, cv2.COLORMAP_MAGMA)
#     cv2.imshow("img", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
ref = cv2.imread("ref.png")
img = cv2.imread("src.png")
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ref = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)

ref = cv2.resize(ref, (img.shape[1], img.shape[0]))

ref = ref.astype(np.float32)
img = img.astype(np.float32)
img = ref - img+40
img = img * (255/img.max())
img = img.astype(np.uint8)

img = cv2.applyColorMap(img, cv2.COLORMAP_MAGMA)
cv2.imshow("img", img)
cv2.waitKey(0)
