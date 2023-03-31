# this program finds to maximum blobs of a red color in an image

import cv2
import numpy as np
import multiprocessing as mp

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


def start(width, height, queue, debug):
    # read image from video
    if Picamera2 is not None:
        cap = Picamera2()
        cap.video_configuration.main.size = (width, height)
        cap.video_configuration.main.format = "XRGB8888"
        cap.video_configuration.controls.FrameRate = 60
        cap.start()
    else:
        cap = cv2.VideoCapture(0)
    print("Starting", debug)

    while True:
        if Picamera2 is not None:
            img = cap.capture_array()
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            img = cap.read()[1]
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if img is None:
            continue

        # blur
        img = cv2.GaussianBlur(img, (31, 31), 0)
        height, width, _ = img.shape

        # convert to hsv
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # define range of red color in HSV
        lower_red = np.array([120 - 5, 75, 75])
        upper_red = np.array([120 + 30, 255, 255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) == 0:
            continue
        # find the largest contour in the mask, then use
        # it to find the bounding box and centroid
        c = max(cnts, key=lambda x: cv2.contourArea(x))
        # find bounding box
        x, y, w, h = cv2.boundingRect(c)
        threshold = np.max([width, height]) / 100
        if w < threshold or h < threshold:
            continue
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        queue.put((width - center[0], height - center[1]))

        if debug:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(img, center, 5, (0, 0, 255), -1)
            # draw bounding box
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # show the frame to our screen
            cv2.imshow("Frame", img)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)


def make_object_finder(width, height, debug=False):
    queue = mp.Queue()
    p = mp.Process(target=start, args=(width, height, queue, debug))
    p.start()

    def stop():
        p.terminate()
        p.join()

    def get():
        if queue.empty():
            return None
        while not queue.empty():
            center = queue.get()
        return center

    return get, stop


if __name__ == '__main__':
    find, stop = make_object_finder(640, 480, debug=False)

    try:
        while True:
            center = find()
            if center is not None:
                print(center)
    except KeyboardInterrupt:
        print("Stopping")
    stop()

