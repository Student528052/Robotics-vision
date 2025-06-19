import numpy as np
import cv2 as cv

cam = cv.VideoCapture(0)

paper_detected = False
paper_roi = None
background_roi = None
tracking_started = False

while True:
    ret, frame = cam.read()
    if not ret:
        break

    im = frame.copy()

    if not paper_detected:
        # Phase 0: Try to detect white paper
        gray_im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
        mask = cv.inRange(gray_im, 180, 255)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.erode(mask, kernel, iterations=2)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        large_contours = [cnt for cnt in contours if cv.contourArea(cnt) > 10000]

        if large_contours:
            cnt = max(large_contours, key=cv.contourArea)
            epsilon = 0.02 * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)
            x, y, w, h = cv.boundingRect(approx)
            paper_roi = (x, y, w, h)
            paper_detected = True

    if paper_detected:
        x, y, w, h = paper_roi
        cv.rectangle(im, (x, y), (x+w, y+h), (0, 255, 0), 2)

        if tracking_started:
            # Phase 2: Compare new frame to saved background
            current_roi = cv.cvtColor(frame[y:y+h, x:x+w], cv.COLOR_BGR2GRAY)
            diff = cv.absdiff(background_roi, current_roi)
            _, thresh = cv.threshold(diff, 30, 255, cv.THRESH_BINARY)

            # Clean up
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
            thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)

            contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv.contourArea(cnt) > 300:
                    bx, by, bw, bh = cv.boundingRect(cnt)
                    cv.rectangle(im, (x+bx, y+by), (x+bx+bw, y+by+bh), (0, 0, 255), 2)
                    print(x+bx , y+by, x+bx+bw, y+by+bh)

        else:
            cv.putText(im, "Press T to lock background", (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv.imshow("Live Feed", im)

    key = cv.waitKey(30) & 0xFF
    if key == 27:  # ESC
        break
    elif key == ord('t') and paper_detected:
        # Save background in paper ROI
        background_roi = cv.cvtColor(frame[y:y+h, x:x+w], cv.COLOR_BGR2GRAY)
        tracking_started = True
        print("Background locked. Tracking started.")

cam.release()
cv.destroyAllWindows()
