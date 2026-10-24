import cv2

capPtr = cv2.VideoCapture(0, cv2.CAP_DSHOW)

capPtr.set(cv2.CAP_PROP_SETTINGS, 1)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
sum = 0
count = 0
while True:

    ret, frame = capPtr.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    corners, ids, rejected = detector.detectMarkers(frame)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("ArUco Detection", frame)
    timestamp_ms = capPtr.get(cv2.CAP_PROP_POS_MSEC)
    sum += timestamp_ms
    count += 1
    if count == 100:
        print(sum/count)
        sum = 0
        count = 0
capPtr.release()

cv2.destroyAllWindows()
