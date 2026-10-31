import cv2
import av

capPtr = av.open(
    "video=USB Camera",
    format="dshow",
    options={"use_video_device_timestamps": "1"}
)

frames = capPtr.decode(video=0)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

total = 0
count = 0
prev_timestamp = None

while True:
    try:
        av_frame = next(frames)
    except StopIteration:
        break

    frame = av_frame.to_ndarray(format="bgr24")
    timestamp_ms = av_frame.time * 1000 if av_frame.time is not None else None

    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow("ArUco Detection", frame)

    if timestamp_ms is not None:
        if prev_timestamp is not None:
            total += timestamp_ms - prev_timestamp
            count += 1

        prev_timestamp = timestamp_ms

    if count == 100:
        print(f"Average frame interval: {total/count:.3f} ms")
        total = 0
        count = 0

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capPtr.close()
cv2.destroyAllWindows()
