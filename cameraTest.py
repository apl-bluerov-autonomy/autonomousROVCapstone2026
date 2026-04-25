import cv2

gst_pipeline = (
    "udpsrc port=5600 "
    "! application/x-rtp, payload=96 "
    "! rtph264depay "
    "! h264parse "
    "! avdec_h264 "
    "! videoconvert "
    "! video/x-raw, format=BGR "
    "! appsink drop=1"
)

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open stream. Check GStreamer install and pipeline.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame received.")
        break
    cv2.imshow("MAVLink Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()