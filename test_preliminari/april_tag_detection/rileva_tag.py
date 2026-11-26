import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "kinect_base"))
from kinect import Kinect

def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    try:
        with Kinect() as kinect:
            while True:
                frame = cv2.cvtColor(kinect.rgb(), cv2.COLOR_RGB2BGR)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)
                label = "ID 0 non rilevato"
                if ids is not None:
                    for marker_corners, marker_id in zip(corners, ids.ravel()):
                        if int(marker_id) != 0:
                            continue
                        points = marker_corners.reshape(4, 2)
                        p = np.column_stack((points, np.ones(4)))
                        center = np.cross(np.cross(p[0], p[2]), np.cross(p[1], p[3]))
                        if abs(center[2]) < 1e-9:
                            continue
                        x, y = np.rint(center[:2] / center[2]).astype(int)
                        outline = np.rint(points).astype(np.int32)
                        cv2.polylines(frame, [outline], True, (0, 255, 0), 2)
                        cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 255), -1)
                        label = f"ID 0 | centro: x={x}, y={y} px"
                        break
                cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (0, 0, 0), -1)
                cv2.putText(
                    frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2,
                )
                cv2.imshow("Kinect RGB - AprilTag 36h11", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
