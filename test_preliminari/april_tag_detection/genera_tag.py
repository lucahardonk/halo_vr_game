from pathlib import Path
import cv2

def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 0, 800)
    printable = cv2.copyMakeBorder(
        marker, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255
    )
    output = Path(__file__).with_name("tag36h11_id0.png")
    if not cv2.imwrite(str(output), printable):
        raise OSError(f"Impossibile salvare {output}")
    print(output)

if __name__ == "__main__":
    main()
