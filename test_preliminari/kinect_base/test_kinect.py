import argparse
from time import monotonic
import cv2
import numpy as np
from kinect import Kinect

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("rgbd", "ir"), default="rgbd")
    args = parser.parse_args()
    frames = 0
    start = monotonic()
    try:
        with Kinect() as kinect:
            while True:
                if args.mode == "ir":
                    cv2.imshow("Kinect IR", kinect.ir())
                else:
                    rgb = kinect.rgb()
                    depth = kinect.depth()
                    valid = depth < 2047
                    view = cv2.applyColorMap(
                        (np.minimum(depth, 2047) >> 3).astype(np.uint8),
                        cv2.COLORMAP_TURBO,
                    )
                    view[~valid] = 0
                    cv2.putText(
                        view, f"Pixel validi: {valid.mean():.0%}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2,
                    )
                    cv2.imshow("Kinect RGB", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Kinect profondita - valori grezzi", view)
                frames += 1
                now = monotonic()
                if now - start >= 2:
                    print(f"{args.mode}: {frames / (now - start):.1f} iterazioni/s")
                    start, frames = now, 0
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
