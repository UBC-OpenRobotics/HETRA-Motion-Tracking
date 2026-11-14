#!/usr/bin/env python3
"""Generate printable ArUco / AprilTag markers or 2x2 boards (PNG).

  python make_marker.py --id 0                       # single marker
  python make_marker.py --id 0 1 2 3                 # one PNG per id
  python make_marker.py --board 10 11 12 13          # 2x2 board: [TL, TR, BL, BR]
  python make_marker.py --id 0 --dict DICT_APRILTAG_36h11

After printing, MEASURE the black square edge (and the gap between squares for a
board) and put those numbers in config/marker_map.yaml. Printers rescale.
"""
import argparse

import cv2
import numpy as np


def marker(d, mid, px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(d, mid, px)
    return cv2.aruco.drawMarker(d, mid, px)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--id", type=int, nargs="+", default=None)
    p.add_argument("--board", type=int, nargs=4, metavar=("TL", "TR", "BL", "BR"))
    p.add_argument("--dict", default="DICT_6X6_250")
    p.add_argument("--px", type=int, default=600, help="marker side in pixels")
    p.add_argument("--gap", type=int, default=150, help="board: white gap between markers in pixels")
    p.add_argument("--border", type=int, default=100, help="white quiet zone in pixels")
    p.add_argument("-o", "--out", default=None, help="output file (single marker or board)")
    a = p.parse_args()

    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a.dict))
    if a.board:
        n = 2 * a.px + a.gap
        img = np.full((n, n), 255, np.uint8)
        for k, mid in enumerate(a.board):
            r, c = divmod(k, 2)
            y, x = r * (a.px + a.gap), c * (a.px + a.gap)
            img[y:y + a.px, x:x + a.px] = marker(d, mid, a.px)
        img = cv2.copyMakeBorder(img, a.border, a.border, a.border, a.border,
                                 cv2.BORDER_CONSTANT, value=255)
        out = a.out or f"board_{a.dict}_{'_'.join(map(str, a.board))}.png"
        cv2.imwrite(out, img)
        print(f"wrote {out}  (gap/marker ratio = {a.gap / a.px:.3f})")
        return

    for mid in a.id or [0]:
        img = cv2.copyMakeBorder(marker(d, mid, a.px), a.border, a.border, a.border, a.border,
                                 cv2.BORDER_CONSTANT, value=255)
        out = a.out if (a.out and len(a.id or [0]) == 1) else f"marker_{a.dict}_{mid}.png"
        cv2.imwrite(out, img)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
