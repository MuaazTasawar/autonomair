import cv2
import numpy as np


class TargetDetector:
    """
    Detects a colored target (default: green) in a video frame using HSV
    color thresholding and contour detection.

    Returns the target's pixel centroid and bounding box if found.
    """

    def __init__(self, config):
        # HSV color range for target — default is green
        lower = config.get("target_color_lower", [40, 50, 50])
        upper = config.get("target_color_upper", [80, 255, 255])

        self.lower_hsv = np.array(lower, dtype=np.uint8)
        self.upper_hsv = np.array(upper, dtype=np.uint8)

        self.min_contour_area = 500   # ignore noise smaller than this
        self.debug = False             # set True to show live OpenCV window

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def detect(self, frame):
        """
        Analyze a single BGR frame.

        Returns:
            dict with keys: found (bool), cx (int), cy (int),
                            area (float), bbox (x,y,w,h), annotated_frame
            or None if no target found.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Clean up the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return {"found": False, "cx": None, "cy": None,
                    "area": 0, "bbox": None, "annotated_frame": frame}

        # Pick the largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_contour_area:
            return {"found": False, "cx": None, "cy": None,
                    "area": area, "bbox": None, "annotated_frame": frame}

        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        cy = y + h // 2

        annotated = self._annotate(frame.copy(), cx, cy, x, y, w, h, area)

        if self.debug:
            cv2.imshow("TargetDetector", annotated)
            cv2.waitKey(1)

        return {
            "found": True,
            "cx": cx,
            "cy": cy,
            "area": area,
            "bbox": (x, y, w, h),
            "annotated_frame": annotated
        }

    def detect_from_camera(self, camera_index=0, callback=None):
        """
        Open a live camera feed and run detection on each frame.
        Calls callback(result) on every frame if provided.
        Press Q to quit.
        """
        cap = cv2.VideoCapture(camera_index)

        if not cap.isOpened():
            raise RuntimeError(f"[TargetDetector] Could not open camera {camera_index}")

        print(f"[TargetDetector] Live detection started on camera {camera_index}. Press Q to quit.")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[TargetDetector] Failed to read frame.")
                    break

                result = self.detect(frame)

                if callback:
                    callback(result)

                cv2.imshow("TargetDetector — Live", result["annotated_frame"])

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
            print("[TargetDetector] Camera released.")

    def set_color_range(self, lower_hsv, upper_hsv):
        """Dynamically update the target color range at runtime."""
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        print(f"[TargetDetector] Color range updated: {lower_hsv} → {upper_hsv}")

    # ------------------------------------------------------------------ #
    #  Internal helpers                                                    #
    # ------------------------------------------------------------------ #

    def _annotate(self, frame, cx, cy, x, y, w, h, area):
        # Bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Centroid crosshair
        cv2.drawMarker(frame, (cx, cy), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)

        # Label
        cv2.putText(frame, f"TARGET ({cx},{cy}) area={int(area)}",
                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)

        # Frame center crosshair
        fh, fw = frame.shape[:2]
        cv2.drawMarker(frame, (fw // 2, fh // 2), (255, 0, 0),
                       cv2.MARKER_CROSS, 30, 1)

        return frame