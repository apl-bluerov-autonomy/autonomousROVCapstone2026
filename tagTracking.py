import numpy as np

tags = []

MAX_HISTORY = 6              # keep only recent detections
MAX_TIME_SINCE_SEEN = 1.0    # seconds before prediction is considered stale
MAX_PREDICTION_AHEAD = 0.75  # max extrapolation past latest tag time
MIN_POINTS = 2               # need at least 2 points for velocity


class tagSnapShot:
    def __init__(self, pos, t):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.t = float(t)
        self.rot = pos[3]
        self.rvec = pos[4]
        self.cart = np.array([self.x, self.y, self.z], dtype=float)


def captureTag(pos, t):
    snap = tagSnapShot(pos, t)
    tags.append(snap)

    # Keep history bounded
    if len(tags) > MAX_HISTORY:
        del tags[:-MAX_HISTORY]

    return snap


def clearTags():
    tags.clear()


def predictTag(t):
    if len(tags) < MIN_POINTS:
        return None

    t = float(t)
    latest = tags[-1]

    time_since_seen = t - latest.t
    if time_since_seen < 0:
        time_since_seen = 0

    if time_since_seen > MAX_TIME_SINCE_SEEN:
        return None

    if t - latest.t > MAX_PREDICTION_AHEAD:
        return None

    recent = tags[-MAX_HISTORY:]

    # Use relative time to avoid numerical issues with large timestamps
    t0 = recent[0].t
    times = np.array([snap.t - t0 for snap in recent], dtype=float)
    positions = np.array([snap.cart for snap in recent], dtype=float)

    # Not enough time difference to estimate velocity
    if np.ptp(times) <= 1e-6:
        return latest.cart.copy()

    A = np.vstack([times, np.ones(len(times))]).T
    coeffs, _, _, _ = np.linalg.lstsq(A, positions, rcond=None)

    velocity = coeffs[0]
    intercept = coeffs[1]

    target_time = t - t0
    predicted = velocity * target_time + intercept

    print("predicted tag:", predicted, "last tag:", latest.cart)

    return predicted


##For testing
def main():
      captureTag(pos=(-680.7364535246329, -235.70809631532924, 2025.2637336993582, 0, 0), t=0)
      captureTag(pos=(-656.6698999603766, -219.39866730345543, 2046.430189021639, 0, 0), t=1)
      captureTag(pos=(-691.6927617881205, -217.03183418074656, 2050.553346820866, 0, 0), t=2)

      pTag = predictTag(3)
    #   print(pTag)


# main()