import time
import numpy as np

tags = []


class tagSnapShot:
        x, y, z = 0, 0 ,0
        t = 0
        rot, rvec, = 0, 0
        cart = []
        def __init__(self, pos, t):
                self.x = pos[0]
                self.y = pos[1]
                self.z = pos[2]
                self.t = t
                self.rot = pos[3]
                self.rvec = pos[4]
                self.cart = np.array([self.x, self.y, self.z],dtype=float)
                pass


def captureTag(pos, t):
       snap = tagSnapShot(pos, t)
       tags.append(snap)
       return snap

def predictTag(t):
    if(len(tags) > 2):
          firstTag = tags[-2]
          secondTag = tags[-1]
          dt = secondTag.t - firstTag.t
          if dt ==0:
                return secondTag.cart.copy()
          #thirdTag 
          A = np.vstack([[firstTag.t, secondTag.t], np.ones(2)]).T
        #   print(A)
          
          yMat = np.array([firstTag.cart, secondTag.cart])
        #   print(yMat)

          m, _, _, _ = np.linalg.lstsq(A, yMat, rcond=None)
        #   print("m", m, '\n')
          pos = m[0] * t + m[1]
          print("predicted tag:", pos, 'last tag:', secondTag.cart)
          return pos

    else:
        return


##For testing
def main():
      captureTag(pos=(-680.7364535246329, -235.70809631532924, 2025.2637336993582, 0, 0), t=0)
      captureTag(pos=(-656.6698999603766, -219.39866730345543, 2046.430189021639, 0, 0), t=1)
      captureTag(pos=(-691.6927617881205, -217.03183418074656, 2050.553346820866, 0, 0), t=2)

      pTag = predictTag(3)
    #   print(pTag)


# main()