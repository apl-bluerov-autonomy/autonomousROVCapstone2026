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
                self.cart = [self.x, self.y, self.z]
                pass


def captureTag(pos, t):
       snap = tagSnapShot(pos, t)
       tags.append(snap)

def predictTag(t):
    if(len(tags) > 2):
          firstTag = tags[-2]
          secondTag = tags[-1]
          #thirdTag 
          A = np.vstack([[firstTag.t, secondTag.t], np.ones(2)]).T
        #   print(A)
          
          yMat = np.array([firstTag.cart, secondTag.cart])
        #   print(yMat)

          m, _, _, _ = np.linalg.lstsq(A, yMat)
        #   print("m", m, '\n')
          pos = m[0] * t + m[1]
          return pos

    else:
        return


#For testing
# def main():
#       captureTag(pos=(0, 0, 0, 0, 0), t=0)
#       captureTag(pos=(1, 2, 3, 0, 0), t=1)
#       captureTag(pos=(2, 4, 3, 0, 0), t=2)

#       pTag = predictTag(3)


# main()