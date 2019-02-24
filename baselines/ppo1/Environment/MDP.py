import environment
import time

height, width = 30, 30
vehNum = 3

CrossRoad = environment.Env(vehNum, height, width, 4)
CrossRoad.showEnv_init()

for count in range(1):
    collisionFlag = False
    endFlag = False
    tag = 0
    CrossRoad.reStart()
    print()
    while not (collisionFlag or endFlag):
        action = [0] * vehNum
        [state, reward, collisionFlag, endFlag] = CrossRoad.updateEnv(action)
        CrossRoad.showEnv()
        tag += 1
        print(count, "step: ", tag, "collision?: ", collisionFlag, "end?: ", endFlag)
        print(state)
        time.sleep(2)


