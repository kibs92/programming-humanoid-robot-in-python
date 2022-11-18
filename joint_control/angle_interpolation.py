'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from scipy import interpolate


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        # target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # uncomment this line for the stand motions
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        if self.start_time < 0:
            self.start_time = perception.time
        real_time = perception.time - self.start_time
        for i in range(len(keyframes[0])):
            name = keyframes[0][i]
            time = keyframes[1][i]
            keys = keyframes[2][i]
            points = ([0], [0])
            points_b = [(0, 0)]
            for i in range(len(time)):
                points_b.append((time[i], keys[i][0]))
                points[0].append(time[i])
                points[1].append(keys[i][0])
            #print(points)
            p = interpolate.CubicSpline(points[0], points[1], bc_type='natural')
            #foo = interpolate.splrep(points[0], points[1])
            if real_time > time[-1]:
                target_joints[name] = points[1][-1]
                # print(name, " end")
            else:
                target_joints[name] =  p(real_time) #interpolate.splev(real_time, foo)

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
