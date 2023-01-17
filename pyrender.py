import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
import math
import cv2

class Render:
    def __init__(self):
        # config
        self.file_path = 'dolphin_without_lefthand.obj'
        # 距离obj重心的距离
        self.dis = 3
        self.w = 1000
        self.h = 1000


    def get_rotate_X(self,angle):
        R_33 = np.eye(3,dtype=np.float32)
        R_33[1,1] = math.cos(angle)
        R_33[1,2] = -1 * math.sin(angle)
        R_33[2,1] = math.sin(angle)
        R_33[2,2] = math.cos(angle)
        return  R_33

    def get_rotate_Y(self,angle):
        R_33 = np.eye(3,dtype=np.float32)
        R_33[0, 0] = math.cos(angle)
        R_33[0, 2] = math.sin(angle)
        R_33[2, 0] = -1 * math.sin(angle)
        R_33[2, 2] = math.cos(angle)
        return R_33

    def get_rotate_Z(self,angle):
        R_33 = np.eye(3,dtype=np.float32)
        R_33[0, 0] = math.cos(angle)
        R_33[0, 1] = -1 * math.sin(angle)
        R_33[1, 0] = math.sin(angle)
        R_33[1, 1] = math.cos(angle)
        return R_33

    def get_RT(self,angle):
        zhuantou = angle # y axis
        taitou = 0 # x axis
        waitou = 0 # z axis

        R_33 = np.matmul(np.matmul(
            self.get_rotate_Y(zhuantou),
            self.get_rotate_X(taitou)
        ),
            self.get_rotate_Z(waitou)
        )
        # 极坐标表示差不多
        T = np.array([self.dis*math.sin(angle), # x
                      0, # y
                      self.dis* math.cos(angle)] # y
                     ,dtype=np.float32)

        RT = np.hstack((R_33,T.reshape((3,1))))
        app = np.array([0,0,0,1]).reshape((1,4))
        RT_44 = np.vstack((RT,app))
        return  RT_44

    def forward(self,angle,_dis):
        self.dis = _dis
        fuze_trimesh = trimesh.load(self.file_path)
        mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)
        scene = pyrender.Scene()
        scene.add(mesh)

        # camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.0)
        camera =  pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.0)
        camera_pose = self.get_RT(angle)

        # print(camera_pose)
        scene.add(camera, pose=camera_pose)

        light = pyrender.PointLight(color=np.ones(3), intensity=40.0)
        light_pose = np.eye(4)
        light_pose[1,3] = 2
        scene.add(light, pose=light_pose)
        # print(light_pose)

        r = pyrender.OffscreenRenderer(self.w,self.h)
        color, depth = r.render(scene)

        return color
        # print(color)
        # plt.figure()
        # plt.imshow(color)
        # plt.xticks([])
        # plt.yticks([])
        # plt.axis('off')

        # plt.subplot(1, 2, 1)
        # plt.axis('off')
        # plt.imshow(color)
        # plt.subplot(1, 2, 2)
        # plt.axis('off')
        # plt.imshow(depth, cmap=plt.cm.gray_r)
        # plt.show()

if __name__ == "__main__":
    render = Render()
    # 写入对象：1.fileName  2.-1：表示选择合适的编码器  3.视频的帧率  4.视频的size
    videoWrite = cv2.VideoWriter('./2.mp4', -1, 20, (render.w,render.h))
    base = math.pi
    delta = math.pi * 2 / 120
    for i in range(121):
        cow_RGB = cv2.cvtColor(render.forward(angle=base - i * delta,_dis=4), cv2.COLOR_BGR2RGB)
        videoWrite.write(cow_RGB)
    dx = (4 - 2) / 30
    for i in range(31):
        cow_RGB = cv2.cvtColor(render.forward(angle=base , _dis=4 - i*dx), cv2.COLOR_BGR2RGB)
        videoWrite.write(cow_RGB)

    for i in range(121):
        cow_RGB = cv2.cvtColor(render.forward(angle=base + i * delta,_dis=2), cv2.COLOR_BGR2RGB)
        videoWrite.write(cow_RGB)
    # color = render.forward(angle=base,_dis=3)
    # plt.figure()
    # plt.imshow(color)
    # plt.xticks([])
    # plt.yticks([])
    # plt.axis('off')
    # plt.show()