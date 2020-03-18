import numpy as np
import copy
import open3d as o3d
from modern_robotics import MatrixLog3, so3ToVec, AxisAng3
import sys
import os

current_path = os.path.dirname(__file__)
module_path = os.path.join(current_path, '../lib')
sys.path.append(module_path)


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


def cal_angle(R_dir):
    angle_in_radians = \
        np.arccos(
            # np.abs(0.00614433*R_dir[0] + 0.944898*R_dir[1] + -0.12439*R_dir[2])
            np.abs(-0.146158 * R_dir[0] + 0.155234 * R_dir[1] + 0.965378 * R_dir[2])
        )
    return angle_in_radians


class RegisterMulti:
    def __init__(self):
        self.cloud_index = 0
        self.initFlag = True
        self.goodResultFlag = True
        self.registrationCount = 0
        self.posWorldTrans = np.identity(4)
        self.posLocalTrans = np.identity(4)
        # test for loop detection info
        self.detectTransLoop = np.identity(4)
        # queue_size should be a little bit big, cause processing speed is not quick enough

        self.cloud_base = o3d.read_point_cloud(
            r"C:\Users\newsmart\Desktop\NSTAlgo\3D\PCL\stitch\2\2-1_Extract.ply")
        self.cloud1 = copy.deepcopy(self.cloud_base)
        self.cloud2 = copy.deepcopy(self.cloud_base)

    def registerLocalCloud(self, target, source):
        """
        get local transformation matrix
        :param target:  Target Point Cloud
        :param source: Source Point Cloud
        :return: transformation from target cloud to source cloud
        """

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)

        source_temp = o3d.voxel_down_sample(source_temp, 0.004)
        target_temp = o3d.voxel_down_sample(target_temp, 0.004)

        o3d.estimate_normals(source_temp, search_param=o3d.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))
        o3d.estimate_normals(target_temp, search_param=o3d.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))

        current_transformation = np.identity(4)
        # use Point-to-plane ICP registeration to obtain initial pose guess
        result_icp_p2l = o3d.registration_icp(source_temp, target_temp, 0.02,
                                              current_transformation, o3d.TransformationEstimationPointToPlane())
        # 0.1 is searching distance

        print("----------------")
        print("initial guess from Point-to-plane ICP registeration")
        print(result_icp_p2l)
        print(result_icp_p2l.transformation)

        p2l_init_trans_guess = result_icp_p2l.transformation
        print("----------------")
        # print("Colored point cloud registration")

    #   Double Normal ICP
        result_icp = o3d.registration_icp(
            source_temp, target_temp, 0.01, p2l_init_trans_guess, o3d.TransformationEstimationPointToPlane())
    #   Color ICP
        # result_icp = o3d.registration_colored_icp(
        #     source_temp, target_temp, 0.01, p2l_init_trans_guess)

        print(result_icp)
        print(result_icp.transformation)
        # print(result_icp.fitness)

        # draw_registration_result_original_color(source, target, result_icp.transformation)

        # write intermediate result for test,
        # but found it will make registration result worse...
        # ---->>>> cause source.transform will change the source!!!!!!
        # source.transform(result_icp.transformation)
        # temp_cloud = target + source
        # write_point_cloud("/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/result/{}-{}.pcd".format(
        # self.cloud_index,self.cloud_index-1), temp_cloud , write_ascii = False)

        return result_icp.transformation
        ######################################
        #    kick-out rule
        ######################################
        # New rule:
        # 1/ rotation is out of plane (5 degree, or 0.087266 in radians);
        # 2/ too big rotation;
        # 3/ too big translation;

        # first calculate what is the rotation direction and rotation angle
        """
        tf = result_icp.transformation
        R = tf[:3, :3]  # rotation matrix
        so3mat = MatrixLog3(R)
        omg = so3ToVec(so3mat)
        R_dir, theta = AxisAng3(omg) # rotation direction
                # rotation angle (in radians)
        theta_degree = theta / np.pi * 180  # in degree
        # angle_with_pl_norm = self.cal_angle(self.rotation_dir, R_dir)
        angle_with_pl_norm = self.cal_angle(R_dir)

        trans_tol = 0.5  # transformation tolerance
        rotation_tol = 30 # 30 degrees
        # angle_with_pl_norm_tol = 0.087266 # in radians (= 5 degrees)
        # angle_with_pl_norm_tol = 0.174533 # in radians (= 10 degrees)
        angle_with_pl_norm_tol = 0.35  # in radians (= 20 degrees)
        if (tf[0, 3] > trans_tol or tf[0, 3] < -trans_tol or
                tf[1, 3] > trans_tol or tf[1, 3] < -trans_tol or
                tf[2, 3] > trans_tol or tf[2, 3] < -trans_tol):
            self.goodResultFlag = False
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            # print("here in 1 ")
            # rospy.logwarn('Something wrong with 1/ translation : (, turn back a little bit...')
            # rospy.logwarn('>> the translation is [{},{},{}]'.format(tf[0,3],tf[1,3],tf[2,3]))
            return np.identity(4)
        elif (theta_degree > rotation_tol or
              theta_degree < - rotation_tol):
            self.goodResultFlag = False
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            # print("here in 2 ")
            # rospy.logwarn('Something wrong with 2/ rotation angle : (, turn back a little bit...')
            # rospy.logwarn('>> the rotation angle is {} (in degrees)'.format(theta_degree))
            return np.identity(4)
        elif angle_with_pl_norm > angle_with_pl_norm_tol:
            self.goodResultFlag = False
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            print("here in 3 ")
            print(" angle with pl norm")
            print(angle_with_pl_norm)
            # rospy.logwarn('Something wrong with 3/ rotation axis : (, turn back a little bit...')
            # rospy.logwarn('>> the rotation axis is {} (in radians) with plane normal'.format(angle_with_pl_norm))
            return np.identity(4)
        else:
            self.goodResultFlag = True
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            # print("here in 4 ")
            return result_icp.transformation

        # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        # print(self.goodResultFlag)

        # if result_icp.fitness > -1:
        #     self.goodResultFlag = True
        #     return result_icp.transformation
        # else:
        #     self.goodResultFlag = False
        #     return np.identity(4)
        """

    def FirstCloud(self, filename):
        self.cloud_base = o3d.read_point_cloud(filename)
        self.cloud1 = copy.deepcopy(self.cloud_base)

    def NonFirstCloud(self, cloud_index):
        self.cloud1 = o3d.read_point_cloud(
            r"C:\Users\newsmart\Desktop\NSTAlgo\3D\PCL\stitch\2\2-{}_Extract.ply".format(cloud_index - 1))
        self.cloud2 = o3d.read_point_cloud(
            r"C:\Users\newsmart\Desktop\NSTAlgo\3D\PCL\stitch\2\2-{}_Extract.ply".format(cloud_index))

        # get local transformation between two lastest clouds
        self.posLocalTrans = self.registerLocalCloud(self.cloud1, self.cloud2)

        # if result is not good, drop it
        if self.goodResultFlag:
            # test for loop detection info
            self.detectTransLoop = np.dot(self.posLocalTrans, self.detectTransLoop)
            # print ("==== loop detect trans ====")
            # print(self.detectTransLoop)
            # print ("==== ==== ==== ==== ==== ====")
            self.posWorldTrans = np.dot(self.posWorldTrans, self.posLocalTrans)
            # update latest cloud
            self.cloud1 = copy.deepcopy(self.cloud2)
            self.cloud2.transform(self.posWorldTrans)
            o3d.write_point_cloud(r"C:\Users\newsmart\Desktop\NSTAlgo\3D\PCL\stitch\2\test.ply",
                                  self.cloud2, write_ascii=False)
            self.cloud_base = self.cloud_base + self.cloud2

            # Down Sampling
            self.cloud_base = o3d.voxel_down_sample(self.cloud_base, 0.001)

            self.registrationCount += 1
            # save PCD file to local
            o3d.write_point_cloud(r"C:\Users\newsmart\Desktop\NSTAlgo\3D\PCL\stitch\2\registerResult.ply",
                                  self.cloud_base, write_ascii=False)

            # o3d.display(cloud_base)


def main():
    RegisterClass = RegisterMulti()

    FirstIndex = 1
    LastIndex = 9

    RegisterClass.FirstCloud(r"C:\Users\newsmart\Desktop\NSTAlgo\3D\PCL\stitch\2\2-1_Extract.ply")
    for i in range(FirstIndex + 1, LastIndex, 1):
        # print(i)
        RegisterClass.NonFirstCloud(i)


if __name__ == "__main__":
    main()
