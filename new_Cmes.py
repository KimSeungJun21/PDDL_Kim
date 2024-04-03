#launch Isaac Sim before any other imports
#default first two lines in any standalone application
import sys
sys.path.append("/home/kimseungjun/.local/share/ov/pkg/isaac_sim-2022.2.0")
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core import World
from omni.isaac.core.materials import VisualMaterial, OmniPBR
from omni.isaac.core.objects import FixedCuboid, DynamicCuboid
from omni.isaac.core.prims import XFormPrim, GeometryPrim, RigidPrim
import numpy as np
import omni.isaac.core.utils.numpy.rotations as rot_utils
import carb
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.franka import Franka
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph
import sys
if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
import omni
import math
import numpy as np
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.bounds as bounds_utils 
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from extension_examples.user_examples.utils.isaac_sim_utils import CollisionCheck
# from utils.testutil import A
extensions.enable_extension("omni.isaac.ros_bridge")

import json


franka_h = 0.41
table_h = 0.41
box_size = 0.04
box_height=0.13
def random_translation():
    translation = np.concatenate([np.random.uniform(-0.005, 0.005, 1),np.random.uniform(-0.005, 0.005, 1), np.array([0])], 0)
    return translation

box_order = [-0.2,-0.1,0,0.1,0.2]
np.random.shuffle(box_order)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

arm_base = world.scene.add(
    FixedCuboid(
        prim_path="/World/arm_base",
        name="arm_base",
        position=np.array([0, 0, franka_h/2]),
        scale=np.array([0.2, 0.2, franka_h]),
        color=np.array([5, 5, 5]),
    ),
    )

table = world.scene.add(
    FixedCuboid(
        prim_path="/World/table",
        name="table",
        position=np.array([0.4, 0, (table_h)/2]),
        scale=np.array([0.6, 1.2, (table_h)]),
        color=np.array([0.95, 1.0, 1.0]),
    ),
    )

obj_prim = world.scene.add(
    XFormPrim(prim_path="/World/obj",
              name="obj_prim",
              translation=np.array([0.4, 0, (table_h)]),
              orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            )
)

franka = world.scene.add(
    Franka(
        prim_path="/World/Franka",
        name = "Franka",
        position=np.array([0, 0, franka_h]),
    )
)
franka.set_joint_efforts(np.array([10, 10]), joint_indices=np.array([7, 8]))


empty_space = world.scene.add(
    FixedCuboid(
        prim_path="/World/obj/empty_space_0",
        name="empty_space_0",
        #translation=np.array([1,1,1]),
        translation=np.array([0.16312, 0.04407, 0.00301]),
        scale=np.array([0.1, 0.1, 0.0001]),
        visible=0
    ),
    )

empty_space_01 = world.scene.add(
    FixedCuboid(
        prim_path="/World/obj/empty_space_1",
        name="empty_space_1",
        #translation=np.array([1,1,1]),
        translation=np.array([0.16312, -0.29426, 0.00301]),
        scale=np.array([0.1, 0.1, 0.0001]),
        visible=0
    ),
    )

empty_space_02 = world.scene.add(
    FixedCuboid(
        prim_path="/World/obj/empty_space_2",
        name="empty_space_2",
        #translation=np.array([1,1,1]),
        translation=np.array([-0.04537, -0.29426, 0.00301]),
        scale=np.array([0.1, 0.1, 0.0001]),
        visible=0
    ),
    )
empty_space_03 = world.scene.add(
    FixedCuboid(
        prim_path="/World/obj/empty_space_3",
        name="empty_space_3",
        #translation=np.array([1,1,1]),
        translation=np.array([-0.0695, 0.04407, 0.00301]),
        scale=np.array([0.1, 0.1, 0.0001]),
        visible=0
    ),
    )


def generate_random_value():
    # x=np.random.uniform(-0.02, 0.15)
    # y=np.random.uniform(-0.23, 0.05)
    # z=np.random.uniform(0.1, 0.3)
    x=0.1
    y=0
    z=0.2
    return [x, y, z]
position_value=generate_random_value()


# box0 = world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/obj/box0",
#         name="box0",
#         #translation=np.array([1,1,1]),
#         translation=np.array(generate_random_value()),
#         scale=np.array([box_size, box_size, box_size]),
#         color=np.array([0.5, 0, 0.5]),
#         mass=0.00002,
#     ),
#     )
# box1 = world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/obj/box1",
#         name="box1",
#         #translation=np.array([1,1,1]),
#         translation=np.array(generate_random_value()),
#         scale=np.array([box_size, box_size+0.05, box_size]),
#         color=np.array([1.0, 0, 0]),
#         mass=0.00002,
#     ),
#     )
# box2 = world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/obj/box2",
#         name="box2",
#         #translation=np.array([1,1,1]),
#         translation=np.array(generate_random_value()),
#         scale=np.array([box_size+0.1, box_size-0.01, box_size]),
#         color=np.array([0, 0.5, 0.5]),
#         mass=0.00002,
#     ),
#     )


add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/KLT_Bin/small_KLT.usd", prim_path="/World/obj/klt")
ab = world.scene.add(
            RigidPrim(prim_path="/World/obj/klt", name="klt",translation=np.array([0.05, -0.1, 0.06]),scale=np.array([2, 2, 0.8]),),
        )



add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/KLT_Bin/small_KLT.usd", prim_path="/World/obj/klt1")
ac = world.scene.add(
            RigidPrim(prim_path="/World/obj/klt1", name="klt1", orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),translation=np.array([0.05, 0.3, 0.06]),scale=np.array([1.2, 1.2, 0.8]),),
        )


add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd", prim_path="/World/obj/cracker_box")
a0 = world.scene.add(
            RigidPrim(prim_path="/World/obj/cracker_box", name="cracker_box",translation=np.array(generate_random_value()),mass=0.0004),
        )
a00 = world.scene.add(
            GeometryPrim(prim_path="/World/obj/cracker_box",name="cracker_box1",scale=np.array([0.4, 0.5, 0.5]),collision=True,disable_stablization=False),
        )
add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/004_sugar_box.usd", prim_path="/World/obj/sugar_box")
a1 = world.scene.add(
            RigidPrim(prim_path="/World/obj/sugar_box", name="sugar_box",orientation=euler_angles_to_quat(np.array([0, 0, 0])),translation=np.array(generate_random_value()),mass=0.0004),
        )
a11 = world.scene.add(
            GeometryPrim(prim_path="/World/obj/sugar_box",name="sugar_box1",scale=np.array([0.5, 0.5, 0.5]),collision=True,disable_stablization=False),
        )
# a11.set_collision_approximation("boundingCube")
add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd", prim_path="/World/obj/tomato_can")
a2 = world.scene.add(
            RigidPrim(prim_path="/World/obj/tomato_can", name="tomato_can",translation=np.array(generate_random_value()),mass=0.0004),
        )
a22 = world.scene.add(
            GeometryPrim(prim_path="/World/obj/tomato_can", name="tomato_can1",scale=np.array([1, 1, 1]),collision=True),
        )

add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/010_potted_meat_can.usd", prim_path="/World/obj/meat_can")
a5 = world.scene.add(
            RigidPrim(prim_path="/World/obj/meat_can", name="meat_can",orientation=euler_angles_to_quat(np.array([0, 0, 0])),translation=np.array(position_value),mass=0.0004),
        )
a6 = world.scene.add(
            GeometryPrim(prim_path="/World/obj/meat_can", name="meat_can1",orientation=euler_angles_to_quat(np.array([0, 0, 0])),scale=np.array([1, 0.6, 1]),collision=True,disable_stablization=False),
        )
# a6.set_collision_approximation("boundingCube")
# position_value2=generate_random_value()
# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/021_bleach_cleanser.usd", prim_path="/World/obj/bleach_cleanser")
# a7 = world.scene.add(
#             RigidPrim(prim_path="/World/obj/bleach_cleanser", name="bleach_cleanser",orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),translation=np.array(position_value2),mass=0.0004,density=0.9),
#         )
# a77 = world.scene.add(
#             GeometryPrim(prim_path="/World/obj/bleach_cleanser",name="bleach_cleanser1",orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),scale=np.array([0.5, 0.5, 0.5]),collision=True,disable_stablization=False),
#         )
# # a77.set_collision_approximation("boundingCube")
# position_value3=generate_random_value()
# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/009_gelatin_box.usd", prim_path="/World/obj/gelatin_box")
# a3 = world.scene.add(
#             RigidPrim(prim_path="/World/obj/gelatin_box", name="gelatin_box",orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),translation=np.array(position_value3),mass=0.0004),
#         )
# a33 = world.scene.add(
#             GeometryPrim(prim_path="/World/obj/gelatin_box", name="gelatin_box1",orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),scale=np.array([1, 0.8, 1]),collision=True),
#         )
# # a33.set_collision_approximation("boundingCube")
# position_value4=generate_random_value()

# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/008_pudding_box.usd", prim_path="/World/obj/pudding_box")
# a8 = world.scene.add(
#             RigidPrim(prim_path="/World/obj/pudding_box", name="pudding_box",orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),translation=np.array(position_value4),mass=0.0004),
#         )
# a88 = world.scene.add(
#             GeometryPrim(prim_path="/World/obj/pudding_box", name="pudding_box1",orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),scale=np.array([0.8, 0.7, 1]),collision=True),
#         )
# # a88.set_collision_approximation("boundingCube")
# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/002_master_chef_can.usd", prim_path="/World/obj/master_chef_can")
# a9 = world.scene.add(
#             RigidPrim(prim_path="/World/obj/master_chef_can", name="master_chef_can",translation=np.array(generate_random_value()),mass=0.0004),
#         )
# a99 = world.scene.add(
#             GeometryPrim(prim_path="/World/obj/master_chef_can", name="master_chef_can1",scale=np.array([0.7, 0.7, 0.7]),collision=True),
#         )
# # a99.set_collision_approximation("boundingCube")
# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/036_wood_block.usd", prim_path="/World/obj/wood_block")
# ax = world.scene.add(
#             RigidPrim(prim_path="/World/obj/wood_block", name="wood_block",translation=np.array(generate_random_value()),mass=0.0004),
#         )
# axx = world.scene.add(
#             GeometryPrim(prim_path="/World/obj/wood_block", name="wood_block1",scale=np.array([0.5, 0.5, 0.5]),collision=True),
#         )
# # axx.set_collision_approximation("boundingCube")
# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned/019_pitcher_base.usd", prim_path="/World/obj/pitcher_base")
# ax1 = world.scene.add(
#             RigidPrim(prim_path="/World/obj/pitcher_base", name="pitcher_base",translation=np.array(generate_random_value()),mass=0.0004),
#         )
# ax1x1 = world.scene.add(
#             GeometryPrim(prim_path="/World/obj/pitcher_base", name="pitcher_base1",scale=np.array([0.5, 0.5, 0.5]),collision=True),
#         )
# # ax1x1.set_collision_approximation("boundingCube")

camera = Camera(
    prim_path="/World/Franka/panda_hand/geometry/realsense/realsense_camera",
)
camera.initialize()

camera.set_focal_length(0.1)
camera.set_focus_distance(0) 
camera.set_horizontal_aperture(0.23)





keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/panda_graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
            ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
            ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
            ("isaac_create_viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("isaac_get_viewport_render_product", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
            ("isaac_set_camera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
            ("ros1_camera_helper", "omni.isaac.ros_bridge.ROS1CameraHelper"),

            ("obj_ros1_publish_transform_tree", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
        ],
        
        keys.SET_VALUES: [
            ("SubscribeJointState.inputs:topicName", "joint_command"),
            ("ArticulationController.inputs:robotPath", "/World/Franka"),
            ("ArticulationController.inputs:usePath", True),
            ("PublishJointState.inputs:topicName", "joint_states"),
            ("PublishClock.inputs:topicName", "clock"),
            ("PublishTF.inputs:topicName","/tf"),
            ("isaac_create_viewport.inputs:name", "realsense_viewport"),
            ("ros1_camera_helper.inputs:frameId", "camera_rgb_optical_frame"),
            ("ros1_camera_helper.inputs:topicName", "/camera/depth_registered/rgb"),
            ("ros1_camera_helper.inputs:type", "rgb"),

            ("obj_ros1_publish_transform_tree.inputs:topicName","/tf_obj"),

        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "isaac_create_viewport.inputs:execIn"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("ReadSimTime.outputs:simulationTime","PublishJointState.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime","PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime","PublishTF.inputs:timeStamp"),
            ("isaac_create_viewport.outputs:execOut", "isaac_get_viewport_render_product.inputs:execIn"),
            ("isaac_create_viewport.outputs:viewport", "isaac_get_viewport_render_product.inputs:viewport"),
            ("isaac_get_viewport_render_product.outputs:execOut", "isaac_set_camera.inputs:execIn"),
            ("isaac_get_viewport_render_product.outputs:renderProductPath", "isaac_set_camera.inputs:renderProductPath"),
            ("isaac_get_viewport_render_product.outputs:renderProductPath", "ros1_camera_helper.inputs:renderProductPath"),
            ("isaac_set_camera.outputs:execOut", "ros1_camera_helper.inputs:execIn"),

            ("OnPlaybackTick.outputs:tick", "obj_ros1_publish_transform_tree.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime","obj_ros1_publish_transform_tree.inputs:timeStamp"),

        ],
    },
)


from omni.isaac.core.utils.prims import set_targets

set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/PublishJointState"),
    attribute="inputs:targetPrim",
    target_prim_paths=["/World/Franka"],
)

# Setting the /Franka target prim to Publish Transform Tree node
set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/PublishTF"),
    attribute="inputs:targetPrims",
    target_prim_paths=["/World/Franka","/World/Franka/panda_hand/geometry/realsense/realsense_camera"],
)
set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/isaac_set_camera"),
    attribute="inputs:cameraPrim",
    target_prim_paths=["/World/Franka/panda_hand/geometry/realsense/realsense_camera"],
)

set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/obj_ros1_publish_transform_tree"),
    attribute="inputs:targetPrims",
    target_prim_paths=["/World/Franka/panda_hand",
                       "/World/obj",
                       "/World/obj/klt",
                       "/World/obj/klt1",
                       "/World/obj/cracker_box",
                       "/World/obj/sugar_box",
                       "/World/obj/tomato_can",
                    #    "/World/obj/gelatin_box",
                    # #    "/World/obj/mustard_bottle",
                    #    "/World/obj/meat_can",
                    #    "/World/obj/bleach_cleanser",
                    #    "/World/obj/pudding_box",
                       "/World/obj/empty_space_0",
                       "/World/obj/empty_space_1",
                       "/World/obj/empty_space_2",
                       "/World/obj/empty_space_3",
                    #    "/World/obj/master_chef_can",
                    #    "/World/obj/wood_block",
                    #    "/World/obj/pitcher_base",
                       ],
)

simulation_app.update()
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()
#####################################################################################3##내가 사용할 함수 ###

# class CollisionCheck:
#     def __init__(self) -> None:
#         self.prime_tuple={}
#         self.prime_tuple_name={}
#         self.prime_list=[]
#         self.background_prime=None

#     def position_check(self):
#         prim_list=self.prime_list[:]
#         prim_list.insert(0,ab)
#         position=[]
#         orientation=[]
#         for i,name in enumerate(prim_list):
#             position1, quert = name.get_local_pose()
#             # orientation=quat_to_euler_angles(quert)
#             position.append(position1)
#             orientation.append(quert)
#         return position,orientation

#     def collistion_check(self,prime,compute_list2):
#         target_position=prime.get_local_pose()[0]
#         for _, i in enumerate(compute_list2):
#             existed_position=i.get_local_pose()[0]
#             #print(math.sqrt((existed_position[0]-target_position[0])**2 + (existed_position[1]-target_position[1])**2))
#             distance=math.sqrt((existed_position[0]-target_position[0])**2 + (existed_position[1]-target_position[1])**2)
#             if distance <= 0.05 and distance != 0.0:
#                 return False
#         return True


#     def comput_relationship(self):
#         prime_tuple=self.prime_tuple
#         prime_tuple_name=self.prime_tuple_name
#         prime_list=self.prime_list[:]
#         # print('prime_list:',prime_list)
#         compute_list=[]
#         compute_list2=[]
#         previous_object_list=[]
#         prime_list.sort(key=lambda x: x.get_local_pose()[0][2])#z축 기준으로 정렬
#         while prime_list:
#             prime=prime_list.pop(0)
#             # print('prime:',prime_tuple_name[prime])
#             if not compute_list:
#                 compute_list.append(f"from_id: {prime_tuple[prime]}, to_id: 0, relation_type: ON")
#                 compute_list2.append(prime)
#             elif self.collistion_check(prime=prime,compute_list2=compute_list2):
#                 compute_list.append(f"from_id: {prime_tuple[prime]}, to_id: 0, relation_type: ON")
#                 compute_list2.append(prime)
#             else:
#                 what_the_fuck=previous_object_list[:] #이미 놓았던 object list 얕은복사
#                 for prime2 in what_the_fuck: #이미 놓았던 object list를 순회
#                     if self.check_IoU(prime=prime,prime2=prime2):
#                         # print('prime2 and prime:',prime_tuple_name[prime2], prime_tuple_name[prime])
#                         compute_list.append(f"from_id: {prime_tuple[prime]}, to_id: {prime_tuple[prime2]}, relation_type: ON")
#             previous_object_list.append(prime)

#         return compute_list

#     def check_IoU(self,prime,prime2):
#         #sugar와 bleach cleanser의 경우는 sugar가 prime2 
#         #bleach_cleanser가 prime
#         prime_tuple=self.prime_tuple
#         prime_tuple_name=self.prime_tuple_name
#         # print(prime_tuple_name[prime])
#         # print(prime_tuple_name[prime2])

#         cache = bounds_utils.create_bbox_cache()
#         prime_shape_list=bounds_utils.compute_aabb(cache, prim_path=f"/World/obj/{self.prime_tuple_name[prime]}")
#         prime_x_half_length=(prime_shape_list[3]-prime_shape_list[0])/2.0
#         prime_y_half_length=(prime_shape_list[4]-prime_shape_list[1])/2.0
#         prime2_shape_list=bounds_utils.compute_aabb(cache, prim_path=f"/World/obj/{self.prime_tuple_name[prime2]}")
#         prime2_x_half_length=(prime2_shape_list[3]-prime_shape_list[0])/2.0
#         prime2_y_half_length=(prime2_shape_list[4]-prime_shape_list[1])/2.0
#         position,_=self.position_check()

#         prime_number=self.prime_tuple[prime]
#         prime2_number=self.prime_tuple[prime2]
#         prime_region=[[(position[prime_number][0]-prime_x_half_length),(position[prime_number][0]+prime_x_half_length)],
#                      [(position[prime_number][1]-prime_y_half_length),(position[prime_number][1]+prime_y_half_length)]]
#         prime2_region=[[(position[prime2_number][0]-prime2_x_half_length),(position[prime2_number][0]+ prime2_x_half_length)],
#                       [(position[prime2_number][1]-prime2_y_half_length),(position[prime2_number][1]+prime2_y_half_length)]]
#         #check intersection
#         intersection_x1=max(prime_region[0][0],prime2_region[0][0]) ##겹치는 영역 x1 좌측
#         intersection_x2=min(prime_region[0][1],prime2_region[0][1]) #겹치는 영역 x2 우측
#         intersection_y1=min(prime_region[1][1],prime2_region[1][1]) #겹치는 영역 y1 상단
#         intersection_y2=max(prime_region[1][0],prime2_region[1][0]) #겹치는 영역 y2 하단
        
#         intersection=max(0,intersection_x2-intersection_x1)*max(0,intersection_y1-intersection_y2) 

#         box1_area = abs((prime_region[0][1]-prime_region[0][0])*(prime_region[1][1]-prime_region[1][0]))
#         box2_area = abs((prime2_region[0][1]-prime2_region[0][0])*(prime2_region[1][1]-prime2_region[1][0]))

#         IoU= intersection/(box1_area+box2_area -intersection +1e-7)

#         # print('IoU:',IoU)
        
#         if IoU < 0.2 :
#             return False
#         else:
#             return True

#     def print_output_message(self):
#         message=[]
#         position,_=self.position_check()
#         prime=self.prime_list[:]
#         message.append(f'id: 0, category: Box, class_name: KLT_Bin_box obj_transform: position: {position[0]}')
#         for number, object in enumerate(prime):
#             message.append(f'id: {self.prime_tuple[object]}, category: objects, class_name: {self.prime_tuple_name[object]} obj_transform: position: {position[0]}')
        
#         message_relationship=self.comput_relationship()
#         message.append(message_relationship)
#         return message


###########################################################################################



collision=CollisionCheck()

                    #    "/World/obj/master_chef_can",
                    #    "/World/obj/wood_block",
                    #    "/World/obj/pitcher_base",

for i in range(5000000):
    # collision.prime_tuple={a0: 1, a1: 2, a2: 3, a3: 4, 
    #                     #    a4: 5, 
    #                        a5: 5, a7: 6, a8: 7,a9:8, ax: 9, ax1:10}
    # collision.prime_tuple_name={a0:"cracker_box", a1:"sugar_box", a2:"tomato_can", a3:"gelatin_box",
    #             #   a4:"mustard_bottle",
    #                 a5:"meat_can", a7:"bleach_cleanser", a8: 'pudding_box',a9:'master_chef_can', ax:'wood_block', ax1:'pitcher_base'}
    # collision.prime_list=[a0,a1,a2,a3,
    #                     #   a4,
    #                       a5,a7,a8,a9,ax,ax1]
    # collision.background_prime=ab   

    # if i%500==0:
    #     print("iter: " + str(i))
    #     compute_list=collision.print_output_message()
    #     unknown_list=collision.make_a_unknown()
    #     # print(compute_list)
    #     collision.make_a_unknown()
    # if i==500:
    #     f = open('/home/kimseungjun/.local/share/ov/pkg/isaac_sim-2022.2.0/graph_data/graph.txt','w+')
    #     for i in compute_list:
    #         # print(i) 
    #         f.write(i+"\n")
    #     f.close()
    # if i==600:
    #     with open('/home/kimseungjun/.local/share/ov/pkg/isaac_sim-2022.2.0/graph_data/graph_unknown.txt','w+') as f:
    #         for j in unknown_list:
    #             f.write(j+"\n")
    world.step(render=True) # execute one physics step and one rendering step
    #i+=1


simulation_app.close() # close Isaac Sim