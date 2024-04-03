#launch Isaac Sim before any other imports
#default first two lines in any standalone application
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

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
import omni

import numpy as np
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims




extensions.enable_extension("omni.isaac.ros_bridge")

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

# arm_base = world.scene.add(
#     FixedCuboid(
#         prim_path="/World/arm_base",
#         name="arm_base",
#         position=np.array([0, 0, franka_h/2]),
#         scale=np.array([0.2, 0.2, franka_h]),
#         color=np.array([5, 5, 5]),
#     ),
#     )

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

# franka = world.scene.add(
#     Franka(
#         prim_path="/World/Franka",
#         name = "Franka",
#         position=np.array([0, 0, franka_h]),
#     )
# )
add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/KLT_Bin/small_KLT.usd", prim_path="/World/obj/klt")
ab = world.scene.add(
            RigidPrim(prim_path="/World/obj/klt", name="klt",translation=np.array([0.05, -0.1, 0.06]),scale=np.array([1.5, 1.5, 0.8]),),
        )
# add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/KLT_Bin/small_KLT.usd", prim_path="/World/obj/klt1")
# ab = world.scene.add(
#             RigidPrim(prim_path="/World/obj/klt1", name="klt1", orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])),translation=np.array([0.05, 0.3, 0.06]),scale=np.array([1.2, 1.2, 0.8]),),
#         )


add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd", prim_path="/World/obj/cracker_box")
a0 = world.scene.add(
            RigidPrim(prim_path="/World/obj/cracker_box", name="cracker_box",translation=np.array([0.05, -0.1, 0.08]),scale=np.array([0.7, 0.7, 0.7]),mass=0.0004),
        )

add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd", prim_path="/World/obj/sugar_box")
a1 = world.scene.add(
            RigidPrim(prim_path="/World/obj/sugar_box", name="sugar_box",translation=np.array([0.05, -0.1, 0.14]),scale=np.array([0.5, 0.5, 0.5]),mass=0.0004),
        )

add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd", prim_path="/World/obj/tomato_can")
a2 = world.scene.add(
            RigidPrim(prim_path="/World/obj/tomato_can", name="tomato_can",translation=np.array([0.05, -0.1, 0.2]),scale=np.array([1, 1, 1]),mass=0.0004),
        )


add_reference_to_stage(usd_path="/home/kimseungjun/Downloads/isaac_sim-assets-1-2023.1.0/Assets/Isaac/2023.1.0/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd", prim_path="/World/obj/tomato_can1")
a3 = world.scene.add(
            RigidPrim(prim_path="/World/obj/tomato_can1", name="tomato_can1",orientation=euler_angles_to_quat(np.array([np.pi/2, 0, np.pi/2])),translation=np.array([0.135, -0.1, 0.12]),scale=np.array([1, 1, 1]),mass=0.0004),
        )



############## Camera Publishing Functions ###############






# Paste functions from the tutorials here
def publish_camera_tf(camera: Camera):

    camera_prim = camera.prim_path


    if not is_prim_path_valid(camera_prim):

        raise ValueError(f"Camera path '{camera_prim}' is invalid.")


    try:

        # Generate the camera_frame_id. OmniActionGraph will use the last part of

        # the full camera prim path as the frame name, so we will extract it here

        # and use it for the pointcloud frame_id.

        camera_frame_id=camera_prim.split("/")[-1]


        # Generate an action graph associated with camera TF publishing.

        ros_camera_graph_path = "/CameraTFActionGraph"


        # If a camera graph is not found, create a new one.

        if not is_prim_path_valid(ros_camera_graph_path):

            (ros_camera_graph, _, _, _) = og.Controller.edit(

                {

                    "graph_path": ros_camera_graph_path,

                    "evaluator_name": "execution",

                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,

                },

                {

                    og.Controller.Keys.CREATE_NODES: [

                        ("OnTick", "omni.graph.action.OnTick"),

                        ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

                        ("RosPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),

                    ],

                    og.Controller.Keys.CONNECT: [

                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),

                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),

                    ]

                }

            )


        # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.

        og.Controller.edit(

            ros_camera_graph_path,

            {

                og.Controller.Keys.CREATE_NODES: [

                    ("PublishTF_"+camera_frame_id, "omni.isaac.ros_bridge.ROS1PublishTransformTree"),

                    ("PublishRawTF_"+camera_frame_id+"_world", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),

                ],

                og.Controller.Keys.SET_VALUES: [

                    ("PublishTF_"+camera_frame_id+".inputs:topicName", "/tf"),

                    # Note if topic_name is changed to something else besides "/tf",

                    # it will not be captured by the ROS tf broadcaster.

                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:topicName", "/tf"),

                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:parentFrameId", camera_frame_id),

                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:childFrameId", camera_frame_id+"_world"),

                    # Static transform from ROS camera convention to world (+Z up, +X forward) convention:

                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),

                ],

                og.Controller.Keys.CONNECT: [

                    (ros_camera_graph_path+"/OnTick.outputs:tick",

                        "PublishTF_"+camera_frame_id+".inputs:execIn"),

                    (ros_camera_graph_path+"/OnTick.outputs:tick",

                        "PublishRawTF_"+camera_frame_id+"_world.inputs:execIn"),

                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",

                        "PublishTF_"+camera_frame_id+".inputs:timeStamp"),

                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",

                        "PublishRawTF_"+camera_frame_id+"_world.inputs:timeStamp"),

                ],

            },

        )

    except Exception as e:

        print(e)


    # Add target prims for the USD pose. All other frames are static.

    set_target_prims(

        primPath=ros_camera_graph_path+"/PublishTF_"+camera_frame_id,

        inputName="inputs:targetPrims",

        targetPrimPaths=[camera_prim],

    )

    return
def publish_camera_info(camera: Camera, freq):

    # The following code will link the camera's render product and publish the data to the specified topic name.

    render_product = camera._render_product_path

    step_size = int(60/freq)

    topic_name = camera.name+"_camera_info"

    queue_size = 1

    node_namespace = ""

    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.


    stereo_offset = [0.0, 0.0]


    writer = rep.writers.get("ROS1PublishCameraInfo")

    writer.initialize(

        frameId=frame_id,

        nodeNamespace=node_namespace,

        queueSize=queue_size,

        topicName=topic_name,

        stereoOffset=stereo_offset,

    )

    writer.attach([render_product])


    gate_path = omni.syntheticdata.SyntheticData._get_node_path(

        "PostProcessDispatch" + "IsaacSimulationGate", render_product

    )


    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate

    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return
def publish_rgb(camera: Camera, freq):

    # The following code will link the camera's render product and publish the data to the specified topic name.

    render_product = camera._render_product_path

    step_size = int(60/freq)

    topic_name = camera.name+"_rgb"

    queue_size = 1

    node_namespace = ""

    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.


    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

    writer = rep.writers.get(rv + "ROS1PublishImage")

    writer.initialize(

        frameId=frame_id,

        nodeNamespace=node_namespace,

        queueSize=queue_size,

        topicName=topic_name

    )

    writer.attach([render_product])


    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(

        rv + "IsaacSimulationGate", render_product

    )

    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


    return
def publish_depth(camera: Camera, freq):


    # The following code will link the camera's render product and publish the data to the specified topic name.

    render_product = camera._render_product_path

    step_size = int(60/freq)

    topic_name = camera.name+"_depth"

    queue_size = 1

    node_namespace = ""

    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.


    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(

                            sd.SensorType.DistanceToImagePlane.name

                        )

    writer = rep.writers.get(rv + "ROS1PublishImage")

    writer.initialize(

        frameId=frame_id,

        nodeNamespace=node_namespace,

        queueSize=queue_size,

        topicName=topic_name

    )

    writer.attach([render_product])


    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(

        rv + "IsaacSimulationGate", render_product

    )

    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


    return

##########################################################

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.44, -0.1, 1.7]),
    frequency=20,
    resolution=(2048, 2048),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)
camera.initialize()

camera.set_focal_length(0.6)
camera.set_focus_distance(0) 
camera.set_horizontal_aperture(0.23)
# ############### Calling Camera publishing functions ###############


# # Setup publishers

publish_camera_tf(camera)

publish_camera_info(camera, 30)

publish_rgb(camera, 30)

publish_depth(camera, 30)

# publish_pointcloud_from_depth(camera, 30)

# import omni.replicator.core as rep

# from omni.isaac.core import World

# from omni.isaac.core.objects import DynamicCuboid

# from omni.isaac.core.utils.semantics import add_update_semantics

# from PIL import Image
# import numpy as np
# #####################################################################
# cam = rep.create.camera(camera)
# rp = rep.create.render_product(cam, RESOLUTION)
# distance_to_camera = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
# distance_to_camera.attach(rp)
# distance_to_camera_data = distance_to_camera.get_data(device="cpu")
# rep.orchestrator.step()


# keys = og.Controller.Keys
# og.Controller.edit(
#     {"graph_path": "/panda_graph", "evaluator_name": "execution"},
#     {
#         keys.CREATE_NODES: [
#             ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#             ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
#             ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
#             ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
#             ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
#             ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
#             ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
#             ("isaac_create_viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
#             ("isaac_get_viewport_render_product", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
#             ("isaac_set_camera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
#             ("ros1_camera_helper", "omni.isaac.ros_bridge.ROS1CameraHelper"),

#             ("obj_ros1_publish_transform_tree", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
#         ],
        
#         keys.SET_VALUES: [
#             ("SubscribeJointState.inputs:topicName", "joint_command"),
#             ("ArticulationController.inputs:robotPath", "/World/Franka"),
#             ("ArticulationController.inputs:usePath", True),
#             ("PublishJointState.inputs:topicName", "joint_states"),
#             ("PublishClock.inputs:topicName", "clock"),
#             ("PublishTF.inputs:topicName","/tf"),
#             ("isaac_create_viewport.inputs:name", "realsense_viewport"),
#             ("ros1_camera_helper.inputs:frameId", "camera_rgb_optical_frame"),
#             ("ros1_camera_helper.inputs:topicName", "/camera/depth_registered/rgb"),
#             ("ros1_camera_helper.inputs:type", "rgb"),

#             ("obj_ros1_publish_transform_tree.inputs:topicName","/tf_obj"),

#         ],
#         keys.CONNECT: [
#             ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
#             ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
#             ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
#             ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
#             ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
#             ("OnPlaybackTick.outputs:tick", "isaac_create_viewport.inputs:execIn"),
#             ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
#             ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
#             ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
#             ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
#             ("ReadSimTime.outputs:simulationTime","PublishJointState.inputs:timeStamp"),
#             ("ReadSimTime.outputs:simulationTime","PublishClock.inputs:timeStamp"),
#             ("ReadSimTime.outputs:simulationTime","PublishTF.inputs:timeStamp"),
#             ("isaac_create_viewport.outputs:execOut", "isaac_get_viewport_render_product.inputs:execIn"),
#             ("isaac_create_viewport.outputs:viewport", "isaac_get_viewport_render_product.inputs:viewport"),
#             ("isaac_get_viewport_render_product.outputs:execOut", "isaac_set_camera.inputs:execIn"),
#             ("isaac_get_viewport_render_product.outputs:renderProductPath", "isaac_set_camera.inputs:renderProductPath"),
#             ("isaac_get_viewport_render_product.outputs:renderProductPath", "ros1_camera_helper.inputs:renderProductPath"),
#             ("isaac_set_camera.outputs:execOut", "ros1_camera_helper.inputs:execIn"),

#             ("OnPlaybackTick.outputs:tick", "obj_ros1_publish_transform_tree.inputs:execIn"),
#             ("ReadSimTime.outputs:simulationTime","obj_ros1_publish_transform_tree.inputs:timeStamp"),

#         ],
#     },
# )




from omni.isaac.core.utils.prims import set_targets

# set_targets(
#     prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/PublishJointState"),
#     attribute="inputs:targetPrim",
#     target_prim_paths=["/World/Franka"],
# )

# Setting the /Franka target prim to Publish Transform Tree node
# set_targets(
#     prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/PublishTF"),
#     attribute="inputs:targetPrims",
#     target_prim_paths=["/World/Franka","/World/Franka/panda_hand/geometry/realsense/realsense_camera"],
# )
# set_targets(
#     prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/isaac_set_camera"),
#     attribute="inputs:cameraPrim",
#     target_prim_paths=["/World/Franka/panda_hand/geometry/realsense/realsense_camera"],
# )

set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/panda_graph/obj_ros1_publish_transform_tree"),
    attribute="inputs:targetPrims",
    target_prim_paths=["/World/obj",
                       "/World/obj/klt",
                       "/World/obj/cracker_box",
                       "/World/obj/sugar_box",
                       "/World/obj/tomato_can",
                       "/World/obj/tomato_can1",
                       "/World/obj/mustard_bottle",
                       "/World/obj/meat_can",
                       "/World/obj/bleach_cleanser",
                       ],
)

simulation_app.update()
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

#import subprocess

#cmd = ["cd ./ros_workspace && source devel/setup.bash && roslaunch gnn_planning gnn_planning.launch"]
#subprocess.run("roslaunch gnn_planning gnn_planning.launch", shell=True)
#subprocess.run("cd ./ros_workspace", shell=True)
#subprocess.run("source devel/setup.bash", shell=True)
#subprocess.run("roslaunch gnn_planning gnn_planning.launch", shell=True)
#for i in range(500):
#i=1
for i in range(500000):
    if i%100==0:
        # position, orientation = box2.get_world_pose()
        # linear_velocity = box2.get_linear_velocity()
        # will be shown on terminal
        print("iter: " + str(i))
        # print("Cube position is : " + str(position))
        # print("Cube's orientation is : " + str(orientation))
        # print("Cube's linear velocity is : " + str(linear_velocity))
        # print(distance_to_camera_data)



    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step
    #i+=1


simulation_app.close() # close Isaac Sim