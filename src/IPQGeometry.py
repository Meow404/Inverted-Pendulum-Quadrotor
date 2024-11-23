import numpy as np

from pydrake.systems.framework import Context
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.all import (
    MultibodyPlant, 
    Parser, 
    MultibodyPositionToGeometryPose, 
    LeafSystem, 
    
)


def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)

class InvertedPendulumQuadrotorGeometry(LeafSystem):

    def __init__(self, scene_graph, Lp:float = 1):

        # Initialize leaf system
        LeafSystem.__init__(self)

        self._Lp = Lp   # Length of pendulum

        # Setup a multibody plant
        self.plant = MultibodyPlant(0.0)
        self.parser = Parser(self.plant, scene_graph)

        # Add the visualization models        
        self.quadrotor_instance = self.parser.AddModels(url="package://drake_models/skydio_2/quadrotor.urdf")
        self.pendulum_instance = self.parser.AddModels("res/Pendulum.urdf")

        # Connect the two models
        pendulum_base_frame = self.plant.GetFrameByName("base")
        quadrotor_base_frame = self.plant.GetFrameByName("base_link")

        self.plant.WeldFrames(quadrotor_base_frame, pendulum_base_frame, X_FM=xyz_rpy_deg([0, 0, 0.03], [0, 180, 0]),)

        # Finalize the model
        self.plant.Finalize()
        
        self.source_id = self.plant.get_source_id()

        # RPY rotation state
        self.quadrotor_state_input_port_index = self.DeclareVectorInputPort("state", 
                                                                            16).get_index() 
        # Quaternion rotation state
        self.quadrotor_quat_state_output_port_index = self.DeclareVectorOutputPort("state_quaternion", 
                                                                                   10, 
                                                                                   self.output_geometry_pose).get_index() 

    def output_geometry_pose(self, context: Context, IPQ_state):
        state = self.EvalVectorInput(context, \
                                     self.quadrotor_state_input_port_index).get_value()
        quat = RollPitchYaw(state[3:6]).ToQuaternion()
        
        x = state[6]
        y = state[7]
        z = np.sqrt(pow(self._Lp,2) - pow(x, 2) - pow(y, 2))
        alpha = np.arctan2(x,y)
        beta = np.arcsin(z/self._Lp) - np.pi/2

        IPQ_state.set_value([quat.w(), quat.x(), quat.y(), quat.z()]    # Quadrotor Quaternion Rotation
                            + list(state[0:3])                          # Quadrotor x-y-z Position
                            + [-beta, 0, alpha]                   # Pendulum Angle
                           ) 

    @staticmethod
    def AddToBuilder(builder, quadrotor_state_port, scene_graph):

        # The visulaization of the Inverted Pendulum Quadrotor
        IPQGeometry = builder.AddSystem(InvertedPendulumQuadrotorGeometry(scene_graph))

        # Convert the quadrotor state output from the plant to joint position for visualizer
        builder.Connect(quadrotor_state_port, IPQGeometry
                        .get_input_port(IPQGeometry.quadrotor_state_input_port_index))

        # Feed joint positions to geometric poses for each link on the body
        ToGeometryPose = builder.AddSystem(MultibodyPositionToGeometryPose(IPQGeometry.plant))
        builder.Connect(IPQGeometry.get_output_port(IPQGeometry.quadrotor_quat_state_output_port_index), \
                        ToGeometryPose.get_input_port())
        
        builder.Connect(ToGeometryPose.get_output_port(), 
                        scene_graph.get_source_pose_port(IPQGeometry.source_id))

        return IPQGeometry
        
        
