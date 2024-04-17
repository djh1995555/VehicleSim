from proto.controller_pb2 import ControllerOutput
from proto.trajectory_pb2 import *
from google.protobuf import text_format

controller_output = ControllerOutput()
controller_output.id.append(0.0)

fields_by_name = controller_output.DESCRIPTOR.fields_by_name
 
field_names = [name for name, field in fields_by_name.items()]

for name, field in fields_by_name.items():
    field_value = getattr(controller_output, name)
    print("value of {} is {}".format(name, field_value))



# trajectory = Trajectory()
# trajectory.id.append(0.0)
# point = trajectory.trajectory_points.add()
# point.x = 1
# print(trajectory)