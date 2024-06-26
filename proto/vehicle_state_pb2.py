# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: vehicle_state.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='vehicle_state.proto',
  package='',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x13vehicle_state.proto\"\xe7\x01\n\x0cVehicleState\x12\x0c\n\x01x\x18\x01 \x01(\x01:\x01\x30\x12\x0c\n\x01y\x18\x02 \x01(\x01:\x01\x30\x12\x0c\n\x01v\x18\x03 \x01(\x01:\x01\x30\x12\x0e\n\x03\x61\x63\x63\x18\x04 \x01(\x01:\x01\x30\x12\x0f\n\x04jerk\x18\x05 \x01(\x01:\x01\x30\x12\x16\n\x0b\x66ront_angle\x18\x06 \x01(\x01:\x01\x30\x12\x1f\n\x14steering_wheel_angle\x18\x07 \x01(\x01:\x01\x30\x12\x12\n\x07heading\x18\x08 \x01(\x01:\x01\x30\x12\x17\n\x0cheading_rate\x18\t \x01(\x01:\x01\x30\x12\x10\n\x05pitch\x18\n \x01(\x01:\x01\x30\x12\x14\n\tcurvature\x18\x0b \x01(\x01:\x01\x30')
)




_VEHICLESTATE = _descriptor.Descriptor(
  name='VehicleState',
  full_name='VehicleState',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='VehicleState.x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='VehicleState.y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='v', full_name='VehicleState.v', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='acc', full_name='VehicleState.acc', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='jerk', full_name='VehicleState.jerk', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front_angle', full_name='VehicleState.front_angle', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='steering_wheel_angle', full_name='VehicleState.steering_wheel_angle', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='heading', full_name='VehicleState.heading', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='heading_rate', full_name='VehicleState.heading_rate', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='VehicleState.pitch', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='curvature', full_name='VehicleState.curvature', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=24,
  serialized_end=255,
)

DESCRIPTOR.message_types_by_name['VehicleState'] = _VEHICLESTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

VehicleState = _reflection.GeneratedProtocolMessageType('VehicleState', (_message.Message,), dict(
  DESCRIPTOR = _VEHICLESTATE,
  __module__ = 'vehicle_state_pb2'
  # @@protoc_insertion_point(class_scope:VehicleState)
  ))
_sym_db.RegisterMessage(VehicleState)


# @@protoc_insertion_point(module_scope)
