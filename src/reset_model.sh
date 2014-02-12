rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'gcb'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 1.001
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
reference_frame: '' " -r 200
