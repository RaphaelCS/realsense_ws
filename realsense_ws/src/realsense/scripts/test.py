from quadrotor import Quadrotor

UAV_NUM = 2


quadros = []
for i in range(UAV_NUM):
    quadros.append(Quadrotor(node_name='uav', rate=30))
    namespace = "uav%d" % (i+1)

    quadros[-1].add_state(ns=namespace)
    quadros[-1].add_front_cam(ns=namespace)
    quadros[-1].add_realsense_cam(ns=namespace)

    quadros[-1].add_control(ns=namespace)

    quadros[-1].enable_motors(ns=namespace)


while not quadros[0].is_shutdown():
    flag = 0
    if quadros[0].go_to_target([5, 0, 2, 0], 0.02):
        flag += 1
    if quadros[1].go_to_target([0, 0, 2, 0], 0.02):
        flag += 1
    if flag == UAV_NUM:
        break

while not quadros[0].is_shutdown():
    flag = 0
    if quadros[0].go_to_target([0, 0, 2, 0], 0.05):
        flag += 1
    if quadros[1].go_to_target([-5, 5, 2, 0], 0.05):
        flag += 1
    if flag == UAV_NUM:
        break
    quadros[0].display_realsense_cam()
