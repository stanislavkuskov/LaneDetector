from laneFinding import find_lane
from preprocessing import *
from pathPlanning import calc_central_line, calculate_offset, offset_in_centimeters,calculate_offset_angle
from pid import Robot, run_PID_once
from PyV4L2Camera.camera import Camera
from PIL import Image

def pipeline_fast(frame, img_size, src, dst, robot):
    # Preprocessing
    combined_thresholded = colorthresh(frame)
    warped_masked_img, M_warp, Minv = warp_image(combined_thresholded, src, dst, (img_size[1], img_size[0]))
    # lane finding
    ploty, lefty, righty, leftx, rightx, left_fitx, right_fitx, out_img = find_lane(warped_masked_img)

    # print(leftx,left_fitx)
    center_fitx, center_offset = calc_central_line(left_fitx, right_fitx, img_size, out_img, imshow=True)
    # center_fitx[-1] - last element in array, botton element of road center
    vehicle_offset = calculate_offset(img_size, center_fitx[-1])
    vehicle_offset_cm = offset_in_centimeters(vehicle_offset, 20, left_fitx[-1],right_fitx[-1])
    offset_angle = calculate_offset_angle(vehicle_offset_cm,maximum_support_vehicle_offset=5, maximum_wheel_angle=15)

    robot.set(0, vehicle_offset_cm, 0)
    # print(robot.y,robot.prew_y)
    x_t_PID, y_t_PID, steering  = run_PID_once(robot, 1.0, 2.8, 0.0004)
    # print(y_t_PID)

    print("vehicle_offset_cm: {}, offset_angle: {}, pid {}, steering: {} ".format(vehicle_offset_cm,offset_angle,y_t_PID,steering))
    print("_______________--")
    # cv2.waitKey(0)



fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output1280x.avi', fourcc, 30.0, (1280, 720))
#     # rpi, ESC, STEER = setup_gpio()
camera = Camera('/dev/video1', 1280, 720)
controls = camera.get_controls()

speed = 1500
angle = 90
DEFAULT_CMD = '11/1500/90'

# for control in controls:
#     print(control.name + ":" + str(control.id))
# camera.set_control_value(ControlIDs.BRIGHTNESS, 0)
# camera.set_control_value(ControlIDs.CONTRAST, 40)
# camera.set_control_value(ControlIDs.BACKLIGHT_COMPENSATION, 50)
# camera.set_control_value(ControlIDs.SATURATION, 0)

# 360x200

img_size = [200, 360, 3]

# src = np.float32([[0, 190], [360, 190],
#                   [280, 90], [80, 90]])

src = np.float32([[20, 200], [350, 200],
                  [275, 120], [85, 120]])

dst = np.float32([[0, img_size[0]], [img_size[1], img_size[0]],
                  [img_size[1], 0], [0, 0]])


robot = Robot()
robot.set_noise(np.pi/48, 0)
robot.set_steering_drift(10.0/180*np.pi)




while (cv2.waitKey(1) != 27):
    frame = camera.get_frame()
    # Decode the image
    im = Image.frombytes('RGB', (camera.width, camera.height), frame, "raw", "BGR")
    # Convert the image to a numpy array
    cv_arr = np.asarray(im)

    out.write(cv_arr)
    resized = cv2.resize(cv_arr.copy(), (img_size[1], img_size[0]))

    try:
        pipeline_fast(resized,img_size,src,dst,robot)
    except:
        print("control error")

    if cv2.waitKey(1) == ord('q'):
        break

out.release()
camera.close()
cv2.destroyAllWindows()
