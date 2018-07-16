from laneFinding import find_lane
from preprocessing import *
from pathPlanning import calc_central_line, calculate_offset, offset_in_centimeters,calculate_offset_angle



def pipeline_fast(frame, img_size, src, dst):
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
    print(vehicle_offset,vehicle_offset_cm,offset_angle)

    cv2.waitKey(0)




path = "test_videos/"
file = "output1280.avi"

cap = cv2.VideoCapture(path + file)
if cap.isOpened() == False:
    print("Cannot open input video")
    exit()

frameNumber = 0

img_size = [200, 360, 3]

src = np.float32([[20, 200], [350, 200],
                  [275, 120], [85, 120]])

dst = np.float32([[0, img_size[0]], [img_size[1], img_size[0]],
                  [img_size[1], 0], [0, 0]])


while (cv2.waitKey(1) != 27):
    frameNumber += 1
    ret, frame = cap.read()
    resized = cv2.resize(frame.copy(), (img_size[1], img_size[0]))
    cv2.imshow("frame",resized)
    try:
        pipeline_fast(resized,img_size,src,dst)
    except: pass

    if cv2.waitKey(1) == ord('q'):
        break
