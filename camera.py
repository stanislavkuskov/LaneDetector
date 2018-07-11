from laneFinding import Line, find_base, find_lane, calc_central_line
from preprocessing import *


def pipeline_fast(frame, img_size, src, dst):
    combined_thresholded = colorthresh(frame)

    warped_masked_img, M_warp, Minv = warp_image(combined_thresholded, src, dst, (img_size[1], img_size[0]))
    ploty, lefty, righty, leftx, rightx, left_fitx, right_fitx, out_img = find_lane(warped_masked_img)
    central_fitx = calc_central_line(left_fitx, right_fitx, img_size, out_img, imshow=True)




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
