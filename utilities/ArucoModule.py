import cv2
import cv2.aruco as aruco
import numpy as np
import os
import utilities.misc as msc
import sys


def getCameraMatr(path, width=3, height=4, nameMatrix="cameraMatrix.txt", nameDistorcion="cameraDistortion.txt"):
    """
    :param path: folder where the matrices are located
    :param width: width number of pixel in the frame
    :param height: height number of pixel in the frame
    :param nameMatrix: name used to save the camera matrix
    :param nameDistorcion: name used to save the camera distortion
    :return: both camera matrices needed to calculate positions
    """
    calib_path = ""
    camera_matrix = np.loadtxt(calib_path + f'{path}/{nameMatrix}', delimiter=',')
    camera_distortion = np.loadtxt(calib_path + f'{path}/{nameDistorcion}', delimiter=',')
    # width = cap.get(cv2.cv2.CAP_PROP_FRAME_WIDTH)
    # height = cap.get(cv2.cv2.CAP_PROP_FRAME_HEIGHT)
    camera_matrix[0, 2] = width / 2
    camera_matrix[1, 2] = height / 2
    camera_matrix[1, 1] = -1 * camera_matrix[1, 1]
    return camera_matrix, camera_distortion


def loadAugImages(path):
    """"
    :param path: folder in which all the marker images with ids are stored
    :return: dictionary with key as the id and values as the augmented image
    """
    myList = os.listdir(path)
    noOfMarkers = len(myList)
    print("Total Number of Markers in the File: ", noOfMarkers)
    augDics = {}
    for imgPath in myList:
        key = int(os.path.splitext(imgPath)[0])
        imgAug = cv2.imread(f'{path}/{imgPath}')
        augDics[key] = imgAug

    return augDics


def findArucoMarkers(img, markerSize=4, totalMarkers=100, draw=True):
    """"
    Set up the dictionary ex. aruco.DICT_4X4_100
    :param img: image in which to find the aruco markers
    :param markerSize: the size of the markers
    :param totalMarkers: total number of markers that compose the dictionary
    :param draw: flag to draw bbox around markers detected
    :return: bounding boxes and id numbers of markers detected
    """
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)
    # print(ids)

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return bboxs, ids


def augmentAruco(bbox, id, img, imgAug, drawId=True):
    """
    :param bbox: the four corner points of the box
    :param id: marker id of the corresponding box used only for display
    :param img: the final image on wich to draw
    :param imgAug: the image that will overlapped on the marker
    :param drawId: flag to display the id of the detected markers
    :return: image with the augment image overlaid
    """
    tl = bbox[0][0][0], bbox[0][0][1]
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]

    h, w, c = imgAug.shape

    pts1 = np.array([tl, tr, br, bl])
    pts2 = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    matrix, _ = cv2.findHomography(pts2, pts1)
    imgOut = cv2.warpPerspective(imgAug, matrix, (img.shape[1], img.shape[0]))
    cv2.fillConvexPoly(img, pts1.astype(int), (0, 0, 0))
    imgOut = img + imgOut
    # print(tl)

    if drawId:
        cv2.putText(imgOut, str(id), tl, cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

    return imgOut


def draw_point(img, position, width, height, color=(0, 200, 0)):
    circle = np.ones((2, 2)) * 3 * position * [1, -1] + [width / 2, height / 2]
    circle = np.transpose(circle)
    # print the point
    cv2.polylines(img, [(np.transpose(circle)).astype(np.int32)], False, color, 7, cv2.LINE_AA)


def draw_axis(img, height, width, color=(100, 100, 100)):
    img[:, int(width / 2 + 0):int(width / 2 + 0) + 1] = color
    img[int(height / 2 - 0) - 1:int(height / 2 - 0), :] = color
    return img





# /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def main():
    focus = 0
    exposure = -5

    cap = cv2.VideoCapture(0)
    # imgAug = cv2.imread("Markers/23.png")
    augDics = loadAugImages("Markers")
    marker_size = 10.2  # - [cm]

    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)  # 1280 // 1920  //1600 //1024 //640
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 576)  # 720 // 1080  //896  // 576  //360

    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    camera_matrix, camera_distortion = getCameraMatr("camera", width=3, height=4)
    position = np.ones((3, 100))
    while True:
        try:
            secuess, img = cap.read()
            arucoFound = findArucoMarkers(img)

            # loop throug all the markers and augment each one
            if len(arucoFound[0]) != 0:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(arucoFound[0], marker_size, camera_matrix,
                                                                camera_distortion)

                for n, (bbox, id) in enumerate(zip(arucoFound[0], arucoFound[1])):
                    # draw augmented arucos

                    if int(id) in augDics.keys():
                        img = augmentAruco(bbox.astype(int), id, img, augDics[int(id)])
                    # save position
                    if id <= position.shape[1] :
                        theta = msc.angle_correction(bbox)  # get the angle value
                        position[0, id - 1] = tvec[n, 0, 0]
                        position[1, id - 1] = tvec[n, 0, 1]
                        position[2, id - 1] = theta

            cv2.imshow("Image", img)
            # cv2.waitKey(1)

            # --- use 'q' to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                cap.release()
                break


        except Exception as e:
            cv2.destroyAllWindows()
            cap.release()
            print("Problem with aruco system, please check Aruco Module")
            print(e)
            print("Error on line {}".format(sys.exc_info()[-1].tb_lineno))
            break

    print("fps :", cap.get(cv2.CAP_PROP_FPS))
    print("width : ", cv2.CAP_PROP_FRAME_WIDTH)
    print("height : ", cv2.CAP_PROP_FRAME_HEIGHT)
    return position


if __name__ == "__main__":
    main()
