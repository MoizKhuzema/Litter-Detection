import cv2
import time
from track import Track
from track import tracks


# ------ STEP 1: READ VIDEO FILE ------
def read_video(camera_name, camera_url):
    reconnect_time = 30
    connection_flag = True
    total_tries = 10

    while(connection_flag == True or total_tries > 0):
        try:
            # read video
            cap = cv2.VideoCapture(camera_url)
            # check if connection failed
            if cap is None or not cap.isOpened():
                raise ConnectionError
            else:
                connection_flag = False
        # if connection failed, retry in 30 seconds
        except ConnectionError:
            print("Retrying connection to ", camera_name," in ", str(reconnect_time), " seconds...")
            time.sleep(reconnect_time)

        total_tries -= 1
    return cap


# ------ STEP 2: PROCESS FRAME TO DETECT MOVING OBJECTS ------
def process_frame(current_frame, prev_frame):
    # Convert to grayscale
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    prev_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

    # Gaussian blur to denoise image
    current_frame = cv2.GaussianBlur(current_frame, (5, 5), 0)
    prev_frame = cv2.GaussianBlur(prev_frame, (5, 5), 0)

    # apply frame differencing, thresholding to detect moving area
    frame_diff = cv2.absdiff(current_frame, prev_frame)
    # global thresholding
    ret, frame_diff = cv2.threshold(frame_diff, 15, 255, cv2.THRESH_BINARY) 
    # adaptive thresholding
    # frame_diff = cv2.adaptiveThreshold(frame_diff, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 2)

    # dilate frame
    dilate_frame = cv2.dilate(frame_diff, None, iterations=2)

    # Find object contours
    MAX_AREA = 50
    contours, _ = cv2.findContours(dilate_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)     
    if len(contours) > 0:
        # Remove Contours with area greater than 500
        contours = list(filter(lambda cnt: cv2.contourArea(cnt) < MAX_AREA, contours))
        # Find bounding box of contour with highest y-value
        contours = sorted(contours, key=lambda cnt: cv2.boundingRect(cnt)[1])
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            box = (x, y, x + w, y + h)
            return box
        return None
    else:
        return None


# ------ STEP 3: TRACK MOVING OBJECT ------
def clean_tracks():
    global tracks
    # Delete tracks that are invalid
    for track in tracks.copy():
        if track.track_status == 'invalid':
            tracks.remove(track)
        elif track.track_status == 'undefined' and track.frames_since_update > 15:
            tracks.remove(track)
    

def track_trajectory(box, x, y, frame):
    global tracks

    # if box is empty
    if x == 0 and y == 0:
        if tracks.__len__() > 0:
            for track in tracks:
                track.frames_since_update += 1
        # Delete invalid trajectories, clean tracks
        clean_tracks()
        return None

    # If this is first object detected
    if tracks.__len__() == 0:
        trk = Track()
        trk.location = [x, y]
        trk.kf.predict(x, y)
        trk.path[frame] = box
        tracks.append(trk)
        return None

    min_dist = 100000000
    flag = False
    # Finding closest trajectory to the object
    for track in tracks:
        if track.is_near(x, y):
            if track.distance_from(x, y) < min_dist and track.track_status != 'completed':
                closest_track = track
                flag = True
            else:
                track.frames_since_update += 1
        else:
            track.frames_since_update += 1

    if flag:
        # update trajectory of track
        closest_track.update(x, y)
        closest_track.path[frame] = box
    else:
        # Create new track
        trk = Track()
        trk.location = [x, y]
        trk.kf.predict(x, y)
        trk.path[frame] = box
        tracks.append(trk)

    # Delete invalid trajectories, clean tracks
    clean_tracks()


# ------ STEP 4: DRAW BOUNDING BOXES ------
def draw_bounding_box(frame, box):
    cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
    return frame


def draw_trace_line(frame, prev_frame):
    # color, thickness of trace line
    color = (255, 0, 0)
    thickness = 2

    # draw trace line
    line_start = (track.path[prev_frame][0], track.path[prev_frame][1])
    line_end = (track.path[frame][0], track.path[frame][1])
    frame = cv2.line(frame, line_start, line_end, color, thickness, cv2.LINE_4)
    return frame


def generate_evidence(track, out):
    loop_counter = 0
    for frame, box in track.path.items():
        # draw bounding box
        frame = draw_bounding_box(frame, box)
        # draw trace lines
        if loop_counter > 0:
            frame = draw_trace_line(frame, prev_frame)
        out.write(current_frame)
        prev_frame = frame
        loop_counter += 1


if __name__ == '__main__':
    # Initialize variables
    input = r'C:\Users\moizk\Desktop\Upwork\Teo-Zhang\high-rise-littering\data-video\video_demo.mp4'
    save_name = r'C:\Users\moizk\Desktop\Upwork\Teo-Zhang\high-rise-littering\frames' + r'\video.mp4'
    camera_name = 'camera_1'
    camera_url = 'P@ssw0rd@192.168.1.100:554/Streaming/Channels/101'

    # Step 1
    cap = read_video(camera_name, camera_url)
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    
    # define codec and create VideoWriter object
    out = cv2.VideoWriter(
        save_name,
        cv2.VideoWriter_fourcc(*'mp4v'), 10, 
        (frame_width, frame_height)
    )
    
    frame_count = 1
    _, prev_frame = cap.read()

    while (cap.isOpened()):
        # get current frame

        ret, current_frame = cap.read()
        if ret == True:
            frame_count += 1

            # Step 2
            box = process_frame(current_frame.copy(), prev_frame)
            if box != None:
                center_x = box[0] + (box[2] - box[0]) / 2
                center_y = box[1] + (box[3] - box[1]) / 2
            else:
                center_x = 0
                center_y = 0

            # Step 3
            track_trajectory(box, center_x, center_y, current_frame)
            prev_frame = current_frame

            # Step 4
            for track in tracks:
                if track.track_status == 'completed':
                    generate_evidence(track, out)
                    tracks.remove(track)
        else:
            break

    # Step 4
    cap = read_video(input)
    frame_count = 1
    prev_frame_count = 0
    while (cap.isOpened()):
        # get current frame
        ret, current_frame = cap.read()
        if ret == True:
            frame_count += 1
            
        else:
            break
  
    cap.release()
    cv2.destroyAllWindows()