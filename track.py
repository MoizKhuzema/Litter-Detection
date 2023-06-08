from kalmanfilter import KalmanFilter


# Global Variables
tracks = []
MAX_DIST = 45
MIN_DIST = 2
MAX_VERTICLE_DIST = 30
MAX_HORIZONTAL_DIST = 5


class Track:
    """ 
                Class contains track information
    Track_id:               (int) To identify unique trajectories
    track_status:           (boolean) Is this trajectory valid
    update_count:           (int) How many times has this trajectory been updated
    frames_since_update:    (int) How many frames since the last update
    kf:                     (class object) Kalman filter instance
    location:               (list) Center coordinate of object's last true position
    path:                   (dictionary) Maps frames to bounding boxes for complete path of trajectory
    
    """
    def __init__(self) -> None:
        self.track_id = tracks.__len__() + 1
        self.track_status = 'undefined'
        self.update_count = 1
        self.frames_since_update = 0
        self.kf = KalmanFilter()
        self.location = []
        self.path = {}


    def is_near(self, x, y):
        """ Is point (x, y) near this trajectory """
        if abs(x - self.location[0]) < MAX_DIST and abs(y - self.location[1]) < MAX_DIST:
            return True
        else:
            return False


    def distance_from(self, x, y):
        """ Distance between point (x, y) and trajectory """
        return abs(x - self.location[0]) + abs(y - self.location[1])


    def predict_path(self):
        """ Predict path of frames where object is hidden """
        # local variables
        val = list(self.path.values())
        diff_x = 0
        diff_y = 0
        prev_loc = val[0]
        loop_counter = 1

        # Get average difference for x, y
        while loop_counter < len(val):
            diff_x += (val[loop_counter][0] - prev_loc[0])
            diff_y += (val[loop_counter][1] - prev_loc[1])      
            prev_loc = val[loop_counter]
            loop_counter += 1
        diff_x /= loop_counter
        diff_y /= loop_counter

        # Add new location
        self.location = [val[-1][0] + diff_x, val[-1][1] + diff_y]


    def update(self, x, y):
        """ Check if new center (x, y) is valid trajectory and update status """
        # If track is invalid, ignore
        if self.track_status == 'invalid':
            return None

        # If object was hidden, predict using kalman filter
        if self.frames_since_update >= 6:
            self.predict_path()

        # if new position is valid
        if self.update_count >= 6 or (abs(x - self.location[0]) <= MAX_HORIZONTAL_DIST and MIN_DIST <= abs(y - self.location[1]) <= MAX_VERTICLE_DIST):
            # update location
            self.location = [x, y]
            self.update_count += 1
            self.frames_since_update = 0
            self.kf.predict(x, y)
            if self.track_status == 'valid' and self.frames_since_update >= 15:
                self.track_status = 'completed'
            else:
                self.track_status = 'valid'
        else:
            self.track_status = 'invalid'
        
        return None
