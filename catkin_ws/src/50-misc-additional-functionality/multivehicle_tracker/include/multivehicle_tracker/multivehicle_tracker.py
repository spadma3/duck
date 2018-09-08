import threading
import uuid
from .tracklet import Tracklet


class NaiveMultitargetTracker:

    def __init__(self, max_nearest_neighbor=0.20):
        self.lock = threading.Lock()
        self.max_nearest_neighbor = max_nearest_neighbor
        self.targets = {}

    def add_detection(self, x, y, confidence, timestamp):
        min_similarity = self.max_nearest_neighbor
        nearest_neighbor = None
        with self.lock:
            for tracklet_id in self.targets:
                tracklet = self.targets.get(tracklet_id)
                similarity = tracklet.similarity((x, y), timestamp)
                if similarity <= min_similarity:
                    min_similarity = similarity
                    nearest_neighbor = tracklet_id

            if nearest_neighbor is not None:
                tracklet = self.targets[nearest_neighbor]
                tracklet.update((x, y), 1 - confidence)
                tracklet.status = Tracklet.STATUS_TRACKING
            else:
                global_id = uuid.uuid1()
                self.targets[global_id] = Tracklet(global_id, x, y, confidence, timestamp)

    def time_elapsed(self, timestamp):
        with self.lock:
            for tracklet_id in self.targets:
                tracklet = self.targets[tracklet_id]
                tracklet.predict(timestamp)
                confidence = tracklet.confidence()
                if confidence > 0.05:
                    tracklet.status = Tracklet.STATUS_LOST

    def clean_dead(self):
        dead_targets = [tracklet_id for tracklet_id in self.targets
                         if self.targets[tracklet_id].status == Tracklet.STATUS_LOST]
        with self.lock:
            for tracklet_id in dead_targets:
                self.targets.pop(tracklet_id)

    def get_tracklets_info(self, status=Tracklet.STATUS_TRACKING):
        with self.lock:
            return [
                self.targets[tracklet_id].get_tracklet_info()
                for tracklet_id in self.targets
                if self.targets[tracklet_id].status == status
            ]

