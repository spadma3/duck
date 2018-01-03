import numpy as np
import tensorflow as tf


class ObjectDetector:

    def __init__(self, inference_graph_path, score_threshold=0.2, denormalize_boundingbox=True):
        # Inference Attributes
        self.inference_graph = None
        self.tf_session = None
        self.image_tensor = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None
        self.num_detections = None

        # Detection Attributes
        self.score_threshold = score_threshold
        self.de_normalize = denormalize_boundingbox

        self.initialize_inference_engine(inference_graph_path)

    def load_inference_graph(self, inference_graph_path):
        if self.inference_graph is None:
            self.inference_graph = tf.Graph()
            with self.inference_graph.as_default():
                graph_definition = tf.GraphDef()
                with tf.gfile.GFile(inference_graph_path, 'rb') as fid:
                    serialized_graph = fid.read()
                    graph_definition.ParseFromString(serialized_graph)
                    tf.import_graph_def(graph_definition, name='')

    def initialize_inference_engine(self, inference_graph_path):
        if self.tf_session is None:
            self.load_inference_graph(inference_graph_path)
            self.inference_graph.as_default()
            self.tf_session = tf.InteractiveSession(graph=self.inference_graph)
            # TODO: add as part of the detector configuration
            self.image_tensor = self.inference_graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes = self.inference_graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.inference_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.inference_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.inference_graph.get_tensor_by_name('num_detections:0')

    def detect(self, image):
        image_tensor_input = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num) = self.tf_session.run(
            [self.detection_boxes,
             self.detection_scores,
             self.detection_classes,
             self.detection_scores],
            feed_dict={
                self.image_tensor: image_tensor_input
            })

        n_boxes = np.squeeze(boxes)
        n_scores = np.squeeze(scores)
        n_classes = np.squeeze(classes)

        image_shape = image.shape
        bounding_boxes = []
        for index, box in enumerate(n_boxes):
            if n_scores[index] >= self.score_threshold:
                ymin, xmin, ymax, xmax = box

                if self.de_normalize:
                    ymin = ymin * image_shape[0]
                    ymax = ymax * image_shape[0]
                    xmin = xmin * image_shape[1]
                    xmax = xmax * image_shape[1]

                bounding_boxes.append({
                    'class_label': 'bot',  # TODO: load this from configuration
                    'class_id': n_classes[index],
                    'score': n_scores[index],
                    'xmin': xmin,
                    'xmax': xmax,
                    'ymin': ymin,
                    'ymax': ymax,
                })

        return bounding_boxes

    def finalize(self):
        self.tf_session.close()


