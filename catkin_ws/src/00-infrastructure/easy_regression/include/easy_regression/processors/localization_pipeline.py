from easy_regression.processor_interface import ProcessorInterface
import duckietown_utils as dtu
from ground_projection import \
    GroundProjection
from complete_image_pipeline.pipeline import run_pipeline

class LocalizationPipelineProcessor(ProcessorInterface):
    
    def __init__(self, line_detector, image_prep, lane_filter, anti_instagram):    
        self.line_detector = line_detector
        self.image_prep = image_prep
        self.lane_filter = lane_filter
        self.anti_instagram = anti_instagram
        
    def process_log(self, bag_in, bag_out):
        
        vehicle_name = dtu.which_robot(bag_in)
    
        dtu.logger.info('Vehicle name: %s' % vehicle_name) 
        
        gp = GroundProjection(vehicle_name) 
    
        topic = dtu.get_image_topic(bag_in)
        
        if isinstance(bag_in, dtu.BagReadProxy):
            start_time = bag_in.get_physical_log_start_time()
        else:
            start_time = bag_in.get_start_time() 
            
        sequence = dtu.d8n_bag_read_with_progress(bag_in, topic, yield_tuple=True)
        for i, (topic, msg, t) in enumerate(sequence):
            
            rel_time = t.to_sec() - start_time
            
            rgb = dtu.rgb_from_ros(msg)
            
            all_details = False
            bgr = dtu.bgr_from_rgb(rgb)
            res, _stats = run_pipeline(bgr, gp=gp,
                                        line_detector_name=self.line_detector, 
                                        image_prep_name=self.image_prep,
                                        lane_filter_name=self.lane_filter,
                                        anti_instagram_name=self.anti_instagram,
                                        all_details=all_details)
            
            rect = (480, 640) if not all_details else (240, 320)
            res = dtu.resize_images_to_fit_in_rect(res, rect, bgcolor=dtu.ColorConstants.BGR_DUCKIETOWN_YELLOW)

            headers = [
                "frame %5d: %.2f s" % (i, rel_time),
                "image_prep: %s | line_detector: %s | lane_filter: %s" % (self.line_detector, 
                                                                      self.image_prep,self.lane_filter,)
            ]
            
            res = dtu.write_bgr_images_as_jpgs(res, dirname=None)
            
            cv_image = res['all']
            
            for head in reversed(headers):
                max_height = 35
                cv_image = dtu.add_header_to_bgr(cv_image, head, max_height=max_height)
#             cv_image = d8_image_resize_fit(cv_image, W=1280)
            
            otopic = "/localization_pipeline/all"
#             omsg = d8n_image_msg_from_cv_image(cv_image, "bgr8", same_timestamp_as=msg)
            omsg = dtu.d8_compressed_image_from_cv_image(cv_image, same_timestamp_as=msg)
            bag_out.write(otopic, omsg)

