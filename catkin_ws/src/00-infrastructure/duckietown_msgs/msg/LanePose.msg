Header header
float32 d   #lateral offset
float32 d_ref #lateral offset reference
float32 sigma_d # Variance of offset
float32 phi #heading error
float32 phi_ref #heading error
float32 sigma_phi # Variance of heading error
float32 c_ref # Refernece Curvature
float32 v_ref # Referenece Velocity
int32 status #Status of duckietbot 0 if normal, 1 if error is encountered
bool in_lane #Status of duckietbot in lane
#Enum for status
int32 NORMAL=0
int32 ERROR=1 
