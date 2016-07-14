import cv2
import matplotlib.pyplot as plt 
from time import time

test_image = cv2.imread('/home/ubuntu/Original_Images_Bag/tests/test1.jpg')
# cv2.imshow('test_image', test_image)
# cv2.waitKey(0)

duckiebot_classifier =  cv2.CascadeClassifier('classifier/cascade.xml')

t0 = time()
duckiebots = duckiebot_classifier.detectMultiScale(test_image, 1.1, 2, maxSize=(100,100))
print "Time taken for detection: %f" % round((time()-t0),3)

print "Duckiebot detected: %d" % (len(duckiebots))

for (x,y,w,h) in duckiebots:
	cv2.rectangle(test_image, (x,y), (x+w, y+h), (255,0,0), 2)


plt.figure(figsize=(9,9))
plt.axis('off')
plt.imshow(cv2.cvtColor(test_image, cv2.COLOR_BGR2RGB))
plt.show('hold')
