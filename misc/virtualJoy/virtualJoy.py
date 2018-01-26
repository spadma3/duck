import pygame
import time
import rospy
from sensor_msgs.msg import Joy
import os, sys

screen_size = 300
speed_tang = 1.0
speed_norm = 1.0

def loop():
    veh_standing = True
    
    while True:

        
                
        # add dpad to screen
        screen.blit(dpad, (0,0))
        
        # prepare message
        msg = Joy()
        msg.header.seq = 0
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs = 0
        msg.header.frame_id = ''
        msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # obtain pressed keys
        keys = pygame.key.get_pressed()

        ### checking keys and executing actions ###
        
        # drive left
        if keys[pygame.K_LEFT]:
            screen.blit(dpad_l, (0,0))
            msg.axes[3] += speed_norm

        # drive right
        if keys[pygame.K_RIGHT]:
            screen.blit(dpad_r, (0,0))
            msg.axes[3] -= speed_norm

        # drive forward
        if keys[pygame.K_UP]:
            screen.blit(dpad_f, (0,0))
            msg.axes[1] += speed_tang

        # drive backwards
        if keys[pygame.K_DOWN]:
            screen.blit(dpad_b, (0,0))
            msg.axes[1] -= speed_tang

        

        # activate line-following aka autopilot
        if keys[pygame.K_a]:
            msg.buttons[7] = 1    

        # stop line-following
        if keys[pygame.K_s]:
            msg.buttons[6] = 1

        # toggle anti-instagram
        if keys[pygame.K_i]:
            msg.buttons[3] = 1

        ## key/action for quitting the program
            
        # check if top left [x] was hit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        # quit program
        if keys[pygame.K_q]:
            pygame.quit()

        ### END CHECKING KEYS ###
            
        # refresh screen
        pygame.display.flip()

        # check for any input commands
        stands = (sum(map(abs, msg.axes)) == 0 and sum(map(abs, msg.buttons)) == 0)
        if not stands:
            veh_standing = False
            
        # publish message
        if not veh_standing:
            pub_joystick.publish(msg)

        # adjust veh_standing such that when vehicle stands still, at least
        # one last publishment was sent to the bot. That's why this adjustment
        # is made after the publishment of the message
        if stands:
            veh_standing = True
            
        time.sleep(0.03)

        # obtain next key list
        pygame.event.pump()


# prepare size and rotations of dpad and dpad_pressed
def prepare_dpad():
    global dpad, dpad_f, dpad_r, dpad_b, dpad_l
    file_dir = os.path.dirname(__file__)
    file_dir = (file_dir + "/") if  (file_dir) else ""
    
    dpad = pygame.image.load(file_dir + "images/d-pad.png")
    dpad = pygame.transform.scale(dpad, (screen_size, screen_size))
    dpad_pressed = pygame.image.load(file_dir + "images/d-pad-pressed.png")
    dpad_pressed = pygame.transform.scale(dpad_pressed, (screen_size, screen_size))
    dpad_f = dpad_pressed
    dpad_r = pygame.transform.rotate(dpad_pressed, 270)
    dpad_b = pygame.transform.rotate(dpad_pressed, 180)
    dpad_l = pygame.transform.rotate(dpad_pressed, 90)

# Hint which is print at startup in console
def print_hint():
    print("\n\n\n")
    print("Virtual Joystick for your Duckiebot")
    print("-----------------------------------")
    print("\n")
    print("[ARROW_KEYS]:    Use them to steer your bot")
    print("         [q]:    Quit the program")
    print("         [a]:    Start line-following aka. autopilot")
    print("         [s]:    Stop line-following")
    print("         [i]:    Toggle anti-instagram")
    print("\n")
    print("Questions? Contact Julien Kindle: jkindle@ethz.ch")
    

    
if __name__ == '__main__':
    
    # obtain vehicle name
    veh_name = os.environ['VEHICLE_NAME']
    
    # prepare pygame
    pygame.init()
    screen = pygame.display.set_mode((screen_size,screen_size))
    prepare_dpad()
    
    # prepare ROS node
    rospy.init_node('virtual_joy',anonymous=False)
    
    # prepare ROS publisher
    pub_joystick = rospy.Publisher("/" + str(veh_name) + "/joy", Joy, queue_size=1)

    # print the hint
    print_hint()

    # start the main loop
    loop()
