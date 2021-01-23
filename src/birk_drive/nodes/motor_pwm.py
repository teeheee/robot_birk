#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from dual_mc33926_rpi import motors, MAX_SPEED

#############################################################
#############################################################
class MotorPwm():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("motor_pwm")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        self.w = rospy.get_param("~base_width", 0.2)

#        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
#        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
        rospy.Subscriber('motor/left_pwm', Float32, self.left_callback)
        rospy.Subscriber('motor/right_pwm', Float32, self.right_callback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left_pwm = 0
        self.right_pwm = 0
        
        motors.motor1.enable()
        motors.motor2.enable()

        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)

    #############################################################
    def spin(self):
    #############################################################

        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        ###### main loop  ######
        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    #############################################################
    def spinOnce(self):
    #############################################################
        motors.motor1.setSpeed(self.left_pwm)
        motors.motor2.setSpeed(self.right_pwm)
        self.ticks_since_target += 1

    #############################################################
    def left_callback(self, msg):
    #############################################################
#       rospy.loginfo("left_callback")
        self.left_pwm = msg.data


    #############################################################
    def right_callback(self,msg):
    #############################################################
#        rospy.loginfo("right_callback")
        self.right_pwm = msg.data

#############################################################
#############################################################
def main():
    try:
        motorPwm = MotorPwm()
        motorPwm.spin()
    except rospy.ROSInterruptException:
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        motors.disable()
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    motors.disable()


if __name__ == '__main__':
    main()
