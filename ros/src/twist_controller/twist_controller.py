
import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        max_abs_angle = kwargs.get('max_steer_angle')        
        self.fuel_capacity = kwargs.get('fuel_capacity')
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.wheel_radius = kwargs.get('wheel_radius')
        self.accel_limit = kwargs.get('accel_limit')
        self.decel_limit = kwargs.get('decel_limit')
        self.brake_deadband = kwargs.get('brake_deadband')
        self.max_acceleration = 1.5
        self.wheel_base = kwargs.get('wheel_base')
        self.steer_ratio = kwargs.get('steer_ratio')
        self.max_steer_angle = kwargs.get('max_steer_angle')
        self.max_lat_accel = kwargs.get('max_lat_accel')
        
        kp = 0.3
        kd = 0.01
        ki = 0.1
        mn = 0
        mx = 0.2
        self.throttle_controller = PID(kp, kd, ki, mn, mx)
        self.yaw_controller = YawController(
            self.wheel_base, self.steer_ratio, 0.1,
            self.max_lat_accel, self.max_steer_angle) 
        
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        #self.brake_lpf = LowPassFilter(0.5, 0.02)


        total_vehicle_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY # 1.774,933
        # max torque (1.0 throttle) and  max brake torque (deceleration lmt)
        self.max_acc_torque = total_vehicle_mass * self.max_acceleration * self.wheel_radius #567
        self.max_brake_torque = total_vehicle_mass * abs(self.decel_limit) * self.wheel_radius #2095
        rospy.loginfo("-------------- initializing Controller  ..max brake torque {}".format(self.max_brake_torque) )
        self.last_time = rospy.get_time()
        
      

    def control(self, desired_linear_vel, desired_angular_vel, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        rospy.loginfo("control desired velocity {}".format(desired_linear_vel ))
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        # Return throttle, brake, steer
        current_linear_vel= self.vel_lpf.filt(current_linear_vel)

        steering = self.yaw_controller.get_steering(desired_linear_vel,desired_angular_vel, current_linear_vel)
        
        velocity_error = desired_linear_vel - current_linear_vel
        velocity_error = max(self.decel_limit,velocity_error)
        velocity_error = min(velocity_error,self.accel_limit)

        self.last_vel = current_linear_vel
        
        # find the time duration and a new timestamp
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(velocity_error, sample_time)
        
        brake = 0
        
        if desired_linear_vel == 0.0 and current_linear_vel < 0.1:
            throttle = 0.0
            brake = 700.0  # N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif throttle < 0.1 and velocity_error < 0:
            throttle = 0.0
            decel = max(velocity_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Torque N*m

        #brake = self.brake_lpf.filt(brake)
        rospy.loginfo("control  throttle {} brake {} steering {}".format(throttle, brake, steering ))

        return throttle, brake, steering
