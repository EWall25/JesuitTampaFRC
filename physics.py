import ctre
import wpilib.simulation
from pyfrc.physics import tankmodel, motor_cfgs
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.units import units

from robot import StealthTigersRobot
from constants import DriveConstants, ArmConstants
from util import Units


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: StealthTigersRobot):
        self.physics_controller = physics_controller
        self.robot = robot

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(DriveConstants.FRONT_LEFT_MOTOR_PORT)
        self.r_motor = wpilib.simulation.PWMSim(DriveConstants.FRONT_RIGHT_MOTOR_PORT)

        # Encoders
        self.l_encoder = ctre.CANCoderSimCollection(robot.container.drive.l_encoder)
        self.r_encoder = ctre.CANCoderSimCollection(robot.container.drive.r_encoder)

        # Gyro
        self.sim_imu = ctre.BasePigeonSimCollection(robot.container.drive.imu, False)

        # Limit switches
        self.upper_limit = wpilib.simulation.DIOSim(ArmConstants.UPPER_LIMIT_SWITCH_PORT)
        self.upper_limit.setValue(False)

        # Drivetrain (arbitrary values used)
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,
            robot_mass=90 * units.lbs,
            gearing=10.71,
            nmotors=2,
            x_wheelbase=21.865 * units.inch,
            wheel_diameter=6 * units.inch
        )

    def update_sim(self, now, tm_diff):
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed() * -1

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        l_position = self.drivetrain.l_position
        r_position = self.drivetrain.r_position
        l_velocity = int(Units.feet_to_metres(self.drivetrain.l_velocity) / DriveConstants.ENCODER_DISTANCE_PER_PULSE)
        r_velocity = int(Units.feet_to_metres(self.drivetrain.r_velocity) / DriveConstants.ENCODER_DISTANCE_PER_PULSE)

        self.robot.container.drive.set_left_distance(l_position)
        self.robot.container.drive.set_right_distance(r_position)
        self.l_encoder.setVelocity(l_velocity)
        self.r_encoder.setVelocity(r_velocity)

        self.sim_imu.setRawHeading(pose.rotation().degrees())
