import ctre
import wpilib.simulation
from pyfrc.physics import tankmodel, motor_cfgs
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.units import units

from robot import StealthTigersRobot
from constants import DriveConstants
from util import Units


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: StealthTigersRobot):
        self.physics_controller = physics_controller

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(DriveConstants.FRONT_LEFT_MOTOR_PORT)
        self.r_motor = wpilib.simulation.PWMSim(DriveConstants.FRONT_RIGHT_MOTOR_PORT)

        # Encoders
        self.l_encoder = wpilib.simulation.EncoderSim.createForChannel(DriveConstants.LEFT_ENCODER_PORTS[0])
        self.r_encoder = wpilib.simulation.EncoderSim.createForChannel(DriveConstants.RIGHT_ENCODER_PORTS[0])

        # Gyro
        self.sim_imu = ctre.BasePigeonSimCollection(robot.container.drive.imu, False)

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
        r_motor = self.r_motor.getSpeed()

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        l_position = Units.feet_to_metres(self.drivetrain.l_position)
        r_position = Units.feet_to_metres(self.drivetrain.r_position)
        l_velocity = Units.feet_to_metres(self.drivetrain.l_velocity)
        r_velocity = Units.feet_to_metres(self.drivetrain.r_velocity)

        self.l_encoder.setDistance(l_position)
        self.r_encoder.setDistance(r_position)
        self.l_encoder.setRate(l_velocity)
        self.r_encoder.setRate(r_velocity)

        self.sim_imu.setRawHeading(pose.rotation().degrees())
