import wpilib.simulation
from pyfrc.physics import tankmodel, motor_cfgs
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.units import units

from constants import DriveConstants


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface):
        self.physics_controller = physics_controller

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(DriveConstants.FRONT_LEFT_MOTOR_PORT)
        self.r_motor = wpilib.simulation.PWMSim(DriveConstants.FRONT_RIGHT_MOTOR_PORT)

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
        self.physics_controller.move_robot(transform)
