from lsst.ts import salobj
import pathlib


class RotatorCSC(salobj.ConfigurableCsc):

    def __init__(self, config_dir=None, initial_state=salobj.State.STANDBY, initial_simulation_mode=0):
        schema_path = pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "rotator.yaml")
        super().__init__("Rotator", index=0, schema_path=schema_path, config_dir=config_dir,
                         initial_state=initial_state, initial_simulation_mode=initial_simulation_mode)
        self.config = None

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        self.config = config

    async def do_configureAcceleration(self, data):
        """Execute the configureAcceleration command."""
        raise NotImplementedError()

    async def do_configureVelocity(self, data):
        """Execute the configureVelocity command"""
        raise NotImplementedError()

    async def do_move(self, data):
        """Execute the move command."""
        raise NotImplementedError()

    async def do_track(self, data):
        """Execute the track command."""
        raise NotImplementedError()

    async def do_test(self, data):
        """Execute the test command."""
        raise NotImplementedError()

    async def do_trackStart(self, data):
        """Execute the trackStart command."""
        raise NotImplementedError()

    async def do_clearError(self, data):
        """Execute the clearError command."""
        raise NotImplementedError()

    async def do_positionSet(self, data):
        """Execute the positionSet command."""
        raise NotImplementedError()

    async def do_moveConstantVelocity(self, data):
        """Execute the moveConstantVelocity command."""
        raise NotImplementedError()

    async def do_velocitySet(self, data):
        """Execute the velocitySet command."""
        raise NotImplementedError()
