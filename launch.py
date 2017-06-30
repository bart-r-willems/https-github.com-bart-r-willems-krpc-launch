<<<<<<< HEAD
import krpc
import time
import math
from enum import Enum

# ----------------------------------------------------------------------------
# Launch parameters
# ----------------------------------------------------------------------------

ORBIT_ALT = 100000
GRAV_TURN_FINISH = 55000  #meters by which vessel should be at 0 pitch
MAX_AUTO_STAGE = 0  # last stage to separate automatically
MAX_Q = 50000
DEPLOY_SOLAR = True
FORCE_ROLL = True
ROLL = 90
INCLINATION = 0
G = 9.82 # standard G

REFRESH_FREQ = 5     # refresh rate in hz
TELEM_DELAY = .3     #number of seconds between telemetry updates
MAX_PHYSICS_WARP = 3 # valid values are 0 (none) through 3 (4x)


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------

def main():
    '''
    main function is run when you just execute this file, but NOT when you
    import it into another file - thus you can choose to call ascent() later
    to go to space, or just use the other functions in this file.
    '''
    # AscentController('Launch').to_orbit()
    mac = ModularAscentControl('Launch')
    # mac.register_controller('throttle', AnnoyingThrottle)
    mac.to_orbit()

# ----------------------------------------------------------------------------
# Class Definitions
# ----------------------------------------------------------------------------
class Status(Enum):
    IDLE = 0
    PRELAUNCH = 10
    LAUNCH = 20
    LIFTOFF = 30
    PITCH = 40
    COAST = 50
    CIRCULARIZE = 60
    FINALIZE = 70
    DONE = 80

class FuelTypes(Enum):
    LIQUIDFUEL = 'LiquidFuel'
    SOLIDFUEL = 'SolidFuel'

class Display(object):
    def __init__(self):
        self.last_update = time.time()

    def telemetry(self, t):
        '''
        Take a Telemetry object t and display it in a pleasing way
        '''
        # only update when last update is older than one second
        if time.time() < self.last_update + 1.0:
            return

        # define the data to be displayed in as many columns needed
        col1 = ('Apoapsis:       {apoapsis:8,.0f}',
                'Time to apo:       {time_to_apo:5,.0f}',
                'Altitude:         {altitude:6,.0f}',
                'Orbital velocity:  {velocity:5,.0f}',
                'Latitude:          {lat:5.1f}',
                'Dynamic Pressure: {q:6,.0f}')

        col2 = ('Periapsis:   {periapsis: 8,.0f}',
                'Time to peri:   {time_to_peri:5,.0f}',
                'Inclination:      {inclination: 3.0f}\n',
                'Vertical speed: {vertical_speed: 5,.0f}',
                'Longitude:      {lon:5.1f}\n',
                'G-force:         {g:4.1f}')
        # zip the columns together and display them
        print('-' * 60)
        for display_line in zip(col1, col2):
            print('     '.join(display_line).format(**t.__dict__))
        print('-' * 60)
        print('\n')
        self.last_update = time.time()

    def status(self, s):
        '''
        Print status message

        s: string
        '''
        print(s)

class Telemetry(object):
    def __init__(self, vessel, flight):
        self.apoapsis = vessel.orbit.apoapsis_altitude
        self.periapsis = vessel.orbit.periapsis_altitude
        self.time_to_apo = vessel.orbit.time_to_apoapsis
        self.time_to_peri = vessel.orbit.time_to_periapsis
        self.velocity = vessel.orbit.speed
        self.inclination = math.degrees(vessel.orbit.inclination)
        self.altitude = flight.mean_altitude
        self.vertical_speed = flight.vertical_speed
        self.lat = flight.latitude
        self.lon = flight.longitude
        self.q = flight.dynamic_pressure
        self.g = flight.g_force

class ModularAscentControl(object):
    '''
    Framework for a controlled ascent with modular components
    '''
    def __init__(self, name):
        # set process variables
        self.conn = krpc.connect(name=name)
        self.sc = self.conn.space_center
        self.vessel = self.sc.active_vessel
        self.flight = self.vessel.flight(self.vessel.orbit.body.reference_frame)
        self.status = Status.IDLE
        # initialize controllers
        self.controllers = ('guidance', 'throttle', 'staging',
                            'warp', 'finalize')
        self.guidance = GuidanceController(self.sc, self.vessel, self.flight)
        self.throttle = ThrottleController(self.sc, self.vessel, self.flight)
        self.staging = StagingController(self.sc, self.vessel, self.flight)
        self.warp = WarpController(self.sc, self.vessel, self.flight)
        self.finalize = FinalizeController(self.sc, self.vessel, self.flight)
        # create a hook to the display functions
        # a simple print-to-console class if nothing else if provided
        self.D = Display()
        self.display_telemetry = self.D.telemetry
        self.display_status = self.D.status

    def register_controller(self, controller_name, controller_class):
        '''
        Allow the default controller to be replaced with an alternate
        at runtime.
        Example: mac.register_controller('throttle', FancyThrottle) --
        This will replace the default Throttle Controller with on based on a
        class called "FancyThrottle."
        Controller classes must be subclassed from the Controller base class,

        controller_name: string, must be in ModularAscentControl.controllers
        controller_class: Controller type class
        '''
        # rudimentary checking to prevent runtime problems
        if controller_name not in self.controllers:
            raise KeyError('Invalid controller name - must be in controllers')
        if not issubclass(controller_class, Controller):
            raise TypeError('Controller Class needed')
        new_controller = controller_class(self.sc, self.vessel, self.flight)
        setattr(self, controller_name, new_controller)

    def to_orbit(self):
        self.set_status(Status.PRELAUNCH)
        while self.status != Status.DONE:
            self.telemetry()
            self.guidance.process()
            self.throttle.process()
            self.staging.process()
            self.warp.process()
            self.finalize.process()
            self.update_status()
            time.sleep(1.0 / REFRESH_FREQ)

    def telemetry(self):
        '''
        provide a telemtry update to the outside world
        '''
        self.display_telemetry(Telemetry(self.vessel, self.flight))

    def set_status(self, status):
        '''
        set the status of the launch process
        propagate status throughout various controllers

        status: Status type enum
        '''
        self.display_status(status.name)
        self.status = status
        self.guidance.set_status(status)
        self.throttle.set_status(status)
        self.staging.set_status(status)
        self.warp.set_status(status)
        self.finalize.set_status(status)

    def update_status(self):
        '''
        based on the flight profile, update the status,
        activing different modes in the controllers

        status: Status type enum
        '''

        # perhaps these should be split out to separate functions?
        # make sure to run them in reverse order, from FINALIZE to
        # PRELAUNCH, else they can trigger each other...
        if self.status == Status.PRELAUNCH:
            # at this point, all controllers have gone through their
            # precheck launch; advance to launch!
            self.set_status(Status.LAUNCH)
        elif self.status == Status.LAUNCH:
            # at this point, all controllers have gone through launch;
            # advance to LIFTOFF
            self.set_status(Status.LIFTOFF)
        elif self.status == Status.LIFTOFF:
            # before advancing to pitch manuever, ensure that
            # minimum height and velocity are achieved
            frame = self.vessel.surface_velocity_reference_frame
            speed = magnitude(self.vessel.velocity(frame))
            min_height = self.flight.mean_altitude > 100
            min_velocity = self.vessel.orbit.speed > 50
            if min_height and min_velocity:
                self.set_status(Status.PITCH)
        elif self.status == Status.PITCH:
            # cut to coasting once apoapsis is achieved within
            # desirable limits. The guidance module will set a node
            # to circularize once coasting mode is entered
            if self.vessel.orbit.apoapsis_altitude > ORBIT_ALT * 0.95:
                self.set_status(Status.COAST)
        elif self.status == Status.COAST:
            # keep coasting until it's time to burn
            node = self.vessel.control.nodes[0]
            if node.time_to <= self.calc_burn_time():
                self.set_status(Status.CIRCULARIZE)
        elif self.status == Status.CIRCULARIZE:
            # circularize will stop once periapsis
            # is close to target apoapsis_altitude
            # or when apoapsis starts to balloon
            min_peri = self.vessel.orbit.periapsis_altitude > ORBIT_ALT * 0.90
            max_apo = self.vessel.orbit.apoapsis_altitude > ORBIT_ALT * 1.10
            if min_peri or max_apo:
                self.set_status(Status.FINALIZE)
        elif self.status == Status.FINALIZE:
            # all controllers have cycled through their
            # finalize task
            self.set_status(Status.DONE)

    def calc_burn_time(self):
        '''
        Return burn time required to execute next node
        '''
        node = self.vessel.control.nodes[0]
        m = self.vessel.mass
        isp = self.vessel.specific_impulse
        dv = node.delta_v
        F = self.vessel.available_thrust
        # calculate burn time based on rocket equation
        return (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

class Controller(object):
    '''
    Baseclass for the various controllers that are utilized during a launch
    '''
    def __init__(self, spacecenter, vessel, flight):
        self.spacecenter = spacecenter
        self.vessel = vessel
        self.flight = flight
        self.status = Status.IDLE

    def process(self):
        '''
        The process class is where all the controller action happens
        The controller will perform actions, based on the readouts of
        the vessel, and the status the controller is in
        '''
        pass

    def set_status(self, status):
        '''
        Set the status through a function, so the controller is aware
        of status changes.

        status: Status type enum
        '''
        self.status = status

class GuidanceController(Controller):
    def process(self):
        '''
        Set direction during ascent
        '''
        if self.status == Status.PRELAUNCH:
            self.prelaunch()
        if self.status == Status.PITCH:
            self.pitch()
        if self.status == Status.COAST:
            self.prograde()
        if self.status == Status.CIRCULARIZE:
            try:
                self.target_node()
            except:
                self.prograde()
        if self.status == Status.FINALIZE:
            self.prograde()


    def prelaunch(self):
        '''
        Set up autopilot for initial flight headings
        '''
        self.stabilize()

    def pitch(self):
        '''
        Perform the pitch manuever (sometimes referred to as "gravity turn")
        Based on Robet Penner's easing equations (EaseOut)
        '''
        progress = self.flight.mean_altitude / GRAV_TURN_FINISH
        target_pitch = 90 - (-90 * progress * (progress - 2))
        target_heading = self.inc_to_heading(INCLINATION)

        # print('Heading: {:3.0f} Pitch: {:3.0f}'.format(target_heading, target_pitch))
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_pitch = target_pitch
        self.vessel.auto_pilot.target_heading = target_heading
        if FORCE_ROLL:
            self.vessel.auto_pilot.target_roll = ROLL

    def set_sas_mode(self, mode):
        '''
        Turn off autopilot mode if it's not in sas mode
        Turn on autopilot in sas mode
        Set autopilot to desired mode if it's not already
        '''
        if not self.vessel.auto_pilot.sas:
            self.vessel.auto_pilot.disengage()
            self.vessel.auto_pilot.sas = True
            time.sleep(0.1)
        if self.vessel.auto_pilot.sas_mode != mode:
            self.vessel.auto_pilot.sas_mode = mode

    def target_node(self):
        '''
        Set autopilot to node
        '''
        self.set_sas_mode(self.vessel.auto_pilot.sas_mode.manuever)

    def prograde(self):
        '''
        Set autopilot to prograde
        '''
        self.set_sas_mode(self.vessel.auto_pilot.sas_mode.prograde)

    def stabilize(self):
        '''
        Set autopilot to stablize
        '''
        self.set_sas_mode(self.vessel.auto_pilot.sas_mode.stability_assist)

    def inc_to_heading(self, inc):
        '''
        Converts desired inclination to a compass heading that can be
        tracked by the autopilot

        inc: inclination in degrees
        '''
        if inc > 180 or inc < -180:
            return 90   #invalid entries get set to 0 inclination
        if inc >= 0:
            value = 90 - inc
        if inc < 0:
            value = -(inc - 90)
        if value < 0:
            value += 360
        return value

    def set_status(self, status):
        '''
        Overriding the set_status function of the controller base class
        if status set to COAST, create node for circularization

        status: Status type enum
        '''
        self.status = status
        if status == Status.COAST:
            self.create_circularize_node()

    def create_circularize_node(self):
        '''
        Create a node for circularization
        '''
        grav_param = self.vessel.orbit.body.gravitational_parameter
        apo = self.vessel.orbit.apoapsis
        sma = self.vessel.orbit.semi_major_axis
        v1 = math.sqrt(grav_param * ((2.0 / apo) - (1.0 / sma)))
        v2 = math.sqrt(grav_param * ((2.0 / apo) - (1.0 / apo)))
        v = v2 - v1
        t  = self.spacecenter.ut + self.vessel.orbit.time_to_apoapsis
        self.vessel.control.add_node(t, prograde=v)

class ThrottleController(Controller):
    def process(self):
        '''
        Set throttle according to various stages of launch process
        '''
        if self.status == Status.PRELAUNCH:
            self.vessel.control.throttle = 1.0
        elif self.status in (Status.COAST, Status.FINALIZE, Status.DONE):
            self.vessel.control.throttle = 0.0
        elif self.status == Status.CIRCULARIZE:
            bt = self.calc_burn_time()
            node = self.vessel.control.nodes[0]
            # print(spacecenter.ut, bt, node.ut)
            burn = self.spacecenter.ut + bt/2 >= node.ut
            if burn:
                self.vessel.control.throttle = 1.0
            else:
                self.vessel.control.throttle = 0.0

    def calc_burn_time(self):
        '''
        Return burn time required to execute next node
        '''
        node = self.vessel.control.nodes[0]
        m = self.vessel.mass
        isp = self.vessel.specific_impulse
        dv = node.delta_v
        F = self.vessel.available_thrust
        # calculate burn time based on rocket equation
        return (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))


class AnnoyingThrottle(Controller):
    def process(self):
        print('REPORTING FOR DUTY, SIR!!')
        if self.status == Status.PRELAUNCH:
            self.vessel.control.throttle = 1.0
        elif self.status in (Status.COAST, Status.FINALIZE, Status.DONE):
            self.vessel.control.throttle = 0.0
        elif self.status == Status.CIRCULARIZE:
            self.vessel.control.throttle = 1.0


class StagingController(Controller):
    def process(self):
        '''
        check if a stage should be activated
        '''
        # out of stages?
        if self.vessel.control.current_stage <= MAX_AUTO_STAGE:
            return

        interstage = True
        for fueltype in FuelTypes:
            if self.carries_fuel(fueltype):
                interstage = False
                if not self.has_fuel(fueltype):
                    self.vessel.control.activate_next_stage()
                    return
        if interstage:
            self.vessel.control.activate_next_stage()

    def resource(self):
        '''
        return the resources in the decouple stage
        '''
        return self.vessel.resources_in_decouple_stage(
            self.vessel.control.current_stage - 1,
            cumulative=False)

    def carries_fuel(self, fueltype):
        '''
        return True if a fuel capacity for the fueltype is available
        in the current stage

        fueltype: FuelType enum
        '''
        return self.resource().max(fueltype.value) > 0

    def has_fuel(self, fueltype):
        '''
        return True if fuel of the fueltype is actually available in
        the current stage

        fueltype: FuelType enum
        '''
        return self.resource().amount(fueltype.value) > 0

class WarpController(Controller):
    def process(self):
        sc = self.spacecenter
        if self.status == Status.COAST and self.in_atmo():
            # no reason to not max warp while coastin in atmosphere
            sc.physics_warp_factor = MAX_PHYSICS_WARP
        elif self.status == Status.COAST and sc.physics_warp_factor > 0:
            # if not in atmosphere, check and still on physics warp:
            # disable physics warp and warp to node
            sc.physics_warp_factor = 0
            bt = self.calc_burn_time()
            node=self.vessel.control.nodes[0]
            sc.warp_to(node.ut - (bt / 2.0) - 5.0)

    def calc_burn_time(self):
        '''
        Return burn time required to execute next node
        '''
        node = self.vessel.control.nodes[0]
        m = self.vessel.mass
        isp = self.vessel.specific_impulse
        dv = node.delta_v
        F = self.vessel.available_thrust
        # calculate burn time based on rocket equation
        return (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

    def in_atmo(self):
        '''
        Return True if vessel still in atmosphere
        '''
        alt = self.flight.mean_altitude
        atmo = self.vessel.orbit.body.atmosphere_depth
        return alt <= atmo

class FinalizeController(Controller):
    def process(self):
        if self.status == Status.FINALIZE:
            self.vessel.control.remove_nodes()

# ----------------------------------------------------------------------------
# Generic helper functions
# ----------------------------------------------------------------------------
def magnitude(vector):
    '''
    Return the magnitude (length) of a vector
    '''
    return math.sqrt(sum(x * x for x in vector))

# ----------------------------------------------------------------------------
# Activate main loop, assuming we are executing THIS file explicitly.
# ----------------------------------------------------------------------------
if __name__ == "__main__" :
    main()
=======
import krpc
import time
import math
from enum import Enum

# ----------------------------------------------------------------------------
# Launch parameters
# ----------------------------------------------------------------------------

ORBIT_ALT = 95000
GRAV_TURN_FINISH = 50000  #meters by which vessel should be at 0 pitch
MAX_AUTO_STAGE = 0  # last stage to separate automatically
MAX_Q = 50000
DEPLOY_SOLAR = True
FORCE_ROLL = True
ROLL = 90
INCLINATION = 5
G = 9.82 # standard G

REFRESH_FREQ = 2    # refresh rate in hz
TELEM_DELAY = 5     #number of seconds between telemetry updates
ALL_FUELS = ('LiquidFuel', 'SolidFuel')
MAX_PHYSICS_WARP = 3 # valid values are 0 (none) through 3 (4x)


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------

def main():
    '''
    main function is run when you just execute this file, but NOT when you
    import it into another file - thus you can choose to call ascent() later
    to go to space, or just use the other functions in this file.
    '''
    # AscentController('Launch').to_orbit()
    mac = ModularAscentControl('Launch')
    mac.register_controller('throttle', AnnoyingThrottle)
    mac.to_orbit()

# ----------------------------------------------------------------------------
# Class Definitions
# ----------------------------------------------------------------------------
class Status(Enum):
    IDLE = 0
    PRELAUNCH = 10
    LAUNCH = 20
    LIFTOFF = 30
    PITCH = 40
    COAST = 50
    CIRCULARIZE = 60
    FINALIZE = 70
    DONE = 80

class FuelTypes(Enum):
    LIQUIDFUEL = 'LiquidFuel'
    SOLIDFUEL = 'SolidFuel'

class Display(object):
    def __init__(self):
        self.last_update = time.time()

    def telemetry(self, t):
        '''
        Take a Telemetry object t and display it in a pleasing way
        '''
        # only update when last update is older than one second
        if time.time() < self.last_update + 1.0:
            return

        # define the data to be displayed in as many columns needed
        col1 = ('Apoapsis:       {apoapsis:8,.0f}',
                'Time to apo:       {time_to_apo:5,.0f}',
                'Altitude:         {altitude:6,.0f}',
                'Orbital velocity:  {velocity:5,.0f}',
                'Latitude:          {lat:5.1f}',
                'Dynamic Pressure: {q:6,.0f}')

        col2 = ('Periapsis:   {periapsis: 8,.0f}',
                'Time to peri:   {time_to_peri:5,.0f}',
                'Inclination:      {inclination: 3.0f}\n',
                'Vertical speed: {vertical_speed: 5,.0f}',
                'Longitude:      {lon:5.1f}\n',
                'G-force:         {g:4.1f}')
        # zip the columns together and display them
        print('-' * 60)
        for display_line in zip(col1, col2):
            print('     '.join(display_line).format(**t.__dict__))
        print('-' * 60)
        print('\n')
        self.last_update = time.time()

    def status(self, s):
        '''
        Print status message

        s: string
        '''
        print(s)

class Telemetry(object):
    def __init__(self, vessel, flight):
        self.apoapsis = vessel.orbit.apoapsis_altitude
        self.periapsis = vessel.orbit.periapsis_altitude
        self.time_to_apo = vessel.orbit.time_to_apoapsis
        self.time_to_peri = vessel.orbit.time_to_periapsis
        self.velocity = vessel.orbit.speed
        self.inclination = math.radians(vessel.orbit.inclination)
        self.altitude = flight.mean_altitude
        self.vertical_speed = flight.vertical_speed
        self.lat = flight.latitude
        self.lon = flight.longitude
        self.q = flight.dynamic_pressure
        self.g = flight.g_force

class ModularAscentControl(object):
    '''
    Framework for a controlled ascent with modular components
    '''
    def __init__(self, name):
        # set process variables
        self.conn = krpc.connect(name=name)
        self.sc = self.conn.space_center
        self.vessel = self.sc.active_vessel
        self.flight = self.vessel.flight(self.vessel.orbit.body.reference_frame)
        self.status = Status.IDLE
        # initialize controllers
        self.controllers = ('guidance', 'throttle', 'staging',
                            'warp', 'finalize')
        self.guidance = GuidanceController(self.sc, self.vessel, self.flight)
        self.throttle = ThrottleController(self.sc, self.vessel, self.flight)
        self.staging = StagingController(self.sc, self.vessel, self.flight)
        self.warp = WarpController(self.sc, self.vessel, self.flight)
        self.finalize = FinalizeController(self.sc, self.vessel, self.flight)
        # create a hook to the display functions
        # a simple print-to-console class if nothing else if provided
        self.D = Display()
        self.display_telemetry = self.D.telemetry
        self.display_status = self.D.status

    def register_controller(self, controller_name, controller_class):
        '''
        Allow the default controller to be replaced with an alternate
        at runtime.
        Example: mac.register_controller('throttle', FancyThrottle) --
        This will replace the default Throttle Controller with on based on a
        class called "FancyThrottle."
        Controller classes must be subclassed from the Controller base class,

        controller_name: string, must be in ModularAscentControl.controllers
        controller_class: Controller type class
        '''
        # rudimentary checking to prevent runtime problems
        if controller_name not in self.controllers:
            raise KeyError('Invalid controller name - must be in controllers')
        if not issubclass(controller_class, Controller):
            raise TypeError('Controller Class needed')
        new_controller = controller_class(self.sc, self.vessel, self.flight)
        setattr(self, controller_name, new_controller)

    def to_orbit(self):
        self.set_status(Status.PRELAUNCH)
        while self.status != Status.DONE:
            self.telemetry()
            self.guidance.process()
            self.throttle.process()
            self.staging.process()
            self.warp.process()
            self.finalize.process()
            self.update_status()
            time.sleep(1.0 / REFRESH_FREQ)

    def telemetry(self):
        '''
        provide a telemtry update to the outside world
        '''
        self.display_telemetry(Telemetry(self.vessel, self.flight))

    def set_status(self, status):
        '''
        set the status of the launch process
        propagate status throughout various controllers

        status: Status type enum
        '''
        self.display_status('Progressing to ' + status.name)
        self.status = status
        self.guidance.set_status(status)
        self.throttle.set_status(status)
        self.staging.set_status(status)
        self.warp.set_status(status)
        self.finalize.set_status(status)

    def update_status(self):
        '''
        based on the flight profile, update the status,
        activing different modes in the controllers

        status: Status type enum
        '''

        # perhaps these should be split out to separate functions?
        # make sure to run them in reverse order, from FINALIZE to
        # PRELAUNCH, else they can trigger each other...
        if self.status == Status.PRELAUNCH:
            # at this point, all controllers have gone through their
            # precheck launch; advance to launch!
            self.set_status(Status.LAUNCH)
        elif self.status == Status.LAUNCH:
            # at this point, all controllers have gone through launch;
            # advance to LIFTOFF
            self.set_status(Status.LIFTOFF)
        elif self.status == Status.LIFTOFF:
            # before advancing to pitch manuever, ensure that
            # minimum height and velocity are achieved
            frame = self.vessel.surface_velocity_reference_frame
            speed = magnitude(self.vessel.velocity(frame))
            min_height = self.flight.mean_altitude > 100
            min_velocity = self.vessel.orbit.speed > 50
            if min_height and min_velocity:
                self.set_status(Status.PITCH)
        elif self.status == Status.PITCH:
            # cut to coasting once apoapsis is achieved within
            # desirable limits. The guidance module will set a node
            # to circularize once coasting mode is entered
            if self.vessel.orbit.apoapsis_altitude > ORBIT_ALT * 0.95:
                self.set_status(Status.COAST)
        elif self.status == Status.COAST:
            # keep coasting until it's time to burn
            node = self.vessel.control.nodes[0]
            if node.time_to <= self.calc_burn_time():
                self.set_status(Status.CIRCULARIZE)
        elif self.status == Status.CIRCULARIZE:
            # circularize will stop once periapsis
            # is close to target apoapsis_altitude
            min_peri = self.vessel.orbit.periapsis_altitude > ORBIT_ALT * 0.95
            if min_peri:
                self.set_status(Status.FINALIZE)
        elif self.status == Status.FINALIZE:
            # all controllers have cycled through their
            # finalize task
            self.set_status(Status.DONE)

    def calc_burn_time(self):
        '''
        Return burn time required to execute next node
        '''
        node = self.vessel.control.nodes[0]
        m = self.vessel.mass
        isp = self.vessel.specific_impulse
        dv = node.delta_v
        F = self.vessel.available_thrust
        # calculate burn time based on rocket equation
        return (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

class Controller(object):
    '''
    Baseclass for the various controllers that are utilized during a launch
    '''
    def __init__(self, spacecenter, vessel, flight):
        self.spacecenter = spacecenter
        self.vessel = vessel
        self.flight = flight
        self.status = Status.IDLE

    def process(self):
        '''
        The process class is where all the controller action happens
        The controller will perform actions, based on the readouts of
        the vessel, and the status the controller is in
        '''
        pass

    def set_status(self, status):
        '''
        Set the status through a function, so the controller is aware
        of status changes.

        status: Status type enum
        '''
        self.status = status

class GuidanceController(Controller):
    def process(self):
        '''
        Set direction during ascent
        '''
        if self.status == Status.PRELAUNCH:
            self.prelaunch()
        if self.status == Status.PITCH:
            self.pitch()
        if self.status == Status.COAST:
            self.prograde()
        if self.status == Status.CIRCULARIZE:
            try:
                self.target_node()
            except:
                self.prograde()
        if self.status == Status.FINALIZE:
            self.prograde()


    def prelaunch(self):
        '''
        Set up autopilot for initial flight headings
        '''
        self.stabilize()

    def pitch(self):
        '''
        Perform the pitch manuever (sometimes referred to as "gravity turn")
        Based on Robet Penner's easing equations (EaseOut)
        '''
        progress = self.flight.mean_altitude / GRAV_TURN_FINISH
        target_pitch = 90 - (-90 * progress * (progress - 2))
        target_heading = self.inc_to_heading(INCLINATION)

        # print('Heading: {:3.0f} Pitch: {:3.0f}'.format(target_heading, target_pitch))
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_pitch = target_pitch
        self.vessel.auto_pilot.target_heading = target_heading
        if FORCE_ROLL:
            self.vessel.auto_pilot.target_roll = ROLL

    def set_sas_mode(self, mode):
        '''
        Turn off autopilot mode if it's not in sas mode
        Turn on autopilot in sas mode
        Set autopilot to desired mode if it's not already
        '''
        if not self.vessel.auto_pilot.sas:
            self.vessel.auto_pilot.disengage()
            self.vessel.auto_pilot.sas = True
            time.sleep(0.1)
        if self.vessel.auto_pilot.sas_mode != mode:
            self.vessel.auto_pilot.sas_mode = mode

    def target_node(self):
        '''
        Set autopilot to node
        '''
        self.set_sas_mode(self.vessel.auto_pilot.sas_mode.manuever)

    def prograde(self):
        '''
        Set autopilot to prograde
        '''
        self.set_sas_mode(self.vessel.auto_pilot.sas_mode.prograde)

    def stabilize(self):
        '''
        Set autopilot to stablize
        '''
        self.set_sas_mode(self.vessel.auto_pilot.sas_mode.stability_assist)

    def inc_to_heading(self, inc):
        '''
        Converts desired inclination to a compass heading that can be
        tracked by the autopilot

        inc: inclination in degrees
        '''
        if inc > 180 or inc < -180:
            return 90   #invalid entries get set to 0 inclination
        if inc >= 0:
            value = 90 - inc
        if inc < 0:
            value = -(inc - 90)
        if value < 0:
            value += 360
        return value

    def set_status(self, status):
        '''
        Overriding the set_status function of the controller base class
        if status set to COAST, create node for circularization

        status: Status type enum
        '''
        self.status = status
        if status == Status.COAST:
            self.create_circularize_node()

    def create_circularize_node(self):
        '''
        Create a node for circularization
        '''
        grav_param = self.vessel.orbit.body.gravitational_parameter
        apo = self.vessel.orbit.apoapsis
        sma = self.vessel.orbit.semi_major_axis
        v1 = math.sqrt(grav_param * ((2.0 / apo) - (1.0 / sma)))
        v2 = math.sqrt(grav_param * ((2.0 / apo) - (1.0 / apo)))
        v = v2 - v1
        t  = self.spacecenter.ut + self.vessel.orbit.time_to_apoapsis
        self.vessel.control.add_node(t, prograde=v)

class ThrottleController(Controller):
    def process(self):
        '''
        Set throttle according to various stages of launch process
        '''
        if self.status == Status.PRELAUNCH:
            self.vessel.control.throttle = 1.0
        elif self.status in (Status.COAST, Status.FINALIZE, Status.DONE):
            self.vessel.control.throttle = 0.0
        elif self.status == Status.CIRCULARIZE:
            self.vessel.control.throttle = 1.0

class AnnoyingThrottle(Controller):
    def process(self):
        print('REPORTING FOR DUTY, SIR!!')
        if self.status == Status.PRELAUNCH:
            self.vessel.control.throttle = 1.0
        elif self.status in (Status.COAST, Status.FINALIZE, Status.DONE):
            self.vessel.control.throttle = 0.0
        elif self.status == Status.CIRCULARIZE:
            self.vessel.control.throttle = 1.0


class StagingController(Controller):
    def process(self):
        '''
        check if a stage should be activated
        '''
        # out of stages?
        if self.vessel.control.current_stage <= MAX_AUTO_STAGE:
            return

        interstage = True
        for fueltype in FuelTypes:
            if self.carries_fuel(fueltype):
                interstage = False
                if not self.has_fuel(fueltype):
                    self.vessel.control.activate_next_stage()
                    return
        if interstage:
            self.vessel.control.activate_next_stage()

    def resource(self):
        '''
        return the resources in the decouple stage
        '''
        return self.vessel.resources_in_decouple_stage(
            self.vessel.control.current_stage - 1,
            cumulative=False)

    def carries_fuel(self, fueltype):
        '''
        return True if a fuel capacity for the fueltype is available
        in the current stage

        fueltype: FuelType enum
        '''
        return self.resource().max(fueltype.value) > 0

    def has_fuel(self, fueltype):
        '''
        return True if fuel of the fueltype is actually available in
        the current stage

        fueltype: FuelType enum
        '''
        return self.resource().amount(fueltype.value) > 0

class WarpController(Controller):
    def process(self):
        sc = self.spacecenter
        if self.status == Status.COAST and self.in_atmo():
            # no reason to not max warp while coastin in atmosphere
            sc.physics_warp_factor = MAX_PHYSICS_WARP
        elif self.status == Status.COAST and sc.physics_warp_factor > 0:
            # if not in atmosphere, check and still on physics warp:
            # disable physics warp and warp to node
            sc.physics_warp_factor = 0
            bt = self.calc_burn_time()
            node=self.vessel.control.nodes[0]
            sc.warp_to(node.ut - (bt / 2.0) - 5.0)

    def calc_burn_time(self):
        '''
        Return burn time required to execute next node
        '''
        node = self.vessel.control.nodes[0]
        m = self.vessel.mass
        isp = self.vessel.specific_impulse
        dv = node.delta_v
        F = self.vessel.available_thrust
        # calculate burn time based on rocket equation
        return (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

    def in_atmo(self):
        '''
        Return True if vessel still in atmosphere
        '''
        alt = self.flight.mean_altitude
        atmo = self.vessel.orbit.body.atmosphere_depth
        return alt <= atmo

class FinalizeController(Controller):
    pass

# ----------------------------------------------------------------------------
# Generic helper functions
# ----------------------------------------------------------------------------
def magnitude(vector):
    '''
    Return the magnitude (length) of a vector
    '''
    return math.sqrt(sum(x * x for x in vector))

# ----------------------------------------------------------------------------
# Activate main loop, assuming we are executing THIS file explicitly.
# ----------------------------------------------------------------------------
if __name__ == "__main__" :
    main()
>>>>>>> 23f0309860910d140bbdb417305801088461f27b
