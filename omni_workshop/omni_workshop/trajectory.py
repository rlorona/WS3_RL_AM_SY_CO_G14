"""Building blocks for the figure the robot follows.

A figure is a chain of moves. Everything is measured from where the robot starts:
x towards its initial front, y towards its initial left, in meters and degrees.

    figure = (Trajectory()
              .line(1.0, 0.0)              # 1 m forward
              .diagonal(0.8, 135)          # 0.8 m diagonally, backwards to the left
              .arc(0.5, 180)               # half turn to the left with a 0.5 m radius
              .turn(90)                    # rotate 90 degrees in place
              .line(0.0, -1.0, rotate=90)) # move sideways while rotating (only an omni can)

To try yours, add it to FIGURES and run:
    ros2 run omni_workshop self_controller --ros-args -p figure:=<name>
"""
import math

LINEAR_SPEED = 0.2    # default speed on lines and arcs [m/s]
TURN_SPEED = 0.5      # default speed on in-place turns [rad/s]


class Trajectory:

    def __init__(self, heading=0.0):
        self.segments = []                  # (duration, function(tau) -> relative state)
        self.x = self.y = self.theta = 0.0  # state at the end of the chain
        self.heading = math.radians(heading)  # initial direction of travel (for arcs), degrees

    # --- moves ---------------------------------------------------------------
    def line(self, dx, dy, v=LINEAR_SPEED, rotate=0.0):
        """Moves (dx, dy) meters in a straight line. With rotate, the robot turns
        that many degrees while moving."""
        length = math.hypot(dx, dy)
        if length == 0:
            return self
        T = length / v
        dth = math.radians(rotate)
        self._add(T, lambda tau: (dx * tau / T, dy * tau / T, dth * tau / T,
                                  dx / T, dy / T, dth / T))
        self.heading = math.atan2(dy, dx)
        return self

    def diagonal(self, distance, angle, v=LINEAR_SPEED, rotate=0.0):
        """Straight line of `distance` meters in the direction `angle` (degrees,
        0 = forward, 90 = left, 45 = forward-left...)."""
        a = math.radians(angle)
        return self.line(distance * math.cos(a), distance * math.sin(a), v, rotate)

    def arc(self, radius, angle, v=LINEAR_SPEED, rotate=0.0):
        """Curve of `angle` degrees (positive to the left, negative to the right)
        with the given radius, following the current direction of travel."""
        a = math.radians(angle)
        T = radius * abs(a) / v
        side = math.copysign(1, a)
        h0 = self.heading
        # Arc center, to the left (or right) of the direction of travel
        cx, cy = -side * radius * math.sin(h0), side * radius * math.cos(h0)
        dth = math.radians(rotate)

        def state(tau):
            phi = h0 + a * tau / T                  # direction of travel at tau
            w = a / T
            return (cx + side * radius * math.sin(phi), cy - side * radius * math.cos(phi),
                    dth * tau / T,
                    side * radius * w * math.cos(phi), side * radius * w * math.sin(phi),
                    dth / T)
        self._add(T, state)
        self.heading = h0 + a
        return self

    def turn(self, angle, w=TURN_SPEED):
        """Rotates `angle` degrees in place, without moving."""
        a = math.radians(angle)
        T = abs(a) / w
        self._add(T, lambda tau: (0.0, 0.0, a * tau / T, 0.0, 0.0, a / T))
        return self

    def pause(self, seconds):
        """Stays still."""
        self._add(seconds, lambda tau: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        return self

    # --- evaluation ----------------------------------------------------------
    @property
    def duration(self):
        return sum(T for T, _ in self.segments)

    def __call__(self, t):
        """Desired state at time t: (x, y, theta, dx, dy, dtheta)."""
        for T, state in self.segments:
            if t <= T:
                return state(t)
            t -= T
        return self.x, self.y, self.theta, 0.0, 0.0, 0.0

    def _add(self, T, relative):
        x0, y0, th0 = self.x, self.y, self.theta

        def state(tau):
            x, y, th, dx, dy, dth = relative(tau)
            return x0 + x, y0 + y, th0 + th, dx, dy, dth
        self.segments.append((T, state))
        self.x, self.y, self.theta = state(T)[:3]


# --- example figures (they fit in the 6 x 6 m arena starting at (0, -1.5)) ----
FIGURES = {
    'square': lambda: Trajectory().line(1.2, 0).line(0, 1.2).line(-1.2, 0).line(0, -1.2),
    'square_2m': lambda: Trajectory().line(2.0, 0).line(0, 2.0).line(-2.0, 0).line(0, -2.0),
    'square_rotating': lambda: (Trajectory().line(1.2, 0, rotate=90).line(0, 1.2, rotate=90)
                                .line(-1.2, 0, rotate=90).line(0, -1.2, rotate=90)),
    'rhombus': lambda: Trajectory().diagonal(1.2, 45).diagonal(1.2, 135).diagonal(1.2, -135).diagonal(1.2, -45),
    'circle': lambda: Trajectory().arc(0.8, 360),
    'figure_eight': lambda: Trajectory(heading=90).arc(0.6, 360).arc(0.6, -360),   # starts towards +y
    'demo': lambda: (Trajectory().line(1.0, 0).turn(90).arc(0.5, 180).diagonal(0.7, -45)
                     .line(0, -0.5, rotate=-90)),
    # Letters R and M, 1.6 m tall, one after the other; the robot always faces +x
    'RM': lambda: (Trajectory()
                   .line(0, 1.6).line(0.5, 0).arc(0.4, -180).line(-0.5, 0)   # R: stem and bowl
                   .line(0.9, -0.8)                                          # R: leg
                   .line(0.4, 0)                                             # to the M
                   .line(0, 1.6).line(0.5, -0.8).line(0.5, 0.8).line(0, -1.6)),  # M
}
