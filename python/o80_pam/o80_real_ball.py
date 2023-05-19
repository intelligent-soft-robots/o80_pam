import tennicam_client


class o80RealBall:
    """
    Wrapper over a tennicam_client.Frontend.

    Arguments:
      segment_id: o80 segment-id of the running tennicam client
    """

    def __init__(self, segment_id: str = "tennicam_client") -> None:
        self._frontend = tennicam_client.FrontEnd(segment_id)

    def get(self) -> tuple[int, list[float], list[float]]:
        """
        Read the latest known time stamp, position and velocity
        of the ball as detected by tennicam.
        """
        observation = self._frontend.pulse()
        time_stamp = observation.get_time_stamp()
        position = observation.get_position()
        velocity = observation.get_velocity()
        return time_stamp, position, velocity
