from .o80_ball import o80Ball


class o80Goal(o80Ball):
    def __init__(self, segment_id, frontend=None):
        o80Ball.__init__(self, segment_id, frontend)
