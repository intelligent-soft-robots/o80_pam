from .o80_ball import o80Ball


class o80Target(o80Ball):
    def __init__(self, segment_id):
        o80Ball.__init__(self, segment_id)
