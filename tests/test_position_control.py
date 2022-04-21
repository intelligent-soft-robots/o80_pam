import pytest
import pam_interface
import o80_pam

@pytest.fixture
def get_1d_position_controller()->o80_pam.PositionController:

    q_current = [10.]*4
    q_desired = [20.]*4
    dq_desired = [5.]*4

    config = pam_interface.Configuration()
    config.max_pressures_ago = [100]*4
    config.min_pressures_ago = [0]*4
    config.max_pressures_antago = [100]*4
    config.min_pressures_antago = [0]*4

    kp = [1.]*4
    kd = [1.]*4
    ki = [1.]*4
    ndp = [50.]*4
    
    time_step = 0.01
    extra_steps = 10

    return o80_pam.PositionController(
        q_current,q_desired,dq_desired,
        config,kp,kd,ki,ndp,
        time_step,extra_steps
    )
    
    
def test_internals(get_1d_position_controller):

    position_controller = get_1d_position_controller

    assert position_controller._range_agos == [100]*4
    assert position_controller._range_antagos == [100]*4

    for dof in range(4):
        assert position_controller._q_trajectories[dof][0] == 10.
        assert position_controller._q_trajectories[dof][-1] == 20.
        assert position_controller._dq_trajectories[dof][0] == 5.
        assert position_controller._dq_trajectories[dof][-11] == 5.
        assert position_controller._dq_trajectories[dof][-1] == 0.
        assert position_controller._dq_trajectories[dof][-10] == 0.

