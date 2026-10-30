"""Current node follows manual odometry rather than an interrupted route."""
from robot_agent.core import AgentCore
from warehouse_sim.kinematics import Pose
from test_manual_routing import make_map


def test_current_node_tracks_manual_position_and_clears_between_nodes():
    agent = AgentCore(warehouse=make_map())
    agent.odometry(Pose(-4, 0, 0), 1.)
    agent.start_route('order', ['A', 'B', 'C'], 1.)
    agent.manual_mode(True)
    assert agent.current_node_id(1.) == 'A'
    agent.odometry(Pose(0, 0, 0), 2.)
    assert agent.current_node_id(2.) == 'B'
    agent.odometry(Pose(2, 0, 0), 3.)
    assert agent.current_node_id(3.) == ''
    agent.odometry(Pose(4, 0, 0), 4.)
    agent.manual_mode(False)
    assert agent.current_node_id(4.) == 'C'
    assert agent.current_node_id(5.) == ''
