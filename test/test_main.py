import pytest
from unittest.mock import Mock, patch
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from main import Follow_Trace_Node

# Constants for tests
TEST_LINEAR_SPEED = 0.5
TEST_ANGULAR_SPEED = 1.0
TEST_LINEAR_SLOW_SPEED = 0.1

# Mock data for tests
mock_odometry_data = Odometry()
mock_odometry_data.pose.pose.position = Point(x=1.0, y=1.0, z=0.0)
mock_odometry_data.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

mock_image_data = Image()
mock_image_data.encoding = 'rgb8'

mock_state_data = String(data='1')

@pytest.fixture
def node():
    return Follow_Trace_Node(TEST_LINEAR_SPEED, TEST_ANGULAR_SPEED, TEST_LINEAR_SLOW_SPEED)

@pytest.mark.parametrize(
    "test_id, initial_state, expected_state",
    [
        ("HP_01", 1, 1),  # Happy path: initial state is 1, expected state remains 1
        ("HP_02", 4, 4),  # Happy path: initial state is 4, expected state remains 4
        ("EC_01", 99, 99),  # Edge case: initial state is 99, expected state remains 99
        ("EC_02", 0, 0),  # Edge case: initial state is 0, expected state remains 0
    ]
)
def test_state_callback(node, test_id, initial_state, expected_state):
    # Arrange
    mock_msg = String(data=str(initial_state))
    
    # Act
    node.state_callback(mock_msg)
    
    # Assert
    assert node.state == expected_state, f"Test ID: {test_id} - State should be {expected_state}, but got {node.state}"

@pytest.mark.parametrize(
    "test_id, initial_yaw_degree, initial_start_angle, expected_do_rotate",
    [
        ("HP_01", 0, -999999, 1),  # Happy path: initial yaw degree is 0, start angle is default, expected do_rotate is 1
        ("EC_01", 90, 0, 0),  # Edge case: initial yaw degree is 90, start angle is 0, expected do_rotate is 0
    ]
)
def test_turn_robot(node, test_id, initial_yaw_degree, initial_start_angle, expected_do_rotate):
    # Arrange
    node.yaw_degree = initial_yaw_degree
    node.start_angle = initial_start_angle
    mock_publisher = Mock()
    
    # Act
    node.turn_robot(mock_publisher, 0.25)
    
    # Assert
    assert node.do_rotate == expected_do_rotate, f"Test ID: {test_id} - do_rotate should be {expected_do_rotate}, but got {node.do_rotate}"

@pytest.mark.parametrize(
    "test_id, initial_total_distance, initial_start_distance, expected_do_forward",
    [
        ("HP_01", 0, -999999, 1),  # Happy path: initial total distance is 0, start distance is default, expected do_forward is 1
        ("EC_01", 10, 0, 0),  # Edge case: initial total distance is 10, start distance is 0, expected do_forward is 0
    ]
)
def test_move_robot(node, test_id, initial_total_distance, initial_start_distance, expected_do_forward):
    # Arrange
    node.total_distance = initial_total_distance
    node.start_distance = initial_start_distance
    mock_publisher = Mock()
    
    # Act
    node.move_robot(mock_publisher, 0.1)
    
    # Assert
    assert node.do_forward == expected_do_forward, f"Test ID: {test_id} - do_forward should be {expected_do_forward}, but got {node.do_forward}"

@pytest.mark.parametrize(
    "test_id, initial_state, expected_zmeika_state",
    [
        ("HP_01", 4, 0),  # Happy path: initial state is 4, expected zmeika_state is 0
        ("EC_01", 1, -1),  # Edge case: initial state is 1, expected zmeika_state remains -1
    ]
)
def test_timer_callback(node, test_id, initial_state, expected_zmeika_state):
    # Arrange
    node.state = initial_state
    
    # Act
    node.timer_callback()
    
    # Assert
    assert node.zmeika_state == expected_zmeika_state, f"Test ID: {test_id} - zmeika_state should be {expected_zmeika_state}, but got {node.zmeika_state}"

@pytest.mark.parametrize(
    "test_id, initial_is_first_message, initial_last_position, expected_total_distance",
    [
        ("HP_01", True, Point(), 0.0),  # Happy path: first message, no movement, expected total distance is 0.0
        ("EC_01", False, Point(x=1.0, y=1.0, z=0.0), 1.4142135623730951),  # Edge case: not first message, moved diagonally, expected total distance is sqrt(2)
    ]
)
def test_pose_callback(node, test_id, initial_is_first_message, initial_last_position, expected_total_distance):
    # Arrange
    node.is_first_message = initial_is_first_message
    node.last_position = initial_last_position
    mock_data = mock_odometry_data
    
    # Act
    node.pose_callback(mock_data)
    
    # Assert
    assert node.total_distance == expected_total_distance, f"Test ID: {test_id} - Total distance should be {expected_total_distance}, but got {node.total_distance}"

@pytest.mark.parametrize(
    "test_id, initial_state, expected_linear_x",
    [
        ("HP_01", 1, TEST_LINEAR_SPEED),  # Happy path: state is 1, expected linear speed is TEST_LINEAR_SPEED
        ("EC_01", 99, 0.0),  # Edge case: state is 99, expected linear speed is 0.0
    ]
)
def test_camera_callback(node, test_id, initial_state, expected_linear_x):
    # Arrange
    node.state = initial_state
    mock_msg = mock_image_data
    
    # Act
    with patch.object(node, '_cv_bridge', autospec=True) as mock_cv_bridge:
        mock_cv_bridge.imgmsg_to_cv2.return_value = Mock()
        node.camera_callback(mock_msg)
    
    # Assert
    assert node._robot_cmd_vel_pub.publish.call_args[0][0].linear.x == expected_linear_x, f"Test ID: {test_id} - Linear x should be {expected_linear_x}, but got {node._robot_cmd_vel_pub.publish.call_args[0][0].linear.x}"
