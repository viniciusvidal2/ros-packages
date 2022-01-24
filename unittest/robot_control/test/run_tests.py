import rosunit
import rotate_robot_test_cases

# rosunit
rosunit.unitrun('robot_control', 'rotate_robot_test_cases',
                'rotate_robot_test_cases.MyTestSuite')