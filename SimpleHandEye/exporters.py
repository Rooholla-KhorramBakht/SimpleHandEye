import numpy as np
from scipy.spatial.transform import Rotation as R

def export2ROS(parent_T_child, parent_name, child_name, file_path):
    """
    Creates a ROS launch file with the given transformation matrix and frame names.

    :param parent_T_child: 4x4 transformation matrix (pose of the child with respect to the parent)
    :param parent_name: Name of the parent frame
    :param child_name: Name of the child frame
    :param file_path: Path where the launch file will be saved
    """
    # Extract translation and rotation from the transformation matrix
    translation = parent_T_child[:3, 3]
    rotation = R.from_matrix(parent_T_child[:3, :3])
    quaternion = rotation.as_quat()

    # Format the translation and quaternion for the launch file
    translation_str = ' '.join(map(str, translation))
    quaternion_str = ' '.join(map(str, quaternion))

    # Prepare the launch file content
    launch_file_content = f"""<launch>
  <!-- The transformation matrix is represented in the form of translation and quaternion -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="{child_name}_broadcaster"
      args="{translation_str} {quaternion_str} {parent_name} {child_name}" />
</launch>"""

    # Write to the specified file path
    with open(file_path, "w") as file:
        file.write(launch_file_content)

    return file_path