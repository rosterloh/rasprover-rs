import os
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

def test_xacro_processing():
    """Test main xacro file (rasprover.urdf.xacro) processing"""
    # Get the file path.
    xacro_file_path = os.path.join(get_package_share_directory("rasprover_description"), 'urdf', 'rasprover.urdf.xacro')

    # Test xacro processing.
    try:
        xacro.process_file(xacro_file_path)
    except Exception as e:
        pytest.fail(f"Xacro processing failed: {e}")