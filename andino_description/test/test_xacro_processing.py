import os
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

andino_description_pkg = get_package_share_directory("andino_description")

def test_xacro_processing():
    # Get the file path
    xacro_file_path = os.path.join(andino_description_pkg, 'urdf', 'andino.urdf.xacro')

    # Test xacro processing
    try:
        xacro.process_file(xacro_file_path)
    except Exception as e:
        pytest.fail(f"Xacro processing failed: {e}")
