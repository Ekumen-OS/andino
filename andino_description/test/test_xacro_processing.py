import os
import pytest
import xacro

def test_xacro_processing():
    # Get the file path
    xacro_file_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'andino.urdf.xacro')

    # Test xacro processing
    try:
        xacro.process_file(xacro_file_path)
    except Exception as e:
        pytest.fail(f"Xacro processing failed: {e}")
