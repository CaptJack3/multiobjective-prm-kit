from pathlib import Path
import shutil
import pytest
from prmkit.cli import main
from prmkit.config import load

@pytest.fixture
def demo(tmp_path):
    project = tmp_path / 'project'
    assert main(['init', str(project)]) == 0
    return load(project / 'experiment.yaml')

@pytest.fixture
def resources():
    return Path(__file__).parents[1] / 'src/prmkit/resources'
