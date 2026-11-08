"""Run flake8 on the package (the ament_python template test)."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """The package has no flake8 errors."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, 'Found %d code style errors / warnings:\n    ' % len(errors) + \
        '\n    '.join(errors)
