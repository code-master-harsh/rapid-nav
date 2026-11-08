"""Run pep257 on the package (the ament_python template test)."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """The package has no pep257 errors."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
