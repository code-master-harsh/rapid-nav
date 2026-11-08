"""
Check the arena model and world.

The SDF files parse, their model URIs resolve, the world is valid, and the free strip that the
sim_check drive uses holds no obstacle.
"""

import glob
import math
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.normpath(os.path.join(HERE, '..'))
MODELS = os.path.join(PACKAGE, 'models')
MODEL_DIR = os.path.join(MODELS, 'rapid_nav_arena')
MODEL_SDF = os.path.join(MODEL_DIR, 'model.sdf')
MODEL_CONFIG = os.path.join(MODEL_DIR, 'model.config')
WORLD = os.path.join(PACKAGE, 'worlds', 'arena.sdf')

# x_min, x_max, y_min, y_max in metres: kept free for the sim_check drive.
FREE_STRIP = (-1.0, 4.5, -0.6, 0.6)


def sdf_files():
    """Return every SDF and model config file of the package."""
    return [WORLD, MODEL_SDF, MODEL_CONFIG]


def test_sdf_files_parse():
    """Every SDF and model.config is well-formed XML."""
    for path in sdf_files():
        ET.parse(path)


def test_model_uris_resolve():
    """Every model://rapid_nav_arena/... URI in the package names a file or folder that exists."""
    pattern = re.compile(r'model://(rapid_nav_arena[^<\s"]*)')
    checked = 0
    for path in sdf_files():
        with open(path, encoding='utf-8') as handle:
            for match in pattern.finditer(handle.read()):
                relative = match.group(1)
                target = os.path.join(MODELS, relative)
                assert os.path.exists(target), '{} refers to missing {}'.format(path, relative)
                checked += 1
    assert checked > 0


def test_textures_exist():
    """The seven textures of the arena are present as JPEG files."""
    names = sorted(os.path.basename(path) for path in glob.glob(
        os.path.join(MODEL_DIR, 'materials', 'textures', '*.jpg')))
    assert names == sorted([
        'crate.jpg', 'floor.jpg', 'pillar.jpg', 'wall_brick.jpg',
        'wall_noise.jpg', 'wall_panels.jpg', 'wall_plain.jpg',
    ])


def gz_check(path):
    """Run gz sdf -k on one file and return the completed process."""
    environment = dict(os.environ, GZ_SIM_RESOURCE_PATH=MODELS)
    return subprocess.run(['gz', 'sdf', '-k', path], env=environment,
                          capture_output=True, text=True, check=False)


def test_sdf_is_valid():
    """
    Check that gz sdf -k accepts model.sdf and the world without its <include>.

    gz sdf -k cannot resolve model:// includes (it sets no resource callback), so the world is
    checked with its include removed. The include itself is covered by test_model_uris_resolve
    and by loading the world in the L2 check.
    """
    if shutil.which('gz') is None:
        pytest.skip('gz is not on PATH')
    result = gz_check(MODEL_SDF)
    assert result.returncode == 0, result.stdout + result.stderr

    tree = ET.parse(WORLD)
    world = tree.getroot().find('world')
    for include in world.findall('include'):
        world.remove(include)
    directory = tempfile.mkdtemp(prefix='arena_world_')
    try:
        path = os.path.join(directory, 'world_without_include.sdf')
        tree.write(path, encoding='utf-8', xml_declaration=True)
        result = gz_check(path)
        assert result.returncode == 0, result.stdout + result.stderr
    finally:
        shutil.rmtree(directory, ignore_errors=True)


def pose_xy(element):
    """Return the x and y of a <pose> element."""
    values = [float(value) for value in element.text.split()]
    return values[0], values[1]


def footprint(model):
    """Return ('box', x, y, half_x, half_y) or ('circle', x, y, radius) of a static obstacle."""
    x, y = pose_xy(model.find('pose'))
    box = model.find('link/collision/geometry/box/size')
    if box is not None:
        size = [float(value) for value in box.text.split()]
        return ('box', x, y, size[0] / 2, size[1] / 2)
    radius = float(model.find('link/collision/geometry/cylinder/radius').text)
    return ('circle', x, y, radius)


def overlaps_strip(shape):
    """Return True when the obstacle footprint touches the free strip."""
    x_min, x_max, y_min, y_max = FREE_STRIP
    if shape[0] == 'box':
        _, x, y, half_x, half_y = shape
        return (abs(x - (x_min + x_max) / 2) < half_x + (x_max - x_min) / 2
                and abs(y - (y_min + y_max) / 2) < half_y + (y_max - y_min) / 2)
    _, x, y, radius = shape
    nearest_x = min(max(x, x_min), x_max)
    nearest_y = min(max(y, y_min), y_max)
    return math.hypot(x - nearest_x, y - nearest_y) < radius


def test_free_strip_has_no_obstacle():
    """The world has six obstacles (crates and pillars), and none touches the free strip."""
    root = ET.parse(WORLD).getroot()
    obstacles = [model for model in root.iter('model')
                 if model.get('name', '').startswith(('crate_', 'pillar_'))]
    assert len(obstacles) == 6
    for model in obstacles:
        shape = footprint(model)
        assert not overlaps_strip(shape), model.get('name')
