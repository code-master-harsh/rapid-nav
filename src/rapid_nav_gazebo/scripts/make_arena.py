"""
Generate the textures and the model files of the RAPID-NAV arena.

Run it with numpy and Pillow installed:

    python3 scripts/make_arena.py [--out DIR]

DIR is the models folder (default: the models/ folder of this package). The script writes
DIR/rapid_nav_arena/model.config, model.sdf and materials/textures/*.jpg. Each texture has a fixed
seed, so running the script again writes identical files. The script is not installed; developers
run it and commit the result.
"""

import argparse
import os

import numpy as np
from PIL import Image, ImageDraw

MODEL_NAME = 'rapid_nav_arena'
SIZE = 1024
JPEG_QUALITY = 88
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Room geometry in metres: the floor spans x in [-5, 5] and y in [-4, 4].
FLOOR_Z = -0.01
FLOOR_THICKNESS = 0.02
WALL_THICKNESS = 0.1
WALL_HEIGHT = 2.0
WALL_CENTRE_Z = 1.0
TILE_SIZE = 2.0
TILE_XS = [-4.0, -2.0, 0.0, 2.0, 4.0]
TILE_YS = [-3.0, -1.0, 1.0, 3.0]

PANEL_COLOURS = [
    (176, 120, 98), (98, 128, 150), (132, 150, 105),
    (200, 180, 120), (110, 95, 130), (160, 90, 90),
]
PILLAR_COLOURS = np.array(
    [[170, 160, 140], [120, 90, 70], [90, 120, 110], [200, 190, 170], [140, 110, 130]],
    dtype=np.float64,
)


def value_noise(rng, grid, octaves):
    """
    Return multi-octave value noise in [0, 1] with SIZE x SIZE pixels.

    Each octave is a random lattice of grid * 2**octave cells, upsampled with bicubic
    interpolation; the octaves are summed with halving weights.
    """
    total = np.zeros((SIZE, SIZE), dtype=np.float64)
    weight_sum = 0.0
    for octave in range(octaves):
        cells = grid * 2 ** octave
        weight = 0.5 ** octave
        lattice = rng.random((cells, cells)).astype(np.float32)
        layer = Image.fromarray(lattice).resize((SIZE, SIZE), Image.Resampling.BICUBIC)
        total += weight * np.asarray(layer, dtype=np.float64)
        weight_sum += weight
    total /= weight_sum
    low = total.min()
    high = total.max()
    return (total - low) / (high - low)


def gaussian_grain(rng, sigma):
    """Return Gaussian noise of SIZE x SIZE pixels with the given standard deviation."""
    return rng.normal(0.0, sigma, (SIZE, SIZE))


def make_floor(seed):
    """Return speckled warm-grey concrete: large-scale variation, fine speckle and a few stains."""
    rng = np.random.default_rng(seed)
    large = value_noise(rng, grid=4, octaves=3) - 0.5
    speckle = gaussian_grain(rng, 10.0)
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    stains = np.zeros((SIZE, SIZE), dtype=np.float64)
    for _ in range(6):
        centre_x, centre_y = rng.uniform(0, SIZE, size=2)
        radius = rng.uniform(40, 110)
        stains += np.exp(-((xx - centre_x) ** 2 + (yy - centre_y) ** 2) / (2 * radius ** 2))
    shade = 18 * large + speckle - 16 * np.clip(stains, 0, 1)
    base = np.array([150.0, 145.0, 138.0])
    return base + shade[..., np.newaxis]


def make_brick(seed):
    """Return a running-bond brick wall: 8 courses, 4 bricks per course, light mortar."""
    rng = np.random.default_rng(seed)
    course = SIZE // 8
    brick = SIZE // 4
    mortar = 6
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    row = yy // course
    shifted = (xx + (row % 2) * (brick // 2)) % SIZE
    brick_id = row * 4 + shifted // brick
    per_brick = rng.normal(0.0, 12.0, (32, 1)) + rng.normal(0.0, 4.0, (32, 3))
    base = np.array([150.0, 72.0, 58.0])
    grain = 22 * (value_noise(rng, grid=64, octaves=2) - 0.5) + gaussian_grain(rng, 5.0)
    img = base + per_brick[brick_id] + grain[..., np.newaxis]
    joints = (yy % course < mortar) | (shifted % brick < mortar)
    mortar_colour = np.array([200.0, 194.0, 184.0])
    img[joints] = mortar_colour + 0.5 * grain[joints][:, np.newaxis]
    return img


def make_panels(seed):
    """Return overlapping muted rectangles on a light background, with grain."""
    rng = np.random.default_rng(seed)
    canvas = Image.new('RGB', (SIZE, SIZE), (190, 184, 172))
    draw = ImageDraw.Draw(canvas)
    for _ in range(18):
        width, height = (int(value) for value in rng.integers(120, 420, size=2))
        left, top = (int(value) for value in rng.integers(-100, SIZE - 50, size=2))
        colour = PANEL_COLOURS[int(rng.integers(len(PANEL_COLOURS)))]
        draw.rectangle(
            [left, top, left + width, top + height],
            fill=colour, outline=(40, 40, 40), width=3,
        )
    img = np.asarray(canvas, dtype=np.float64)
    return img + gaussian_grain(rng, 5.0)[..., np.newaxis]


def make_marble(seed):
    """Return marble-like blue, green and grey noise with thin light veins."""
    rng = np.random.default_rng(seed)
    t = value_noise(rng, grid=3, octaves=5)
    yy = np.mgrid[0:SIZE, 0:SIZE][0] / SIZE
    veins = (0.5 + 0.5 * np.sin(2 * np.pi * (3 * yy + 1.5 * t))) ** 6
    dark = np.array([46.0, 70.0, 94.0])
    middle = np.array([96.0, 128.0, 124.0])
    light = np.array([198.0, 206.0, 202.0])
    body = dark + (middle - dark) * t[..., np.newaxis]
    img = body + (light - body) * (0.6 * veins)[..., np.newaxis]
    return img + gaussian_grain(rng, 3.0)[..., np.newaxis]


def make_plain(seed):
    """Return near-uniform light grey (200 +- 2 levels): the deliberately textureless wall."""
    rng = np.random.default_rng(seed)
    grey = 200 + rng.integers(-2, 3, size=(SIZE, SIZE)).astype(np.float64)
    return np.repeat(grey[..., np.newaxis], 3, axis=2)


def make_crate(seed):
    """Return a wooden crate face: horizontal planks, grain streaks, gaps and a dark frame."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    plank_height = SIZE // 8
    plank = yy // plank_height
    tint = rng.normal(0.0, 10.0, size=8)[plank]
    streaks = rng.normal(0.0, 6.0, size=(SIZE, 1))
    grain = gaussian_grain(rng, 4.0)
    base = np.array([152.0, 110.0, 66.0])
    img = base + (tint + streaks + grain)[..., np.newaxis]
    img[yy % plank_height < 4] *= 0.6
    frame = (xx < 24) | (xx >= SIZE - 24) | (yy < 24) | (yy >= SIZE - 24)
    img[frame] *= 0.55
    return img


def make_pillar(seed):
    """Return vertical colour stripes of varying width with grain."""
    rng = np.random.default_rng(seed)
    column_colour = np.zeros(SIZE, dtype=np.int64)
    start = 0
    while start < SIZE:
        width = int(rng.integers(40, 141))
        column_colour[start:start + width] = int(rng.integers(len(PILLAR_COLOURS)))
        start += width
    stripes = np.repeat(PILLAR_COLOURS[column_colour][np.newaxis, :, :], SIZE, axis=0)
    return stripes + gaussian_grain(rng, 6.0)[..., np.newaxis]


# (texture name, generator, seed)
TEXTURES = [
    ('floor', make_floor, 101),
    ('wall_brick', make_brick, 102),
    ('wall_panels', make_panels, 103),
    ('wall_noise', make_marble, 104),
    ('wall_plain', make_plain, 105),
    ('crate', make_crate, 106),
    ('pillar', make_pillar, 107),
]


def save_texture(values, path):
    """Write float RGB values as a JPEG with the fixed quality."""
    data = np.clip(np.rint(values), 0, 255).astype(np.uint8)
    Image.fromarray(data).save(path, format='JPEG', quality=JPEG_QUALITY)


def material_xml(texture):
    """Return the PBR material that uses one arena texture."""
    return (
        '<material>\n'
        '          <ambient>1 1 1 1</ambient>\n'
        '          <diffuse>1 1 1 1</diffuse>\n'
        '          <specular>0.1 0.1 0.1 1</specular>\n'
        '          <pbr>\n'
        '            <metal>\n'
        '              <albedo_map>model://{model}/materials/textures/{texture}.jpg</albedo_map>\n'
        '              <metalness>0</metalness>\n'
        '              <roughness>0.9</roughness>\n'
        '            </metal>\n'
        '          </pbr>\n'
        '        </material>'
    ).format(model=MODEL_NAME, texture=texture)


def box_geometry(size):
    """Return the geometry block of a box with the given size string."""
    return '<geometry><box><size>{}</size></box></geometry>'.format(size)


def visual_xml(name, pose, size, texture):
    """Return a textured box visual."""
    return (
        '      <visual name="{name}">\n'
        '        <pose>{pose}</pose>\n'
        '        <geometry><box><size>{size}</size></box></geometry>\n'
        '        {material}\n'
        '      </visual>\n'
    ).format(name=name, pose=pose, size=size, material=material_xml(texture))


def collision_xml(name, pose, size):
    """Return a box collision."""
    return (
        '      <collision name="{name}">\n'
        '        <pose>{pose}</pose>\n'
        '        {geometry}\n'
        '      </collision>\n'
    ).format(name=name, pose=pose, geometry=box_geometry(size))


def model_config():
    """Return model.config."""
    return (
        '<?xml version="1.0"?>\n'
        '<model>\n'
        '  <name>{model}</name>\n'
        '  <version>0.1.0</version>\n'
        '  <sdf version="1.10">model.sdf</sdf>\n'
        '  <author>\n'
        '    <name>Harsh Vardhan</name>\n'
        '    <email>harsh.v.sharma@outlook.com</email>\n'
        '  </author>\n'
        '  <description>Textured 10 m x 8 m arena for the RAPID-NAV optical-flow '
        'experiments.</description>\n'
        '</model>\n'
    ).format(model=MODEL_NAME)


def model_sdf():
    """Return model.sdf: the static room, one link with the floor tiles and the walls."""
    parts = [
        '<?xml version="1.0" ?>\n',
        '<!-- Generated by scripts/make_arena.py. Edit the script, not this file. -->\n',
        '<sdf version="1.10">\n',
        '  <model name="{}">\n'.format(MODEL_NAME),
        '    <static>true</static>\n',
        '    <link name="arena">\n',
    ]
    parts.append(collision_xml(
        'floor_collision', '0 0 {} 0 0 0'.format(FLOOR_Z), '10 8 {}'.format(FLOOR_THICKNESS)))
    for x in TILE_XS:
        for y in TILE_YS:
            parts.append(visual_xml(
                'floor_{}_{}'.format(x, y),
                '{} {} {} 0 0 0'.format(x, y, FLOOR_Z),
                '{} {} {}'.format(TILE_SIZE, TILE_SIZE, FLOOR_THICKNESS),
                'floor',
            ))

    wall_z = WALL_CENTRE_Z
    segment_long = '{} {} {}'.format(TILE_SIZE, WALL_THICKNESS, WALL_HEIGHT)
    segment_side = '{} {} {}'.format(WALL_THICKNESS, TILE_SIZE, WALL_HEIGHT)
    # north and south: full length including the corners; 2 m segments as visuals
    for name, y, texture in [('north', 4.05, 'wall_brick'), ('south', -4.05, 'wall_panels')]:
        parts.append(collision_xml(
            name + '_collision', '0 {} {} 0 0 0'.format(y, wall_z), '10.2 {} {}'.format(
                WALL_THICKNESS, WALL_HEIGHT)))
        for x in TILE_XS:
            parts.append(visual_xml(
                '{}_{}'.format(name, x), '{} {} {} 0 0 0'.format(x, y, wall_z),
                segment_long, texture))
    # east and west: full length of the room; 2 m segments as visuals
    parts.append(collision_xml('east_collision', '5.05 0 {} 0 0 0'.format(wall_z),
                               '{} 8.0 {}'.format(WALL_THICKNESS, WALL_HEIGHT)))
    parts.append(collision_xml('west_collision', '-5.05 0 {} 0 0 0'.format(wall_z),
                               '{} 8.0 {}'.format(WALL_THICKNESS, WALL_HEIGHT)))
    for y in TILE_YS:
        parts.append(visual_xml('east_{}'.format(y), '5.05 {} {} 0 0 0'.format(y, wall_z),
                                segment_side, 'wall_noise'))
    for y in TILE_YS:
        texture = 'wall_noise' if y < 0 else 'wall_plain'
        parts.append(visual_xml('west_{}'.format(y), '-5.05 {} {} 0 0 0'.format(y, wall_z),
                                segment_side, texture))
    parts.append('    </link>\n  </model>\n</sdf>\n')
    return ''.join(parts)


def write_text(path, text):
    """Write a text file with Unix line endings."""
    with open(path, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(text)


def main(argv=None):
    """Write the textures and the model files; return the exit status."""
    parser = argparse.ArgumentParser(description='Generate the RAPID-NAV arena model.')
    parser.add_argument(
        '--out',
        default=os.path.join(PACKAGE_DIR, 'models'),
        help='models folder; the model is written to OUT/rapid_nav_arena (default: this package)',
    )
    args = parser.parse_args(argv)

    model_dir = os.path.join(args.out, MODEL_NAME)
    texture_dir = os.path.join(model_dir, 'materials', 'textures')
    os.makedirs(texture_dir, exist_ok=True)
    for name, generate, seed in TEXTURES:
        save_texture(generate(seed), os.path.join(texture_dir, name + '.jpg'))
    write_text(os.path.join(model_dir, 'model.config'), model_config())
    write_text(os.path.join(model_dir, 'model.sdf'), model_sdf())
    print('wrote {} textures and the model files to {}'.format(len(TEXTURES), model_dir))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
