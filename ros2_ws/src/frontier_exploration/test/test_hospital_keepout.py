import numpy as np

from frontier_exploration.hospital_keepout import (
    collada_support_polygons,
    rasterize_safe_floor,
)


def test_collada_support_polygons_excludes_raised_platform(tmp_path):
    dae = tmp_path / 'floor.dae'
    dae.write_text(
        '''<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <unit meter="0.01" name="centimeter"/>
    <up_axis>Y_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="g">
      <mesh>
        <source id="positions">
          <float_array id="positions-array" count="24">
            0 0 0 100 0 0 100 0 -100 0 0 -100
            200 15 0 300 15 0 300 15 -100 200 15 -100
          </float_array>
          <technique_common>
            <accessor source="#positions-array" count="8" stride="3"/>
          </technique_common>
        </source>
        <vertices id="vertices">
          <input semantic="POSITION" source="#positions"/>
        </vertices>
        <polylist count="2">
          <input semantic="VERTEX" source="#vertices" offset="0"/>
          <vcount>4 4</vcount>
          <p>0 1 2 3 4 5 6 7</p>
        </polylist>
      </mesh>
    </geometry>
  </library_geometries>
</COLLADA>
''',
        encoding='utf-8',
    )

    polygons = collada_support_polygons(
        dae,
        max_support_height_m=0.03,
    )

    assert len(polygons) == 1
    assert polygons[0] == [
        (0.0, -0.0),
        (1.0, -0.0),
        (1.0, 1.0),
        (0.0, 1.0),
    ]


def test_rasterize_safe_floor_keeps_center_and_erodes_edge():
    safe, origin_x, origin_y = rasterize_safe_floor(
        [[(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]],
        resolution=0.10,
        safety_margin_m=0.20,
        outer_padding_m=0.50,
    )

    def cell(x, y):
        column = int((x - origin_x) / 0.10)
        row = int((y - origin_y) / 0.10)
        return bool(safe[row, column])

    assert cell(0.0, 0.0)
    assert not cell(0.95, 0.0)
    assert not cell(1.20, 0.0)
    assert np.count_nonzero(safe) > 0


def test_rasterize_safe_floor_rejects_invalid_resolution():
    try:
        rasterize_safe_floor(
            [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]],
            resolution=0.0,
            safety_margin_m=0.2,
            outer_padding_m=0.5,
        )
    except ValueError as exc:
        assert 'resolution' in str(exc)
    else:
        raise AssertionError('Expected invalid resolution to raise ValueError')
