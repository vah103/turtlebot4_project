import math

import pytest

from frontier_exploration.hospital_canvas import (
    Bounds2D,
    ModelPlacement,
    canvas_from_bounds,
    collada_ground_bounds,
    read_model_placements,
    transform_bounds,
    union_bounds,
)


def test_collada_ground_bounds_uses_unit_scale_and_y_up(tmp_path):
    dae = tmp_path / 'mesh.dae'
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
          <float_array id="positions-array" count="12">-100 0 -200 300 0 -200 300 0 400 -100 0 400</float_array>
          <technique_common>
            <accessor source="#positions-array" count="4" stride="3"/>
          </technique_common>
        </source>
        <vertices id="vertices">
          <input semantic="POSITION" source="#positions"/>
        </vertices>
      </mesh>
    </geometry>
  </library_geometries>
</COLLADA>
''',
        encoding='utf-8',
    )

    bounds = collada_ground_bounds(dae)

    assert bounds == Bounds2D(
        min_x=-1.0,
        max_x=3.0,
        min_y=-4.0,
        max_y=2.0,
    )


def test_read_model_placements_from_sdf_includes(tmp_path):
    world = tmp_path / 'hospital.sdf'
    world.write_text(
        '''<sdf version="1.9">
  <world name="hospital_world">
    <include>
      <uri>model://floor</uri>
      <pose>1.0 2.0 0 0 0 0.25</pose>
    </include>
    <include>
      <uri>model://walls</uri>
    </include>
  </world>
</sdf>
''',
        encoding='utf-8',
    )

    placements = read_model_placements(world, {'floor', 'walls'})

    assert placements['floor'] == ModelPlacement('floor', 1.0, 2.0, 0.25)
    assert placements['walls'] == ModelPlacement('walls', 0.0, 0.0, 0.0)


def test_transform_bounds_rotates_and_translates_planar_aabb():
    bounds = Bounds2D(-1.0, 2.0, -3.0, 4.0)
    placement = ModelPlacement('test', 10.0, 20.0, math.pi / 2.0)

    transformed = transform_bounds(bounds, placement)

    assert transformed.min_x == pytest.approx(6.0)
    assert transformed.max_x == pytest.approx(13.0)
    assert transformed.min_y == pytest.approx(19.0)
    assert transformed.max_y == pytest.approx(22.0)


def test_union_bounds_combines_models():
    combined = union_bounds([
        Bounds2D(-2.0, 1.0, -1.0, 4.0),
        Bounds2D(0.0, 5.0, -3.0, 2.0),
    ])

    assert combined == Bounds2D(-2.0, 5.0, -3.0, 4.0)


def test_canvas_from_bounds_adds_margin_and_snaps_outward():
    canvas = canvas_from_bounds(
        Bounds2D(-1.01, 2.01, -0.01, 1.01),
        resolution=0.05,
        margin_m=1.0,
    )

    assert canvas['fixed_canvas_configured'] is True
    assert canvas['canvas_origin_x'] == pytest.approx(-2.05)
    assert canvas['canvas_origin_y'] == pytest.approx(-1.05)
    assert canvas['canvas_max_x'] == pytest.approx(3.05)
    assert canvas['canvas_max_y'] == pytest.approx(2.05)
    assert canvas['canvas_width_cells'] == 102
    assert canvas['canvas_height_cells'] == 62


def test_canvas_rejects_invalid_parameters():
    bounds = Bounds2D(0.0, 1.0, 0.0, 1.0)

    with pytest.raises(ValueError):
        canvas_from_bounds(bounds, resolution=0.0, margin_m=1.0)
    with pytest.raises(ValueError):
        canvas_from_bounds(bounds, resolution=0.05, margin_m=-1.0)
