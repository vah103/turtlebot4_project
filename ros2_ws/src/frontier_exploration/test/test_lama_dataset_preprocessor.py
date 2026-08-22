import struct

from frontier_exploration.lama_dataset_preprocessor import (
    decode_signed_int8,
    downsample_occupancy,
    encode_grayscale_png,
    occupancy_to_top_down_pixels,
    pad_occupancy_positive_axes,
    plan_dataset_geometry,
    unknown_mask_to_top_down_pixels,
)


def test_hospital_geometry_becomes_608_by_296() -> None:
    geometry = plan_dataset_geometry(
        source_width=1203,
        source_height=583,
        source_resolution=0.05,
        target_resolution=0.10,
        pad_multiple=8,
    )

    assert geometry.scaled_width == 602
    assert geometry.scaled_height == 292
    assert geometry.output_width == 608
    assert geometry.output_height == 296
    assert geometry.pad_right == 6
    assert geometry.pad_top == 4


def test_decode_signed_int8_restores_unknown() -> None:
    assert decode_signed_int8(bytes([0, 100, 127, 255]), 4) == [0, 100, 127, -1]


def test_downsample_keeps_unknown_only_if_whole_footprint_unknown() -> None:
    # ROS rows, 4x2 source at 0.05 m/cell -> 2x1 at 0.10 m/pixel.
    # Left 2x2 footprint contains a free cell, right contains occupied.
    source = [
        -1, -1, 100, -1,
        -1, 0, -1, 0,
    ]

    output, width, height = downsample_occupancy(
        source,
        source_width=4,
        source_height=2,
        source_resolution=0.05,
        target_resolution=0.10,
    )

    assert (width, height) == (2, 1)
    assert output == [0, 100]


def test_downsample_all_unknown_stays_unknown() -> None:
    output, width, height = downsample_occupancy(
        [-1, -1, -1, -1],
        source_width=2,
        source_height=2,
        source_resolution=0.05,
        target_resolution=0.10,
    )

    assert (width, height) == (1, 1)
    assert output == [-1]


def test_padding_is_on_positive_x_and_y() -> None:
    source = [0, 100]
    padded = pad_occupancy_positive_axes(
        source,
        width=2,
        height=1,
        output_width=3,
        output_height=2,
    )

    assert padded == [0, 100, -1, -1, -1, -1]


def test_image_and_mask_are_top_down() -> None:
    # ROS order: bottom row first, then top row.
    occupancy = [
        0, 100,
        -1, 0,
    ]

    image = occupancy_to_top_down_pixels(occupancy, 2, 2)
    mask = unknown_mask_to_top_down_pixels(occupancy, 2, 2)

    # Top image row is [-1, 0] -> [127, 255].
    assert image == bytes([127, 255, 255, 0])
    assert mask == bytes([255, 0, 0, 0])


def test_png_encoder_writes_expected_dimensions() -> None:
    png = encode_grayscale_png(3, 2, bytes([0, 1, 2, 3, 4, 5]))

    assert png.startswith(b'\x89PNG\r\n\x1a\n')
    width, height = struct.unpack('>II', png[16:24])
    assert (width, height) == (3, 2)
