"""Generate SCSA.txt from a CT volume and seven anatomical landmarks.

Required landmark JSON keys:
    coordinateSystem: "RAS" or "LPS"
    units: "mm"
    F_2-1, F_2-2, F_2-3
    Mandible Angle-1, Mandible Angle-2
    Lateral Pole-1, Lateral Pole-2

Use --neighboring-planes to measure the reference plane and ten parallel planes
at offsets -5 to -1 mm and +1 to +5 mm and retain the largest area.

Run this file with 3D Slicer, not a standard Python interpreter:
    Slicer --no-splash --no-main-window --python-script generateSCSA.py -- \
        --ct CT.nrrd --landmarks SCSA_landmarks.json --output SCSA.txt
"""

import argparse
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import traceback

import numpy as np
import slicer
import vtk


LANDMARK_KEYS = (
    "F_2-1",
    "F_2-2",
    "F_2-3",
    "Mandible Angle-1",
    "Mandible Angle-2",
    "Lateral Pole-1",
    "Lateral Pole-2",
)

MUSCLE_PLANES = (
    ("temporalis_right", "T_R", "T"),
    ("temporalis_left", "T_L", "T"),
    ("masseter_right", "Ma_R", "MaMP"),
    ("masseter_left", "Ma_L", "MaMP"),
    ("medial_pterygoid_right", "MP_R", "MaMP"),
    ("medial_pterygoid_left", "MP_L", "MaMP"),
    ("lateral_pterygoid_right", "LP_R", "LP"),
    ("lateral_pterygoid_left", "LP_L", "LP"),
)

MAMP_ANGLE_DEGREES = 30.0
MAMP_OFFSET_MM = 25.0
TEMPORALIS_OFFSET_MM = 10.0
LATERAL_PTERYGOID_OFFSET_MM = 10.0
NEIGHBORING_PLANE_OFFSETS_MM = tuple(range(-5, 0)) + tuple(range(1, 6))
ZERO_TOLERANCE = 1e-8


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run TotalSegmentator head_muscles and calculate eight muscle SCSAs."
    )
    parser.add_argument("--ct", required=True, help="CT volume file readable by 3D Slicer.")
    parser.add_argument(
        "--landmarks",
        required=True,
        help="JSON file containing the seven required landmarks in RAS or LPS millimetres.",
    )
    parser.add_argument(
        "--output",
        default="SCSA.txt",
        help="Output text file. Values are written in cm^2.",
    )
    parser.add_argument(
        "--cpu",
        action="store_true",
        help="Force CPU inference. By default TotalSegmentator uses its normal GPU selection.",
    )
    parser.add_argument(
        "--neighboring-planes",
        action="store_true",
        help="Use the largest area from the reference plane and ten parallel planes at +/-1 to 5 mm.",
    )
    parser.add_argument(
        "--status-file",
        help="Optional status file used by the MATLAB launcher.",
    )
    return parser.parse_args()


def _point(value, key):
    try:
        point = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Landmark '{key}' must contain three numeric coordinates.") from exc
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError(f"Landmark '{key}' must contain exactly three finite coordinates.")
    return point


def load_landmarks(path):
    landmark_path = Path(path).expanduser().resolve()
    if not landmark_path.is_file():
        raise FileNotFoundError(f"Landmark file not found: {landmark_path}")

    with landmark_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("The landmark JSON root must be an object.")

    allowed_keys = set(LANDMARK_KEYS) | {"coordinateSystem", "units"}
    unknown_keys = sorted(set(data) - allowed_keys)
    if unknown_keys:
        raise ValueError(f"Unknown landmark JSON keys: {', '.join(unknown_keys)}")

    missing_keys = [key for key in LANDMARK_KEYS if key not in data]
    if missing_keys:
        raise ValueError(f"Missing landmark JSON keys: {', '.join(missing_keys)}")

    coordinate_system = str(data.get("coordinateSystem", "")).upper()
    if coordinate_system not in {"RAS", "LPS"}:
        raise ValueError("coordinateSystem must be explicitly set to 'RAS' or 'LPS'.")
    if str(data.get("units", "")).lower() != "mm":
        raise ValueError("units must be explicitly set to 'mm'.")

    points = {key: _point(data[key], key) for key in LANDMARK_KEYS}
    if coordinate_system == "LPS":
        lps_to_ras = np.array([-1.0, -1.0, 1.0])
        points = {key: value * lps_to_ras for key, value in points.items()}
    return points


def _unit(vector, description):
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= ZERO_TOLERANCE:
        raise ValueError(f"Cannot define {description}; its points are coincident or degenerate.")
    return vector / norm


def compute_planes(points):
    fh1 = points["F_2-1"]
    fh2 = points["F_2-2"]
    fh3 = points["F_2-3"]
    fh_normal = _unit(np.cross(fh2 - fh1, fh3 - fh1), "the FH plane")
    if abs(float(fh_normal[2])) <= ZERO_TOLERANCE:
        raise ValueError("The FH plane normal has no superior/inferior direction.")
    fh_origin = (fh1 + fh2 + fh3) / 3.0

    mandible1 = points["Mandible Angle-1"]
    mandible2 = points["Mandible Angle-2"]
    mandible_direction = _unit(mandible2 - mandible1, "the Mandible Angle line")
    inferior_fh_normal = fh_normal if fh_normal[2] < 0.0 else -fh_normal
    target_cosine = math.cos(math.radians(MAMP_ANGLE_DEGREES))
    projected_fh_normal = (
        inferior_fh_normal
        - np.dot(inferior_fh_normal, mandible_direction) * mandible_direction
    )
    maximum_cosine = float(np.linalg.norm(projected_fh_normal))
    if maximum_cosine + 1e-7 < target_cosine:
        raise ValueError("The Mandible Angle line cannot define a plane 30 degrees from the FH plane.")
    projected_unit = _unit(projected_fh_normal, "the Ma/MP plane basis")
    perpendicular_unit = _unit(
        np.cross(mandible_direction, projected_unit), "the Ma/MP plane basis"
    )
    projected_coefficient = target_cosine / maximum_cosine
    perpendicular_coefficient = math.sqrt(max(0.0, 1.0 - projected_coefficient**2))
    superior_candidates = [
        -_unit(
            projected_coefficient * projected_unit
            + sign * perpendicular_coefficient * perpendicular_unit,
            "the Ma/MP reference plane",
        )
        for sign in (-1.0, 1.0)
    ]
    anterosuperior_candidates = [
        candidate
        for candidate in superior_candidates
        if candidate[1] > ZERO_TOLERANCE and candidate[2] > ZERO_TOLERANCE
    ]
    if not anterosuperior_candidates:
        raise ValueError(
            "The landmarks cannot define an anterosuperior Ma/MP plane normal; check their labels and coordinate system."
        )
    ref_mamp_normal = max(anterosuperior_candidates, key=lambda candidate: candidate[1])
    superior_fh_normal = -inferior_fh_normal
    angle_residual = abs(float(np.dot(ref_mamp_normal, superior_fh_normal)) - target_cosine)
    if angle_residual > 1e-7:
        raise RuntimeError("The Ma/MP reference plane did not reach 30 degrees.")
    ref_mamp_origin = (mandible1 + mandible2) / 2.0

    lateral1 = points["Lateral Pole-1"]
    lateral2 = points["Lateral Pole-2"]
    ref_lp_normal = _unit(
        np.cross(lateral1 - lateral2, fh_normal),
        "the lateral-pterygoid reference plane",
    )
    if abs(float(ref_lp_normal[1])) <= ZERO_TOLERANCE:
        raise ValueError("The lateral-pterygoid plane normal has no anterior/posterior direction.")
    ref_lp_origin = (lateral1 + lateral2) / 2.0

    mamp_normal = ref_mamp_normal if ref_mamp_normal[2] > 0.0 else -ref_mamp_normal
    if abs(float(mamp_normal[2])) <= ZERO_TOLERANCE:
        raise ValueError("The Ma/MP plane normal has no superior/inferior direction.")
    temporalis_normal = fh_normal if fh_normal[2] > 0.0 else -fh_normal
    lp_normal = ref_lp_normal if ref_lp_normal[1] > 0.0 else -ref_lp_normal

    return {
        "MaMP": (ref_mamp_origin + mamp_normal * MAMP_OFFSET_MM, mamp_normal),
        "T": (fh_origin + temporalis_normal * TEMPORALIS_OFFSET_MM, temporalis_normal),
        "LP": (ref_lp_origin + lp_normal * LATERAL_PTERYGOID_OFFSET_MM, lp_normal),
    }


def run_total_segmentator(ct_path, force_cpu):
    ct_file = Path(ct_path).expanduser().resolve()
    if not ct_file.is_file():
        raise FileNotFoundError(f"CT volume not found: {ct_file}")

    if not hasattr(slicer, "mrmlScene"):
        raise RuntimeError("This script must be run by 3D Slicer.")
    try:
        from TotalSegmentator import TotalSegmentatorLogic
    except Exception as exc:
        raise RuntimeError(
            "The TotalSegmentator extension is not installed or failed to load in this 3D Slicer installation."
        ) from exc

    volume_node = slicer.util.loadVolume(str(ct_file))
    if volume_node is None or volume_node.GetImageData() is None:
        raise RuntimeError(f"3D Slicer could not load the CT volume: {ct_file}")
    spacing = np.asarray(volume_node.GetSpacing(), dtype=float)
    if spacing.shape != (3,) or not np.all(np.isfinite(spacing)) or np.any(spacing <= 0.0):
        raise ValueError("The CT volume has invalid voxel spacing.")

    segmentation_node = slicer.mrmlScene.AddNewNodeByClass(
        "vtkMRMLSegmentationNode", "Head muscles"
    )
    logic = TotalSegmentatorLogic()
    if "head_muscles" not in logic.tasks:
        raise RuntimeError("This TotalSegmentator version does not provide the head_muscles task.")
    missing_packages = [
        name for name in ("torch", "nnunetv2", "totalsegmentator")
        if importlib.util.find_spec(name) is None
    ]
    if missing_packages:
        raise RuntimeError(
            "TotalSegmentator Python dependencies are not initialized in this 3D Slicer installation: "
            + ", ".join(missing_packages)
        )
    logic.useStandardSegmentNames = True
    process_parameters = inspect.signature(logic.process).parameters
    quality_arguments = {}
    if "quality" in process_parameters:
        quality_arguments["quality"] = "normal"
    elif "fast" in process_parameters:
        quality_arguments["fast"] = False
    else:
        raise RuntimeError("Unsupported TotalSegmentator process API: no quality or fast parameter.")
    logic.process(
        volume_node,
        segmentation_node,
        cpu=bool(force_cpu),
        task="head_muscles",
        interactive=False,
        **quality_arguments,
    )
    return segmentation_node


def _segment_and_id(segmentation_node, required_id):
    segmentation = segmentation_node.GetSegmentation()
    segment = segmentation.GetSegment(required_id)
    if segment is not None:
        return segment, required_id
    segment_id = segmentation.GetSegmentIdBySegmentName(required_id)
    if segment_id:
        return segmentation.GetSegment(segment_id), segment_id

    available = vtk.vtkStringArray()
    segmentation.GetSegmentIDs(available)
    names = []
    for index in range(available.GetNumberOfValues()):
        item_id = available.GetValue(index)
        item = segmentation.GetSegment(item_id)
        names.append(f"{item_id} ({item.GetName()})")
    raise ValueError(
        f"Required TotalSegmentator segment '{required_id}' is missing. "
        f"Available segments: {', '.join(names)}"
    )


def _surface_in_world(segmentation_node, segment):
    surface = segment.GetRepresentation("Closed surface")
    if surface is None or surface.GetNumberOfPoints() == 0:
        raise RuntimeError(f"Closed surface is empty for segment '{segment.GetName()}'.")

    surface_copy = vtk.vtkPolyData()
    surface_copy.DeepCopy(surface)
    parent_transform = segmentation_node.GetParentTransformNode()
    if parent_transform is None:
        return surface_copy

    world_transform = vtk.vtkGeneralTransform()
    slicer.vtkMRMLTransformNode.GetTransformBetweenNodes(
        parent_transform, None, world_transform
    )
    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetInputData(surface_copy)
    transform_filter.SetTransform(world_transform)
    transform_filter.Update()
    world_surface = vtk.vtkPolyData()
    world_surface.DeepCopy(transform_filter.GetOutput())
    return world_surface


def cross_section_area_mm2(surface, origin, normal, allow_empty=False):
    plane = vtk.vtkPlane()
    plane.SetOrigin([float(value) for value in origin])
    plane.SetNormal([float(value) for value in normal])

    cutter = vtk.vtkCutter()
    cutter.SetInputData(surface)
    cutter.SetCutFunction(plane)
    cutter.Update()
    intersection = cutter.GetOutput()
    if intersection.GetNumberOfPoints() < 3:
        if allow_empty:
            return 0.0
        raise RuntimeError("The SCS plane does not intersect this muscle surface.")

    triangulator = vtk.vtkContourTriangulator()
    triangulator.SetInputData(intersection)
    triangulator.Update()
    triangulated = triangulator.GetOutput()

    area = 0.0
    triangle_count = 0
    for cell_index in range(triangulated.GetNumberOfCells()):
        cell = triangulated.GetCell(cell_index)
        if cell.GetNumberOfPoints() != 3:
            continue
        p0 = cell.GetPoints().GetPoint(0)
        p1 = cell.GetPoints().GetPoint(1)
        p2 = cell.GetPoints().GetPoint(2)
        area += vtk.vtkTriangle.TriangleArea(p0, p1, p2)
        triangle_count += 1

    if not math.isfinite(area):
        raise RuntimeError("The muscle intersection produced a non-finite area.")
    if triangle_count == 0 or area <= 0.0:
        if allow_empty:
            return 0.0
        raise RuntimeError("The muscle intersection could not be triangulated into a positive area.")
    return float(area)


def sampled_cross_section_area_mm2(surface, origin, normal, use_neighboring_planes):
    offsets = (0.0,)
    if use_neighboring_planes:
        offsets += NEIGHBORING_PLANE_OFFSETS_MM
    areas = [
        cross_section_area_mm2(
            surface,
            np.asarray(origin, dtype=float) + float(offset) * np.asarray(normal, dtype=float),
            normal,
            allow_empty=use_neighboring_planes,
        )
        for offset in offsets
    ]
    maximum_area = max(areas)
    if not math.isfinite(maximum_area) or maximum_area <= 0.0:
        raise RuntimeError("None of the sampled SCS planes produced a positive muscle area.")
    return maximum_area


def calculate_scsa(segmentation_node, planes, use_neighboring_planes=False):
    if not segmentation_node.CreateClosedSurfaceRepresentation():
        raise RuntimeError("3D Slicer could not create the muscle closed surfaces.")

    values_cm2 = {}
    for segment_id, output_label, plane_key in MUSCLE_PLANES:
        segment, _ = _segment_and_id(segmentation_node, segment_id)
        surface = _surface_in_world(segmentation_node, segment)
        origin, normal = planes[plane_key]
        values_cm2[output_label] = sampled_cross_section_area_mm2(
            surface, origin, normal, use_neighboring_planes
        ) / 100.0
    return values_cm2


def write_scsa(path, values_cm2):
    output_path = Path(path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    expected_labels = [item[1] for item in MUSCLE_PLANES]
    missing = [label for label in expected_labels if label not in values_cm2]
    if missing:
        raise ValueError(f"Cannot write SCSA; missing values: {', '.join(missing)}")

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=output_path.name + ".",
            suffix=".tmp",
            dir=output_path.parent,
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            for label in expected_labels:
                value = float(values_cm2[label])
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError(f"SCSA value for {label} is not positive and finite.")
                stream.write(f"{label}: {value:.12f}\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return output_path


def write_status(path, success, message):
    if not path:
        return
    status_path = Path(path).expanduser().resolve()
    status_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=status_path.name + ".",
            suffix=".tmp",
            dir=status_path.parent,
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            stream.write("SUCCESS\n" if success else "ERROR\n")
            stream.write(str(message).rstrip() + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, status_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def main(arguments):
    points = load_landmarks(arguments.landmarks)
    planes = compute_planes(points)
    segmentation_node = run_total_segmentator(arguments.ct, arguments.cpu)
    values_cm2 = calculate_scsa(
        segmentation_node, planes, arguments.neighboring_planes
    )
    output_path = write_scsa(arguments.output, values_cm2)
    print(f"SCSA written to: {output_path}")
    for _, label, _ in MUSCLE_PLANES:
        print(f"{label}: {values_cm2[label]:.12f} cm^2")


if __name__ == "__main__":
    exit_code = 0
    arguments = None
    try:
        arguments = parse_arguments()
        main(arguments)
        write_status(arguments.status_file, True, "SCSA generation completed.")
    except Exception as error:
        exit_code = 1
        error_details = traceback.format_exc()
        print(f"SCSA generation failed: {error}", file=sys.stderr)
        print(error_details, file=sys.stderr)
        if arguments is not None:
            write_status(arguments.status_file, False, error_details)
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        slicer.util.exit(exit_code)
