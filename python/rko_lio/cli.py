# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Entrypoint typer application for the python wrapper.
"""

import sys
from pathlib import Path

import numpy as np
import typer

from .util import error, info, warning


def version_callback(value: bool):
    if value:
        from importlib.metadata import version

        rko_lio_version = version("rko_lio")
        info("RKO_LIO Version:", rko_lio_version)
        raise typer.Exit(0)


def dump_config_callback(value: bool):
    if value:
        import yaml

        from .lio import LIOConfig

        def pybind_to_dict(obj):
            return {
                attr: getattr(obj, attr)
                for attr in dir(obj)
                if not attr.startswith("_") and not callable(getattr(obj, attr))
            }

        config = pybind_to_dict(LIOConfig())
        config["extrinsic_imu2base_quat_xyzw_xyz"] = []
        config["extrinsic_lidar2base_quat_xyzw_xyz"] = []

        with open("config.yaml", "w") as f:
            yaml.dump(config, f, default_flow_style=False)
        info(
            "Default config dumped to config.yaml. Note that the extrinsics are left as an empty list. If you don't need them, delete the two respective keys. If you need them, you need to specify them as \[qx, qy, qz, qw, x, y, z]."
        )
        raise typer.Exit(0)


def dataloader_name_callback(value: str):
    from .dataloaders import available_dataloaders

    if not value:
        return value
    dl = available_dataloaders()
    if value.lower() not in [d.lower() for d in dl]:
        raise typer.BadParameter(f"Supported dataloaders are: {', '.join(dl)}")
    for d in dl:
        if value.lower() == d.lower():
            return d
    return value


def parse_extrinsics_from_config(config_data: dict):
    def convert_quat_xyzw_xyz_to_matrix(quat_xyzw_xyz: np.ndarray) -> np.ndarray:
        from pyquaternion import Quaternion

        qx, qy, qz, qw = quat_xyzw_xyz[:4]
        xyz = quat_xyzw_xyz[4:]

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = Quaternion(x=qx, y=qy, z=qz, w=qw).rotation_matrix
        transform[:3, 3] = xyz
        return transform

    extrinsic_imu2base = None
    extrinsic_lidar2base = None

    imu_config_val = config_data.pop("extrinsic_imu2base_quat_xyzw_xyz", None)
    lidar_config_val = config_data.pop("extrinsic_lidar2base_quat_xyzw_xyz", None)

    if imu_config_val is not None:
        if not len(imu_config_val) == 7:
            error(
                "extrinsic_imu2base_quat_xyzw_xyz cannot be of length",
                len(imu_config_val),
                "but should be 7. Please modify the config.",
            )
            sys.exit(1)
        extrinsic_imu2base = convert_quat_xyzw_xyz_to_matrix(
            np.asarray(imu_config_val, dtype=np.float64)
        )
    if lidar_config_val is not None:
        if not len(lidar_config_val) == 7:
            error(
                "extrinsic_lidar2base_quat_xyzw_xyz cannot be of length",
                len(lidar_config_val),
                "but should be 7. Please modify the config.",
            )
            sys.exit(1)
        extrinsic_lidar2base = convert_quat_xyzw_xyz_to_matrix(
            np.asarray(lidar_config_val, dtype=np.float64)
        )
    return extrinsic_imu2base, extrinsic_lidar2base


def transform_to_quatxyzw_xyz(transform: np.ndarray):
    import pyquaternion

    return [
        float(x)
        for x in (
            *pyquaternion.Quaternion(matrix=transform[:3, :3]).vector,
            pyquaternion.Quaternion(matrix=transform[:3, :3]).scalar,
            *transform[:3, 3],
        )
    ]


app = typer.Typer()


@app.command(
    epilog="Please open an issue on https://github.com/PRBonn/rko_lio if the usage of any option is unclear or you need some help!"
)
def cli(
    data_path: Path = typer.Argument(..., exists=True, help="Path to data folder"),
    config_fp: Path | None = typer.Option(
        None, "--config", "-c", exists=True, help="Path to config.yaml"
    ),
    dataloader_name: str | None = typer.Option(
        None,
        "--dataloader",
        "-d",
        help="Specify a dataloader: [rosbag, raw, helipr]. Leave empty to guess one",
        show_choices=True,
        callback=dataloader_name_callback,
        case_sensitive=False,
    ),
    viz: bool = typer.Option(False, "--viz", "-v", help="Enable Rerun visualization"),
    log_results: bool = typer.Option(
        False,
        "--log",
        "-l",
        help="Log trajectory results to disk at 'results_dir' on completion",
    ),
    results_dir: Path | None = typer.Option(
        "results", "--results_dir", "-r", help="Where to dump LIO results if logging"
    ),
    run_name: str | None = typer.Option(
        None,
        "--run_name",
        "-n",
        help="Name prefix for output files if logging. Default takes the name from the data_path argument",
    ),
    sequence: str | None = typer.Option(
        None,
        "--sequence",
        help="Extra dataloader argument: sensor sequence (for helipr only)",
    ),
    imu_topic: str | None = typer.Option(
        None, "--imu", help="Extra dataloader argument: imu topic (for rosbag only)"
    ),
    lidar_topic: str | None = typer.Option(
        None, "--lidar", help="Extra dataloader argument: lidar topic (for rosbag only)"
    ),
    base_frame: str | None = typer.Option(
        None,
        "--base_frame",
        help="Extra dataloader argument: base_frame for odometry estimation, default is lidar frame (for rosbag only)",
    ),
    imu_frame: str | None = typer.Option(
        None,
        "--imu_frame",
        help="Extra dataloader argument: imu frame overload (for rosbag only)",
    ),
    lidar_frame: str | None = typer.Option(
        None,
        "--lidar_frame",
        help="Extra dataloader argument: lidar frame overload (for rosbag only)",
    ),
    version: bool | None = typer.Option(
        None,
        "--version",
        help="Print the current version of RKO_LIO and exit",
        callback=version_callback,
        is_eager=True,
        rich_help_panel="Auxilary commands",
    ),
    dump_config: bool | None = typer.Option(
        None,
        "--dump_config",
        help="Dump the default config to config.yaml and exit",
        callback=dump_config_callback,
        is_eager=True,
        rich_help_panel="Auxilary commands",
    ),
):
    """
    Run RKO_LIO with the selected dataloader and parameters.
    """

    if viz:
        try:
            import rerun as rr

            rr.init("rko_lio")
            rr.spawn(memory_limit="2GB")
            rr.log_file_from_path(Path(__file__).parent / "rko_lio.rbl")

        except ImportError:
            error(
                "Please install rerun with `pip install rerun-sdk` to enable visualization."
            )
            sys.exit(1)

    config_data = {}
    if config_fp:
        with open(config_fp, "r") as f:
            import yaml

            config_data.update(yaml.safe_load(f))

    extrinsic_imu2base, extrinsic_lidar2base = parse_extrinsics_from_config(config_data)
    need_to_query_extrinsics = not (
        extrinsic_imu2base is not None
        and len(extrinsic_imu2base)
        and extrinsic_lidar2base is not None
        and len(extrinsic_lidar2base)
    )
    if need_to_query_extrinsics:
        warning(
            "One or both extrinsics are not specified in the config. Will try to obtain it from the data(loader) itself."
        )

    from .dataloaders import get_dataloader

    dataloader = get_dataloader(
        name=dataloader_name,
        data_path=data_path,
        sequence=sequence,
        imu_topic=imu_topic,
        lidar_topic=lidar_topic,
        imu_frame_id=imu_frame,
        lidar_frame_id=lidar_frame,
        base_frame_id=base_frame,
        query_extrinsics=need_to_query_extrinsics,
    )
    print("Loaded dataloader:", dataloader)
    if need_to_query_extrinsics:
        extrinsic_imu2base, extrinsic_lidar2base = dataloader.extrinsics
        print("Extrinsics obtained from dataloader.")
        print("Imu to Base:\n\tTransform:")
        for row in extrinsic_imu2base:
            print("\t\t" + str(row))
        print(
            "\tAs a quat_xyzw_xyz:\n\t\t", transform_to_quatxyzw_xyz(extrinsic_imu2base)
        )
        print("Lidar to base:\n\tTransform:")
        for row in extrinsic_lidar2base:
            print("\t\t" + str(row))
        print(
            "\tAs a quat_xyzw_xyz:\n\t\t",
            transform_to_quatxyzw_xyz(extrinsic_lidar2base),
        )

    from .lio import LIOConfig

    config = LIOConfig(**config_data)

    from .lio_pipeline import LIOPipeline

    pipeline = LIOPipeline(
        config,
        extrinsic_imu2base=extrinsic_imu2base,
        extrinsic_lidar2base=extrinsic_lidar2base,
        viz=viz,
    )

    from tqdm import tqdm

    for kind, data_tuple in tqdm(dataloader, total=len(dataloader), desc="Data"):
        if kind == "imu":
            pipeline.add_imu(*data_tuple)
        elif kind == "lidar":
            pipeline.add_lidar(*data_tuple)

    if log_results and results_dir:
        results_dir.mkdir(parents=True, exist_ok=True)
        pipeline.dump_results_to_disk(results_dir, run_name or data_path.name)


if __name__ == "__main__":
    app()
