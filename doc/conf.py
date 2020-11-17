"""Sphinx configuration file for an LSST stack package.

This configuration only affects single-package Sphinx documentation builds.
"""

from documenteer.sphinxconfig.stackconf import build_package_configs
import lsst.ts.mtrotator


_g = globals()
_g.update(
    build_package_configs(
        project_name="ts_mtrotator", version=lsst.ts.mtrotator.__version__
    )
)

intersphinx_mapping["ts_xml"] = ("https://ts-xml.lsst.io", None)  # noqa
intersphinx_mapping["ts_salobj"] = ("https://ts-salobj.lsst.io", None)  # noqa
intersphinx_mapping["ts_hexrotcomm"] = ("https://ts-hexrotcomm.lsst.io", None)  # noqa
