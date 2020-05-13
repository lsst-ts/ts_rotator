from setuptools import setup, find_namespace_packages

install_requires = []
tests_require = ["pytest", "pytest-cov", "pytest-flake8", "asynctest"]
dev_requires = install_requires + tests_require + ["documenteer[pipelines]"]
scm_version_template = """# Generated by setuptools_scm
__all__ = ["__version__"]

__version__ = "{version}"
"""

setup(
    name="ts_rotator",
    description="CSC for main telescope camera rotator",
    use_scm_version={
        "write_to": "python/lsst/ts/rotator/version.py",
        "write_to_template": scm_version_template,
    },
    setup_requires=["setuptools_scm", "pytest-runner"],
    install_requires=install_requires,
    package_dir={"": "python"},
    packages=find_namespace_packages(where="python"),
    package_data={"": ["*.rst", "*.yaml"]},
    scripts=[
        "bin/run_rotator.py",
        "bin/run_mock_rotator_pxi.py",
        "bin/command_rotator.py",
    ],
    tests_require=tests_require,
    extras_require={"dev": dev_requires},
    license="GPL",
    project_urls={
        "Bug Tracker": "https://jira.lsstcorp.org/secure/Dashboard.jspa",
        "Source Code": "https://github.com/lsst-ts/ts_rotator",
    },
)
