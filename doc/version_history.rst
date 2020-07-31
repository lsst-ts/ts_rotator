.. py:currentmodule:: lsst.ts.rotator

.. _lsst.ts.rotator.version_history:

###############
Version History
###############

v0.6.0
======

Changes

* Added missing ``config_dir`` constructor argument to `RotatorCsc`.
* Use `lsst.ts.salobj.BaseCscTestCase` and `lsst.ts.salobj.CscCommander` instead of the versions in ts_hexrotcomm.
* Add attribute ``position_jitter`` to `MockMTRotatorController` and update the unit tests to use it.
  Also make test_move more robust by giving the slew more time to finish.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.5.0
======

Changes

* Make `RotatorCsc` configurable.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.4
======

Changes:

* Add ``tests/test_black.py`` to verify that files are formatted with black.
  This requires ts_salobj 5.11 or later.
* Update ``.travis.yml`` to remove ``sudo: false`` to github travis checks pass once again.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5.11
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.3
======

* Fix flake8 violations.
* Improve Jenkins.conda build script so it will label PRs and branches packages as dev and upload them to anaconda.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.2
======

* Fix flake8 violations.
* Add Jenkinsfile for CI job.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.1
======

* Include conda package build configuration.
* Added a Jenkinsfile to support continuous integration and to build conda packages.
* Remove unused schema file.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.0
======

Update `MockMTRotatorController` to use the ``TrackingActuator`` from ts_simactuators.
Formerly `MockMTRotatorController` used a locally defined point to point actuator, which gives somewhat lower fidelity and duplicates code in ts_simactuators.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.3.0
======

Major changes:

* Added a revision history.
* Code formatted by ``black``, with a pre-commit hook to enforce this.
  See the README file for configuration instructions.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.2.0
======

Update for changes to Rotator XML.
Tested with the rotator.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.1.0
======

Still not fully tested with the real rotator.

Requires:

* ts_hexrotcomm 0.1
* ts_salobj 5
* ts_idl 1
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``
