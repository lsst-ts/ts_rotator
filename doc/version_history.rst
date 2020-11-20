.. py:currentmodule:: lsst.ts.mtrotator

.. _lsst.ts.mtrotator.version_history:

###############
Version History
###############

v0.10.0
=======

Changes:

* Update to use and require ts_hexrotcomm 0.12:

    * Add argument ``curr_tai`` to `MockMTRotatorController.update_telemetry` and use it.

* Update the mock controller to report generated path data instead of target data
  in the telemetry fields used to set the demand fields of the rotation and application telemetry topics.
  This matches what the real rotator does.
* Update the unit tests to handle the new rotation and application telemetry data.
* Rename the `Telemetry` struct demand field names to clarify their content.
* Update the rotator commander to handle the rotation telemetry event better.
  Ignore the timestamp field when deciding whether the information has changed enough to justify printing the new sample.
  Update the custom motors telemetry callback to work in the same way, ignoring the raw field when deciding whether to print the data.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.9.0
======

Changes:

* Updated to use and require ts_salobj 7.0, ts_idl 2.2, and ts_hexrotcomm 0.11:

    * Rename the SAL component ``Rotator`` to ``MTRotator``.
    * Rename ts_idl ``Rotator`` enum module to ``MTRotator``.

* Rename the package from ``ts_rotator`` to ``ts_mtrotator``.

Requires:

* ts_hexrotcomm 0.11
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.8.0
======

Changes:

* Updated to use and require ts_salobj 6.1 and ts_hexrotcomm 0.10.
* Update the handling of initial_state in `RotatorCsc`:

    * If initial_state != OFFLINE then report all transitional summary states and controller states at startup.
    * Require initial_state = OFFLINE unless simulating.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.7.3
======

Changes:

* Use the time in the telemetry header to set the ``rotation`` telemetry topic's time stamp.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6.0
* ts_simactuators 1
* ts_idl 1.4, or 2 with salobj 6.0
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.7.2
======

Changes:

* Fix Jenkinsfile.conda.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6.0
* ts_simactuators 1
* ts_idl 1.4 with salobj 5, or 2 with salobj 6
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.7.1
======

Changes:

* Fix conda build.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6
* ts_simactuators 1
* ts_idl 1.4, or 2 with salobj 6
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.7.0
======

Changes:

* Updated to read telemetry from a newer version of the low-level controller: changes added in https://jira.lsstcorp.org/browse/DM-25994.
* Updated to write new event and telemetry information added in ts_xml 6.2.
* Use corrected spelling of ``Rotator.ApplicationStatus.SAFETY_INTERLOCK``.
  This requires ts_idl 1.4 or later.
* Updated the git pre-commit hook to prevent the commit if black formatting needed.
  This encourages the user to properly commit the necessary reformatting.
* Modernize the documentation.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6
* ts_simactuators 1
* ts_idl 1.4, or 2 with salobj 6
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.6.0
======

Changes:

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

Changes:

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
