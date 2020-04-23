.. py:currentmodule:: lsst.ts.rotator

.. _lsst.ts.rotator.revision_history:

###########################
ts_rotator Revision History
###########################

v0.4.0
======

Update `MockMTRotatorController` to use the ``TrackingActuator`` from ts_simactuators.
Formerly `MockMTRotatorController` used a locally defined point to point actuator, which gives somewhat lower fidelity and duplicates code in ts_simactuators.
Requires:

* ts_hexrotcomm v0.2.0
* ts_salobj 5
* ts_simactuators v1
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

* ts_hexrotcomm v0.2.0
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.2.0
======

Update for changes to Rotator XML.
Tested with the rotator.

Requires:

* ts_hexrotcomm v0.2.0
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.1.0
======

Still not fully tested with the real rotator.

Requires:

* ts_hexrotcomm v0.1.0
* ts_salobj 5
* ts_idl 1
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``
